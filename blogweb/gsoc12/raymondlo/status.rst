My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: raymondlo

.. blogpost:: 
  :title: Benchmarking PNG Image dumping for PCL 
  :author: raymondlo
  :date: 08-23-2012

  Here is the result we got from PNG dumping benchmarking: 640x480 (16 bits) depth map + 640 x 480 (24 bits) color image.

    .. line-block::

      08-23 20:57:43.830: I/PCL Benchmark:(10552): Number of Points: 307200, Runtime: 0.203085 (s)
      08-23 20:57:54.690: I/PCL Benchmark:(10552): Number of Points: 307200, Runtime: 0.215253 (s)


  If we are dumping the result to the /mnt/sdcard/, we are getting:

    .. line-block:: 

      08-23 21:02:23.890: I/PCL Benchmark:(14839): Number of Points: 307200, Runtime: 0.332639 (s)
      08-23 21:02:40.410: I/PCL Benchmark:(14839): Number of Points: 307200, Runtime: 0.328380 (s)

  There is a significant overhead (about 0.1 second!) with the SD Card I/O. 

   We shall verify these with a faster SD card. The /mnt/sdcard seems to be mounted onto an internal SD Card on the Tegra 3 dev board that I have no access to? I tried to open the back and so already. 
  
  Also, I have tried different compression levels and it seems that level 3 is giving the best compression ratio vs the speed. More plots will come next to justify my observations.

  

.. blogpost::
  :title: Code Testing
  :author: raymondlo
  :date: 08-21-2012

  Have gone through the whole setup again and replicated on Ubuntu 12.04 + Mac OSX 10.8 environments. The README file is now updated to reflect what is needed to have the environment setup. 

  At the end, it was only 4 scripts, and maybe we can automate these completely.  We have also added the scripts for dumping the images store in the SD Card. check out all .sh files in the directory and that may save hours of your time.




.. blogpost::
  :title: Lossless Image Dumping with libpng + Android and Tegra 3
  :author: raymondlo
  :date: 08-17-2012

  Today, I've added the support of libpng + libzlib for the Tegra 3 project and so we can dump the raw images from the Kinect (or any OpenNI supported devices) onto the SDCard for post-processing or debugging. After hours of fiddling with the parameters and hacking away on the code, now we can capture and compress 4-6 images per second (2-3x 24-bit RGB image + 2-3x 16-bit depth image) on a Tegra 3. I believe these libraries are already NEON optimized and thus we shall be getting the best performance from them. Here is the little magic that gives me the best performance so far.

    
  .. line-block::

    // Write header (16 bit colour depth, greyscale)
    png_set_IHDR(png_ptr, info_ptr, width, height, 16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_COMPRESSION_TYPE_DEFAULT);
    //fine tuned parameter for speed!
    png_set_filter(png_ptr, PNG_FILTER_TYPE_BASE, PNG_FILTER_SUB);
    png_set_compression_level(png_ptr, 1); //1 is Z_BEST_SPEED in zlib.h!
    png_set_compression_strategy(png_ptr, 3); //3 is Z_RLE

  Next step, if time permitted I will use the PCL library compression code instead. Using the libpng, however, has taught me where the critical paths are and how we shall handle the data. Right now, I am sure that I wasn't introducing any overheads from the data copying or manipulations. I was handling the raw data pointers the whole time. 

  For the longest time, I have had trouble getting any performance out from the Tegra 3, mainly because of the floating point operations! Again, avoid these operations at all cost unless we have a more powerful processor!

  Here is a screenshot of some of the images that were dumped from my Kinect in real-time!

  .. image:: images/png_dump_screenshot.jpg
      :width: 720px
      :align: center


.. blogpost:: 
  :title: PCL Android (milestones)
  :author: raymondlo
  :date: 08-01-2012
  

  The lack of hardware accelerated libraries for Android is the key bottleneck I've been facing. After spending many hours on NEON assembly and other tools, I finally come across this...

  Ne10: A New Open Source Library to Accelerate your Applications with NEON
  http://blogs.arm.com/software-enablement/703-ne10-a-new-open-source-library-to-accelerate-your-applications-with-neon/
 
  http://projectne10.github.com/Ne10/

  Next I've verify the actual speedup we can get with such library, and see how we can accelerate some of the PCL calls with these. With NEON + multithreading, I am looking for a 10x speedup on Tegra 3.

  More to come next...
  Update: I've added Ne10 to the project tree, and have it compiled. Shall be ready to verify the theortical speedup we can obtain with the new hardware accelerated libraries. =)
  

.. blogpost::
  :title: ARM Optimization 
  :author: raymondlo
  :date: 07-29-2012

  Often time we have ignored the importance of writing efficient source code. With the mobile platform, every bit of computation matters. Imagine you have a video player that can only achieve 10 fps, while the competitors are running at 60fps. These differences may define a successful or failure application. 
	
  To get started, these weeks I've gathered some NEON material and wrote some small functions that's optimized with NEON instruction set. In fact it is surprisingly difficult due to the lack of documentations and example sometimes (maybe I've not tried hard enough?)

  It wasn't very difficult to have NEON Intrinsics code compiled and run on Tegra 3 after all. 

  All we need is adding the #include <arm_neon.h> and compile with -mfloat-abi=softfp -mfpu=neon options. 

  With the native c code, we can perform a simple array sum (i.e., adding all elements in an array) in about 0.034 second. 

  .. line-block::

	07-30 02:46:03.170: I/PCL Benchmark:(1426): Number of Points: 65536, 65536, Runtime: 0.034658 (s)

  With the NEON, we get about 2x the perform. 

  .. line-block::

	07-30 02:48:04.070: I/PCL Benchmark:(2392): Number of Points: 65536, 65536, Runtime: 0.015879 (s)

  .. line-block:: 
	int16_t sum=0;
	for (; size != 0; size -= 1)
	{
		sum+=array[size-1];
	}
	return sum;


   .. line-block:: 

     int16x4_t acc = vdup_n_s16(0);
     int32x2_t acc1;
     int64x1_t acc2;
     assert((size % 4) == 0);
     for (; size != 0; size -= 4)
     {
          int16x4_t vec;
          vec = vld1_s16(array);
          array += 4;
          acc = vadd_s16(acc, vec);
      }
      acc1 = vpaddl_s16(acc);
      acc2 = vpaddl_s32(acc1);
      return (int)vget_lane_s64(acc2, 0);

   Code Example Source:
   http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0205j/BABGHIFH.html

   Reference Links:
   https://pixhawk.ethz.ch/_media/software/optimization/neon_support_in_the_arm_compiler.pdf

   The next step would be optimizing for the floating point operations, and it seems to be a rather difficult task. It seems to be promising now as if I can multithread the work to 3 cores together with NEON, we can get 4-6x speed up, and thus a 5fps application will now run smoothly at 30fps. That's a big improvement for sure.

   

.. blogpost::
  :title: PCL Performance Benchmark on Tegra 3 (Android 4.0)
  :author: raymondlo
  :date: 07-12-2012

  I've rebuilt the PCL libraries using the build script (see pcl_binary/ in the svn respository) on Ubuntu 12.04 and the compilation works with a few hipcup. First we need to turn off the ENABLE_EXAMPLE flag due to the dependency problem. Second, we have to compile it with make -j 1 flag. Otherwise, everything ran smoothly. 

  I notice there isn't any performance benchmark of PCL on Tegra 3. Here I've done a few simple testing. I believe it is important to see how such floating point operations we can do per second with the Tegra 3 architecture. float vs double? float vs int? Again, compiling the library as arm vs thumb mode may make different. Here I will provide a quick but throughout summary of what we can achieve with the Tegra 3 under different settings. 


  For simplicity, I've first benchmarked the passthrough filter by averaging the runtime of the filter over ten trials. I know this filter shall have a linear behaviour to the number of points, so far the benchmark results seem to be consistent. 

    .. line-block::

      // Create the filtering object
      pcl::PassThrough < pcl::PointXYZ > pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, 1.0);
      pass.filter(\*cloud_filtered);


    .. line-block::

      07-12 21:37:50.070: I/PCL Benchmark:(2785): Number of Points: 10000, Runtime: 0.002583 (s)
      07-12 21:37:50.190: I/PCL Benchmark:(2785): Number of Points: 10000, Runtime: 0.002652 (s)
      ...

    .. line-block::

      07-12 21:41:14.330: I/PCL Benchmark:(3614): Number of Points: 100000, Runtime: 0.036954 (s)
      07-12 21:41:14.880: I/PCL Benchmark:(3614): Number of Points: 100000, Runtime: 0.038295 (s)
      ...

    .. line-block::

      07-12 21:39:49.130: I/PCL Benchmark:(3344): Number of Points: 1000000, Runtime: 0.397860 (s)
      07-12 21:39:53.720: I/PCL Benchmark:(3344): Number of Points: 1000000, Runtime: 0.392162 (s)
      ...

  With these information, we can start optimizing our work by reducing the bottlenecks in each of these filters. But let's recompile it in ARM mode and see if it will make a world of difference. 

  After I know what the Tegra 3 is capable of. It is time to design a simple 3D application. Argumented reality? and a few segmentation algorithms will do the tricks. What can we achieve with the current hardware? 


 
.. blogpost::
  :title: Tegra 3 + Android + OpenNI + PCL's VoxelGrid filter Sample 
  :author: raymondlo
  :date: 07-06-2012
  
  I've collect some statistics and screenshots of our new sample app that demostrates the voxel grid filtering using the PCL library. The performance isn't something I would be very proud of, i.e., only ~2fps with all of the processing with about ~0.3 million points (307200) as input. However, it is quite usable if we are using this for capturing something steady, perhaps to be used for 3D reconstruction in real-time. 

  Here are some screenshots of the sample apps, and it shows the RGB images, depth image, and also the 3D PointCloud data all using the OpenGL ES2. The downsampling does provide us at least 50% reduction on the number of points. 
   
    .. image:: images/voxel_pcl_sample_july_6.jpg
       :width: 720px
       :align: center

    .. image:: images/voxel_pcl_sample_july_6_2.jpg
       :width: 720px
       :align: center
 
  Here is a little video demo of the application (running at 2fps). 
       
    .. raw:: html

       <iframe width="720" height="480" src="http://www.youtube.com/embed/tQUxxZiJcoA" frameborder="0" allowfullscreen></iframe>


  We also collects some simple statistics on the performance of the algorithm, both the runtime statistics and the compression ratio we can achieve. 
    .. line-block::

      ...
      07-05 21:46:59.150: I/Render Loop:(10204): Display loop 0.543398 (s)
      07-05 21:46:59.730: I/PCL FILTER TESTING:(10204): Original: 307200, Filtered: 75208, Ratio: 0.244818, Sum 564766.304984 
      ... 
      
  In some cases, the voxel grid filter can reduce the number of points to only a small fraction. I have seen cases where the ratio is below 10% for flat surfaces. We have only touched the surface of the PCL library, but I can see number of applications can be built using these. Possibilities are just limitless. ;) 
  
  TODO: need multithreading to utilizing the quadcore on the Tegra 3. It seems to be an easy task. CPU # 1: OpenNI engine; CPU # 2: main thread with GUI; CPU#3 & 4: PCL and other processing engines. :). That way we will be fully untilizing all for the cores on Tegra 3. Also, I wonder if we can use float instead of double for all operations, and also turn off the THUMB mode! Obviously, we need to squeeze more performance out from everything. NEON optimization? Anyone?
  

.. blogpost:: 
  :title: Android + Openni Autoscript
  :author: raymondlo
  :date: 07-01-2012
  
  Thanks to Radu and others, we have finalized the autoscript for compiling the OpenNI for Android. These are all integrated to the Android sample project. Also, we have compiled the PCL for android, and in the coming weeks we shall have a complete sample project. These can be used as our standard template for development of PCL + OpenNI on Android. 
  
  Happy Canada Day. 

.. blogpost:: 
  :title: Android + PCL?
  :author: raymondlo
  :date: 06-18-2012

  Having a hard time replicating the result here.

  http://dev.pointclouds.org/projects/pcl/wiki/How_to_compile_PCL_for_Android

  My next task is to update the instruction on the mobile_apps directory and compile a binary that I can use my Android development. The mobile_apps directory will be updated with our new code for Android next. Most likely the first thing we will see is a port of OpenNI + PCL  sample on Android. Then, a simple tracking will be added to it next. 

  I really wonder what the performance is like on Tegra 3. That will be my first report with some sort of side by side comparison. A few optimization would be added if I can get the code compile properly first. 

.. blogpost:: 
  :title: Continue my work from NVSC. 
  :author: raymondlo
  :date: 06-14-2012

  I believe today I will be starting updating the blog on the GSoC side, and continue my work on the Tegra 3 + Android + OpenNI + PCL etc...

  See http://www.pointclouds.org/blog/nvcs/raymondlo84/index.php for my previous posts on OpenNI ports and other tricks.

  Now the leading issue is to get PCL running on my Tegra 3 tablet. I hope I can get a decent performance out from the box. 
