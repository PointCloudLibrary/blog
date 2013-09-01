My status updates
=================

.. blogpost::
  :title: Kintinuous: Spatially Extended KinectFusion
  :author: mfallon
  :date: 05-26-2012

  Thomas Whelan and John McDonald, two of our collaborators from the National University of Ireland in Maynooth, have been feverishly working away on extensions and improvements to PCL's implementation of KinectFusion - KinFu.

  Two of the major limitations of KinectFusion have been overcome by the algorithm, which Tom calls Kintinuous:
  
  * By constructing a 3D cyclic buffer, which is continually emptied as the camera translates, the original restriction to a single cube (e.g. 5x5x5m) has been removed
  * Operation in locations where there are insignificant constraints to reliably carry out ICP (KinectFusion's initial step) is now possible by using visual odometry (FOVIS).

  Tom has created some amazing mesh reconstructions of apartments, corridors, lecture theaters and even outdoors (at night) - which can be produced real-time. Below is a video overview showing some of these reconstructions.

  .. raw:: html

    <iframe title="Kintinuous - Spatially Extended KinectFusion" width="800" height="500" src="http://www.youtube.com/embed/ggvGX4fwT5g?rel=0" frameborder="0" allowfullscreen></iframe>


  We are hoping to integrate Kintinuous into a full SLAM system  (iSAM) with loop-closures to create fully consistent 3D meshes of entire buildings. More details including the output PCD files and a technical paper `are available here <http://www.cs.nuim.ie/research/vision/data/rgbd2012/index.html>`_.



.. blogpost::
  :title: FoVis Visual Odometry Library released
  :author: mfallon
  :date: 03-30-2012

  Just a quick note that the visual odometry library we have been using within our particle filter - called FoVis (Fast Visual Odometry) - has been released. It supports both stereo cameras (including Pt Grey Bumblebee) and depth cameras (MS Kinect, Asus Pro Live) and achieves 30Hz using a single CPU core. I haven't used another VO system but its highly recommended.

  FoVis was developed by Nick Roy's group here in MIT. More details can be found in this paper:

  Visual Odometry and Mapping for Autonomous Flight Using an RGB-D Camera. Albert S. Huang, Abraham Bachrach, Peter Henry, Michael Krainin, Daniel Maturana, Dieter Fox, and Nicholas Roy Int. Symposium on Robotics Research (ISRR), Flagstaff, Arizona, USA, Aug. 2011

  You can `download the paper and the library from here <http://code.google.com/p/fovis>`_.


.. blogpost::
  :title: Testing WebGL
  :author: mfallon
  :date: 03-21-2012

  Test of WebGL and pcl:

  .. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://pointclouds.org/assets/pcd/pcl_logo.pcd" align="center" width="800" height="315" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>

  Original:  

  .. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://people.csail.mit.edu/mfallon/share/pcl/viewer/pcl_logo.pcd" align="center" width="800" height="315" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>


.. blogpost::
  :title: Overview of Computation
  :author: mfallon
  :date: 03-21-2012

  So I've been doing some analysis to determine the overall benefit of the improvements mentioned below within our application. It will give a feel for the impact of various improvements in future.

  **High Level Timing**

  First lets look at the high level computing usage. We did several tests and looked at elapsed time for different numbers of particles and also looked at (1) doing the likelihood evaluation on the GPU (using the shader) or (2) on the CPU (as previously).

  .. image:: images/timing_per_function_high_level.png
    :alt: high level computation usage
    :align: center
    :width: 1007 px

  The data in this case was at 10Hz so real-time performance amounts to the sum of each set of bars being less than 0.1 seconds. For low number of particles, the visual odometry library, FoVis, represents a significant amount of computation - more than 50 percent for 100 particles. However as the number of particles increases the balance of computation shifts to the likelihood function. 

  The visual odometry and likelihood function can be shifted to different threads and overlapped to avoid latency. We haven't explored it yet.

  Other components such as particle propogation and resamping are very small fractions of the overall computation and are practically negiligible.
  
  In particular you can see that the likelihood increases much more quickly for the CPU evaluation.

  **Low Level Timing  of Likelihood**

  The major elements in the likelihood are the rendering of the model view and the scoring of the simulated depths - again using the results of these same runs.

  .. image:: images/timing_per_function_low_level.png
    :alt: low level computation usage
    :align: center
    :width: 942 px

  Basically the cost of rendering is fixed - a direct function of the (building) model complexity and the number of particles. We insert the ENTIRE model into OpenGL for each iteration - so there will be a good saving to be had by determining the possibly visible set of polygons to optimize this. This requires clustering the particles and extra caching at launch time but could result in an order of magnitude improvement.

  It's very interesting to see that the cost of doing the scoring is so significant here. Hordur put a lot of work into adding OpenGL Shader Language (OpenGLSL) support. The effect of it is that for large numbers of particles e.g. 900 scoring is only 5% of available time (5% of 0.1 seconds). Doing this on the CPU would be 30%.

  For the simplified model, this would be very important as the scoring will remain as is, but the render time will fall a lot.

  NOTE: I think that some of these runs are slightly unreliable. For portions of operation there was a charasteristic chirp in computation during operation. I think this was the GPU being clocked down or perhaps my OpenGL-based viewer soaking up the GPU. I'll need to re-run to see.

.. blogpost::
  :title: OpenGL/GPU Optimizations
  :author: mfallon
  :date: 02-27-2012

  In this post I'm going to talk about some GPU optimizations which have increased the speed of the likelihood function evaluation. Hordur Johannsson was the one who did most of this OpenGL magic.

  **Evaluating Individual Measurement to Model Associations**

  This is the primary approach of this method. Essentially by using the Z-buffer and an assumption of a generative ray-based cost function, we can evaluate the likelihood along the ray rather than the distance in Euclidean space. This is as was previously discussed below, I just mention it here for context.

  **Computing Per-pixel Likelihood Functions on the GPU using a Shader**

  The likelihood function we are currently evaluating is a normalized Gaussian with an added noise floor:

  .. image:: images/pixel_lh.png
    :alt: pixel lh equation
    :align: center
    :width: 654 px

  Previously this was computed on the CPU by transferring the depth buffer back to the CPU from the GPU. We have instead implemented a GPU shader to compute this on the GPU.

  Currently we look-up this function from a pre-computed look-up table. Next we want to evaluate this functionally, to explore the optimization of function's shape as well as things like image decimation. (Currently we decimate the depth image to 20x15 pixels).

  **Summing the Per-pixel Likelihood on the GPU**

  Having evaluted the log likelihood per pixel, the next step is to combine them into a single log likelihood per particle by log summation:

  .. image:: images/particle_lh.png
    :alt: particle lh equation
    :align: center
    :width: 376 px

  This can be optimized by parallel summation of the pixel images: e.g. from 32x32 to 16x16 and so on to 1x1 (the final particle likelihood).  In addition there may be some accuracy improvement by summing simularly sized values and avoiding floating point rounding errors: e.g. (a+b) + (c+d)    instead of ((a+b)+c) +d

  We're currently working on this but the speed up is unlikely to be as substantial as the improvement in the previous section.

  **Single Transfer of Model to GPU**

  An addition to the above, previously we used the mixed polygon model. We have now transferred to using only a model made up of only triangles. This allows us to buffer the model on the GPU and instead we transmit the indices of the model triangles which should be rendered in a particular iteration. (Which is currently all the triangles).

  Future work will look at determining the set of potentially visible polygons - perhaps using a voxel-based indice grid. This is more complex as it requires either implicit or explicit clustering of the particle set.

  In addition to the above, we are now using an off screen buffer which allows us to renderer virtual images without being limited by the resolution of the machine's specific resolution.

  **Putting it all together**

  We've benchmarked the improvements using a single threaded application which carries out particle propogration (including Visual Odometry), renderering, likelihood scoring and resampling. The test log file was 94 seconds long at about 10Hz (about 950 frames in total). We are going to focus on the increased number of particles for real-time operation with all modules being fully computed.

  The biggest improvement we found was using the triangle model. This resulted in about a 4 times increase in processing speed. Yikes!

  .. image:: images/tri_versus_polygons.png
    :alt: particle lh equation
    :align: center
    :width: 900 px

  Using the triangle model, we then tested with 100, 400 and 900 particles the effect of computing the likelihood on the GPU using a shader:

  .. image:: images/gpu_shader.png
    :alt: particle lh equation
    :align: center
    :width: 900 px

  This results in a 20-40% improvement, although we would like to carry out more sample points to verify this. The result of this is that we can now achieve real-time performance with 1000 particles. For the log file we didn't observe significant improvement in accuracy beyond about 200 particles - so the next step is to start working with more aggressive motion and data with motion blur. There, being able to support extra particles will become important.

  In my next post I'll talk about how that computation is distributed across the various elements of the application.


.. blogpost::
  :title: RGB-D Localization Software Framework
  :author: mfallon
  :date: 02-14-2012

  An overview of the system we are developing is illustrated at the bottom of this post. Sensor output in illustrated in lemon color, although only RGB-D data is considered in this project. Modules we have been actively developing are colored blue. The module in orange is our Incremental Motion Estimation algorithm. This module current utilizes a feature-based visual odometry algorithm. One alternative to this approach would be Steinbrucker et al. (ICCV 2001) which was developed specifically for RGB-D.

  .. image:: images/system_graph.png
    :alt: System Diagram
    :align: center
    :width: 750 px

  We have also illustrated in pink three modules which would naturally augment the point cloud-based localization system by providing redundancy and recovery methods. We have been developing these modules independently of PCL. They are illustrated here to demonstrate the flexibility of an SMC-based approach to localization.

  Some typical VO performance for motion along a corridor. The boxes are 5m in size. We used a Kinect and the Freenect driver for this.

  .. image:: images/vo_kmcl_5m_grid.png
    :alt: Typical VO Performance
    :align: center
    :width: 750 px

  I'm currently finalizing a particle filter which used Eigen Isometry3d and boost. My old version used GSL for random number generation - which I want to move away from.

.. blogpost::
  :title: Quantifying Performance
  :author: mfallon
  :date: 01-20-2012

  So as to quantify the benefit of using our depth image simulation mode for localization, we need to do some benchmarking. There are a number of parameters we need to test:
  
  * number of particles
  * accuracy and/or completeness of the model
  * downsampling rate of the incoming imagery
  * Framerate required for successful tracking

  And some metrics:

  * Achieveable framerate [fps]
  * Mean Error [in meters]
  * Per cent of frames within some distance of the true location

  Where error is measured using LIDAR-based ground truth.

  In addition to this we also developed a second type of likelihood function. This method is essentially equivalent to ICP - without the iterative. Each pixel's location is compared to the nearest point in euclidean space.

  In this figure we try to illustrate some examples of where the scores would be substantially different. So we have an RGB-D sensor sensing two depth samples (purple) something like the end of a corridor (green, from above):

  .. image:: images/ranging.png
    :alt: Illustration of ranging issue
    :align: center
    :width: 400 px

  For the depth image simulation method (blue, Ray-to-Plane) the likelihood would be formed by comparing with the difference in depth (blue star). For the ICP-type method (red, point-to-plane) the distance can be very different. For the lower case the distances are approximately the same. For the upper case, the association is very different. Also as the ICP-type method requires searching over the entire set of planes for the explict correspondence and it is also quite expensive.

  We wanted to test what effect this choice has on accuracy. Below are some figures showing the results across an extensive experimentation. The figures were produced with my 2 year old 4-core 2.53GHz Pentium Core2 with an Nvidia Quadro 1700M with 32-cores. Each result is the average of 20 independent Monte Carlo runs. Total testing is equivalent to 16 hours of runtime.

  First of all, error broadly converges to about the same accuracy as the number of particles increases: 

  .. image:: images/stats.png
    :alt: Error for both methods with varying number of particles
    :align: center
    :width: 600 px

  .. image:: images/stats1.png
    :alt: Failure rate for both methods with varying number of particles
    :align: center
    :width: 600 px

  The 50cm figure is largely because we aim to maintain multi-modal estimate - so as to be more robust when tracking. To get the optimal performance for each frame you could use ICP using the most likely particle as the starting condition. We haven't done that here. 

  This figure shows the timing performance as the number of particles increases:

  .. image:: images/time_stats.png
    :alt: Framerate for both methods with varying number of particles
    :align: center
    :width: 600 px

  **OUR** implementation of the nearest plane lookup was pretty slow. However our target framerate of 10Hz was achieved with 100 particles for the depth image likelihood function (ray-to-plane). As you saw above, for 100 particles the error has typically converged, so thats been sufficient to be realtime with the type of motion you see in the figure below.

  For all of these figures we decimated the original RGB-D image by a factor of 32. Next we would like to look at what effect that has - especially now that my new laptop has 256 GPU cores.

  Additionally we want to look at sub-dividing the full model, so that only the planes near to the particle poses [within 20m] are passed to OpenGL for testing. This is likely to give us a substantial performance improvement. I believe that 1000 particles at 10Hz should be achieveable.

  Interestingly the approach developed by Ryohei Ueda during his internship at PCL/Willow Garage is very close to what we do here: the tracking paradigm is essentially inverted. Later in this project it would be interesting to apply depth image simulation method to his tracking algorithm and see if it can be speeded up. Given the object models he was using are much smaller than a building model, it should do.

.. blogpost::
  :title: Bugs removed and global alignment verified
  :author: mfallon
  :date: 12-09-2011

  After some prodding from Christian at Willow, we fixed a few bugs with our coordinate frames. Thanks! Applying the camera transform to the point clouds 
  now results in perfect registeration of two views. This is three views of a teapot without noise or quantization:

  .. image:: images/clean_teapot.png
    :alt: Simulated Data
    :align: center
    :width: 600 px

  And with noise and quantization:

  .. image:: images/noisy_teapot.png
    :alt: Simulated Data
    :align: center
    :width: 600 px

  Basically the bugs came about when we inverted the Z axis when reading the depth buffer making our system left handed. (Stop hating on us leftys!) This is the coordinate frames we're using now.

  * OpenGL:  +X right, +Y up, +Z backwards out of the screen
   
  * Computer Vision and PCL: +X right, +Y down, +Z into the view
  
  * Robotics: +X right, +Y forward, +Z up

  We're just about ready to add shader-based cost functions.

.. blogpost::
  :title: Added PLY support (including colour) to simulator
  :author: mfallon
  :date: 12-01-2011

  We are getting pretty close to having a complete RGB-D simulator integrated into PCL. Below you can see some figures showing:

  * Top Right: a view of the model (complete with garish default colors)

  * Top Left: the depth image from OpenGL's depth buffer

  * Bottom: the same information in the PointCloud viewer (including color)

  .. image:: images/sim.png
    :alt: Simulated Data
    :align: center
    :width: 600 px

  .. image:: images/sim2.png
    :alt: Simulated Data 2
    :align: center
    :width: 600 px

  Note the disparity-based quantization and the Gaussian noise. A fully realistic simulator will be much more complicated though! The images correspond to a 3D model of our 
  Stata Center building which we have, at about this location:

  .. image:: images/third_floor.jpg
    :alt: Third floor view at MIT's Stata Center
    :align: center
    :width: 300 px

  In addition here is the RangeImage using Bastian Steder's work - which we've integrated:

  .. image:: images/bs_ri.png
    :alt: Range Image created using pcl->RangeImage library
    :align: center
    :width: 300 px

  For some reason it appears very small, hence the low resolution (to-be-fixed). We haven't done much else but feature extraction, segmentation
  registeration should be possible and it could be useful for unit testing and stochastic 

  We have a GLUT-based application takes as input a single .ply file and the user can use a mouse to 'drive around and take shots. 
  For the really interested, you can try out our range range-test program in pcl/simulation. Perhaps its useful to people who 
  have had problems using OpenNi. We are re-writing it using VTK currently.

  Here's the sample ply file to use from MIT's Stata Center:

  http://people.csail.mit.edu/mfallon/share/pcl/stata_03.ply

  We've been talking with Alex about combining efforts - towards a library for point cloud simulation. 
  Hordur Johannsson has been looking at using OpenGL Shaders to do comparison between these types of simulated views and real data - to give a measure of a match between to 
  images.

  NOTE: the models were generated by students working with Professor Seth Teller. More details and models here:

  http://rvsn.csail.mit.edu/stata/

.. blogpost::
  :title: Adding a simulated RGB-D sensor to PCL
  :author: mfallon
  :date: 11-17-2011

  The first major step is to port over our work for generating simulated range images from a global world view. 
  It uses OpenGL to render simulated image views in much the same way as it would for a gaming application.
  The implementation has a few extra bells which mean that arrays of (smaller) views can be read efficiently. This 
  allowed us to achieve 100s of simulated views at 10s fps. 

  When this is fully working, there is a test progam which will allow the user to "drive" around a simulated world
  and generate a log of RGB-D data.

  .. image:: images/depth_images.png
    :alt: Left simulated Depth images on the, right read measured depth image
    :align: center
    :width: 900 px

  The maps we were using previous used a pretty funky file format: basically each plane was read from an individual PCD
  file - so litterally 1000s of files were read in to build the map. The next step in our work is to enable support for 
  obj, vtk, ply file types. Thankfully PCL already has good support for reading these files.	

.. blogpost::
  :title: Global Point Cloud Localization: Project Intro
  :author: mfallon
  :date: 11-16-2011

  Our contribution to the Toyota Code Sprint will be focused on Global Point Cloud Localization.

  We've been working on this problem previously using RGB-D/Kinect sensors. You can see an overview of our work over on the main part of pointclouds.org by `clicking here <http://www.pointclouds.org/news/kmcl.html>`_

  This video provides an overview of the method:

  .. raw:: html

    <iframe title="Kincet Monte Carlo Localization" width="480" height="390" src="http://www.youtube.com/embed/wBdz5JJJYhg?rel=0" frameborder="0" allowfullscreen></iframe>

  The problem is summarised as localizing the sensor/robot/vehicle within a prior map of points or higher level objects (plane, curves or even building models). Its common to use probabilistic methods such as Sequential Monte Carlo to propogate a belief in a certain state.

  What prior work is there on this domain? Well for one, the Google Autonomous car is localized in this way!

  Related to this we hope to extend PCL support for simulated sensors as well as exploring optimization and improvement of these probabilistic methods. In terms of simulation, the addition of various noise models is also important given the various wacky effects that we see with the Kinect/Xtion/Primesense devices.
