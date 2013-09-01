My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: raymondlo84

.. blogpost:: 
  :title: Preview: Finger Tracking + Android 4.0 + Tegra 3 + Kinect + OpenNI + OpenCV
  :author: raymondlo84
  :date: 06-11-2012

    In the coming tutorial, we will show you how to implement the 'well-known' finger tracking algorithm on Kinect + Android + OpenNI + Tegra3 + OpenCV. The algorithm is indeed very simple and computationally efficient. We can achieve real-time performance easily on Tegra 3! 

    We will be using the range camera (depth map) to create a contour of our hand. Then, we will perform a simple polynomial approximate on the contour, and extract the convex hull (the peaks) as our finger tips. All of these operations can happen in real-time on the Tegra 3 @ 25fps! That's pretty impressive.

    The requirements are as follows:
	
    .. line-block::
	
      1. Tegra 3 or Tegra 2 Android device.
      2. Microsoft Kinect or Asus Xtion (not tested)
      3. NVIDIA NDK + OpenNI binary (see my previous tutorials)
      4. OpenCV binary (will be provided below)
      5. and 10 mins of your time :)


    First of all, the method provided here are simplified for real-time usage, and we can get better results if we have a better way of extracting the hand location and filtering the depth map (See Limitation section at the end).  Right now, we are thresholding on a range, i.e., we assume that the hand of a person will be positioned closer to the range camera.

    Depth Segmentation and Contour Extraction 
      .. raw:: html

        <iframe width="420" height="315" src="http://www.youtube.com/embed/stGkEniKrX0" frameborder="0" allowfullscreen></iframe>


    Bounding Boxes 
      .. raw:: html

        <iframe width="560" height="315" src="http://www.youtube.com/embed/w84Ejv0lVGI" frameborder="0" allowfullscreen></iframe>


    Simple Gesture recognition - finger and grasp detection
      .. raw:: html

        <iframe width="560" height="315" src="http://www.youtube.com/embed/cFL67qsYGfc" frameborder="0" allowfullscreen></iframe>

    Snapshot
      .. raw:: html

        <img src=http://4.bp.blogspot.com/-bQZy690-muc/T9JjC8OwICI/AAAAAAAAAME/jK6JwURtFPs/s1600/device-2012-06-07-184231.png width=640 >


    Tegra 3 Tablet + Microsoft Kinect + My custom 12V battery
      .. raw:: html 

	<img src=http://1.bp.blogspot.com/-lrhwiV6O6i0/T9kOhW7oRpI/AAAAAAAAAMQ/UU0IRPDi-zs/s1600/IMG_6990.JPG width=640 >


    Hand Tracking with 3D PointCloud Data rendering in real-time on Tegra3.
      .. raw:: html
        
        <iframe width="560" height="315" src="http://www.youtube.com/embed/R_7Zr5jDLco" frameborder="0" allowfullscreen></iframe>



    Source code:
      PCL svn repository (in about two weeks)
    
    What's next? 
      Using the OpenNI hand tracking and create several interfaces. Right now we have the point cloud data visualization on Android controlled by the Kinect at 15-20fps (with about 2 million dots).
 
   
.. blogpost:: 
  :title: OpenNI + Android FAQ
  :author: raymondlo84
  :date: 24-05-2012

  Recently, we have received several emails about the OpenNI and Android development. Here we will address some of the most frequently asked questions and some of the future directions about this project. 

  
  **FAQ:** 

  Q: What tablet have we used?
  
  Ans: We've used the Tegra 2 and Tegra 3 Development Board provided by NVIDIA. http://developer.nvidia.com/tegra-development-kits. I can safely assume that as long as your Tablet is as capable as Tegra 2, the OpenNI library shall be able to grab the depth map image and color images at 640*480 resolution at full speed. However, I have experienced some hiccups with the OpenNI drivers, especially the color image gets corrupted sometimes. 


  Q: What are the minimum OS requirements? 
  
  Ans: We've tested everything from Android 3.0 to Android 4.x. However, I noticed that I have got better performance on Android 3.x OS? That's something I'm currently working on. 


  Q: How could I know if my tablet work with the Kinect?
  
  Ans: Try dmesg under 'adb shell'. You should be able to see the Microsoft Kinect is recognized as a USB hub with camera, motor, and audio devices. If you cannot see any USB devices attached, I believe we need to look into that further. 


  Q: I've trouble getting the 'multitouch' sample code compiled? 
  
  Ans: We notice that the NDK package from NVIDIA http://developer.nvidia.com/tegra-resources. There are some dependencies that become missing in the latest SDK. I'm current working on this (basically rewriting part of the code to be conformed with the "Android Application Lifecycle in Practice: A Developer's Guide". LINK: http://developer.nvidia.com/sites/default/files/akamai/mobile/docs/android_lifecycle_app_note.pdf  . Please check this blog again in the future for a more self-contained package.


  Q: I'm getting errors when compiling the OpenNI library. What should I do? 
  
  Ans: We would highly recommend others to use the binary (as provided in the last tutorial) for the first time. I will provide a sample script for recompiling in the future.


  Q: How do I push the .so libraries to the device? 
  
  Ans: Please read the push_lib.sh file in the package. We've provided a few comments on how to remount the /system directory for read-write.   
  

  Q: Who's raymondlo84?
  
  Ans: I'm a Ph.D student studying ECE at UofT. I'm currently working with PCL and NVIDIA's summer code program and free feel to let us know what you think about the Android project. 


  Thank you.


.. blogpost::
  :title: Quick Tutorial: A simple C++ OpenNIWrapper for using OpenNI on Android Devices
  :author: raymondlo84
  :date: 14-05-2012

  **Important:** In this tutorial, we assume that you have already installed the OpenNI shared libraries as previously discussed to your device and so. If you haven't done so, please follow our *previous post* and adb push the binary libraries to the proper location Additionally, you are **required** to perform 
    .. line-block::

	mount -o devmode=0666 -t usbfs none /proc/bus/usb 

  everytime after you rebooted your device. The example code provided below has been tested on the NVIDIA Tegra 3 dev board and Microsoft Kinect only. If you would like to use Xtion or other range sensors, you may need to compile the drivers, and make appropriate updates to the modules.xml file. For more information, see our previous post. 

  To get started, we will first introduce the simple c++ wrapper we have written for handling the OpenNI calls. In this example, we will be only handling the image buffers, the color image buffer (24 bits RGB image) and the depth image buffer (16 bits single channel). I believe we can also record audio with the Kinect (?), but we have not verify that yet.  Here we have the header files from our *OpenniWrapper.h*. 

  **Sample Usage**
  
  To get started, you can download the header and cpp file here
    http://openvidia.svn.sourceforge.net/viewvc/openvidia/tegra_kinect/jni/OpenniWrapper.cpp?view=log
    http://openvidia.svn.sourceforge.net/viewvc/openvidia/tegra_kinect/jni/OpenniWrapper.h?view=log

  Our wrapper consists of three main function calls, start(), release(), and WaitAndUpdate(). By default, the openni wrapper will initialize the depth map and rgb image and runs in a separate thread (i.e., non-blocking) after the start(). To obtain the depth map or rgb images, we simply call WaitAndUpdate() (blocking call) and then provide a pointer which stores our depth and rgb image. The OpenniWrapper does not provide any memory allocation and it is our responsibility to handle the malloc and free.  


    .. line-block::

      ...
      #include "OpenniWrapper.h"
      ...
      OpenniWrapper \*openni_driver;
      ...
      int main(){
         ...  
       
         openni_driver = new OpenniWrapper();
       
         ... 
         
         //initialize the driver
         if(!openni_driver->start()){
            //something is wrong. see log files
            return 1;
         }
         
         int width = openni_driver->getWidth();
         int height = openni_driver->getHeight();
         unsigned char \*rgb_buffer = (unsigned char\*)malloc(width*height*3*sizeof(unsigned char));
         unsigned short \*depth_buffer = (unsigned short\*)malloc(width*height*sizeof(unsigned short));
         
         while(run_me){
             WaitAndUpdate(); //blocking call
             openni_driver->getRGB(rgb_buffer);
             openni_driver->getDepth(depth_buffer);
             ...
             process_buffers(); //can be multithreaded 
             ...
         }

	 //release the resources 
         openni_driver->release();
	 free(rgb_buffer);
         free(depth_buffer):
      }
  
  To compile the source with the OpenNI libraries, we need to include the OpenNI headers and the shared lib paths to the Android.mk file. Particularly, I have added 
    .. line-block::

       LOCAL_LDFLAGS += \
 	  -Lopenni_kinect_include/libs \

       LOCAL_LDLIBS += -lOpenNI
        
       LOCAL_C_INCLUDES += openni_kinect_include/include

  You can find the openni_kinect_include/include directory from the SVN respository below. Or you can simply create that directory by copying the headers from the OpenNI source directly.

  **Performance**
      .. line-block::

        CPU usage and FPS:
        PID PR CPU% S  #THR     VSS     RSS PCY UID      Name
        12539  2  60% R    14 532164K  65048K  fg app_39   com.nvidia.devtech.multi
        I/Render Loop:(12539): Display loop 0.014692 (s) per frame --> over 60fps for rendering, the capture loop is threaded.
	

    .. raw:: html

      <iframe width="560" height="315" src="http://www.youtube.com/embed/xn5AZBUJ4dU" frameborder="0" allowfullscreen></iframe>
    

    .. raw:: html
    
      <iframe width="560" height="315" src="http://www.youtube.com/embed/ODVdPoidagc" frameborder="0" allowfullscreen></iframe>
  
  **OpenNI Binary (Kinect Only)**

  http://www.eyetap.org/~raymondlo84/multitouch/openni_binary.tgz
  
  **Sample Source Code**

  http://openvidia.svn.sourceforge.net/viewvc/openvidia/tegra_kinect/

.. blogpost::
  :title: Porting OpenNI to Android 4.0 + Microsoft Kinect + Tegra 2 + Sample Code
  :author: raymondlo84
  :date: 25-04-2012

  In this tutorial, we will show you how to compile OpenNI shared libraries and install them on your Android devices. Also, we will show you how extract the depth map information from the Microsoft Kinect with the SensorKinect driver (note: the OpenNI framework does not come with any drivers! Instead it dynamically loads the modules in runtime!). Before we start, I assume that we have already installed the following packages. I have tested this setup on my Mac OSX 10.6.8, and I believe Linux users shall not have any problems replicating these. Any Windows users? :)
  

  System Requirements: 
    * root access to the device (i.e., adb shell)
    * Android NDK or Tegra Android Development Pack http://developer.nvidia.com/tegra-android-development-pack
    * Ventana Development Kits or other Android Devices which supports USB.   
    * git 
    * and more? 


  First, let's get the OpenNI sources from the Git repository

    .. line-block::

      cd ~/NVPACK/android-ndk-r7b/sources
      git clone https://github.com/OpenNI/OpenNI.git
      cd OpenNI/Platform/Android/jni
      ndk-build

  If everything goes well, we will see the following files in this directory.
    
    .. line-block::

      ls ~/NVPACK/android-ndk-r7b/sources/OpenNI/Platform/Android/libs/armeabi-v7a/ 
      Sample-SimpleRead       libOpenNI.so            libnimRecorder.so       niReg
      Sample-SimpleSkeleton   libnimCodecs.so         libusb.so
      libOpenNI.jni.so        libnimMockNodes.so      niLicense

  Now, we will compile the SensorKinect Driver.
  
    .. line-block::

      export NDK_MODULE_PATH=$HOME/NVPACK/android-ndk-r7b/sources/OpenNI/Platform/Android/jni
      mkdir ~/NVPACK/openni
      cd ~/NVPACK/openni
      git clone https://github.com/avin2/SensorKinect.git
      cd SensorKinect
      git checkout faf4994fceba82e6fbd3dad16f79e4399be0c184
      cd Platform/Android/jni
      ndk-build

  Again, if everything goes well, the ndk-build will create another set of .so shared libraries files in this directory

    .. line-block::

        ls ~/NVPACK/openni/SensorKinect/Platform/Android/libs/armeabi-v7a
	libOpenNI.so           libXnDDK.so            libXnDeviceSensorV2.so libusb.so
	libXnCore.so           libXnDeviceFile.so     libXnFormats.so

  Finally, we are ready to push these libraries on the Android device. One problem is the /system directory is read-only. Therefore, we have to remount this directory first by 
    
    .. line-block::

        adb shell
        mount -o remount rw /system
        mkdir /data/ni
  
  Then, we push the packages to the device with these commands (these are our previously compiled .so files!)
  
    .. line-block::

        adb push libOpenNI.so /system/lib
        adb push libOpenNI.jni.so /system/lib
        adb push libXnCore.so /system/lib
        adb push libXnDDK.so /system/lib
        adb push libXnDeviceFile.so /system/lib
        adb push libXnDeviceSensorV2.so /system/lib
        adb push libXnFormats.so /system/lib
        adb push libusb.so /system/lib
        adb push libnimCodecs.so /system/lib
        adb push libnimRecorder.so /system/lib
        adb push libnimMockNodes.so /system/lib
   
  In runtime, the OpenNI framework will look for these shared libraries. To inform the OpenNI which modules to load, we need to commit this .xml file to this directory. It is very very *important* that all these files are put in the properly directories!     

      .. line-block::

        adb push data/in/modules.xml /data/ni 

  Here is example modules.xml file we have used.

      .. line-block::

        <Modules>
          <Module path="/system/lib/libnimMockNodes.so" />
          <Module path="/system/lib/libnimCodecs.so" />
          <Module path="/system/lib/libnimRecorder.so" />
          <Module path="/system/lib/libXnDeviceSensorV2.so" configDir="/data/ni/" />
          <Module path="/system/lib/libXnDeviceFile.so" configDir="/data/ni/" />
        </Modules>
    
  As we can see from the modules.xml, the XnDeviceSensorV2 will look for 

       .. line-block::

         vim ~/NVPACK/openni/SensorKinect/Data/GlobalDefaultsKinect.ini
         #and we will need to set the flag UsbInterface to 1.
         UsbInterface=1
	 #assume you are in that directory
         adb push GlobalDefaultsKinect.ini /data/ni/

  Lastly, we will copy the SamplesConfig.xml to the /data/ni/ directory as well. You can find this file from the sample code in OpenNI or from our svn repository.

        .. line-block::

          adb push SampleConfig.xml /data/ni/


  That's it! Now, we have the latest OpenNI + Kinect Driver compiled and install on your Android! In the coming up tutorial, I explain how we can use these drivers with NDK with sample codes! and more tricks are coming up! (e.g., mount -o devmode=0666 -t usbfs none /proc/bus/usb) !!

  If you would like to check if the modules are installed successfully, you can run trying *niReg* on your Android device. 

       adb push niReg /system/bin
  
  and run
        
       root@android:/ # niReg -l (under adb shell)
  
  and you should see something similar to...
       
      .. line-block::

          . . .
         /system/lib/libXnDeviceSensorV2.so (compiled with OpenNI 1.5.2.23):
         Device: PrimeSense/SensorV2/5.1.0.41
         Depth: PrimeSense/SensorV2/5.1.0.41
         Image: PrimeSense/SensorV2/5.1.0.41
         IR: PrimeSense/SensorV2/5.1.0.41
         Audio: PrimeSense/SensorV2/5.1.0.41
          . . .

  Hirotaka Niisato (http://www.hirotakaster.com/) has also provided excellent tutorials on how to compile OpenNI on Android! I would like to thank him for providing some of the basic scripts for compiling the libraries! ;). However, I've trouble running the sample code he has provided. Instead, I have ported the Sample-NiSimpleRead to run on Android instead. ;) Stay tuned!

   Here is the complete script we have used: copy and paste these and that will get you started.

      .. line-block::

        #adb shell
        #mkdir /data/ni
        #mount -o remount rw /system
        #mount -o devmode=0666 -t usbfs none /proc/bus/usb
 
        adb push system/lib/libOpenNI.so /system/lib
        adb push system/lib/libOpenNI.jni.so /system/lib
        adb push system/lib/libXnCore.so  /system/lib
        adb push system/lib/libXnDDK.so /system/lib
        adb push system/lib/libXnDeviceFile.so /system/lib
        adb push system/lib/libXnDeviceSensorV2.so /system/lib
        adb push system/lib/libXnFormats.so /system/lib
        adb push system/lib/libusb.so /system/lib
        adb push system/lib/libnimCodecs.so /system/lib
        adb push system/lib/libnimRecorder.so /system/lib
        adb push system/lib/libnimMockNodes.so /system/lib
        adb push modules.xml /data/ni/
        adb push GlobalDefaultsKinect.ini /data/ni/
        adb push SamplesConfig.xml /data/ni/

