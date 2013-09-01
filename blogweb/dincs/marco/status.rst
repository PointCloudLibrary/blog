M status updates
=================

.. blogpost::
    :title: Tutorial of the Dinast Grabber Framework
    :author: marco
    :date: 02-07-2013
    
    **The PCL Dinast Grabber Framework**

    At PCL 1.7 we offer a new driver for Dinast Cameras making use of the generic grabber interface that is present since PCL 1.0. This tutorial shows, in a nutshell, how to set up the pcl grabber to obtain data from the cameras. 
  
    So far it has been currently tested with the `IPA-1110, Cyclopes II <http://dinast.com/ipa-1110-cyclopes-ii/>`_ and the `IPA-1002 ng T-Less NG <http://dinast.com/ipa-1002-ng-t-less-ng-next-generation/>`_ but it is meant to work properly on the rest of the Dinast devices, since manufacturer specifications has been taken into account.
    
    .. image:: images/07022013/camerass.png
       :height: 390px
       :align: center
    
    **Small example**

    As the Dinast Grabber implements the generic grabber interface you will see high usage similarities with other pcl grabbers. In *applications* you can find a small example that contains the code required to set up a pcl::PointCloud<XYZI> callback to a Dinast camera device.
    
    Here you can see a screenshot of the PCL Cloud Viewer showing the data from a cup laying on a table obtained through the Dinast Grabber interface:
    
    .. image:: images/07022013/cup.png
       :height: 390px
       :align: center
    
    And this is a video of the PCL Cloud Viewer showing the point cloud data corresponding to a face:
    
    .. raw:: html
      :align: center

     <center><iframe title="PCL Dinast Grabber example" width="480" height="390" src="https://www.youtube.com/embed/6hj57RfEMBI?rel=0" frameborder="0" allowfullscreen></iframe></center>
     
    Dinast Grabber currently offer this data type, as is the one currently available from Dinast devices:
    
    * `void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&)`
     
    **The code**
    
    The code from *apps/src/dinast_grabber_example.cpp* will be used for this tutorial:
    
    .. code-block:: cpp
     :linenos:

      #include <pcl/common/time.h>
      #include <pcl/point_types.h>
      #include <pcl/io/dinast_grabber.h>
      #include <pcl/visualization/cloud_viewer.h>

      template <typename PointType>
      class DinastProcessor
      {
        public:
          
          typedef pcl::PointCloud<PointType> Cloud;
          typedef typename Cloud::ConstPtr CloudConstPtr;
          
          DinastProcessor(pcl::Grabber& grabber) : interface(grabber), viewer("Dinast Cloud Viewer") {}

          void 
          cloud_cb_ (CloudConstPtr cloud_cb)
          {
            static unsigned count = 0;
            static double last = pcl::getTime ();
            if (++count == 30)
            {
              double now = pcl::getTime ();
              std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
              count = 0;
              last = now;
            }
            if (!viewer.wasStopped())
              viewer.showCloud(cloud_cb);
          }
          
          int 
          run ()
          {
                  
            boost::function<void (const CloudConstPtr&)> f =
              boost::bind (&DinastProcessor::cloud_cb_, this, _1);
            
            boost::signals2::connection c = interface.registerCallback (f);

            interface.start ();
            
            while (!viewer.wasStopped())
            {
              boost::this_thread::sleep (boost::posix_time::seconds (1));
            }
            
            interface.stop ();
            
            return(0);
          }
          
          pcl::Grabber& interface;
          pcl::visualization::CloudViewer viewer;  
          
      };

      int
      main () 
      {
        pcl::DinastGrabber grabber;
        DinastProcessor<pcl::PointXYZI> v (grabber);
        v.run ();
        return (0);
      }

    **The explanation**  
  
    At first, when the constructor of DinastProcessor gets called, the Grabber and CloudViewer Classes are also initialized:

    .. code-block:: cpp
    
      DinastProcessor(pcl::Grabber& grabber) : interface(grabber), viewer("Dinast Cloud Viewer") {}
    
    At the run function what we first have is actually the callback and its registration:

    .. code-block:: cpp    
   
      boost::function<void (const CloudConstPtr&)> f =
        boost::bind (&DinastProcessor::cloud_cb_, this, _1);
            
      boost::signals2::connection c = interface.registerCallback (f);
    
    We create a *boost::bind* object with the address of the callback *cloud_cb_*, we pass a reference to our DinastProcessor and the argument place holder *_1*.
    The bind then gets casted to a boost::function object which is templated on the callback function type, in this case *void (const CloudConstPtr&)*. The resulting function object is then registered with the DinastGrabber interface. 
    
    The *registerCallback* call returns a *boost::signals2::connection* object, which we do not use in the this example. However, if you want to interrupt or cancel one or more of the registered data streams, you can call disconnect the callback without stopping the whole grabber:
    
    .. code-block:: cpp

      boost::signals2::connection = interface (registerCallback (f));

      // ...

      if (c.connected ())
        c.disconnect ();
    
    After the callback is set up we start the interface.
    Then we loop until the viewer is stopped. Finally interface is stopped although this is not actually needed since the destructor takes care of that.
    
    On the callback function *cloud_cb_* we just do some framerate calculations and we show the obtained point cloud through the CloudViewer.
    
    **Testing the code**
    
    We will test the grabber with the previous example. Write down the whole code to a file called *dinast_grabber.cpp* at your preferred location. Then add this as a *CMakeLists.txt* file:
      
    .. code-block:: cmake
      :linenos:
    
      cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

      project(dinast_grabber)

      find_package(PCL 1.7 REQUIRED)

      include_directories(${PCL_INCLUDE_DIRS})
      link_directories(${PCL_LIBRARY_DIRS})
      add_definitions(${PCL_DEFINITIONS})

      add_executable (dinast_grabber dinast_grabber.cpp)
      target_link_libraries (dinast_grabber ${PCL_LIBRARIES})
    
    Then just proceed as a usual cmake compilation::
    
      $ cd /PATH/TO/DINAST_EXAMPLE
      $ mkdir build
      $ cd build
      $ cmake
      $ make
      
    If everything went as expected you should now have a binary to test your Dinast device. 
    Go ahead, run it and you should be able to see the point cloud data from the camera::
    
      $ ./dinast_grabber
      
    **Troubleshooting**
   
    **Q:** When I run the application I get an error similar to this one::
    
     $ ./dinast_grabber 
     libusb: 0.000000 error [op_open] libusb couldn't open USB device /dev/bus/usb/002/010: Permission denied.
     libusb: 0.009155 error [op_open] libusb requires write access to USB device nodes.
    
    Where the last numbers of the */dev/bus/usb/...* might vary.
    
    **A:** This means you do not have permission to access the device. You can do a quick fix on the permissions of that specific device::
    
     $ sudo chmod 666 /dev/bus/usb/002/010
    
    Or you can make this changes permanent for all future Dinast devices writing a rule for udev. 
    In debian-like systems it is usually done writing this::
    
      # make dinast device mount with writing permissions (default is read only for unknown devices)
      SUBSYSTEM=="usb", ATTR{idProduct}=="1402", ATTR{idVendor}=="18d1", MODE:="0666", OWNER:="root", GROUP:="video"
    
    to a file like */etc/udev/rules.d/60-dinast-usb.rules*.
   
    If you still have problems you can always use the users mailing list: *pcl-users@pointclouds.org* to find some extra help.
   
    **Conclusions**

    With this new grabber a new kind of short-range sensors are available through the PCL Grabber interface. 
    It is now a breeze to connect and obtain data from Dinast devices as you do with the rest of devices supported at PCL.
    
    If you have any development suggestions on these or new devices you can contact us through *pcl-developers@pointclouds.org*.
   
.. blogpost::
   :title: Update on Dinast cameras work
   :author: marco
   :date: 10-03-2012

   It has been quite a long time since my last post so I will give a full update on all the developments that I have taken care of in this time. 

   After a first implementation of the pcl::DinastGrabber for Dinast cameras (IPA-1002, IPA-1110, IPA-2001) I did some testing with multiple cameras. For that I mounted two of them on a robotics arm. After calibration, I got the combination of two pointclouds in one single view. The following picture shows the result of me standing in front of the two cameras.

	.. image:: images/10032012/bothArm.png
		:width: 738px
		:height: 536px
		:align: center

   Also here is the setup of the cameras on the robotics manipulator:

	.. image:: images/10032012/armAndCameras.png
		:width: 420px
		:height: 315px
		:align: center

   Then we used the RRT implementation with the robotic arm for planning purposes. The calibrated cameras where also used to build a collision shield around the robot. First RRT was run to get the path to the goal. When the robot was moving, collision checking was performed using the information obtained from the cameras. When an object was detected, the robot was stopped and RRT was run again in order to obtain a new path to the goal avoiding the detected object. The pic below shows the replanning part of the whole testing, while trying to avoid a box that was in the path to the goal.

	.. image:: images/10032012/replanningRRT.png
		:width: 420px
		:height: 315px
		:align: center


.. blogpost::
   :title: Testing the PCL DinastGrabber
   :author: marco
   :date: 08-11-2012

   Cameras just arrived! After some minor troubles with the shipping the cameras have finally arrived, so here is a picture of what I got:

	.. image:: images/08112012/picture-cameras.png
		:width: 738px
		:height: 536px
		:align: center

   I got two DINAST CYCLOPES II cameras and some related hardware and software. They will be used for the application on multiple cameras point cloud generation. This cameras obtain short range 3D data up to 80 cms at a resolution of 320x240. After some code tunning the PCL Grabber works properly and gets the 3D and 2D data from the cameras. Here is a video of the PCL DINAST Camera Grabber using a CYCLOPES II camera:
	
   .. raw:: html 

      <center><iframe width="420" height="315" src="http://www.youtube.com/embed/lNiPal9P5Y4" frameborder="0" allowfullscreen></iframe></center>



.. blogpost::
   :title: PCL DinastGrabber and RRT demo
   :author: marco
   :date: 30-07-2012

   Along this time I added the pcl::DinastGrabber interface to the pcl subversion along with a grabber demo example, it compiles but I cannot say it works since I still do not have the sensors for testing. I have not messed around much with the code since I will first would like to test that it works with the cameras. As soon as they are here I will clean the code so it looks nice and meets the PCL C++ Programming Style Guide. So in the meantime I have implemented a simple Rapidly Exploring Random Tree (RRT) demo with some visualization and I have recorded some of its performance on a little video. It first shows a 2D RRT with no visualization, which you can see gets to the goal pretty fast, then I added visualization and it takes some more time, and finally I showed the 3D RRT with visualization that takes quite a long time to reach the goal (even the 3D with no visualization also takes long time). The goal is at x=100,y=100,z=100 and starting point at the x=0,y=0,z=0 for the 2D version the z dimension is always set to 0. The blue dot is the starting point and the red one the goal, points added to the tree are in green and the white lines represent the edges. Here you have the video:

   .. raw:: html 

      <center><iframe width="420" height="315" src="http://www.youtube.com/embed/YP7KpfHKo3A" frameborder="0" allowfullscreen></iframe></center>

   I expect the sensors to arrive soon this week so I can get hands on on some more coding. I will be also adding some collision checking on the RRT algorithms.
	
.. blogpost::
  :title: Warming up
  :author: marco
  :date: 18-07-2012
	
  First blog post just to get in touch with the blogging system and write a bit about my first steps. I started spending some time reading documentation mostly related to pcl and the code sprint project. I am currently working on the pcl::DinastGrabber interface for the DINAST cameras. Two Cyclopes sensors from DINAST will be shipped to me in the next few days. As soon as they arrive I will test the pcl grabber code and get started with the realtime multiple sensors point cloud map generation. In the meantime I will also start with some basic rapidly exploring random tree (RRT) reading and coding.
  
