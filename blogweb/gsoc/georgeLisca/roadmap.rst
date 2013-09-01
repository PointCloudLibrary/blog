Roadmap
=======
.. _georgeLisca_roadmap:

Abstract
--------
During the summer we will work out a general framework for registration and SLAM in the PCL.

Overview
--------
The PCL contains many algorithms that are usable in the registration and SLAM context like features, kd trees and octrees. What is missing, is a general framework to combine all of it and integrate it with other SLAM algorithms from literature. This project will try to bring all of this together to create a new SLAM experience. For this we will look into different approaches out there and put them into the PCL context, meaning mainly getting familiar with the needed algorithms, integrate them into the PCL and add them to the SLAM framework.

Part 0: Getting into the PCL
----------------------------
* work through the `tutorials <http://www.pointclouds.org/documentation/tutorials/>`_ for PCL
* get familiar with `SLAM <http://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping>`_, ICP and graph SLAM variants
* write a registration prototype using the ICP in PCL

Part I: GICP
------------
As a starting point we will integrate `GICP <http://stanford.edu/~avsegal/generalized_icp.html>`_ into the PCL.

Work Plan
^^^^^^^^^
* get GICP running
* write a wrapper to use it from inside the PCL and that does the data structure conversion
* integrate it with the ICP implementation in the PCL
* write unit tests
* write documentation
* bonus: visualize the registration process

Milestone
^^^^^^^^^
A visualize of the registration using GICP. the optimal result would be a small program loading a set of .pcd files, visualize the registration one by one and prints the resulting transformations to screen as well as writing a registered point cloud to the hard drive.

Part II: RGBD-SLAM
------------------
RGBD-SLAM was one of the participants in the `ROS 3D <http://www.ros.org/wiki/openni/Contests/ROS%203D>`_ contest and by that time it used the PCL to do its computations. Due to licensing restrictions we can not integrate it right away but we can bring in all dependencies and re implement the rest.

Work Plan
^^^^^^^^^
* get `RGBD-SLAM <http://www.ros.org/wiki/openni/Contests/ROS%203D/RGBD-6D-SLAM>`_ running
* record a test data set
* find usable alternatives for its dependencies or write a wrapper around them
* port it to PCL version 1
* test it using the recorded data
* make it work using live Kinect data
* write unit tests
* write documentation

Milestone
^^^^^^^^^
A small application using data from the Kinect showing the registered point cloud as seen in the `video <http://www.ros.org/wiki/openni/Contests/ROS%203D/RGBD-6D-SLAM#Video>`_.

Part III: Integrate new Features
--------------------------------
By now :ref:`Pararth Shah <pararthshah_roadmap>`, how is working on feature base registration, should have something to test. So we should try to integrate it into the SLAM process.

Work Plan
^^^^^^^^^
* learn how to use the new code
* write an example application to use it
* integrate it into the pipeline
* find a good test case

Milestone
^^^^^^^^^


Final Result
------------
An awesome SLAM framework using all kinds of nifty algorithms capable of processing everything from Kinect to large laser scanner data as well as a tutorial how to use it.
