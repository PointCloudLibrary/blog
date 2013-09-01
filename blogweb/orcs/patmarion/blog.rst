.. blogpost::
   :title: ORCS Kickoff
   :author: patmarion
   :date: 11-26-2012

   I'm excited to start the PCL - Ocular Robotics code sprint.  This week I'll have a kickoff meeting over Skype with engineers from Ocular Robotics.
   In the meantime, I'm reading about the RE0x sensors and updating my PCL build directories!


.. blogpost::
  :title: RobotEye lidar scan animation
  :author: patmarion
  :date: 01-05-2013

    Ocular Robotics sent me some lidar scans to get started with.  The data is stored in binary files with azimuth, elevation, range, and intensity fields.
    I wrote some code to load the binary data and convert it to the pcl::PointCloud data structure.  From there, I saved the data in the pcd file format and
    opened it with `ParaView <http://paraview.org>`__ using the `PCL Plugin for ParaView <http://www.paraview.org/Wiki/ParaView/PCL_Plugin>`__.
    I used ParaView's animation controls to create a video of the lidar scan:

  .. raw:: html

    <iframe width="640" height="480" src="http://www.youtube.com/embed/lSns6qgU-tA?rel=0" frameborder="0" allowfullscreen></iframe>


.. blogpost::
  :title: RobotEye Viewer
  :author: patmarion
  :date: 02-28-2013

    I've finished developing a new pcl::Grabber called the pcl::RobotEyeGrabber, and a visualization application called RobotEye Viewer.
    The grabber uses the boost asio library to do asynchronous I/O on a UDP socket using a separate thread.  The RobotEye sensor sends
    lidar data in UDP data packets which are converted by the grabber into pcl::PointCloud data.  The RobotEye Viewer is a Qt application
    that uses the QVTKWidget to embed a pcl::visualization::PCLVisualizer visualization window.

    I developed the code using MacOSX and Ubuntu Linux.  I ssh to a machine that is located at the Ocular Robotics lab and has a live
    sensor connected to it. With this machine, I was able to run the RobotEye Viewer application remotely using the RE05 lidar sensor.
    Ocular Robotics setup a webcam that points at the RobotEye sensor so I can see it in action as I run the application
    remotely.  This setup worked out very well.  Here's a screenshot of the application, stay tuned for a video.

  .. image:: images/RobotEye-Viewer-screenshot.png
    :alt: Screenshot of RobotEye Viewer application
    :width: 700 px


.. blogpost::
  :title: RobotEye Viewer
  :author: patmarion
  :date: 04-15-2013

    Here's a video demonstrating the RobotEye Viewer application with the Ocular Robotics `RE05 lidar sensor <http://www.ocularrobotics.com/Products/RE05/Overview.html>`__.
    See my previous blog post for a description of the application.


  .. raw:: html

    <iframe width="640" height="360" src="http://www.youtube.com/embed/ceHeWA93fj4?rel=0" frameborder="0" allowfullscreen></iframe>
