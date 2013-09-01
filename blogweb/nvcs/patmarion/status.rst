My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: patmarion

.. blogpost::
  :title: Point cloud streaming to mobile devices with real-time visualization
  :author: patmarion
  :date: 05-02-2012

  The following video demonstrates point cloud streaming to mobile devices with real-time visualization.  The point clouds are streamed from a desktop server to mobile devices over wi-fi and 4G networks.  Point clouds are captured using the PCL OpenNI grabber with a Microsoft Kinect sensor.  Visualization on the mobile devices is performed inside KiwiViewer using the VES and Kiwi mobile visualization framework.

  .. raw:: html

    <iframe title="Point cloud streaming" width="500" height="281" src="http://player.vimeo.com/video/41377003" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowfullscreen></iframe>

  The rendering frame rate is decoupled from the streaming rate.  In the video, the app streams roughly ten clouds per second.  In a follow up blog post I'll describe the implementation of the current code and list future development goals.  In the meantime, you can reach me on the `PCL developers mailing list <http://pointclouds.org/mailman/listinfo/pcl-developers>`_ or the `VES mailing list <http://public.kitware.com/cgi-bin/mailman/listinfo/ves>`_.  For more information about VES and Kiwi, the mobile visualization framework used in the demonstration please see the `VES wiki page <http://vtk.org/Wiki/VES>`_

  .. image:: images/pointcloud-streaming1.png
    :alt: point cloud streaming app on mobile phone
    :width: 250 px
