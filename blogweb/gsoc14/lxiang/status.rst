My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Lingzhu Xiang


.. blogpost::
  :title: Dataset collection and curiosities in Kinect calibration
  :author: Lingzhu Xiang
  :date: 20-06-2014

  **Difficult poses for the current PCL people detector**

  This project aims to improve people detection capability in PCL. To validate the improvement we need measurements and data as proof. Therefore the first step of the project is to collect and establish datasets of relevant human poses. PCL's people detector assumes that human will be in upright poses instead of arbitrary poses, and the multiple stages of detection including segmentation clustering and classification in its implementation rely on such assumption, making it susceptible to varied human poses in real world. Thus I intend to collect various representative poses that deviate from upright poses. As is discussed with my mentor Matteo, those include:

  * Crouching
  * Hands up
  * Sitting
  * Dancing/running
  * Lying on sofa or floor

  These poses are not equal. Crouching, dancing, and running pose challenges to the image based human classifier which currently assume a sample window of a full-body upright pose. Sitting likely involves occlusion of some part of body and increases difficulty in segmentation. Hands up would introduce errors in segmentation or even false segmentation in the current head based subclustering as it determines each subcluster by its highest local maxima which could erroneously be the hands. While lying on sofa or floor would completely defeat the current approach of segmentation because the person is no longer an independent object to be segmented. So these are the types of data we will collect and evaluate upon.

  **Methods to collect data**

  There are different ways to collect point cloud data, but as is suggested in `OpenNI and PCL Recording Codes <https://wiki.ccs.neu.edu/display/GPC/OpenNI+and+PCL+Recording+Codes>`_, the most efficient capturing method in PCL is ``pcl_openni_image`` (or the only method that works because other ones in PCL either totally fail or explode in I/O) which grabs RGBD frames in compact pclzf format (`samples <https://github.com/PointCloudLibrary/pcl/tree/master/test/grabber_sequences/pclzf>`_). You use ``pcl_openni_image '#1'`` to test Kinect configuration and ``pcl_openni_image '#1' -visualize 0`` to save batch frames. This produces compact data stream in 400KB per frame, or 12MB/second, with point clouds reconstructed afterwards when being used. You can visualize the data with ``pcl_image_grabber_viewer -pclzf -dir captured_frames/ -fps 30``, or programmatically replace ``pcl::OpenNIGrabber`` with ``pcl::ImageGrabber`` like the image grabber viewer. The other way to collect data is to save raw frames and camera information in `ROS bags <http://wiki.ros.org/openni_launch/Tutorials/BagRecordingPlayback>`_ and play back the image frames to reconstruct the point cloud on the fly and feed depth-registered point cloud topic to PCL via ``pcl_ros``.

  **Problems with depth registration**

  pclzf format seems fine, until it is actually reconstructed to point clouds with a large bias in depth registration:

  .. image:: images/depthregisoff.jpg
    :align: center
    :height: 400px

  Depth registration is the first step of data collections. PCL's people detector makes use of image based people classifier, which relies on correctly color registered point cloud. Somehow there are some peculiarities in how we perform the depth registration and calibration prior to that. The image above shows that color of the background is mapped onto the points that belong to the person.

  My guess of this was that this might be caused by wrong calibration, or loss of calibration during the capturing and reconstructing process. So after redoing the intrinsic calibration of RGB camera and IR camera and extrinsic calibration as described in `openni_launch tutorials <http://wiki.ros.org/openni_launch/Tutorials>`_ and this `Kinect calibration tutorial <https://sites.google.com/site/rameyarnaud/research/ros/kinect-calibration>`_, I tried the other method with ROS ``openni_launch depth_registration:=true publish_tf:=true/false``. The problem with depth registration persists, and OpenNI seems to ignore my calibration parameters or frame transforms no matter what and only use its built-in factory default parameters:

  .. image:: images/roscalrviz.png
    :align: center
    :height: 400px

  It turns out `PCL does not perform any internal calibration <http://dev.pointclouds.org/issues/235>`_ at all and relies on OpenNI providing correct depth registration, and there is no way of updating OpenNI's calibration parameters. Yes you can calibrate the Kinect all you want but there is no way to make use of the result in existing code base. `This thread about what is happening behind OpenNI depth registration <http://www.pcl-users.org/What-is-happening-behind-the-Depth-to-RGB-registration-mode-in-OpenNI-td4029195.html>`_ with pointers inside is a good read.

  We can still do the whole depth registration process manually, as in `pointcloud_utils.cpp <https://github.com/ros-industrial/human_tracker/blob/develop/packages/pointcloudPublisher/src/pointcloud_utils.cpp>`_, but unfortunately the data captured by ``pcl_openni_image`` is already depth registered somehow with wrong extrinsic calibration. To avoid spending too much time on perfecting data calibration, I decided to make a minimal fix to extrinsic error in the depth registration in pclzf data by shifting the RGB layer along X axis, in pseudo-code like this::

      for (x = 0; x < width; x++)
        for (y = 0; y < height; y++)
          cloud(x, y).rgb = cloud(x + 8, y).rgb;

  The result looks mostly acceptable with some residual "borders" around and can be improved later on:

  .. image:: images/shiftingcal.png
    :align: center
    :height: 400px

  Note that green box in the right of the person.

  **Datasets and first results**

  I collected datasets of various poses standing, crouching, sitting, and lying on sofa. One standing dataset looks like this (selected thumbnails):

  .. image:: images/standing.jpg
    :align: center
    :height: 600px

  There is also this result demonstrating the failure mode of the current head based subclustering method:

  .. image:: images/segglitch.jpg
    :align: center
    :height: 400px

  These extra erroneous segments (green boxes) are caused by the hands up pose with which the current clustering method would recognize the hands as heads. So this is where I will improve upon.

  The following video are the first results on all current collected datasets. Green boxes are segmented clusters and red boxes are people detections. The minimum height is set to 0.1 so you will see lots of green boxes, which is for evaluating the current performance of the segmentation.

  .. raw:: html

    <center><iframe width="854" height="510" src="//www.youtube.com/embed/mvuBM_vCBIM" frameborder="0" allowfullscreen></iframe></center>

  Some major observations for the results:

  * Segmentation is very good and almost never misses the true person clusters.
  * Hands up pose likely generates a bunch of false positive clusters, as explained above.
  * Segmentation totally fails in the sofa dataset.
  * The classifier can have some room of improvement.

  So this is about everything for this time. Next time I will write about dataset annotation and performance improvement of some first techniques that I will implement.
