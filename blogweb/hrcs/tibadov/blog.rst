.. blogpost::
   :title: Hello World!
   :author: tibadov
   :date: 05-14-2013

   The project "Stereo-based road area detection" has been started!

.. blogpost::
   :title: Progress on stereo-based road area detection.
   :author: tibadov
   :date: 06-05-2013

   A few words about the project: the goal is to detect a drivable area (continuously flat area, segmented by a height gap such as curb). As an input we have two rectified images from the cameras on the car's roof and a disparity map. An example of such images is below.

   .. image:: img/view_example.png
      :align: center

   The point cloud, that was computed from them:

   .. raw:: html

    <center><iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/hrcsweb/source/tibadov/files/cloud_example.pcd" width="770" height="480" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe></center>

   The point cloud is converted into the Digital Elevation Map (DEM) format to simplify the task. DEM is a grid in column-disparity space with heights associated to each node. A projection of DEM onto the left image is illustrated below.

   .. image:: img/DEM_vizualization.png
      :align: center

   On the following image you can see that despite of a low resolution of the DEM it is still possible to distinguish the road from the sidewalk.

   A front view of the DEM in 3D space (nodes without corresponding points, i.e. the disparity map had no points, that should be projected onto this node, are marked with red):

   .. image:: img/DEM_front_labeled.png
      :align: center

   DEM as a point cloud:

   .. raw:: html

    <center><iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/hrcsweb/source/tibadov/files/DEM.pcd" width="770" height="480" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe></center>

   As a starting point we are going to implement an algorithm: J. Siegemund, U. Franke, and W. Forstner, “A temporal filter approach for detection and reconstruction of curbs and road surfaces based on conditional random fields,” in Proc. IEEE Intelligent Vehicles Symp., 2011, pp. 637–642.

   All source code related to the project can be found `here <https://github.com/TIbadov/pcl>`_.

.. blogpost::
   :title: Start labeling
   :author: tibadov
   :date: 07-08-2013

   For evaluation of the developed algorithm we need ground truth so we decided to outsource manual labelling. For this purpose a highly efficient tool was developed. A person could easily solve this task however there are some difficulties.

   First of all, what to do, if there are several separated roads in the scene? The solution is to mark as "road" only the pixels of the road, on which the vehicle is. Below is an example of such frame (left) and the labeling for it (right).

   .. image:: img/labeled_example_1.png
      :align: center

   How to label an image if two different roads on the previous frames are merging in the current frame? We decided to mark pixels of the first road and the second road pixels lying above the horizontal line, drawn through the end of the curb that separates the roads. Here an example for explanation:

   .. image:: img/labeled_example_2.png
      :align: center

   We optimize manual labelling time in 10 times in contrast to our initial version and now we could obtain enough labelled data in reasonable time. All results will be publicly available later.

.. blogpost::
   :title: Labeling of the Castro dataset is finished
   :author: tibadov
   :date: 08-27-2013

   We are pleased to report, that labeling of the Castro dataset (6440 frames) is finished. Here are some examples of labeled images:

   .. image:: img/castro_labeled.png
      :align: center

   We also tested an algorithm, developed by `Alex Trevor <http://www.pointclouds.org/blog/hrcs/atrevor/index.php>`_ in the previous HRI code sprint. This algorithm segments points using their normal distribution, this makes it very sensitive to noise. 

   Basically, this algorithm computes a disparity for a stereo pair using its own dense matching method, implemented by Federico Tombari. But I additionally tested it using disparity maps precomputed by HRI. Here are the typical results (left - disparity is computed with Federico Tombari's method, right - precomputed by HRI):

   .. image:: img/castro_results_hrcs1.png
      :align: center

   You can see that Federico Tombari's method is friendlier to the normal-based algorithm. But it is not good enough for describing a scene, there are a lot of false positives.

   Some noise is presented at the HRI's disparity maps, even a lot of pixels have no valid disparity, sometimes there are no segments that are similar to road and there are a lot frames in which road was not found.

   This algorithm has thresholds for disparity and doesn't mark as "road" any point which doesn't satisfy these thresholds. I didn't take it to account because it would make these results not completely correct. Therefore, 50% recall would be a very good result.

   The goal is find all pixels, that belong to the road. Total results are on the image below (precision is a ratio of right detected road's points to all detected pixels, recall is a percent of detected road's points):

   .. image:: img/castro_results_hrcs1_total.png
      :align: center