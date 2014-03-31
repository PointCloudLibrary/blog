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

   As a starting point we are going to implement an algorithm: J. Siegemund, U. Franke, and W. Forstner, "A temporal filter approach for detection and reconstruction of curbs and road surfaces based on conditional random fields," in Proc. IEEE Intelligent Vehicles Symp., 2011, pp. 637-642.

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

.. blogpost::
   :title: First results.
   :author: tibadov
   :date: 10-28-2013

   We have implemented an algorithm which processes frames independently (i.e. without the connection to the previous frame). Also, now we make an assumption that both of the curbs (left and right) are presented in the scene.

   Below you can see a projection of the labeled DEM to the left image. Green points correspond to the left sidewalk, blue - to the right one. Red points mark the road surface. The algorithm couldn't find the right curb on this image, so right side of the road was labeled uncorrectly. The good news is that the left curb was detected correctly.

   .. image:: img/DEM_labeled.png
      :align: center

   However our goal is to label a road on the image, not on the DEM. So, if we mark each pixel with label corresponding to the DEM's cell we get the following labeling of the road surface:

   .. image:: img/image_labeled.png
      :align: center

   You can see a lot of holes in the road area. They caused by holes in the disparity map. We decided not to fill them, because someone/something can be situated there (we have no information).

   A disparity map of this frame is shown below. Points without disparity are marked with red.

   .. image:: img/disparity_map.png
      :align: center

.. blogpost::
   :title: Results on the Castro dataset.
   :author: tibadov
   :date: 03-31-2014

   We've implemented almost everything, what was planned, and now we want to present our results.

   First of all please watch this video with results on the whole Castro dataset. Road is marked with red, left and right road borders are indicated by green and blue lines respectively.

   .. raw:: html

      <center><iframe width="640" height="480" src="http://www.youtube.com/embed/GS38zprOorg" frameborder="0" allowfullscreen></iframe></center>

   The main problem is the presence of holes in the road surface. This is caused by the holes in the input disparity maps. We decided not to inpaint them, because we have no information about scene in those points. But we will compute the quality of the labeling only in points with known disparity. It allows to estimate the results of our method independently of the quality of the method for generating disparity maps.

   Our method has a significant advantage in situations with one or both curbs are clearly visible (with corresponding sidewalks). You can compare  the result of the previous sprint's method (left) with the result of our method (right) on the same frame (which has two clearly visible sidewalks).

   .. image:: img/castro_diff.png
      :align: center

   Next, I'm going to show you the numerical results. Precision is a ratio of right detected road's points to all detected pixels, recall is a percent of detected road's points. Only points with known disparity are taken into account.

   .. image:: img/castro_results_disp.png
      :align: center
