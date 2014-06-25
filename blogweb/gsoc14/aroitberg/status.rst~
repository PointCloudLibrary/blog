My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Alina Roitberg


.. blogpost::
  :title: Modifications of the gpu/people module and first steps for data collection
  :author: Alina Roitberg
  :date: 25-06-2014


  As the first weeks of GSOC are over I am going to summarize my progress so far. The goal of my project is to apply machine learning techniques on collected skeletal data for activity recognition. Most research in this area has been using Nite or Kinect SDK for skeleton tracking. As PCL already has a `pose detector <http://pointclouds.org/documentation/tutorials/gpu_people.php>`_ available, we want to try using it to collect skeletal information, which however, requires some modifications of the gpu/people module, which was the major focus of my work until now. 

  **First steps**
  

  Before the actual start of coding it was important to research the existing methods for action recognition. We decided to implement a novel approach published by *Ferda Ofli, Rizwan Chaudhry, Gregorij Kurillo, Reneé Vidal, Ruzena Bajcsy - "Sequence of the Most Informative Joints (SMIJ): A new representation for human skeletal action recognition"*. In the presented paper, skeleton joints are ranked based on their variance and the sequence of the resulting rankings is used as the input feature vector for classification. 

  The next step was to make a draft of activities that we are planning to recognize. We decided to stick with the set of actions proposed by the `IAS-Lab of University of Padua <http://robotics.dei.unipd.it/actions/index.php/overview>`_, which includes: *check watch, cross arms, kick, get up, pick up, point, punch, scratch head, sit down, stand, throw from bottom up, throw over head, turn around, walk and wave*.

 
  **Trying out the pcl/gpu/people module**

  Obtaining skeleton data with gpu/people module was not as easy as it seemed to be from the first sight. After the challenge of compiling the source with GPU enabled and making it run with Openni, the detector worked with following configuration:

	* Ubuntu 14.04 LTS
	* CUDA-toolkit 5.5.22
	* Openni 1.5.4
	* Avin2 Sensor Kinect driver v 5.1.2

  The tree files necessary for the detection were not provided in trunk and should be downloaded here: https://github.com/PointCloudLibrary/data/tree/master/people/results . 


  The pose detector runs on the RGB and Depth data obtained from an RGB-D sensor (we use Microsoft Kinect) and produces colour labelling of each pixel, depicting the corresponding body part. 
  After testing the detector following observations should be mentioned:

	* The people detector worked very well in certain positions, the best case includes frontal orientation, near-range with no walls or ground visible.
	* Despite the picture in the `tutorial <http://pointclouds.org/documentation/tutorials/gpu_people.php>`_ , the current implementation does not provide positions of the skeletal joints, which was also an issue for `discussion <http://www.pcl-users.org/GPU-People-Detect-Sceleton-td4029575.html>`_ . Consequently, obtaining the joint position is one of the challenges of this project.
	* The program has problems with large surfaces, mostly the walls and the floor, which are often labelled as the human. This issue also occurs in the `official demo video <https://www.youtube.com/watch?v=Wd4OM8wOO1E>`_ . As out project requires full body visibility, it is necessary to fix this problem (especially as it comes to the floor).
	* In unusual positions, especially while turning, the body segmentation is usually correct, but the labelling often fails.

  Trying out the pcl/gpu/people pose detector with one tree (left), three trees (middle), problems with large surfaces (right)

  .. image:: img/3.png
   :height: 210px

  .. image:: img/1.png
   :height: 210px

  .. image:: img/2.png
   :height: 210px


  **Extending the pose detector to estimate the positions of the joints**

  At the beginning it was necessary to browse through the code in order to understand how the program works.
  The detection happens by calling the  `process()  function <https://github.com/PointCloudLibrary/pcl/blob/master/gpu/people/src/people_detector.cpp#L183>`_  of the PeopleDetector class. In the end of the function, the variable sorted2 (with type BlobMatrix) contains the data of each body-part-label.


  There are 26 labels (currently 24 used) all together: 
  *Lfoot, Lleg, Lknee, Lthigh,Rfoot, Rleg, Rknee, Rthigh, Rhips, Lhips, Neck, Rarm, Relbow, Rforearm, Rhand, Larm, Lelbow, Lforearm, Lhand, FaceLB, FaceRB, FaceLT, FaceRT, Rchest, Lchest, Rshoulder, Lshoulder*.

  The labels *Rshoulder* and *Lshoulder* exist but are currently not implemented in the detection.


  The PeopleDetector class was extended with an array containing the coordinates of the joints and finding a way to calculate those coordinates was a major challange. 
  The first idea was to simply use the mean values of the corresponding blobs. In spite of the simplicity of this approach, the results were satisfying.
  The second idea was to use the buildTree() function, which estimates the optimal blob-tree starting from the neck and then recursively browse through the child-blobs and use their mean values. The buildTree() function uses the "ideal" lengths of the limbs to estimate the optimal blobs (those optimal values are defined `here <https://github.com/PointCloudLibrary/pcl/blob/master/gpu/people/include/pcl/gpu/people/label_common.h#L156>`_).
  I also want to thank Koen Buys for giving me tips on the calculation.

  As we are interested in the position of the limbs, using the mean values of the blobs is not always appropriate. For example, we are more interested in the upper border of the hip, which is connected to the torso, instead of the hip centroid. Besides this, we are also interested in the shoulder position, which was not implemented. The elbow label was also a special case as it usually has very small area and is often not detected. Consequently, I made some additional modifications to estimate those positions, which are described below. 

  Shoulders: 

	* Basic idea: Use the highest part of the chest blob as the shoulder position 
	* The point cloud of the left/right chest blob is extracted. 
   	* The maximum Y-value of this point cloud is calculated
   	* All the points of the chest-blob that have Y-value close to to the maximum (chosen threshold: 10 cm) are taken, their 3D mean value is calculated and used as the shoulder position

  Elbows:

   	* If an elbow-blob already exists, nothing is done: the mean value is used.
   	* Otherwise: The elbow is the point of the arm (upper arm) blob, which has the largest distance from the shoulder 

  Hips:

   	* The mean of the “lowest points” (in a certain threshold) of the hip-blob (not the mean of the whole blob). This modification was done due to the fact that the blob itself is covering the whole torso.

  In general the quality of the joint position depends directly on the quality of labelling. As no tracking is implemented yet, the movement of the joints is not continuous.

  Skeleton visualization: "good" example with "checking watch" action (left), labelling fails when the floor is visible (right)

  .. image:: img/4.png
   :height: 300px

  .. image:: img/5.png
   :height: 300px

  **Using Ground Plane People Detector for body segmentation**

  As mentioned before, the original People Pose Detector has some problems with large surfaces, especially the floor. We tried to solve this problem by combining the original People Pose Detector with `Ground Plane People Detector <http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php>`_ (implemented by my mentor, Matteo Munaro), to segment the body cluster before the actual labelling. 
  
  In the resulting application, at first the three points of the ground plane are selected, after which the Ground Plane People Detector removes the ground plane and estimates the point cloud belonging to the human. The points of the cluster are then transformed to the depth image, setting all other depth pixels to very high values. 

  Some additional corrections were added to improve the segmentation results (depth filtering, extending the legs, as too many ground floor points are removed). Additionally, the RGB and Depth calibration (8 pixel shift) is done as proposed by `Lingzhu Xiang <http://www.pointclouds.org/blog/gsoc14/lxiang/index.php>`_ .

  Using the Ground Plane People Detector improves the performance significantly if the full body visibility is required as it completely solves the large-surfaces-problem. 

  It should also be considered, what should be done if the Ground Plane People Detector does not detect the human (meaning that none of the detected clusters had confidence over the defined threshold). In this case we use the segmentation from the last frame, in which the person was detected.

  Pose detector with full body visibility without (left) and with (right) segmentation.

  .. image:: img/8.png
   :height: 250px

  .. image:: img/9.png
   :height: 250px

  Examples of activities: cross arms, check watch, kick, point, turn, wave

  .. image:: img/cross_arms.png
   :width: 400px
   :height: 170px

  .. image:: img/watch.png
   :width: 400px
   :height: 170px

  .. image:: img/kick.png
   :width: 400px
   :height:  170px


  .. image:: img/point.png
   :width: 400px
   :height: 170px


  .. image:: img/turn.png
   :width: 400px
   :height:  170px


  .. image:: img/wave.png
   :width: 400px
   :height: 170px
 
  **Storing the data**

  I am currently working on completing the framework for data collection. Storing skeletal information (3-D position of the joints) in TXT files is already implemented. People Pose Detector already includes optional storage of depth and RGB-data as PNG images. However, we decided to store the RGB and Depth images with more efficient method using the **lzf-format** (thanks to Lingzhu Xiang for the tip). Another idea I am working right now is to run the people pose detector offline on the stored images to use the full speed. 
  
  

