My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Alina Roitberg


.. blogpost::
  :title: Summary
  :author: Alina Roitberg
  :date: 08-13-2014
 
  Initially, the main objective of my project was to quickly obtain skeleton information using the `GPU People <http://pointclouds.org/documentation/tutorials/gpu_people.php>`_ module and focus the construction of the classification framework to recognize human actions. 
  As it turned out that strong modifications of the gpu/people module were necessary in order to obtain the desired output of skeleton positions, the main direction of this project has moved more to extending and improving the gpu/people module. 
  In the end, a simple action recognition framework was implemented, using K-Nearest-Neirghbour classifier on the positions and angles of selected joints, and RGB-D data of people performing the defined actions was collected for training.

  The project had following major contributions:
    
   * Calculation of the skeleton joint positions and their visualization (in the new People App the skeleton is visualized by default)
   * Using `Ground Plane People Detector <http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php>`_ in order to segment the body before the body labeling is applied. Without this modifications, the pose detector has big problems with large surfaces (walls, floor ...) and works robustly only in near-range. Besides this, the speed of the detector is increased as most of the voxels obtain very high depth depth values
   * Tracking of the skeleton joints (see my previous post). Alpa-Beta filter is applied on the measured joint positions.
   * Offline skeleton detection from the recorded LZF files. 
   * Classification of skeleton data sequences with K-Nearest-Neighbours
    

  The source code of the new version of the gpu/people module can be downloaded here:
  https://github.com/AlinaRoitberg/pcl/tree/master/gpu/people

  Building the app requires PCL installation with GPU enabled. For more information, see the PCL information on `compiling with GPU <http://pointclouds.org/documentation/tutorials/gpu_install.php>`_ and the tutorial for the `initial /gpu/people module <http://pointclouds.org/documentation/tutorials/gpu_people.php>`_
  

  The new application can be executed in the same way as the `initial PeopleApp <http://pointclouds.org/documentation/tutorials/  gpu_people.php>`_, while new functions can be actuvated with folowing additional flags:

  -tracking		<bool>  activate the skeleton tracking
  -alpha    		<float> 	set tracking parameter
  -beta    		<float> 	set tracking parameter
  -lzf			<path_to_folder_with_pclzf_files>
  -lzf_fps    		<int>		fps for replaying the lzf-files (default: 10)
  -segment_people       <path_to_svm_file> 	 activates body segmentation




  Tree files for people detection:
  https://github.com/PointCloudLibrary/data/tree/master/people/results

  SVM file for body segmentation (Ground Plane People Detector):
  https://github.com/PointCloudLibrary/pcl/tree/master/people/data

  Example of execution (active segmentation, no tracking, using live data):

  ./pcl_people_app -numTrees 3 -tree0  ../tree_files/tree_20.txt -tree1 ../tree_files//tree_20_1.txt -tree2 ../tree_files/tree_20_2.txt  -segment_people ../svm_path/trainedLinearSVMForPeopleDetectionWithHOG.yaml

  Example of execution (active segmentation and tracking, using recorded PCLZF data)

  ./pcl_people_app -numTrees 3 -tree0  ../tree_files/tree_20.txt -tree1 ../tree_files//tree_20_1.txt -tree2 ../tree_files/tree_20_2.txt  -segment_people ../svm_path/trainedLinearSVMForPeopleDetectionWithHOG.yaml -lzf /path_to_lzf_files/ -lzf_fps 10 -tracking 1


  Take a look at the video demonstrating the modifications of the skeleton tracker (sorry for the format:) ): 
  http://youtu.be/GhCrK3zjre0


.. blogpost::
  :title: Further improvements of the skeleton detection, data collection and activity recognition framework
  :author: Alina Roitberg
  :date: 08-12-2014

  **Joint tracking**
  
  Although the body segmentation has solved the problems with full body visibility the resulting joint positions were still not sufficient. Besides the Gaussian noise, big "jumps" occured if suddenly a blob with a compeletly wrong position was labelled as the corresponding body part.

  To solve this, I implemented a simple tracking function (Alpha-Beta-Filter) on top of the joint calculation. The filter estimates the new position based on the predicted (calculated from the previous position and velocity) and measured position. The weight of the measured position is given by the parameter alpha, while beta shows the weight of the velocity update.

  Activation of tracking, alpha and beta parameters can be set in the PeopleApp.

  The parameters should be chosen wisely, current default values worked good for me, but the also depend on the frame rate, which is not easily predictable, as it depends on the GPU.

  **Other changes to the skeleton calculation**


  Integration of tracking improved the results, however, problems still occured if the voxel labelling failed. Unfortunately, this happed a lot with the hands if they were very close to the rest of the body (and could be hardly distinguished). 

  That’s why I added some correction based on the position of the elbows and the forearms. I estimate the expected position based on the positions of those body parts and if the measured position is too far away, the predicted result is used.

  Besides I discovered the global variable AREA_THRES2 (in people_detector.cpp) and increasing it from 100 to 200 improved the labelling. Increasing it reduces the noise, while making the probability that small blobs (like hands) will be missed higher.
  

  **Data collection**
  
  As I have mentioned in my previous post, I wanted to collect the skeleton information from the recorded Kinect data in order to use the full framerate.
  To do so, I extended the people app with the option of using recorded PCLZF videos instead of the live Kinect data. 

  The positions of skeleton joints can be stored in a txt file with following flag: -w <bool val>

  The data format is defined as following:
  SeqNumber Timestamp
  Pos1_x Pos1_y Pos1_z
  Pos2_x Pos2_y Pos2_z
  
  I recorded the Kinect data of ten people performing the activities (defined in my previous post) multiple times.
  Afterwards I segmented the PCLZF-files with each directory containing the one execution of an action and run the people detector on each of them (I had a lot of fun with it:)).
  As copying the files and running the skeleton detector really took me "forever" I had to stick with a part of data and focus on the activity detection framework. 
  The current framework contains segmented training data from 6 participants (269 recordings all together) and I will of course add the rest of the data, but I doubt that it would happen before the end of GSOC.
    
  Another file format was defined for the training data, which includes all recorded skeletons, annotation of the action and UserID.

  **Classification Framework**

  I implemented a simple K-Nearest-Neighbours algorithm to classify a skeleton sequence.
  As all sequences have different lengths and the number of frames is high, one should find a way to calculate a good feature vector.
  In the implemented framework, classification happens in following way:
    
    * Read skeleton data is divided into a fixed number of segments
    * For each segment the joint positions are calculated as the mean value over all frames in the segment
    * For each resulting skeleton (one per segment) further feature selection takes place: the decision which joint positions and angles to use
    * All joint positions are calculated in relation to the Neck position (and not the sensor)
    * The height of the person is estimated (difference between the head and the feet in the first or last segment) and the positions are normalized accordingly
    * Mean and variance of each dimension of the feature vector is calculated over the whole training set and the data is normalized accordingly (this is important in KNN, to let all dimensions contribute in the same way).
    * The feature vectors can be classified with K-NN (1-NN by default).

  The application can classify a new sample as well as perform Leave-One-Out cross validation on the training set and print out the confusion matrix.
  
  Currently I am getting following results:

  .. image:: img/confusion.png
   :height: 200px


  The recognition rate of 34% might not sound that great. However one should consider the high number of actions and the fact that the skeleton data has some problems with joint positions, especially with the hands, which makes the recognition very challenging. The people detector also has some severe problems with unusual poses.
Further data processing, feature selection and more complex classification methods might improve the performance significantly in the future.

 

 
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
  
  

