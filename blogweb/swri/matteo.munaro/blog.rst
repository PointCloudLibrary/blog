.. blogpost::
   :title: SwRI/NIST Code Sprint :  First post
   :author: matteo.munaro
   :date: 04-25-2013
   
	Hi! This is the first post for the SwRI/NIST Code Sprint.

	This code sprint deals with developing algorithms for human detection and tracking, out of 2D camera imagery fused with 3D point cloud data.
	The Southwest Research Institute (SwRI) just contributed a novel algorithm for human detection and tracking to ROS-Industrial's repository on GitHub.

	At `this <https://github.com/ros-industrial/human_tracker>`_ link, you can find the source code of the *human_tracker* project.
	For this code sprint, I am requested to improve accuracy and/or framerate of the existing code. All my updates will be inserted into the *develop* branch of the repository. 

	The *human_tracker* project aims at developing software able to track people while meeting a number of constraints which have been defined by NIST. These requirements can be summarized as follows:

	- track all people in the distance range 1-5m

	- tracking when the system is moving up to 3m/s and 1.5rad/sec

	- nominal update rate: 15+/-2 Hz (30Hz desired)

	- ID will persist for no less than 5 seconds

	- track people in the height range 153-190cm

	- track people up to 20% occluded


.. blogpost::
   :title: Overview of the human_tracker Code
   :author: matteo.munaro
   :date: 05-14-2013

	Today I will give an overview of the *human_tracker* code.

	The code is a collection of ROS packages which perform detection, tracking, visualization and labeling. The detection packages are five (*Consistency*, *HaarAda*, *HaarDispAda*, *HaarSvm* and *HogSvm*) and they implement different approaches which are then fused in a detection cascade for improving results. The tracking algorithm, instead, is implemented in a single node (*object_tracking*). The visualization package (*roi_viewer*) allows to plot people bounding boxes onto the rgb image. The labeling procedure is implemented in the *labeler* package and allows to annotate bounding boxes in the image for creating a ground truth.

	Together with the source code, the *human_tracker* repository also contains some pre-trained classifiers and a documentation folder.
	In order to run the code in your machine, you need to set an environment variable, *NIST_CLASSIFIERS*, which points to the folder containing the pre-trained classifiers.

	In the *System_Launch* package, there are some launch files which allow to perform tracking live from a Kinect/Xtion (*run_kinect_nodelet.launch*) or from a pre-recorded ROS bag (*run_from_bag_kinect.launch*).

	As the output of the tracking algorithm, the 3D positions and velocities of people are written to a topic (*/human_tracker_data*) and, optionally, to a CSV file.

	I report here some considerations which are useful when working with the *human_tracker* algorithms:

	- the camera should not be inclined more than 15° wrt the ground plane because, in the consistency node, this assumption is used to discard part of the image;

	- the tracking node automatically estimates the ground plane equation and exploits it for working on people positions in terms of ground plane coordinates;

	- a new thread is used for every track and the maximum number of threads/tracks is set to 5. Thus, it is important to use a machine with enough threads (6-7) for running the code at the maximum framerate.

	In the figure below, the output of all the detection nodes and of the tracking node is reported:

	.. image:: ../images/Default.png
	  :height: 300 pt
	  :align: center


.. blogpost::
   :title: Image Publisher Node Added
   :author: matteo.munaro
   :date: 05-23-2013

	In order to allow to perform detection and tracking on images contained in a folder, I added a new package (*imagePublisher*) which reads rgb and disparity images from a folder and publishes them to the standard rgb and disparity topics published by a Kinect/Xtion.
	This node also reads camera and projector parameters, such as baseline, optical center and focal length, from file (*cameraInfo.yml*) and publishes them to the */camera/depth_registered/camera_info* and */camera/projector/camera_info* topics.

	I also provided a launch file (*run_from_dir_nodelet.launch*) for performing tracking from folder which uses the *imagePublisher* node to create the input stream for the detection cascade.
	A publishing framerate (*replayRate*) can be chosen to produce streams at the desired rate and a delay (*delayStart*) can be introduced before the stream starts.


.. blogpost::
   :title: Accuracy and Framerate Evaluation
   :author: matteo.munaro
   :date: 06-28-2013

	In this post, I am describing the method I chose for evaluating people tracking algorithms.

	In order to compare the different approaches I will develop for this code sprint, I implemented some Matlab files which compute some quantitative indices from the tracking output written to CSV file.
	First, I made the tracking algorithm to output to file also people bounding boxes inside the image in order to compare them with the ground truth by exploiting the PASCAL rule usually exploited for evaluating object detectors (*The PASCAL Visual Object Classes (VOC) Challenge. Everingham, M. , Van Gool, L. , Williams, C. K. I. , Winn, J. and Zisserman, A. International Journal of Computer Vision (2010)*). 

	Then, I implemented functions for reading the CSV file with the tracking output, comparing tracking bounding boxes with ground truth bounding boxes and computing two indices:

	- False Rejection Rate (%): 100*miss/(TP + miss) = % miss

	- False Positives Per Frames: FP/frames.

	Given that tracking results can be considered good even if the computed bounding box is a bit off with respect to the person center, a threshold of 0.3 has been used in the PASCAL rule, instead of the standard 0.5 threshold.

	For evaluating the framerate, the *rostopic hz* command has been used for measuring the publishing rate of the tracking topic (*/human_tracker_data*). All the framerates reported in this blog have been computed with an Intel i7-3630QM processor at 2.4-3.4GHz (4 cores, 8 threads).


	The original code provided by SwRI produced the following results:

	- FRR: 18.24%

	- FPPF: 0.58

	- framerate: 23.8 fps


.. blogpost::
   :title: Improvements to PCL's *people* Module and SwRI's *Consistency* Node
   :author: matteo.munaro
   :date: 07-12-2013

	Since the last post, some updates have been provided both to PCL and ROS-Industrial repositories.

	For PCL, some work had to be done in order to release the new version of PCL: 1.7.
	This version has been directly integrated also in the forthcoming ROS Hydro.
	In particular, a new and optimized version of the HOG code has been provided. This new version exploits SSE2 operations to obtain a major speed-up, but a version of that code which does not exploit SSE has also been provided, in order to make it work on every machine. Also some Windows compilation bugs have been fixed for the new release.

	For what concerns the *human_tracker* code, the *Consistency* node has been updated. 
	I noticed that the *Consistency* node outputted too many ROIs which were highly overlapping and this was the cause of a major loss in the frame rate of the detection cascade. Thus, a procedure for removing overlapping ROIs which was used in the *HaarAda* node has been implemented also in the *Consistency* node (in the launch file this function can be activated/deactivated with the *RemoveOverlappingRois* parameter).
	As a result, the framerate increased of 8-20% (from 23.8 to 28.2 fps) while maintaining the same accuracy.

	In the figure below, the output of all the detection nodes and of the tracking node is reported while performing the removal of the ROIs overlapping more than 80% with another ROI in the *Consistency* node:

	.. image:: ../images/CROR.png
	  :height: 300 pt
	  :align: center


.. blogpost::
   :title: Pointcloud Publisher Node Added
   :author: matteo.munaro
   :date: 07-16-2013

	A node for creating XYZRGB pointclouds from RGB and disparity images has been contributed to the *human_tracker* repository. This node is called *pointcloudPublisher* and performs the following steps:

	- reading images and camera parameters from folder

	- pointcloud creation

	- depth to rgb registration

	- pointcloud publishing to */camera/depth_registered/points* topic, which is the standard pointcloud topic for OpenNI devices.

	This node allows to apply PCL's people detector, which works on pointclouds, even if only RGB and disparity images are available. Later on, I will contribute a ROS node which exploits this detector.


.. blogpost::
   :title: Major Improvements to Accuracy and Framerate
   :author: matteo.munaro
   :date: 07-24-2013

	For reducing the latency and increasing the framerate of the detection cascade, I tried to reduce the detection nodelets to only two:

	- *Consistency* nodelet

	- *HogSvm* nodelet

	The framerate considerably slowed down (from 28.2 to 9.8 fps) because the *HogSvm* nodelet was too slow to process all the rois outputted by the *Consistency* nodelet. 

	For this reason, I implemented a new nodelet, called *HogSvmPCL*, which exploits the HOG descriptor and the pre-trained Support Vector Machine in PCL 1.7 for detecting whole persons in RGB image patches.
	This new node is much faster than *HogSvm* while being very accurate in classification.
	By using the reduced cascade with *HogSvmPCL* in place of *HogSvm* (*Consistency* + *HogSvmPCL*), the tracking framerate tripled with respect to the *Consistency* + *HogSvm* approach (from 9.8 to 30 fps).
	The FRR decreased of 4% (from 18.24% to 22.48%) wrt the default code from SwRI, while the FPPF improved of 35% (from 0.58 to 0.38).

	I also trained a SVM targeted to recognize upper bodies in order to be more robust when the lower part of a person is occluded or out of the image. 
	This approach led to a further reduction of false positives of about 15%, while maintaining the same false rejection rate and framerate.
	From the launch files, the wholebody or halfbody classifier can be selected by setting the *mode* parameter and specifying the path to the classifier file with *classifier_file*.

	In order to further improve accuracy, I exploited also disparity information adding to the detection cascade the *HaarDispAda* nodelet which uses Haar features on the disparity and Adaboost as a classifier. With respect to the detection cascade only composed by the *Consistency* and the *HogSvmPCL* nodelets, the accuracy considerably improved. In particular, the FRR decreased of 0.5% and the FPPF decreased of 55%.

	The nodes of this cascade (*Consistency* + *HogSvmPCL* + *HaarDispAda* + *ObjectTracking*) can be launched with *run_kinect_nodelet_CodeSprint.launch*.


.. blogpost::
   :title: Detection Confidence Visualization
   :author: matteo.munaro
   :date: 07-26-2013

	In order to better analyze detection results with the *roiViewer* package, I added a *confidence* field to the *RoiRect* message. It could be used to store the score computed by a detection node for every Roi. 

	I also modified the *roiViewer* so that it could display that confidence if the *show_confidence* parameter is set to true.


.. blogpost::
   :title: ROS node with PCL's People Detector
   :author: matteo.munaro
   :date: 08-07-2013

	As the final step of the code sprint, I created a ROS node which performs people detection with the ground based people detector present in PCL 1.7 (`GroundBasedPeopleDetectionApp <http://docs.pointclouds.org/trunk/classpcl_1_1people_1_1_ground_based_people_detection_app.html>`_). 

	Instead of rgb and disparity images, this node takes as input a XYZRGB pointcloud, performs people detection and outputs a message containing the detected rois, as the other detection nodes of the *human_tracker* project. This node is called *ground_based_people_detector* (GPD) and it is used in *run_kinect_CodeSprint_bis.launch* of the *System_Launch* package. In this launch file, the detection cascade is composed of the *GPD* and the *HaarDispAda* nodes. 
	The GPD node produces very good detections which are persistent in time and well centered on people, while the *HaarDispAda* node removes some false positives.
	The framerate is of about 25 fps, thus slightly lower than the approach in *run_kinect_nodelet_CodeSprint.launch*.

	In the figure below, an example of the output of the nodes launched by *run_kinect_CodeSprint_bis.launch* is reported:

	.. image:: ../images/GPD.png
	  :height: 150 pt
	  :align: center

.. blogpost::
   :title: Final Results
   :author: matteo.munaro
   :date: 08-08-2013

	I report here below a summary of the tracking framerates for the main approaches I tested (measured on an Intel i7-3630QM processor at 2.4-3.4GHz with 4 cores and 8 threads). These framerates have been measured when publishing input images at 50 fps.

	- original approach in *run_from_kinect_nodelet.launch*: 23.8 fps

	- approach in *run_kinect_nodelet_CodeSprint.launch* (with *HogSvmPCL* node): 33 fps

	- approach in *run_kinect_CodeSprint_bis.launch* (with PCL's people detector): 25 fps.

	In the figure below, I report *Detection Error Trade-off* (DET) curves which compare the main tracking approaches contributed to the *human_tracker* project in terms of *False Positives Per Frame* (x axis) and *False Rejection Rate* (y axis). They have been obtained varying the minimum confidence parameter for the detector using HOG+SVM.
	The best working point for these curves is located at the bottom-left corner (with FRR = 0% and FPPF = 0). For visualization purposes, the curves are reported in logarithmic scale.

	.. image:: ../images/DET_comparison_blog.png
	  :height: 300 pt
	  :align: center

	It can be noticed how the approaches developed during this Code Sprint (red and green curves) obtain considerably less FPPF with respect to the original code (blue curve).
	Morover, they are also shown to be faster than the original approach.






