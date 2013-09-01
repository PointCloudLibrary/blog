My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: gioia

.. blogpost::
	:title: Reading days 
	:author: gioia
	:date: 5-14-2012

	My primary activities in the last weeks were setting up the working environment, doing some basic testing on the new `3D object recognition tutorial <http://	pointclouds.org/documentation/tutorials/correspondence_grouping.php>`_, and fixing feature #644 while solving some related bugs. Beyond these tasks, I also have been engaged in some important readings. I list them below, since they could be useful to someone who approaches to the object recognition field:

		* Salti, S.; Tombari, F.; Stefano, L.D.; , **"A Performance Evaluation of 3D Keypoint Detectors,"** 3D Imaging, Modeling, Processing, Visualization and 		Transmission (3DIMPVT), 2011 International Conference on , vol., no., pp.236-243, 16-19 May 2011
		* Yu Zhong; , **"Intrinsic shape signatures: A shape descriptor for 3D object recognition,"** Computer Vision Workshops (ICCV Workshops), 2009 IEEE 12th 			International Conference on , vol., no., pp.689-696, Sept. 27 2009-Oct. 4 2009
		* Tombari, F.; Di Stefano, L.; , **"Object Recognition in 3D Scenes with Occlusions and Clutter by Hough Voting,"** Image and Video Technology (PSIVT), 2010 			Fourth Pacific-Rim Symposium on , vol., no., pp.349-355, 14-17 Nov. 2010

	I want to do a final remarkable consideration, it is useful to those users who are approaching to the 3D object recogntition tutorial. To test the tutorial with a more extended dataset, you could refer to `http://vision.deis.unibo.it/SHOT/ <http://vision.deis.unibo.it/SHOT/>`_ (scroll down till the dataset section is reached).


.. blogpost::
	:title: Testing functionalities for the existing keypoint detectors in PCL 
	:author: gioia
	:date: 5-22-2012

	I am currently developing a test program in order to visually compare the keypoints extracted by the existing detectors in PCL. The program looks like a tutorial and it is possible to set up the characteristic parameters of each detectors at the time the program executes. I chose to introduce this possibility since I have noted that in most of the cases the detector parameters are dependent from the features and the structure of the input point cloud.

	In what follows, I show the keypoints extracted by the:
	
	* **NARF keypoint detector**

			.. image:: images/2nd_post/narf_screenshot041.png
    				:width: 800px
				:height: 500px
    				:align: center

	* **SIFT keypoint detector**

			.. image:: images/2nd_post/sift_screenshot034.png
    				:width: 800px
				:height: 500px
    				:align: center
		
	* keypoint detector based on the **uniform sampling** technique

			.. image:: images/2nd_post/uniform_screenshot056.png
    				:width: 800px
				:height: 500px
    				:align: center
	
	
	while the examples related to the harris 2D, 3D and 6D detectors are still under development.


.. blogpost::
	:title: A framework for keypoint detection evaluation
	:author: gioia
	:date: 6-16-2012

	I spent the last two weeks working on a framework for keypoint detection evaluation in PCL. First, I've developed simple evaluators, one for each detector under consideration. The evaluators take as input a pair of files (model + scene) and they first set the right groundtruth rotation and translation. For each detector, keypoints are extracted from both the model and the scene cloud based on a set of typical parameters. Some parameters are kept fixed for each detector in order to insure the  fairness of the tests. In particular the detectors are tested on various scales and the scale is considered as a multiple of the model resolution. Finally, the results are displayed by means of the PCLVisualizer: both keypoints, repeatable keypoints and keypoints that are not occluded in the scene are shown. I show here the visual results achieved by the NARF keypoint detector on data acquired by means of the Microsoft Kinect sensor:

	* model
			.. image:: images/3rd_post/model_screenshot0119.png
    				:width: 800px
				:height: 500px
    				:align: center
	* scene
			.. image:: images/3rd_post/scene_screenshot0115.png
    				:width: 800px
				:height: 500px
    				:align: center


	* Absolute repeatability: 7
	* Relative repeatability: 0.146 

	The objective of this week was to extend the simple framework evaluators to run on entire datasets. The values computed refers to the average absolute and relative repeatability. Recently, a new idea has come: the repeatability measures will be accompanied with an analysis about the detectors' time performances. Time performances will be computed based on the extraction time of the keypoints. In the weekend, I plan to define some final parameters of the detectors and to add the time performance analysis to the frameworks. Finally, my roadmap is constantly updated with my recent advances.

	**Coming soon: tables and graphs about this work.**



.. blogpost::
	:title: Tests launched today!
	:author: gioia
	:date: 6-22-2012

	This week I've lost some time to debug my repeatability calculator. Since I had some misunderstanding about how the keypoint repeatability shall be computed, I've decided to devote a little section of my blog to this aim. You can find how to compute the absolute and relative repeatability :ref:`here <gioia_repeatability>`. After having solved such issues, I spent some time in defining more suitable parameters for the detectors under consideration. To this aim, I used the frameworks for simple keypoint detection evaluation since they allow to visualize results and so to immediately detect errors. I have also changed something in the visualization. 

	Now, the display of the model shows:

        * all the keypoints extracted from the model by applying a specific detector (green color) 
        * all the model keypoints that are not occluded in the scene (red color)
        * all the model keypoints that are **repeatable** (blue color).

	While, the display of the scene shows:

        * all the keypoints extracted from the scene by applying a specific detector (fuchsia color)
        * all the scene keypoints that are not occluded in the model (green color).

	Tests have been executed on synthetic data. Here, I post two screenshots related to the Harris3D detectors: 

	* model view
			.. image:: images/4th_post/model_screenshot027.png
    				:width: 800px
				:height: 500px
    				:align: center

	* scene view
			.. image:: images/4th_post/scene_screenshot046.png
    				:width: 800px
				:height: 500px
    				:align: center


	Finally, today I've launched the first evaluation tests, so in the following days I will post the available results.


.. blogpost::
	:title: First results
	:author: gioia
	:date: 6-26-2012

	Here, I show the first results coming from testing all the detectors but NARF. NARF has been under investigation a little bit more, but thanks to Bastian Steder the issues I have encountered should be solved. 

	The absolute repeatability graph:

			.. figure:: images/5th_post/absolute_repeatability_all_but_narf.png
    				:scale: 20 %
    				:align: center

	The relative repeatability graph:

			.. figure:: images/5th_post/relative_repeatability_all_but_narf.png
    				:scale: 20 %
    				:align: center

	The time performances graph (related to a 6mr scale):

			.. figure:: images/5th_post/time_performances_6mr_all_but_narf.png
    				:scale: 20 %
    				:align: center

.. blogpost::
	:title: Final repeatability results for the dataset based on kinect data
	:author: gioia
	:date: 6-29-2012

	I'm currently visiting the LAAS-CNRS in Toulouse, in order to see if it could be a good place to me to accomplish PhD studies. While doing so, I've finished to test the detectors on the kinect-based dataset. Again, I want to say thank you to Bastian Steder for helping me on dealing with the NARF detector. Results are graphically shown below:

	.. figure:: images/6th_post/absolute_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/6th_post/relative_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

.. blogpost::
	:title: Final performances results for the dataset based on kinect data
	:author: gioia
	:date: 6-30-2012

	In what follows, the time performances related to all the considered detectors have been collected and graphically visualized to foster a better understanding. In our framework, time performances refer to the keypoint extraction time of each detector and different scales have been taken into account: 6, 9, 12, 15 * ``scene_resolution`` are considered. Results are given below.

	.. figure:: images/7th_post/time_performances_6mr_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/7th_post/time_performances_9mr_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/7th_post/time_performances_12mr_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/7th_post/time_performances_15mr_all.png
   		 :scale: 20 %
    	         :align: center


.. blogpost::
	:title: Still looking for a PhD position
	:author: gioia
	:date: 7-03-2012

	Finally, I decided to decline the offer by the LAAS-CNRS and I'm still looking for a PhD position. Any suggestion about it?

.. blogpost::
	:title: Hard working days
	:author: gioia
	:date: 7-03-2012

	While being in Toulouse I had to sacrifice some of the time devoted to PCL, so this week I'm going to work hard in order to recover that time. Currently, I am testing the detectors on a synthetic dataset (the well-know Stanford dataset). Unfortunately, since this dataset does not contain the RGB information the only detectors under testing are: Harris 3D, NARF and uniform sampling. Indeed, those three detectors are characterized by having a shape-based saliency measure. As for the tests executed on the Kinect-based dataset, Harris 3D is evaluated with regard to all the possible different response methods. While the tests are executing, today I've decided to run up with my roadmap and I started to take some knowledge about the 3DGSS detector. In particular, the reference paper I've read is:

	* John Novatnack and Ko Nishino, "`Scale-Dependent 3D Geometric Features <https://www.cs.drexel.edu/~kon/3DGSS/>`_," in Proc. of IEEE Eleventh International Conference on Computer Vision ICCV'07, Oct., 2007.

	**Coming soon: the final evaluation results on the synthetic dataset.**

.. blogpost::
	:title: Final repeatability results for the Stanford dataset 2
	:author: gioia
	:date: 7-04-2012

	I just got the results for the execution of tests on the Stanford dataset. All the tests (both on the Kinect and the Stanford dataset) have been executed on a 2nd generation Intel® Core™ i5 processor with a speed equal to 2.4 GHz (3 GHz if TurboBoost technology is enabled). The results regarding the repeatability measure confirm the superiority of the NARF detector on the others. The repeatability results are graphically shown below:

	.. figure:: images/10th_post/absolute_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/10th_post/relative_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

.. blogpost::
	:title: Final performances results for the Stanford dataset 2
	:author: gioia
	:date: 7-04-2012

	In what follows, the time performances related to all the considered detectors have been collected with regards to the Stanford dataset 2 and graphically visualized to foster a better understanding. In our framework, time performances refer to the keypoint extraction time of each detector and different scales have been taken into account: 6, 9, 12, 15 * ``scene_resolution`` are considered. Results are given below.

	.. figure:: images/11th_post/time_performances_6mr_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/11th_post/time_performances_9mr_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/11th_post/time_performances_12mr_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/11th_post/time_performances_15mr_all.png
   		 :scale: 20 %
    	         :align: center

.. blogpost::
	:title: Working in parallel on new detectors: GSS and ISS
	:author: gioia
	:date: 7-11-2012

	Last week I started working on GSS but I stopped this week since I need some feedback from Alex and he may be on vacation now. In order not to lose much time, I have decided to switch to the implementation of ISS. After reading the related paper and after taking some knowledge on the code already implemented, I have defined the input and output parameters of the detector and the operations that have to be done in order to compute the keypoints. Then, I have prepared a skeleton class and now I'm currently filling and extending it with the implementation of new methods.

	What were my other activities during the past and the current week?
	Bug solving and user support.

.. blogpost::
	:title: Working hard on ISS refinement
	:author: gioia
	:date: 7-21-2012

	Some time has passed since my last post. My recent and current activities regard mainly the ISS detector. First of all, I completed the detector so that it has a basic functionality (e.g. it works ! ). In order to be sure that it really works, I developed a simple evaluation framework at the same time. This framework looks like the basic frameworks I developed at the beginning of the GSoC code sprint and it allows the user both to compute the repeatability of the detector based on a pair of files (model + scene) and to visualize the resulting keypoints. My current activities regards the refinement of the ISS detector, and now I'm particularly focusing on time performances. After a brief analysis about the time performances of the detector, I refined it by using low-cost data structures. Now, I'm working on the introduction of the openMP directives in order to further speed-up the computation when 2 or more cores are avalilable. The detector will be available in the trunk when it will be fully completed and optimized. 

	**Stay tuned with my roadmap**: it always shows my current activities even if I'm not posting so much.

.. blogpost::
	:title: ISS is available on trunk now!
	:author: gioia
	:date: 7-28-2012

	Now, ISS is available on the trunk and it is properly documented. In the section :ref:`how to use the ISS 3D keypoint detector <gioia_iss>` of this blog I will post some code snippets useful to the user who wants to exploit the ISS detector. Currently, the ISS detector is under testing. It will be tested for different configurations: 

	1. using 1 thread and disabling the boundary estimation.
	2. using 1 thread and enabling the boundary estimation.
	3. using 1, 2, 3 and 4 threads (4 is the maximum number of threads allowed in my system) and chosing the best configuration among that described previously in 1. and 2. .

	The tests will show both the repeatability results and the time performances of the detector. The results related to the configurations 1. and 2. can be compared with the results already obtained for the other detectors tested at the beginning of my GSoC work.

	**Coming soon**: the test results for the ISS detector.

.. blogpost::
	:title: First pictures from the ISS detector
	:author: gioia
	:date: 7-28-2012

	In what follows I'll show some snapshots related to the behaviour of the ISS detector. They have been obtained by not setting the border radius.

	**Shapshots from the Kinect dataset.**
	
	* Model:

			.. figure:: images/14th_post/model_screenshot041.png
    				:scale: 100 %
    				:align: center

	* Scene: 

			.. figure:: images/14th_post/scene_screenshot095.png
    				:scale: 1 %
    				:align: center

	* Results: 

	  * Absolute repeatability: 8
	  * Relative repeatability: 0.195122


	**Shapshots from the Stanford 1 dataset.**
	
	* Model:

			.. figure:: images/14th_post/model_screenshot058.png
    				:scale: 1 %
    				:align: center

	* Scene: 

			.. figure:: images/14th_post/scene_screenshot070.png
    				:scale: 1 %
    				:align: center

	* Results: 

	  * Absolute repeatability: 413
	  * Relative repeatability: 0.769088

.. blogpost::
	:title: News on tests
	:author: gioia
	:date: 8-1-2012

	While developing the evaluator for the ISS 3D detector, I realized that I set the harris 3D and 6D normal estimation radius to::
	
	  multiplier * cloud_resolution
	
	where multiplier is set to 6, 9, 12 and 15 cloud_resolution each time. Instead of this setting, I should have set the normal estimation radius to::

	  4 * cloud_resolution

	in order to obtain fair tests. The previous tests are valid but the reader should take  into account this consideration. While testing the ISS 3D detector I re-run the tests to obtain the desidered fairness. I have also decided to collect this final tests in a specific blog page, so as the user can immediately reach the results without looking for it in all my posts. I have just completed the evaluation on the Kinect dataset and I will post it soon both on the blog and on :ref:`Detectors evaluation: repeatability and time performances <gioia_tests>`.

.. blogpost::
	:title: Fair tests on the Kinect dataset
	:author: gioia
	:date: 8-2-2012

	In what follows I'll show the main results of the tests on the kinect dataset. The complete results are given in :ref:`Detectors evaluation: repeatability and time performances <gioia_tests>`. The fairness of the comparison is ensured by fixing properly some common parameters among all the detectors. NARF and ISS are clearly the two best PCL detectors among the ones tested.

	I think it is required a brief explanation of the graphs legend. First the harris 3D detector is tested with all its possible response methods, so the abbreviations *HA*, *NO*, *LO*, *TO*, and *CU* respectively refer to the response methods: *HARRIS*, *NOBLE*, *LOWE*, *TOMASI*, *CURVATURE*. The abbreviation *ISS - WBE* refers to the execution of the ISS 3D detector without the border extraction, while *ISS - BE* refers to the execution of the detector with the extraction of boundary points. Finally, the abbreviations *1 th*, *2 th*, *3 th* and *4 th*  stand for *1*, *2*, *3* and *4* threads and they are related to the OpenMP optimization of the code.

	.. figure:: images/detectors_evaluation/kinect/absolute_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/relative_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/time_performances_all_6cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/iss_time_performances_6cr.png
   		 :scale: 20 %
    	         :align: center

.. blogpost::
	:title: Fair tests on the Stanford dataset 2
	:author: gioia
	:date: 8-4-2012

	In what follows I'll show the main results of the tests on the Stanford dataset. The complete results will be given in :ref:`Detectors evaluation: repeatability and time performances <gioia_tests>`. The fairness of the comparison is ensured by fixing properly some common parameters among all the detectors. ISS has clearly the best relative repeatability among all the PCL detectors that have been under testing. With regards to the legend,  the same considerations made in the previous blog post apply this time. 

	.. figure:: images/detectors_evaluation/stanford/absolute_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/relative_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/time_performances_all_6cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/iss_time_performances_6cr.png
   		 :scale: 20 %
    	         :align: center

.. blogpost::
	:title: Fair tests on the Stanford dataset 2
	:author: gioia
	:date: 8-5-2012

	I've finally completed the section :ref:`Detectors evaluation: repeatability and time performances <gioia_tests>` and it is now available for consulting.

.. blogpost::
	:title: A new tool for PNG to PCD conversions is now available on the trunk!
	:author: gioia
	:date: 8-10-2012

	I've developed a simple utility that enables the user to convert a PNG input file into a PCD output file. The converter takes as input both the name of the input PNG file and the name of the PCD output file. It finally performs the conversion of the PNG file into the PCD file by creating a::

	  pcl::PointCloud<pcl::RGB>> 

	point cloud. Now, the PNG2PCD converter is available on the trunk version of PCL under the *tools* directory.

.. blogpost::
	:title: A new point type to handle monochrome images is now available on the trunk!
	:author: gioia
	:date: 8-18-2012

	I've developed a new point type to handle monochrome images in the most effective way. It contains only one field, named *intensity*, of type *uint8_t*. Consequently, I've updated the pcl::ImageViewer so that it is able to manage the point clouds related to the new point type, named *pcl::Intensity*. Finally, I've changed the PNG2PCD converter by adding more functionalities: now the user can choose if the written cloud should be based on pcl::RGB or on pcl::Intensity. For more information, please see the documentation related to each single class or file.  

