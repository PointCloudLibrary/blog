Detectors evaluation: repeatability and time performances
=========================================================
.. _gioia_tests:

This section of my blog is devoted to collect all the graphical results obtained during the testing of the PCL detectors. 

Preliminary information
-----------------------

Datasets
********

The tests have been executed onto two different dataset: 

  1. Dataset 5 - Kinect
  2. Dataset 2 - Stanford

Additional information about the datasets are available at `http://vision.deis.unibo.it/SHOT/ <http://vision.deis.unibo.it/SHOT/>`_ (scroll down till the dataset section is reached). The website allows you also to download the datasets and to use them for your own purposes.

Fairness
********

The fairness of the tests has been ensured by setting common parameters among all the tested detectors. Specifically, in the test case of each detector:

  * The scale has been fixed respectively to 6, 9, 12, 15 x cloud_resolution.
  * The radius used during the application of the non maxima suppression algorithm has been set to 4 x cloud_resolution.
  * The radius used to estimate normals has been set to 4 * cloud_resolution.

All the other parameters have been set accordingly to the specific functionality of the detector at issue.

How to read the legend of the graphs
------------------------------------
Before showing the graphs, a brief explanation of the abbreviations used in the legends is needed. 

+------------------------+-------------------------------------------------------------------------------------+
| **Abbreviations**      | **Meaning**                                                                         |
+========================+=====================================================================================+
| *HARRIS 3D - HA*       | the Harris 3D detector has been executed with the HARRIS response method enabled    | 
+------------------------+-------------------------------------------------------------------------------------+
| *HARRIS 3D - NO*       | the Harris 3D detector has been executed with the NOBLE response method enabled     | 
+------------------------+-------------------------------------------------------------------------------------+
| *HARRIS 3D - LO*       | the Harris 3D detector has been executed with the LOWE response method enabled      | 
+------------------------+-------------------------------------------------------------------------------------+
| *HARRIS 3D - TO*       | the Harris 3D detector has been executed with the TOMASI response method enabled    | 
+------------------------+-------------------------------------------------------------------------------------+
| *HARRIS 3D - CU*       | the Harris 3D detector has been executed with the CURVATURE response method enabled | 
+------------------------+-------------------------------------------------------------------------------------+
| *HARRIS 6D*            | the Harris 6D detector has been executed                                            | 
+------------------------+-------------------------------------------------------------------------------------+
| *NARF*                 | the NARF detector has been executed                                                 | 
+------------------------+-------------------------------------------------------------------------------------+
| *SIFT*                 | the SIFT detector has been executed                                                 | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - WBE 1th*     | the ISS 3D detector has been executed Without performing the Border Estimation and  |
|                        | and using 1 thread (OpenMP optimization)                                            | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - WBE 2th*     | the ISS 3D detector has been executed Without performing the Border Estimation and  |
|                        | and using 2 threads (OpenMP optimization)                                           | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - WBE 3th*     | the ISS 3D detector has been executed Without performing the Border Estimation and  |
|                        | and using 3 threads (OpenMP optimization)                                           | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - WBE 4th*     | the ISS 3D detector has been executed Without performing the Border Estimation and  |
|                        | and using 4 threads (OpenMP optimization)                                           | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - BE 1th*      | the ISS 3D detector has been executed by performing the Border Estimation and by    |
|                        | using 1 thread (OpenMP optimization)                                                | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - BE 2th*      | the ISS 3D detector has been executed by performing the Border Estimation and by    |
|                        | using 2 threads (OpenMP optimization)                                               | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - BE 3th*      | the ISS 3D detector has been executed by performing the Border Estimation and by    |
|                        | using 3 threads (OpenMP optimization)                                               | 
+------------------------+-------------------------------------------------------------------------------------+
| *ISS 3D - BE 4th*      | the ISS 3D detector has been executed by performing the Border Estimation and by    |
|                        | using 4 threads (OpenMP optimization)                                               | 
+------------------------+-------------------------------------------------------------------------------------+

Fair tests on the Kinect dataset
--------------------------------

Here, the results related to the tests executed on the Kinect dataset are shown.

Repeatability results
*********************

	.. figure:: images/detectors_evaluation/kinect/absolute_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/relative_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

Time performances (all the detectors - 1 thread)
************************************************

	.. figure:: images/detectors_evaluation/kinect/time_performances_all_6cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/time_performances_all_9cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/time_performances_all_12cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/time_performances_all_15cr.png
   		 :scale: 20 %
    	         :align: center

Time performances (ISS 3D detectors - 1, 2, 3, 4 threads)
*********************************************************

	.. figure:: images/detectors_evaluation/kinect/iss_time_performances_6cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/iss_time_performances_9cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/iss_time_performances_12cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/kinect/iss_time_performances_15cr.png
   		 :scale: 20 %
    	         :align: center

Fair tests on the Stanford 2 dataset
------------------------------------

Here, the results related to the tests executed on the Stanford 2 dataset are shown.

Repeatability results
*********************

	.. figure:: images/detectors_evaluation/stanford/absolute_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/relative_repeatability_all.png
   		 :scale: 20 %
    	         :align: center

Time performances (all the detectors - 1 thread)
************************************************

	.. figure:: images/detectors_evaluation/stanford/time_performances_all_6cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/time_performances_all_9cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/time_performances_all_12cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/time_performances_all_15cr.png
   		 :scale: 20 %
    	         :align: center

Time performances (ISS 3D detectors - 1, 2, 3, 4 threads)
*********************************************************

	.. figure:: images/detectors_evaluation/stanford/iss_time_performances_6cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/iss_time_performances_9cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/iss_time_performances_12cr.png
   		 :scale: 20 %
    	         :align: center

	.. figure:: images/detectors_evaluation/stanford/iss_time_performances_15cr.png
   		 :scale: 20 %
    	         :align: center


