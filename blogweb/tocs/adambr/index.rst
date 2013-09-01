Adam Barclay
==============

This is my personal page

:email: ws578632@gmail.com
:Project: Multi-Descriptor Optimizations across the 2D/3D domains
:Mentors: Radu B. Rusu, Joesph Djusgash


About me
--------

I am a student at the Technical University / Vienna under the supervision of Dr. Hannes Kaufmann.

My research interests include computer vision, machine learning, and multi-modal fusion. 

Problem Description
-------------------

The goal of this project is to investigate multi-descriptor types as an optimal alternative to a single descriptor type across the 2D and 3D domains.	A common scenario is environmental changes, as in trail following outdoors, or indoor navigation, where a single descriptor type is no longer effective due to lack/changes in texture or range data. A multi-descriptor approach is expected to yield better results in such scenarios by utilizing the best performing descriptors; where a single descriptor would simply fail.

This project consists of 2 main components.  The first component is a calibration/benchmarking step that captures scene properties and vital statistics of a set of feature descriptors.   The 2nd main component consists of one or more objective functions, that enable the auto-selection of feature descriptor types to be applied, based on the initial calibration/benchmarking step, and the desired outcome, such as precision, speed, or compute-resources.  The initial step could be repeated periodically in a background thread to re-calibrate the feature descriptors' scene dependent vital statistics.

Although some of the research precedes this code sprint, as it is part of my thesis and interests, the PCL TOCS has given us the opportunity to enhance this work through the guidance of the mentors, and to contribute our work to the wider open source community.


Roadmap
-------

* Literature review to provide research level numbers and the required 
  parameters for selected key point detectors and feature descriptors.

	* Key point detector selection: SIFT, HARRIS, BRISK
	* Non-max suppression / redundant key point removal
	* 2D feature descriptors: SIFT, SURF, BRISK or BRIEF
	* 3D feature descriptors: FPFH, SHOT, C-SHOT
	
* Descriptor calibration/benchmarking:  Build a framework that is able to capture 
  vital statistics of the selected descriptor types:

	* 2-Way and multi-descriptor matching rates.
	* L2-distance, L2-ratio, and uniqueness of the top 2 kNN correspondences.
	* True positive/Inliers rate based on a simulated ground truth.
	* RANSAC-based evaluation as a compliment to the simulated ground truth approach.
	* Background threading to be invoked periodically.

* Objective Function(s):

	* Obtain the best key point correspondences based on the
          multi-descriptor approach described above.
	* Select one or more descriptor type(s) that satisfies desired
          performance characteristics (precision, compute resources, execution time).

* Verification step: using a selected set of point clouds.



Recent status updates
---------------------

.. blogbody::
   :author: adambr
   :nr_posts: 35



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
   :hidden:

   status
   roadmap
