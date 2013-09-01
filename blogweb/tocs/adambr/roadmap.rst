Roadmap
=======
.. _adambr_roadmap:

* Literature review to provide research level numbers and the required 
  parameter for selected key point detectors and feature descriptors.

	* Key point detector selection: SIFT, HARRIS, BRISK
	* Non-max suppression / redundant key point removal
	* 2D feature descriptors: SIFT, SURF, BRISK or BRIEF
	* 3D feature descriptors: FPFH, SHOT, C-SHOT
	
* Descriptor calibration/benchmarking:  Build a framework that is able to capture 
  vital statistics of the selected descriptor types:

	* 2-Way and multi-descriptor matching rates.
	* L2-distance, L2-ratio, and uniqueness of the top 2 kNN correspondences.
	* True positive/Inliers rate supported by a simulated ground truth.
	* RANSAC-based evaluation as a compliment to the simulated ground truth approach.
	* Background threading to be invoked periodically.

* Objective Function(s):

	* Obtain the best key point correspondences based on the
          multi-descriptor approach described above.
	* Select one or more descriptor type(s) that satisfies desired
          performance characteristics (precision, compute resources, execution time).

* Evaluate on a set of selected point clouds.

