My status updates
=================

.. blogpost::
  :title: Milestone 2, Part 1: Introducing the Evaluation Dataset
  :author: adambr
  :date: 10-04-2013

  In this entry, I'll go over the evaluation dataset, the reasons I decided to create my own as opposed to using existing ones, and the needed criteria to evaluate multi-descriptor performance.   

  The first requirement is a real-world challenging dataset, to help quantify the limiting factors of each given descriptor as a function of the associated sensor.  The result is a ground truth confidence interval of each descriptor's performance.  The dataset in this case contains (2) subsets,  subset (A)  with no scene difference, and subset (B) differs only in photometric properties and sensor noise.

  The second requirement is a dataset where a percentage of any given scene is invalid or not suitable for a subset  of the descriptors in the multi-descriptor set. This requirement is expressed in subset (C) which contains a large percentage of out of range depth data.

  The third requirement is a minimal dataset for efficient and tractable computational performance, as expressed in subsets A, B, and C.

  To evaluate each descriptor's performance, pose estimation, as expressed by a single-angle drift around one axis, vs a known ground truth is used as a comparative measurement unit.  Additionally, the inlier rate is considered in evaluating the quality of correspondence selection.

  .. image:: images/dataset.png
    :width: 600 px

.. blogpost::
  :title: Correspondence Rejection: A Quick Reference Guide
  :author: adambr
  :date: 10-04-2013

  Correspondence rejection classes implement methods that help eliminate correspondences based on specific criteria such as distance, median distance, normal similarity measure or RanSac to name a few. Couple of additional filters I've experimented with include a uniqueness measure, and Lowe's ratio measure as in "Distinctive image features from scale invariant keypoints", D.G. Lowe, 2004. I've also explored the tradeoffs in implementing the filters within CorresondenceEstimation itself, or as external CorrespondenceRejection classes. The former is computationally more efficient if the rejection process is done in one pass, while the latter allows for scene-specific squential filter banks.

  Follows is a quick reference guide of the available correspondence rejection classes with remarks extracted from the source code.

  .. image:: images/QG_CorrRej.png
    :width: 600 px
 

.. blogpost::
  :title: Correspondence Estimation: A Quick Reference Guide
  :author: adambr
  :date: 09-28-2013

  With my current work on optimizing correspondence estimation across the uv/xyz domains, it is worth providing a topology of the available correspondence estimation classes in PCL.  For a highlevel treatment of the registration API, please refere to the registration tutorial.

  Correspondence estimation attempts to match keypoints in a source cloud to keypoints in a target cloud, based on some similarity measure, feature descriptors in our case. Although applying scene relevant descriptor parameters and correspondence thresholds may reduce erronous matches, outliers persist with impact on pose estimation. This is due to the implied assumption that for each source keypoint, a corresponding target keypoint exists. The difficulty in estimating model or scene-specific descriptor parameters is another factor.

  Follows is a quick reference guide of the available correspondence estimation classes with remarks extracted from the source code. 

  .. image:: images/QG_CorrEst.png
    :width: 600 px
 
	   
.. blogpost::
  :title: Milestone 1: Metrics Framework for 3D descriptors
  :author: adambr
  :date: 29-07-2013
	
  As mentioned in the roadmap, one of the goals is to implement a framework that captures vital statistics of selected descriptors and correspondence types. These vital statistics would then be analyzed by one or more objective function(s) to enable scene based optimizations.

  The first milestone, a metrics framework for descriptor evaluation is now complete, and its output is in-line with the characteristics cited in Rublee et. al. ICCV 2011 paper, among other publications.

  Specifically, the framework computes the intended vital statistics including: 2-Way and multi-descriptor matching and inlier rates. The filter banks include L2-distance, L2-ratio, and uniqueness measure. A simulated ground truth is also implemented and is generated during runtime. The framework has been applied to local 3D descriptors (FPFH33, SHOT352, and SHOT1344) across a range of downsampling leaf-sizes (0.01-0.07) and across a range of in-plane (0-90 degrees) rotations. A sample of the results is illustrated in the bar graphs below, which reflect the various metrics, computed at a 30 degree simulated rotation and at 2 levels of downsampling: 0.01 for the top bar graph and 0.07 for the next one. In total, 1680 rates were generated for further analysis by the objective function(s). A link is included below to a sample extended output for other 3D descriptors.  Next step: to extend the framework to support 2D descriptors.
  
	
  .. image:: images/fpfh01deg30.png
    :width: 800 px

  .. image:: images/fpfh07deg30.png
    :width: 800 px

  The extended output for other 3D descriptors follows, [click to enlarge]:
	
  .. image:: images/shot01deg30.png
    :width: 800 px
		
  .. image:: images/shot07deg30.png
    :width: 800 px

  .. image:: images/cshot01deg30.png
    :width: 800 px

  .. image:: images/cshot07deg30.png
    :width: 800 px


My status updates
=================
.. blogpost::
  :title: Project - Multi-Descriptor Optimizations across the 2D/3D Domains
  :author: adambr
  :date:   06.03.2013

  The project has started. 


	
