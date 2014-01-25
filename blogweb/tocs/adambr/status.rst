My status updates
=================

.. blogpost::
  :title: Final Report
  :author: adambr
  :date: 01-24-2014

  It is with pleasure to share the successful completion of this Toyota Code Sprint in this final blog post.  In this project, homography estimation based on multi-modal, multi-descriptor correspondence sets has been explored, and inspired the introduction of the multi-descriptor voting approach (MDv). The proposed MDv approach achieved a consistent accuracy in the 0.0X range, a level of consistency that is better than those based on single-type state of the art descriptors including SIFT. In the process, a framework for analyzing and evaluating single and multi-descriptor performance has been developed, and employed to validate the robustness of MDv, as compared with homography estimations based on a single descriptor type, as well as those based on RANSAC registration of best-K multi-descriptor correspondence sets.  The code and dataset for this project are hosted on https://github.com/mult-desc/md, with dependencies on both PCL 1.7 and OpenCV 2.4.6.

  Follows is an in-depth report detailing the project's accomplishments, as well as design and validation considerations:

  .. raw:: html

    Click <a href="https://drive.google.com/file/d/0ByU7lj1rOUWOWkplTmRqakN3c00/edit?usp=sharing">here</a> for a high resolution version of the report.

  .. raw:: html

    <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftocs%2Fadambr%2Ffiles%2FAdamBr_TOCS_FinalReport_LowRes.pdf%3Fraw%3Dtrue&embedded=true" width="600" height="800" style="border: none;"></iframe></center>

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


	
