My status updates
=================
 
	   
.. blogpost::
  :title: Milestone 1: Metrics Framework for 3D descriptors
  :author: adambr
  :date: 29-07-2013
	
  As mentioned in the roadmap, one of the goals is to implement a framework that captures vital statistics of selected descriptors and correspondence types. These vital statistics would then be analyzed by one or more objective function(s) to enable scene based optimizations.

  The first milestone, a metrics framework for descriptor evaluation is now complete, and its output is in-line with the characteristics cited in Rublee et. al. ICCV 2011 paper, among other publications.

  Specifically, the framework computes the intended vital statistics including: 2-Way and multi-descriptor matching and inlier rates. The filter banks include L2-distance, L2-ratio, and uniqueness measure. A simulated ground truth is also implemented and is generated during runtime. The framework has been applied to local 3D descriptors (FPFH33, SHOT352, and SHOT1344) across a range of downsampling leaf-sizes (0.01-0.07) and across a range of in-plane (0-90 degrees) rotations. A sample of the results is illustrated in the bar graphs below, which reflect the various metrics, computed at a 30 degree simulated rotation and at 2 levels of downsampling: 0.01 for the top bar graph and 0.07 for the next one. In total, 1680 rates were generated for further analysis by the objective function(s). A link is included below to a sample extended output for other 3D descriptors.  Next step: to extend the framework to support 2D descriptors.
  
	
  .. image:: images/fpfh01deg30.png
    :width: 1000 px

  .. image:: images/fpfh07deg30.png
    :width: 1000 px

  The extended output for other 3D descriptors follows:	
	
  .. raw:: html

    <iframe src="http://docs.google.com/viewer?url=http://svn.pointclouds.org/tocsweb/source/adambr/files/MD3DDescriptorsExtendedOutput.pdf&amp;embedded=true" style="border: none;" height="400" width="800"></iframe>
		

My status updates
=================
.. blogpost::
  :title: Project - Multi-Descriptor Optimizations across the 2D/3D Domains
  :author: adambr
  :date:   06.03.2013

  The project has started. 


	
