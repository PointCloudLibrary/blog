Changhyun Choi
==============

:email: cchoi@cc.gatech.edu
:project: 3D edge extraction from an organized point cloud
:mentor: Alex Trevor

About me
--------
I am a Ph.D. student in Center for Robotics and Intelligent Machines, College of Computing, Georgia Institute of Technology. 

Project Description
-------------------
The goal of our GSOC is to implement a 3D edge extraction algorithm from an organized point cloud. We are interested in the edges come from boundaries of either geometric structures or photometric texture. To find these edges, we will combine multiple cues from RGB-D channels. We plan to extract 2D edges from the given RGB channels and back-project these edges to the given depth channel so as to determine reliable 3D edges. We also plan to exploit edges from depth discontinuities or high curvature regions that may not be captured from the RGB channels. We want the code to provide several options to choose the types of resulting edges such as, depth discontinuity, RGB edge, high curvature, and etc.

Roadmap
-------

* Implement various types of edges

	- Occluding boundary edges (edge of occluding object)
	- Occluded boundary edges (edge on surface being occluded)
	- High curvature edge
	- 2D edge (image only edge, not curvature or occlusion boundary)
	- 2D + high curvature edge
	- 2D + occluding boundary edge
	- 2D + occluded boundary edge

* Implement a nice interface for those edges

Recent status updates
---------------------

.. blogbody::
  :author: cchoi
  :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:

  status
