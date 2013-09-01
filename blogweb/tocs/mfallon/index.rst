Maurice Fallon
==============

This is my personal page

:email: mfallon@mit.edu
:project: Real-time Global Point Cloud Localization


About me
--------
I am a Research Scientist in the Computer Science and Artifical Intelligence Laboratory (CSAIL) at MIT. 

I work with Professor John Leonard and am interested in probabilistic methods for localization, tracking and mapping.

Click here to checkout my `MIT webpage <http://people.csail.mit.edu/mfallon/>`_

Roadmap
-------
First implement a simulated RGB-D sensor (e.g kinect)

* initial input: vector of PolygonMesh   (read from .obj, .ply, .pcd etc)

* subsequent input: pose and calibration of sensor

* output: simulated range and colour images: using RangeImage class [Bastian Steder's work]

* Extension/addition of LIDAR scanners

Probabilistic Methods for comparing simulated data to actual data

* Simple distance metrics

* Probabilistic metrics based on sensor configuration (e.g. particle filter likelihood)

* Study trade-offs for a variety of simplification scenarios e.g. depth image sub-sampling, map simplification

Integration of a particle filter for state propagation

* Integration with PCL's ICP methods for optimized position estimation i.e. the gradient and pose of the optimal particle can be passed to ICP for further refinement. 

* Global localization within a large point cloud?

Recent status updates
---------------------

.. blogbody::
  :author: mfallon
  :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:

  status
