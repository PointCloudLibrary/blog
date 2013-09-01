Ioan Dumitru Dragan
===================

This is my personal page

:email: ioan.dumitru.dragan@gmail.com
:project: Real-time segmentation and tracking
:mentor: **Nico Blodow** (Radu B. Rusu [Suat Gedikli, Michael Dixon])

About me
--------

I'm a student at `West University of Timisoara, Romania <http://math.uvt.ro>`_.  

My hobbies include:

* Coding
* Fishing
* Snowboarding
* Reading


Roadmap
-------

First, :ref:`read this description of the current pcl_cuda codebase <pcl_cuda>` for an overview.

Here is a brief outline of my GSoC project milestones. Click :ref:`here <ioan_roadmap>` for a more detailed roadmap.

* Fill out this page, discuss road map with mentor(s).

..

* Get to know the current code base for ``pcl_cuda``.

..

* *Documentation*: For the remainder of the milestones, while developing, make
  sure to document all code to allow the mentors to easily follow your train of
  thought and progress. 

..

* Together with the mentors, put in place a system to evaluate performance and
  correctness of the implemented methods.

..

* Port a seleciton of the existing parametric models and sample consensus
  algorithms variations to the new framework.

..

* Implement a bridging layer between OpenCV GPU and ``pcl_cuda`` where we can
  convert from a structured ``pcl_cuda::PointCloudAOS`` to ``cv::gpu::GpuMat``
  and vice versa.

..

* For true 3D cluster segmentation, implement a graph-based region growing
  segmentation algorithm (see ``pcl::EuclideanClusterExtraction``).

..



Recent status updates
---------------------

.. blogbody::
   :author: ioan
   :nr_posts: 5

.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
   :hidden:

   roadmap
   status
