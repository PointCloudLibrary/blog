Roadmap
=======
.. _ioan_roadmap:

First, some notes on the existing code base:
--------------------------------------------

.. important::
   For a small overview on the current state of the ``pcl_cuda`` project, see :ref:`this article <pcl_cuda>`.

Here is a detailed, preliminary outline of my GSoC project roadmap:
-------------------------------------------------------------------

If there is anything you would like to change about this roadmap, feel free to
discuss this with your mentor(s). Milestones will be very broad at the beginning
and will be specified in further detail throughout the project.

Milestones / Steps to happiness:

* Get to know the current code base for ``pcl_cuda``.
  Install the CUDA SDK and driver, and get the repository to compile. Run the
  sample programs included in ``tools``. Look at their main functions for
  arguments (usually when started with any argument (``./kinect_ransac foo``),
  it will use the GPU implementation, and CPU otherwise).

..

* *Documentation*: For the remainder of the milestones, while developing, make
  sure to document all code to allow the mentors to easily follow your train of
  thought and progress. 

.. At the end of the project, write a tutorial explaining the use of the methods
   developed during the GSOC project.

* Together with the mentors, put in place a system to evaluate performance and
  correctness of the implemented methods.

..

* For starters, using the existing ``RandomSampleConsensus`` class, implement ``MSAC``
  (M-estimator SAmple and Consensus). Incrementally, port the existing
  parametric models and sample consensus algorithms variations to the new
  framework.

..

* Test this first in a simple program (like ``tools/kinect_ransac``) and once it
  works, also with MultiRandomSampleConsensus à la ``tools/kinect_multi_ransac``.

..

* OpenCV has several image algorithms for GPUs, e.g.
  ``cv::gpu::meanShiftSegmentation``. Implement a bridging layer where we can
  convert from a structured ``pcl_cuda::PointCloudAOS`` to ``cv::gpu::GpuMat``
  and vice versa.

.. Alternatively, implement Mean Shift Segmentation for the GPU, as an
   efficient algorithm for segmenting a depth image or organized (regular grid)
   Point Cloud. 

* For true 3D cluster segmentation, implement a graph-based region growing
  segmentation algorithm (see ``pcl::EuclideanClusterExtraction``).

  - For starters, the organized neighborhood search from the normal estimation
    code could be used.
  - As other GSOC students progress with their projects, it might be possible to
    use a different spatial locator structure (octree or kd-tree) at this point
    in order to expand the applicability of these methods to true unstructured
    3D point clouds.
  - Since ``thrust`` extensively uses functors / function objects, provide a way
    to pass in different distance functions (e.g.  to improve euclidean
    clustering by specifying color thresholds as well.)

.. Since on a frame-to-frame basis, most segmentation results will stay more or
   less stable, results from the previous frame should be incorporated as an
   initial guess for the next frame. This general principle is applicable to any
   kind of segmentation method, and the method should be designed accordingly

