PCL CUDA
========
.. _pcl_cuda:

First, some notes on the existing code base:
--------------------------------------------

  Currently, the existing code base using CUDA in PCL is hosted in a seperate repo
  (``pcl_cuda``) because of some compiler issues. It is not possible to use the
  existing ``pcl::PointCloud<PointT>`` class within ``pcl_cuda`` because the
  NVidia nvcc compiler has some issues with the (heavily templated) Eigen types in
  the ``PointCloud`` class.
  
  Therefore, there is a ``pcl_cuda::PointCloudAOS`` class, which is templated on
  a class Storage. There are two classes that can be used as Storage,
  ``pcl_cuda::Device`` and ``pcl_cuda::Host``, which both contain nothing but a
  typedef for the respective host or device vectors from thrust.
  
  This defines the basic structure for most classes and algorithms: everything
  starts with a beautiful:
  
  .. code-block:: cpp
  
     template <template <typename> class Storage>
     class MyClass
     {
       // this takes a const reference to a shared_ptr to a const PointCloud, templated on Storage ..
       void do_point_cloud_stuff (const boost::shared_ptr <const PointCloudAOS <Storage> > &input);
  
       // this returns a device vector of float4
       typename pcl_cuda::Device<float4>::type compute_normals ();
  
       // this returns vector of float4 which either lives on the device or the host, depending on the template param.
       typename pcl_cuda::Storage<float4>::type compute_normals ();
     };

  .. note::
     we are using `thrust <http://code.google.com/p/thrust/>`_, which is a very
     nice C++ template library for CUDA based on the Standard Template Library,
     included in CUDA as of version 4. Make sure to read some of the tutorials
     from the thrust documentation!
  
  Concerning the organization of the code: Everything is split up in different
  folders such as ``common``, ``features``, or ``sample_consensus``. Headers live in
  ``include/*/*.h``, and CPU source code in ``src/*/*.cpp``. CUDA coda needs to be
  compiled with nvcc and must reside in ``src/*/*.cu`` files. There are certain
  things that can not be done in .cpp files, such as:
  
  .. code-block:: cpp

     thrust::device_vector<int> ints; // equivalent to pcl_cuda::Device<int>::type
     ints.resize (17);  // <-- this does not work
  
  This is the reason why there is e.g. ``normal_3d.cu`` which basically just wraps the
  CUDA/thrust function calls so they appear in the shared object file and can be
  linked to from the CPU code. See the ``CMakeLists.txt`` files for examples on
  how to add new libraries or executables.
  
  .. note::
     In the future, we might have a new ``PointCloud`` class in pcl which will make
     integration with ``pcl_cuda`` easier.
  
  
What is currently implemented in ``pcl_cuda``?
----------------------------------------------
  As of now, we have several algorithms implemented:

  * ``PointCloudAOS`` data structure for Host and Device Storage (see above): as
    of now, the point cloud contains a vector of ``PointXYZRGB``. For other cloud
    types, such as normals, we simply use e.g. ``Storage<float4>::type``.

  * an important module is ``io``, it contains conversion functions from
    ``pcl::`` to ``pcl_cuda::`` (``cloud_from_pcl.h``, ``cloud_to_pcl.h``),
    image debayering (``debayering.h``), PointCloud projection from depth
    images (``disparity_to_cloud.h``), and host to device transfer functions
    (rarely needed, ``host_device.h``). Also, ``colorIndices``, a function to
    change the RGB values of points lives in ``extract_indices.h``.
    This contains everything needed to go from a pair of Kinect images (RGB +
    Depth) to a point cloud with colors.

  * Surface Normals and Curvature are the only features currently implemented
    (``features/normal_3d.h`` and ``common/eigen.h``). They assume a point
    cloud that originated from a depth image for neighborhood searches, since
    there are no spatial search structures implemented (yet).

  * There are some basic plane estimation algorithms in ``sample_consensus``.
    Note that the only relatively stable combination right now is the
    ``SACModel1PointPlane`` in combination with a ``RandomSampleConsensus`` or
    ``MultiRandomSampleConsensus`` class. See
    ``src/tools/kinect_ransac.cpp`` as an example.

  * The currently available tools need to link against ``pcl`` for some things,
    such as ``libpcl_io`` for OpenNIGrabber etc.

