My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: anurag


.. blogpost::
  :title: Test post
  :author: anurag
  :date: 08-08-2014

      * **Introduction**
        Point cloud library (PCL) is a collection of algorithms which can be used from simple computer vision applications (filtering, object recognition) to highly computationally expensive machine learning applications. PCL is used today in various fields like robotics, gaming, entertainment, computer vision, medical etc. PCL is cross-platform library, and can be used from Desktop to mobile platforms. To simplify development, PCL is separated into several modules like filtering, features, registration etc. These modules have algorithms which are computationally expensive and from them real time performance is required.

OpenCL Module for PCL is one among the others attempt which are trying to increase the user productivity by increasing real time performance of algorithms. The goal of this GSOC Project is to optimize PCL modules using OpenCL so that these optimized modules can be used on various heterogeneous platforms.

For implementing an OpenCL algorithm, we have to allocate the memory to device, run the kernel and get back the results from device to host. We have implemented a common platform Utils inside OCL module which can be used by different algorithms for memory allocation and running the kernel. Utils also take care of different point types of PCL library. For different point type we are not able to define structure for all point types inside OpenCL kernel. This module takes the point type as an argument and allocates memory according to that and removes the necessity of defining structure inside kernel.   

The most important two modules which are used mostly inside PCL library are filtering and searching. During this summer we have optimized some filtering algorithms and wrote the whole octree based searching method using OpenCL which is used in almost all PCL modules. We have also optimized the tracking library which uses particle filtering based tracker to track the object and heavily depends on searching and filtering modules.
