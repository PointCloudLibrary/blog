Roadmap
=======

.. role:: strike
    :class: strike
    
.. _rosen_roadmap:

* Research various out-of-core spacial data structures and databases as well as
  investigate preexisting out-of-core techonologies.

   * Provide references for the development of the project
   * Design new file formats for storage of spacial data structure and databases
   * Implement point cloud processing tools for creating new file formats

* Initial integration of Urban Robotics's octree-based point cloud format in PCL

   * :strike:`Get code compiling within PCL framework`
   * :strike:`Use Google Test instead of Boost's Unit Test Framework`
   * Refactor current code

      * Everything's in a header
      * Remove old deprecated code
      * Cleanup cut/paste sections
      * Understand how everything works
      * Documentation

   * Replace underlying point data structures with PCL's, ie PointType
   * Remove depenedency on cJSON library

* Develop a barebones visualization UI with QVTK providing interactive
  frame rates of massive point clouds.

   * Learn (Q)VTK and the plausaility of using their framework out-of-the-box
   * May need to develop first class objects for direct rendering
   * Incorporate GLSL shaders.
   * Implement multiresolution rendering algorithms for reading in processed
     point clouds

* Research aspects of parallel programming
* ...