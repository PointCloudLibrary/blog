Roadmap
=======
.. _shapovalov_roadmap:

Here is a detailed outline of my GSoC project milestones:

* Fill out this page
* Decide the appropiate benchmark data set for object reconition and pose.

  - CAD models (e.g. Google Warehouse -- synthetic data).
  - Data obtained from the Kinect (real data).

* Implement generic trainer API interface

  - Given training data in a certain format, it should need to load the data and compute the specific descriptor and make all the needed information persistent for the matcher.
  - Documentation.
  - Some descriptors might need specific subclasses to fulfill its specific needs.

* Implement generic matcher API interface

  - Similar to the generic trainer but for matching purposes.
  - Implement a brute force matcher
  - Implement an approximate kd-tree matcher
  - Implement a RANSAC based matcher interface

* Benchmarking of existing descriptors.
  
  - Eventually create a new training set.

