.. _glong_roadmap:

Surface Reconstruction with Textures - Project Roadmap
======================================================

Surface reconstruction methods produce very varied outputs, yet some of them (or perhaps all) share common functionality or output type. These similarities need to be identified, not only based on the current methods, but such that it is general enough to be applied to methods as well that are not yet implemented.

The detailed outline of the GSoC project milestones is given below:

  * Design a common interface (API) for all surface reconstruction methods in PCL

  * Extend PCL's libpcl_surface using the following algorithms:

    - Marching Cubes
    - Poisson (Misha allows us to integrate his code directly into PCL: http://www.cs.jhu.edu/~misha/Code/PoissonRecon/)

  * Connect PCL to VTK's surface smoothing algorithms

  * Learn what new methods we need to implement by looking at Meshlab (code is GPL - do not take anything from it, just look at the performance of certain algorithms which we can afterwards reimplement from the original papers)

  * Unit test, document, provide code samples for each method implemented

