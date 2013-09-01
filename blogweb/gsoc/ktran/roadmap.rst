.. _ktran_roadmap:

Surface Reconstruction with Textures - Project Roadmap
======================================================

The surface reconstruction algorithms that are currently in PCL solve the core problem, but they need to be more flexible with respect to the used input data.

The milestones of this GSoC project are detailed in two parts below.

Part I
------

In the first part, the task is to extend the existing surface reconstruction methods to be able to deal with textured surfaces.

  * starting from one of the existing methods (or designing one new method), design and implement the surface reconstruction API for texture mapping onto a mesh

  * work on perfect texture blending for two overlapping textures onto a surface mesh

    - References: 

      - http://vision.ucsd.edu/kriegman-grp/papers/egsr03.pdf
      - http://research.microsoft.com/en-us/um/people/hoppe/lapped.pdf
      - http://vcg.isti.cnr.it/publications/papers/tessjourn.pdf
      - http://www.cemyuksel.com/research/meshcolors/meshcolors_tog.pdf
      - http://www.cadanda.com/V2Nos1to4_48.pdf 
  

Part II
-------

The goal in the second part is to extend the flexibility and functionality of 3D surface reconstruction methods, in combination with the results from the first part.

  * add capabilities for surface mesh updates, i.e. when a new point cloud dataset is added to an existing surface mesh, without recreating the entire mesh

  * combine the mesh update and texture blending for merging two textured surfaces

  * extend the MLS algorithm by providing capabilities for resampling, hole filling, and setSearchSurface

    - References:

      - http://www.cs.jhu.edu/~misha/Fall05/bolitho.10.25.05.pdf
      - http://www.cs.jhu.edu/~misha/
      - http://files.rbrusu.com/publications/Marton09ICRA.pdf


