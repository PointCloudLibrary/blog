Roadmap
=======
.. _mfallon_roadmap:

Implement a simulated RGB-D sensor (e.g kinect)
* initial input: vector of PolygonMesh   (read from .obj, .ply, .pcd etc)
* subsequent input: pose and calibration of sensor
* output: simulated range and colour images: using RangeImage class [Bastian Steder's work]
Probabilistic Methods for comparing simulated data to actual data
* simple distance metrics
* probabilistic metrics based on sensor configuration (e.g. particle filter likelihood)
integration with ICP for optimized position estimation
* i.e. the gradient and pose of the optimal particle can be passed to ICP for further refinement. 
