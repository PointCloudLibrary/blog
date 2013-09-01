Interesting Classes
===================
   This is a list of interesting classes that require code examples and documentation.  It is organized by module.

Common
------

Features
--------
* BoundaryEstimation
* CVFHEstimation
* FPFHEstimation
* FPFHEstimationOMP
* IntegralImage2D
* IntensityGradientEstimation
* IntensitySpinEstimation
* MomentInvariantsEstimation

Filters
-------
* ConditionalRemoval - Filters out data that does not satisfy specific conditions.  It can be passed two types of conditions; ConditionAnd and ConditionOr.  If passed a ConditionAnd type then only data that satisfies ALL of those conditions will be kept.  If passed one of the other type of condition (ConditionOr) then all data that satisfies ANY of those conditions will be kept.  One can chose to remove the filtered data and either just throw it away or keep it in another set by setting the *keep_organized_* data member.
* ExtractIndices - Extracts a set of indices from a PointCloud into another seperate PointCloud.  By setting *negative_* equal to true all points except the specified indices will be returned.
* PassThrough - Can either remove or modify points which satisfy a user specified constraint.
* ProjectInliers
* RadiusOutlierRemoval - Removes any points that have less than some user specified number of neighbors with a certain search radius.  All you have to do is use setRadiusSearch to set the radius around each point in which you are looking for neighbors and setMinNeighborsInRadius to set the minimum number of neighbors that a point must have within the specified radius to be considered not an outlier (inlier).
* StatisticalOutlierRemoval
* VoxelGrid - Creates a 3-dimensional grid (pixel = 2d, voxel = 3d) over a PointCloud.  It also downsamples and filters the voxels because it turns out after this process it represents the underlying surface more acurately.

Registration
------------
* IterativeClosestPoint - this is an algorithm that is used to minimize the differences between two PointClouds.  It essentially takes 2 PointClouds usually from two raw scans of an object or scene and refines them into a transformation of the 2.  It links each point with another in the other PointCloud using a nearest neighbor search, and then estimates the parameters of the transformation using the mean square cost. The algorithm iteratively repeates this over and over again until the criteria specified for stopping is met.

