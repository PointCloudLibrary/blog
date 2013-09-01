PCL Tutorials
=============
.. _goleary-tutorials:

   This is a list of all features available in the PCL and whether or not each one currently has a tutorial made for it.

Completed
======================

Features
--------

   - FPFHEstimation - done
   - NormalEstimation - done
   - PFHEstimation - done
   - VFHEstimation - done

Filter - done
-------------

   - :doc:`ConditionalRemoval <tutorials/conditional_removal>`
   - ExtractIndices
   - PassThrough
   - ProjectInliers
   - :doc:`RadiusOutlierRemoval <tutorials/radius_outlier_removal>`
   - StatisticalOutlierRemoval
   - VoxelGrid

IO
--

   - PCDReader - done
   - PCDWriter - done
   - concatenateFields - done
   - concatenatePointCloud - done

Octree
------

   - OctreePointCloudChangeDetector - done

Range_Image
-----------

   - RangeImage - done

Registration
------------

   - :doc:`IterativeClosestPoint - done <tutorials/iterative_closest_point>`

Segmentation
------------

   - EuclidianClusterExtraction - done
   - SACSegmentation - done
   - SACSegmentationFromNormals - done

Surface
-------

   - :doc:`ConcaveHull - done <tutorials/concave_hull>`
   - ConvexHull - done
   - MovingLeastSquares - done
   - GreedyProjectionTriangulation - done

Visualization
-------------

   - CloudViewer - done
   - PCLVisualizer - done

Modules
=======

Common
------
* Classes

   - BivariatePolynomialT
   - NdConcatenateFunctor
   - PCA
   - PiecewizeLinearFunction
   - PointCorrespondence
   - PointCorrespondence3d
   - PointCorrespondence6d
   - PolynomialCalculationsT
   - PosesFromMatches
   - ScopeTime
   - TimeTrigger
   - TransformationFromCorrespondences
   - VectorAverage
   - PointXYZ
   - PointXYZI
   - PointXYZRGBA
   - PointXYZRGB
   - PointXY
   - InterestPoint
   - Normal
   - PointNormal
   - PointXYZRGBNormal
   - PointXYZINormal
   - PointWithRange
   - PointWithViewpoint
   - MomentInvariants
   - PrincipalRadiiRSD
   - Boundary
   - PrincipalCurvatures
   - PFHSignature125
   - FPFHSignature33
   - VFHSignature305
   - Narf36
   - BorderDescription
   - IntensityGradient
   - Histogram
   - PointWithScale
   - PointSurfel
   - cuda::ScopeTimeCPU

Features
--------
* Classes

   - BoundaryEstimation
   - CVFHEstimation
   - Feature
   - FPFHEstimationOMP
   - IntegralImage2D
   - IntegralGradientEsitmation
   - IntensitySpinEstimation
   - MovementInvariantsEstimation
   - Narf
   - NarfDescriptor
   - NormalEstimationOMP
   - PrincipalCurvaturesEstimation
   - RangeImageBoarderExtractor
   - RIFTEstimation
   - RSDEstimation

* Functions

   - solvePlaneParameters
   - computerPointNormal
   - flipNormalTowardsViewpoint
   - computePairFeatures
   - computerRSD

IO
--
   - getFieldIndex
   - getFields
   - getFieldsList
   - getFieldSize
   - getFieldType
   - copyPointCloud
   - getPointCloudAsEigen
   - getEigenAsPointCloud
   - loadPCDFile
   - savePCDFile
   - savePCDFileASCII
   - savePCDFileBinary
   - saveVTKFile

KDTree
------
* Classes

   - KdTree
   - KdTreeFLANN
   - OrganizedDataIndex

* Functions

   - initTree

Keypoints
---------
* Classes

   - Keypoint
   - NarfKeypoint
   - SIFTKeypoint

* Functions

   - operator<<

Octree
------
* Classes

   - Octree2BufBase
   - OctreeBase
   - OctreeLowMemBase
   - OctreePointCloud
   - OctreePointCloudDensity
   - OctreePointCloudOccupancy
   - OctreePointCloudVector
   - OctreePointCloudSinglePoint
   - OctreePointCloudVoxelCentroid


Range_Image
-----------
* Classes

   - RangeImagePlanar

Registration
------------
* Classes

   - CorrespondenceEstimation
   - CorrespondenceRejector
   - CorrespondenceRejectorDistance
   - CorrespondenceRejectorOneToOne
   - CorrespondenceRejectorReciprocal
   - CorrespondenceRejectorSampleConsensus
   - CorrespondenceRejectorTrimmed
   - sortCorrespondencesByQueryIndex
   - sortCorrepsondencesByMatchIndex
   - sortCorrepsondencesByDistance
   - sortCorrepsondencesByQueryIndexAndDistance
   - sortCorrepsondencesByMatchIndexAndDistance
   - SampleConsensusInitialAlignment
   - IterativeClosestPointNonLinear
   - Registration
   - registration::TransformationEstimation
   - registration::TransformationEstimationSVD

* Functions

   - transformPointCloud
   - transformPointCloudWithNormals

Segmentation
------------
* Classes

   - ExtractPolygonalPrimsData
   - SegmentDifferences

* Functions

   - ExtractEuclidianClusters
   - comparePointClusters
   - isPointIn2DPolygon
   - isXYPointIn2dXYPolygon
   - getPointCloudDifference

Surface
-------
* Classes

   - GridProjection
   - SurfaceReconstruction

* Functions

   - comparePoints2D
   - isVisible

Visualization
-------------
* Classes

   - PCLHistogramVisualizer
   - PCLVisualizerInteractorStyle
   - PointCloudGeometryHandler
   - PointCloudGeometryHandlerXYZ
   - PointCloudGeometryHandlerSurfaceNormal
   - PointCloudGeometryHandlerCustom
   - PointCloudGeometryHandlerRandom
   - PointCloudGeometryHandlerRGBField
   - PointCloudGeometryHandlerGenericField

* Function

   - getCorrespondingPointCloud
   - savePointData
   - createPolygon
   - createLine
   - createPlane
   - create2DCircle
   - createCone
