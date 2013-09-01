How to use the ISS 3D keypoint detector
=========================================
.. _gioia_iss:

This section of my blog should be considered as an extra help to the usage of the ISSKeypoint3D detector. Basically, the ISSKeypoint3D detector could be run into two different modes:

  1. Without the border estimation.
  2. With the border estimation. 

In the first case, the boundary points are estimated by means of the pcl::BoundaryEstimation class and they are subsequently discarded while computing the final keypoints. In the second case, the estimation of the boundary points is skipped and the time performances do not depend on the pcl::BoundaryEstimation class.


ISS 3D detector: obtaining keypoints without performing first the border estimation.
************************************************************************************

In order to run the ISSKeypoint3D in this modality, the user must set:

  * the salient radius (strict positive)
  * the non maxima suppression radius (strict positive)
  * the border radius (equal to 0.0, as the default value)

while he can chose to set or not all the other parameters. 

The following code snippet allows to run the ISS 3D detector without the border estimation.::

  //
  //  ISS3D parameters
  //
  double iss_salient_radius_;
  double iss_non_max_radius_;
  double iss_gamma_21_ (0.975);
  double iss_gamma_32_ (0.975);
  double iss_min_neighbors_ (5);
  int iss_threads_ (4);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

  // Fill in the model cloud 

  double model_resolution;

  // Compute model_resolution
	
  iss_salient_radius_ = 6 * model_resolution;
  iss_non_max_radius_ = 4 * model_resolution; 

  //
  // Compute keypoints
  //
  pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;

  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (iss_salient_radius_);
  iss_detector.setNonMaxRadius (iss_non_max_radius_);
  iss_detector.setThreshold21 (iss_gamma_21_);
  iss_detector.setThreshold32 (iss_gamma_32_);
  iss_detector.setMinNeighbors (iss_min_neighbors_);
  iss_detector.setNumberOfThreads (iss_threads_);
  iss_detector.setInputCloud (model);
  iss_detector.compute (*model_keypoints);


ISS 3D detector: obtaining keypoints by performing first the border estimation.
*******************************************************************************

In order to run the ISSKeypoint3D in this modality, the user must set:

  * the salient radius (strict positive)
  * the non maxima suppression radius (strict positive)
  * the border radius (strict positive)
  * the normal radius (strict positive)

while he can chose to set or not all the other parameters. 

The following code snippet allows to run the ISS 3D detector with the border estimation.::


  //
  //  ISS3D parameters
  //
  double iss_salient_radius_;
  double iss_non_max_radius_;
  double iss_normal_radius_;
  double iss_border_radius_;
  double iss_gamma_21_ (0.975);
  double iss_gamma_32_ (0.975);
  double iss_min_neighbors_ (5);
  int iss_threads_ (4);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

  // Fill in the model cloud 

  double model_resolution;

  // Compute model_resolution
	
  iss_salient_radius_ = 6 * model_resolution;
  iss_non_max_radius_ = 4 * model_resolution;
  iss_normal_radius_ = 4 * model_resolution;
  iss_border_radius_ = 1 * model_resolution; 

  //
  // Compute keypoints
  //
  pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;

  iss_detector.setSearchMethod (tree);
  iss_detector.setSalientRadius (iss_salient_radius_);
  iss_detector.setNonMaxRadius (iss_non_max_radius_);

  iss_detector.setNormalRadius (iss_normal_radius_);
  iss_detector.setBorderRadius (iss_border_radius_);
 
  iss_detector.setThreshold21 (iss_gamma_21_);
  iss_detector.setThreshold32 (iss_gamma_32_);
  iss_detector.setMinNeighbors (iss_min_neighbors_);
  iss_detector.setNumberOfThreads (iss_threads_);
  iss_detector.setInputCloud (model);
  iss_detector.compute (*model_keypoints);


