Construct a concave hull polygon
================================

	In this tutorial we will learn how to calculate a simple 2D concave hull polygon for a set of points supported by a plane.

The code
========

	First download the dataset and save it somewher to disk.  Then, create a file, let's say, concave_hull_2d.cpp in your favorite editor and place the following inside:

	.. code-block:: cpp

		#include <pcl/ModelCoefficients.h>
		#include <pcl/io/pcd_io.h>
		#include <pcl/point_types.h>
		#include <pcl/sample_consensus/method_types.h>
		#include <pcl/sample_consensus/model_types.h>
		#include <pcl/filters/passthrough.h>
		#include <pcl/filters/project_inliers.h>
		#include <pcl/segmentation/sac_segmentation.h>
		#include <pcl/surface/concave_hull.h>

		int
		 main (int argc, char** argv)
		{
		  pcl::PointCloud<pcl::PointXYZ>::Ptr
		    cloud (new pcl::PointCloud<pcl::PointXYZ>),
		    cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
		    cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
		  pcl::PCDReader reader;
		  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);

		  // Build a filter to remove spurious NaNs
		  pcl::PassThrough<pcl::PointXYZ> pass;
		  pass.setInputCloud (cloud);
		  pass.setFilterFieldName ("z");
		  pass.setFilterLimits (0, 1.1);
		  pass.filter (*cloud_filtered);
		  std::cerr << "PointCloud after filtering has: " <<
		    cloud_filtered->points.size () << " data points." << std::endl;

		  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		  // Create the segmentation object
		  pcl::SACSegmentation<pcl::PointXYZ> seg;
		  // Optional
		  seg.setOptimizeCoefficients (true);
		  // Mandatory
		  seg.setModelType (pcl::SACMODEL_PLANE);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setDistanceThreshold (0.01);

		  seg.setInputCloud (cloud_filtered);
		  seg.segment (*inliers, *coefficients);

		  // Project the model inliers 
		  pcl::ProjectInliers<pcl::PointXYZ> proj;
		  proj.setModelType (pcl::SACMODEL_PLANE);
		  proj.setInputCloud (cloud_filtered);
		  proj.setModelCoefficients (coefficients);
		  proj.filter (*cloud_projected);

		  // Create a concave Hull representation of the projected inliers
		  pcl::PointCloud<pcl::PointXYZ>::Ptr
		    cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		  pcl::concaveHull<pcl::PointXYZ> chull;
		  chull.setInputCloud (cloud_projected);
		  chull.reconstruct (*cloud_hull);

		  std::cerr << "Concave hull has: " << cloud_hull->points.size () <<
		     " data points." << std::endl;

		  pcl::PCDWriter writer; 
		  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull,
		  false);

		  return (0);
		}

The explanation
===============

	The only interesting part is in the lines below, where the ConcaveHull object gets created and the reconstruction is performed:

	.. code-block:: cpp

		pcl::PointCloud<pcl::PointXYZ>::Ptr
		  cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ConcaveHull<pcl::PointXYZ> chull;
		chull.setInputCloud (cloud_projected);
		chull.reconstruct (*cloud_hull);

Compiling and running the program
=================================

	Add the following lines to your CMakeLists.txt file:

	.. code-block:: cpp

		cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

		project(concave_hull_2d)

		find_package(PCL 1.0 REQUIRED)

		include_directories(${PCL_INCLUDE_DIRS})
		link_directories(${PCL_LIBRARY_DIRS})
		add_definitions(${PCL_DEFINITIONS})

		add_executable (concave_hull_2d concave_hull_2d.cpp)
		target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})

	After you have made the executable, you can run it. Simply do:

	.. code-block:: cpp

		$ ./concave_hull_2d

	You will see something similar to:

	.. code-block:: cpp

		PointCloud after filtering has: 139656 data points.
		Concave hull has: 30 data points.

