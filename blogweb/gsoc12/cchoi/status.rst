My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: cchoi

.. blogpost::
	:title: Testing existing boundary estimation in PCL 
	:author: cchoi
	:date: 5-24-2012

	As a first step for 3D edge detection, I would like to focus on edges from geometric shape (e.g. depth discontinuities or high curvature regions). Before I write codes, I have tested the existing functions in PCL which are pcl::NormalEstimation and pcl::BoundaryEstimation. Since the boundary estimation is designed for unorganized point clouds, it takes several seconds to process a Kinect frame. For an efficient boundary estimation, I will design a boundary estimation for organized point cloud (i.e. Kinect frame).

	Following images are the curvature and boundary images from pcl::NormalEstimation and pcl::BoundaryEstimation.

	* **Curvature**

		.. image:: images/05242012/screenshot-1337804031.png
			:width: 738px
			:height: 536px
			:align: center

	* **Boundaries**

		.. image:: images/05242012/screenshot-1337804028.png
			:width: 738px
			:height: 536px
			:align: center

.. blogpost::
	:title: Occluding & Occluded Boundary Edges
	:author: cchoi
	:date: 6-8-2012

	For the last couple of weeks, I have developed a boundary edge detection (pcl::OrganizedEdgeDetection) for an organized point cloud. It mainly searches for depth discontinuities with a given threshold value which is linearly adpated with respect to depth values. Since the point cloud is organized, operation is quite efficient. While pcl::BoundaryEstimation takes several seconds, my unoptimized code takes about 70 ms. It also returns edge labels: occluding, occluded, and boundary (i.e. neither occluding nor occluded) edges. 

	In Kinect or similar sensors, it happens that 'nan' points exist between occluding and occluded edges. So my algorithm searches for corresponding points across the 'nan' area. This search is done in an organized fashion, so it isn't so time consuming.

	Following images show the detected occluding (green), occluded (red), and boundary (blue)edges:

		.. image:: images/06082012/screenshot-1338586968.png
			:width: 640px
			:height: 480px
			:align: center

		.. image:: images/06082012/screenshot-1338586994.png
			:width: 640px
			:height: 480px
			:align: center

		.. image:: images/06082012/screenshot-1338587035.png
			:width: 640px
			:height: 480px
			:align: center

.. blogpost::
	:title: High Curvature Edges
	:author: cchoi
	:date: 6-22-2012

	During this week, I have implemented 'high curvature edge' estimation. Here, the high curvature edges are defined as ridge or valley edges that are not on objects' boundaries. These sharp edges could be useful in registration, recognition, and tracking applications. I first tried the curvature values obtained from normal estimation, but it turned out that a simple thresholding scheme using curvature values does not work very well. Especially, it was hard to get clean and thin edges on these high curvature regions. I noticed that non-maximum suppression and hysteresis thresholding are required. So I employed a canny edge implementation (pcl::pcl_2d::edge::canny()). Instead of using the RGB values available in the given organized point cloud, I used 'normal_x' and 'normal_y' images, since high gradient responses on these normal images correspond to high curvature regions.

	Following images show the detected occluding (green), occluded (red), boundary (blue), and high curvature (yellow) edges:

		.. image:: images/06222012/screenshot-1340407069.png
			:width: 640px
			:height: 480px
			:align: center

		.. image:: images/06222012/screenshot-1340407079.png
			:width: 640px
			:height: 480px
			:align: center

		.. image:: images/06222012/screenshot-1340407086.png
			:width: 640px
			:height: 480px
			:align: center

.. blogpost::
	:title: RGB Edges
	:author: cchoi
	:date: 7-10-2012

	I added RGB edge detection, which is 2D canny edge detection from the RGB channels, in pcl::OrganizedEdgeDetection. Right now, the class only takes point types having RGB channels, but it will be changed so that possible edges can be detected from a given point cloud type. For example, 'occluding', 'occluded', and 'boundary' edges can be detected from any XYZ point types. And 'high curvature' and 'rgb' edges can be obtained from Normal and RGB point types, respectively. 

	Following images show the detected occluding (green), occluded (red), boundary (blue), high curvature (yellow), and rgb (cyan) edges:

		.. image:: images/07102012/screenshot-1341964342.png
			:width: 640px
			:height: 480px
			:align: center

		.. image:: images/07102012/screenshot-1341964387.png
			:width: 640px
			:height: 480px
			:align: center

.. blogpost::
	:title: Better interface for various point cloud types
	:author: cchoi
	:date: 7-31-2012

        I have modified the previous code so that it finds possible edges from various point cloud types:

        * OrganizedEdgeBase accepts PCL_XYZ_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, and EDGELABEL_OCCLUDED.
        * OrganizedEdgeFromRGB accepts PCL_RGB_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_RGB_CANNY.
        * OrganizedEdgeFromNormals accepts PCL_XYZ_POINT_TYPES with PCL_NORMAL_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, and EDGELABEL_HIGH_CURVATURE.
        * OrganizedEdgeFromRGBNormals accepts PCL_RGB_POINT_TYPES with PCL_NORMAL_POINT_TYPES and returns EDGELABEL_NAN_BOUNDARY, EDGELABEL_OCCLUDING, EDGELABEL_OCCLUDED, EDGELABEL_HIGH_CURVATURE, and EDGELABEL_RGB_CANNY.

        OrganizedEdgeFromRGB and OrganizedEdgeFromNormals are derived from OrganizedEdgeBase. OrganizedEdgeFromRGBNormals is then derived from both OrganizedEdgeFromRGB and OrganizedEdgeFromNormals. 

.. blogpost::
	:title: Openni interface for 3D edge detection
	:author: cchoi
	:date: 8-10-2012

        .. raw:: html
        
                <center><iframe width="640" height="480" src="http://www.youtube.com/embed/dePKTsVsxq4" frameborder="0" allowfullscreen></iframe></center>

        An openni interface for 3D edge detection is added in PCL trunk. Once it starts, you can show and hide each edge by pressing the corresponding number:
        
        * 1: boundary edges (blue)
        * 2: occluding edges (green)
        * 3: occluded edges (red)
        * 4: high curvature edges (yellow)
        * 5: rgb edges (cyan)

        The high curvature and rgb edges are not enabled for fast frame rate, but you can easily enable these two edges if you want to test.
        
.. blogpost::
	:title: Wrap-up posting for 3D edge detection
	:author: cchoi
	:date: 8-28-2012

		.. image:: images/07102012/screenshot-1341964387.png
			:width: 320px
			:height: 240px
			:align: center

        My primary goal of GSOC'12 was to design and implement a 3D edge detection algorithm from an organized point cloud. Various edges are detected from geometric shapes (boundary, occluding, occluded, and high curvature edges) or photometric texture (rgb edges). These edges can be applicable to registration, tracking, etc. Following code shows how to use the organized edge detection::
       
                pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> oed;
                oed.setInputNormals (normal);
                oed.setInputCloud (cloud);
                oed.setDepthDisconThreshold (0.02); // 2cm
                oed.setMaxSearchNeighbors (50);
                pcl::PointCloud<pcl::Label> labels;
                std::vector<pcl::PointIndices> label_indices;
                oed.compute (labels, label_indices);
                
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>), 
                        occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>), 
                        boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
                        high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
                        rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);
                
                pcl::copyPointCloud (*cloud, label_indices[0].indices, *boundary_edges);
                pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
                pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
                pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
                pcl::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);
        
        For more information, please refer to following codes in PCL trunk:

        * `pcl-trunk/apps/src/pcd_organized_edge_detection.cpp <http://svn.pointclouds.org/pcl/trunk/apps/src/pcd_organized_edge_detection.cpp>`_.
        * `pcl-trunk/apps/src/openni_organized_edge_detection.cpp <http://svn.pointclouds.org/pcl/trunk/apps/src/openni_organized_edge_detection.cpp>`_.
        
        It was a great pleasure to be one of the GSOC participants. I hope that my small contribution will be useful to PCL users. Thank Google and PCL for the nice opportunity and kind support. Lastly, thank Alex Trevor for mentoring me.