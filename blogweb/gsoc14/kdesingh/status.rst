My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: kdesingh


.. blogpost::
  :title: Richtsfeld's code and results (First post)
  :author: kdesingh
  :date: 16-06-2014

        * **Introduction**

                Goal of this project is to integrate cluttered scene segmentation methods into pcl::segmentation. 
                This sprint involves implementation of modules from two publications below. 
        

                - Z.-C. Marton, F. Balint-Benczedi, O. M. Mozos, N. Blodow, A. Kanezaki, L. C. Goron, D. Pangercic, and M. Beetz: Part-based geometric categorization and object reconstruction in cluttered table-top scenes; Journal of Intelligent and Robotic Systems, January 2014

                - A.-Richtsfeld, T. Mörwald, J. Prankl, M. Zillich and M. Vincze: Learning of Perceptual Grouping for Object Segmentation on RGB-D Data; Journal of Visual Communication and Image Representation (JVCI), Special Issue on Visual Understanding and Applications with RGB-D Cameras, July 2013
        
                These papers have made their code available and has pcl implementation to some extent already. But we will aim to make the modules interoperable in our implementation.

        * **Richtsfeld's code and results**
         
                As a first step, Richtsfeld's code was analyzed. Below is the highlevel picture of the structure of his code base along with comments on their functionality.
        
                .. image:: images/v4r.png
                        :width: 1400px
                        :height: 1100px
                        :align: center

                His code was tested on some scenes that were grabbed by me using Kinect. Below are the snapshots of the same.
        
                .. image:: images/scene_01.png
                        :width: 650px
                        :height: 250px
                        :align: center

                .. image:: images/scene_02.png
                        :width: 650px
                        :height: 250px
                        :align: center

                .. image:: images/scene_03.png
                        :width: 650px
                        :height: 250px
                        :align: center

                His code works on organized pointcloud of type PointXYZRGB.

                Code is yet to be tested for quantitative results with our annotated dataset. 

        * **Brainstorming**
                
                These are some of the discussions I had with my mentor Zoltan and the summary is listed below.
                        
				- Zoltan's work classifies the relations based on features computed on group of segments as 1-8 elements in a group.
				- Richtsfeld's work classifies the relations based on segment pairs.
				- Richtsfeld's work is limited to organized pointcloud and cannot handle a cloud that is fused out of many pointclouds of a scene say through registration.
				- Richtsfeld's work computes features which are inspired from Gestalt's principles. There are someother features that are worth testing. These are the features used for structure discovery in a pointcloud data. Features are as below and more details on them are available in the publication - Collet, Alvaro, Siddhartha S. Srinivasa, and Martial Hebert. "Structure discovery in multi-modal data: a region-based approach." Robotics and Automation (ICRA), 2011 IEEE International Conference on. IEEE, 2011.
				- Additional features to test are:

						- Shape Model
						- Self Continuity
						- Contour compactness
						- Pair continuity
						- Verticality
						- Concavity
						- Projection
						- Alignment
						- Surface compatibility 
        

               

.. blogpost::
  :title: RSD Feature computation and analysis
  :author: kdesingh
  :date: 26-06-2014

  RSD Feature is a local feature histogram that describes the surface local to a query point. There is pcl implementation for this that is available in the features folder. 
  With the help of my mentor I understood the algorithm by which this feature is obtained. To very if this is working perfectly we took a real object whose radius is known and 
  generated the RSD computation on the entire point cloud of the object. This gives RSD Feature Histogram for all the points in the pointcloud. We can also get the min and max 
  radius of the local surface patch around each point in the pointcloud. 
  I generated various combination of parameters to know how the radius computed varies. Below is the object used which has a radius of 3.5cm which is 0.035m
  
                  .. image:: images/coffee_mug.png
                        :width: 333px
                        :height: 400px
                        :align: center

  Below are some of the params chosen and their corrresponding effect on the min and max radius in the local surface patch of each point.
  For Normal Radius search = 0.03
  Max_radius = 0.7 (maximum radius after which everything is plane)
  RSD_radius search = 0.03
  
                  .. image:: images/s1.png
                        :width: 1080px
                        :height: 400px
                        :align: center
                        
  For Normal Radius search = 0.03
  
  Max_radius = 0.1 (maximum radius after which everything is plane)
  
  RSD_radius search = 0.03
  
                  .. image:: images/s2.png
                        :width: 1080px
                        :height: 400px
                        :align: center
                        

  For Normal Radius search = 0.02
  
  Max_radius = 0.1 (maximum radius after which everything is plane)
  
  RSD_radius search = 0.03 - This is found to be good way for generating histograms
  
  
                  .. image:: images/s3.png
                        :width: 1080px
                        :height: 400px
                        :align: center
                        
  I tried to do MLS smoothing on the point cloud data and then compute the RSD feature which makes the normal computation better and resulting in
  consistency over all the points on the object surface.
  
  For Normal Radius search = 0.03
  
  Max_radius = 0.7 (maximum radius after which everything is plane)
  
  RSD_radius search = 0.03
  
                  .. image:: images/sm1.png
                        :width: 1080px
                        :height: 400px
                        :align: center
                        
  For Normal Radius search = 0.03
  
  Max_radius = 0.1 (maximum radius after which everything is plane)
  
  RSD_radius search = 0.03
  
                  .. image:: images/sm2.png
                        :width: 1080px
                        :height: 400px
                        :align: center
                        

  For Normal Radius search = 0.02
  
  Max_radius = 0.1 (maximum radius after which everything is plane)
  
  RSD_radius search = 0.03 - This is found to be good way for generating histograms
  
                  .. image:: images/sm3.png
                        :width: 1080px
                        :height: 400px
                        :align: center
                        
                                                
  Now I tested out how the actual feature looks like at a point on the sphere to check if it matches with the histogram in the paper.
  The same is compared between raw point cloud from the kinect and MLS smoothened point cloud. Below is the result of the same.
  
                    .. image:: images/sphere_comparison_blog.png
                        :width: 1080px
                        :height: 600px
                        :align: center
                        
  It was really hard to fix the previous image that it can show the histograms with values and good resolution. So below is the snapshot of the spherical and cylinderical surfaces.
  
  Cylinderical Surface:
  
                      .. image:: images/cylinder_rsd_mls_1.png
                        :width: 400px
                        :height: 300px
                        :align: center
                        
  Spherical Surface:
  
                      .. image:: images/sphere_rsd_mls_2.png
                        :width: 400px
                        :height: 300px
                        :align: center                        
                        
  
  Next post will have the details of how GRSD results are and how they differentiate the characteristics of two surfaces. GRSD code from the author will be integrated into the PCL code base. We also plan to categorize the pipeline into modules that fit into the PCL code base as features, surface and segmentation sections. These information will be posted in the next post.
 


.. blogpost::
  :title: GRSD Descriptor computation and analysis
  :author: kdesingh
  :date: 28-07-2014

  Global Radius-based surface descriptor concatenates the RSD descriptor as discussed in the previous post to represent the complete object.
  GRSD gives a good description of the 3D shape of the object. Below are the set of objects and its GRSD descriptors i.e. histograms. I have
  used University of Washington's "Large scale RGBD dataset" for the experiments.
                      
  For an object whose surface is planar but has 2 different planes in the view
                      .. image:: images/box_image.png
                        :height: 240px
                      .. image:: images/box.png
                        :height: 200px


  For an object whose surface is planar but has 1 planes in the view
                      .. image:: images/plane_image.png
                        :height: 240px
                      .. image:: images/plane.png
                        :height: 200px

  For an object whose surface is spherical
                      .. image:: images/sphere_image.png
                        :height: 240px
                      .. image:: images/sphere.png
                        :height: 200px

  For an object whose surface is cylinderical but doesn't have any planar surface in view
                      .. image:: images/cylinder_image.png
                        :height: 240px
                      .. image:: images/cylinder.png
                        :height: 200px

  It can be seen that all the descriptors are different from eachother. Planes and box surfaces are similar as the surface characteristics
  are similar in this case.                         
  Both GRSD and RSD are pushed into the pcl-trunk for people to use. The test files for these two features are also included in the trunk
  for the basic usage of the same.

  Currently working on the NURBS for small surface patches. Since NURBS are already available in PCL we will be looking at how to tailor the
  same for our needs. After this we plan to work on the features that compute the relationship between the surface patches. 


.. blogpost::
  :title: Different Feature representation of surface patches and analysis
  :author: kdesingh
  :date: 18-08-2014
  
  This project aims at getting features for each surface patch and group them to gether based on a machine learning technique. I have been experimenting
  on the features that could represent the surface patches belonging to an object and how they relate with the inter and intra object surface patches.
  Below are some of them, with their description and plots.

  For all the below analysis please look at the image below and the segments derived out of basic `region growing <http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php>`_ which doesn't make use of RGB color and purely based on surface curvature. 

                      .. image:: images/segmentation.png
                        :height: 300px
                        :align: center

  **ColorHistogram:** 
  Binning colors is pretty normal thing to get the appearance cues. But they can be done in two ways. One that captures the color channels independently
  and the other that captures a dependent binning. In this section I am showing the independent binning of values on three different channels. Independent means at a pixel/point in scene we check RGB values different and increment the bin where these values belong to. Below is the plot that shows how the histograms look like for RGB and HSV color space. 

                      .. image:: images/color_rgb.png
                        :height: 350px
                        :align: center
                      .. image:: images/color_hsv.png
                        :height: 350px
                        :align: center

  
  **ColorHistogram 3D:**                        
  Binning colors dependently means having a matrix with RGB in 3 dimensions and increment the value of the (r, g, b) bin only if that combination is 
  satified. This gives a 3D histogram. Below images show the 3D histogram concatenated to a 1D histogram for RGB, HSV and YUV color space. This binning style is 
  referred from [1].

                      .. image:: images/colorh_3d_rgb.png
                        :height: 350px
                        :align: center
                      .. image:: images/colorh_3d_hsv.png
                        :height: 350px
                        :align: center
                      .. image:: images/colorh_3d_yuv.png
                        :height: 350px
                        :align: center                        
   
  **Verticality:**
  Verticality actually represents how the surface patch is orientated with respect to the camera viewpoint. Histogram is developed by binning the 
  difference of angles between the normals and the direction in which the camera is pointing to (i.e the positive z axis for any point cloud from 
  Kinect). Below is the histogram plot for this feature. However this is not so useful to this segmentation project but more aimed towards the object 
  discovery and saliency related problems to distinguish the a surface patch from its peers. This is implemented based on [2] which was adopted from [3].

                      .. image:: images/verticality.png
                        :height: 350px
                        :align: center


  **Dimentionality Compactness:**
  This actually shows how compact a surface patch is. One could do a PCA of a surface patch and derive a local frame of reference. This when followed by 
  creating a bounding box gives the 3 dimensions in which the patch ranges the max. This bounding box will have 3 dimensions and their ranges are computed. Once this ranges (xrange, yrange, zrange) are sorted into min_range, mid_range and max_range, two ratios are computed. 1) min_range / max_range and 2) mid_range / max_range. Below is plot of this histogram for the above mentioned segments. This is implemented based on [2] which was adopted from [3].

                      .. image:: images/dimensionality_compactness.png
                        :height: 325px
                        :align: center


  **Perspective scores:**
  This is the ratio of the area projected in the image to the maximum area spread by the region in 3D. The pixel_range below means the bounding box range in the
  particular direction of the image pixels. xrange, yrange and zrange are the dimentions of the 3D bounding box of the surface patch. Note that PCA shouldn't done here as we are comparing the 3D surface patch with its appearance on the image. This is implemented based on [2] which was adopted from [3]. Below are the elements of the 
  histogram.
	
	1) pixel_x_range (in meters) / xrange

	2) pixel_x_range (in meters) / yrange

	3) pixel_x_range (in meters) / zrange

	4) pixel_y_range (in meters) / xrange

	5) pixel_y_range (in meters) / yrange

	6) pixel_y_range (in meters) / zrange

	7) diameter_of_bounding_box_pixels (in meters)/ diameter_of_cuboid_bounding_box_in_3d

	8) area_of_bounding_box (in metersquare) / area_of_the_largest_two_dimentions_in_3d


                      .. image:: images/perspective_hist.png
                        :height: 350px
                        :align: center

  **Contour Compactness:**
  
  This is the ratio of the perimeter to the area of the region. This is the ratio of number of boundary points computed for a segment to the total 
  number of points in the region. This is implemented based on [2] which was adopted from [3].

                      .. image:: images/contour_compactness.png
                        :height: 350px
                        :align: center 

  **Referrences:**
  
  [1] A.Richtsfeld, T. Mörwald, J. Prankl, M. Zillich and M. Vincze: Learning of Perceptual Grouping for Object Segmentation on RGB-D Data; Journal of Visual Communication and Image Representation (JVCI), Special Issue on Visual Understanding and Applications with RGB-D Cameras, July 2013.

  [2] Karthik Desingh, K Madhava Krishna, Deepu Rajan, C V Jawahar, "Depth really Matters: Improving Visual Salient Region Detection with Depth", BMVC 2013.
  
  [3] Alvaro Collet Romea, Siddhartha Srinivasa, and Martial Hebert, "Structure Discovery in Multi-modal Data : a Region-based Approach", ICRA 2011.

  These features will go into the features module of the pcl_trunk pretty soon. Currently working on the relational features which tells how two
  surfaces are related to each other. Next blog post should be on that.                        

  Almost close to the end stage of GSoC, but this project has long way to go! Hope to keep pushing stuff till the entire pipeline is up on PCL.

