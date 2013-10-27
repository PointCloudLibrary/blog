My status updates
=================


.. blogpost::
  :title: Started implementing 3DGSS Features
  :author: aichim
  :date: 12-02-2011
	
  As mentioned in the roadmap, one of the steps would be to implement a method that would help find edges in the depth images. The one I started looking into was proposed by John Novatack and Ko Nishino in "Scale-Dependent 3D Geometric Features". The final goal of the paper is to have scale-dependent 3D feature descriptors. But on the way, they compute edges and corners in 3D.
	
  The novelty of the approach is that they compute the scale-space of a dense and regular 2D representation of the surface using the normals of the scan. Technically, they create the Gaussian pyramid of the "normal images" and also the first and second derivative (i.e. Laplacian) of the levels of this pyramid.
	 
  Just like in 2D computer vision, the edges are found by looking for the zero-crossings of the Laplacian of the normal maps at different scales (+ some thresholding on the corresponding first derivative).
	
  An example of the pyramid of normal maps:
	
  .. image:: figures/gss3d_normal_maps.png
    :width: 800 px
		
  Edges found in an example scene:
		
  .. image:: figures/gss3d_edges.png
    :width: 400 px
		
  More results will be posted once this is finished.

	
	  
.. blogpost::
  :title: Results from analyzing MLS smoothing
  :author: aichim
  :date: 11-19-2011
	
	I started off by doing some experiments with one of the smoothing algorithms we already have implemented in the PCL library: Moving Least Squares smoothing.

	First, the influence of the search radius on the smoothing output was analyzed (without fitting a polynomial). The following figure shows the results: from bottom to top, left to right: original kinect cloud, MLS with search radii: 0.01, 0.02, 0.05; color coding by curvature.

  .. image:: figures/kinect_mls_experiment_search_radii.png
    :width: 900 px
	
  Best result that smooths the wall to a plane and keeps the shapes of the objects is obtained with search_radius of 0.02 (=2 cm). 0.01 does not perfectly smooth the wall and 0.05 eliminates the depth of the small figure on the desk.
	
  The time performance was looked into and the collected data is presented in the following table. Two different search approaches were used: the kdtree implementation from FLANN and the search::OrganizedNeighbor class using a window-based approach (approximate method).
	
  =================   ===========   =================   ================
  MLS search_radius   KdTreeFLANN   OrganizedNeighbor   Time improvement
  =================   ===========   =================   ================
  0.01                5.6 s         3 s                 46 %
  0.02                19.1 s        10.8 s              43 %
  0.05                107 s         61.8 s              42 %
  0.07                201.3 s       117.3 s             42 %
  =================   ===========   =================   ================
	
  So, the immediate conclusion is that MLS is definitely not suited for real-time application. It would be a viable option as a post-processing step for the registration pipeline we mentioned in the roadmap.

  Next, we varied the order of the polynomial to be fitted. The following figure shows the results: MLS with polynomial fitting of orders 0, 1, 2, and 3 with a constant search radius of 0.02 (ordered left to right, bottom to top).

  .. image:: figures/kinect_mls_experiment_poly_fit.png
    :width: 900 px
	
  The result differences are rather subtle, some fine details tend to be preserved with higher order polynomial fitting. But these fine details are mostly due to noise and the time expenses one has to pay for the additional polynomial fitting is not totally worth the small improvements, as the following table shows:

  ====================   =======   ===================
  MLS polynomial order   Time      Increase to order 0
  ====================   =======   ===================
  0                      18.4 s    0 %
  1                      19.4 s    5 %
  2                      22.5 s    22 %
  3                      25.8 s    40 %
  ====================   =======   ===================
	
	
	
.. blogpost::
  :title: Virtual Scanner Improvements
  :author: aichim
  :date: 11-26-2011
	
  In the past days, I have been doing some research in the literature regarding point cloud upsampling and smoothing, trying to find some approaches that might be suitable with the PrimeSense cameras. Will produce a blogpost regarding this as soon as I have done some conclusive experiments.
	
  Until then, the following figure shows the current status of the Virtual Scanner application. It now has a GUI written in VTK, where the user can load VTK-compatible objects, freely manipulate a camera and produce 3D scans of the scene. The scanned cloud is shown live in another window.
	
  .. image:: figures/virtual_scanner.png
    :width: 900 px
	
  I have tried to make the artifacts of the output cloud to be similar with the ones produced by the Kinect. The solution for the quantization artifacts was suggested by Suat and it consists of the following:
	
  * the depth of a pixel is defined by :math:`Z = f * b / d` where :math:`f` is the focal length in pixels (measured at 575 pixels for the Kinect), :math:`b` is the baseline (7.5 cm) and :math:`d` is the disparity measured in pixels.
	
  * the Kinect quantizes the disparity by 1/8-th of a pixel.
	
  * add Gaussian noise before quantizing 
	
  * an example of such an artifact:
	
    * consider a pixel with a disparity of :math:`d_2 = 5 px \Rightarrow Z_2 = 8.625 m`
		
    * the next disparity value is :math:`d_1 = 5.125 px \Rightarrow Z1 = 8.415 m`
		
    * and the previous one was :math:`d_3 = 4.875 px \Rightarrow Z3 = 8.8461 m`
		
    * The difference is of :math:`21 cm` between the first two and increases to :math:`22.1 cm` at the next quantized disparity value and will continue to increase at larger distances
		
		
  There are still some interface issues to be solved, and this will be commited to trunk soon.
	
	
.. blogpost::
  :title: Back in Action and new Project requirements
  :author: aichim
  :date: 12-21-2011
	
  I have not been too active lately due to intense school activities (exams and end of semester projects/presentations). I am now ready to continue with my TOCS assignments.

  A couple of weeks ago, some discussions took place between Toyota and PCL representatives and my project got a bit more clearer. The things I am going to spend my following days on is creating a database of recordings of different household items and household-specific	scenes. Next, I shall apply all the current algorithms we have in PCL for surface smoothing and reconstruction and report back with the results of a qualitative analysis of the output.
	
	
.. blogpost::
  :title: PCL Surface Architecture Updates
  :author: aichim
  :date: 12-30-2011
	
  With the help of Michael and Radu, we have made a few changes to the pcl::surface module. We have now structured it by adding three base classes which differentiate between algorithms with distinct purposes:

  * MeshConstruction - reconstruction algorithms that always preserve the original input point cloud data and simply construct the mesh on top (i.e. vertex connectivity)	
	
    * input: point cloud
		
    * output: PolygonMesh using the input point cloud as the vertex set
		
    * examples: ConcaveHull, ConvexHull, OrganizedFastMesh, GreedyProjectionTriangulation
	
  * SurfaceReconstruction - reconstruction methods that generate a new surface or create new	vertices in locations different than the input point cloud
	
    * input: point cloud
		
    * output: PolygonMesh with a different underlying vertex set
		
    * examples: GridProjection, MarchingCubes, MovingLeastSquares, SurfelSmoothing
				
  * MeshProcessing - methods that modify an already existent mesh structure and output a new mesh

    * input: PolygonMesh
		
    * output: PolygonMesh with possibly different vertices and different connectivity
		
    * examples: EarClipping, MeshSmoothingLaplacianVTK, MeshSmoothingWindowedSincVTK, MeshSubdivisionVTK
		
		
  Please notice the new classes ending with VTK. We already had these implemented in PCL before, but in quite a simple state. They are now fully usable and documented.
	
  The recordings for the required datasets is in progress and they will be tested with most of the algorithms mentioned above.
	
  Also, a new Poisson implementation is underway.
	
	
.. blogpost::
  :title: Point Cloud Smoothing Project DATASETS
  :author: aichim
  :date: 01-04-2012
	
  As required by Toyota, we started recording a series of typical household scenes. This first post shows the first 23 recordings we did using an **Asus Xtion Pro** camera. One can easily download them by the following command::

    svn co http://svn.pointclouds.org/data/Toyota
	
  Those datasets are mainly meant to represent realistic situations that a personal robot might face in an undirected human environment. All of the scenes are recorded starting from a distance of about 3-4 meters from the main subject and getting close and rotating around it, in order to simulate the behavior of a robot and to capture most of the artifacts that the PrimeSense cameras present.
	
  These are split into the following categories:	
    
  * *Bed Sheets* - 3 styles of bed sheets in bedrooms:

    * **bed_sheets/style_1/** - 152 frames

      .. image:: figures/toyota_datasets//bed_sheets__style_1.png
        :width: 300 px

    * **bed_sheets/style_2/** - 205 frames

      .. image:: figures/toyota_datasets//bed_sheets__style_2.png
        :width: 300 px

    * **bed_sheets/style_3/** - 240 frames

      .. image:: figures/toyota_datasets//bed_sheets__style_3.png
        :width: 300 px


  * *Bottles* - 2 layouts on a table in the kitchen

    * **bottles/set_1/** - 180 frames

      .. image:: figures/toyota_datasets//bottles__set_1.png
        :width: 300 px

    * **bottles/set_2/** - 260 frames

      .. image:: figures/toyota_datasets//bottles__set_2.png
        :width: 300 px

  * *Door Handles* - 5 styles of indoor/outdoor door handles

    * **door_handles/style_1/** - 200 frames

      .. image:: figures/toyota_datasets//door_handles__style_1.png
        :width: 300 px

    * **door_handles/style_/** - 330 frames

      .. image:: figures/toyota_datasets//door_handles__style_2.png
        :width: 300 px

    * **door_handles/style_3/** - 232 frames

      .. image:: figures/toyota_datasets//door_handles__style_3.png
        :width: 300 px

    * **door_handles/style_4/** - 199 frames

      .. image:: figures/toyota_datasets//door_handles__style_4.png
        :width: 300 px

    * **door_handles/style_5/** - 256 frames

      .. image:: figures/toyota_datasets//door_handles__style_5.png
        :width: 300 px

  * *Glasses* - one recording for opaque mugs and one for transparent glasses in the kitchen

    * **glasses/opaque/** - 246 frames

      .. image:: figures/toyota_datasets//glasses__opaque.png
        :width: 300 px

    * **glasses/transparent/** - 364 frames

      .. image:: figures/toyota_datasets//glasses__transparent.png
        :width: 300 px

  * *Keyboards* - 4 different laptop keyboards on an office desk

    * **keyboards/laptop_1** - 249 frames

      .. image:: figures/toyota_datasets//keyboards__laptop_1.png
        :width: 300 px

    * **keyboards/laptop_2** - 220 frames

      .. image:: figures/toyota_datasets//keyboards__laptop_2.png
        :width: 300 px

    * **keyboards/laptop_3** - 157 frames

      .. image:: figures/toyota_datasets//keyboards__laptop_3.png
        :width: 300 px

    * **keyboards/laptop_4** - 221 frames

      .. image:: figures/toyota_datasets//keyboards__laptop_4.png
        :width: 300 px

  * *Shoes* - 2 recordings

    * **shoes/single/** - 275 frames

      .. image:: figures/toyota_datasets//shoes__single.png
        :width: 300 px

    * **shoes/multiple/** - 200 frames

      .. image:: figures/toyota_datasets//shoes__multiple.png
        :width: 300 px

  * *Tupperware* - 3 recordings of tupperware on the kitchen table

    * **tupperware/single/** - 358 frames

      .. image:: figures/toyota_datasets//tupperware__single.png
        :width: 300 px

    * **tupperware/multiple/** - 337 frames

      .. image:: figures/toyota_datasets//tupperware__multiple.png
        :width: 300 px

    * **tupperware/stacked/** - 286 frames

      .. image:: figures/toyota_datasets//tupperware__stacked.png
        :width: 300 px

  * *Other* - 2 other recordings I found interesting for the point cloud smoothing problem

    * **other/small_windows/** - 262 frames

      .. image:: figures/toyota_datasets//other__small_windows.png
        :width: 300 px

    * **other/textured_wall/** - 219 frames

      .. image:: figures/toyota_datasets//other__textured_wall.png
        :width: 300 px


.. blogpost::
  :title: Point Cloud Smoothing Benchmarks - MovingLeastSquares
  :author: aichim
  :date: 01-05-2012

  After we have collected part of our datasets of interest *(there are still some objects missing from our collection, will get them next week)*, we proceed in testing our available smoothing algorithms. Please note that these tests use only real sensor data of scanned objects that are rather irregular, so we do not have any ground truth for our benchmarks. As such, we will limit ourselves just to a visual inspection of the results. This inspection will look mostly into sensor artifacts that we might have in the clouds after the algorithms were applied (please see the problem description page for more details) or artifacts caused by the algorithm itself (issues such as over-smoothing).

  **Bed_sheets Dataset**

  One of the best algorithms we currently have in the PCL library is the MovingLeastSquares implementation. We ran this algorithm on the *bed_sheets* dataset and tweaked the parameters to see the situations it creates.

  The first image, from left to right: 

  * input cloud **bed_sheets/style_1/frame_00050.pcd**

  * MLS-smoothed with parameters:

    * search_radius: 0.05
    * sqr_gauss_param: 0.0025
    * processing time: ~19 seconds

  * MLS-smoothed with parameters:

    * search_radius: 0.03
    * sqr_gauss_param: 0.0009
    * processing time: ~46 seconds.

  .. image:: figures/smoothing_results_1/mls_bed_sheets_style_1.png
    :width: 700 px

  The results seem satisfactory, in general. MLS removes some of the quantization effects (note that the bed was at about 1.5-2m away from the camera), although the slices are still clearly visible. Due to the fact that the details in some wrinkles were lost using a 5 cm smoothing radius, we also tried a 3 cm radius, which seemed to reduce the over-smoothing effect.

  The second image, left to right:

  * input cloud **bed_sheets/style_2/frame_00050.pcd**

  * MLS-smoothed with parameters:

    * search_radius: 0.05

    * sqr_gauss_param: 0.025

    * processing time: ~46 seconds

  * MLS-smoothed with parameters:

    * search_radius: 0.05

    * sqr_gauss_param: 0.0025

    * use_polynomial_fit: 1

    * polynomial_order: 2

    * processing time: ~73 seconds

  .. image:: figures/smoothing_results_1/mls_bed_sheets_style_2.png
    :width: 700 px

  Here, we show that the usage of polynomial fitting in the MLS algorithm is useful for preserving sharp edges. One can see that the image in the middle is over-smoothed with the 5 cm radius, but the ridges are preserved in the third image.

  **Tupperware Dataset**

  MLS was applied to the *tupperware* dataset and obtained the following results.

  Both images, from left to right:

  * input cloud **tupperware/multiple/frame_00050.pcd**

  * MLS-smoothed with parameters:

    * search_radius: 0.03

    * sqr_gauss_param: 0.0009

    * use_polynomial_fit: 1

    * polynomial_order: 2

    * processing time: ~11 seconds

  * MLS-smoothed with parameters:

    * search_radius: 0.05

    * sqr_gauss_param: 0.0025

    * use_polynomial_fit: 1

    * polynomial_order: 2

    * processing time: ~22 seconds

  .. image:: figures/smoothing_results_1/mls_tupperware_quantization.png
    :width: 700 px

  On on hand, MovingLeastSquares seems to group points together and form visible 'long holes'. This is due to the heavy quantization errors introduced by the sensor - the table and the curtains in the back are at about 2.5-4m from the camera.

  .. image:: figures/smoothing_results_1/mls_tupperware_smoothing.png
    :width: 700 px

  On the other hand, it clearly improves the shape of the objects. The second figure shows a top-down view of the table. The tupperware seems much more smoother and grippable, without loss of information.


  **Glasses Dataset**

  In the list of objects we are interested in, there are transparent glasses/mugs. Unfortunately, the PrimeSense technology proves incapable of recording ANY depth for the points corresponding to the glasses, as shown in the following image. There is nothing a surface reconstruction algorithm can do in order to recreate the points on the glasses, so we shall discard this dataset in our following benchmarks.

  .. image:: figures/smoothing_results_1/glasses_transparent.png
    :width: 700 px

  **Bottles Dataset**

  As expected, the transparent parts of the plastic bottles have not been recorded by the depth sensor.

  The image below, from left to right:

  * input cloud **bottles/set_1/frame_00050.pcd**

  * MLS-smoothed with parameters:

    * search_radius: 0.03

    * sqr_gauss_param: 0.0009

    * use_polynomial_fit: 1

    * polynomial_order: 2

    * processing time: ~19 seconds

  * MLS-smoothed with parameters:

    * search_radius: 0.05

    * sqr_gauss_param: 0.0025

    * use_polynomial_fit: 1

    * polynomial_order: 2

    * processing time: ~45 seconds

  .. image:: figures/smoothing_results_1/mls_bottles.png
    :width: 700 px

  The result is very satisfactory. MLS does NOT add any points in the reconstruction, but one can notice the very good silhouette of the bottles, as compared to the very noisy input.



.. blogpost::
  :title: VTK Smoothing Algorithms and Other Updates
  :author: aichim
  :date: 01-17-2012

  For the VTK smoothing tests, we took the raw clouds, triangulated them using the OrganizedFastMesh triangulation with the TRIANGLE_ADAPTIVE_CUT option, and then fed this to the 3 smoothing algorithms from the VTK library.

  The first one to be tested is **MeshSmoothingLaplacianVTK** with the default parameters recommended by VTK, but with an increase on the number of iterations from 20 to 100.

  **Bed_sheets Dataset**

  Here, the results are satisfactory, in both cases, the quantization artifacts are reduced (they are still visible).

  .. image:: figures/smoothing_results_2/mesh_smoothing_laplacian_bed_sheets_1.png
    :width: 700 px

  .. image:: figures/smoothing_results_2/mesh_smoothing_laplacian_bed_sheets_2.png
    :width: 700 px

  Also, if we look at the corresponding mesh, the reconstruction after smoothing looks more natural, with a better surface curvature.

  .. image:: figures/smoothing_results_2/mesh_smoothing_laplacian_bed_sheets_2_surface.png
    :width: 700 px


  **Bottles and Tupperware Datasets**

  In this case, the Laplacian smoothing does not work well anymore. The quantization and the high noise level is still present in the case of both the bottles and tupperware datasets. The main reason for this is the fact that the objects of interest were quite far away from the sensor and the quantization artifacts are quite accentuated (i.e., there are large gaps between the points belonging to the same object).

  .. image:: figures/smoothing_results_2/mesh_smoothing_laplacian_bottles.png
    :width: 700 px

  .. image:: figures/smoothing_results_2/mesh_smoothing_laplacian_tupperware.png
    :width: 700 px

  The mesh subdivision schemes we have been provided by the VTK library are not of great use for our scenarios, as they just split up triangles in the mesh, inheriting from their artifacts. Furthermore, these schemes are highly dependent on the quality of the initial triangulation - which in our case is the simple OrganizedFastMesh - does not yield excellent results. They basically just resample point on the triangles present in the input mesh, without taking into consideration any more complex information about the vertex neighborhood.

  .. image:: figures/smoothing_results_2/mesh_subdivision.png
    :width: 700 px

  Another thing we tried was to combine the simple subdivision with the previous laplacian smoothing, and the results are  visually decent, as shown in the next figure. Again, we inherit the problems of the subdivision scheme (the holes caused by the incorrect triangulation).

  .. image:: figures/smoothing_results_2/upsampling_and_laplacian.png
    :width: 700 px


  In the meantime, I have worked on solving some issues with Zoltan Marton's Greedy Projection Triangulation. Two trac issues regarding this were solved, but its current state does not allow us to reconstruct Kinect scans - once we solve this, I will do benchmarking on the gp3 algorithm too. Other time-consuming fixes were done for OrganizedFastMesh.

  A direction we definitely need to look into is to have some algorithms that also add points during reconstruction. The original MLS and GP3 papers do mention this possibility, but they have not been implemented in PCL yet. It is clear so far that we still do not have the Holy Grail of smoothing yet.



.. blogpost::
  :title: TOCS Dataset collection now complete!
  :author: aichim
  :date: 01-27-2012

  We have managed to collect all the datasets required by Toyota. For a complete description, please visit the following :ref:`link <aichim_datasets>` (also accessible from my main page).

  Programming-wise, we have spent time fixing bugs and beautifying the pcl_surface module. After I will finish my exams next week, I shall start looking into implementing some new algorithms.

.. blogpost::
  :title: Mesh Construction Methods
  :author: aichim
  :date: 02-02-2012

  In this blog post, we shall inspect the mesh construction methods available in PCL.

  **Marching Cubes**

  The algorithm was first presented 25 years ago in:

    * William E. Lorensen, Harvey E. Cline: Marching Cubes: A high resolution 3D surface construction algorithm. In: Computer Graphics, Vol. 21, Nr. 4, July 1987

  In PCL, these are implemented in the MarchingCubes class with the variants MarchingCubesGreedy and MarchingCubesGreedyDot. The 'greedy' comes from the way the voxelization is done. Starting from a point cloud, we create a voxel grid in which we mark voxels as occupied if a point is close enough to the center. Obviously, this allows us to create meshes with a variable number of vertices (i.e., subsample or upsample the input cloud). We are interested in the performance of this algorithm with the noisy Kinect data. Time-wise, the algorithm ran in about 2-3 seconds for a 640x480 cloud.

  The following figure shows the results for various leaf sizes (from left to right, bottom to top: leaf size of 0.5 cm, 1 cm, 3 cm, and 6 cm, respectively):

  .. image:: figures/mesh_construction/marching_cubes_varying_leaf.png
    :width: 700 px

  And a close-up on the highest resolution cloud:

  .. image:: figures/mesh_construction/marching_cubes_0_05_cm_leaf.png 
     :width: 700 px

  We conclude that the results are not satisfactory, as the upsampling is 'artificial' and does not inherit the properties of the underlying surface. Furthermore, there is no noise-removal mechanism and the blocking artifacts are disturbing.

  **Naive Algorithm for Organized Point Cloud Triangulation**

  This algorithm is implemented in the OrganizedFastMesh class in PCL. The idea behind is very simple: it takes each point in the inherent 2D grid of the Kinect clouds and triangulates it with its immediate neighbors in the grid. One can quickly understand that NaN points (points that were not captured by the sensor) will result in holes in the mesh. This is a mesh construction method and will output a mesh with exactly the same vertices as the input cloud. It does not take care of noise or NaN values in any way.

  A screenshot of the output can be seen in the following figure. Visually, the result is decent, considering that the processing time is extremely small - just a single pass through all the points of the clouds.

  .. image:: figures/mesh_construction/organized_fast_mesh.png
     :width: 700 px



.. blogpost::
  :title: Moving Least Squares Upsampling Methods
  :author: aichim
  :date: 02-08-2012

  With some very good advice from Zoltan and a lot of hacking, we now have 3 upsampling methods for the MLS algorithm.

  **1. NONE**

  No additional points are created here. The input pointcloud is projected to its own MLS surface. This is exactly what we previously tested and presented in a recent blog post.

  .. image:: figures/mesh_construction/organized_fast_mesh.png
     :width: 400 px

  **2. SAMPLE_LOCAL_PLANE**

  For each point, sample its local plane by creating points inside a circle with fixed radius and fixed step size. Then, using the polynomial that was fitted, compute the normal at that position and add the displacement along the normal. To reject noisy points, we increased the threshold for the number of points we need in order to estimate the local polynomial fit. This guarantees that points with a 'weak neighborhood' (i.e., noise) do not appear in the output.

  And a few results:

  .. image:: figures/mls_upsampling/mls_slp_table_bottles.png
     :width: 700 px

  .. image:: figures/mls_upsampling/mls_slp_table_tupperware.png
     :width: 700 px

  .. image:: figures/mls_upsampling/mls_slp_door_handle.png
     :width: 700 px

  The first picture is the reconstruction of coke bottles on a table. Please notice the correction for the quantization effects. The table surface is now planar and the objects look 'grippable'. The second picture is a similar scenario, now using tupperware. The last picture shows how well the door handle is reconstructed. We conclude that visually, the results are very good.

  An immediate problem is that this method adds the same amount of new samples to all points, not taking into account the local point density. An improvement we can make on this approach is to filter it with a voxel grid in order to have a uniform point density. Of course, this operation is superfluous, and is useful just for memory saving (see picture below for comparison between upsampled and upsampled + voxel grid).

  .. image:: figures/mls_upsampling/mls_slp_with_without_voxel_grid.png
     :width: 400 px


  **3. UNIFORM_DENSITY**

  Take as input a desired point density within a neighborhood with a fixed radius. For each point, based on the density of its vicinity, add more points on the local plane using a random number generator with uniform distribution. We then apply the same procedure as for **2** to project the point to the MLS surface.

  .. image:: figures/mls_upsampling/mls_uniform_curtains.png
     :width: 900 px

  The results are satisfying. As compared to **2**, we do not need to apply the expensive voxel grid filter anymore. An issue might be the fact that, because we generate the points using a random number generator, the output point cloud looks a bit messy (as compared to **2**, where the points are generated on a grid determined by the step size), but the surface is still well preserved. Also, the time performance is poor because of the rng.


  **4. FILL_HOLES**

  This method makes sense theoretically, but in practice we are having serious issues optimizing it to fit into main memory. The idea behind it is to take each point pair within a fixed radius neighborhood and to sample the line connecting these two points. Ideally, this would fill up any small holes inside the cloud. The downside is that it also creates a lot of additional points in already dense areas. Handling this elegantly is something we need to think about.

.. blogpost::
  :title: Moving Least Squares Upsampling Methods (cntd)
  :author: aichim
  :date: 02-19-2012

  In the last period, I have concentrated on coming up with new and better upsampling methods for the Moving Least Squares algorithm. Also, a lot of issue on the ones presented last time were solved.

  Everything was committed to trunk (along with a complete interface and documentation) and should be included in the next PCL release. The upsampling methods are the following:

  * **NONE** - no upsampling will be done, only the input points will be projected to their own MLS surfaces

  * **SAMPLE_LOCAL_PLANE** - the local plane of each input point will be sampled in a circular fashion using the upsampling_radius and the upsampling_step parameters

  * **RANDOM_UNIFORM_DENSITY** - the local plane of each input point will be sampled using an uniform random distribution such that the density of points is constant throughout the cloud - given by the desired_num_points_in_radius parameter

  * **VOXEL_GRID_DILATION** - the input cloud will be inserted into a voxel grid with voxels of size voxel_size; this voxel grid will be dilated dilation_iteration_num times and the resulting points will be projected to the MLS surface of the closest point in the input cloud; the result is a point cloud with filled holes and a constant point density.

  .. image:: figures/mls_upsampling/mls_all_upsampling_methods.png
     :width: 900 px

  A quick timing analysis shows us that the running times are well within the 2 minutes as mentioned in the project requirements. Important to note is the fact that the bulk of the time (~35s) is spent on computing the MLS surface, and about 1-3s is spent on the actual upsampling. Thus, we can conclude that the quality improvements of the upsampling are well worth the additional ~5% increase in execution time.

  ======================   =======   ==================
  Upsampling method        Time(s)   Resulting # points
  ======================   =======   ==================
  NONE                     35        256.408
  SAMPLE_LOCAL_PLANE       36        2.051.264
  RANDOM_UNIFORM_DENSITY   36        740.510
  VOXEL_GRID_DILATION      38        1.225.989
  ======================   =======   ==================

  A more conclusive test would be to take a Kinect cloud of a wall at a distance where the noise is accentuated (~3m) and try to fit a plane in each of the resulting upsampled clouds. In order to make the experiment more realistic, we took the picture of the wall at an angle, such that the quantization effects would increase along the wall. The numeric results are the following:

  ====================== ================== =========
  Upsampling method      Cloud # points     % inliers
  ====================== ================== =========
  original               275.140            81.3
  NONE                   275.140            81.1 
  SAMPLE_LOCAL_PLANE     2.201.120          81.2
  RANDOM_UNIFORM_DENSITY 732.186            73
  VOXEL_GRID_DILATION    1.050.394          73
  ====================== ================== =========

  Unfortunately these numerical values do not represent the actual quality of the fit, because of the varying point density across the cloud in the different upsampling methods (i.e., the parts of the wall closer to the sensor had a larger density and precision in the original cloud, and as points get farther from the sensor, the sparsity and noise increase; BUT in VOXEL_GRID_DILATION and RANDOM_UNIFORM_DENSITY, the density is constant across the cloud, meaning that the noisy part of the wall has the same amount of points as the more precise part).

  As such, in order to analyze the quality of the fit, we do a visual analysis of the inliers/outliers ratio, as shown in the following pictures:

  **Original** cloud and its plane inliers

  .. image:: figures/mls_upsampling/original_inliers.png
     :width: 400 px


  **NONE** cloud and its plane inliers

  .. image:: figures/mls_upsampling/none_inliers.png
     :width: 400 px


  **SAMPLE_LOCAL_PLANE** cloud and its plane inliers

  .. image:: figures/mls_upsampling/slp_inliers.png
     :width: 400 px


  **RANDOM_UNIFORM_DENSITY** cloud and its plane inliers

  .. image:: figures/mls_upsampling/random_inliers.png
     :width: 400 px


  **VOXEL_GRID_DILATION** and its plane inliers

  .. image:: figures/mls_upsampling/vgd_inliers.png
     :width: 400 px

  The conclusion is that the VOXEL_GRID_DILATION method behaves the best, as it has the least holes out of all the options.


  So this is a wrap-up for the MLS smoothing. Next, I shall be looking into image-based hole-filling and how this can be applied to our problem. This will involve some experiments using MatLab and adding some code into the PCL I/O framework.


.. blogpost::
  :title: Joint Bilateral Upsampling for the Kinect
  :author: aichim
  :date: 02-26-2012

  Time for a new blog post. Lately, I have been working on image-based approaches for solving our smoothing and surface reconstructions problems. A straight-forward, but very effective method I wanted to implement for a long time is the one in:
  
  * Johannes Kopf, Michael Cohen, Dani Lischinski, and Matt Uyttendaele - Joint Bilateral Upsampling, ACM Transactions on Graphics (Proceedings of SIGGRAPH 2007)

  The idea behind is to use the RGB image in order to enhance the depth image, in a joint bilateral filtering, based on the following formula:

  :math:`\tilde{S}_p = \frac{1}{k_p} \sum_{q_d \in \Omega} {S_{q_d} f(||p_d - q_d|| g(||\tilde{I}_p-\tilde{I}_q||})`

  where, in our case, :math:`S` is the depth image and :math:`\tilde{I}` is the RGB image.

  The nice thing about it is the fact that we can use the 15Hz mode of the Kinect in order to produce high resolution (1280x1024 px) RGB images and normal (640x480 px) depth images. By using this method, we can obtain 1280x960 depth images. I faced some problems with the registration of depth to high-res RGB images, so the results below show just the case of 640x480 depth and color images.


  .. image:: figures/image_based/low_res_color.png
     :width: 900 px

  .. image:: figures/image_based/low_res_random.png
     :width: 900 px

  As you can see, there are a lot of new points in the filtered point cloud (168152 vs 141511 in the input), no noisy points, and their positions are coherent with their neighbors.

  Just as a teaser, an example of the high-resolution image-based upsampling (will come back with more results once we solve the registration problem mentioned above):


  .. image:: figures/image_based/high_res_unregistered.png
     :width: 900 px


  Also, in the meantime, I have spent a frustratingly large amount of time fixing the Poisson implementation we had in PCL. It seems that there was a memory error in the version a Google Summer of Code participant introduced in PCL, so in the next days we shall try to bring the newer version in.



.. blogpost::
  :title: Updates
  :author: aichim
  :date: 03-11-2012

  I finally managed to put Misha's Poisson reconstruction implementation in PCL. It now works properly and passes the unit tests, with the same behavior as the original application. Futhermore, we are adapting it to the PCL coding style.

  A new pcl::surface base class has been introduced, due to some confusion communicated via the pcl-users mailing lists. Now, there is the CloudSurfaceProcessing class, which represents the base class for algorithms that take a point cloud as an input and produce a new output cloud that has been modified towards a better surface representation. These types of algorithms include surface smoothing, hole filling, cloud upsampling etc.

  In this category, we have the MovingLeastSquares algorithm and its additional features we mentioned in previous blog posts and the all-new BilateralUpsampling algorithm, based on the bilateral filter (for more details about the workings of the algorithm, please see the previous post). Suat was kind enough to help me by modifying the OpenNIGrabber such that it will output 1280x1024 PointXYZRGB clouds when the Kinect is set to high-resolution RGB -> this means that each second row and column contains nan depth values. And here is where the BilateralUpsampling comes in, using the RGB info to fill in the missing depth data, as visualized in the following example image:

  .. image:: figures/screen_hole_filling.png
     :width: 900 px



.. blogpost::
  :title: Wrapping up - Reports and Presentation
  :author: aichim
  :date: 04-12-2012

  As a final blog post for this Toyota Code Sprint, I am attaching the final report I have written for the sponsors.

  .. raw:: html

    <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftocs%2Faichim%2Ffiles%2Ftocs_final_ichim.pdf%3Fraw%3Dtrue&embedded=true" width="600" height="800" style="border: none;"></iframe></center>
	
	
  The presentation slides associated with the report:	
	
  .. raw:: html

    <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftocs%2Faichim%2Ffiles%2FTOCS_final_presentation.pdf%3Fraw%3Dtruehttps%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftocs%2Faichim%2Ffiles%2FTOCS_final_presentation.pdf%3Fraw%3Dtrue&embedded=true" width="600" height="800" style="border: none;"></iframe></center>
    
  And the midterm report:	
	
  .. raw:: html

    <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftocs%2Faichim%2Ffiles%2Ftocs_midterm_aichim_v2.pdf%3Fraw%3Dtrue&embedded=true" width="600" height="800" style="border: none;"></iframe></center>
    

.. blogpost::
  :title: TOCS 2.0 - Superquadrics
  :author: aichim
  :date: 10-25-2013


  I am pleased to announce that I just finished the second Toyota Code Sprint. The topic we tackled this time was superquadrics applied for Computer Vision tasks such as object modeling and object detection.

  The code we developed was not yet integrated into PCL (it does use PCL for all the processing), but lies in a separate repository which you can find here: https://github.com/aichim/superquadrics .

  An extensive report that presents some of the theory behind the concepts and algorithms we used in the project, as well as implementation details and results can be found at the end of this post.

  At the end of this work, we present a set of ideas that can be used to extend the project in subsequent code sprints:

  * performance evaluation of the superquadric-based algorithms compared to other state-of-the-art object modeling and object detection approaches and integrating my code into PCL if the results are satisfactory

  * explore further possibilities of object modeling inside point clouds. This includes techniques different from superquadrics (see the report for refecences), or improving superquadric-based techniques (see supertoroids, deformable superquadrics)

  * in this code sprint we explored one approach for multipart object segmentation using superquadrics. This technique is valuable for point cloud compression, but not very efficient nor robust. More work in this direction can bring interesting results.

  * more robust and efficient object fitting using superquadrics - right now we use only the 3d location of the points, but the quality of the fitting can be improved by using normal and/or curvature information.

  * considering that most of the scans of objects will not cover the complete sphere of possible views, we should think about how to fit only partial superquadrics


  .. raw:: html

    <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftocs%2Faichim%2Ffiles%2FAlex_Ichim_TOCS2_superquadrics_report.pdf%3Fraw%3Dtrue&embedded=true" width="600" height="800" style="border: none;"></iframe></center>
