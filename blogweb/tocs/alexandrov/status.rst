My status updates
=================


.. blogpost::
  :title: Table-top Object Segmentation
  :author: alexandrov
  :date: 07-31-2013

  Hi! This is my first blog post in the scope of "Segmentation/Clustering of
  Objects is Cluttered Environments" code sprint. Today I would like to discuss
  the problem of table-top object segmentation and the tools that PCL currently
  has to offer.

  Consider a situation when a robot observes a table with several objects
  standing on top of it. Assuming that the table surface is flat and is large
  relative to the objects' size, and also that the objects do not touch each
  other (either standing side-by-side or on top of each other), the following
  standard simple pipeline allows to break the scene into objects:

  #. Detect dominant plane.
  #. Find a polygon (convex hull) that encloses the points that belong to the
     detected plane.
  #. Extract the points that are above the found polygon (i.e. that are
     supported by the dominant plane).
  #. Perform Euclidean clustering of the extracted points.

  PCL offers components to solve each of the mentioned subtasks. After putting
  them together and tweaking several involved parameters, one could get an
  object segmentation system that works reasonably well under aforementioned
  assumptions. The problem, though, is that those assumptions are far too
  restrictive.

  A typical real-world table with objects on top of it will be more challenging.
  Just have a look at your desk. The main properties are: a) the number of
  objects is unknown and is potentially very large; b) the objects do not stand
  in isolation, but rather touch and occlude each other; c) there might be
  several supporting planes, for example shelves or large objects that support
  smaller objects.

  Unfortunately, there is nothing in PCL that could bring you anywhere close to
  segmenting such a scene. The available components include:

  * ``MinCutSegmentation``
    (
    `doc <http://docs.pointclouds.org/trunk/min__cut__segmentation_8h_source.html>`__,
    `tutorial <http://pointclouds.org/documentation/tutorials/min_cut_segmentation.php>`__
    )
  * ``GrabCut``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_grab_cut.html>`__
    )
  * ``CRFSegmentation``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_crf_segmentation.html>`__,
    `blog <http://www.pointclouds.org/blog/gsoc12/cpotthast/all.php>`__
    )
  * ``SeededHueSegmentation``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_seeded_hue_segmentation.html>`__
    )
  * ``RegionGrowing``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_region_growing.html>`__,
    `tutorial <http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php>`__
    )
  * ``RegionGrowingRGB``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_region_growing_r_g_b.html>`__,
    `tutorial <http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php>`__
    )
  * ``ConditionalEuclideanClustering``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_conditional_euclidean_clustering.html>`__,
    `tutorial <http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php>`__
    )
  * ``OrganizedConnectedComponentSegmentation``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_organized_connected_component_segmentation.html>`__
    )
  * ``SupervoxelClustering``
    (
    `doc <http://docs.pointclouds.org/trunk/classpcl_1_1_supervoxel_clustering.html>`__,
    `tutorial <http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php>`__
    )

  The first two methods are supposed to break a scene into foreground and
  background. Therefore unless one knows the number and approximate locations of
  all objects, these are of no use.

  ``CRFSegmentation``, as far as I understand, represents an unfinished work and
  also requires an initial guess (to which cluster it belongs) for each point in
  the dataset.

  Next five modules implement region-growing segmentation.
  ``SeededHueSegmentation`` requires seeds and considers only Euclidean distance
  and difference in hue. ``RegionGrowing`` considers Euclidean distance and
  normal orientations, whereas ``RegionGrowingRGB`` works with differences in
  RGB color instead of normals. ``ConditionalEuclideanClustering`` may use any
  user-defined comparison function to judge whether two points belong to one
  segment. Finally, ``OrganizedConnectedComponentsSegmentation`` is similar to
  the previous one, but is crafted for organized (Kinect-style) point clouds.
  According to my experiments, none of these modules is able to come up with a
  reasonable segmentation of a real-world table-top scene.

  ``SupervoxelClustering`` is the latest addition to PCL. As its name suggests,
  it can be used to split (oversegment) a point cloud into a set of supervoxels.
  Th algorithm uses geometrical structure, normals, and RGB values of points.
  The results I get are very nice and object boundaries are typically preserved
  (i.e. a single cluster does not span over multiple objects), however further
  processing is required to merge clusters into objects.

  To conclude, there are tools in PCL that one can use to segment trivial
  table-top scenes. However, there is no module that could segment a complex
  real-world table-top scene out-of-the-box. Therefore, design of such a module
  will be the main focus of this code sprint.


.. blogpost::
  :title: Supervoxel Segmentation
  :author: alexandrov
  :date: 08-03-2013

  In the previous blog post I briefly mentioned ``SupervoxelSegmentation``
  algorithm that has recently become available in PCL. Today I would like to
  discuss it in a greater detail.

  In a nutshell, ``SupervoxelSegmentation`` generates an over-segmentation of a
  3D point cloud into small spatially compact regions (supervoxels) in which all
  points possess similar local low-level features (such as color, normal
  orientation). The main properties of the algorithm are that the supervoxels
  are evenly distributed across 3D space and (in most cases) do not cross object
  boundaries. A detailed explanation of the algorithm could be found in `this
  tutorial <http://www.pointclouds.org/documentation/tutorials/supervoxel_clustering.php>`__
  and in the original paper:

  * J. Papon, A. Abramov, M. Schoeler, F. Wörgötter
    `"Voxel Cloud Connectivity Segmentation - Supervoxels for Point Clouds" <http://www.cv-foundation.org/openaccess/content_cvpr_2013/papers/Papon_Voxel_Cloud_Connectivity_2013_CVPR_paper.pdf>`__
    In Proc. of CVPR, 2013

  Let's consider a scene with a cluttered table from `Object Segmentation Database (OSD) <http://users.acin.tuwien.ac.at/arichtsfeld/?site=4>`__:

  +-------------------------------+-----------------------------------------+
  | .. image:: img/osd/test55.png | .. image:: img/osd/test55-voxels.png    |
  |   :width: 320 px              |   :width: 320 px                        |
  +-------------------------------+-----------------------------------------+
  | Color image                   | Voxelized point cloud (voxel size 6 mm) |
  +-------------------------------+-----------------------------------------+

  Here are the results of supervoxel segmentation with two different seed sizes
  (0.1 m, which is the default, and 0.03 m):

  +-----------------------------------------------+---------------------------------------------------------+
  | .. image:: img/osd/test55-010-supervoxels.png | .. image:: img/osd/test55-010-supervoxels-adjacency.png |
  |   :width: 320 px                              |   :width: 320 px                                        |
  +-----------------------------------------------+---------------------------------------------------------+
  | Supervoxel segmentation (seed size 0.1 m)     | The same with overlaid adjacency graph                  |
  +-----------------------------------------------+---------------------------------------------------------+

  +-----------------------------------------------+---------------------------------------------------------+
  | .. image:: img/osd/test55-003-supervoxels.png | .. image:: img/osd/test55-003-supervoxels-adjacency.png |
  |   :width: 320 px                              |   :width: 320 px                                        |
  +-----------------------------------------------+---------------------------------------------------------+
  | Supervoxel segmentation (seed size 0.03 m)    | The same with overlaid adjacency graph                  |
  +-----------------------------------------------+---------------------------------------------------------+

  The over-segmented point clouds look like patchwork sheets. The smaller the
  seed size, the smaller the patches are. The white meshes represent the
  adjacency graph of the supervoxels. It is not immediately obvious which seed
  size would result in a better final object segmentation. On one hand, smaller
  patches are more likely to reflect true object boundaies. Furthermore, in
  order to segment small objects the patch size should not be greater that the
  smallest object we wish to segment. On the other hand, smaller patches mean
  more edges and more time to process them. Depending on the asymptotic
  complexity of the merging algorithm that could become a crucial consideration.

  The next step after supervoxel segmentation is to merge supervoxels to obtain
  final object segmentation. To begin with, I decided to implement the algorithm
  proposed in the following paper:

  * P. Felzenszwalb, D. Huttenlocher
    `"Efficient Graph-Based Image Segmentation" <http://www.cs.cornell.edu/~dph/papers/seg-ijcv.pdf>`__
    Int. Journal of Computer Vision, 2004

  The basic idea is to assign each edge in the adjacency graph a weight that
  expresses the difference (or dissimilarity) between the pair of supervoxels that it
  connects. The algorithm starts by sorting the edges in the non-decreasing
  weight order and also creates disjoint sets, where each set contains
  supervoxel belonging to the same object. In the beginning each supervoxel gets
  its own set. Then the algorithm iterates over all edges and unites the sets to
  which the supervoxels it connects belong if a certain condition holds. The
  condition I use is that the weight of the edge should be small compared to the
  weights of the edges between the supervoxels that are already in the sets.
  This algorithm is actually a modification of `Kruskal's algorithm
  <http://en.wikipedia.org/wiki/Kruskal%27s_algorithm>`_ for finding a minimum
  spanning tree in a connected weighted graph.

  Defining a good difference function for a pair of supervoxels is crucial for
  the performance. It should consider all the available information about
  supervoxels, including their size, area, border, orientation, color, and so on.
  At the moment I use relatively simple function which only depends on the
  orientation of the supervoxels. More specifically, if two supervoxels have
  centroids :math:`C_i` and :math:`C_j` and average normals :math:`N_i` and
  :math:`N_j`, then the difference is:

  :math:`D_{i,j} = \begin{cases} 1-abs\left(N_{i}\cdot N_{j}\right)\qquad & if\,\left(C_{i}-C_{j}\right)\cdot N_{i}<0\\ 0 & otherwise \end{cases}`

  In other words, if the supervoxels are relatively concave to each other, then
  the dissimilarity is proportional to the angle between their normals.
  Otherwise it is zero, i.e. relatively convex supervoxels are always similar.

  Here are the results of running the algorithm on the supervoxel segmentations
  that were demonstrated before. The figures on the right show the edges that
  the algorithm kept (used to unite sets) on top of the object clusters:

  +--------------------------------------------+--------------------------------------------------+
  | .. image:: img/osd/test55-010-clusters.png | .. image:: img/osd/test55-010-clusters-edges.png |
  |   :width: 320 px                           |   :width: 320 px                                 |
  +--------------------------------------------+--------------------------------------------------+
  | Object segmentation (seed size 0.1 m)      | The same with overlaid edge graph                |
  +--------------------------------------------+--------------------------------------------------+

  +--------------------------------------------+--------------------------------------------------+
  | .. image:: img/osd/test55-003-clusters.png | .. image:: img/osd/test55-003-clusters-edges.png |
  |   :width: 320 px                           |   :width: 320 px                                 |
  +--------------------------------------------+--------------------------------------------------+
  | Object segmentation (seed size 0.03 m)     | The same with overlaid edge graph                |
  +--------------------------------------------+--------------------------------------------------+

  There are a number of problems in both cases. For the large supervoxel size
  case the book at the bottom is united with the box on top of it, and the
  second book is segmented into two parts and merged with the table and the box
  on top of it. Also the saucepan in the back is joined with the table surface.
  The segmentation with more fine-grained supervoxels exposes different
  problems. Here the tetra-pak is joined with the box it stands on, and also the
  box nearby it is merged with the table surface.  Moreover, there are several
  single-supervoxel "objects" that were not joined to any other cluster.
  Finally, both cases share other two problems: the table is split into many
  pieces, and the bowl on the left is split into two parts.

  I think these initial results are rather good, especially considering the fact
  that a very simple dissimilarity measure is used. In the next blog post I plan
  to zoom in at the problematic areas and discuss how the dissimilarity function
  could be improved to solve the observed problems.


.. blogpost::
  :title: How Supervoxels Grow
  :author: alexandrov
  :date: 08-06-2013

  As I mentioned in the previous blog post, I started to zoom in at the areas
  that cause problems for the segmentation/clustering algorithms. This made me
  explore how supervoxels actually grow, and in this short post I would like to
  share the gained insights.

  Here is a part of the scene I used in the previous blog post observed from a
  slightly different viewpoint. On the left is the voxelized point cloud and on
  the right is the result of supervoxel segmentation with the default seed size
  (click on the image to see full-resolution version):

  +--------------------------------------------+-------------------------------------------+
  | .. image:: img/03/voxels.png               | .. image:: img/03/supervoxels.png         |
  |   :width: 320 px                           |   :width: 320 px                          |
  |   :target: ../../_images/voxels.png        |   :target: ../../_images/supervoxels.png  |
  +--------------------------------------------+-------------------------------------------+
  | Voxelized point cloud (voxel size 0.006 m) | Supervoxel segmentation (seed size 0.1 m) |
  +--------------------------------------------+-------------------------------------------+

  One thing that caught my attention in this segmentation is the cyan supervoxel
  in the central part of the image. If you examine it carefully you will mention
  that it consists of two disjoined parts: a big blob to the south-west of the
  center and a few voxels that are exactly in the center of the image. The
  distance between these two parts is quite large compared to the seed size, so
  I became curious about how exactly this segmentation came about. Even though
  the `tutorial <http://www.pointclouds.org/documentation/tutorials/supervoxel_clustering.php>`_
  and the `paper <http://www.cv-foundation.org/openaccess/content_cvpr_2013/papers/Papon_Voxel_Cloud_Connectivity_2013_CVPR_paper.pdf>`_
  provide a detailed explanation of the process, I decided to visualize it to
  gain a better understanding.

  Below is an animated GIF where frames show supervoxels after each of the 28
  iterations made by the algorithm to segment the input point cloud:

  +-----------------------------------------+
  | .. image:: img/03/supervoxel-growth.gif |
  |   :width: 640 px                        |
  +-----------------------------------------+
  | Supervoxel growth (seed size 0.1 m)     |
  +-----------------------------------------+

  The wavefront of the blue supervoxel chases the wavefront of the cyan one.  At
  some point in time it breaks it into two parts and eventually "kills" the left
  one. The right one survives, presumably because its voxels are on the vertical
  surface of the box, which is similar to the rest of the cyan cluster.
  Therefore the blue supervoxel (which is mostly "horizontal") can not seize
  them.

  Although this particular case does not seem to be much of a problem, I could
  imagine a situation when a supervoxel is split into two or more chunks that
  are equally large. The centroid and normal of such a "broken" supervoxel will
  make no geometrical sense. Therefore reasoning about them in further
  processing steps will also make no sense and lead to random results.

  How to cure this problem is an open question. We could require that
  supervoxels stay connected during expansion, though it might be
  computationally expensive to enforce. Alternatively, we could add a
  post-processing step which will split disjoined supervoxels into several
  smaller supervoxels. I put this issue in the todo-list and will come back to
  it later.


.. blogpost::
  :title: Simple Supervoxel Refinement
  :author: alexandrov
  :date: 08-09-2013

  In this blog post I will discuss one of the problems with supervoxel
  segmentation and a simple refinement procedure that resolves it.

  Let us consider exactly the same part of the scene as in the previous blog
  post. For reference here are (again) the voxelized point cloud and object
  segmentation that I am getting:

  +--------------------------------+-------------------------------------------+
  | .. image:: img/03/voxels.png   | .. image:: img/04/test55-010-clusters.png |
  |   :width: 320 px               |   :width: 320 px                          |
  +--------------------------------+-------------------------------------------+
  | | Voxelized point cloud (voxel | | Object segmentation (seed               |
  | | size 0.006 m)                | | size 0.1 m) with overlaid               |
  | |                              | | edge graph                              |
  +--------------------------------+-------------------------------------------+

  Below is the output of supervoxel segmentation. It is exacltly the same as in
  the previous post, though the colors are different (because they are chosen
  randomly on each execution). Additionally the seeds from which supervoxels
  evolved and their centroids are visualized:

  +------------------------------------------------------------+
  | .. image:: img/04/test55-010-supervoxels.png               |
  |   :width: 640 px                                           |
  +------------------------------------------------------------+
  | | Supervoxel segmentation (seed size 0.1 m) with overlaid  |
  | | supervoxel seeds (red), supervoxel centroids (blue), and |
  | | edge graph                                               |
  +------------------------------------------------------------+

  Last time I focused attention on the green and pink (formely cyan and blue)
  supervoxels in the central part of the image. The problem I mentioned is that
  the pink supervoxel split the green one into two disjoint parts.  Another
  problem I did not talk about is that both supervoxels extend over the edge
  between the book spine and cover. This is undesirable because spine and cover
  are two distinct surfaces with different geometrical properties.

  According to my understanding, the cause of this undesired segmentation is
  unlucky initial seeds. Both problematic supervoxels evolved from seeds which
  are on (or almost on) the edge between spine and cover. Right from the
  beginning both supervoxels started to absorb voxels from both surfaces. Only
  towards the end the pink supervoxel started to converge to the cover surface
  and the green one to the spine surface. But since the number of iterations is
  limited, the supervoxels never reached convergence. (Please see `here <../../_images/supervoxel-growth.gif>`__
  how the supervoxels evolved.)

  The described problems have a heavy impact on the final object segmentation.
  Firstly, the green supervoxel is considered to be adjacent to the dark green
  supervoxel in the center because of the tiny disjoint part we discussed last
  time. The orientations of these two supervoxels are similar (they both are
  vertical surfaces), therefore an edge is established and they are merged in
  the same object. (Unfortunately it is impossible to see in the image, but
  actually there is no edge from the dark green supervoxel to the pink one, but
  rather an edge from it to the green one, and then from the green one to the
  pink supervoxel.) Secondly, there is an edge between the pink supervoxel and
  the brown one below it because they are adjacent and have similar
  orientations. Was the supervoxel segmentation better, they would not touch
  each other and would not be connected. This explains why in the final object
  segmentation the table surface is merged with the book and the box on top of
  it.

  I see two ways to address this problem. First is to come up with a smart
  seeding algorithm which would ensure that the seeds do not land on the edges
  between surfaces and at the same time guarantee an even distribution of seeds
  over the space. Second way is to introduce a refinement step which would
  post-process the supervoxels output by the segmentation algorithm.

  In fact, there already exists a function ``SupervoxelSegmentation::refineSupervoxels()``.
  It has to be invoked explicitly after the initial segmentation is obtained.
  The function simply uses the centroids of supervoxels as seeds and runs the
  segmentation algorithm once again. So, in a sense, it does implement the
  "smart seeding" approach. The improvement is massive. Below are the results of
  supervoxel and object segmentation when a single round of refinement is used:

  +------------------------------------------------------+---------------------------------------------------+
  | .. image:: img/04/test55-010-refined-supervoxels.png | .. image:: img/04/test55-010-refined-clusters.png |
  |   :width: 320 px                                     |   :width: 320 px                                  |
  +------------------------------------------------------+---------------------------------------------------+
  | | Supervoxel segmentation (seed                      | | Object segmentation (seed                       |
  | | size 0.1 m, with refinement)                       | | size 0.1 m, with refinement)                    |
  +------------------------------------------------------+---------------------------------------------------+

  The final result is very good, no object is merged with any other one or the
  table. Unfortunately, I do not get equally good segmentations of other
  cluttered scenes. In the next blog post I will demonstrate them and discuss
  how this simple refinement step could be improved.


.. blogpost::
  :title: Supervoxel Refinement with Splitting
  :author: alexandrov
  :date: 08-12-2013

  Last time I mentioned that the simple refinement procedure does not always
  lead to good results. In this blog post will I discuss some of the problems
  and show an improved refinement procedure that addresses them.

  Here is another scene with a cluttered table from `Object Segmentation Database (OSD) <http://users.acin.tuwien.ac.at/arichtsfeld/?site=4>`__:

  +-------------------------------+-----------------------------------------+
  | .. image:: img/osd/test56.png | .. image:: img/05/voxels.png            |
  |   :width: 320 px              |   :width: 320 px                        |
  +-------------------------------+-----------------------------------------+
  | | Color image of the scene    | | Fragment of the voxelized             |
  |                               | | point cloud (voxel size 6 mm)         |
  +-------------------------------+-----------------------------------------+

  I will concentrate on the stack with two books and two boxes in the
  foreground. The figures below demonstrate the results of supervoxel
  segmentation without and with one round of simple refinement:

  +----------------------------------------------------+---------------------------------------------------+
  | .. image:: img/05/supervoxels-no-refine.png        | .. image:: img/05/supervoxels-1-refine.png        |
  |   :width: 320 px                                   |   :width: 320 px                                  |
  |   :target: ../../_images/supervoxels-no-refine.png |   :target: ../../_images/supervoxels-1-refine.png |
  +----------------------------------------------------+---------------------------------------------------+
  | | Supervoxel segmentation (seed size 0.1 m) without (left) and                                         |
  | | with (right) refinement. Supervoxel centroids are overlaid.                                          |
  | | Some supervoxels are enumerated on the left image.                                                   |
  +--------------------------------------------------------------------------------------------------------+

  There are many deficits in segmentation output if no refinement is used. #3 is
  split into two large parts and #8 has a small disjoint region. #2, #5, and #9
  each cover two distinct surfaces. The simple refinement improves segmentation
  in some respects (#1 and #4 became better, #8 no longer has a disjoint part),
  however it fails to "join" #3 and, what's more, splits #5 and #7. Performing
  more rounds of the simple refinement does not help and e.g. after 5 iterations
  all three supervoxels remain disjoint.

  Obviously, the simple refinement can only help if after segmentation the
  supervoxel centroid moved to a new spot which is better than the original
  seed. Unfortunately for supervoxel #3 the centroid ends up somewhere in
  between the two parts, therefore the seed does not move significantly, and the
  same result is reproduced over and over again.

  The simple refinement consists of creating new seeds from the centroids of
  supervoxels and running the segmentation algorithm again. I decided to improve
  it by splitting disjoint supervoxels prior to re-seeding. In each supervoxel I
  compute `connected components <http://en.wikipedia.org/wiki/Connected_component_(graph_theory)>`_
  (in terms of Euclidean distance). Each connected component that is not too
  small (has at least 15 points) defines a new seed voxel.

  Below is an animated GIF where frames show supervoxels after each round of
  the improved refinement procedure. The number in the upper right corner gives
  round number (0 means before refinement):

  +----------------------------------------------------+
  | .. image:: img/05/supervoxel-refinement.gif        |
  |   :width: 640 px                                   |
  +----------------------------------------------------+
  | | 5 rounds of supervoxel refinement with splitting |
  +----------------------------------------------------+

  In the first iteration the yellow supervoxel is successfully split into two
  parts. As mentioned before, this refinement iteration degrades segmentation by
  making two other supervoxels disjoint. But this is fixed in the next iteration
  of refinement when they are also split. In the last two iterations the number
  of supervoxels does not change anymore, they only re-shape slightly.

  The obtained supervoxel segmentation is far from being perfect. Nevertheless,
  it was significantly improved using updated refinement procedure.


.. blogpost::
  :title: Under-segmentation Error
  :author: alexandrov
  :date: 08-19-2013

  I concluded the previous blog post with a claim that the improved refinement
  procedure yields a better segmentation. That conclusion was based purely on a
  visual inspection and rang a bell reminding that it is time to start thinking
  about quantitative evaluation metrics and prepare a tool set to preform
  evaluations. In this blog post I will describe my first steps in this
  direction.

  Let us start with a prerequisite for any evaluation, ground truth data. So far
  I have been using scenes from `Object Segmentation Database (OSD) <http://users.acin.tuwien.ac.at/arichtsfeld/?site=4>`__.
  This dataset contains a ground truth annotation for each point cloud, however
  I am not completely satisfied with its quality. Consider the following point
  cloud (left) and the provided annotation (middle):

  +----------------------------------------+--------------------------------------------+-------------------------------------------+
  | .. image:: img/06/test1-pointcloud.png | .. image:: img/06/test1-osd-annotation.png | .. image:: img/06/test1-my-annotation.png |
  |   :width: 210 px                       |   :width: 210 px                           |   :width: 210 px                          |
  +----------------------------------------+--------------------------------------------+-------------------------------------------+
  | | Color point cloud                    | | Original ground                          | | Improved ground                         |
  |                                        | | truth annotation                         | | truth annotation                        |
  +----------------------------------------+--------------------------------------------+-------------------------------------------+

  Some of the points in the close vicinity of the object boundaries are wrongly
  annotated. For example, almost all points on the left edge of the standing box
  are annotated as belonging to the table. I hypothesize that the annotation was
  obtained automatically using some color image segmentation algorithm. (Thus it
  is not really "wrong", it is correct in terms of the color data.) As we are
  working with 3D point clouds, the spatial relations derived from the depth
  data should have larger weight than color and the annotation should be revised
  to respect geometrical boundaries of the objects. Unfortunately, there is no
  tool in PCL that would allow to load a point cloud and edit the labels. It
  costed me quite a few hours of work to put together a simple editor based on
  ``PCLVisualizer`` that could do that. An example of refined annotation is
  shown in the figure above (right).

  The authors of ``SupervoxelSegmentation`` used two metrics to evaluate how
  nicely the supervoxels adhere to the object boundaries: *under-segmentation error*
  and *boundary recall*. These metrics were introduced in:

  * A. Levinshtein, A. Stere, K. Kutulakos, D. Fleet, S. Dickinson, K. Siddiqi
    `"TurboPixels: Fast Superpixels Using Geometric Flows" <http://www.cs.toronto.edu/~sven/Papers/turbopixels.pdf>`__
    IEEE Transactions on Pattern Analysis and Machine Intelligence, 2009

  I decided to begin with the under-segmentation error. Formally, given a ground
  truth segmentation into regions :math:`g_1,\dots,g_M` and a supervoxel
  segmentation into supervoxels :math:`s_1,\dots,s_K`, the under-segmentation
  error for region :math:`g_i` is defined as:

  :math:`E_{i} = \frac{\left[\sum_{\left\{s_j | s_j \bigcap g_i \neq\emptyset\right\}}Size(s_j)\right] - Size(g_i)}{Size(g_i)}`

  Simply put, it takes a union of all the supervoxels that overlap with a given
  ground truth segment and measures how much larger its total size is than the
  size of the segment. The authors of ``SupervoxelSegmentation`` use a slightly
  modified version which summarizes the overall error in a single number:

  :math:`E = \frac{1}{N}\left[\sum^{M}_{i=1}\left(\sum_{\left\{s_j | s_j \bigcap g_i \neq\emptyset\right\}}Size(s_j)\right) - N\right]`,

  where :math:`N` is the total number of voxels in the scene. In practice, this
  error sums up the sizes of all the supervoxels that cross the ground truth
  boundaries.

  In my opinion, this definition biases error with the average supervoxel size.
  Consider the supervoxel segmentation (left):

  +---------------------------------------------+-------------------------------------------+------------------------------------------+
  | .. image:: img/06/test1-010-supervoxels.png | .. image:: img/06/test1-010-2overlaps.png | .. image:: img/06/test1-010-my-error.png |
  |   :width: 210 px                            |   :width: 210 px                          |   :width: 210 px                         |
  +---------------------------------------------+-------------------------------------------+------------------------------------------+
  | | Supervoxel                                | | Supervoxels that                        | | Erroneous voxels                       |
  | | segmentation                              | | cross ground truth                      | | according to the                       |
  | | (seed size 0.1 m)                         | | boundaries (red)                        | | proposed error                         |
  |                                             |                                           | | definition (red)                       |
  +---------------------------------------------+-------------------------------------------+------------------------------------------+

  Clearly, the segmentation is rather good in terms of the border adherence. The
  only failure is in the right corner of the lying box, where the green
  supervoxel "bleeds" on the table. The middle image shows which voxels will be
  counted towards the error, i.e. those supervoxels that cross the ground truth
  boundaries. In fact, **every** supervoxel in the close vicinity of an edge is
  counted. The reason is simple: the edge between two surfaces in a typical
  point cloud obtained with an RGB-D camera is ambiguous. The ground truth
  segmentation is therefore random to some extent and the chance that
  the supervoxel boundary will exactly coincide with the ground truth segment
  boundary is close to zero. Consequently, the error always includes all the
  supervoxels around the boundary, and the larger they are on average, the
  larger the error is. The "real" error ("bled" green supervoxel) is completely
  hindered with it.

  I decided to use a modified definition. Whenever a supervoxel crosses ground
  truth boundary(ies), it is split in two (or more) parts. I assume that the
  largest part is correctly segmented, and only count the smaller parts as
  erroneous. The figure above (right) demonstrates which voxels would be counted
  towards the error in this case. Still, there is some "noise" around
  boundaries, but it does not hinder the "real" error. I also do not like that
  the error in the original definition is normalized by the total point cloud
  size. I think that the normalization should be related to the amount of
  objects or the amount (length) of object boundaries. I will consider this
  options in future, but for now I just count the number of erroneous pixels and
  do not divide it by anything.

  I would like to conclude with the results of evaluating simple and improved
  supervoxel refinement procedures using the described error metric. The figure
  below shows the voxels considered as erroneous (in red) in supervoxel
  segmentations obtained with 2 rounds of simple refinement (left) and 2 rounds
  of improved refinement (right):

  +--------------------------------------------+-------------------------------------------+
  | .. image:: img/06/test56-use-simpleref.png | .. image:: img/06/test56-use-splitref.png |
  |   :width: 320 px                           |   :width: 320 px                          |
  +--------------------------------------------+-------------------------------------------+
  | | Under-segmentation error                 | | Under-segmentation error                |
  | | (seed size 0.1 m, with                   | | (seed size 0.1 m, with                  |
  | | simple refinement)                       | | improved refinement)                    |
  +--------------------------------------------+-------------------------------------------+

  The under-segmentation error without refinement is 15033. Simple refinement
  reduces it to 11099 after the first iteration and 10755 after the second.
  Improved refinement reduces it to 9032 and 8373 respectively. I think the
  numbers do agree with the intuition, so I will continue using this metric in
  future.


.. blogpost::
  :title: Segmentation using Random Walks
  :author: alexandrov
  :date: 08-28-2013

  In the last few weeks I have been working on two things in parallel. On one
  hand, I continued to improve supervoxel segmentation, and a description of
  this is due in a later blog post. On the other hand, I started to look into an
  alternative approach to point cloud segmentation which uses random walkers.
  In this blog post I will discuss this approach and show my initial results.

  The random walker algorithm was proposed for interactive image segmentation in:

  * L. Grady `"Random Walks for Image Segmentation" <http://webdocs.cs.ualberta.ca/~nray1/CMPUT615/MRF/grady2006random.pdf>`__
    IEEE Transactions on Pattern Analysis and Machine Intelligence, 2006

  The input is an image where several pixels are marked (by the user) with
  different labels (one for each object to be segmented out) and the output is a
  label assignment for all the remaining pixels. The idea behind the algorithm
  is rather intuitive. Think of a graph where the nodes represent image pixels.
  The neighboring pixels are connected with edges. Each edge is assigned a
  weight which reflects the degree of similarity between pixels. As it was
  mentioned before, some of the nodes are marked with labels. Now take an
  unlabeled node and imagine that a random walker is released from it. The
  walker randomly hops to one of the adjacent nodes with probability
  proportional to the edge weight. In the process of this random walk it will
  occasionally visit labeled nodes. The one that is most likely to be visited
  first determines the label of the node where the walker started.

  Luckily, one does not have to simulate random walks from each node to obtain
  the probabilities of arriving at labeled nodes. The probabilities may be
  calculated analytically by solving a system of linear equations. The matrix of
  coefficients consists of edge weights and their sums, arranged in a certain
  order. The author did a great job explaining why this is so and how exactly
  the system should be constructed, so those who are interested are referred to
  the original paper.

  Originally, the algorithm was applied to segment 2D images, however it could
  be used for any other data as long as there is a way to model it using a
  weighted graph. For us, of course, 3D point clouds are of a particular
  interest.  The following paper describes how random walker segmentation could
  be applied for meshes or point clouds:

  * Y. Lai, S. Hu, R. Martin, P. Rosin
    `"Rapid and Effective Segmentation of 3D Models using Random Walks" <http://users.cs.cf.ac.uk/Yukun.Lai/papers/cagd09.pdf>`__
    Computer Aided Geometric Design, 2009

  The authors construct the graph from a point cloud is the following way. Each
  point :math:`p_i` in the cloud becomes a node :math:`v_i` in the graph and the
  normal vector :math:`n_i` is computed for it.

  For a pair of nodes :math:`v_i` and :math:`v_j` two distances are defined:

  * Euclidean distance between points :math:`d_1(v_i,v_j) = ||p_i-p_j||^2`

  * Angular distance between normals :math:`d_2(v_i,v_j) = \frac{\eta}{2}||n_i-n_j||^2`

  In the angular distance :math:`\eta` is a coefficient which depends on the
  relative concavity between points. For the convex case it is set to
  :math:`0.2`, effectively discounting the difference, whereas for the concave
  case it is equal to :math:`1.0`.

  Using *K*-nearest neighbors search the neighborhood :math:`N(v_i)` of a node
  is established. An edge is created between the node and each other node in the
  neighborhood. The weight of the edge depends on the two distances and is
  defined as:

  :math:`w_{ij} = \exp{\left\{-\frac{d_1(v_i,v_j)}{\sigma_1\bar{d_1}}\right\}\cdot\exp\left\{-\frac{d_2(v_i,v_j)}{\sigma_2\bar{d_2}}\right\}}`

  :math:`\sigma_1` and :math:`\sigma_2` are used to balance the contributions of
  different distances. :math:`\bar{d_1}` and :math:`\bar{d_2}` in the
  denominators stand for the mean values of the distances over the whole point
  cloud.

  In my implementation I decided to voxelize point cloud like it is done in
  ``SupervoxelSegmentation``. I also reused the ``OctreePointCloudAdjacency``
  class contributed by Jeremie Papon to establish the neighborhood relations
  between voxels. The weight is computed exactly as it is proposed by Lai *et
  al*. Finally, I use the ``Sparse`` module of ``Eigen`` to solve the linear
  system.

  I mentioned before, but did not stress attention on the fact that this
  segmentation approach is semi-automatic. This means that the user has to
  provide a set of labeled points. My current implementation either accepts user
  input or generates labeled points uniformly using the same approach as
  ``SupervoxelSegmentation`` does for seed generation. There are smarter ways of
  producing initial labeling and I plan to consider this issue later.

  Here are the some initial results that I got using random walk segmentation
  algorithm. The big red dots show the locations of labeled points:

  +----------------------------------------------+---------------------------------------------+
  | .. image:: img/osd/test55-voxels.png         | .. image:: img/07/test55-rwc-few-manual.png |
  |   :width: 320 px                             |   :width: 320 px                            |
  +----------------------------------------------+---------------------------------------------+
  | | Voxelized point cloud (voxel               | | Random walk segmentation                  |
  | | size 0.006 m)                              | | (with one manually labeled                |
  |                                              | | point per object)                         |
  +----------------------------------------------+---------------------------------------------+
  | .. image:: img/07/test55-rwc-many-manual.png | .. image:: img/07/test55-rwc-uniform.png    |
  |   :width: 320 px                             |   :width: 320 px                            |
  +----------------------------------------------+---------------------------------------------+
  | | Random walk segmentation                   | | Random walk segmentation                  |
  | | (with multiple manually                    | | (labeled points are uniformly             |
  | | labeled points in each object)             | | distributed)                              |
  +----------------------------------------------+---------------------------------------------+

  In the first case (top right) I manually labeled each object in the scene with
  one point. Many of the object boundaries are correctly labeled, however there
  are vast regions with totally random labels. This is especially evident in the
  upper right corner, where there is just one seed. A circular region around it
  is correctly labeled with single color, however the points outside of it are
  all randomly colored. According to my understanding, this happens because the
  edge weights are always less than 1, so a random walker can not get arbitrary
  far from the starting point. Thus if there are no labeled points in the
  vicinity of a point, then all the computed probabilities are nearly zero and
  label assignment happens randomly (because of numerical imprecision).

  In the second case (bottom left) I manually labeled each object in the scene
  with multiple points guided by an intuition about how large the "zones of
  influence" are around each labeled point. The resulting segmentation is rather
  good.

  In the third case (bottom right) the labeled points were selected uniformly
  from a voxel grid with size 10 cm. As a result I got an over-segmentation that
  resembles the ones produced by ``SupervoxelSegmentation``.

  I think the initial results are rather promising. In the future I plan to work
  on the weighting function as it seems to be the key component for the random
  walk segmentation. I would like to understand if and how it is possible to
  vary the size of the "zone of influence" around labeled points.


.. blogpost::
  :title: Fixing Bugs in Segmentation using Random Walks
  :author: alexandrov
  :date: 09-01-2013

  This is a follow-up post to the segmentation using random walks. It turned out
  that my initial implementation had several bugs which significantly worsened
  the performance. In this blog post I will describe them and show the outputs
  produced by the fixed algorithm.

  The segmentation results that I demonstrated last time expose two
  (undesirable) features: vast regions with random label assignment and "zones
  of influence" around seed points. My first intuitive explanation was that
  since the edge weights are always less than 1, a random walker can not get
  arbitrary far from its starting point, therefore if a seed is reasonably far,
  the probability of getting there is very small. Surprisingly, I did not find
  any mentions or discussions of this effect in the literature. Moreover, while
  carefully re-reading "Random Walks for Image Segmentation" I mentioned that
  the algorithm outputs for each point and label pair not the probability that a
  random walker started from that point will reach the label, but rather the
  probability that it will **first** reach that label (i.e. earlier than other
  labels).

  I decided to visualize edge weights and the probabilities I get with my
  implementation. In the following experiment I labeled three points, one on the
  table, and the other two on the boxes (see the left figure):

  +----------------------------------------------+------------------------------------------+------------------------------------------+
  | .. image:: img/08/test1-pointcloud-seeds.png | .. image:: img/08/test1-edge-weights.png | .. image:: img/08/test1-potentials-1.png |
  |   :width: 210 px                             |   :width: 210 px                         |   :width: 210 px                         |
  +----------------------------------------------+------------------------------------------+------------------------------------------+
  | | Color point cloud                          | | Edges between                          | | Probabilities that                     |
  | | with three labeled                         | | voxels                                 | | a random walker                        |
  | | points                                     |                                          | | will first reach the                   |
  |                                              |                                          | | top right label                        |
  +----------------------------------------------+------------------------------------------+------------------------------------------+

  The figure in the middle shows all edges along which random walkers move.
  Each edge is visualized by its middle point colored according to edge weight
  (using "jet" color map where 0 is dark blue and 1 is dark red). The weights do
  make sense, as the edges in the planar regions are mostly reddish (i.e. weight
  is close to 1), whereas on the surface boundaries they are bluish (i.e. weight
  is close to 0). Moreover, it is clear that concave and convex boundaries have
  different average weights.

  The figure on the right shows all points colored according to the probability
  that a random walker started from that point will **first** reach the labeled
  point on the box in the back. The probability images for the other labels are
  similar, thus for the majority of points the probability of first reaching
  **either** label is close to zero. Clearly, this can not be right, because
  some label has to be reached first anyways, and therefore the probabilities at
  each point should sum up to one. This observation triggered a long search for
  bugs in my implementation, but in the end I discovered and fixed two issues.

  As I mentioned before, the solution for random walker segmentation is obtained
  by solving a system of linear equations, where coefficients matrix consists of
  edge weights and their sums, arranged in a certain order. The system is huge
  (there are as many equations as there are unlabeled points), but sparse (the
  number of non-zero coefficients in a row depends on the number of edges
  incident to a point). The first issue was due to a bug (or a feature?) of
  ``OctreePointCloudAdjacency`` class that I used to determine neighborhoods of
  points. In a nutshell, it is a specialized octree, where leaves store pointers
  to their direct neighbors (in terms of 26-connectivity). For some reason, a
  leaf always stores a pointer to itself in the list of neighbors. This caused
  bogus self-edges in my graph which themselves did not show up in the matrix
  (because diagonal elements are occupied by sums of weights of incident edges),
  however treacherously contributed to those sums, thus invalidating them.

  The second issue was more subtle. For the system to have a solution it has to
  be non-singular. In terms of the graph it means that it either has to be
  connected, or should contain at least one seed in every connected component.
  From the beginning I had a check to enforce this requirement, however I did
  not take into account that the weighting function may assign zero weight to an
  edge! In the pathological case all the edges incident to a point may happen to
  be "weightless", thus resulting in an all-zero row in the system.

  Below are the results obtained using the random walker algorithm after I fixed
  the described issues:

  +-----------------------------------------------+------------------------------------------------+
  | .. image:: img/08/test55-rwc-fixed-manual.png | .. image:: img/08/random-walker-potentials.gif |
  |   :width: 320 px                              |   :width: 320 px                               |
  +-----------------------------------------------+------------------------------------------------+
  | | Random walk segmentation                    | | Probabilities that a random                  |
  | | with overlaid seed points                   | | walker will first reach each                 |
  |                                               | | of the labels                                |
  +-----------------------------------------------+------------------------------------------------+

  As I did it previously, I labeled a single point in each object. (Actually, a
  single point in each disjoint part of each object. That is why there are
  multiple labeled points on the table.) The scene is perfectly segmented. On
  the right is an animated GIF with a sequence of frames which show
  probabilities (potentials) for each label. In most cases the separation
  between object and background is very sharp, but in two cases (the small box
  on top of the book and standing box on the right) some non-zero probabilities
  "spill" outside the object. Nevertheless, this does not affect the final
  segmentation since the potentials for the correct labels are higher.

  I am very happy with the results, however one should remember that they were
  obtained using manual selection of labels. This code sprint is aimed at an
  automatic segmentation, so next I plan to consider different strategies of
  turning this into a fully autonomous method.
