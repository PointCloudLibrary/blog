My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Christian Potthast

.. blogpost::
   :title: Modified Voxel Grid Filter for PointXYZRGBL
   :author: Christian Potthast
   :date: 06-20-2012

   To be able to construct a Conditional Random Field from a point cloud I am using a voxel grid for sensor input discretization. This allows to work in grid cells rather than euclidean distances. The problem what I was facing was the following, which made it necessary to extend the current voxel grid implementation with a new subclass inherited from pcl::VoxelGrid<PoinT>. The current implementation of the voxel grid filter does a linear interpolation over the position and PointCloud field lying in the same grid cell. This is problematic for the PointCloud type PointXYZRGBL, which assigns a label to any point in the cloud. By interpolating these labels can become wrong since they are just simple unsigned integer values. In my implementation I modified the voting scheme for the field 'label'. If many point lie in the same grid cell, the label number with the highest occurrence wins.

   .. image:: images/voxel_grid_label.png
	  :align: center

   On the left side you can see the behaviour of the modified voxel grid filter. You can see on the right side, which is the standart voxel grid filter, that the labels are wrong due to the interpolation.

.. blogpost::
   :title: First segmentation results
   :author: Christian Potthast
   :date: 06-20-2012

   I implemented and modified the following two paper to handle n-dimensional data inputs given as a point cloud.

   * Efficient Inference in Fully Connected CRFs with Gaussian Edge Potentials, Philipp Krähenbühl, Vladlen Koltun

   *  Fast high-dimensional filtering using the permutohedral lattice, A. Adams, J. Baek, and M. A. Davis.

   The input for the segmentation algorithm is a point cloud with XYZRGB (will be extended further) ordered and unordered. To convert the input point cloud into a Conditional Random Field I am using a modified voxel grid for scene discretization. Each cell with a measurement becomes a node in the graphical model. For now the edge potential features incorporate position as well as color information.

   As a first step I am using the algorithm for supervised segmentation, hand label an input scene. In the following picture the input cloud (left) and the labeled cloud (right) can be seen.

   .. image:: images/input_label.png
      :align: center

   With the labels are used to initialize the unary potentials of the CRF. The potentials are initialized as follows. A point with associated label gets a 0.3 probability that the label is correct. Further more I am assigning 10% of the points a randomly chosen wrong label.

   In the next image you can see on the left the the noisy point cloud initialized with the unary energies. On the right you can see the result after segmentation. 

   .. image:: images/unary_seg.png
      :align: center

.. blogpost::
   :title: Segmentation using color and normals features
   :author: Christian Potthast
   :date: 07-10-2012

   The following picture two pictures show the input data set. On the left side you see the captured point cloud captured using an Asus camera. On the right side you see the han labeled data set. I will show different segmentation results using different features and different levels of noise when setting the labels as unary potentials. One challenging part of the image (red circle) is the boundary between the box and the table. The box in the lower right corner has (almost) the same color as the table. 

   .. image:: images/2012-07-10/pc_anno.png
      :align: center

   In the following image sequence you'll thesegmentation using onlycolor information. The input labels arewith 50% noise assigned, meaning each unary potential is with 50% probability a random label assigned. From left to right the different results after x number of iterations can be seen. Whereas X is [0, 1, 3, 5, 10, 15]. Notice that when using only color information the table label grows into the box (red circle).

   .. image:: images/2012-07-10/noisy50_it-0_it-1_it-3_noNormal.png
      :align: center
   .. image:: images/2012-07-10/noisy50_it-5_it-10_it-15_noNormal.png
      :align: center

   In the next image sequence we use only the normals as features. One can see that normals by itself are very powerful. However, we will also see that using only normal information has it's limitations as well. The number of iterations per image is kept the same as well as the noise level.

   .. image:: images/2012-07-10/noisy50_it-0_it-1_it-3_Normal.png
      :align: center
   .. image:: images/2012-07-10/noisy50_it-5_it-10_it-15_Normal.png
      :align: center

   Lastly Color + Normal features are used for segmentation. Notice that using color and normal features has extremely fast convergence. After only 5 iterations we have a very acceptable result.

   .. image:: images/2012-07-10/noisy50_it-0_it-1_it-3_Normal+Color.png
      :align: center
   .. image:: images/2012-07-10/noisy50_it-5_it-10_it-15_Normal+Color.png
      :align: center

.. blogpost::
   :title: Segmentation using color and normals features
   :author: Christian Potthast
   :date: 07-10-2012

   In the second segmentation experiment I wanted to push the algorithm to the limit. For this I made the unary potentials extremely noisy. The potentials get with 80% a random label assigned. The first image sequence shows the segmentation result from left to right with different number of iterations [0, 1, 3, 5, 10, 15]. For the first test we use again only color features. We can see that by using only color features the algorithm performs poorly, which is not surprising. Changing the weights might help a little bit, however to make it a fair comparision I kept the weights and the standard deviations for the Gaussian kernels constant.

   .. image:: images/2012-07-10/noisy80_it-0_it-1_it-3_noNormal.png
      :align: center
   .. image:: images/2012-07-10/noisy80_it-5_it-10_it-15_noNormal.png
      :align: center

   Next we use only the normals as features. Using the normals results in surprisingly good results. The background is labeled almost perfectly as well as the objects on the table. The table itselfhowever, remains unlabeled. To this point I have no good explanations why this is the case. Further investigation might be interesting. 

   .. image:: images/2012-07-10/noisy80_it-0_it-1_it-3_Normal.png
      :align: center
   .. image:: images/2012-07-10/noisy80_it-5_it-10_it-15_Normal.png
      :align: center

   Lastly we use Color + Normals features. To my surprise, I actually did not expect such a good result. The only part that seems to be mislabeled are table legs. 

   .. image:: images/2012-07-10/noisy80_it-0_it-1_it-3_Normal+Color.png
      :align: center
   .. image:: images/2012-07-10/noisy80_it-5_it-10_it-15_Normal+Color.png
      :align: center

.. blogpost::
   :title: Supervised Segmentation
   :author: Christian Potthast
   :date: 08-10-2012

   The supervised segmentation is a two step process. Consisting of training phase and segmentation. In the training phase we extract the objects from the scene. We use the FPFH features as classifiers and as a prior assignment of the unary potentials of the CRF.
   We compute the FPFH histogram features for all points in one object. To reduce computation and feature comparisons in the recognition step we use a k-means cluster algorithm and cluster the feature into 10 classes. The training objects can be seen in the following image.

   .. image:: images/2012-08-10/training.png
      :align: center

   In the segmentation and recognition step we use the learned features to assign prior probabilities to a new scene. The prior assignment of the most likely label can be seen in the following image. As you can see in the image, many of the points of the objects we want to segment and recognize are not labeled correctly. This is because the distance of two FPFH features are two far apart. However, as a first initial estimate, FPFH features are well suited. The advantage of using these features is the fact that it only captures the geometry of the features and not color information. Whit this the training data set can be much smaller. 

   .. image:: images/2012-08-10/prior.png
      :align: center

   As a second and to refine the assignment we use the fully connected CRF. The following image shows the segmentation and labeling after 10 iterations. 

   .. image:: images/2012-08-10/seg.png
      :align: center
