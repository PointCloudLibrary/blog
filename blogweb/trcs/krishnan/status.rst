My status updates
=================

.. blogpost::
  :title: Frustum Culling
  :author: krishnan
  :date: 09-30-2012

  Filters points lying within the frustum of the camera. The frustum is defined by pose and
  field of view of the camera. The parameters to this method are horizontal FOV, vertical FOV, near plane distance and
  far plane distance. I have added this method in the filters module. The frustum and the filtered points are shown in
  the images below.

  .. image:: images/frustum-culling-1.png
     :height: 300 pt 
     :width: 300 pt
     
  .. image:: images/frustum-culling-2.png
     :height: 300 pt 
     :width: 300 pt

  .. image:: images/frustum-culling-3.png
     :height: 300 pt 
     :width: 300 pt

.. blogpost::
  :title: ShadowPoints filter
  :author: krishnan
  :date: 07-08-2012

  This filter removes the ghost points that appear on the edges. This is done by thresholding the dot product of the
  normal at a point with the point itself. Points that obey the thresholding criteria are retained. This completes the
  port of libpointmatcher to PCL. Now, I will be writing examples for all the techniques I added.

.. blogpost::
  :title: SamplingSurfaceNormal filter
  :author: krishnan
  :date: 07-04-2012

  This filter recursively divides the data into grids until each grid contains a maximum of N points. Normals are
  computed on each grid. Points within each grid are sampled randomly and the computed normal is assigned to these
  points. This is a port from libpointmatcher.

.. blogpost::
  :title: Adding a new filtering technique
  :author: krishnan
  :date: 05-12-2012

  I am working on a new filtering technique that divides the point cloud into boxes that have similar densities and
  approximates the box by the centroid. This will be completed soon. Parallely I am also working on the registration
  tutorial that I mentioned in the previous blog post.

.. blogpost::
  :title: Correspondence rejection based on computing an optimal inlier ratio
  :author: krishnan
  :date: 04-22-2012

  This method extends the existing outlier rejection method 'CorrespondenceRejectionTrimmed' based on the fraction of
  inliers. The optimal inlier ratio is computed internally instead of using a user specified value. The method is
  described in the paper 'Outlier Robust ICP for Minimizaing Fractional RMSD' by Jeff M. Philips et al. I am adding the 
  test code in the test folder. This wraps up the porting of correspondence rejection methods from libpointmather to PCL.
  Next I am planning to add some filtering methods. Before that I will be adding tutorials on using the registration
  pipleline in PCL.

.. blogpost::
  :title: Correspondence rejection based on the orientation of normals
  :author: krishnan
  :date: 04-18-2012

  As mentioned in my previous blog post I added a new correspondence rejection method based on the orientation of the
  normals. A threshold based on the dot product of the normals at corresponding points is used as a criteria for
  correspondence rejection. I am adding sample codes demonstrating the usage of the new classes in the test folder. It
  should be in the trunk after the code review. I will be adding a couple of new rejection methods over the next week to
  wrap up the port of correspondence rejection methods from libpointmatcher.

.. blogpost::
  :title: Added new correspondence rejection method base on median distance of the correspondences
  :author: krishnan
  :date: 04-08-2012

  I ported a new correspondence rejection method from libpointmatcher. This method operates on a new thresholding
  criteria. The median of the distances between corresponding points, times a factor is used as a threshold for rejecting
  correspondences. The user can set the desired factor. A discussion of this method can be found in the paper "Simultaneous Localization and Mapping with Active Stereo Vision. Diebel, J. and Reutersward, K. and Thrun, S. and Davis, J. and Gupta". 
  Next I will be adding a rejection method based on the normals at the corresponding points.

.. blogpost::
  :title: Integrating libpointmatcher with PCL
  :author: krishnan
  :date: 04-02-2012

  I will be integrating some modules from libpointmacher (https://github.com/ethz-asl/libpointmatcher) to PCL. In the next couple of weeks new methods will be ported
  from libpointmatcher to the CorrespondenceRejection module in PCL. Documentation and results for the same will also be
  added.

.. blogpost::
  :title: Results for correspondence estimation based on normal shooting
  :author: krishnan
  :date: 02-23-2012

  For every point in the source cloud, the normal and K nearest points in the target cloud are computed. Among these K points, the point
  that has the least distance to the normal (point to line distance in 3d http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html) is considered as the corresponding point. Result for a test dataset created is shown below. Two parallel planes differing in their y co-ordinates are created. Correspondences are shown by connecting lines. Implementation files and test program are available in the trunk.

  Points on the planes shown in white dots. Correspondence shown by red lines.

  .. image:: images/normal-shooting-1.png
     :height: 300 pt 
     :width: 600 pt

  .. image:: images/normal-shooting-2.png
     :height: 300 pt 
     :width: 600 pt

.. blogpost::
  :title: Started my experiments
  :author: krishnan
  :date: 02-12-2012

  I have started validating the existing registration algorithms on Trimble data.  I am beginning with validating Point to Plane ICP on the entire dataset. Its going to take a few hours for the algorithm to finish given the size of the dataset. On the implementation side, I have implemented CorrespondenceEstimation based on normal shooting. A new class CorrespondenceEstimationNormalShooting should be available in the trunk shortly. The reference to this method is "Efficient Variants of ICP" as mentioned in my previous blog post.

.. blogpost::
  :title: Hello Everybody
  :author: krishnan
  :date: 02-05-2012


  This is my first blog post for TRCS and I am pretty excited about this project. I was getting a hang of the registration pipeline in PCL and understanding the various modules over the last couple of days. I read the paper "Efficient Variants of ICP" which I recommend reading for people interested in understanding the variants of ICP. This should give a good idea on customizing the different modules of the PCL registration pipeline to suit your dataset. Also it helps in understanding the registration pipleline in PCL. We had a discussion on the modules where new algorithms can be added and we have shortlisted some for now. I will be working on pairwise registration to begin with.

