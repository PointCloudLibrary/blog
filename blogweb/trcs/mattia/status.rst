My status updates
=================
.. blogpost::
  :title: Tests and report
  :author: Mattia
  :date: 05-26-2012
  
  I'm working on a smart clustering algorithm which will improve the segmentation accuracy of a cloud based on the region growing algorithm. In the meantime I'm finishing the SVN classifier and I'll be ready to put it into PCL very soon. 

  In this period, I had to work also for some projects in my place; so I haven't been spending too much time for the final report of the automated noise filtering plugin. I give me one week as a deadline to finish it. 

.. blogpost::
  :title: Last modifies for the final program
  :author: Mattia
  :date: 05-05-2012

  The last weeks I've been also busy with some works in my town. I just finished my part of the "final noise removal system" and I aim to finish the report for in the next few days. Our work is finally returning promising results but we still need a feedback from our mentors to improve the solution.

.. blogpost::
  :title: Completing modules for PCL
  :author: Mattia
  :date: 04-20-2012
  
  The last days I've been working on two methods I would like to put into PCL. The first regards the contour extraction of objects in a cloud, the second is the Support Vector Machine. Two issues have been threw at:

  - `http://dev.pointclouds.org/issues/665 <http://dev.pointclouds.org/issues/665>`_
  - `http://dev.pointclouds.org/issues/667 <http://dev.pointclouds.org/issues/667>`_

  From tomorrow I'll work at the system that I and Florentinus are setting up. I will have to work on a smart way to easily train a classifier.

.. blogpost::
  :title: Principal Component Analysis for edges and contours extraction
  :author: Mattia
  :date: 04-16-2012

  PCA (principal component analysis) is a simple method to extract information from a set of data. Geometrically, its objective is to present the data from the reference axes that mainly highlights their structure.

  The idea was to give a meaning to points depending on their positions and using the PCA. For each point I extracted the k-neighbors, calculated the PCA (from the centroid) and analyzed the eigenvalues. While the eigenvectors give the direction of the axes along which the data are extended, the eigenvalues ​​are their length. 

  From a geometrical point of view, three eigenvalues ​​of equal magnitude represents a uniform distribution of a solid; a limited distribution on two axes represents a plane, while a distribution skewed on a single eigenvalue represents a line. Carrying this theory on a cloud of points, we have that points in the proximity of the edges  has one eigenvalue much larger than the other. For the inner points, in case of uniform distribution of the point density, the length of the eigenvalues ​​will be better distributed along all the three eigenvectors.
  
  Normalizing the three eigenvectors, we have that in correspondence of a contour or an angle of an object the largest eigenvalue is a value greater than 2/3 (about 66%) of the total propagation of the points. The results are shown below.

  .. image:: image/04162012/1.jpg 
     :width: 800 pt

  .. image:: image/04162012/2.jpg 
     :width: 800 pt

.. blogpost::
  :title: First Automated Noise Filtering program
  :author: Mattia
  :date: 04-12-2012

  Yesterday, I sent to Jorge the first revision of the program that implements the pipeline designed for the removal of vegetation and ghost points. The program, designed by me and Florentinus, segments the image into large groups which are then classified.

  The biggest problem encountered is related to the number of samples needed for optimal training of the classifier. Dividing the cloud into large groups, these samples are too few to make this happens. So we think of a more refined method for the recognition of the problems.

  Since Florentinus will be busy this week, I will concentrate on implementing a filtering algorithm based on Principal Component Analysis. Throught this approach it's possible the filtering of lines (1D), flat surfaces (2D) and solid objects (3D). More details will come up soon.

.. blogpost::
  :title: Implementation progress
  :author: Mattia
  :date: 04-08-2012
  
  I haven't been posting for a while. In this week I didn't get any new result apart working on the SVM learning machine implementation following the PCL standards. As Florentinus, I'm working at the same program for an automated filtering elaboration. I've been testing new features for the classifier like:

  - PCA.
  - Point density within a cluster.

  After the feature extraction, my steps are twofold: a first training procedure for the classifier and a classification. 

  In the first case, since the generated clusters are pretty big, they are iteratively displayed one by one asking for an user feedback. The data are then used for a .model file which is generated and loaded for future predictions.

  Among the improvements, i found the way to calculate a score percentage which indicates if a cluster is more likely to belong to a class instead of another.

.. blogpost::
  :title: Pre-Filtering progresses
  :author: Mattia
  :date: 04-02-2012

  During the pre-filtering, me and want florentinus take advantage of the structured nature of the point cloud to make a pre-segmentation of the cloud. The latter is based on the search of adjacent pixels having similar intensity characteristics. The first step is therefore to generate an image as the one shown below:
  
  .. image:: image/04022012/1.jpg 
     :width: 800 pt

  In red the nan-points. The points are very scattered, but a pre-segmentation would immediately highlight the individual positions of the leaves. Among the algorithms studied, I took into account the GrabCut implemented in OpenCV (GrabCut is an image segmentation method based on graph cuts) and a Graph-Based Segmentation discussed in this `paper <http://www.cs.brown.edu/~pff/papers/seg-ijcv.pdf>`_.

  The method implemented in opencv is very powerful and segments the image by dividing the background from the foreground. The result is not very useful in our case and, consequently, I haven't investigated the use.
  The second method proved to be very powerful! It is based on the simple proximity of similar tonalities and a simple result is shown in this figure:

  .. image:: image/04022012/2.jpg 
     :width: 800 pt  

  Different colors represent different segments. Besides highlighting many details, the algorithm is very fast and the resulting segmentation could have a great practical implication.

  After a chat with Jorge, me and Florentinus decided to conclude a first "release" of our filtering pipeline by the end of this week. Therefore, I will leave aside the pre-filtering (which will take some time to be adapted to our requirements) and I will spend more time for the optimization of the steps that have already been tested.

.. blogpost::
  :title: Best features for noise recognition
  :author: Mattia
  :date: 03-30-2012
  
  Together with Florentinus, we concluded to divide the job for removing noisy points following a pipeline of this type.

  .. image:: ../florentinus/09.png
    :width: 720 pt
    :height: 72 pt

  (by courtesy of Florentinus)

  The work done during these days was to improve the classification stage of the pipeline. This mainly consists in the search for features outlining the errors and in making flexible as possible the SVM learning machine. This required great efforts in terms of analysis of the codes and research between the methods already present in PCL for the extraction of features.

  What has been most tested is the VFH (`link <http://pointclouds.org/documentation/tutorials/vfh_estimation.php>`_) that provides in output 308 features for each cluster analyzed. Unfortunately, the results were realy bad. The goal for the future is to find features that describe the surroundings of a cluster together with the intrinsic properties.

  In the coming days I will also work to make SVM (based on libsvm) compatible with the other libraries of PCL.

.. blogpost::
  :title: SVM ready. Next step: pre-filtering
  :author: Mattia
  :date: 03-27-2012
  
  In the last few days I have been working to improve the implementation and performance of the machine learnng SVM. By providing a pre-labeled training set, I managed to get a performance rating of about 95%.

  The trained classifier shows good performance as highlighted in the following screenshots:

  .. image:: image/03282012/1.png 
     :width: 800 pt

  .. image:: image/03282012/2.png 
     :width: 800 pt

  .. image:: image/03282012/3.png 
     :width: 800 pt

  The next days I will deal with the extraction of segments of the cloud in order to reduce the number of points on which to perform the classification.

  Florentinus, meanwhile, is working on finding the best features to be extracted in a cluster to improve performance and minimize mistakes.

  Soon I will test also a classifier k-NearestNeighbour based on a kDTree representation.

.. blogpost::
  :title: SVM ready. Next step: pre-filtering
  :author: Mattia
  :date: 03-24-2012

  The SVM classifier is implemented and ready to be used and tested in our automated noise filtering working chain. It is of course based on libsvm and i created classes for training,testing and use the classification algorithm.

  An interesting chat with Florentinus, highlighted a new methods which is worth to be tested. They came up after reading `this <http://gfx.cs.princeton.edu/pubs/Golovinskiy_2009_SRO/paper.pdf>`_ paper (more info in Florentinus' blog).
  
  In the next days i want to test the classifier for clusters recognition. Then I'll start thinking on a pre-filtering process based on organized grid representations of Tribmle's datasets.

.. blogpost::
  :title: Studying machine learning uses
  :author: Mattia
  :date: 03-20-2012

  Recently I'm working hard on understanding and best configuring the most famous machine learning algorithms. The purpose is to use Support Vector Machines to generate a weighting value for clusters in a cloud, then to group the ones with similarities and finally remove leaves and ghosts.

  I've also thought to use Artificial Neural Networks and I started to implement the learning algorithm. But after discussing about it with Jorge and Federico, they addressed me toward more sofisticated and better performing approaches, exactly like SVMs. 

  Results will come up soon.

.. blogpost::
  :title: Features Analysis
  :author: Mattia
  :date: 03-17-2012
  
  Moving forward on my journey to noise removal, I'm facing the problem of point clusters identification for noise extraction. The problem is not easy at all, and I show in this blog post the frequency histograms meant to compare different features for the cluster identification. Moreover, I want to find range of values of these features with the purpose of marking  a cluster with a *matching percentage score*.
  
  The next step is the use of a good method to build a classifier. I really would like to implement **Artificial Neural Networks**. I know that it's not the shortest and easiest way, but it's probably the most powerful and gratifying.

  - *Frequency Histograms representing the intensity distribution:*

  .. image:: image/03172012/1.png 
     :width: 800 pt
     :height: 300 pt 

  - *Frequency Histograms representing the cardinality distribution:*

  .. image:: image/03172012/3.png 
     :width: 800 pt
     :height: 300 pt

  - *Frequency Histograms representing the Standard Deviation distribution of normal vectors inside the clusters:*

  .. image:: image/03172012/2.png 
     :width: 800 pt
     :height: 300 pt

  - *Frequency Histograms representing the Standard Deviation distribution of curvatures inside the clusters:*

  .. image:: image/03172012/4.png 
     :width: 800 pt
     :height: 300 pt

  - *Frequency Histograms representing the distribution of the eigenvalues calculated from the covariance matrices of each cluster:*

  .. image:: image/03172012/5.png 
     :width: 800 pt
     :height: 300 pt

.. blogpost::
  :title: Studying existing segmentation procedures
  :author: Mattia
  :date: 03-13-2012
  
  We try to decrease the number of elements in a point cloud by collecting the points into groups called *clusters*. For this purpose it is very important the choice of a measuring distance that links points into a cluster.
  The most common types are:

   - **Euclidean** takes into account the direction and magnitude of vectors:  :math:`\sqrt{(x_i-y_i)^2}`

   - **Squared Euclidean** accentuates the distance between entities and tends to give more weight to outliers:  :math:`(x_i-y_i)^2`

   - **Manhattan** is greater than Euclidean but it might appear less compact: :math:`|x_i-y_i|`

  Once the method is decided, the strategy of classification can be *hierarchical* or *non-hierarchical*. A non-hierarchical method fixes the number of clusters a priori (like *K-Means* or *Self Organizing Map*). A hierarchical method starts from a number of clusters equal to the number of points and progressively reduces the number of clusters combining the calculated ones based on a "closeness" criteria.

  Following these steps, the points are organized into single entities to which we want to attribute a physical meaning. The aim of the noise removal requires to distinguish ghost points and vegetation from the useful data. The idea is then to assign labels associated to a classification score. To do this we will train a classifier with a large amount of training datasets, and analyze some features like:

   - cardinality;
   - eigenvalue decomposition ​​of the covariance matrix;
   - intensity;
   - curvature;
   - normal variance.

  Obviously, the cloud will be full of classification errors and mismatches. So, we will introduce a *linking policy* with which a leaf or a ghost cluster must be surrounded by clusters of the same type to be removed. This further analysis has the goal of making the method more robust and flexible. To do this we need to define the "distance" between clusters and different criteria like:

  - ***Local Measures***
    
    - **Minimum or Nearest-Neighbour Method**: the distance between two groups is the minimum distance between all pairs of points belonging to the first and second group. The criterion generates groups with the shape of "extended chains" of scattered elements.
    - **Maximum or Furthest-Neighbour Method**: the measure of distance between two groups is the maximum distance between all pairs of points belonging to the first and second group. The criterion generates compact groups with very close elements.

  - ***Global Measures***

    - **Within Groups Clustering Method**: it considers the average of all the distances between pairs of points in the first and second group.
    - **Centroid**: the distance between two groups is determined by the distance of their centers of gravity.
    - **Ward**: the clusters are aggregated so that the variance increment in the new group is the minimum possible (every time you add a connection the variance increases; we want to minimize this increase).

.. blogpost::
  :title: Clustering process
  :author: Mattia
  :date: 03-04-2012

  After the use of a Region Growing clustering process based on the Euclidean distance, I show a good result which will definitelly be good for the recognition of leaves on trees. In the image below, different colors mean different clusters. Next step is the use of a classifier to distinguish good and noisy clusters.

  .. image:: image/03042012/1.png 
     :width: 900 pt

.. blogpost::
  :title: Some successful result for the leaves removal filter
  :author: Mattia
  :date: 03-02-2012

  From the last chat meeting had with Jorge, Radu and Federico (see Florentinus' blog) I got to experiment with new ideas.

  The proposed filtering step is based on the calculation of the covariance matrix of points coordinates, in the neighborhood of a sphere of radius R. Using an EVD (Eigen Value Decomposition), the filtering policy is based on:

  :math:`s <= \frac{\lambda_{min}}{\lambda_{max}}`

  where :math:`s` is an user defined constant (in my case 0.04). All the points respecting the previous constraint are deleted from the cloud.

  The second step of filtering uses a simple RadiusOutlierRemoval filter iterated twice. The results are shown in figure:

  .. image:: image/03022012/1.png 
     :height: 300 pt 
     :width: 900 pt

  The method on the global cloud reported minimal loss of non-noisy points and high processing time (just over an hour).

  .. image:: image/03022012/2.png 
     :height: 300 pt 
     :width: 900 pt
 
  .. image:: image/03022012/3.png 
     :height: 300 pt 
     :width: 900 pt

  This solution is therefore an excellent candidate for the removal of vegetations. In the next study I will try to segment the image to apply the filter only in the areas which are marked as "vegetation". Hopefully, this will minimize the loss of details not meant to be filtered.  Once I get good results I'll optimize the algorithm to reduce the computation time.
  
.. blogpost::
  :title: Integral Images  with point clouds
  :author: Mattia
  :date: 02-25-2012

  I got back on track after my graduation holidays. I'm currently studying the Integral Images approach and how to use the technique for the point cloud.
  Nice results are expected from Tuesday.

.. blogpost::
  :title: New filtering idea (work in progress...)
  :author: Mattia
  :date: 02-16-2012

  Among all algorithms the filter StatisticalOutlierRemoval is definitely the best, although this has many faults such as the elimination of good portions of the cloud. Thanks to the suggestions of Jorge and Federico, I spent some time considering optimizations and finding out how far we can improve the algorithm.

  The studied subjects are two:

    1. Optimizing the search for adjacent points with the study of integral images.
    2. Optimizing the search of noisy points.

  The last three days I was focused on the second point. First, it is important to consider how the StatisticalOutlierRemoval works:

    - For all the points of the cloud, it is calculated the average Euclidean distance of the target point with respect to a set of N neighboring points.
    - Then, it is estimated variance and standard deviation of all the mean points resulting in a bell-shaped distribution similar to the Gaussian one.
    - It iterates again to the cloud points and deletes all those which fall outside a certain variance range.

  A major drawback is that this algorithm does not take into account the direction of the calculated distances. A good idea is to change the previous filter introducing the covariance matrix and the eigenvalue decomposition​​:

   - For all cloud points it is calculated the vector of the mean value distances (:math:`u`) within a fixed radius R.
   - It calculates the covariance matrix using :math:`X_i` measurements (along axis x, y and z) and the expected value :math:`u`.
   - From the covariance matrix, the eigenvalue decomposition is performed ​​and the smaller eigenvalue is taken into account (NB: The eigenvalue with the largest value corresponds to the axis that has the greatest variance; it will therefore be the variance of the main component 1. Then, the other eigenvalues ​​are the variances along the other  "weak" directions. A large eigenvalue indicates that the query point is slightly connected to other points along a direction, and vice versa for a small eigenvalue. Taking the smallest eigenvalue we can assess the best "connection" level with the rest of the point cloud).
   - *Variance* and *standard deviation* are extracted from the eigenvalues, resulting in a bell-shaped distribution similar to the Gaussian one.
   - The algorithm iterates again to the cloud points and deletes all those which fall outside a certain variance range.

  From a theoretical side, the changes should remove only the points that are weakly connected to the rest of the cloud along the three directions x, y and z. After various tests, and despite the validity of the theoretical functioning, the attached picture shows that real results are not really promising as there are removed points almost exclusively from the "good" part of the cloud.

  .. image:: image/02162012/1.png 
     :height: 300 pt 
     :width: 600 pt

  The ghost points are still present and the points are particularly affected in the corners (see the building facade and the car).

.. blogpost::
  :title: Benchmark of the computation time
  :author: Mattia
  :date: 02-13-2012

  I have prepared a detailed report for the analysis of the computational speed of the filtering methods: RadiusOutlierRemoval and StatisticalOutlierRemoval. To do that, I performed the tests on two point clouds: *A* of 3.7Mln points and * B * of 24.4Mln points. The laptop I used has Linux Kubuntu 11.04, Intel Core Duo P8600 2.4GHz and 4GB RAM.

  .. centered:: **RadiusOutlierRemoval**

  A for loop iterates through all the N elements of the cloud (complexity O(N)). Within the same cycle, all points within a radius r are found to check if the wuery point has a sufficient number of neighbors. Assuming that the search algorithm of the points within a radius r ('RadiusSearch') is brute-force (O(N) for unordered point clouds), the computational complexity of this method is O(N·N) .

  The table has been constructed by varying the searching radius of the points, and the results are expressed in seconds:

  ========  =============  =============
  Ray (mm)  Cloud A (sec)  Cloud B (sec)
  ========  =============  =============
  10             12s          24s
  20             24s          43s
  50             93s         151s
  100           320s         530s
  200          1569s        2200s
  ========  =============  =============
  
  By increasing the searching radius, the computation time grows very fast. This means that the search algorithm totally affects the filtering speed of this methodology.
  
  .. image:: image/02132012/1_1.png 
     :height: 250 pt 
     :width: 500 pt

  .. centered::  **StatisticalOutlierRemoval**

  For this filter, a for loop iterates through all the elements of the point cloud (O (N)). Then, still within the same cycle, the method 'nearestKSearch' searches for the closest *meanK* points to the query point (O(N·meanK)). Afterthat the average distance is calculated to obtain the Gaussian distribution (O(N)). Finally, the filtering end up taking into consideration the variance analysis (O (N)). Thus, the computational complexity is approximately: O(N (N·meanK) + N + N).

  ============  =============  =============
  Num of neigh  Cloud A (sec)  Cloud B (sec)
  ============  =============  =============
  10                 12s          25s
  20                 17s          33s
  50                 33s          65s
  100               186s         134s 
  200              1600s         406s
  ============  =============  =============

  .. image:: image/02132012/2_1.png 
     :height: 250 pt 
     :width: 500 pt

  .. centered::  **Conclusion**

  The above results show that the search algorithms are the most time consuming part of the classes. Therefore it's very important to develop a  spherical ordered search algorithm in order to optimize any type of filter operation that requires the searching of surrounding points: 'SphericalOrganizedNeighbor'.

.. blogpost::
  :title: Finding the best method to filter vegetation
  :author: Mattia
  :date: 02-12-2012

  A first analysis of the filters available in PCL, showed that only two can be considered almost valid for the removal of "ghost points" and vegetation.
  Though a good setting of the parameters can return good results for the removal of shadows and moving objects, the results were not as satisfactory for the removal of vegetation. As a point of reference I took a cloud in which the trees are covered with many leaves. The goal was to minimize the leaves which are described as small groups of flying points in a PointCloud.
  
  **Results with RadiusOutlierRemoval** (in the right the original cloud, in the left the filtered one):

  .. image:: image/02132012/1.png 
     :height: 300 pt 
     :width: 600 pt

  .. image:: image/02132012/2.png 
     :height: 300 pt 
     :width: 600 pt


  **Results with StatisticalOutlierRemoval** (in the right the original cloud, in the left the filtered one):

  .. image:: image/02132012/3.png 
     :height: 300 pt 
     :width: 600 pt

  .. image:: image/02132012/4.png 
     :height: 300 pt 
     :width: 600 pt


  In conclusion I can say that neither of the filters is actually able to offer an intelligent removal of vegetation without damaging the "good" background of the point cloud.


.. blogpost::
  :title: Finding the best method to filter scattered ghost points
  :author: Mattia
  :date: 02-07-2012

  For the purpose of the filtering of ghost points, me and Florentinus decided to take as reference the dataset Statues_1. Inspecting that scenario, we marked as possible ghost noises some points highlighted in the following images:

  .. image:: image/02072012/1.png 
     :height: 300 pt 
     :width: 600 pt

  The inspected methods are:

   - PassThrough< PointT >
   - RadiusOutlierRemoval< PointT >
   - ConditionalRemoval< PointT >
   - StatisticalOutlierRemoval< PointT >
   - VoxelGrid< PointT >
   - ApproximateVoxelGrid< PointT >

  It turned out that only two methods can be succesfully used for our purpose: the *RadiusOutlierRemoval* and the *StatisticalOutlierRemoval*. 

  **RadiusOutlierRemoval**: the user specifies a number of neighbors which every indices must have within a specified radius to remain in the PointCloud. Because the filter is based on a fixed radius, many points on the object contours are deleted. Due to a good filtering result, the "good" objects are also affected.

  .. image:: image/02072012/4.png 
     :height: 300 pt 
     :width: 600 pt

  **StatisticalOutlierRemoval**: for each point, it computes the mean distance from it to a specified number of neighbors. By assuming that the resulted distribution is Gaussian with a mean and a standard deviation, all points whose mean distances are outside an interval defined by the global distances mean and standard deviation are trimmed from the dataset.
  This is so far the best method for the ghost deletion; moreover, it strictly depends to the parameters and the filter often removes portions of small "good" objects.

  .. image:: image/02072012/5.png 
     :height: 300 pt 
     :width: 600 pt

.. blogpost::
  :title: Start the real work
  :author: Mattia
  :date: 02-01-2012

  Everything is ready to start testing the PCL filters on Trimble data. I plan to talk with Florentinus, Jorge and Radu to define the "noise" and the expected accuracy of the filters.

.. blogpost::
  :title: Setting up my workspace
  :author: Mattia
  :date: 23-01-2012

  Today I realized a small program meant to test the filter functions. It permits to compare the differences on a point cloud before and after the filtering operation; moreover it is possible to change parameters without recompiling the program.

.. blogpost::
  :title: First SVN connection
  :author: Mattia
  :date: 18-01-2012

  I started learning how to use Subversion and Sphinx.
  Tomorrow I will meet some PhD students of my university; I would like to collaborate with some of them and get access to some tools (cameras) to test the algorithms I will develop.
