.. blogpost::
   :title: Hello Everybody!
   :author: sushakov
   :date: 06-28-2013

   The project "Fast 3D cluster recognition of pedestrians and cars in uncluttered scenes" has been started!

.. blogpost::
   :title: Roadmap
   :author: sushakov
   :date: 07-09-2013

   At first, we decided to implement some simple features such as:

   * width and hight of the AABB (axis aligned bounding box)
   * center of mass
   * volume of the convex hull
   * area of the projection of the cloud
   * eccentricity
   * moment of inertia

   I've already started implementing them. After this step I will implement some more complex descriptors (e.g. 3D SURF, RoPS - Rotational Projection Statistics). And finally I'm going to use machine learning methods for the object recognition.

.. blogpost::
   :title: Simple Features
   :author: sushakov
   :date: 08-19-2013

   Hello everybody. I finally finished the code for simple features. I've implemented a `pcl::MomentOfInertiaEstimation` class
   which allows to obtain descriptors based on eccentricity and moment of inertia. This class also allows to extract axis aligned
   and oriented bounding boxes of the cloud. But keep in mind that extracted OBB is not the minimal possible bounding box.

   The idea of the feature extraction method is as follows.
   First of all the covariance matrix of the point cloud is calculated and its eigen values and vectors are extracted.
   You can consider that the resultant eigen vectors are normalized and always form the right-handed coordinate system
   (major eigen vector represents X-axis and the minor vector represents Z-axis). On the next step the iteration process takes place.
   On each iteration major eigen vector is rotated. Rotation order is always the same and is performed around the other
   eigen vectors, this provides the invariance to rotation of the point cloud. Henceforth, we will refer to this rotated major vector as current axis.

   .. image:: img/eigen_vectors.png
      :height: 360px

   For every current axis moment of inertia is calculated. Moreover, current axis is also used for eccentricity calculation.
   For this reason current vector is treated as normal vector of the plane and the input cloud is projected onto it.
   After that eccentricity is calculated for the obtained projection.

   .. image:: img/projected_cloud.png
      :height: 360px

   .. image:: img/moment_of_inertia.png
      :height: 360px

   Implemented class also provides methods for getting AABB and OBB. Oriented bounding box is computed as AABB along eigen vectors.

.. blogpost::
   :title: ROPS Progress
   :author: sushakov
   :date: 09-12-2013

   Hello everybody. I have finished implenting the ROPS feature. Next step I am going to do is to write to the authors and ask them about some
   samples of data and precomputed features, so that I could compare the result. After that I am planning to test ROPS feature for object recognition.
   For this purpose I am going to use Implicit Shape Model algorithm from PCL.

.. blogpost::
   :title: ROPS code and tutorial
   :author: sushakov
   :date: 09-19-2013

   Hello everybody. I'd like to thank Yulan Guo, one of the authors of the RoPS feature, for his help.
   I've tested my implementation against his and got the same results. I have also tested my implementation for memory leaks with the VLD and
   it works fine, no memory leaks were detected. Right now the code is ready for commit. And as always I have wrote a tutorial about using the code.
   Right now all is left is to discuss where to place the implemented code.

.. blogpost::
   :title: Recognition results
   :author: sushakov
   :date: 10-25-2013

   Hello everybody. Last few weeks I was trying to train an SVM for car recognition. For this purpose I was using some clouds that I had. These were the clouds of the city of Enschede, Netherlands, that I had manually labeled earlier. Training set consists of 401 clouds of cars and 401 cloud of the other objects (people, trees, signs etc.). As for the classifier, I was using Support Vector Machine from the libSVM library.

   During the training I was using 5-fold cross validation and the grid search in order to get the best values of gamma and soft margin C (parameters of the Gaussian kernel). The best accuracy achived during cross validation was 91.2718% with Gamma and C equal :math:`2^{-5}` and :math:`2^{13}` respectively.

   The model obtained after training was then used for recognition. The set for recognition consists of the 401 cars and 401 other objects. Training and testing sets were taken randomly from different scanned streets. The best accuracy achived this far when trying to reconize test set is 90.7731% (728 correctly recognized objects of 802).

   As for descriptors, I was using combination of RoPS feature and some global features such as height and width of the oriented bounding box. RoPS feature was calculated for the center of mass of the cloud with the support radius big enough to include all the points of the given cloud.

   Since RoPS is better fits for the purpose of local feature extraction, I believe that using it with ISM and Hough Transform voting will result in higher accuracy.

