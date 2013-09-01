.. blogpost::
  :title: My first status update
  :author: pararthshah
  :date: 5-23-2011

   Today I learned how to add content to the developer blogs.

   Here's a code snippet:

   .. code-block:: c

    	// This is a really boring block of code...
    	int n = 10;
    	for (int i = 0; i < n; ++i)
      {
        printf ("%d\n", i);
      }
   
   And here's an equation:

   .. math::

      ax^2 + bx + c = 0

      
.. blogpost::
   :title: First Week In
   :author: pararthshah
   :date: 5-30-2011

   One week into the coding period, the work I have completed includes:
   
   - Downloaded, compiled and installed PCL library and dependencies.
   - Gone through majority of tutorials, and successfully executed them on my machine.
   - Started prelimnary work on my first task, developing a benchmarking utility for Feature Correspondence and Registration Tests.
     (See project roadmap for details)
     
   For Feature Correspondence Tests, I have gone ahead with the following design:
   
   FeatureCorrespondenceTest is a base class that implements high level tasks for benchmarking. 
   To test any specific algorithm, derive a new class from this base and implement functions for:
   
   1. Taking algorithm parameters as input.
   2. Computing feature descriptors using that algorithm.
   
   Use an object of this derived class in the main function to run the tests.
   
   The reason I went ahead with this is that each feature descriptor algorithm can have a unique number and types of parameters, 
   and the exact procedure of computing the feature descriptors may differ to an extent that cannot be captured solely by use of templates.
   So each algorithm should have a separate class for testing it, while the common functionality is implemented in the base class.  

   The source code is over :ref:`here <feature_test>`.
   
   This is still a prelimnary version of the utility, to test the general design. 
   I will be adding more functionality and addressing robustness issues in this week. Any ideas/comments/suggestions for this are highly welcome.


.. blogpost::
   :title: Feature Evaluation Framework
   :author: pararthshah
   :date: 6-07-2011

   After discussing the basic structure of the Feature Correspondence Test and Feature Evaluation Framework classes with Michael, I have been working on the class implementations, and getting them to run bug-free on a toy program. I hope to get a basic version of these classes finished in the next two days.
   
   The important points I would like to mention are:
   
   - Modified the format of ground_truths of the feature test, to accept Eigen::Matrix4f as a transformation matrix of the source input points into the corresponding points.
   - Read up about homogenous transformations from point of view of 4x4 matrices, to get a better understanding of how it should work out in my code.
   - Worked on integrating a Framework class with the FeatureCorrespondenceTest class, to automate the testing of multiple algorithms with different parameters.
   
   Once I get hold of the benchmark datasets, I will run some actual tests on the code to eliminate runtime bugs, and upload it here subsequently.
   
   Next task would be to write up the Registration Test Framework and RegistrationTest base class, which would be very similar to the present task, at least at an abstract level.

   
.. blogpost::
   :title: Evaluating Feature Descriptor Algorithms
   :author: pararthshah
   :date: 6-15-2011

   Finally, I have finished integrating a Feature Evaluation Framework class for testing multiple feature desciptor algorithms, on multiple datasets, with multiple sets of parameters.
   The code for it is :ref:`here <feature_evaluation_framework>`.
   
   To test any algorithm, first a Test class for that algorithm must be derived from the :ref:`FeatureCorrespondenceTest <feature_test>` class.
   Then link the algorithm with the FeatureEvaluationFramework class, so that it can be tested on user provided datasets and parameters.
   
   I have written a :ref:`toy program <test_features>` to test FeatureEvaluationFramework class. It uses FPFHTest to run the FPFHEstimation algorithm on two sample datasets, using three sets of parameters.
   
   The output of the test is currently printed to std::out in the following manner::
   
	   $ ./test_feature 
	   ----------Test Details:----------
	   Feature Name:  FPFHEstimation
	   Input Dataset: bun0.pcd
	   Parameters:    threshold=0.01, searchradius=0.03
	   ----------Test Results:----------
	   Input Size:    397
	   Successes:     397
	   Failures:      0
	   ---------------------------------
	
   
   :ref:`Click here <test_output>` to see the complete output. 
   
   Next steps would be:
   
      - Deciding a good format for storing and visualizing the test results.
      - Using the class to run tests on the `conference room benchmark dataset <http://people.willowgarage.com/mdixon/benchmarks/conference_room.tar.bz2>`_ provided by Michael.
      - Uploading the tested code to the repository, and the test results to PCL website and/or repository.
      - Going ahead with the Registration Framework, in a similar manner.


.. blogpost::
   :title: Effect of Downsampling on Feature Computations
   :author: pararthshah
   :date: 6-24-2011

   This week I have been running tests on the benchmark dataset using the FeatureEvaluationFramework class.
   
   As suggested by Michael, I have added functionality to the Framework class to preprocess the input clouds, i.e. downsample them using VoxelGrid filter, to reduce the running times of feature computations.
   
   For this, I have modified the Framework class, as well as the FeatureCorrespondenceTest class. The major changes to code are: (reflected :ref:`here <evaluation_framework_6_24>`)
   
      - Added functions to control preprocessing of input clouds, using VoxelGrid filter to downsample the clouds.
      - Restructured the functions for running the tests, added a function to run tests over a range of leaf sizes (of VoxelGrid filter).
      - Added a minimal TestResult class to store the results of each test case as a separate object. Functions can be (will be) added to this class for stuff like printing to CSV, publishing to ReST, plotting graphs, etc.
      
   I used this class to run FPFHEstimation algorithm on a sample dataset (cloud_000.pcd) for various values of the leaf sizes. Here are the results:
   
   **Feature Name:** FPFHEstimation
   **Parameters:**   threshold=0.01, searchradius=0.003
   **Dataset:**      cloud_000.pcd
   **Input size:**   307200
   
   **Machine Config** Intel Core 2 Duo P8700 @ 2.53 GHz, 4GB RAM, on Ubuntu 10.10
   
   **Testcases:**

   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | Leaf size       | Preprocessed  | No. of Successful  | Time Taken For     | Time Taken For    | Total Time For      |
   |                 | Input Size    | Correspondences    | Source Features    | Target Features   | Feature Computation |
   +=================+===============+====================+====================+===================+=====================+
   | 0.5             | 28            | 1                  | 0                  | 0                 | 0                   |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.1             | 369           | 56                 | 0                  | 0                 | 0.01                |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.05            | 1232          | 1219               | 0.01               | 0.02              | 0.04                |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.01            | 22467         | 22465              | 3.18               | 3.21              | 6.78                |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.007           | 40669         | 40667              | 10.69              | 10.74             | 22.47               |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.005           | 69912         | 69910              | 31.86              | 31.82             | 66.42               |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.001           | 234228        | 234226             | 671.67             | 674.85            | 1390.69             |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   | 0.0007          | 235729        | 235727             | 729.75             | 722.02            | 1497.79             |
   +-----------------+---------------+--------------------+--------------------+-------------------+---------------------+
   
   **Note:** In case of FPFHEstimation, the total time taken for Feature computation includes the time for calculating normals of the input points.
   
   Anyone volunteering to provide benchmark results using this class (alongwith you machine config) is highly appreciated.


.. blogpost::
   :title: Finalising The Feature Test Class
   :author: pararthshah
   :date: 7-01-2011

   This week I have been focusing on finalising the Feature Test class, to be used for benchmarking of Feature Descriptor algorithms. The class supports the following pipeline:
   
      - loadData (source and target clouds, ground truth)
      - setThreshold (either a single threshold value, or a threshold range, specified by lower bound, upper bound, and delta)
      - setParameters (specific to the Feature Descriptor algorithm, given as a "key1=value1, key2=value2, ..." string)
      - performDownsampling (filter input clouds through VoxelGrid filter, with specified leaf size)
      - extractKeypoints (extract keypoints from the downsampled clouds)
      - computeFeatures (compute features of the keypoints if extracted, or else on the preprocessed clouds)
      - computeCorrespondences (match the source and target features) (or) computeTransformation (register the source and target clouds)
      - computeResults (evaluate total successes and/or runtime statistics)
      
   Important tasks include:
   
      - The FeatureEvaluationFramework class should support running multiple tests over a single independent variable, eg input clouds/ threshold/ parameters/ leaf size, etc
      - The output of each set of runs should be published in CSV format, which can be plotted in a graph/table
      - An executable "evaluate_feature" located in "trunk/test/" should allow running of multiple tests by choosing an independent variable and other input values, through the command line
      - It should also support quickly running some predefined standard tests on Feature algorithms, to compare the results of newly implemented algorithms with previous ones


.. blogpost::
   :title: Extracting Keypoints And Performing Registration
   :author: pararthshah
   :date: 7-06-2011
   
   I have completed the Feature Evaluation Framework class, except for (i) integrating keypoint extraction, and (ii) registration of source and target clouds.
   Adding these functionalities will require some reading by me, which I plan to do currently.
   
   Also, I am playing with various methods of visualizing the output of the FeatureCorrespondenceTests, and I intend to first run a series of tests on Feature algoithms to gather
   a set of benchmarking results and then devise a way to visualize it.
   
   I am stuck up with a minor problem involving taking the ground truths as input, specifically converting an Eigen::Vector3f and Quaternion to a Eigen::Matrix4f representation.
   Hopefully I'll find something useful in the Eigen documentation.


.. blogpost::
   :title: Commandline Tool For Benchmarking Features 
   :author: pararthshah
   :date: 8-07-2011
   
   Sorry for the late post, but the progress since my last post includes a commandline tool for benchmarking feature descriptor algorithms, 
   which I have added under /trunk/test/ 
   
   On a related note, I have written a simple tool for extracting features from given pointcloud, which is under /tools/extract_feature.cpp. Currently
   it supports the PFH, FPFH and VFH algorithms, and I will be adding more. An issue I am facing is of how to dynamically select the point_type of the
   input cloud, depending on the input PCD file, and currently it is converting all input clouds into PointXYZ types.

.. blogpost::
   :title: Tutorial For Using The Benchmarking Class
   :author: pararthshah
   :date: 8-15-2011
   
   I have added a tutorial which explains the functionality of the FeatureEvaluationFramework class, for benchmarking feature descriptor algorithms.
   Also, the tutorial explains a sample use case, which is "determining effect of search radius on FPFHEstimation computations".
   
   I have mentioned how the Framework can be extended to include testing of many (all?) feature algorithms, quite easily. 
   