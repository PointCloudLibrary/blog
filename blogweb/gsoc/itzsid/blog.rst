  
.. blogpost::
   :title: Work status
   :author: itzsid
   :date: 8-6-2011

   Lately I have been working on integrating octree gpu functions with the search class. Since GPU functions can only have an advantage when we have queries in batch mode. For that a module as mentioned in my previous post is written which taken a vector of queries as an input and calls the appropriate gpu function. Right now, the calling gpu function is not working correctly which I am still working on. 


.. blogpost::
   :title: Octree GPU functions
   :author: itzsid
   :date: 7-24-2011

   I started with reading octree search functions for probable porting of them onto GPU. For this I have been going through the GPU functions written by Anatoly for the last few days. Other than this I have added two functions which would act as an interface to CUDA search functions.
   
   .. code-block:: none

      virtual int
      radiusSearchGPU (std::vector<const PointT>& point, const double radius, std::vector<std::vector<int> >& k_indices,    std::vector<std::vector<float> >& k_distances, int max_nn = -1) const;

      virtual int
      nearestKSearchGPU (std::vector<const PointT>& point, int k, std::vector<std::vector<int> >& k_indices,    std::vector<std::vector<float> >& k_sqr_distances);


   Keeping in mind that GPU power can only be leveraged when multiple query search is done, I have used a vector of point as an input to the function. Next I'll try and implement some octree search functions onto GPU.

  
.. blogpost::
   :title: Work status
   :author: itzsid
   :date: 7-16-2011

   Last week I refined various interface functions to different search methods. deleteTree() and addPointsfromInputCloud() functions in Octree class were merged into setInputCloud(). The functions inside OrganizedDataIndex class were merged into OrganizedNeighborSearch with the function names asapproxNearestKSearch() and approxRadiusSearch(). Other than these as suggested to include a function to select the best search method on the basis of input cloud, I included a new method as,

   .. code-block:: none
   
      Search<PointXYZ>* search = new AutotunedSearch<PointXYZ>(AUTO_TUNED);
      search->evaluateSearchMethods(cloudIn, NEAREST_K_SEARCH|NEAREST_RADIUS_SEARCH);

   It can compare different datastructure on the bais of run time for the given search method in the second argument. Next I'll start with interfacing GPU functions in search class and put some CPU-GPU checks in auto-tuned search class.

.. blogpost::
   :title: Integration of Kdtree, Octree and OrganizedNeighborClass into search base class
   :author: itzsid
   :date: 7-9-2011
   
   I have done the initial integration of all search classes into one search base class. The class structure followed till now is:

   .. code-block:: none

      class Search
      class OctreePointCloud : public Search
      class Kdtree : public Search
      class OrganizedNeigbhorSearch: public Search
      class AutotunedSearch : public Search 

   So, the appropriate search functions can be called as:

   **KdTree:**

   .. code-block:: none

      Search<PointXYZ>* kdtree = new KdTree<PointXYZ>(); 
      kdtree->setInputCloud(cloudIn);
      kdtree->nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);
      kdtree->radiusSearch (test_point, radius, k_indices, k_distances);

   **Octree:**

   .. code-block:: none

      Search<PointXYZ>* octree = new OctreePointCloud<PointXYZ>(0.1f); 
      octree->setInputCloud(cloudIn);
      octree->addPointsFromInputCloud ();
      octree->nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);
      octree->radiusSearch (test_point, radius, k_indices, k_distances);

   **OrganizedNeighborSearch:**

   .. code-block:: none

      Search<PointXYZ>* organized_index = new OrganizedNeighborSearch<PointXYZ>(); 
      organized_index->setPrecision(1);  // 1 for using OrganizedNeighborSearch functions and 0 for using OrganizedDataIndex functions      
      organized_index->setInputCloud(cloudIn);
      organized_index->nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);
      organized_index->radiusSearch (test_point, radius, k_indices, k_distances);
   
   **Auto-Tuned Search:** 
   For now AutotunedSearch takes the appropriate datastructure as a parameter to initiate the search function.

   .. code-block:: none

      Search<PointXYZ>* auto_search = new AutotunedSearch<PointXYZ>(KDTREE_FLANN); // or OCTREE, ORGANIZED_INDEX
      auto_search->setInputCloud(cloudIn);
      auto_search->nearestKSearch (test_point, no_of_neighbors, k_indices, k_distances);
      auto_search->radiusSearch (test_point, radius, k_indices, k_distances);
	
   The above set of functions are implemented and pushed into trunk. Right now, I am working on AutotunedSearch class to make it more user friendly.
 
   

.. blogpost::
   :title: Benchmarking OrganizedNeighborClass and OrganizedDataIndex using office dataset
   :author: itzsid
   :date: 7-3-2011

   I tried benchmarking the search functions in OrganizedNeighborClass and OrganizedDataIndex classes using the office dataset which Julius gave. The dataset is available at: http://svn.pointclouds.org/data/office/ . 

   Following are the time benchmark tests done on radiusSearch functions for OrganizedNeighborSearch and OrganizedDataIndex. The number of nearest neighbors for both classes is the same for most of the experiments. However the indices of nearest neighbors vary for both the classes. In some cases the number of nearest neigbhors is also different for both the classes but they are nearby which suggests that OrganizedDataIndex might be an approximate version of OrganizedNeighborSearch. The search point is decided at random ensuring that the search point is not NaN. The time calculated for each search radius is average of 100 iterations.

   **Data:** office1.pcd

   **Search Point:** -1.16809, 0.0467238, 4.906

   +--------+-----------------+----------------+-------------------+
   | Search | Organized       | Organized      | Number of         |
   | Radius | Neighbor Search | Data Index     | Nearest Neighbors |
   +========+=================+================+===================+
   | 0	    | 0.0511418	      | 1.3416e-06     |              1    |
   +--------+-----------------+----------------+-------------------+
   | 0.1    | 0.00958581      | 3.3005e-05     |   1172            |
   +--------+-----------------+----------------+-------------------+
   | 0.2    | 0.0102894	      | 0.000175295    |   3917            |
   +--------+-----------------+----------------+-------------------+
   | 0.3    | 0.0118625	      | 0.000422468    |   7432            |
   +--------+-----------------+----------------+-------------------+
   | 0.4    | 0.0148702	      | 0.000436519    |   9336            |
   +--------+-----------------+----------------+-------------------+
   | 0.5    | 0.0157803	      | 0.000821651    |  11850            |
   +--------+-----------------+----------------+-------------------+
   | 0.6    | 0.0166538	      | 0.00117252     |  14238            |
   +--------+-----------------+----------------+-------------------+
   | 0.7    | 0.0177392	      | 0.00146132     |  16663            |
   +--------+-----------------+----------------+-------------------+
   | 0.8    | 0.0187914	      | 0.0017615      |  21237            |
   +--------+-----------------+----------------+-------------------+
   | 0.9    | 0.0198163	      | 0.0020671      |  25461            |
   +--------+-----------------+----------------+-------------------+

   **Data:** office2.pcd

   **Search Point:** -0.723531, 0.218086, 1.347

   +--------+-----------------+----------------+-------------------+
   | Search | Organized       | Organized      | Number of         |
   | Radius | Neighbor Search | Data Index     | Nearest Neighbors |
   +========+=================+================+===================+
   | 0.000  | 0.09484         | 0.00000        |      1            |
   +--------+-----------------+----------------+-------------------+
   | 0.100  | 0.01409         | 0.00022        |   1743            |
   +--------+-----------------+----------------+-------------------+
   | 0.200  | 0.01513         | 0.00045        |   3739            |
   +--------+-----------------+----------------+-------------------+
   | 0.300  | 0.01823         | 0.00091        |  14067            |
   +--------+-----------------+----------------+-------------------+
   | 0.400  | 0.02444         | 0.00195        |  25933            |
   +--------+-----------------+----------------+-------------------+
   | 0.500  | 0.02855         | 0.00289        |  44137            |
   +--------+-----------------+----------------+-------------------+
   | 0.600  | 0.03438         | 0.00409        |  63127            |
   +--------+-----------------+----------------+-------------------+
   | 0.700  | 0.04080         | 0.00531        |  74889            |
   +--------+-----------------+----------------+-------------------+
   | 0.800  | 0.05044         | 0.00649        |  92927            |
   +--------+-----------------+----------------+-------------------+
   | 0.900  | 0.05616         | 0.00811        | 111587            |
   +--------+-----------------+----------------+-------------------+


   **Data:** office3.pcd

   **Search Point:** 0.0144343, -0.43784, 1.263

   +--------+-----------------+----------------+-------------------+
   | Search | Organized       | Organized      | Number of         |
   | Radius | Neighbor Search | Data Index     | Nearest Neighbors |
   +========+=================+================+===================+
   | 0.000  | 0.07388         | 0.00000        |      1            |
   +--------+-----------------+----------------+-------------------+
   | 0.100  | 0.01170         | 0.00012        |   4538            |
   +--------+-----------------+----------------+-------------------+
   | 0.200  | 0.01426         | 0.00061        |  13565            |
   +--------+-----------------+----------------+-------------------+
   | 0.300  | 0.01612         | 0.00110        |  24162            |
   +--------+-----------------+----------------+-------------------+
   | 0.400  | 0.01890         | 0.00196        |  36527            |
   +--------+-----------------+----------------+-------------------+
   | 0.500  | 0.02515         | 0.00354        |  53609            |
   +--------+-----------------+----------------+-------------------+
   | 0.600  | 0.03459         | 0.00555        |  73626            |
   +--------+-----------------+----------------+-------------------+
   | 0.700  | 0.05051         | 0.00613        |  92836            |
   +--------+-----------------+----------------+-------------------+
   | 0.800  | 0.06044         | 0.00957        | 152082            |
   +--------+-----------------+----------------+-------------------+
   | 0.900  | 0.06897         | 0.01178        | 199561            |
   +--------+-----------------+----------------+-------------------+


   **Data:** office4.pcd

   **Search Point:** -0.447771, 0.0920419, 1.306

   +--------+-----------------+----------------+-------------------+
   | Search | Organized       | Organized      | Number of         |
   | Radius | Neighbor Search | Data Index     | Nearest Neighbors |
   +========+=================+================+===================+
   | 0.000  | 0.04692         | 0.00000        |      1            |
   +--------+-----------------+----------------+-------------------+
   | 0.100  | 0.00948         | 0.00009        |   3093            |
   +--------+-----------------+----------------+-------------------+
   | 0.200  | 0.00970         | 0.00038        |  12636            |
   +--------+-----------------+----------------+-------------------+
   | 0.300  | 0.01246         | 0.00083        |  28916            |
   +--------+-----------------+----------------+-------------------+
   | 0.400  | 0.01408         | 0.00162        |  48606            |
   +--------+-----------------+----------------+-------------------+
   | 0.500  | 0.01939         | 0.00282        |  75014            |
   +--------+-----------------+----------------+-------------------+
   | 0.600  | 0.02332         | 0.00395        | 102830            |
   +--------+-----------------+----------------+-------------------+
   | 0.700  | 0.03006         | 0.00550        | 131138            |
   +--------+-----------------+----------------+-------------------+
   | 0.800  | 0.03190         | 0.00629        | 154212            |
   +--------+-----------------+----------------+-------------------+
   | 0.900  | 0.03357         | 0.00718        | 168709            |
   +--------+-----------------+----------------+-------------------+


   For now, I have integrated OrganizedDataIndex into OrganizedNeighborSearch with a setMethod() function and pushed the same into trunk. I have started with the integration of Octree function into search base class.  




.. blogpost::
   :title: Comparing OrganizedNeighborClass with OrganizedDataIndex
   :author: itzsid
   :date: 6-23-2011
  
   During the last few days I integrated OrganizedNeighborClass into the base search class and pushed the same into trunk. The functions inside OrganizedDataIndex is merged into OrganizedNeighborClass. 
   OrganizedNeighborClass performs the search operation by projecting the search sphere onto the disparity image and then do a 2d search in projected search space. OrganizedDataIndex performs the same search operation by projecting the search point onto 2d and then defining the search space. So, in the first case we have a reduced projected search space with a overhead of projection computation whereas in the second case the 2d search space is bigger but with no projection computation overhead. 
   To decide which method would work best on organized data, we do some benchmark tests by comparing the time taken by radiusSearch for both methods. 

   Here are some of the results:

   +--------------------------+------------------------+-----------------------+---------------------------+
   | Point Cloud Resolution   | Search Radius          | OrganizedDataIndex    | OrganizedNeighborSearch   | 
   +==========================+========================+=======================+===========================+
   | 640 x 480                | 0.0249623              | 0.00160503            | 0.00819993                |
   +--------------------------+------------------------+-----------------------+---------------------------+
   | 640 x 480                | 0.73077                | 0.000736952           | 0.00690317                |
   +--------------------------+------------------------+-----------------------+---------------------------+
   | 640 x 480                | 0.861978               | 0.000688791           | 0.00860214                |
   +--------------------------+------------------------+-----------------------+---------------------------+
   | 1024 x 768               | 0.0519982              | 0.00165892            | 0.0255649                 |
   +--------------------------+------------------------+-----------------------+---------------------------+
   | 1024 x 768               | 0.367919               | 0.00194287            | 0.019875                  |
   +--------------------------+------------------------+-----------------------+---------------------------+
   | 1024 x 768               | 0.809822               | 0.00166297            | 0.0175359                 |
   +--------------------------+------------------------+-----------------------+---------------------------+

   K Nearest Neighbor module is not working fine, which I hope to debug soon. 

   Plans for next few days include,
   
   * Committing the integrated code into trunk
   * Benchmarking organized search functions with kinect data
   * Start with the integration of Octree functions


.. blogpost::
   :title: Work Status 
   :author: itzsid
   :date: 6-17-2011
   
   Committed base search class into trunk inheriting KdTree class with all the kdtree unit tests working fine. Next I am working on integrating OrganizedNeighborSearch class into base class. For now, it is giving some errors in certain unti tests, which hopefully would be debugged soon. 

   So, during next week, my plans are to complete the integrating of OrganizedNeighborSearch class into the base search class, commit it and start looking at Octree implementation. 


.. blogpost::
   :title: Base Class to KdTree inherited class 
   :author: itzsid
   :date: 6-13-2011

   Last week I started coding the base search class, with the initial focus on KdTree functions to be inherited by the base class. The basis construction of the generic class is almost done and I hope to commit it after some final tweakings. 

   After this I would be looking at the functions in OrganizedDataIndex and OrganizedNeighborSearch classes which will be interfaced by base search class.



.. blogpost::
   :title: Structure of the generic search interface class 
   :author: itzsid
   :date: 6-8-2011
 
   After some initial problems in designing the basic interface and properly using the boost pointers along with some discussions on the developers list, the basic interface for the search functions is working good.

   So, there would be one Search base class and three inherited classes for kdtree, octree and organized search. The base class would have a class pointer, which would in accordance point to the appropriate child class during initialization. The functions available to the user in the search interface would be defined as virtual functions which would be overriden by similar named functions in inherited classes. 

   I hope to complete the common interface for kdtree and organized search by the next week, after which I can move on to octree.    
   
   

.. blogpost::
   :title: Progress status on 28th May 
   :author: itzsid
   :date: 5-28-2011

   The work done till now includes,
	
	* Downloaded and compiled the complete code successfully and run a few tests and it all went smoothly
	* Understood the basis tutorials of reading/writing of PCD files and some advanced tutorials pertaining to the octree data structure. Even tested a few codes regarding the nearest neighbour search on Octrees.

   Currently I am reading the implementation of nearest neighbour search operations in octree and kdtree and trying to identify the relevant parameters in reference to the nearest neighbour operation. 	 


.. blogpost::
   :title: My first status update
   :author: itzsid
   :date: 5-24-2011

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

