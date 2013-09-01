Roadmap
=======
.. _itzsid_roadmap:

Many algorithms in the PCL heavily rely on neighbor search operations. Their performance critically depends on
the speed of neighbor search methods. Hence increasing the speed of neighbor search operations has the ability to
significantly improve the overall performance of the Point Cloud Library. Two methods based on spatial decomposition are
often used for fast neighboring search in multidimensional space: octrees and kd-trees. Furthermore, in case of so call
organized point clouds, an efficient neighbor search is also possible in the two dimensional disparity image. All these
search techniques and hierarchical data structures are fully integrated into the PCL.

Interestingly, the performance of these neighbor search routines depend on many aspects. First of all the statistical
distribution of the point set have a strong influence on the search speed. Furthermore, there is a trade-off between the
building time of the search structures and the actual search operation. In other word, the amount of performed search
operations also influences the best suited search technique.

To address this issue, we would like to integrate a dedicated neighbor search component into PCL. It should provide a
standard interface to all search methods that are currently included in PCL. Furthermore, as the search performance
depends on so many parameters and properties, profiling and testing functionality of different parameter sets becomes
highly interesting. This could allow to detect the fastest search configuration during the initialization of the
library. According to the results of such performance evaluation, the fastest methods can be then automatically
selected, transparent from the user and the application.

Once such dedicated search component is available, the next step is to focus on improving the performance of the
individual search techniques itself. Particularly, promising in this context is to port these algorithms to a GPU and
parallelize the search operation. In this context, the octree-based neighbor search routines should be investigated and
ported to a CUDA implementation.

Goal: 
-----
Improving the overall speed of neighbor search operations for PCL, independent from point cloud characteristics
and application scenario.

Milestones:
~~~~~~~~~~~~~~~
  * Design a general framework for a search component to be integrated into PCL. It should be an interface to all currently included neighbor search routines.  

  * Define methods and parameter sets for performance evaluation of neighbor search in point clouds

  * Develop and integrate methods for an automatic search method selection based on the characteristics of point clouds and amount of planned search operations

  * Developing of octree-based neighboring search in CUDA

  * Integrate CUDA-based neighbor search into the search component 

Getting started: 
~~~~~~~~~~~~~~~~

  * Get familiar with PCL

    * Create an account on dev.pointclouds.org

      * Send public SSH key to Radu
      * Download the PCL source tree

        * (``svn co svn+ssh://svn@svn.pointclouds.org/pcl/trunk``)

    * Read, compile, experiment with several tutorials

  * Set up the project website/status reports

    * Install Sphinx (instructions :ref:`here <How-to-install-Sphinx>`)
    * Fill out the :doc:`index <index>` page
    * Write an introductory status update (instructions :ref:`here <How-to-write-a-blog-post>`)

  * Start writing code for reading point clouds from files and measure the speed of nearest neighbor search operations

  * Try to identify important and relevant parameters influencing the search performance 



