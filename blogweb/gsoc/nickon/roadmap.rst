Roadmap
=======
.. _nickon_roadmap:

Here is a detailed outline of my GSoC project milestones:

  * Get familiar with the FLANN, linabo and nnn nearest neighbor search libraries

    - FLANN - http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN
     
      - is currently used in PCL for k-nn search and for radius-search
      - contains several index types, of special interest is the low dimensionality kd-tree index (KDTreeSingleIndex)
		
    - libnabo - https://github.com/ethz-asl/libnabo
    - nnn - http://www.ros.org/wiki/nnn

      - fast radius search in 3D implemented by Garratt

  * Create automated benchmarking framework that allows easy benchmarking of different libraries, on many datasets and with different search parameters. Use this framework for benchmarking the above libraries.

    - Each of the above libraries has strengths and weaknesses, there are datasets for which each of them can be the fastest
    - We would like to have a unified 3D search library that has the advantages of all of them and it's as fast as the fastest of them for any data

  * Integrate libnabo and nnn in FLANN

    - Using the benchmarking results determine the cases when libnabo, nn and/or FLANN's KDTreeSingleIndex are faster and optimize/rewrite/merge the 3 algorithms into a unified one that will replace the KDTreeSingleIndex in FLANN as the best CPU-based 3D search algorithm
    - Document and write a tutorial on how to use the new index

  * Experiment with speeding up nearest neighbor search using multithreading

    - FLANN search is thread safe, but we didn't experiment much with multithreading search
    - Determine what speedups can be achieved (if any) using multithreading and what's the optimum number of threads to use for search? 
    - Optimizations that can be made to make the algorithm more efficient in multithreading environments?

  * KDtree on GPU

    - Examine and existing implementation of kd-tree on GPU: http://www.cs.unc.edu/~shawndb/
    - Benchmark the performance of this compared to the unified CPU kd-tree
    - Apply lessons learned from the unified kd-tree algorithm to create a GPU kd-tree implementation (or to optimize an existing one)

