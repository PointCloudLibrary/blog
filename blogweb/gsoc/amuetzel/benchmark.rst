.. blogpost::
   :title: GPU vs CPU Benchmarking Results
   :author: amuetzel 
   :date: 6-13-2011

   In the last days, I finally figured out why Shawn Brown's CUDA code (http://www.cs.unc.edu/~shawndb/) crashed on my PC (actually my fault), so here are some benchmarking results!
   4 implementations in total were tested: FLANN (KDTREE_SINGLE_INDEX), the existing CUDA code, my CUDA code and my CPU fallback. The tests were done on a Phenom II 955 CPU with 12GB of RAM and a GTX260 GPU.

   The main difference between the original and my CUDA code is that I don't use a stack for tree traversal; instead, I evaluate the decision about which branch to follow again when
   moving up the tree. So in the table, CUDA means Shawn Brown's code, and Stackless CUDA is my code. Both use minimal trees with rotating split axes, while FLANN tries to
   select the optimal split axes and has to store extra information about the nodes.

   I used two artificial test cases. All of them consisted of randomly distributed 3D points, but their distribution is different for each test.

   **Test 1:**
   In the first test, all the points have x,y,z coordinates in the range of [0,1[. In all tests, the number of query and search points is the same, but the points are different. (All times are in seconds.)


   +-----------------+------------------------+---------+---------+------------------------+---------+---------+
   | Algorithm       | Build Time, 10k points | 1-NN    | 16-NN   | Build Time, 100k points| 1-NN    | 16-NN   |
   +=================+========================+=========+=========+========================+=========+=========+
   | FLANN           | 0.003662               | 0.008056| 0.045166| 0.048583               |0.141357 |0.667724 |
   +-----------------+------------------------+---------+---------+------------------------+---------+---------+
   | CUDA            | 0.011676               | 0.001118| 0.014650| 0.143819               |0.014667 |0.115384 |
   +-----------------+------------------------+---------+---------+------------------------+---------+---------+
   | Stackless CUDA  | 0.057128               | 0.002929| 0.013916| 0.102539               |0.010742 |0.152343 |
   +-----------------+------------------------+---------+---------+------------------------+---------+---------+
   | CPU Fallback    | 0.057128               | 0.011230| n/a     | 0.102539               |0.138671 | n/a     |
   +-----------------+------------------------+---------+---------+------------------------+---------+---------+


   +-----------------+-----------------------+---------+---------+----------------------+---------+---------+
   | Algorithm       | Build Time, 1M points | 1-NN    | 16-NN   | Build Time, 4M points| 1-NN    | 16-NN   |
   +=================+=======================+=========+=========+======================+=========+=========+
   | FLANN           | 0.681396              | 2.213623| 8.890380| 3.179687             |10.11523 |37.945800|
   +-----------------+-----------------------+---------+---------+----------------------+---------+---------+
   | CUDA            | 1.651866              | 0.133243| 1.100975| 7.380264             |0.547088 | OOM     |
   +-----------------+-----------------------+---------+---------+----------------------+---------+---------+
   | Stackless CUDA  | 0.654541              | 0.107910| 1.694580| 3.307128             |0.468261 | OOM     |
   +-----------------+-----------------------+---------+---------+----------------------+---------+---------+
   | CPU Fallback    | 0.654541              | 1.908203| n/a     | 3.307128             |9.09375  | n/a     |
   +-----------------+-----------------------+---------+---------+----------------------+---------+---------+

   All GPU search times include transfer times, and the build time of the last two algorithms is always the same as they work on the same tree. (Also, the transfer time to the GPU is included here.)

   The 16-NN test could not be performed with my CPU fallback because it is not implemented yet. The CUDA implementations failed at the largest data set because of a simple reason: My GTX260 has 768MB of RAM, but the buffers for storing the tree and returning the results would use more than
   500 MB. The result: std::bad_alloc on allocation of the thrust::device_vector.

   **Test 2:**
   Here, the x coordinate of the search points was limited to [0,.1[, everything else remained the same. About 90% of the query points were outside the range of the search points now.


   +-----------------+------------------------+---------+---------+-------------------------+---------+---------+
   | Algorithm       | Build Time, 10k points | 1-NN    | 16-NN   | Build Time, 100k points | 1-NN    | 16-NN   |
   +=================+========================+=========+=========+=========================+=========+=========+
   | FLANN           | 0.003906               | 0.019042| 0.077636| 0.050292                |0.348632 |1.320312 |
   +-----------------+------------------------+---------+---------+-------------------------+---------+---------+
   | CUDA            | 0.011676               | 0.013418| 0.053049| 0.148135                |0.365875 |1.450399 |
   +-----------------+------------------------+---------+---------+-------------------------+---------+---------+
   | Stackless CUDA  | 0.068359               | 0.008300| 0.024902| 0.093261                |0.213378 |0.470214 |
   +-----------------+------------------------+---------+---------+-------------------------+---------+---------+
   | CPU Fallback    | 0.068359               | 0.081054| n/a     | 0.093261                |2.822753 | n/a     |
   +-----------------+------------------------+---------+---------+-------------------------+---------+---------+


   +-----------------+------------------------+---------+---------+
   | Algorithm       | Build Time, 1M points  | 1-NN    | 16-NN   |
   +=================+========================+=========+=========+
   | FLANN           | 0.705566               | 7.083007| 25.00878|
   +-----------------+------------------------+---------+---------+
   | CUDA            | 1676.332               | timeout | OOM     |
   +-----------------+------------------------+---------+---------+
   | Stackless CUDA  | 0.655761               | 11.65576| OOM     |
   +-----------------+------------------------+---------+---------+
   | CPU Fallback    | 0.655761               | 147.4165| n/a     |
   +-----------------+------------------------+---------+---------+

   In this test, FLANN was noticeably slower than the in the first test, but the CUDA results of the 1M point test were even slower than FLANN and always timed out on the 4M point test.
   Since CUDA usually kills GPU kernels that take too much time after about 4 seconds, some of the tests could not be completed.
   

   In conclusion, it is possible to say that for "uniform" data sets, the GPU implementation is faster than FLANN by a factor of about 20. But for skewed data sets like in the second test, the performance drops significantly.
   The overhead of storing explicit information about the splits might result in a speed gain on the GPU as well, even though this might also result in a worse performance due to 
   more memory accesses. So I think it might be interesting to implement and benchmark that as well.
   But first, my next step would be to test a more "real-world"-like case. This could be done by taking two kinect frames and searching for the neighbors of frame 1 in frame 2, which would be an approximation of the searches performed in the ICP algorithm.

   For this, I think I'll have to do some restructuring of my code first. At the moment, it requires that the code that calls it is also compiled via nvcc, which doesn't work for some PCL code, especially those files that include Eigen headers. Somehow nvcc doesn't like template-heavy code...
   As soon as this is done, I'll publish the code somewhere. This should be done the end of the week. I hope I can start integrating it into FLANN by then, too.
 
.. blogpost::
   :title: Real Data Benchmark
   :author: amuetzel 
   :date: 7-9-2011

   So far, I only benchmarked the algorithms on synthetic data, where the search and query points were randomly scattered inside a 3d unit cube. There, the minimal kd tree was faster that FLANN by a factor of about 20 in the 1NN case, as shown in my last benchmark post. 
   In some (unpublished) benchmarks, the FLANN kd tree ported to the GPU was always about 8 times faster than FLANN with the same random data set. 

   In this post, I'm going to show some results with more realistic data sets: I modified Nick's benchmarking code to test the kNN search of FLANN, my FLANN GPU port and my minimal kd tree on the GPU. Here, two pcd files are loaded; the first one is used to build the NN index while the points from the second one are used as query points.  All the tests used the data from the office dataset available in the PCL data repository and were performed on a Phenom II 955 CPU and a GTX260 gpu.

   For the first test, 'office2.pcd' was used both as search and query points. The result is this:

   .. image:: images/bench1_office22.png

   As you can see, with real-world data, the FLANN GPU kd tree is always faster than both FLANN on the CPU and the minimal tree. Build time is about 3% slower for the GPU FLANN algorithm than for the normal FLANN implementation, and both are about twice as fast as the minimal tree. (The GPU build times includes the upload to the GPU, and the GPU searches include the up- and download to and from the gpu.)
   The pattern of the FLANN options' build time being about twice as fast as the other build time repeated in all the tests, so it is not shown in the other benchmarks.

   The search time of the FLANN GPU algorithm is shortest; in then case of k=1, it is about 6x faster than FLANN, and 30% faster than the minimal kd tree. With k=64, the speed of both is already equal. This is because the GPU is bad at handling the complexity of maintaining the result heap used in the kNN search, which takes more and more time with increasing k.

   In the second test, 'office1.pcd' was used as the search points, while the query points came from 'office2.pcd':

   .. image:: images/bench1_office12.png

   Here, the minimal kd tree was not tested as it timed out already on k=32.  With k=1, the FLANN GPU search takes only 0.025s, while the FLANN seach takes 
   0.86s, which is 34x the GPU search time!
   In this test, the influence of a decreasing speed advantage with increasing k can be seen again. For example, with k=128, FLANN takes 19s, and the CUDA FLANN 8s. To show the differences with low values of k, the following image shows only the tests until k=16:

   .. image:: images/bench1_office12_k16.png

   The last test used 'office3.pcd' to build the tree and searched for the points in 'office4.pcd'. Here, only the FLANN and GPU FLANN results are shown, as the minimal tree started to time out on the GPU very soon.

   .. image:: images/bench1_office34.png

   Here, the GPU search time was about 10x faster for the case of k=1, but the speed advantage decreased to a factor of 7 at k=8 and 2 at k=128.

   To repeat the benchmark with your own pcd files, please download, build and install my patched FLANN from svn+ssh://svn@svn.pointclouds.org/flann/amuetzel/trunk/flann. Then, build the benchmarking code from svn+ssh://svn@svn.pointclouds.org/flann/amuetzel/trunk/benchmark.
   Finally, you can run the benchmark as "nnlib_benchmark <searchdata.pcd> <querydata.pcd>" and you'll get a nice output of the search times, plus an output text file that can be plotted using gnuplot. If you want to use the code in your own application, 

   The result did not really surprise me. I guess the main reason for the fast results is the spatial organization of the search and query points; as both are (organized) point clouds from a kinect, query points that are next to each other in the cloud array are likely also spatially close. This leads to more coherent memory accesses, which is a good thing since the speed of the search is computationally cheap, but severely limited by memory access performance.
   The slow memory accesses also lead to the huge slowdowns with large values of k. There is not enough (fast) on-chip memory to store the neighbor heaps, so they have to be stored in (slow) GPU RAM. Thus, inserting new neighbors becomes expensive, as the data often has to be moved around in the heap structure.
   So as a conclusion, I think I can say that if you need few nearest neighbors, use your GPU, but if you need many nearest neighbors or have some spare CPU cores to parallelize the search, use the CPU.

   If you test the code, I would really appreciate it if you tell me about the results, along with your system specs (GPU+CPU)! Of course, if you have any trouble building it, you can also contact me ;-) I would be especially interested in any benchmarks on Fermi-series cards, to see if the cache found on these cards improves performance.

   Some more benchmarks will follow, for example about the approximate NN search and with some more other data sets.


     