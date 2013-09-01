My status updates
=================

.. blogbody::
   :nr_days: 30
   :author: nickon


.. blogpost::
   :title: Multi tasking
   :author: nickon 
   :date: 6-02-2011

   As a student from Belgium, at the moment I'm in a difficult period as it is the start of GSoC and the start of my exam period... To not let the university snoop any more time of my GSoC coding period, I will be doing my best to pass my exams in the 1st session so I won't have to do re-examinations. I hope Murphy is on my side on this :)

   I will be trying to be as active as possible during the exam period, so I can really get it going when my exams are finished. At the moment I installed Ubuntu 10.04 LTS and as you can see I've posted my first blog update. I have been discussing some design considerations with Marius and Andreas by email regarding the interface adaptations FLANN will need to undergo to allow the maximum degree of parallelization.

   What is on my planning for the next days is study for my exam (Bussines Skills, wish me luck :D) and to try and get PCL, nnn, libnabo and FLANN to compile succesfully on Linux.


.. blogpost::
   :title: (Exam?) progress report
   :author: nickon 
   :date: 6-09-2011

   Not much has happened since last week, if you do not count the gain in bussines skills I have acquired during the last couple of days ;) Unfortunately, but as expected, I've been really busy with studying for my exams so far.. T minus 1 day before I have to go and take my first exam!

   After that, it's another burdon that will drop off my shoulders, and I might have some mental power left to fiddle a bit with compilation of all the different libraries on Linux and Windows. I'll post another status update when we come to that point :)


.. blogpost::
   :title: More exams...
   :author: nickon 
   :date: 6-21-2011

   They don't seem to end do they... Finished my exam for Digital Electronics today, 5 hours of intensive digital circuit drawing and design, my brain is fried!

   But now for GSoC achievements:

   The past evenings, I found some time to spend on GSoC, so I've typed my internship plan and Radu signed it. Thanks Radu :)

   I've also been trying to get the current CUDA implementation that Andreas developed up till now to compile on Windows. As I have a CUDA optimus enabled device, drivers for linux aren't supported. So all CUDA development work will be on Windows for me. It's not that bad actually, because the CUDA implementation should run on Windows to, so it's a good thing to get all possible cross-platform errors fixed as early in the development stage as we can.

   After fixing a decent amount of build errors, there is still one build issue that keeps me from compiling the implementation, and we don't seem to get rid off. Its the CUDA compiler that won't compile source files where the flann header gets included. Strangely Andreas has the same version of the compiler and on linux it doesn't complaint. Together we are trying to figure out how to solve this tedious issue.


.. blogpost::
   :title: Bye bye exams! Welcome GSoC!
   :author: nickon 
   :date: 6-30-2011

   Finally my exams are done! Eagerly waiting for my result won't be an option this year, as GSoC will keep my mind on some more important stuff!

   I got all libraries (libnabo, nnn, FLANN) to build on Ubuntu 10.04LTS and am ready to benchmark their performance. So, my primary focus now will be on the benchmarking of the libraries and find the possible sweet spots of each. You'll find out more in my next blog post :)


.. blogpost::
   :title: Benchmarking results of NNN vs FLANN
   :author: nickon 
   :date: 7-6-2011

   I've done some extensive benchmarking of the `NNN (Naive Nearest Neighbor) library <http://www.ros.org/wiki/nnn>`_ and the library for nearest neighbor searching currently used in PCL, namely `FLANN (Fast Library for Approximate Nearest Neighbors) <http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN>`_.

   NNN provides 3 ways of doing a radius search for the nearest neighbours, each more naive then the other. Naive isn't always a bad thing here, the more naive the method, the less overhead it incurs. For example the most naive method of NNN doesn't need to build a data structure for the radius search, but just calculates the results directly. The two other naive methods are the SplitCloud and SplitCloud2 methods, which respectively subdivide the provided cloud into 8/64 smaller (overlapping!) subclouds. Radius search can then be performed on each of these (smaller! **[1]**) subclouds, such that the radius search is contained within that cloud (because of the overlapping construction of the subclouds). Meaning that during the search in one particular subcloud, no other subclouds need to be visited.

   As I've said, the benchmarking is quite extensive, and it has cost me quite some time to collect these results. Although such results provide a good foundation for drawing conclusions, I'll keep the time spent in mind for the benchmarking of the `libnabo library <https://github.com/ethz-asl/libnabo>`_ and probably will try to keep the benchmarking more brief than this one.

   I'll present here some graphs I created from the benchmarking numbers accompanied with my conclusions. Feel free to look at the fully detailed results, which can be found by clicking `here <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/NNNvsFLANN/benchmark%20NNN%20v.s.%20FLANN%20(random%20query%20point).htm>`_.

   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/nnnvsflann0.5.png
   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/nnnvsflann1.png
   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/nnnvsflann2.png

   The first three graphs show the search time of both the standard NNN (without splitting the cloud, so no build time) and FLANN for radiuses = 0.5, 1 and 2 on a cloud of points randomly generated in the interval [0,1]. So the radius is respectively underfitting, nearly fitting and overfitting all points in the point cloud. We can see that we start benefiting from the standard NNN approach as soon as the radius becomes pretty large in comparison with the point cloud, a point where FLANN starts performing worse.

   The case where the radius = 2 and thus is overfitting all the points, is just to show that there is a standard NNN method that has an optimization which tries to avoid calculating the euclidean distance, hence not being able to return the distance of each point to the query point. In some cases this optimization can have an impact on the time spent on the radius search. In this particular case when the radius is overfitting the points in the point cloud, the optimization will be exploited maximally, resulting in a significant better time for the radius search.

   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/splitcloud0.10.png
   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/splitcloud0.25.png
   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/splitcloud0.50.png
   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/splitcloud1.png

   Next I compared the SplitCloud and SplitCloud2 methods with the standard NNN search method. A similar optimization as in the standard NNN can be applied, but it is not represented in the graphs, as the SplitCloud and SplitCloud2 method only incur overhead from this approach, resulting in times that are far worse than without "optimization".

   As said before, the SplitCloud and SplitCloud2 method rely on splitting the cloud in respectively 8/64 subclouds, taking into account the radius. We can see that for smaller radiuses, thus radiuses not containing a large quantity of the points in the cloud, splitting makes sense. This is intuitive, as for larger radiuses we don't benefit from splitting the cloud in almost entirely overlapping subclouds. However in the cases where SplitCloud and SplitCloud2 can make a difference to the standard NNN (radius < 1), FLANN is still the faster option regarding radius search times.

   For the case where the radius = 1, SplitCloud and SplitCloud2 perform approximately equally well as the standard NNN, of which we know from the preceding figures that for a radius = 1 it outperforms FLANN.

   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/buildtimesc.png
   .. image:: BenchResults/NNNandLibnabo/NNNvsFLANN/buildtimesc2.png

   However FLANN "dominates" **[2]** the SplitCloud methods in terms of search time in the lower radius cases, another thing to consider is the build time of the datastructure needed to gain this performance. We can see that the build times for the SplitCloud2 and especially the SplitCloud method, outperform the build time of the FLANN kd-tree greatly **[3]**.

   **Conclusion**
   Overall we can conclude that in the cases where the radius is large with respect to the point cloud, the direct method (standard NNN, with returning distances) of performing a radius search is more appropriate. The SplitCloud and SplitCloud2 cases are more delicate.. I can carefully say that using the SplitCloud or SplitCloud2 method is more suitable when performing radius searches for only a small number of query points. This way we can maintain the advantage gained in the building of the datastructure.

   **Future work**
   As stated in this rather lenghty but complete blogpost, I will proceed with benchmarking libnabo more briefly. After the conclusions drawn from the benchmarkings and with the blessing of Marius, I will start integrating the two libraries into FLANN.

   What I would also like to mention is that I'm curious on what the performance of the SplitCloud and SplitCloud2 methods would do when they would benefit from multithreading. As this approach is clearly suitable for multithreading (when multiple query points are provided) the NNN library doesn't introduce this kind of concurrency. This might be an interesting thing to consider when starting the second phase of my roadmap (which consists of multithreading FLANN after I'm done with the integrations).

   **[1]** in case of a meaningful subdivision

   **[2]** mind the scale on the graphs, on the relative scale FLANN seems to dominate the SplitCloud methods, but on the absolute scale (in the case of a low number of query points) there isn't much difference because the radius search times are really small.

   **[3]** again, mind the scale


.. blogpost::
   :title: Benchmarking 
   :author: nickon 
   :date: 7-7-2011

   For those of you looking at my previous benchmarking results, there are some erroneous numbers in there. You should **ignore all** NNN, SC, SC2 **"without returning distances" timings** from the fully detailed benchmarking results. A bug introduced overhead for this method and hence polluted the numbers for the NNN, SC and SC2 (without returning distances) methods.

   Normally this doesn't affect the evaluation I made in the previous blogpost. What we need to straighten out is the following:

      - standard NNN, SplitCloud and SplitCloud2 (without returning distances) will actually be faster than respectively standard NNN, SplitCloud and SplitCloud2 (with returning distances)

   So the results of the optimized version, which doesn't return distances, will run faster then the non-optimized one, which ofcourse sounds intuitive. How much faster this is, I will straighten out in a following blogpost as I am currently working on an automated benchmarking program. This program will enable me to keep the level of detail in the benchmarks, but greatly reduce the time needed to provide the results, as everything (including graphs) will be automatically generated.

   Once this program is finished I will provide some new benchmarks that are evaluated on "real world" pointclouds, so this way we will have a fresh and entirely correct view of how NNN compares to FLANN. By then you will also see some benchmarking figures pop up on Andreas' blog, so be sure to check those out as well. They are all related to optimizing the current nearest neighbor searching library used in PCL (FLANN).


.. blogpost::
   :title: Automated benchmarking for NNN and FLANN 
   :author: nickon 
   :date: 7-11-2011

   I just finished the automated benchmarking **for NNN and FLANN**. Tomorrow I'll post the figures of some more realistic benchmarks, meaning that I will only take into account point clouds that are captured by a kinect device and try to provide a brief overview of my findings. Also, since large radiuses that cover almost the entire point aren't relevant in real world situations, these scale of radiuses will not be covered in the bencmarking tests. For this, I'll have to take into account the scale of every individual pointcloud that is being benchmarked, but normally this won't take too much time.

   So tomorrow you can expect a zip file with some folders containing the benchmarking figures for a radius search with **one** random query point in all of the 'real world point clouds' you can find @ `http://svn.pointclouds.org/data/ <http://svn.pointclouds.org/data/>`_. All of these results are automatically generated by the benchmarking program, written in Python and C++. I'll upload this program to the svn repo and post a link, so you can try it out yourself with different settings if you like.

   Tomorrow I'll also start inspecting some more of `libnabo <https://github.com/ethz-asl/libnabo>`_. For this library I've already added some support for initializing the appropriate data structues that are necessary for calling the appropriate procedures, but benchmarking for this library isn't yet supported. I'll try and get this finished tomorrow, together with some benchmarking figures of this library versus FLANN.


.. blogpost::
   :title: Benchmarking results for NNN
   :author: nickon 
   :date: 7-12-2011

   As promised, here are the results of the benchmarking tests for NNN. You can download them @ `http://dl.dropbox.com/u/32660516/NNN_benchmark.zip <http://dl.dropbox.com/u/32660516/NNN_benchmark.zip>`_.

   The benchmarking results that you can download are run on real world pointclouds grabbed from a kinect. You can find them @ http://svn.pointclouds.org/data/. In the benchmarking results I only added radiuses that cover up to approx 15% of the volume of the tightest bounding box around the whole point cloud, since larger radiuses are rarely used in practice.

   We can see that by far FLANN always outperforms NNN for these situations. The build time is also lower than the build time of the SplitCloud techniques used in NNN, so no advantage can be gained there either.

   I have currently partially added support for libnabo into the automated benchmarking program, but there are still some errors in there I need to fix before I can properly generate benchmarking figures for libnabo.

   Stay tuned!


.. blogpost::
   :title: All benchmarking results in one post!
   :author: nickon 
   :date: 7-21-2011

   **UPDATE** a debug build sneaked up on me and poluted the results... Hence this final benchmarking update. Now it's a fair battle between the libraries and FLANN :)

   The automated benchmarking framework is finished and you can all find it here: `http://svn.pointclouds.org/flann/nickon/trunk/automated%20benchmark/ <http://svn.pointclouds.org/flann/nickon/trunk/automated%20benchmark/>`_. Please read the README file that is included and everything should be clear after that :) The included python scripts are able to draw graphs of the benchmarking results and bundle all of them into a single html page. Have a look at the scripts I used for benchmarking libnabo and nnn and you should be ready to easily create your own benchmark automation.

   **LIBNABO**

   The results for libnabo can be found by clicking on this link: `Libnabo vs. FLANN <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark/index.html>`_. Brute force search times for the libnabo library are intentionally not plotted in the graphs. This would cause a different y axis scale, which would disable us to decently compare the kdtree implementations of libnabo to the one in FLANN. The actual search times of the brute force search can still be found in the results table above the respective graph.

   The results indicate that FLANN still is the faster library regarding knn search, with respect to libnabo. For almost every pointcloud FLANN 'outperforms' the libnabo library in terms of search times. Outperforming might be an overstatement here, because the search times are really close, but FLANN still performs a tad bit better then libnabo. When we look at the build times libnabo seems to gain some ground. Ofcourse the build time of the bruteforce search is just a fraction of a second (the NearestNeighbourSearch object just being initialized), but the kdtree methods of libnabo are consistently built faster then the kdtree index built by FLANN.

   The better search time performance of FLANN could be due to a different and hence better constructed kdtree. Otherwise the faster search times can be attributed to other optimizations in FLANN and we could benefit from building the tree like it is done in the libnabo library.

   **NNN**

   The results for NNN can be found by clicking on this link: `NNN vs. FLANN (realistic range) <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/NNN_benchmark/index.html>`_. The same comment applies here. To not overload the graph, only the search times of the optimized versions of the NNN, SplitCloud (SC) and SplitCloud2 (SC2) are plotted. The radiuses ranges considered in this 'realistic' scenario contain up to 15% of the entire point cloud.

   We can see that for all datasets, FLANN performs better on smaller radiuses. This is because the kdtree exploits the sparsity of the point cloud. Noticable is that the SplitCloud2 method comes pretty near the search times of FLANN! We can also see that the build time of the SplitCloud2 is always smaller than the build time of the FLANN kdtree. Depending on the point cloud distribution it can even be half of FLANN's build time.

   To get a view on the behaviour of the naive methods on larger radiuses I've also included a 'zoomed out' view of the graphs. You can find them over here: `NNN vs. FLANN (full range) <http://svn.pointclouds.org/gsocweb/source/nickon/NNN_benchmark_full/index.html>`_. We notice that when we move to larger radiuses, the benefit of the more naive methods (especially NNN, which doesn't use any datastructure at all), shows off **[1]**. However these radiuses are not very realistic scenarios so actually this is just to show the trend of these methods.

   So to conclude the benchmarking of NNN, for the realistic radius ranges we have the kdtree approach of FLANN that performs better than any naive method. We must however note that the SplitCloud2 method is a good tradeoff since it's build time is noticeably smaller than that of FLANN. For few searches this might prove valuable.

   **FUTURE STEPS**

   Now, since phase 2 is all about integration of the previously mentioned libraries but they don't introduce any real performance gains w.r.t. FLANN, (except for the faster build times of libnabo, which I will investigate) I can immediatly move forward to phase 3 which is the multithreading of FLANN. I already initiated this phase (sorry for the late blogpost on the libnabo results, my bad) and am currently looking into and evaluating Intel TBB for managing optimum thread spawning, cache coherency,... for the FLANN library.

   **[1]** I already mentioned that the larger the radiuses become the less benefit the SplitCloud methods will have, because they will construct allmost entirely overlapping point clouds.


.. blogpost::
   :title: Conclusions for integration after some more detailed benchmarking
   :author: nickon 
   :date: 7-27-2011

   Since I mentioned in my earlier blogpost that I would investigate the smaller build times of SC2 and Libnabo than those of FLANN, here is the conclusion on integration (and some other interesting stuff I picked up along the way):

   **The larger build times of FLANN**

   SplitCloud2 and Libnabo *seemed* the have smaller build times than FLANN, but this was due to the use of the PCL wrapped FLANN version instead of using FLANN directly. The larger build time of the PCL wrapped FLANN is due to a copy of the pointcloud that is being made in the PCL kdtree wrapper that really should be avoided. You can see the difference in build-time between wrapped and direct FLANN in these benchmarking figures (SplitCloud2 is also considered):

   `http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/SC2vsFLANN_benchmark/index.html <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/SC2vsFLANN_benchmark/index.html>`_

   You can see now that it is in no way beneficial (in realistic scenario's) to choose SplitCloud2 over FLANN.

   **Equivalent kdtrees in libnabo and FLANN**

   What I would like to mention is that I did some refactoring on the benchmark framework in my trunk. For libnabo and FLANN benchmarking I added a way to easily build kdtrees with the same leafsize (number of points in a leaf node) such that benchmarkings can be created where the two libraries always fight with equal weapons. For the default leafsize that is currently used in PCL (=15), we can see that FLANN is performing better in both measurements (build and search time):

   `http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark_leafsize_15/index.html <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark_leafsize_15/index.html>`_

   For different meaningful leafsizes, FLANN keeps outperforming Libnabo, meaning that integrating Libnabo (or certain aspects of it) would not be beneficial in any way.

   **Optimal leafsize for kdtree**

   It also turns out that when changing the leafsize that is used for building the FLANN kdtree, some more performance can be gained. Changing the leafsize from 15 to 50 **[1]** improves the kdtree build time with approx *20%* while radius search times only incur an increase of approx *5%*, and knn search times incur an increase of approx *5-10%*. This is a valuable insight that can be used in choosing a leafsize that balances the build and search times for it's particular use in PCL. The results of these benchmarkings are linked below for the ones that are interested:

   `http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/FLANN_leafsize_benchmark/index.html <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/FLANN_leafsize_benchmark/index.html>`_

   `http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark_leafsize_50/index.html <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark_leafsize_50/index.html>`_

   `http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark_leafsize_100/index.html <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/NNNandLibnabo/Nabo_benchmark_leafsize_100/index.html>`_

   **Multithreading of FlANN**

   Sorry for the late update on the final conclusion of integration for libnabo and SC2, this is because I have ben context switching between multithreading development and interpreting benchmarking results. Currently I've set up a test project that locates Intel TBB using pkg-config and a FindTBB.cmake file I wrote. This is a clean and basic project that I can easily expand while developing a multithreaded variant of the SingleKDTreeIndex in FLANN. I'm currently working on this as we speak. The goal will be to highly optimise this index and when CMake detects that Intel TBB is installed on the system, it will maximally exploit the capabilities of the hardware platform.

   **[1]** I noted from the benchmarkings that a leafsize of 50 provides a good balance between kdtree build time improvement and search time degrading, if the current leafsize value should be changed is currently being discussed on the developer mailing list. In effect some more fine grained benchmarkings could be generated to reach a well founded consensus.
   

.. blogpost::
   :title: Multithreaded FLANN
   :author: nickon 
   :date: 8-12-2011

   It's been a while since I've posted a blog update, but I haven't been sitting around lately. Since my last blogpost I completed the multithreading of FLANN by making use of the Intel Threading Building Blocks library. This embodies that now all knn- and radiusSearches make use of multiple cores when specified by the user. The parallelism can be exploited when multiple query points are passed trough the knn- or radiusSearch interface and the amount of cores is specified in the SearchParam struct.

   Because using Intel TBB provides some additional overhead (using function objects, spawning threads(!), ...) over the normal singlecore FLANN implementation, it is expected that it will only pay off when enough work is being done by the spawned worker threads. How is work increased? Ofcourse as the k and radius parameter increases and/or the more query points are passed through the interface, the more work is introduced and we start benefiting from the multithreaded version at a particular point. Cfr. the GPU implementation of FLANN (Andreas).

   **Multithreading results**

   Opposed to the GPU implementation, multithreading starts to pay off at much smaller workloads. Depending on the k/radius parameter, multithreaded search can already start paying of at as few as 100 query points. You can have a look at the benchmarkings I did on my i5 450M processor (quadcore; 2 pyshical cores, each being a logical 2 core processor): `FLANN TBB payoff comparison <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/flann-tbb/knn/>`_. This comparison is actually for knnSearch, but I also did the benchmarkings for radiusSearch and they are exactly the same. This ofcourse makes sense as the radius also just acts as a "work increasing" parameter.

   We can see that a fairly consistent speedup of 2x can be achieved when we transcend the payoff barrier. I've also made a benchmark for comparing traditional FLANN with FLANN using 2 cores and FLANN using 4 cores: `FLANN numof cores comparison <http://svn.pointclouds.org/gsocweb/source/nickon/BenchResults/flann-tbb/cores/>`_. Interesting to see is that introducing twice as many cores doesn't always double the search time performance as we all intuitively might think. You can see that going from 2 to 4 cores introduces an extra gain in searchtime speedup, but not as significant a speedup as going from 1 to 2 cores.

   **Future steps**

   Currently I'm wrapping up for the GSoC period, meaning that I'm cleaning code and integrating it nicely into FLANN, writing documentation and unit tests for it and so on. Once this is done the deadline of GSoC will probably be reached.

   Things that I want to perfect / examine later on is having a look at how the caching mechanisms in Intel TBB could potentially increase the performance of multithreaded FLANN even further. For example a first thing that I can have a look at is, when building the KDtree, try and see what the effect is of allocating each node or points in that node on single cache lines instead of using the standard malloc procedure, which doesn't offer any guarantee on this. This way we might already have an increase in searchtime because of the better fitting into and the more effective use of, the cache.

   Another more challenging but logical extention of this first optimization is to take into account which query points were assigned to which CPU, such that the warmed up on-processor cache can be used for doing the search through the tree. Potential major speedup can be obtained if large portions of the kdtree can be fit into the on-processor cache. This is detailed optimization work, but truly interesting stuff to look at and speed up FLANN even further.


.. blogpost::
   :title: Wrapping up
   :author: nickon 
   :date: 8-18-2011

   Ah, long time no seen! I've been busy with the redo of my exam (business skills, a broad view on economics. Wasn't really my cup of tea but I think I passed it this time *thumbs up*). Back to GSoC then! Currently I'm wrapping up for the final submission to the Google repos and for the merge with the git head of FLANN: doxygen comments, unit tests using GTest and writing a paragraph in the manual of FLANN on how to use the multithreading search.

   I'll send out another blog post when this is finished and the multithreaded search is is merged succesfully with the git head of FLANN!


.. blogpost::
   :title: Multithreaded search available in FLANN
   :author: nickon 
   :date: 8-26-2011

   Today Marius has applied the patch for multithreaded search to the git head of FLANN. So, multithreaded search is available now! Give it a try! If you encounter any problems, contact the mailing list and we will try to resolve them as quickly as possible :)

   This will also be my last blogpost as this is the end of GSoC. I would like to thank all PCL developers (especially Marius, my mentor) for their help during the summer and for the opportunity they gave me by accepting me for GSoC 2011! It was a blast and you'll probably see me around!
