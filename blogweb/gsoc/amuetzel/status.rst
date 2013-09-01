My status updates
=================

.. blogbody::
   :nr_days: 30
   :author: amuetzel

.. blogpost::
   :title: Late Start
   :author: amuetzel 
   :date: 5-26-2011

   Today I finally returned to Germany and started with my GSoC work. I'm one of the GPU NN guys and, of course, developing on Linux. I already compiled PCL on my laptop, but currently I'm working on getting all the stuff (incl. CUDA) to run on my desktop PC, since the laptop doesn't have a Nvidia graphics card. 

   In the meantime, I'm having a look at the FLANN KD-Tree implementation and the CUDA one mentioned in my roadmap and already encountered the first problem: the CUDA implementation was developed with Visual Studio, which means: no Linux Makefile, meh... 
   
   A first observation I made when having a first look at comparing my own KD-Tree implementation with FLANN: I use a left-balanced median split scheme which stores the points of the tree without any overhead and switches cyclically between the split axes. When using randomly generated tree and query points in the 3D unit cube, my implementation is slightly faster than FLANN and the OpenCL version that I did a few months ago is about 8 times faster than FLANN on a GTX 260 vs. a Phenom II 955. But when using configurations such as ones where the x axis of all query points is restricted to [0,0.5] while the y axis of all tree points can only take values in [0,0.25], the performance degrades seriously. The CPU implementation takes about 30 times longer, as well as the OpenCL one, but FLANNs search time does not change. So I guess the flexibility of choosing optimal split planes is worth the overhead of storing additional info, which is what FLANN does if I saw it correctly.
   
   In the next days, I will focus on the first milestone, the definition of a suitable interface for GPU KD-Tree search, should the current FLANN interface not prove sufficient.


.. blogpost::
   :title: CUDA Madness
   :author: amuetzel 
   :date: 5-27-2011
  
   I seriously started wondering why Linux people even look at CUDA... Trying to install it right now, bu it seems like only horribly outdated distros with GCC < 4.5 are supported. (And in turn, most of those aren't really supported any more.) I hope I'll get gcc 4.4 running on Archlinux, otherwise I'll have to switch to an unsupported Fedora. Or is OpenSUSE 11.2 still supported?

   Update: GCC 4.4 is running now, but I still get tons of errors in the PCL CUDA code. Never thought that this would be such a dependency nightmare :-( Seems like I have to try Ubuntu 10.04 then...

.. blogpost::
   :title: Start of the Second Week
   :author: amuetzel 
   :date: 5-31-2011
  
   The good news: CUDA and PCL are working now on my ArchLinux PC. So no Ubuntu for me :-) The way it works is kind of hackish now. I compile the CPU code with GCC 4.6, but pass the option '-DCUDA_NVCC_FLAGS="--compiler-bindir=/opt/gcc-4.4"' to cmake, so that nvcc uses GCC 4.4 for the CPU stuff in the .cu files. 
   At least it works... 

   The bad news: The GPU implementation on http://www.cs.unc.edu/~shawndb/ doesn't build on my PC. Missing header files in the project, as well as nvcc errors such as "error: attribute "global" does not apply here". 
   Seems like I have to leave out the benchmarking and start directly with the implementation without learning anything from that implementation...

   In the meantime, I'm figuring out the optimal NN search API to allow for GPU and CPU searches. See the mailing lists ;-)

.. blogpost::
   :title: Discussing Things and Learning CUDA 
   :author: amuetzel 
   :date: 6-7-2011
  
   It's been a busy last week. So far, I've been discussing PCL and FLANN API changes for the GPU NN search on the mailing list and with Marius and I started getting familiar with CUDA and Thrust. Additionally, I got the existing CUDA NN search to compile after getting help from Shawn, 
   but it results in runtime errors on my PC...

   Additionally, I just ported my toy 3D OpenCL kD-Tree to CUDA. Nice to see that the "porting" was basically a copy&paste job. And it works about 30-50% faster than before on the same graphics card. Good to see that it's faster than the OpenCL version, but also kind of sad since it won't work on AMD graphics cards.

.. blogpost::
   :title: More GPU Programming 
   :author: amuetzel 
   :date: 6-10-2011
 
   The last days, I continued working on the kD tree. Spent quite a few hours on a problem where the 1NN GPU code would find a slightly different point from the CPU implementation and finally traced it down to floating point imprecesions... 

   Additionally, the kNN search works now. All of this is still a naive implementation, but can be faster than FLANN in some cases. Benchmarks will hopefully follow this week, next week otherwise.

.. blogpost::
   :title: First Code Uploaded
   :author: amuetzel 
   :date: 6-16-2011
 
   I finally cleaned up the code a lot and restructured it so that it can be used in regular cpp files that are not processed by nvcc. (It's not integrated into PCL, but will go straight into FLANN.)

   If you want to try it, even though I wouldn't consider it production-ready: the code is located at http://dl.dropbox.com/u/32615544/kdtree.zip until it is integrated into FLANN. The interface is slightly similar to FLANN, but it can't be used as a drop-in replacement in the current form.
   You will likely need to change one include path in the CMakeLists.txt, as it points to a directory in my own home directory, but apart from that, it should work out of the box.

   Any benchmark results would be welcome, together with information about your GPU and CPU! (I usually execute the tests with 1M points.)

.. blogpost::
   :title: Next Experiments
   :author: amuetzel 
   :date: 6-25-2011

   The last days I have started working on translating the FLANN KDTreeSingleIndex to a GPU version. FLANN's implementation of the kD-tree has two main advantages in comparison to my implementation: First, the worst-case performance when the query points lie far away from the closest search points is way better. Second, it is possible to store multiple points in a single leaf node, which is not possible with the minimal tree representation. 

   I hope that both points will be beneficial in the GPU implementation. In the last days, I worked on the tricky problem of translating the search algorithm to a non-recursive version, along with the necessary changes in the data structure. Right now, I am porting this to CUDA and will report on the performance as soon as it is done.
    
.. blogpost::
   :title: More FLANN progress 
   :author: amuetzel 
   :date: 7-03-2011

   Last week, I didn't have that much time for the GSoC work. But at least I finished the 1NN and kNN searches, integrated them into FLANN and uploaded the code to svn.pointclouds.org/flann/amuetzel/trunk/flann. To use it, first build the modified flann library you can find there. 

   The only difference to using the normal FLANN KdTreeSingleIndex is to create the index like this: 

   .. code-block:: cpp

     flann::Index<flann::L2<float> > flannindex( dataset, flann::KDTreeCudaLowdimIndexParams() );

   If you do this, you will likely get an exception about an unknown index type as soon as you run the code.
   To solve this, #define FLANN_USE_CUDA before including flann.hpp and link libflann_cuda to the executable.
   The CUDA index types are enabled this way to avoid having to link the lib when no CUDA index is going to be used.
 
   If you want to try the code, there is one other thing to keep in mind: Right now, it only works with the flann::L2<float> distance on 3D vectors. 

   In the next days, I will start working on four things, likely in this order: Restructuring the GPU kernel code to make it easier to adapt it to other distance types, porting the minimal kD-Tree to the FLANN API, making the interface work with 2D and 4D vectors and finally creating a benchmark with real-world data sets. I hope to be able to reuse some parts of Nick's benchmark code for that, as he is doing the comparison of of the NN search libraries at the moment.


.. blogpost::
   :title: Optimizations 
   :author: amuetzel 
   :date: 7-10-2011

   Since the speedup in comparison to the KDTree CPU implementation didn't seem that high to me, I tried several things to optimize the search algorithm. These are:

      - Structure of Arrays (SoA): split the node structure into several (32 bit and 128 bit) structs and store each part in a single array: normally, this is supposed to result in performance gains on GPUs, but here, it always decreased performance.
      - Recursive Search: In the beginning, I re-wrote the tree traversal to work without a stack. But for this, I needed to store the parent index of each node, which increased the struct size by 128 bit due to alignment issues. I tried to instead keep a stack in shared memory and do the search in a recursive way. But this was also about 10% slower in the benchmarks presented in the last post.
      - Rotating split axes: Another way to save 128 bit on the node struct size was to rotate the split axis and remove the information about it from the struct. Result: again, no speed improvement.

   So it seems like I came up with the best solution on the first try. I still have one or two ideas that might increase speed, but my hopes aren't that high.

   I think that the kD tree traversal just isn't that well suited for GPU computation because of the large number of random memory accesses and generally the high number of memory accesses and low number of computations.

.. blogpost::
   :title: Performance In Comparison To Other Implementations
   :author: amuetzel 
   :date: 7-11-2011

   Basically my kd tree is up and running now. The problem is that the performance is always better than FLANN, but the speed advantage in some tests is sometimes as low as as 8x, while it is more around 10x-15x usually, and in some rare peak cases almost 25x. I tried finding
   some other GPU NN search implementations to compare my results to, but those are quite rare. One thing I found was http://nghiaho.com/?p=437, but the search was slightly slower than my implementation on synthetic data sets and much slower on real-world data.

   In "GPU-accelerated Nearest Neighbor Search for 3D Registration" by Deyuan Qiu, Stefan May, and Andreas Nüchter, a gpu kd tree is used for ICP and, via an approximate search, achieves a speedup of up to 88x, though this is for the complete ICP procedure with a slower CPU and slightly faster GPU than my system. Also,
   no information about the NN search time is given, so to compare the speeds, I would have to test the complete ICP setup. If I interpret the results and fig. 4 correctly, their search algorithm is faster than my implementation when only a coarse approximation of the nearest neighbor is needed, but when an exact nearest neighbor is needed, I suspect that my code should be a lot faster.

.. blogpost::
   :title: Status Update 
   :author: amuetzel 
   :date: 7-20-2011

   Last week I was waiting for a FLANN API change that would be necessary for radius search on the GPU. As the next steps (implementation of radius search, integration into FLANN and documentation) depend on this, I started with the next independent tasK: KD tree construction on the GPU. This isn't as easy as the GPU search was, but I think it's about half done. Most of the single functions should work, but not all of them are done yet (and of course, the pieces will still need to be put together). 

   I just noticed that the API change is done in GIT, so I'll put the GPU build on hold and finish radius search first.


.. blogpost::
   :title: Radius Search and GPU Tree Build
   :author: amuetzel 
   :date: 7-26-2011

   My current status: Radius search: works, but could need some improvement concerning the speed... GPU tree build: Naive implementation basically works, but would only be about 25-50% faster than the CPU version. Zhou et al achieved a speedup of about 8x, so I'm starting over with an approach that is more similar to theirs. (I'm not trying to implement their approach exactly as there are some implementation details missing, so I don't know for sure how they implemented it...)

.. blogpost::
   :title: Nearly Done 
   :author: amuetzel 
   :date: 8-7-2011

   Sorry for the long time without updates, I was basically working on the GPU tree build and didn't have any interesting results until now. Basically the result is: Building the kd tree on the GPU can be about 2-3 times faster than the FLANN CPU build method, though I didn't do any complete benchmarks until now.

   A nice side effect is that the GPU tree build leads to a slightly different memory layout that makes tree traversal a bit (~5%) faster than before. 

   I'm currently starting to polish the code, document it and write tests to verify that it works correctly. If everything goes the way I planned, it should be ready for a merge with FLANN HEAD by the end of the week.

.. blogpost::
   :title: GPU KD Tree Integrated Into FLANN
   :author: amuetzel 
   :date: 8-16-2011

   A few days ago, I sent a patch containing my GSoC work to Marius, and it can be found in FLANN (git head) now. If you want to try it, you simply have to install CUDA and then build FLANN from git. Instructions on how to use the new index type can be found in the manual. In short, I finished all three mandatory points of my roadmap, but sadly there wasn't enough time for the implementation of high-dimensional index structures in FLANN. 

   This was probably my final GSoC update, since I'm leaving on a trip tomorrow and returning on sunday. I really enjoyed working with this project, so I guess you can expect some more improvements on the CUDA FLANN kd tree after GSoC :-)

  
