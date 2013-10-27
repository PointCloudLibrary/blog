.. blogpost::
   :title: Going Live with the PCL-URCS Developer Blog 
   :author: stfox
   :date: 02-11-2012

   Greetings to the PCL community. My name is Stephen Fox, and I am
   the new developer who has just come aboard for the Urban Robotics
   Code Sprint. Stay tuned for updates on the project as we move
   forward with the integration of Urban Robotics's out-of-core octree
   implementation!

.. blogpost::
   :title: Diving into the UR Out-of-core Octree Code
   :author: stfox
   :date: 02-15-2012

   Thanks to Justin Rosen, the Urban Robotics out-of-core octree code
   is now in the trunk, cleaned up quite a bit from its original
   state, and compiling in the PCL framework (but we haven't changed
   the data structures to PCL's yet). I have been studying the code
   and starting to determine what else will need to be done as far as
   integration goes. I have to thank Justin and Radu for their warm
   welcome, and for getting me up to speed over the weekend.

   So far, I have run the unit tests that Justin converted from Boost
   to Google unit testing framework, and experimented with the
   limitations of the code. Currently, I'm working on a combination of
   two things. I'm working on refactoring the code to work with PCL's
   templating and allow point clouds, rather than vector<PointT> as
   inputs. The transition is going smoothly so far. I can still build
   the octrees on disk with the unit tests, but there are some issues
   with querying the points / bounding boxes that I'm still looking
   into. I'll have a more detailed update as soon as I get it
   straightened out.

   For tracking down bugs in the PCL in general, I've set up a
   double-buffered installation of PCL trunk and whatever revision I'm
   interested in, and set up a build environment for test programs to
   build and run against the different versions. Now that I'm getting
   settled in, I'm hoping to establish a more specific roadmap by the
   end of the week, and add some literature review for out-of-core and
   octree background reading.

.. blogpost::
   :title: Preliminary conversion from UR's point type to PCL's PointT
   :author: stfox
   :date: 02-16-2012
	  
   I've started to make the change to PCL's templated point type. It
   required minimal refactoring since the code is already templated,
   and very few operations are done on the point itself except
   accessing the fields (which are fortunately mostly the same as
   pcl's). To accommodate the Urban Robotics point data structure,
   we'll need to use the PointXYZRGBNormal data type, but for the time
   being, I jettisoned three additional fields (error, cameraCount and
   traidID) and have been testing with PointXYZ. The extra fields are
   currently just payload (within the library), and are never used
   within. We may need to develop some custom point types for the
   additional payload fields in the long run.

   Once I made the change, I had to rewrite some of the unit tests, as
   they relied on the operator== in the URCS point struct. This was a
   minor change, but now I can construct and query octrees out of core
   with the pcl::PointXYZ. I'd still like to benchmark HDD performance
   for queries, since performance seems to deteriorate quite quickly
   on my slow hard drive. I'm also planning to use the concepts in the
   existing octree unit tests as a basis to do a more exhaustive test
   of the out-of-core octree interface. 

   .. code-block:: c

		   [==========] Running 6 tests from 1 test case.
		   [----------] Global test environment set-up.
		   [----------] 6 tests from PCL
		   [ RUN      ] PCL.Outofcore_Octree_Pointcloud_Test
		   [       OK ] PCL.Outofcore_Octree_Pointcloud_Test (3448 ms)
		   [ RUN      ] PCL.Bounding_Box
		   [       OK ] PCL.Bounding_Box (0 ms)
		   [ RUN      ] PCL.Point_Query
		   [       OK ] PCL.Point_Query (215 ms)
		   [ RUN      ] PCL.Octree_Build
		   [       OK ] PCL.Octree_Build (700 ms)
		   [ RUN      ] PCL.Octree_Build_LOD
		   [       OK ] PCL.Octree_Build_LOD (56 ms)
		   [ RUN      ] PCL.Ram_Tree
		   [       OK ] PCL.Ram_Tree (42 ms)
		   [----------] 6 tests from PCL (4461 ms total)

		   [----------] Global test environment tear-down
		   [==========] 6 tests from 1 test case ran. (4461 ms total)
		   [  PASSED  ] 6 tests.

   Tonight and tomorrow I hope to start working on changing the
   container type to PointCloud pointers. Justin and I have also been
   discussing ways to remove the JSON dependency. Once the basic port
   is done, I'll be focusing on improving the interface, improving
   features and improving overall performance of the out-of-core
   octree. Finally, I hope to help develop supporting algorithms for
   Justin's out-of-core octree visualization tools.

.. blogpost::
   :title: PCL's Point Clouds in Outofcore Octrees
   :author: stfox
   :date: 02-29-2012

   Over the weekend I committed a lot more refactoring and
   documentation for the outofcore octree code. It is now wrapped in
   the pcl::outofcore namespace, and I added support for a member
   function to the pcl::outofcore::octree_base class to copy data from
   a point cloud to an outofcore octree. 

   The outofcore library contains four classes:

       * octree_base
       * octree_base_node
       * octree_disk_container
       * octree_ram_container

   Users interact with the outofcore octree entirely through the
   public interface of the octree_base class, which is templated by
   type of tree (disk v. ram) and the Point Type, and manages the
   nodes of the octree. The header files for the outofcore files are:

   .. code-block:: c

     #include <pcl/outofcore/outofcore.h>
      #include <pcl/outofcore/outofcore_impl.h>

   For an out-of-core octree, use the octree_disk_container, which
   will create a root directory for the tree, and up to eight
   directories, labeled 0-7 within each subsequent directory. Using
   the octree_ram_container constructs or loads an octree entirely in
   main memory, which is similar to the Octree already in PCL. 

   Each directory represents a node of the octree, a cubic
   axis-aligned region of space whose dimensions are specified by a
   "bounding box". All nodes at the same depth, but in different
   branches, represent disjoint regions of 3D space. On disk, each
   node is a folder containing:

   	* a JSON metadata file
	* up to eight child-node directories labeled 0-7
	* an optional data file of points subsampled from its children. However, if the node is a leaf, then it contains the highest "Level of Detail," i.e. all the points falling  within that bounding box, whether or not the intermediate LOD point sets were computed.
   
   The bounding box of the root node must be specified upon the first
   creation of the octree. This represents the region of 3D space
   which contains the entire out-of-core point cloud. Resizing an
   octree dynamically upon insertion of a point, or set of points,
   that do not fall within this bounding box is an expensive task,
   though not impossible (cf. Scheiblauer and Wimmer's strategy in
   [1] ). However, it currently is not implemented, meaning any point
   that is inserted that does not fall within the root's bounding box
   will be discarded.

   For building up octrees, I've added support for the addition of an
   arbitrary number of PointCloud::Ptr inputs via octree_base's public
   member function:

   .. code-block:: c

		   uint64_t
      		   addPointCloud(PointCloudConstPtr cloud)

   Justin has written a command line tool for constructing an
   out-of-core octree from a PCD file. Once this tool has been used to
   create an out-of-core octree stored entirely on disk, the following
   code sample can be used to load and manipulate the out-of-core
   data.

   At this point, Justin and I are looking forward to focusing on
   analysis of query, insertion, scalability and addition of
   features/algorithms for the outofcore data octree. I will also be
   adding examples to the trunk later this week.

   .. [1] Claus Scheiblauer and Michael Wimmer, "Out-of-Core Selection and Editing of Huge Point Clouds." Computers and Graphics, April 2011.

.. blogpost::
   :title: Chats about Outofcore Octrees
   :author: stfox
   :date: 03-14-2012
   
   Justin, Radu, Julius, Jacob and I have been discussing outofcore
   octrees. Some particularly interesting points of note arose in the
   conversation regarding the method of serialization. The UR
   constructs the octree in a depth-first manner, storing the point
   data in the leafs of the tree. If the LODs are generated, the
   folders (which contain internal node data) can be read (and
   rendered) in a breadth first manner, providing a successively more
   detailed octree as deeper nodes are read (see Justin's blog).

   Julius encodes his outofcore octree using a method similar to the
   Nested Octrees in the Scheiblauer and Wimmer paper [2]. Each file
   serialized is itself a sub-octree.

   Currently, I am investigating appraoches to serialization for
   octrees. I'm studying two papers in particular that I've found useful:

   * Aaron Knoll, `"A Survey of Octree Volume Rendering Methods" <http://www.mcs.anl.gov/~knoll/octsurvey.pdf>`_.
   * Sarah F. Frisken and Ronald N. Perry, `"Simple and Efficient Traversal Methods for Quadtrees and Octrees" <http://www.merl.com/reports/docs/TR2002-41.pdf>`_.

   .. [2] Claus Scheiblauer and Michael Wimmer, "Out-of-Core Selection and Editing of Huge Point Clouds." Computers and Graphics, April 2011.

.. blogpost::
   :title: Outofcore Octree Update
   :author: stfox
   :date: 03-28-2012

   Recently I've been investigating the issues with the outofcore
   octree reported by Justin. I have not been able to track down the
   reason that we cannot yet handle insertion of very large pcd files.

   I have also been re-implementing the outofcore containers (ram and
   disk) to inherit a common abstract interface class. While I am
   working on this, I am cleaning up the binary serialization code,
   and plan to transition the binary point data at each node to PCL's
   binary format. This will also allow us easy access to the
   lmzCompression of the point data.

   As we go along, I have been adding unit tests to monitor
   bugs/features. These still are not being run by the build server,
   but I hope to see the outofcore code base sufficiently stable to
   enable by default soon.

.. blogpost::
   :title: Changes in the outofcore code
   :author: stfox
   :date: 04-09-2012

   I was able to fix the issue reported by Justin in his March 22nd
   posting about construction of large out-of-core trees. There was an
   issue in construction of octrees from inputs that were too large
   (the TRCS point sets were too large). However, if the clouds are
   broken up into pieces, or come initially from a set of smaller
   point clouds, there doesn't seem to be any issue. For now, I
   refactored some of the reading/writing constants used, and that
   seems to have fixed it.

   Justin and I have been discussing modifying the PCD reader to
   iterate through PCD files out-of-core to avoid loading the cloud
   entirely into memory. This goes for both reading and writing PCD
   files. I think this is a good idea, especially since this software
   may be running on a busy server at Urban Robotics, in addition to
   encoding/decoding the octree data. I'm almost done with the change
   from C-struct dumped binary data files (ending in ".oct_dat") at
   each node to a PCD file. This will provide a lot of convenience for
   debugging, as well as a simple way to save in binary compressed
   (lmz) format, and give us access to PCL algorithms at the leaves.

   Radu, Julius and I have been chatting about compression within the
   out-of-core point cloud. As Radu pointed out to me, the chief issue
   here that we need to be wary of is I/O speed, particularly in
   reading and decoding the compressed data from disk, since this is
   supposed to speed up rendering/visualization of enormous data
   sets. Speed of writing to disk isn't currently a primary concern
   for optimization, though it has its place. Construction of the
   out-of-core tree is usually an off-line pre-processing step. Julius
   is going to help us determine how we could use the compression from
   the existing octree for fast decoding at variable densities.

   As a final note, I've added the abstract parent class for the disk and ram
   container classes. This is another step toward refactoring the code base and
   standardizing the interface throughout.

.. blogpost::
   :title: Out of core node data to PCD
   :author: stfox
   :date: 04-16-2012
	 
   For the past week, I have finished the change from dump binary
   C-struct to PCD file at each node. This should help with debugging,
   and will make manipulating the data, at each node and
   pre-rendering, easier. I need to clean up the code a bit more
   before I commit to the repository.

   In addition to cleaning up before committing, a few things that remain outstanding in this change are:

	  * block reading of point cloud files
	  * efficient appending to PCD files
	  * a more detailed consideration of how to take advantage of compression methods
	  * tending to the insertion and query methods of the octree_ram container class for use with PCD files

.. blogpost::
   :title: Back on track
   :date: 06-06-2012
   :author: stfox
	 
   After a busy couple of weeks, I am back to work on the out of
   core library. Justin, Julius, Jacob, Radu, and I have been
   discussing some pending changes to get outofcore performing at
   the appropriate level. Julius has provided some excellent
   feedback, and I think we will have some good demos soon.
      
   Summarizing the OOC interface as it currently stands, remaining tasks on the OOC side fall into the following categories:

      #. OOC Interface (octree_base/octree_base_node)
	 Responsible for recrusively traversing the top level in-memory octree

	 #. point/region insertion methods

	    * addDataToLeaf
	    * addPointCloud
	    * addDataToLeaf_and_genLOD
	    * addPointCloud_and_genLOD
	    * TODO: Need some tools for building point clouds from directory of PCDs
	    * TODO: Input support for PointCloud2 
	    * TODO: Improve the speed of tree-building (slow with zlib compression)

	 #. frustrum/box/region requests

	    * queryBBIntersects
	    * queryBBIncludes
	    * queryBBIncludesSubsample
	    * TODO: add PointCloud2 query support (almost done)

	 #. Parameterization

	    * container type
	    * downsampling
	    * compression (lossy, lossless)
	    * depth/BB resolution
	    * TODO: work out the interface for controlling these parameters; cross compatibility, etc...

      #. Encoding and Decoding of compressed data (Lossy/Lossless)

	 * I have already added zlib compression into PCD containers
	 * TODO: look into lossy compression based on the PCL Octree compression
	 * TODO: Delay write for faster construction

      #. File I/O
	 
	 * Added some additional debug output to the PCDReader methods

   **Roadmap for the next few days:**

      * Finish adding support for PointCloud2 Queries
      * Add support for PointCloud2 as input

   **Roadmap for the next couple of weeks:**

      * Finish improvements to OOC construction (support of containers/point types, PointCloud2, caching, etc...)
      * Work with Julius on adding lossy-compression features
      * Clean up templating of interface class
      * Clean up construction of octree for speed
      * Abstract the hierarchy for easier modification of parameters
      * Make tools for OOC tree construction more flexible

.. blogpost::
   :title: PointCloud2 in outofcore
   :date: 06-09-2012
   :author: stfox
   
   Added insertion for PointCloud2 today. Justin and I have decided on
   a few architectural changes to the outofcore classes. I will
   eliminate the templating on the classes, leaving templating only on
   the necessary methods for insertion and query (similar to the
   PCDWriter implementation). This will require a major overhaul, but
   streamlines the interface a lot. There is also some work to do on
   insertion with building the LOD using PCL downsampling techniques
   given the change in internal data storage. With the addition of
   PointCloud2 for I/O to the class, we have more flexibility with the
   fields, and Justin's visualizer does not need to know the point
   type at compile time.

.. blogpost::
   :title: Outofcore Updates
   :date: 06-17-2012
   :author: stfox

   I am still straightening out some I/O and insertion features with
   PointCloud2 and the lzf compression. I am hoping to have that part
   completely functional this week so Justin can finish integrating
   the API changes with his visualizer. I have not started changing
   the structure of the classes to reduce the amount of templating--I
   will hold off on that until we have the outofcore system up and
   running. Stay tuned for updates.

.. blogpost::
   :title: Outofcore Octree Updates and Changes for PointCloud2
   :date: 07-16-2012
   :author: stfox

   I have been a bit quiet on my blog, but have finally checked in a lot of improvements to the OOC libraries, smoothing out construction of the trees with PointCloud2 data types. This allows us to compile OOC code without having to know the point type in the PCD file. I have finally finished implementing the PointCloud2 interface. I have not added some auxiliary functionality yet such as buildLOD (after insertion of data to leaves), but LOD can be built automatically on insertion using addPointCloud_and_genLOD. 

   OOC has also been enabled to build in trunk by default as it is approaching more stability. Justin is working on having the out of core visualization engine running on VTK while I am supporting lingering performance issues with OOC construction and query, and doing some final refactoring of the code and library. With the integration of PointCloud2, point clouds can be created with any XYZ-based data type by first using toROSMsg, then inserting the PointCloud2 to the OOC octree. Once Justin and I have a working out-of-core visualization pipeline (he's handling the heavy lifting with rendering), there is still much more we can do to add to the capabilities of the library.

.. blogpost::
   :title: New outofcore refactoring, cleaning of API
   :date: 08-04-2012
   :author: stfox
   
   I have been working on cleaning up the outofcore library for its initial release. I still need to finish updating the documentation, add the examples to the doxygen mainpage (which is currently blank), and write a tutorial on how to use the out of core libraries. There is still quite a bit of unused code to pull, and a lot of refactoring to get the code fully to PCL's style standards. I am still debating whether it makes sense to remove the templating. I committed some refactoring this afternoon, and will continue to do so concurrently while I am preparing the code for final testing. I have also started writing a final report to wrap up the code sprint. There has been some growing interest in outofcore features in PCL on the mailing lists lately, so I hope to have the code base fully useable soon.

.. blogpost::
   :title: Out of core: Pending API Improvements
   :date: 08-26-2012
   :author: stfox

   Over the past two weeks, I have made some considerable changes to the outofcore API: 

	    #. Renamed all of the out of core classes to meet the PCL class naming convention

	       #. octree_base to OutofcoreOctreeBase
	       #. octree_base_node to OutofcoreOctreeBaseNode
	       #. octree_disk_container to OutofcoreOctreeDiskContainer
	       #. octree_ram_container to OutofcoreOctreeRamContainer

	    #. Renamed some public and private methods in OutofcoreOctreeBase, as well as pulled unused code, renamed protected methods, etc...

	       #. printBB->printBoundingBox
	       #. insertsectsWithBB->insertsectsWithBoundingBox
	       #. pointWithinBB->pointInBoundingBox
	       #. getBB->getBoundingBox
	       #. withinBB->inBoundingBox 

	    #. Moved all Boost dependencies into a separate outofcore boost header
	    #. Encapsulated the metadata of the OutofcoreOctreeBaseNode class into a separate class handling the JSON I/O. This should allow flexibility with metadata format (and the possibility of seamlessly switching to XML/YAML, etc...). This functionality lives in a separate class called OutofcoreOctreeNodeMetadata.

   A few changes I am currently working on are:

	    #. Implementing depth first and breadth first iterators, similar to the implementation in pcl_octree by Julius
	    #. Improving the readability of the doxygen documentation
	    #. Parameterizing of the LOD building algorithm (customization via pcl::Filter input)

.. blogpost::
   :title: URCS Final Report
   :date: 09-30-2012
   :author: stfox

   Over the weekend, I finished the URCS final report. I would like to thank my mentors, Jacob Schloss at Urban Robotics, Radu, and Julius, who all provided me a lot of patient support and inspiration on this project. Justin Rosen on TRCS has been a great collaborator and deserves a lot of credit for the success of the outofcore integration.

   I would like to draw some attention to a few items related to outofcore that I explain in this document in further detail. First of all, even though the URCS is drawing to a close, outofcore is sill very much under development. The API is not 100% set, though significant strides have been made in restructuring the internals for long term stability and integration with PCL. Most importantly, **all functionality is available for PointCloud2-based insertion and query methods now! These should be the preferred methods of the interface.**

   Currently, for the average user, the pipeline will be:

	    * Construct an outofcore octree with pcl_outofcore_process from a set of PCD files

	    * Use one of the two available outofcore viewers for visualization.
	    * When building LOD, remember the number of internal root nodes can grow exponentially the deeper the tree gets. As a consequence, building LOD can take quite a long time if you are inserting tens or hundreds of millions of points into a tree that is very deep.

	    * pcl_outofcore_process computes the size of the bounding box of the ENTIRE set of PCD files. That said, if you want to update the tree later, it is your responsibility to make sure the points fall within the original bounding box. If you want to set an arbitrarily large bounding box, the source of pcl_outofcore_process is easy to modify for this case.

   For developers, I should emphasize two things to ensure compatibility with future changes that will be introduced into outofcore. If anyone has opinions on this, we would certainly like to entertain some discussion on the pcl-developers@ mailing list.

	    * The classes are still templated, but this will change. Because rendering is dynamic, please use the PointCloud2 methods of the outofcore octree. It is admittedly a little confusing because the interfaces are still in a single class with overloaded methods. The easiest way to handle this while it is in a state of flux is via a typedef:

   .. code-block:: c

	 typedef OutofcoreOctreeBase<OutofcoreOctreeDiskContainer<pcl::PointXYZ>, pcl::PointXYZ> OutofcoreBase;

   Then, FORGET that it is templated on pcl::PointXYZ; it is not important. You can use whatever Point Type you would like packed into a PointCloud2 data structure. You can easily convert your PointCloud to a PointCloud2 by using toRosMsg.

	    * I also emphasize that Outofcore, while an octree, is NOT related to pcl_octree at this time. I have added some methods to make their interfaces somewhat similar, but please beware of a false sense of consistency.

   The final report should contain enough information to get started developing with outofcore. I hope it also provides a sense of where the code is coming from and where the library is heading in the context of its future place in the PCL library.

   .. raw:: html 

      <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Furcs%2Ffiles%2Freport.pdf%3Fraw%3Dtrue&embedded=true" width="400" height="800" style="border: none;"></iframe></center>

