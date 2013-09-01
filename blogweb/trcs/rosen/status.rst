My status updates
=================

.. blogpost::
   :title: Pressing On
   :author: rosen
   :date: 4-25-2012
   
   Back from vacation and getting into some of the more exciting stuff that is required to get our out-of-core viewer
   up and fully functional.  There has been quite a bit of talk on implementation and this week I plan on adding three
   important features:
   
      * Frustum Culling - Providing a way to query the octree for nodes on disk given the camera frustum.  This will
        provide a good start in determining what data needs to be streamed in.
      * Threading - All data processing needs to happen in a separate thread as to not block the main UI.
      * Caching - Implement some type of Least Recently Used(LRU)/Least Frequently Used(LFU) cache to store streamed
        data and discard the less relevant.
   
   I've spent most of my time recently learning and understanding VTK and getting the new mapper working with Radu to
   get this integrated into the pcd_viewer.  The current viewer doesn't handle large datasets and this integration will
   allow for much heavier data sets in realtime.
  
   The new VTK classes are far from finished and will require quite a bit of work to handle all the data VTK can throw
   at it.  I've sent Marcus, a core VTK developer, my work in progress in hopes to get some help on proper and stable
   integration with VTK.  Here are some of the todo items still to be addressed.
   
   vtkVertexBufferObject - Vertex Buffer Object Wrapper
  
      * Needs to support both VTKs float and double data types
      * Needs to support indices via GetVerts and \*vtkIdType
      * First implementation supports vertices, indices and colors.  Need to add all of VTKs attributes, i.e. normals,
        textures, ...
      
   vtkVertexBufferObjectMapper - Vertex Buffer Object Rendering Wrapper

      * Need the ability to set vertex, fragment and geometry shader (Currently uses simple shaders)
      * The current mapper has vertex, indices and colors VBOs.  Need to support all of VTKs attributes, i.e. normals,
        textures, ...
      * Determining whether to pull point/cell data and if colors are present on the data passed in.
      * Handle all VTK data types to make sure calls to glVertexAttribPointer are correct and consistent.

.. blogpost::
   :title: vtkVBOPolyDataMapper Commit - WIP
   :author: rosen
   :date: 4-3-2012
   
   I've committed a working version of my vtkVBOPolyDataMapper that the outofcore_viewer is now using.  This is the
   working version from my previous post.
   
   I started breaking out the VBO functionality into the separate class vtkVertexBufferObject that'll be used by the
   mapper, vtkVBOPolyDataMapper.  I'm running into a few issues I need to resolve before committing and getting feedback
   from Marcus at VTK and the PCL group.  The interface for the new vtkVertexBufferObject should be quite simple and
   handle the basic VTK objects.  Will post another update once I've got this all working.

.. blogpost::
   :title: New Commits and More on Visualization
   :author: rosen
   :date: 3-22-2012
   
   I've updated the pcl_outofcore_process runtime to dump numerous pcd files into an octree.  There are currently a few
   issues with this tool that need to be resolved:

      * numpts isn't written to the json file
      * Given that numpts isn't written it's hard to tell if all points specified within the pcd files are getting
        written to disk
      * Should we store bounding box info within the header of a pcd file so an entire cloud doesn't require parsing?
      * Should we write a pcd reader that can iterate through the points so the entire cloud doesn't have to be loaded
        into memory?  This'll be useful when loading up clouds in the millions+
      
   I've committed the octree_viewer from my last post with very basic VTK functionality.  This required some additional
   methods that already existed within the pcl_octree.  Although, I've noticed that the voxels displayed aren't square,
   which I think should be.  I'll have to look into this further.
   
   On to the more exciting things!
   
   I've been in talks with Radu and Marcus Hanwell, one of the developers on VTK on how we should move forward with our
   visualizations tools.  The current version of VTK is based on OpenGL display lists, which is a very old technology
   deprecated in 3.0, removed in 3.1 with support via the ARB_compatibility extension and a compatibility profile
   introduced in 3.2.  That being said, there are classes within VTK that use the newer technology, just not
   the pieces we're interested in.
   
   Why is this important?  Well, because writing an out-of-core viewer doesn't make much sense with display lists and
   requires a more dynamic implementation.
   
   As VTK 6 evolves various newer OpenGL features will be integrated, vertex buffer objects (VBOs) among them.  Until
   then, I plan on helping Marcus get these new features in faster by helping prototype and possibly develop the classes
   required.  The good news is the VTK guys now have immediate testers with a simple test case, billions of points!
   
   I've started to hack together a new VTK mapper vtkVBOPolyDataMapper to replace the vtkPolyDataMapper.  I have a very
   barebones version working.  It's got a long way to go, makes quite a few assumptions and'll need some love to work
   generically in the VTK framework.  I'll post more on this when I update the octree_viewer with the newer
   functionality.
   

.. blogpost::
   :title: Visualization with VTK
   :author: rosen
   :date: 3-7-2012
   
   I've started to wrap my mind around VTK and the PCL Visualizer.  I wrote an application similar to the octree_viewer
   using straight VTK.  The following is a processed outofcore cloud with 4 LODs.
   
   .. image:: images/OutofcoreViewer01.png
      :align: center
   

.. blogpost::
   :title: Urban Robotics Octree Framework
   :author: rosen
   :date: 2-28-2012
   
   The following diagrams the general framework PCL received from Urban Robotics for use in the out-of-core project.
   The diagram is broken up into three parts:
   
   * Creation
   
      * Points of type PointT are added to the octree_base data structure.  This data structure is in charge of
        managing the underlying child nodes and subsequently divided data.
      * As the points are subdivided, octree_base_nodes are created containing a random subsample or LOD of the points
        that are contained within each node (branch).  These nodes are in charge of managing bounding box and meta data
        on disk and hold payload data read from and written to disk, but doesn't handle the lower level read/writes.
      * Once a max depth or leaf node is reached a container type is created to manage disk or ram access.  These are 
        currently the only types of containers available within the framework.
      * The disk containers handle the low disk I/O
  
   * Directory Structure
      
      * At the top level of the directory structure lives a .octree file containing the octree depth or LOD, number
        of points at each LOD and various other bits of meta data.  This maps to the octree_base.
      * Each directory from the top level root directory maps to an octree_base_node.  Each node directory contains a
        .oct_idx file providing a nodes bounding box and LOD data.  Leaf nodes have no children (child directories) and
        are found at the max depth of the tree providing access to the original payload data (Not a subsample).
      
   * Query
   
      * When reading or querying the tree the octree_base provides an interface to the underlying data structure.
      * Querying the tree is accomplished by providing a bounding box that intersects with the underlying
        octree_base_nodes.  These octree_base_nodes provide access to the point data via containers or filepaths
        containing the binary point data.
   
   .. image:: images/UrbanRoboticsOctree.png  
      :align: center
      
   Stephen and I have been documenting and refactoring the underlying the code and are at a point where we can start
   investigating some of the more interesting features to be implemented.
   
   In addition I've started to commit tools that'll be useful in the processing of pcd files for use in the framework.
   

.. blogpost::
   :title: Urban Robotics Octree Refactor and Documentation
   :author: rosen
   :date: 2-13-2012
   
   I've started to dig through Urbans code a bit more, refactoring where
   possible.  The code is now broken out into a few more manageable pieces.  I've
   also started commenting and documenting the portions I've walked through.
   
   Radu and I welcomed `Stephen Fox <http://www.pointclouds.org/blog/urcs/stfox88/index.php>`_
   on today who is working on the URCS code sprint.  Stephens been brought up to
   speed and we'll now have two minds diving into the world of out-of-core
   visualization.
   
   I committed to the trunk a pcl_outofcore module.  It's unclear to me at the
   moment if this will eventually be rolled into the existing octree module.
   
.. blogpost::
   :title: PCL Visualizer
   :author: rosen
   :date: 2-12-2012
   
   Spent some time with Radu going over some of the issues related to the
   current PCL Visualizer.  We were able to knock down memory performance and
   rendering speed for larger datasets.  In the current implementation we had
   multiple copies of the cloud used by PCL and VTK.
   
   For the time being we updated the visualizer to run in immediate mode which
   should speed things up significantly, while taking a hit during the creation
   of the display list.  This won't work for applications which require a more
   interactive session, i.e. filtering.

.. blogpost::
   :title: Urban Robotics Octree Unit Tests
   :author: rosen
   :date: 1-31-2012
   
   I've successfully compiled and ran Urban Robotics' Octree code and unit
   tests.  In doing so I created a new library named pcl_outofcore (which is
   likely to change), but is giving me a test bed for compiling their code.
   
   There were minor code changes including switching the unit tests from Boost
   to GoogleTest::
      
      [==========] Running 4 tests from 1 test case.
      [----------] Global test environment set-up.
      [----------] 4 tests from PCL
      [ RUN      ] PCL.Octree_Build
      [       OK ] PCL.Octree_Build (1584 ms)
      [ RUN      ] PCL.Bounding_Box
      [       OK ] PCL.Bounding_Box (5 ms)
      [ RUN      ] PCL.Point_Query
      [       OK ] PCL.Point_Query (356 ms)
      [ RUN      ] PCL.Ram_Tree
      [       OK ] PCL.Ram_Tree (482 ms)
      [----------] 4 tests from PCL (2427 ms total)
      
      [----------] Global test environment tear-down
      [==========] 4 tests from 1 test case ran. (2427 ms total)
      [  PASSED  ] 4 tests.
      
   This created tree of files on disk which represent the octree.  The depth of
   the directory structure is the depth of the tree.  Each directory
   represents a branch or tree node::
   
      ├── 0
      │   ├── ...
      ├── 1
      │   ├── ...
      ├── 2
      │   ├── ...
      ├── 3
      │   ├── ...
      ├── 4
      │   ├── ...
      ├── 5
      │   ├── ...
      ├── 6
      │   ├── ...
      ├── 7
      │   ├── ...
      ├── ade37c05-a2bb-4da4-8768-0aaa4f67a0e7_node.oct_dat
      ├── tree_test2.oct_idx
      
   Within each directory (node) we'll find a `JSON <http://www.json.org/>`_
   formatted metadata index file (oct_idx), binary point data (oct_dat) and
   multiple directories which are the nodes children::
   
      {
        "version":      2,
        "bbmin":        [0, 0, 0],
        "bbmax":        [1, 1, 1],
        "bin":  "ade37c05-a2bb-4da4-8768-0aaa4f67a0e7_node.oct_dat"
      }
      
   I've also updated my roadmap which now contains a bit more detail on where
   I'll be going with a refactor of the current codebase.
      

.. blogpost::
   :title: Urban Robotics Octree Road Map Updates
   :author: rosen
   :date: 1-23-2012

   With the availability of Urban Robotics' octree-based point cloud format I'll
   be doing the initial integration of their work into PCL.  I've made a few
   updates to my roadmap related to Urban's octree format as well as some
   additional visualization tasks related to OpenGL and VTK.
   
   In addition to Urban's code I've started doing quite a bit of research trying
   finding the most relevant references related to the topic of out-of-core
   visualization.  This topic spans a large problem set where octrees play a key
   role, but are not the end all be all solution.
   
   The list of references on my blog are sure to grow as the sprint moves
   forward, but let's get started with some of my initial findings:
   
     * Hanan Samet

       * The Design and Analysis of Spatial Data Structures
       * Applications of Spatial Data Structures
       * `Spatial Data Structures* <http://www.cs.umd.edu/~hjs/pubs/kim.pdf>`_
      
     * Claus Scheiblauer, Michael Wimmer and N. Zimmermann 

       * `Instant Points: Fast Rendering of Unprocessed Point Clouds
         <http://www.cg.tuwien.ac.at/research/publications/2006/WIMMER-2006-IP/WIMMER-2006-IP-Paper.pdf>`_
       * `Domitilla Catacomb Walkthrough – Dealing with more than 1 Billion Points
         <http://www.cg.tuwien.ac.at/research/publications/2008/Scheiblauer-2008-DCW/Scheiblauer-2008-DCW-Paper.pdf>`_
       * `Interactive Domitilla Catacomb Exploration
         <http://www.cg.tuwien.ac.at/research/publications/2009/SCHEIBLAUER-2009-IDCE/SCHEIBLAUER-2009-IDCE-Paper.pdf>`_
       * `Out-of-Core Selection and Editing of Huge Point Clouds
         <http://www.cg.tuwien.ac.at/research/publications/2011/scheiblauer-2011-oocsehpc/scheiblauer-2011-oocsehpc-paper.pdf>`_
          
     * Oliver Kreylos, Gerald W. Bawden and Louise H. Kellogg
      
       * `Immersive Visualization and Analysis of LiDAR Data
         <http://www.springerlink.com/content/u014742162126831/>`_
        
     * Rico Richter and Jürgen Döllner
      
       * `Out-of-Core Real-Time Visualization of Massive 3D Point Clouds
         <http://dl.acm.org/citation.cfm?id=1811178&dl=ACM&coll=DL&CFID=81365378&CFTOKEN=37609096>`_


.. blogpost::
   :title: hello world!
   :author: rosen
   :date: 1-17-2012

   .. code-block:: python
      :linenos:

      #!/usr/bin/likepython
      
      yo just print like "hello world!" bro
