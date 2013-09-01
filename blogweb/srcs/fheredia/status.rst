My status updates
=================

.. blogpost::
  :title: Using Kinfu Large Scale to generate a textured mesh
  :author: fheredia
  :date: 07-06-2012

  We added a tutorial that describes the pipeline of KinFu Large Scale in order to generate a textured mesh. We hope that this tutorial is very helpful and encourages people to share their experience with the application. We are interested also in hearing your feedback and impressions on KinFu Large Scale.
  
  The tutorial can be found in the Tutorials section of pointclouds.org
  
  Francisco and Raphael

.. blogpost::
  :title: Kinfu Large Scale available
  :author: fheredia
  :date: 06-25-2012


  We have pushed the first implementation of KinFu LargeScale to PCL trunk. It can be found under $PCL-TRUNK/gpu/kinfu_large_scale.

  We will make more complete tutorial for the application. But for now we put these recommendations for those who want to try it out. Some recommendations for its use:

  Make smooth movements at the time of shifting in order to avoid losing track.

  Use in an environment with enough features to be detected. Remember ICP gets lost in co-planar surfaces, or big planar surfaces such as walls, floor, roof. 

  When you are ready to finish execution, press 'L' to extract the world model and shift once more. Kinfu will stop at this point and save the world model as a point cloud named world.pcd. The generated point cloud is a TSDF cloud.
  
  .. image:: 09.png
    :width: 400 pt
    :height: 350 pt

  In order to obtain a mesh from the generated world model (world.pcd), run -> " ./bin/process_kinfuLS_output world.pcd " . This should generate a set of meshes (.ply) which can then be merged in Meshlab or similar.
  
  .. image:: 10.png
    :width: 400 pt
    :height: 350 pt

  Francisco and Raphael

.. blogpost::
  :title: Kinfu Large Scale
  :author: fheredia
  :date: 06-18-2012
  
  We have been working on the implementation of Kinfu's extension to large areas for several weeks now. This week we will start with the integration of the code to the latest trunk. However, it will be for now pushed as a separate module named Kinfu Large Scale. The reason behind this is to keep the functionality of the current KinFu, but at the same time to make available the large scale capabilities to those interested in exploring them. 

  The diagrams below show the distribution of classes in KinFu Large Scale, as well as a flowchart of the behaviour in the application.  

  .. image:: 07.png
    :width: 400 pt
    :height: 350 pt
      
  .. image:: 08.png
    :width: 400 pt
    :height: 350 pt
    
  We will post an update on the integration status by the end of this week. 

  Francisco and Raphael

.. blogpost::
  :title: Volume shifting with Kinfu
  :author: fheredia
  :date: 05-24-2012
  
  Hello all,

  This week, we focus on accumulating the world as we shift our volume around. The next video shows a point cloud we generated with multiple shifts. The TSDF data that is shifted out of the cube is compressed before sending it to CPU; this decreases the required bandwidth when transmitting the data.

  The saved vertices are only those close to the zero-crossings (the isosurface). The saved vertices include the TSDF value for later use (raycasting, marching cubes, reloading to GPU). In the video, the two-colored point cloud represents tsdf positive (pink) and negative (blue) values.
  
  .. raw:: html
  
    <iframe width="560" height="315" src="http://www.youtube.com/embed/FEfs4Trefz8" frameborder="0" allowfullscreen></iframe>

  We are now implementing a class that will manage the observed world. Each time the volume will be shifted, new observations will be sent to the world manager which will update the known world and will allow quick access to some parts of it. 

    .. image:: 06.png
      :width: 400 pt
      :height: 350 pt

  The following is a large point cloud generated and saved with our current implementation.      
      
    .. image:: 05.png
      :width: 400 pt
      :height: 350 pt
      
  Francisco and Raphael

.. blogpost::
  :title: Volume shifting with Kinfu
  :author: fheredia
  :date: 05-11-2012
	
  After a few weeks wrapping our minds around KinFu and CUDA, we have included the shifting functionality while scanning. Using the cyclic buffer technique (introduced in our previous post), we are able to shift the cube in the world. 

  Since we shift in slices, some information about the scene is kept in memory. this information is used to keep track of the camera pose even when we shifted the cube. 

  At this point, the data that is being 'shifted out' is currently lost, because we are clearing the TSDF volume slice to make space for the new information. 

  The next step is to extract the information from the TSDF volume before clearing it. This will allow us to compress it and save it to disk, or to a world model being saved to GPU/CPU memory. 

  We have some ideas on how to perform this compression and indexing, and we will explore them in the coming days. 

  At this point we are cleaning the code and adding useful comments. We want to push this to the trunk soon. 
  
  This video shows the shifting for a cube with volume size of 3 meters. The grid resolution is 512 voxels per axis.

  .. raw:: html

    <iframe width="560" height="315" src="http://www.youtube.com/embed/RIbtpjhHBbI" frameborder="0" allowfullscreen></iframe>

  This video shows the shifting for a cube with volume size of 1 meter. The grid resolution is 512 voxels per axis.
	
  .. raw:: html

    <iframe width="560" height="315" src="http://www.youtube.com/embed/Fl2PsAWXgGc" frameborder="0" allowfullscreen></iframe>
		
  Francisco & Raphael

.. blogpost::
  :title: Update
  :author: fheredia
  :date: 04-27-2012

  It's been some time  since our last post, and we have been learning a whole lot about CUDA and Kinfu itself. 

  Two big points have been done so far:

  # We analyzed the application from a complete perspective, to try to determine which are its strong points and its improvement areas. What we found when analyzing Kinfu looks somewhat like this figure:

    .. image:: 03.png
      :width: 400 pt
      :height: 350 pt

  The first impression is that Kinfu has clearly defined modules, which are reflected in the code distribution among the different files. However, the application requires that the functionalities and information shared between these modules is tightly coupled. It can be seen in several parts of the code that the parameters just keep cascading down the calls all the way from kinfu_app.cpp (highest level) to the GPU kernel calls (e.g. raycasting or integration operations). The main reason for this constant copying of parameters and long parameters list is precisely that the modules are separated from each other. 
  
  This might sound basic to experienced CUDA users, but we find it good to remind it there, as this was one of our main "aha moment".

  For example, if one was to declare a variable in internal.h - which is a header file included in all modules (via device.hpp) -, what you would obtain is a copy of this variable for every CUDA module, instead of getting only one of them accesible from all the modules. Once again, this is the result of having the modules compiled as independent.

  After discussing with Anatoly, it has been defined that in the mid-term all the internal functionalities of Kinfu will be consolidated into a single TsdfVolume class which will contain all operations for integration, ICP, raycasting, and memory handling. This will result in a more readable code, avoid long parameter lists between operations, while keeping the real-time performance of the current implementation. In other words, at a higher level the code will be clearer and more concise, while at low level it will have the same performances. 

  # We have been working on the solution to tackle the scalability limitations in Kinfu. The expected behavior can be described as follows: Whenever the scene is being scanned, the user may approach the borders of the current cube. At that moment, part of the existing information about the cube must be compressed and stored to either GPU memory, Host RAM, or HDD to be saved. For now, the latter two are not in scope.

  The cube that we are reconstructing is shifted, but it is partially overlapped with the previous cube. In other words, we will see a scene that is partially empty, but still contains part of the information of the previous cube. The origin of the TsdfVolume will be shifted as well, depending on where we approached the border of the initial scene. We believe that adding this overlapped information will help to estimate the pose of the camera once the new cube is loaded. This shift is handled via a circular 3D buffer implementation in the TsdfVolume class.

  For clarity, we show this process in the figure below. 

  .. image:: 04.png
    :width: 450 pt
    :height: 700 pt
    
  Francisco & Raphael

.. blogpost::
  :title: Texturing KinFu outputs
  :author: fheredia
  :date: 04-05-2012

  In parallel with the SRCS sprint, we have been working on texturing the meshes generated with KinFu.
  
  The code has been pushed to trunk under surface/texture_mapping.
  
  | So far, textures are not blended and a face is attached to the first texture that sees it entirely.
  | In the future, we wish to come up with simple heuristics that will select textures more efficiently (like the closest and most-facing one) and/or blend them together.
  
  A first result can be seen here:

  .. raw:: html

    <iframe width="560" height="315" src="http://www.youtube.com/embed/0-cuWmrQA0s" frameborder="0" allowfullscreen></iframe>
    
  In this video, 5 meshes have been textured an manually aligned to form the full room.
  Hopefully, the possibility to stitch volumes in KinFu (see our previous entry) will allow us to scan the room as one big mesh and skip the alignment process.
  
  Raphael & Francisco

.. blogpost::
  :title: Volume stitching 101
  :author: fheredia
  :date: 04-04-2012

  At this point, it is possible to detect when the sensor is reaching the border of the cube along the x-axis. Implementation for Y and Z still remains, but have to think on a smart/elegant way to determine when these boundaries have been passed.

  Since there is a VOLUME_SIZE in internal.h, a VOLUME_SHIFT_THRESHOLD has been included as well. The latter represents the distance to the border of the cube that will trigger the volume shifting.

  Volume shifting is toggled by pressing the s key while running Kinfu. It would be nice to have it as parameter in command line.

  Thinking about the following steps, the question arises whether the camera pose is the only reference to the global coordinates, because then shifting the camera would make us lose any reference to the world whatsoever.

  We got some interesting results by using the cube reset at the time of reaching the threshold. The link is at the bottom of this post.

    For now we are saving the last pose before doing the shift. This functionality could be similar to a video we saw on youtube as well which also refers to volume stitching.

  By stitching the volumes using the transform that is saved, the post-processing of the whole is could be possible. Although this is not yet ideal with respect to memory usage.

  .. raw:: html

    <iframe width="560" height="315" src="http://www.youtube.com/embed/QVu0ZANwv3o" frameborder="0" allowfullscreen></iframe>

  Francisco & Raphael

.. blogpost::
  :title: General proposed outline and some results on loading TSDF
  :author: fheredia
  :date: 04-02-2012

  Last Friday we were discussing more with Anatoly on how to extend KinFu for large areas. After two weeks of code and solution exploration, we have set the broad strategy to go forward.
  In general terms, the goal is to implement as much functionality as possible within the GPU. This means that we will minimize the information exchange between GPu and CPU, since the PCIe bus is a well-known bottleneck. Three main steps have been identified as well:

  1. Implement demo that allows to travel within office without tracking failures. When Kinect goes out of volume, the volume changes its physical position to fit camera frustum again. So that camera tracking could continue without resetting. We agree that filtered out data is dropped at this stage [1].

  2. Implement a repacking procedure of the volume. When the volume is shifted, the information of the cube is extracted and compressed within GPU for later use. The new area must be initialized with some values (TBD).

  3. Develop smart heuristics  to decided when the volume is shifting, and the consequences of such shift. 

  Therefore, the tasks starting from today are [1]:

  1.      Familiarize with all KinFu sources.
  2.      Implement integration/ray-casing that takes into consideration physical volume position. 
  3.      Implement volume content repacking to be consistent with 3D world after shift	

  Last week we were also experimenting with loading the TSDF from the filesystem. We created a small application to load data from a file to the GPU, perform raycasting and generate a screenshot.

  The figure below shows the stored TSDF point cloud. This is already in KinFu trunk. 

  .. image:: 01.png
    :width: 252 pt
    :height: 203 pt

  The result that we get is shown below. We used three camera pose (also stored in files). These are the results :

  .. image:: 02.png
    :width: 320 pt
    :height: 120 pt

  For now, this code will not be included in PCL because it needs some clean-up and matching to the coding standards. Furthermore it is part of the second step so it will be explored afterwards. 

  Francisco & Raphael

  .. rubric:: Footnotes

  .. [1] Anatoly Baksheev, Minutes of meeting, March 30, 2012.

.. blogpost::
  :title: First blog entry
  :author: fheredia
  :date: 03-22-2012

  This is Raphael's and Francisco's first blog entry. We have had a discussion with Anatoly to introduce ourselves, as well as brainstorm on potential solutions to gets us closer to the goal. Friday the 30th we will have a more concrete discussion about the most promissing solution(s). It will also help to determine more concrete tasks, since for now we are in an exploratory stage.
  
  Francisco & Raphael

