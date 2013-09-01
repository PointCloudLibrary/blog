My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Yangyan Li

.. blogpost::
	:title: Drag and Drop(dnd) Support and More Filters
	:author: Yangyan Li
	:date: 8-19-2012

	dnd support inside the scene tree is added, the users can drag and drop point cloud items to different render window, which will help them observe the point cloud. It will be nice to add more dnd support, for example, drag point cloud directly from the file system to the modeler app, there can be many such nice features, and I will add them after GSOC. And new filters can be added very easily to the current framework. But there seems an unknown exception thrown from file save module, which crashes the app, I will anaylse what's the problem and finish file/project load/save things.

.. blogpost::
	:title: Placement for ICP Registration
	:author: Yangyan Li
	:date: 8-15-2012

	The initial position and orientation can be tunned in a dialog triggered by double clicking the point cloud item ICP registration, as shown in the following snapshot. And ICP paremeters are exposed to give more control to the users.
        I will add more workers, add project support and document the functions next.

.. blogpost::
	:title: ICP Registration
	:author: Yangyan Li
	:date: 7-31-2012

	ICP registration is added. The input point clouds and the result can be shown in same or different render windows for better inspection, as indicated by the following snapshot.

	.. image:: images/snapshot-2012-07-31.png
		:width: 756px
		:height: 406px
		:align: center

        Initial positions of the input point clouds are important for the registration to converge. I will implement two types of tools for the initial placement of the input point clouds: setting corresponding points and directly tuning the orientation/position of the point clouds. The tuning will be accomplished by interactively adjusting the orientation/position parameters or by draggers(if it's not very complicated to implement draggers with vtk...).

.. blogpost::
     :title: Simplify Scene Tree and Point Cloud Storage
     :author: Yangyan Li
     :date: 7-24-2012
 
     After some efforts on making the scene tree with model/view, I gave up and just build it with QTreeWidget, which is simpler and works for desired UI interactions. And the work for making a PCL 2.x PointCloud is more than I expected, so I also gave up on this point cloud and found that pcl::PointCloud<pcl::PointSurfel> meets the requirements and I just store the point cloud in it for now. The storage will be upgraded to PCL 2.x point cloud type when it's ready. After I removed the over kill code, it becomes much easier to progress, and I will move to the registration part next.

.. blogpost::
     :title: Moving to the future PointCloud type
     :author: Yangyan Li
     :date: 7-4-2012
 
     I didn't choose PointT as the core data structure, since different algorithms will interact with PCLModeler requesting different point cloud types, so I decided to use PointCloud2 because it is a container of many fields. But now I found it's not easy to support some functions, for example, updating a specified field, or adding some fields, the current code is messy because of the accommodation for PointCloud2. According to `this thread <http://www.pcl-users.org/pcl-PointCloud-lt-T-gt-vs-sensor-msgs-PointCloud2-td2983165.html>`_, PointCloud2 is going to be deprecated, and `the proposed point cloud type in PCL 2.x <http://www.pointclouds.org/documentation/advanced/pcl2.php#pcl2>`_ is much more friendly for PCLModeler. So I will drop PointCloud2 and re-design the core based on the proposed data structure.

.. blogpost::
	:title: Current UI and Functions
	:author: Yangyan Li
	:date: 6-26-2012

	The current UI is shown as the following image. There are 3 groups of functions to be implemented, filters, registration and surface reconstruction. I've implemented one filter. More filters can be added quickly, but I will do it later after the whole framework is more stable. I will implement one function for each group, then add more. I am working on poisson reconstruction now, and then registration. 

	.. image:: images/snapshot-2012-06-26.png
		:width: 756px
		:height: 406px
		:align: center

	Features:

	* The point clouds can be rendered either in the main window or any other dockable windows, which is quite useful for registration, for example, the user can put the frames in the main window, and put each frame in a sperated dockable window, make interaction in the dockable windows and see how well the frames align in the main window.
	* The render windows and clouds are organized in the scene explorer, where contextual menus are supported, so the user can easily access the avaiable functions for the elements in the scene.
	* The user can turn on/off some channels when rendering the clouds.


.. blogpost::
     :title: Qt Model/View and Thread
     :author: Yangyan Li
     :date: 6-21-2012
 
     Now the point clouds and render windows are managed by the scene tree, and the thread support is added for the workers. QThread document seems to be a mess, and there's a lot of discussion on it, finally I took the one that seems to be the best practice of using QThread(http://mayaposch.wordpress.com/2011/11/01/how-to-really-truly-use-qthreads-the-full-explanation/).

.. blogpost::
     :title: Parameter and Worker
     :author: Yangyan Li
     :date: 6-18-2012
 
     It seems vtk doesn't have ready to use draggers...so I leave it for now. Instead, I implemented a rough support for invoking workers and setting up the parameters for the workers. I will polish it and add support for more workers.

.. blogpost::
     :title: Multiple render window, scene tree and context menu
     :author: Yangyan Li
     :date: 6-15-2012
 
     PCL Modeler is able to render point cloud in specified render window, the scene tree is provided to keep a track of the objects in the scene, and context menu is provided to make functions more accessible to specified objects. The basic "view" related things are ready, and I will implement draggers to set the initinal positions for registration.

.. blogpost::
    :title: Evolve from PCLVisualizer
    :author: Yangyan Li
    :date: 5-25-2012

    PCLVisualizer enclosed many useful functions for point cloud rendering, I planned to inherit from it but then found something has to be changed in the parent class to support what I want, so I just copied PCLVisualizer to PCLModeler for now, and let it evolve from there as I progress. It's a simpler and faster solution since I can deal with PCLModeler only. They may be refactored to remove duplicate code if they do share a lot in the end.

    Another thing I want to mention is that sometimes UI may have very different behaviors on different platform(Windows and Linux in my case), and it's very painful to tune the code and then switch(restart!!) between them to make sure the code have consistent behaviors. Two computers are required in this case, store source code on one computer, make two builds, code on one platform(Windows in my case) and check results *instantly* on the other computer!


.. blogpost::
    :title: UI framework
    :author: Yangyan Li
    :date: 5-21-2012

    After some playing with vtk code, I am getting used to it. I've created an simple Qt based UI which supports load/save point cloud/project files and some utility functions. I will clean the code a bit, submit the diff for mentor reviewing and then make an initial commit. 


.. blogpost::
    :title: Setting up code structure
    :author: Yangyan Li
    :date: 5-11-2012

    I've prepared cmake files for placing PCLModeler under apps/ and set up a very simple UI for me to try and learn vtkPolyData. I think pcl_visualizer is a good reference for learning it.
    

.. blogpost::
    :title: Getting familiar to VTK
    :author: Yangyan Li
    :date: 5-7-2012

    I am new to VTK and after some digging I agreed the reputation that VTK has a steep learning curve seems to be true. However, it's really versatile and for sure is competent for this project. I am still not very clear about the VTK pipeline, but I think it should be OK after I read more code examples.


.. blogpost::
    :title: Setting up development environment
    :author: Yangyan Li
    :date: 5-6-2012

    I prefer to develop under Windows then double check and commit under Linux, and I've managed to get pcl compiled for both. Well, there are some errors for some modules, but I will leave it for now.


