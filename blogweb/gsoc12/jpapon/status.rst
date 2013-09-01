My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Jeremie
  
.. blogpost::
  :title: Cloud Manipulations!
  :author: Jeremie
  :date: 20-8-2012

  Just a quick update to let you know that you can now manipulate clouds using the mouse. This allows you to do some pretty interesting things when combined with other tools, like Euclidean Clustering. 

  For instance, in the image below, there are three frames. The first (left-most) shows the original point cloud. I use the Euclidean clustering tool on it, which allows me to pull out the hand as a separate cloud, and select it (now shown in red in the middle). I can then use the mouse manipulator to move the hand as I please, and, for instance, have it pick up the red peg (right most frame).

   .. image:: images/ManipulateHand.png
    :width: 1400px
    :height: 358px
    :align: center

  Everything is coming together; the core of the app is now in place... basically I just need to implement the rest of the PCL tutorials as plugins, and you can do basically everything PCL can do... with the exception of working with movies.

.. blogpost::
	:title: Handling templated clouds, Double Dispatch in cloud_composer
	:author: Jeremie
	:date: 13-8-2012
	
	So I finally decided to bite the bullet and add support for the templated classes to the GUI, rather than just using the sensor_msgs::PointCloud2. After some discussion on the dev boards and in irc (thanks to everyone for their input, especially Radu), I decided to put together a system which allows use of the templated classes even though point type is unknown at compile time. This is inherently kind of difficult, since templates and run-time polymorphism are two opposing concepts. 
	
	Before I get into the technical stuff, let me just say that the consequence of everything that follows is that for every CloudItem, we maintain both a PointCloud2 and a templated PointCloud<> object. Because of the way the model was designed to enable undo/redo, these are always synchronized automatically, since they can only be accessed through the CloudItem interface. 
	
	Everything is centered on the CloudItem class, which inherits from QStandardItem. For those who haven't been following, the application is built around these types of items, which are stored in a ProjectModel object. There GUI is essentially a bunch of different views for displaying/editing this model. Anyways, clouds are always loaded from file as binary blobs (PointCloud2 objects).   The header of the PointCloud2 object is then parsed to determine what the underlying data looks like. By doing this, we can figure out which template PointType we need, and using a nasty switch statement, can instantiate the appropriate PointCloud<T> object. 
	
	We then take the pointer to the PointCloud<T> and store it in a QVariant in the CloudItem (we also store the PointType in an enum). When we fetch this QVariant, we can cast the pointer back to the appropriate template type using the enum and a little macro. This means one can write a tool class which deduces the template type and calls the appropriate templated worker function.   
	
	One interesting thing popped up when I was doing this. The tools use run-time polymorphism (virtual functions) to determine what work function to call. That is, I manipulate base-class Tool pointers, and let the v-table worry about what type of Tool the object actually is. A problem arises with templated types though, since virtual function templates are a `no-no. <http://	http://stackoverflow.com/questions/5742531/virtual-template-functions>`_  
	
	To get around this, I worked out a double dispatch system - `the visitor design pattern. <http://en.wikipedia.org/wiki/Visitor_pattern>`_ This allows me to determine what code to execute based on the run-time types of two different objects (rather than just one, which could be handled by a vtable). The core idea here is that we first do a run time look up in the vtable to determine what tool is being executed, passing it a reference to the CloudItem. We then take the QVariant, deduce the PointType of the cloud it references, then execute the tool using the appropriate template type.

	I'm sorry if none of that made sense... I'll draw up some diagrams of how the whole application functions in a week or so, which should help alot. I'm now finishing up the code which allows manipulation of vtk actors with the mouse to be applied back into the model (with undo/redo as well).
	
	tl;dr: Templates now work even though you don't know the PointType at compile-time. Mouse manipulation of actors in the PCLVisualizer render window now works, and will shortly be properly propagated back to the model.
	
	
.. blogpost::
	:title: Cloud Commands: Now with 99.9% Less Memory Leakage!
	:author: Jeremie
	:date: 8-8-2012

	Today I reworked the Cloud Command classes and their undo/redo functionality so that everything gets properly deleted when the time comes. There are two different scenarios where a command needs to get deleted:

		*   When the undo stack reaches its limit (set to 10 commands atm), we need to delete the command at the bottom of the stack (first in)
		*   If we eliminate the existence of a command by undoing it, then pushing another command on the stack
  
	These two cases have different deletes though. In the first case we need to delete the original items, since we've replaced them with the output of the command. In the latter case, we need to delete the new items the command generated, and leave the original ones alone.

	Additionally, different commands need to delete data which is structured in different ways. For instance, a split command (such as Euclidean Clustering) needs to delete one original item or many created items, while a merge command needs to delete many original items or a single created item... and so forth... 

	Oh, on a side note, I'm not sure what I should have the app do when one tries to merge two Cloud items which have different fields. For now, I just say "Nope"... but there may be a way to combine them that makes sense. Unfortunately this currently doesn't exist for PointCloud2 objects in PCL afaik. Anyone feel like adding some cases to pcl::concatenatePointCloud(sensor_msgs::PointCloud2&, sensor_msgs::PointCloud2&, sensor_msgs::PointCloud2&) that checks for different but compatible field types? For instance, concatenating rgb and rgba clouds could just give a default alpha value for all the rgb cloud points. This would be pretty easy to code if one did it inefficiently by converting to templated types, concatenating, then converting back...  
   
    
.. blogpost::
    :title: Merge Cloud Command, Create New Cloud from Selection
    :author: Jeremie
    :date: 8-7-2012   
 
      Just a quick update on what I've added. 
      
      The frustum selection now connects back to the model through an event it invokes. It can search for points if needed, but right now I'm sanitizing all clouds (no NaNs) so there's a 1-1 correspondence from vtk to PCL. Of course I have to figure out what cloud the vtk point belongs to, but that can be done using the CloudActorMap from PCLVisualizer.
      
      I've added the last undo/redo command, merging. I also added a tool which creates a new cloud from the current selection. There are now two "selectors"; a green one for points selected in the VTK window, and a red selector for points selected in the cloud browser dock. Right now all commands applied to selections work on both, but that may change in the future.
      
.. blogpost::
    :title: Axes Widgets, Signal Multiplexer, InteractorStyleSwitch, Rectangular Frustum Selection
    :author: Jeremie
    :date: 8-3-2012 
    
      Crunch time is here, so I'll just give a short update on the things I've added:
      
      There's an axes widget now that shows the orientation of the camera, as in ParaView.
      
      I added a signal multiplexer class which greatly simplifies the architecture. This class allows you to specify connections without having to specify the receiving object when you make the connections. You then can switch receiving objects, and all the connections are adjusted automatically. For me, this means that I have all my GUI actions connected to the multiplexer, and then I just tell the multiplexer which project is currently visible. The GUI actions are then routed to that project, and all the other projects just idle. It also allows me to update the GUI state automatically to project state when projects are switched. Why a class like this isn't included in Qt itself is kind of a mystery to me.
      
      I added an InteractorStyleSwitch class which allows you to switch what interactor style PCLVisualizer is using. This means that with one function call we can switch from the normal PCLVisualizer camera nteractor style to one which lets us select sub-clouds with the mouse.
      
      Which brings us to the last addition, the ability to select subclouds using a rectangular frustum selector. As seen in the images below, this works, and it works on multiple clouds in the scene. What it doesn't do yet is take the selected vtkPolyData points and figure out what indices (and what clouds) they correspond to in the original PCL clouds contained in the Project Model. 
      
      That's for tomorrow... plus the ability to split off the selected points into a new CloudItem (with undo/redo of course).
      
      .. image:: images/frustum_select.png
        :width: 1200px
        :height: 275px
        :align: center
        
      Stay classy, PCL blog readership.

.. blogpost::
    :title: Tool Input Checking
    :author: Jeremie
    :date: 7-30-2012 
    
      So I implemented a way of showing what inputs are required for a tool and if the currently selected items are a valid input for the tool. It's a pretty simple system really; if the selected item(s) is valid input for a tool, it becomes enabled, and you can use it. If the selected item(s) don't match what is needed by a tool, the tool is greyed out, and you get a tooltip which says why you can't use the tool. In the case of the image below, that's because calculating FPFH features requires normals. 
      
      .. image:: images/Tool_Input.png
        :width: 492px
        :height: 235px
        :align: center
        
      Now I'm working on mouse selections in the QVTKWidget/PCLVisualizer view. Things like rectangular-drag and point by point.
      
      Then I'll add registration, the merge command, and the ability to manually shift point clouds (using the keyboard, and perhaps the mouse).
      
      Then I'll add as many additional tools as I can manage before GSoC ends. It's crunch-time; it's going to be tough to implement all the features I would like to by the middle of August. This has turned out to be a pretty large project for one person... Fortunately the end of GSoC won't be the end of this App; I'll be contributing one day a week this Autumn/Winter to continued development of it.
      
.. blogpost::
    :title: New Plugins, Deleting, Properties, a Screenie
    :author: Jeremie
    :date: 7-27-2012 
    
      Another quick pre-sleep update... Voxel Grid Downsample and Statistical Outlier Removal have been added, which means the cloud modification undo/redo functionality works now. Delete undo/redo command has also been added. The only remaining command type to be added is the merge command, as would be needed for registration. This is up next. 
      
      Then on to selection tools... which may need to be implemented directly, rather then as  plugins, since plugins have no way to interact with the VTK render windows (and they shouldn't, since their processing is done in non-GUI threads). Adding additional tools now is really quite fast... though I think at some point in the coming days I'm going to polish the plugin interface a little bit, as well as clean up the items, mainly by giving them a templated way to get/set data that doesn't rely on Qt's awkward UserRole enum system. 
      
      I'll leave you with a little screenshot of what it looks like now. 
      
      .. image:: images/CloudComposer_SS_27JUL12.png
        :width: 800px
        :height: 600px
        :align: center
      
      Oh, one other thing, if you're fooling around in the GUI, and something goes horribly wrong, please submit a bug report. The whole app is starting to become a little more complex these days, so ugly bugs that I haven't found are bound to start cropping up. 
      
      Feel free to submit feature requests too... but I make no guarantees as to how long it might be before they can be implemented!
  
.. blogpost::
    :title: Clustering, Splitting clouds, Selection highlighting
    :author: Jeremie
    :date: 7-24-2012 
    
    Just a quick update before I go to sleep... Euclidean clustering now works; this means that the cloud splitting command & undo/redo functionality is implemented now too. If you load a cloud, run clustering, it will split it into clusters and whatever remains from the original. Basic (ugly) highlighting now works too; the cloud selected in the browser on the left gets "highlighted" in the view by turning red... which is terrible. I want to do a more subtle highlighting effect, maybe some sort of glow, or tinting, but my attempts are failures so far. I played around with adding a colored ambient/diffuse light to the selected cloud actor (in pclVisualizer) but none of the lighting changes I made had any effect. I suspect I'm making some sort of noobish vtk mistake. I think I need to read through a quick vtk tutorial tomorrow morning.
    
.. blogpost::
    :title: To sanitize or not to sanitize?
    :author: Jeremie
    :date: 7-21-2012 
    
    So I've stumbled upon a design issue that I'm not sure how to handle. The problem is that some functions in PCL require sanitized input (such as FPFH), namely, they can't handle NANs. This means I have to use either a passthrough filter or the removeNAN filter at some point. It might seem natural to do this within the plugin that needs sanitized inputs, but then the result won't really  correspond to its input cloud, ie, it will have less points. To get around this, I'm just sanitizing all input clouds when they are loaded, but this isn't a satisfactory solution either, since it means you break up organized clouds (and modify the cloud automatically, which the user might not want to do). 
    
    So, what should I do here? Should plugins specify that they require sanitized input, and have their tool icon greyed out unless the selected input cloud is valid? What would be the best way to notify the user about why a tool is greyed out? Tooltips which give the reason it is disabled when you hover over the tool icon?
    
    This seems like a sensible solution to me, but please, if anyone actually reads these blogs, let me know if you have a better idea, or if my solution seems foolish to you.
    
    Oh, and more tools are coming, I promise, I'm just trying to make the plugin specification as complete as possible before I start implementing more tools... and I keep discovering things it is missing.
    
.. blogpost::
    :title: Changes to displaying, interaction with models
    :author: Jeremie
    :date: 7-18-2012 
    
    I haven't been posting much on the blog, I'll try to keep up with this better.
    I've done some rewriting of how things are displayed. Now the item classes define their behavior with respect to the various view classes. This means that a CloudItem defines its own paint function, which the View window (which uses PCLVisualizer)  just calls when it needs to paint the CloudItem (ie, when it changes, when its added/removed).
    Also, now when changes are made in the inspector to properties they propagate through the models and update all the other views. For now, I'm only making properties editable which don't require another call to the actual tool functionality, just ones that change how things look. For instance, you can change the scale and level of Normals, but you can't change the radius, since that would require recalculating them, which is an expensive operation. This may change in the future, but for now, if you want to change the radius, you just have to calculate a new set by running the tool again (with a different radius).
    
.. blogpost::
    :title: More plugins, more fun
    :author: Jeremie
    :date: 7-09-2012

    So I've finally gotten my satellite internet connection up and running here in France, so now I can start committing again every day.
    I'm finished with the FPFH plugin, with display of histograms in the inspector view. 
    Now I need to work on a splitting plugin, segmentation, so I'll have to spend a little more time in getting the undo/redo stuff working for that.
    I think I need to do some more thinking about the structure of how the items are working as well. Right now I'm still using a QStandardItem subclass, where it probably makes more sense to subclass from the QAbstractItem directly and implement some things myself. I'll probably spend the next day working on that, along with the segmentation plugin. 
    The plan is to have the following plugins working by the end of the week: Normals, FPFH, Euclidean Segmentation, Plane Segmentation, ICP for registering two clouds.
    
.. blogpost::
    :title: First functional plugin!
    :author: Jeremie
    :date: 6-22-2012

    The normal estimation plugin works as you might expect it would- it calculates normals. It uses the undo/redo framework, and the work_queue system, so you can undo/redo adding normals to your project as much as you want, and calculations are done in a separate thread, so the GUI doesn't lock up while it's thinking. 
    
    I'll add progress bars soon, but that begs the question, how can I estimate progress for pcl functions? I can emit progress updates BETWEEN individual PCL calls in a plugin (such as between KD tree calculation and normal estimation in the normal estimation plugin) but getting real timing info would require putting some sort of macro in the functions themselves.
    
    Another consideration is how tools should be activated. Right now I have a button which you click, which runs the selected tool. This of course is only temporary, but I'm not sure what the best replacement is. For selector tools, it's pretty easy, but for things like adding normals or segmentation, what's the most intuitive way of activating the tools?     
    
.. blogpost::
    :title: A Quick Update on Plugins
    :author: Jeremie
    :date: 6-21-2012

	Work is progressing on the plugin tool, undo/redo command, work queue. After spending quite a bit of time thinking about how the architecture would work, I had a basic idea of how the components would connect together. So, after making the interface and basic headers, I began writing the first plugin tool (normal_estimation). As I went along, I realized certain parts of the design weren't practical, or inefficient (such as having commands handle all memory and preventing plugins from reading from the model items directly, also, having the work queue dispatch multiple worker threads at once). Overall though, the design is what I showed in my last post, and things are coming together well, even if somewhat slower then I would have hoped.
	Once the first plugin is finished, things should progress quickly in getting the other tools working, since, for the most part, I'll just be implementing the tutorials as plugins.
	  
.. blogpost::
    :title: A Closer Look at the Framework
    :author: Jeremie
    :date: 6-15-2012
    
	Well, I've spent the last week or so putting the model/view stuff together into a useable GUI, which is available in the trunk under apps/cloud_composer... but before you run off and try to use it, let me warn you... it doesn't do anything (other then show you clouds). This is because the next major step in the project, the tools plugins, are just getting started. More about that later though, first lets go over the current state of the Model/View structure.

	Lets go over the components:

		* Project Model: This is primary model "container" of the app. It can contain an unlimited number of clouds, which are stored in a tree structure. The top level of the tree (children items of the root item) are clouds. The children of the clouds are things like normals, point feature histograms, labels, calculated models, etc... All of these are stored in their own items, which are derived from abstract base class ComposerItem. They can represent any sort of data, but shouldn't actually contain the data, but simply store pointers to the data. Currently the pointers are stored in QVariant objects, allowing me to use the standard Qt get/set data functions... but I think this may change in the future, as I think I'll probably need finer control over what the set/get data functions are doing. In any case, along with pointer(s) to the data, the ComposerItems have another important member:a QStandardItemModel, which contains its properties. These are things like height/width for a cloud, or radius for normals. 

      These properties can be editable or not (though if they are, the plugin which defined the item will need to specify slots which can handle what happens when a property changes).

		* CloudBrowser - This is the main tree viewer for the project model, showing what clouds are in the project, and what items those clouds contain. Selecting an item or cloud here will bring it up in the CloudInspector.
		* CloudInspector - This shows the properties of an item (which is a model itself, contained within the item) in a tree view. It allows editing of fields if possible. Since the properties are a model themselves, one can easily specify that widgets should be shown here for a property, such as a QSlider for a parameter which has a range.
		* CloudViewer - This is a tabbed view which shows the projects currently open as different tabs. Each tab contains its own QVTKWidget and PCLVisualizer. When a new tab is selected, the current model for the cloud_composer application is switched. This makes it very simple to switch back and forth between projects, and ensures that all the views are updated correctly whenever a switch occurs.
		* UndoView - This is a QUndoViewer, showing the current undo/redo stack, and lets one jump back/forward in time by clicking. Many PCL operations aren't reversible (and take a long time to compute), so we store clouds in the stack, so undo/redo just amounts to switching the pointers in the cloud_item(s) which were modified.
		* Tools - this still just a blank toolbox, but will contain icons for the different plugins. More on plugins after the image.

		.. image:: images/PluginArchitecture.png
			:width: 950px
			:height: 547px
			:align: center

	So that's the plugin framework in a nutshell. None of that XML stuff I mentioned before, though that may come at a later date (i.e. after GSoC). Let's go over what happens when a user clicks on a tool icon.

		#. Clicking on the tool once causes the tool parameter model to be displayed in the tool parameter view.
		#. Clicking on the tool again causes the tool action to trigger, unless it is something like a drag selector, in which case clicking in the cloud view triggers the action.
		#. The action triggering causes the tool's factory to create a new instance of the tool and a new instance of one of the command types (merge, split, modify, delete, create). These commands objects are all defined in the GUI code, not inside of individual plugins. This is very important; plugins do not actually interact with the model, only commands do. This makes things like undo/redo and safe multithreading feasible.
		#. The cloud command is sent to the work queue, which is responsible for spawning threads where tools do their processing. It maintains a reference count of what items from the project model are currently being processed, and pops commands off the stack until the top command depends on data currently being worked on. The command object is responsible for making a copy of the item being worked on, and sending the copy and the tool object off to their thread to go do work.
		#. When a tool returns with a result, the command will push the old value onto the undo/redo stack, and send the resulting data back to the project model.

	That's about it for now... I'll let you know when I've got a demo tool up and running; I'm starting with normal estimation. I'm sure some of this architecture will be changed as I come across things that aren't possible, or think of better ways to do things. If any of you see something of that nature right now (won't work, or a better way) please let me know! Also, you get a cookie for having managed to read all the way through this.
  
.. blogpost::
	:title: Model/View Framework for PCL
	:author: Jeremie
	:date: 6-3-2012
	
	So, I've been working on the basic GUI, and I've decided to go with a classic model/view architecture, using the Qt framework. In architecture, the data the user sees is encapsulated inside of a model, which can then be viewed using various GUI elements. This decouples the data objects from the viewing interface presented to the user, which increases flexibility and reuse. 
	So, the architecture looks something like this:

		.. image:: images/CloudComposerArchitecture.png
			:width: 950px
			:height: 587px
			:align: center

	The core idea here is that we have the CloudModel as the core object, maintaining references to the multiple clouds it may contain - multiple being necessary to allow things like registration, or segmentation of points into distinct clouds to be manipulated. The CloudModel maintains information about the clouds, which can be viewed in treeform in the CloudBrowser. This will look very much like the Pipeline Browser in Paraview. Additionally, clicking on an element in the browser will display further information about the selected element in the CloudInspector. Things like number of points, datatype, etc... It will also allow the adjustment of properties; say you have normals selected, it will allow you to adjust the radius used to calculate them. 
	
	There's also the CloudToolSelector, which is an interface to the plugins which provide PCL functionality. As I said in my previous post, I'm still on the fence on how to implement the plugins. Ideally, I'd like them to be generated automatically based on XML description files, but it remains to be seen how difficult that will be, and if it is even possible due to the templated nature of all the PCL functions.
	
	Finally, there's the CloudViewer, which implements a QAbstractItemView, and contains a QVTKWidget - PCLVisualizer. The eventual plan is to have this be a tabbed view, with tabs switching between cloud projects, ie, switching which model is being viewed. That will come later though, lets get it working with one project first...
	
	In any case, I'll push this basic framework (minus the tools) to the SVN in the coming days. Let me know what you think, and if anyone out there sees any flaws in this architecture, please let me know. This is my first foray into the model/view world, and I'd appreciate finding out if I'm doing something wrong sooner rather than later!

.. blogpost::
	:title: GUI for Manipulating Point Clouds
	:author: Jeremie
	:date: 5-30-2012

	Hello everyone, I just wanted to give a belated introduction to this project, and a quick status update on what I've been up to. 
	To begin with, the goal of this project is to develop a GUI which acts as a graphical means of using the various modules of the PCL.
	The basic idea is to develop something is similar to `Paraview <http://www.paraview.org/>`_ (without the distributed part, that may come later). Basically one can load multiple clouds, or capture them from an OpenNI device, and then apply PCL functions to analyze them, modify them, or merge them.
	The interface is a pretty standard Qt design, with docks on each side containing tools, a list of clouds in the current window, and a bottom dock with text output. PCL calls are performed in separate threads of course.
	I have the basic application layout done, with basic functionality - loading/saving clouds and viewing them using the PCLVisualizer class. I'll be pushing it to the server as soon as I get back from this review meeting in Denmark. I'd like to apologize for the slow start here, I haven't been home in 3 weeks now thanks to conferences and meetings, and so all I've really been able to do is read.
	On that note, I'd like to discuss *what* I've been reading, and what I intend to do with it.
	Let's start with what I, and I assume the community, wants. Namely, a GUI application which is easy to maintain and extend as the PCL code behind it evolves:

		* Changes in underlying algorithms should have no effect on the GUI.
		* Changes to module interfaces should require as little change in GUI code as possible.
		* Adding new functionality shouldn't require editing the application code - when a programmer adds a new function, they should be able to add it to the GUI with minimal hassle and no without the possibility of breaking the rest of the app.

	This leads us to a few conclusions. First of all, we need to isolate PCL functionality into a set of plugins. This could be one plugin per module, or one plugin per tool (ie FPFH calculation, SACModel calculation, Outlier removal, etc...), or any level of granularity in between. 
	Next, the interface for these plugins should be as simple as possible, while still remaining flexible enough to allow all of the PCL functionality to pass through it.
	Finally, when someone adds a function to PCL, they should be able to add it as a tool in the GUI with minimal, *if any* coding.
	In my mind, this leaves us with two options:

		* A standard plugin system, where we define an interface, and then code a class for each tool, which performs the desired calls to PCL.
		* A meta-compiler system, where tools are specified in an XML format, and we either parse it at either run-time (to determine what to do when a tool is selected) or at compile time (to generate code which is used when the tool is selected).

	The second option is obviously more complicated, but would be much easier to maintain in the long run, since the only code would be the compiler/parser. The XML specification of how to use the PCL would be relatively simple, which would make updating and adding tools as simple as changing/adding a few lines of XML (copied from a general template). In the first option, a new tool (or changes to a modules interface) would require editing the code of the plugin class. This means (imho) that tools would be much more prone to breaking.
	So, what am I reading?

		* Di Gennaro, Davide. *Advanced C++ Metaprogramming*, 2012.

	Which is kind of blowing my mind. I'm feeling more confident about templates by the day, but I'm also beginning to think this may be overkill for the project. On the other hand, I'm not terribly interested in programming another standard plugin interface. That would mean I was basically spending the entire summer writing implementations which call PCL functions... which would be prone to breaking, and would require quite a bit of babysitting to keep the application running properly. I know how to do that, there's nothing new there, and I'd just be making a clone of many other things which are already out there; just with a PCL backend. The XML version would be pretty novel (at least for me - Paraview does something somewhat similar), and would result in an application that would be very easy to extend as PCL evolves.
	On the other hand, the XML version is higher risk- it could result in a month of coding which fails miserably, followed by a frantic month of implementing the standard plugin class version.
	
	**Now if you've made it through all the text, I'd like to ask what do you guys think? Any suggestions, or advice? As I said, I'll be at this review meeting until the end of the week, so I won't be starting any serious coding of the plugin mechanism until next week. I would really appreciate a discussion of this over the next few days.**
	
	Oh, and what's a good name for this beast?
	I've come up with the following two:

		* Cloud Composer
		* Cloud Jockey

	**Anyone have anything better?**

