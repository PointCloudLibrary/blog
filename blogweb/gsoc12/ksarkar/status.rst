My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Kripa



.. blogpost::
	:title: Initial arrangements 
	:author: Kripa
	:date: 5-16-2012

	I was having difficulty in accessing repositories with svn+ssh as I was behind a proxy server in my university with port 22 blocked. So here is the solution: get a direct connection! (well not exactly, but you have to get port 22 open somehow to get ssh working without server's help).
	Now I am in my home with everything working perfectly :).
	
.. blogpost::
	:title: 'Modern C++' 
	:author: Kripa
	:date: 5-25-2012

	I was browsing the code of PCL, hoping for good old C++ with lots of pointers and clean readable inherited classes like that in VTK and QT (`example <http://www.vtk.org/doc/release/5.8/html/a06173.html>`_).
	But instead, I found something like `this <http://pastebin.com/cPWV3Gga>`_ and a light bulb went off in my head.
	
	So, I understood that I must know and read about this 'modern C++'style which heavily uses templates, 'smart pointers', boost, etc, etc.. before I proceed further to play with pcl code.
	After doing some googling, I found the following book to be the best and recommended for people like me:
	
		* **C++ Templates: The Complete Guide / Vandevoorde & Josuttis**
	
	I have been reading this book and checking up boost, stl and pcl code since my last update. 
	
	Finally, I can now roughly understand pcl code and have started experimenting and exploring *pcl::visualization::PCLVisualizer* about which I will update soon. 
	
	
.. blogpost::
	:title: 2D Drawing APIs in PCL and VTK
	:author: Kripa
	:date: 5-27-2012

	Played with PCLVisualizer and surprised to see that it already contains API (simple function calls like addCircle) to draw primitives like Circle, Cube, Sphere, etc.
	It supports many other visualization functions; no wonder it is very powerful, much more than the simple CloudViewer.
	
	Also played with vtkContext2D and vtkChart and got overwhelmed with the amount of APIs (which bypasses the 'unnecessary' VTK pipeline) for 2D drawing and Charts. Next steps seems clear.
	
	* Extend these classes to form *PCL flavored* classes for 2D drawing and making charts. 
	* Either make a new 'visualizer' by subclassing vtkContextView or make a way to connect those extended classes to PCLVisualizer.
	
	.. image:: images/chartdemo.png
    				:width: 250px
				:height: 150px
    				:align: center
	
	
.. blogpost::
	:title: Hectic Times
	:author: Kripa
	:date: 6-17-2012

	Several things happened in the past weeks. It has been very busy and hectic.
	
	Firstly, I got relocated to Chennai (about 1500 KMs away from my home) for my job after undergraduate. I never expected the joining date to be this early. Usually, joining starts around mid July for most of the companies recruiting from our college.
	But, it seems that I got little (**un**)?lucky! Whatever the case maybe, I will make sure that this work on PCL goes smooth and gets completed.
	
	Before the relocation, I pushed myself hard to figure out the path, approach, and things I will use to get my API classes done. Browsing over vtk classes and getting help from Marcus, I made a concrete logical design (the way VTK objects should interact in the PCL API) before moving to this new place.
	
	Finally, after relocation, coding in the night, I completed the PCLPlotter class which is now available in the trunk under visualization. I will ask the PCL community about what else they want from the class.
	I will post about the features of this 'plotter' class in my coming blog. 
		

.. blogpost::
	:title: pcl::visualization::PCLPlotter
	:author: Kripa
	:date: 6-18-2012

	I completed the first draft of pcl::visualization::PCLPlotter which is available in the trunk. So, you can update your trunk to use it and give feedback and suggestions. 
	
	Using this class one can plot graphs, given the point correspondences. Several types of plots can be done which includes line, bar and points.
	
	It also includes API for plotting histogram given data. This is a similar and does the same type of functionalities as done by the histogram plotting function (hist) of **matlab**.
	
	The type of plot which can be created using this class is almost same as that of using its building block VTK classes. There is very less restriction without having any knowledge of VTK and its pipeline, which makes it powerful.
	I am adding some snapshots of the output of the plotter class to show its extent. 
		
	.. image:: images/sinecos.png
    				:width: 400px
				:height: 300px
    				:align: center
    	.. image:: images/sinecos1.png
    				:width: 400px
				:height: 300px
    				:align: center
    	.. image:: images/histuniform.png
    				:width: 400px
				:height: 300px
    				:align: center
    	.. image:: images/group.png
    				:width: 400px
				:height: 300px
    				:align: center
    	.. image:: images/smoothred.png
    				:width: 400px
				:height: 300px
    				:align: center
    				
.. blogpost::
	:title: PCLHistogramVisualizer rewrite
	:author: Kripa
	:date: 7-05-2012

	All important functionalities of PCLHistogramVisualizer are now incorporated in the PCLPlotter class. They are rewritten here so that this single class can take responsibility of all ploting related functionalities.
	The signatures of the PCLHistogramVisualizer functions are retained as of now so that one can directly use the PCLPlotter class with previous signatures to get similar result.
	I will post snapshots in the evening when I get a good internet connection. 
	
	I made some changes in pcd_viewer so that it can use this Plotter class instead of HistogramViewer. As signatures of the functions are same, the changes are minor; but the fact that it is now using this unified Plotter class.
	I will discuss with the community before committing the new pcd_viewer.
	
	I really want to get some feedback about this Plotter class. Please update the trunk and use the class to plot something nonsense and tell me if you would like to have something in addition.

.. blogpost::
	:title: Plot from polynomial
	:author: Kripa
	:date: 7-06-2012

	Added the functionality to plot from polynomial. This occurred to me as a useful functionality a plotter class should have. 
	User needs to provide a vector which stores the coefficients of the polynomial and range. PCLPlotter will plot and display them on the screen.
	
	I still don't have a good internet connection. I will post the snapshots later. 
	
	I am very happy that the training period of my job will get over by the end of next week. It won't be hectic after that as the office hours will go to normal (Currently its like 12-14 hours :()

.. blogpost::
	:title: Plot from rational functions and user defined callback function
	:author: Kripa
	:date: 7-07-2012

	Added two functionalities:
	
	  1) plot from rational functions which are the ratio of polynomials. Plot of 1/x looks nice.
	  2) plot from a user defined callback depicting the relation between Y and X axis. The function should be continuous.
	
	Snapshots coming!
	
.. blogpost::
	:title: Snapshots
	:author: Kripa
	:date: 7-08-2012

	I am adding snapshots of pcl_plotter in action showing examples of the functionalities which I discussed in my previous blogs. Till now, I didn't get a good internet connection in my new apartment, but I am uploading them from my limited cellphone connection anyway. 
	
	Most of them are plots from a given function like polynomial/rational or a user defined custom function. Last two snapshots are provided for the comparison between PCLPlotter and PCLHistogramVisualizer.
	
	* *Plot from polynomial*
		
	.. image:: images/x2.png
    				:width: 640px
				:height: 480px
    				:align: center
		
	.. image:: images/x2x3.png
    				:width: 640px
				:height: 480px
    				:align: center
    				
	* *Plot from Rational function: y = 1/x*
		
	.. image:: images/1byx.png
    				:width: 640px
				:height: 480px
    				:align: center
    	
    	* *Plot from an arbitrary Rational function: y = (3x^2 + x + 2)/(6x^5 + 5x^4 + 4x^3 + 3x^2 + 2x + 1)*
		
	.. image:: images/comprational.png
    				:width: 640px
				:height: 480px
    				:align: center
    	
    	* *Plot from user-defined callback function (eg taken: step, abs, and identity)*
		
	.. image:: images/callback_steps.png
    				:width: 640px
				:height: 480px
    				:align: center

    				
    	* *Comparison between PCLHistogramVisualizer and PCLPlotter*
		
	.. image:: images/histcomp.png
    				:width: 640px
				:height: 480px
    				:align: center

    	* *Multiple Histogram view in PCLPlotter*
		
	.. image:: images/histmany.png
    				:width: 1024px
				:height: 280px
    				:align: center
    				
.. blogpost::
	:title: Design of the 2D painter class 
	:author: Kripa
	:date: 7-09-2012

	In this post I will discuss about the design which I thought for the 2D painter class. The aim is to have a very simple interface (just like PCLPlotter) which allows user to add figures by simple add*() methods and, in the end, a display() method to show the canvas.
	Something like the following is desirable:
	
	*PCLPainter2D painter;*
	
	*painter.addCircle(0,0,5);*
	
	*painter.addLine(0,0, 5,0);*
	
	. . .
	
	*painter.display();*
	
	The underlaying implementation of PCLPainter2D in the above design will not be as straight forward as PCLPlotter where we have an instance of vtkChartXY and vtkContextView inside the class. 
	The only job was to convert the plot data (correspondences) to a format (which is vtkPlot) appreciated by vtkChartXY. That is, we had a direct mapping in term of functionality from vtkChartXY to PCLPlotter (with difference in the type of data they process and an additional "view" object in PCLPlotter).
	The problem in the above design is the fact that we don't have any vtkContextItem class which share similar properties of Painter2D class. Instead, 2D drawing works in the following way in VTK. The VTK user needs to first:
	
	  1) Make a subclass of vtkContextItem
	  2) Re-implement (override) Paint() of vtkContextItem. (shown in the figure)
		
	.. image:: images/contextItem.png
    				:width: 350px
				:height: 250px
    				:align: center
    	
    	It would be really nice to have a vtkContextItem class which cuts off the overhead of subclassing and allows user to draw directly from the function calls. Unfortunately, we don't have any (out of vtkChart, vtkPlot, vtkAxis,..., etc.) vtkContextItem class with that kind of behavior. 
    	So, before directly writing a Painter class for PCL it may be wise to write something like vtkPainter2D class for vtk and extend it to PCL. In this way it can be used to avoid subclassing in both VTK and PCL and its rendering could be further optimized in the future. 
	
	Now, the steps for creating "vtkPainter2D" (or PCLPainter2D) which would be a subclass of vtkContextItem are roughly the following:  
	
	1) Store information of 2D primitives in some data structures in every call of add*() calls.
	2) Implement Paint() using those data structures.
	
	These things have already been discussed with Marcus. It would be nice to hear your comments and suggestions.
    	
.. blogpost::
  :title: PCLPainter2D 
  :author: Kripa
  :date: 7-17-2012

  OK, so PCLPainter2D class is now available in the trunk. Currently it allows user to draw all the 2D primitives. The usage is same as discussed in design: 
  
  *PCLPainter2D painter;*
  
  *painter.addCircle(0,0,5);*
  
  . . .
  
  *painter.display();*
  
  The implementation is also exactly same as discussed in the previous blog. I'm storing drawing information in a data structure which is a vector of a class Figure2D. Later I'm using this information to re-implement paint() of the contextItem class (PCLPainter2D).
  
.. blogpost::
  :title: PCLPainter2D 
  :author: Kripa
  :date: 7-22-2012

   I added all the transform functionality in the Painter2D class now. So, one can perform transform operations like:
   
   *painter.addCircle(0,0,5);*
   
   *painter.translatePen(5,0);*
   
   *painter.addCircle(0,0,5);*
   
   *painter.setIdentityTransform();*
   
   . . .
   
   Since applying transform is a real time operation, it is required to keep track of when it is called. I solved this issue in similar way I tackled the underlaying vtkPen and  vtkBrush.
   I stored a transformation matrix for each of the figure and updated it with the *current* transform stored in the painter class as state. Like I said before, implementation of this 2D class was not straightforward as I thought before. In fact it has been full of tricks.
   
   Adding this functionality more or less completes the Painter2D class and my proposed work for the gsoc. For the rest of the period I will try to improve these two classes (PCLPlotter and PCLPainter2D)
   and add more features on request. So, if you think something should be added just email me; I will add if I think it is feasible ;-) I will also probably be assigned more work by Alex.
   
   
.. blogpost::
  :title: Additional features and spin*() functions
  :author: Kripa
  :date: 7-31-2012

  As I posted before, my work is now to improve and add additional features to my two classes PCLPlotter and PCLPainter2D. I always had in my mind that I will add spin*() functions (spinOnce(time) and spin()), which are a part of all the existing visualization classes (like PCLVisualizer, PCLHistogramVisualizer, etc), to my classes. Frankly, I did not understand these functions much based on the documentation and the code, perhaps because of my no knowledge of vtk's event handling (vtkCommand and all). All I knew that this functions someway start the interactor. 
  
  So, I finally understood those function after getting familiar to vtkCommand and going through their implementation. I kind of find the names confusing. *spinOnce(t)* runs the interactor event-loop for time t. *spinOnce* sounds like spinning (looping) one time which is confusing. *spin()* runs the interactor event-loop for indefinite time using *spinOnce(t)* thereby providing the ability to update the scene with time. But following is the description for *spin()* provided in the documentation in verbatim: "Calls the interactor and runs *an* internal loop.". Either I am missing out something or the documentation is misleading!
  
  Apart from the above, I was stuck for the most of the time figuring out the usage of *RepeatingTimer*. The repeating timer event is caught and the timer is destroyed- right in the first time! A **SingleShotTimer** very well suited this purpose. I did not understand the use of *RepeatingTimer*. I used *SingleShotTimer* in my spin* methods and it works as it should.
  
  Other than spin*() functions, I added *other features* about which I will post in the next blog. I would also like to comment here on the general design of the "visualizer"s in pcl. Unfortunately, blogging takes time and I will post about them in the next few days, one by one.
  

.. blogpost::
  :title: Bugs and additional features
  :author: Kripa
  :date: 8-7-2012

  Fixing bugs takes time. PCLPlotter was behaving weird in pcd_viewer. A window was appearing just after the creation of an object of 2D classes (Plotter and Painter2D) without even the call of plot/spin/display functions. Thus, had to move *vtkRenderwindowInteractor::Initialize()* and therefore *vtkRenderwindowInteractor::AddObserver()* to the display triggering calls (plot/spin/display).
  
  Added other small functionalities like *setTitle\** in Plotter.


.. blogpost::
  :title: Tutorials, bug with pcd_viewer, PCLVisualizer and important additions
  :author: Kripa
  :date: 8-16-2012

  Wrote `tutorials <http://pointclouds.org/documentation/tutorials/pcl_plotter.php>`_ for the 2D classes. Got another weird bug found by Radu. Plotter not working in pcd_viewer on point picking. Still struggling in it.
  
  Tried to make PCLVisualizer cleaner and readable. Removed some unnecessary function calls. Didn't commit yet.
  
  Added some important functionalities in Plotter and a seperate vtkCommand event handler.
