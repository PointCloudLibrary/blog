My status updates
=====================

.. blogbody::
  :nr_days: 60
  :author: bbferka

.. blogpost::
	:title: Getting started 
	:author: bbferka
	:date: 5-21-2012

	These past weeks I have been reading up on the two segmentation methods described in the papers (Mean-shift and Active Segmentation in Robotics) and in the meanwhile checking out the segmentation API and the available tutorials in PCL. I worked on setting up my environment and making everything work. My next steps will involve doing some research on the adaptation of Active Segmentation to 3D data as well as creating some draft classes and outlining the main steps of the method.

.. blogpost::
	:title: Progress on Active Segmentation
	:author: bbferka	
	:date: 6-16-2012
	
	As I mentioned in my first entry, I have decided to implement/adapt the Active Segmentation with Fixation approach first, developed by A.Mishra. As a first step I took the time and thoroughly studied the works of the aforementioned author, and found that there are several improvements to the first approach. Details on this can be found on the authors `home page <http://www.umiacs.umd.edu/~mishraka>`_. 
	
	For those who are not familiar how active segmentation based on fixation works and don't want or don't have the time to read the initial paper here is a short description. The reasoning behind the choice of segmenting based on a fixation point originates from the way humans perceive objects in their surroundings. This method proposes an approach where we do not segment the whole image in several regions and then reason on what these regions might be, but segment only the region which contains our fixation point. So in short segmenting an object from the scene implies finding a "fixation point" and finding the contour of the object that encloses our point of interest. 
	The main steps of the segmentation algorithm are(these steps are for rgb images using monocular cues):
	
		1. Obtain the boundary map 
		2. Improve boundary map using using monocular cues.  
		3. Set fixation points (points chosen must be part of the object we want to segment out).
		4. Convert to polar coordinates around the fixation point
		5. Search for shortest path/ apply graph-cut algorithm in order to find the contour of the object.
		6. Convert back to Cartesian space, points found on the "left" of the shortest path in polar space belong to our object of interest.

	Later works improve on this method among other things by adding an automated fixation strategy for finding interest points and by using depth information. Although, as stated in the work of A. Mishra we can find object boundaries by checking for depth discontinuities, but often these boundary points do not correspond to the true boundaries of the objects. In order to find the true boundaries color and texture cues are recommended. While the purpose of my project is the implementation of the segmentation algorithm, if I find the time for it, I will pursue the implementation of a fixation strategy.
	
	Implementation wise I have created the basic API of the segmentation based on the the other segmentation algorithms already implemented in PCL. I played around with existing implementations of boundary detection and I realized that I will probably be able to use the code of my fellow GSOC developer `Changhyun Choi <http://www.pointclouds.org/blog/gsoc12/cchoi/index.php>`_. I am looking forward to seeing how his different implementations of edge detection will influence the performance of segmentation. 

	I will be back with more information soon (mostly on implementation issues).

.. blogpost::
	:title: Quick update 
	:author: bbferka
	:date: 6-21-2012
	
	I'm trying to push harder this week to try and get an implemented working version of the active segmentation approach. I started inserting my already existing code parts in the PCL API, created the base classes and added the basic functionalities to it. I'm planing on creating a flowchart to illustrate how things work, but life keeps getting in the way:). Anyway, I am having a little bit of trouble on deciding which way to go after I have the fixation point and the boundary map. I have to decide on doing this the way the guys did it in the original publication and implement a mapping between the 2d image and point cloud, or I could try implementing it using region growing having the fixation point as the seed. I will have to decide on this, or maybe try out both and see where that leads me.

.. blogpost::
	:title: Short notification
	:author: bbferka
	:date: 6-24-2012

	After thinking and discussing about it I have decided to go on and implement a region growing method based on a boundary map and the fixation point. Calculating and setting the boundary map  will be left up to the user the method segmenting the region of the fixation point that is enclosed by w boundary. I will also shortly add an example of how this is done in pcl/examples. 

.. blogpost::
	:title: Segmenting around a fixation point
	:author: bbferka
	:date: 6-29-2012

	As promised in a previous post I took the time and created a basic flow chart of the involved steps of segmenting around a fixation point. Upper part of the chart (getting a boundary map, setting input cloud etc. ) are steps that need to be implemented by whoever want to use the *ActiveSegmentation* class(example of this will be shortly available *pcl/examples*), steps in the lower part are implemented in the class. It is left to the users discretion to choose an appropriate boundary map. I chose to do it this to have a bigger flexibility, and because many others are working on edge/boundary detection at the moment. My test will be based on the already implemented Boundary detection (which works for ordered and unordered point clouds as well) and trying it out with mapping 2d edge detections to the 3d cloud.

	* **Flow Chart Active Segmentation**

	.. image:: images/as/flow_chart_as.png
		:width: 943px
		:height: 1219px
		:align: center
	
	In my next post I will share some preliminary results as well.
	
.. blogpost::
	:title: Region growing based only on boundaries. 
	:author: bbferka
	:date: 7-02-2012
	
	For the approximation of the boundaries	I used the class implemented by our fellow GSOC student Changhyun Choi.

	Because the active segmentation method is one that is most likely to be used in a robotic application, as described in the paper scenes involved are table top scenes containing several objects. I chose the scene below as the one for testing because there are multiple objects on the table some occluding others, but as this is the scene that i will be running the algorithm on the first time I did not want it to be too complex (e.g. extremely cluttered scenes)

	.. image:: images/as/scene_rgb.png
		:width: 640px
		:height: 372px
		:align: center
	
	After implementing and testing the first version of the region growing I got the results shown in the screen shot below....just to clarify....everything that is blue gets segmented out. The green points on the large box are the points near the "fixation point". This version of region growing is based only on the borders and since there are no borders at the touching point of the box with the table growing does not stop. To get around this problem I will take into consideration the normals of the points as well when growing. 

	.. image:: images/as/scene_bad.png
		:width: 640px
		:height: 372px
		:align: center

.. blogpost::
	:title: Reading up on mean shift
	:author: bbferka
	:date: 7-05-2012

	Since I was a little swamped with work this week I did not have time to try out how adding the normals will change my region growing yet, so I did some reading in my spare time about mean shift segmentation and existing implementations. Since the original papers on mean shift are not the easiest to understand and to clarify to myself about steps involved in segmenting i started searching for other articles about this topic. I've found this article to be the most helpful:  

		* `Introduction to mean-shift <http://saravananthirumuruganathan.wordpress.com/2010/04/01/introduction-to-mean-shift-algorithm/>`_

.. blogpost::
	:title: adding normals and more
	:author: bbferka
	:date: 7-08-2012
	
	I've finaly managed to add the normals to the region growing algorithm and results are promising. I've conditioned adding of points to the region on the angle between the seed points' normal and the current point. Results doing this are shown in the screen shot below.

	.. image:: images/as/scene_better.png
		:width: 640px
		:height: 372px
		:align: center

	Adding this condition helped and not at the same time. Although now growing stops when we reach the table top, parts of the object that are parallel to it don't get added as well. This was expected of course. Still...better then the first try. 

	Thanks to a friend of mine I found out about this theory: 

		* `Recognition by Components <http://en.wikipedia.org/wiki/Recognition_by_Components_Theory>`_ 
		* `the publication <http://geon.usc.edu/~biederman/publications/Biederman_RBC_1987.pdf>`_

	The interesting part for me is that according to this theory every object can be broken down into piece primitives and there are only 32 kinds of these primitive shapes and all of them are convex, meaning that complex objects can be separated into these parts at their concavenesses.

	The following is an extract from the book *From Fragments to Objects - Segmentation and Grouping in Vision T.F. Shipley and P.J. Kellman (Editors)  2001 Elsevier Science B.V. All rights reserved.*

	*"The simplest shapes are convex shapes—whose outlines have positive curvature throughout (see, e.g., Rosin, 2000, for the role of convexity in parsing). If the outline of a shape has regions of negative curvature, especially if these regions contain salient negative minima of curvature, this usually indicates that the shape can be further parsed to give simpler subshapes. [...] Three main geometrical factors determine the perceptual salience of a part (Hoffman & Singh, 1997): (1) its protrusion, (2) its relative area, and (3) the strength of its boundaries.  Salience of a part increases as its protrusion, relative area, or boundary strength increases.  In this section we briefly consider protrusion and relative area, and then discuss in more detail the strength of part boundaries.  We restrict attention to 2D shapes; the theory for 3D shapes is more complex and discussed elsewhere (Hoffman & Singh, 1997)."*

	*"Hypothesis of normalized curvature: The salience of a part boundary increases as the magnitude of normalized curvature at the boundary increases."*

	*"Hypothesis of Turning Angle: The salience of a negative-minimum boundary increases as the magnitude of the turning angle around the boundary increases."*

	Based on these theories I introduces another constraint to my region growing: when adding a point the angle between the line connecting that point to the fixation point and the points normal is checked. If this angle is concave it means that the point currently being verified belongs to the object. Doing so resulted in the following results (blue points are the ones segmented out and the green patch on the objects is the neighborhood of the fixation point): 
	
	.. image:: images/as/scene_1.png
		:width: 640px
		:height: 372px
		:align: center
	.. image:: images/as/scene_2.png
		:width: 640px
		:height: 372px
		:align: center
	.. image:: images/as/scene_3.png
		:width: 640px
		:height: 372px
		:align: center
	.. image:: images/as/scene_4.png
		:width: 640px
		:height: 372px
		:align: center
	.. image:: images/as/scene_bad_2.png
		:width: 640px
		:height: 372px
		:align: center

	As it can be observed introducing that extra condition improved results a lot.The last scene is not a good result. My next step will involve investigating the cause of that erroneous segmentation. As a final observation, it was clear for me since the beginning, that the whole of this method is dependent on having a good fixation point. If this is not the case, the algorithm would not work at the moment. One of my next steps will be to investigate the method recently proposed by the authors of the original paper for automatic fixation point estimation.
	
	With this occasion I would like to thank Zoltan-Cs. Marton for coming up with the idea of using convex vs. concave angles, it helped me a lot:). THX m8:)

.. blogpost::
	:title: fixing things
	:author: bbferka
	:date: 7-17-2012
	
	This past week I have been fine tuning and rewriting the region growing part of the segmentation method in order to fix the eroneous segmentation shown in my last post.

.. blogpost::
	:title: fixed problem
	:author: bbferka
	:date: 7-25-2012
	
	I managed to fix the region growing, so now, if the seed point is on a plane paralell to the table top the method does not fail. Below you can see some screen shots of the 
	
	.. image:: images/as/screenshot-1344673807.png
		:width: 640px
		:height: 372px
		:align: center
	.. image:: images/as/screenshot-1344674049.png
		:width: 640px
		:height: 372px
		:align: center
	.. image:: images/as/screenshot-1344674128.png
		:width: 640px
		:height: 372px
		:align: center


.. blogpost::
	:title: starting work on mean shift segmentation
	:author: bbferka
	:date: 8-01-2012

	I have started working on implementing mean shift. I am going to try to keep the API as modular as possible, thinking of the fact that mean shift can be used for a variety of things among which segmentation is only one. First I am just writing a script like program to check if the algorithm works, and then I'll implement it in the API. (segmentation will be based on color space and on normals at a first try)


.. blogpost::
	:title: reading code on MS
	:author: bbferka
	:date: 8-04-2012
	
	Tomorrow I am going on vacation for two weeks, I just hope I will be able to take the time to finish the API for MeanShift and get it up and running until the deadline. In my spare time I am checking out some other existing implementations of MeanShift, namely the `Edison library <http://coewww.rutgers.edu/riul/research/code/EDISON/index.html>`_ and the one existing in OpenCV as well as some existing Matlab implementations. I've also wrote a first script doing segmentation based on color, but results were not exactly what I was hoping for.
