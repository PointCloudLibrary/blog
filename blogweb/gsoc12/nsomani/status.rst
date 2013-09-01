My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: somani

.. blogpost::
	:title: First few weeks in GSoC
	:author: nsomani
	:date: 6-11-2012

	I've been working on the project "Organized Point Cloud Data Operations". I'm adding some functionality for 2D images in PCL. These functions can be used on the RGB information or any single-channel information contained in organized point clouds. I use a datatype vector< vector< float> > to represent images. This choice makes it easy to integrate it with the PCL datatypes. Also, because it uses such simple data-types and does not have any PCL-specific dependencies right now, one can just pluck this 2D library from PCL and use it anywhere. 

	I found that there were some image write functions in the pcl_io module, but no read functions! So, I created some simple image read/write functions using VTK, which I will later place in pcl_io. 

	Till now, I've implemented things like convolution, edge detection and morphological operations. I've compared the outputs to that obtained from OpenCV implenmentations. I used some synthetic images to check for corner cases. Everything seems to be working properly! I'm yet to write gTests to do a pixel-to-pixel level comparison. 
	
	A lot of energy actually went into adapting to the PCL style of coding, writing code comments and documenting my work. Initially I used uncrustify to check for styling errors before committing. Then I found a PCL styling file for Eclipse which made life much easier. Still adapting to the "PCL style" of coding which is quite different to my default style. I guess it will take some time :) 

	Will be adding a lot more functionality to this module in the coming few weeks. Hope to have some pretty pictures to add in the next blog entry.

.. blogpost::
	:title: Results from implementations
	:author: nsomani
	:date: 6-20-2012
	
	There was a PCL tutorial at CVPR recently, where some of the upcoming features in PCL were also presented.
	This included some of the work that I have been doing. Following are some of the results that I obtained : 
	
	Edge Detection : 
	
	.. image:: images/edge.jpg
			:width: 894px
			:height: 596px
			:align: center
	 
	Morphological Operations : 
	
	.. image:: images/morphology.jpg
			:width: 830px
			:height: 559px
			:align: center
	
	Harris Corners : 
	
	.. image:: images/harris.jpg
			:width: 946px
			:height: 487px
			:align: center
	 
	 
.. blogpost::
	:title: Changes to API
	:author: nsomani
	:date: 7-3-2012
        
	Based on some discussions on how people would want to use the 2D module, there have been some changes in the API for this module. Images are now being represented in the point cloud format itself. 2D filters will extend the pcl::Filter inteface and 2D keypoints will extend the pcl::Keypoint interface. This will lead to a code structure more consistent with the rest of PCL. 

	This module does operations only on the RGB/Intensity channels of Organized Point Clouds. It ignores all x,y,z information for now. There are lots of features in PCL which deal with the x,y,z data. Now that the 2D module works with the same data-types, the user could use these existing features for processing the x,y,z information and the 2D module to process the RGB/Intensity information.

	I've been focusing on designing and implementing this new API in the past few days. I'm also converting the code I wrote earlier to comply with this new API.

