My status updates
=================

.. blogbody::
	:nr_days: 60
	:author: pkalogiros


.. blogpost::
	:title: PCL Datasets
	:author: pkalogiros
	:date: 30-05-2012

	I finally convinced myself I had something to share and say. During the first week of the coding period I decided to tackle issue `#682 <http://	dev.pointclouds.org/issues/682>`_,.
	The current datasets when accessed online are a list (xml) generated directly though the svn. While it works and is very fast it is a bit simplistic and improvements can be made, for example utilizing the WebGL PCD viewer, auto-loading the readme files etc.
	  
	Long story short, 
  
  			.. image:: images/1/pcl_1.png
    				:width: 940px
				:height: 505px
    				:align: center


	A lot of features have been added so far, including, binary compressed PCD support for the WebGL Viewer, search/filtering results, mirroring controls between pointclouds, loading the readme file instantly, drag n drop PCD files from your desktop etc - among others.
	This mini project will hopefully be online soon, and a detailed post with instructions and included features will be made then. 
	You can preview it if you go to issue #682 and feedback would be greatly appreciated.
       
	 
	Apart from this, I have installed PCL, ran a couple of examples, and tested image encoding (jpeg/png) of depth/rgb data. Lossy image compression algorithms (jpeg) are optimized for images and not depth so unfortunately while the compression rate is phenomenal a lot of unwanted noise is added. Entropy encoding does not suffer from this of course.
	The advantage of using images and videos lies in the browser, see browsers support images and video natively (the internet would be very ascii-art-like if they didn't) so no overhead is added in the Javascript for decoding, plus it is quite trivial to import images in the GLSL shaders for very fast analysis. I am still performing tests to find an optimal solution.
	  
		
	Fine reads,
	   
	Julius Kammerl `"Development and Evaluation of Point Cloud Compression for the Point Cloud Library"	<http://	pointclouds.org/assets/files/presentations/kammerl_intern_presentation.pdf>`_ Institute for Media Technology, TUM, Germany - May 2011,
	   
	Fabrizio Pece, Jan Kautz, Tim Weyrich `"Adapting Standard Video Codecs for Depth Streaming" <http://	web4.cs.ucl.ac.uk/staff/j.kautz/publications/depth-streaming.pdf>`_ Department of Computer Science, University College London, UK, Joint Virtual Reality Conference of EuroVR - EGVE (2011)
  