.. blogpost::
   :title: Spectrolab PCL Code Sprint :  First post
   :author: adasta
   :date: 05-5-2013

   
	This is the first post of the Spectrolab PCL Code Sprint.  `Spectrolab <http://www.spectrolab.com/>`_ is 
	a Boeing company that has developed a `new LIDAR camera <http://www.spectrolab.com/sensors/pdfs/products/SPECTROSCAN3D_RevA%20071912.pdf>`_. 
	This camera is meant for industrial and robotics uses in both outdoor 
	and indoor environments.  It uses a scanning time of flight laser to 
	generate a 256x128 range image at 5-6 hertz.  It is still in development right now,
	but it promises to be a valuable tool for robotics and perception work.
	Spectrolab has sponsored integration of the camera into PCL so that when
	it hits the market, it can take advantage of the wealth of tools and knowledge
	within the PCL community.  

	As apart of the code sprint, I will be developing a new PCL grabber
	driver for the Spectrolab camera and a 3D viewer interface.  
	As I complete the driver and software, I will post screen shots and code
	explanations to this blog.  
	
 	.. image:: img/spectroscan3d.jpg
		:scale: 100 %
		:alt: spectroscan3d
		:align: center


.. blogpost::
   :title: Spectroscan3D Driver Implemented
   :author: adasta
   :date: 05-24-2013

	The first milestone of the PCL Spectrolab Code Sprint has been completed.
	I received the Spectroscan 3D two weeks ago and I have used it to write a
	driver for the camera.  In addition, I have created a simple 3D viewer
	for the camera which streams the data to a PCLVisualizer.
	All of the code for the driver  (and the rest of the code sprint) can be found
	`here <https://github.com/adasta/spectrolab>`_. 
	There are no Spectroscan3D units commericially available right now, 
	but by the time I am done there will be a good set of open source tools
	to visualize and manipulate the data.
	
	Here is an example scan from the camera.
	
	.. figure::   img/test_scan_3d.png
		:align: center
 		
		An amplitude colorized view of an interior door. 
		
		
	.. figure:: img/test_scan_amplitude_img.png
		:align: center
 		
		An amplitude image of the interior door scan. 

	The Spectroscan3D driver is split into two components. The first component encapsulates
	the communication with the lidar camera. It speaks the lidar's  UDP  
	protocol and handles receiving the raw range/amplitude images from the scanner. 
	This component is meant to be independent of PCL. This way
	future developers can use it with other software systems as well.
	The second layer wraps the driver in pcl grabber interface and transforms
	the range image into a point cloud. You can register for either PointXYZ
	or PointXYZI point cloud events from the scanner.

	The next stage of the code sprint will be writing a visualizer capable of 
	controlling the camera like a video camera.  It will be able to watch a live
	stream, record the stream, or play back an old stream from disk. Stay tuned for more updates!

.. blogpost::
   :title: Spectrolab Viewer
   :author: adasta
   :date: 07-24-2013

    The Spectrolab Viewer has been completed.  The viewer streams data from a Spectroscan 3D, records point clouds, and plays the point clouds back.  Additionally, supports multiple renderers.  It can color points based on the x,y,z coordinates, as black and white intensity coloring, and as a fused Z range and intensity coloring.  This provides maximal visibility for all of the structures in the point cloud.  An example of a Spectroscan 3D point cloud and the fused Z/intensity rendering can be seen in the below image and movie.  

	.. figure:: img/hanger_pcd.png
		:align: center
		:width: 640
	
    .. raw :: html
    
	<div align="center"><iframe width="420" height="315" src="http://www.youtube.com/embed/Jd3BbZGP0ds" frameborder="0" allowfullscreen="">
	</iframe><div/>
	
	
    The Spectrolab Viewer can directly record PCD files which can later be viewed with the viewer or any other PCL tool.  Below is an example PCD file recorded by the Spectroscan 3D and can be shown in the PCL web viewer.  You can clearly see the vehicle and person being scanned in the point cloud. 
	
    .. raw :: html
    
     <div align="center"><iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=https://dl.dropboxusercontent.com/s/72sax6ck96fic6a/hanger_000000.pcd?token_hash=AAHK29p70PMNyX3Vkbw1coP6BHwJg78X3ToZjVTRhqy1ng&dl=1" width="770" height="480" marginwidth="0" marginheight="0" frameborder="no" allowfullscreen="" mozallowfullscreen="" webkitallowfullscreen="" style="max-width: 100%;"></iframe></div>
    
    This viewer completes the PCL Spectrolab code sprint.  This sprint has created open source tools to stream, save, and view data from Spectrolabs in development Spectroscan 3D.  When it hits the market, it will be ready to operate with all of the opensource tools PCL has to offer.   
 

