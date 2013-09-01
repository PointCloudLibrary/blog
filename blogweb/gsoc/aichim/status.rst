My status updates
=================

.. blogbody::
   :nr_days: 30
   :author: aichim
                            

.. blogpost::
	:title: My very first status update
	:author: aichim
	:date: 5-29-2011
	                               
	I finally convinced myself to write something on my GSoC page. I actually started playing with PCL before the 1.0 release and before the GSoC coding period started (because I will be offline during the next week as I am graduating => all the fun stuff of moving out, tons of documents (love Germans for that) etc. - will keep me overly busy).

	Now for the techy bits. I am using a Mac - with an Ubuntu VM just in case. After some very helpful discussions on the IRC channel and the bug tracker, I realized that the compiler bundled with Snow Leopard is outdated so I installed the MacPorts gcc4.5. After hitting another ton of problems (mostly related to Apple anti-Unix stupidity) I managed to get all the 3rd party libraries up and running and PCL now happily compiles and all of the examples are running. I saw that there are other GSoCers using Macs and switched to Ubuntu. If there are requests I could write a more comprehensive tutorial on how to install PCL on OS X. Just for fun, you can find a picture of pcd_viewer running on OS X on my main page. For this, I made a small application that takes an image and creates a beveled point cloud out of it using OpenCV and PCL (I have to add some code too :-) ):
	
	.. code-block:: cpp    
		
		#include <cv.h>
		#include <highgui.h>
		#include <iostream> 

		#include "pcl/io/pcd_io.h"
		#include "pcl/point_types.h"                                     

		using namespace cv;
		using namespace std;

		int main(int argc, char **argv)
		{
		  if (argc != 3) {
		    cerr << "Syntax: ./sampleImage2PCL image_file_name output_pcd_file" << endl;
		  }        

		  Mat image = imread(argv[1]);
		  pcl::PointCloud<pcl::PointXYZ> cloud;   

		  for(size_t i = 0; i < image.rows; ++i) {
		    for(size_t j = 0; j < image.cols; ++j) {
		      if (image.at <char> (i, j, 0) > 10) {
		  	  for(float z = 0; z < 0.04; z += 0.001) 
		  	    cloud.points.push_back(pcl::PointXYZ( ((float)j) / image.rows, ((float)i) /image.cols, z));
		  	}
		    }
		  }
		
		  pcl::io::savePCDFileASCII (argv[2], cloud);	

		  return 0;
		}

	I also started looking into the more advanced tutorials and had fun playing with parameters and experimenting a bit. Furthermore, I already have a small collection of publications about 3D features I will read and analyze. I will start implementing the 'Model Globally, Match Locally: Efficient and Robust 3D Object Recognition' within the next days.
	
	I mentioned earlier about the fact that I added a picture on my main page. There seems to be a bug with the blogpost and blogbody macros - cannot add images/figures inside a blogpost.
	
	#### Update: now images work 
	
	.. image:: images/gsoc_pcd.png
		:width: 500
  
   


.. blogpost::
	:title: My Second Week @GSOC
	:author: aichim
	:date: 6-12-2011  
	
	A lot of things have happened in the last week. I officially finished my undergraduate studies and I now am a Bachelor of Science in Electrical Engineering and Computer Science :-). It was a really crazy week, but I managed to fit in some hours of coding and reading. Yesterday was my first full day of GSoC.
	
	As I said in my previous blog post, I started implementing the 'Model Globally, Match Locally: Efficient and Robust 3D Object Recognition' and a couple of hours ago I had the first results on some datasets. There is quite a lot of work to be done from the structural point of view, as everything has to be smoothly blended in the PCL framework. The picture below shows my result compared to the FPFH features (using the application from Aligning object templates to a point cloud tutorial <http://pointclouds.org/documentation/tutorials/template_alignment.php#template-alignment>):
	
	.. image:: images/surflet_first_results.png
		:width: 300
	
	I am now reading Radu's PhD thesis - an exceptional document I truly recommend for any object recognition enthusiast or any computer vision person in general. You can find it here: <http://files.rbrusu.com/publications/RusuPhDThesis.pdf>       
	
	
.. blogpost::
	:title: GSoC Updates
	:author: aichim
	:date: 6-15-2011
	
	Some more updates about my latest work. I fully implemented the 'Model Globally, Match Locally: Efficient and Robust 3D Object Recognition' paper. The results are as presented by the authors (Please see the figure for a dataset used by the authors in the paper). I also added a small optimization that will theoretically speed up the object recognition in larger scenes (will do some proper benchmarks next).
	
  	.. image:: images/chicken_in_clutter.png
		:width: 700
		
	An interesting problem I faced during this time was to find a proper solution to average rotations, as my algorithm clusters the many results it gets from an internal Hough Transform and outputs the average poses on each such cluster. It seems that the naive solution of directly averaging quaternions in vector space and then normalizing them is accepted (you can find the proof in the publication: "On Averaging Rotations" by Claus Gramkow).
	
	As the next steps, I am intending to test the algorithm on noisier data such as the one from the Kinect. In this process, a repository of pcd files for such tasks will be created and will be made available to the Pointclouds.org community.     
	
	
.. blogpost::
	:title: Updates about my progress
	:author: aichim
	:date: 6-29-2011
	               
	A lot has happened since my last update. I have read a lot about applying the multiscale concept in 3D feature representation, as I found this field was not exploited enough - and it is very successful when applied to 2D images.
	
	As such, I implemented the multiscale feature persistence algorithm proposed by Radu et al. in "Persistent Point Feature Histograms for 3D Point Clouds". Also, I tried the rather slow method introduced by Ranjith Unnikrishnan in his PhD thesis. I am still at the stage of tuning the algorithms, but they seem to work as expected so far. In this process, I ran into software engineering problems again - having to decide on the architecture of new abstract classes in the PCL framework - for this I asked the help of the other developers and will decide on the best solution after some thorough discussions.                            
	
	Another interesting problem I faced was that of geodesic distances inside the point clouds. There are quite a few methods to solve this problem - most of them refer to creating a graph representation of the point cloud using different heuristics. I am not yet convinced about the ideal version for this, so I will be looking into it in the next days.
	
	Furthermore, I contacted the author of an algorithm I implemented a few weeks ago - Slobodan Ilic to ask for some hints about his own experiments using noisy data. After following some of his advice, I managed to tune the parameters so that it works satisfactorily on Kinect datasets such as my dad's garage :-) - where it detects a chair and a watering can:
	
	.. image:: images/ppf_registration.png
		:width: 700     
		
  	I forgot to mention that I received my very own Kinect and spent some time playing with it (under Mac OS X - and it works after a looot of tweaks and hacks!!!). I will be collecting all the datasets I am using for my own experiments around the house and will upload them in a structured way to the PCL dataset repository: svn+ssh://svn@svn.pointclouds.org/data


.. blogpost::
	:title: Surfel Surface Smoothing and Pyramid Feature Matching
	:author: aichim
	:date: 7-05-2011
	
	A few days have passed and it's time for a new blog post :-).     
	
	To begin with, I added a useful new algorithm to the PCL library: Surfel smoothing based on the work of:
	
	- Xinju Li and Igor Guskov - "Multiscale features for approximate alignment of point-based surfaces"
	
	It is an interesting iterative modification of the Gaussian mesh smoothing. Right now only the smoothing part works, the salient point extraction seems to need more testing, as I am not totally convinced about the results. Nevertheless, you can see a picture below of the effect:
	
	.. image:: images/surfel_smoothing.png
		:width: 500
	
	Another interesting result I have is the use of:
	
	- Kristen Grauman and Trevor Darrell - "The Pyramid Match Kernel: Discriminative Classification with Sets of Image Features"
	
	I have implemented a generic class for comparing surfaces represented by unordered sets of features. The class then returns a value in [0, 1] representing the similarity between the two surfaces. To test it, I tried the very naive, raw representation of PPF features (i.e., pair features between all the pairs in the cloud) and the results are rather interesting: 
	  
	- my face with two different expressions: 0.867585
	- my face compared to a watering can: 0.586149
	- watering can compared to a chair: 0.399959           
	- obviously, any object compared to itself: 1.0
	
	.. image:: images/pyramid_surface_matching.png
		:width: 800 	   
		
.. blogpost::
	:title: Interest Region Extraction and other things
	:author: aichim
	:date: 7-23-2011
	                                                                    
	I have not updated my blog for a substantial period because I have not done any cool things until the last few days. I have spent most of my time enhancing the code I have written so far during this summer. All the 'boring' bits: documentation, unit testing, small bug fixes and optimizations here and there have been the things eating up my time.
	
	As for the more interesting subjects: I have created a new architecture, as agreed upon with other PCL developers, for the MultiscaleFeaturePersistence. It can easily be used with any type of feature extractor/descriptor and I am attaching an example of the results using FPFH, as it was done in the paper describing the method:
	
	- Radu Bogdan Rusu, Zoltan Csaba Marton, Nico Blodow, and Michael Beetz - "Persistent Point Feature Histograms for 3D Point Clouds", Proceedings of the 10th International Conference on Intelligent Autonomous Systems (IAS-10), 2008, Baden-Baden, Germany. 
   
	In a few words, this algorithm should extract only the features that have signatures distinguishable from the mass of features found in the respective scene, such as corners in an office scene as the one in the image. (Please note that the color image is not calibrated with the depth image with my Kinect under OS X [did not find a way to calibrate it properly])

	.. image:: images/office_multiscale_feature_persistence_fpfh.png
		:width: 500 
		
	.. image:: images/table_scene_multiscale_feature_persistence.png
		:width: 500
	
	Another thing worth mentioning is the StatisticalMultiscaleInterestRegionExtraction, inspired by the work Ranjith Unnikrishnan has done for his PhD thesis. I have polished and tested the algorithm and it is now in working state. In a previous blog post I said some things about the fact that I need geodesic distances between all point pairs inside the input point cloud and also geodesic radius searches in order to do the statistical computations. It seems that the method I went for initially is considered one of the efficient ones, which is building a graph where edges are created between each point and its K nearest neighbors (I chose 16 as a decent number - the results made sense). Then, I create an NxN distance matrix, as the output of the Johnson's search algorithm for sparse graphs. For radius searches, I simply go through a whole row of distances in the matrix and extract only the ones below my threshold (An O(N) that can be further improved to O(logN) by using binary trees at the expense of some additional loops in the initialization). The result of this, as applied on the Stanford dragon:                                        
	
	.. image:: images/dragon_statistical_multiscale_interest_regions.png
		:width: 500
	
	
	
.. blogpost::
	:title: Work updates - a lot of different topics
	:author: aichim
	:date: 8-18-2011
	
	I started working on a lot of different things lately. A short description of every little branch will be given here, more details when I reach cool results.
	
	1. In a previous post, I mentioned the Surfel smoothing algorithm I implemented. A lot of modifications have been made to it, to make it closer to the original paper by Amenta et al. After this, benchmarking it against the Moving Least Squares reconstruction algorithm was necessary. The immediate conclusions are that MLS performs faster and with better reconstruction results and I am still working on understanding why Amenta 's smoothing method would be theoretically much more precise. Also, in this section, we are looking into adapting a smoothing algorithm with real-time performance for the Kinect.
		
	2. Next, I looked into trying to combine a few of the things I wrote this summer for a novel application for PCL. So, I decided to try hand pose/gesture recognition. I have successfully used the pyramid feature matching I talked about previously to get some rough initial classification results. I am now trying to use Support Vector Machines in combination with pyramid matching and different kinds of features in order to train classifiers to recognize hand poses recorded with the kinect.	For those interested, I have recorded a rather extensive dataset with 8 different hand postures with 10 different images per class. They are uploaded in the dataset repository of PCL.
		
		.. image:: images/hand_poses_dataset.png
			:width: 500
		
	3. SmoothedSurfacesKeypoint is up and running. Just need to add unit tests. What this algorithm does is extract features from a set of smoothed versions of the input cloud by looking at the "height" differences between the points smoothed with different scales.
	
	4. As for bulky programming tasks, I proposed the pcl_tools idea on the mailing list and contributed with a few tools such as: add_gaussian_noise, compute_cloud_error, mls_smoothing, passthrough_filter. Also, two new apps for the kinect have come to life: openni_mls_smoothing and openni_feature_persistence - which can be found in the apps/ folder.
	
	5. Not all of them are currently commited to trunk, but I have been working on creating new features by integrating color information into 3D feature signatures. So, today I started using Pararth's feature benchmarking framework to compare the performance of each feature signature.
	
	6. Last, but not least, I contacted the Willow Garage people working with Android and trying to help in that direction too.
	
	Over and out!
	
	
.. blogpost::
	:title: PFHRGB Feature results!
	:author: aichim
	:date: 8-24-2011
	
	Cool results using the PFHRGB feature. It is based on the PFH feature which can be found in the following publication:
	
	- R.B. Rusu, N. Blodow, Z.C. Marton, M. Beetz. - "Aligning Point Cloud Views using Persistent Feature Histograms", In Proceedings of the 21st IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Nice, France, September 22-26 2008.
	
	The implementation of this feature in PCL currently uses a histogram of 125 bins. The PFHRGB feature we introduced uses an additional 125 bins for storing color information around the point of interest. By using Pararth's feature evaluation framework, we concluded from a simple experiment that PFHRGB are more distinguishable than simple PFH features.
	
	The experiment was carried out at 3 different feature search radii. A single cloud from the Kinect was used - the original input as source and a transformed version as target. After subsampling, feature signatures were calculated for each point. Correspondences are made by using a simple nearest neighbor heuristic in feature space. The results are the following:
	
	- PFH vs PFHRGB at search_radius 0.02:    4% vs 24% repeatability
	- PFH vs PFHRGB at search_radius 0.05:    37% vs 85% repeatability
	-	PFH vs PFHRGB at search_radius 0.06:    46% vs 92% repeatability
	
	To apply the newly implemented feature to a real world situation, I recorded 6 frames of my desk and incrementally aligned them by using PFHRGB features and RANSAC. The whole procedure took about 5 minutes for registering 5 pairs of clouds.
	
	.. image:: images/pfhrgb_registration_clouds.png
		:width: 600
		
	.. image:: images/pfhrgb_registration_color.png
		:width: 600