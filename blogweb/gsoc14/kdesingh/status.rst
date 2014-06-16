My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: kdesingh


.. blogpost::
  :title: Richtsfeld's code and results (First post)
  :author: kdesingh
  :date: 16-06-2014

        * **Introduction**

                Goal of this project is to integrate cluttered scene segmentation methods into pcl::segmentation. 
                This sprint involves implementation of modules from two publications below. 
        

                - Z.-C. Marton, F. Balint-Benczedi, O. M. Mozos, N. Blodow, A. Kanezaki, L. C. Goron, D. Pangercic, and M. Beetz: Part-based geometric categorization and object reconstruction in cluttered table-top scenes; Journal of Intelligent and Robotic Systems, January 2014

                - A.-Richtsfeld, T. Mörwald, J. Prankl, M. Zillich and M. Vincze: Learning of Perceptual Grouping for Object Segmentation on RGB-D Data; Journal of Visual Communication and Image Representation (JVCI), Special Issue on Visual Understanding and Applications with RGB-D Cameras, July 2013
        
                These papers have made their code available and has pcl implementation to some extent already. But we will aim to make the modules interoperable in our implementation.

        * **Richtsfeld's code and results**
         
                As a first step, Richtsfeld's code was analyzed. Below is the highlevel picture of the structure of his code base along with comments on their functionality.
        
                .. image:: images/v4r.png
                        :width: 1400px
                        :height: 1100px
                        :align: center

                His code was tested on some scenes that were grabbed by me using Kinect. Below are the snapshots of the same.
        
                .. image:: images/scene_01.png
                        :width: 650px
                        :height: 250px
                        :align: center

                .. image:: images/scene_02.png
                        :width: 650px
                        :height: 250px
                        :align: center

                .. image:: images/scene_03.png
                        :width: 650px
                        :height: 250px
                        :align: center

                His code works on organized pointcloud of type PointXYZRGB.

                Code is yet to be tested for quantitative results with our annotated dataset. 

        * **Brainstorming**
                
                These are some of the discussions I had with my mentor Zoltan and the summary is listed below.
                        
				- Zoltan's work classifies the relations based on features computed on group of segments as 1-8 elements in a group.
				- Richtsfeld's work classifies the relations based on segment pairs.
				- Richtsfeld's work is limited to organized pointcloud and cannot handle a cloud that is fused out of many pointclouds of a scene say through registration.
				- Richtsfeld's work computes features which are inspired from Gestalt's principles. There are someother features that are worth testing. These are the features used for structure discovery in a pointcloud data. Features are as below and more details on them are available in the publication - Collet, Alvaro, Siddhartha S. Srinivasa, and Martial Hebert. "Structure discovery in multi-modal data: a region-based approach." Robotics and Automation (ICRA), 2011 IEEE International Conference on. IEEE, 2011.
				- Additional features to test are:

						- Shape Model
						- Self Continuity
						- Contour compactness
						- Pair continuity
						- Verticality
						- Concavity
						- Projection
						- Alignment
						- Surface compatibility 
        

                
