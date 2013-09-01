Marco A.  Gutierrez
===================

:email: marcog@unex.es
:website: http://robolab.unex.es/marcog
:project: DINAST pcl Camera grabber interface and realtime multiple sensors point cloud map generation.
:mentors: **Dugan Um** and **Radu B. Rusu**


About me
--------
I am a PhD Student at the Robotics and Artificial Vision Laboratory (`Robolab <http://robolab.unex.es/>`_), from the University of Extremadura, Spain. I currently do extensive work on the development of robocomp, an open source, component oriented robotics framework. Most of the research I have done involves computer vision, although some of my work has also got some software engineering. Currently I am doing an internship at KUKA laboratories, where I am developing a solution on object recognition for a pick and place application. 

Problem Description
-------------------

The idea is to get DINAST cameras integrated in pcl through a pcl::DinastGrabber interface and to develop a realtime point cloud map generation with multiple Cyclope sensors. A collision shield will be develop around a robot manipulator using several sensors and tested with a Rapidly exploring random tree (RRT) algorithm. A soon as a simple RRT is tested we will move to other more interesting RRT solutions.

Roadmap
-------

My current work is beeing done on the boldfaced bullets .

* Develop an interface for Dinast Cameras for pcl.
* Realtime multiple sensors point cloud map generation.
* Built a collision shield around a manipulator type robot using 2 Cyclope sensors.
* RRT path planning in a generated map for collision free motion.
* Test collision field and realtime rehearsal with RRT.
* **Multiple point cloud integration**
* Implement and test BNM algorithm

Recent status updates
---------------------

.. blogbody::
   :author: marco
   :nr_posts: 35



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
   :hidden:

   status
