Gioia Ballin
============

:email: gioia.ballin@gmail.com	
:project: Additional functionalities and improvement for PCL modules
:mentor: Federico Tombari and Alexandru-Eugen Ichim

About me
--------
I am a recently graduated student from University of Padua, Italy, with a Master's degree in Computer Engineering. My research interests include computer vision and robotics, with particular attention to tracking and recognition tasks and I am looking for an appealing PhD position on these subjects. From the spare time side, my hobbies are doing sports and travelling. 

Project Description
-------------------
My project aims to improve the existing tools in PCL, while adding at the same time new functionalities at the state of the art of the research. The focus is on the following modules: 

* **pcl_keypoints**
* **pcl_recognition**


Roadmap
-------
In what follows, I show an outline of my GSoC roadmap. The boldface writings represent my current activities.

* Module keypoints:

  * First approach to the object recognition research field: documentation on techniques at the state of the art.
  * Test functionalities for the new 3D object recognition tutorial and the existing detectors in PCL.
  * Implement a framework for simple keypoint detection evaluation.
  * Extend the framework in order to perform tests on complete datasets.
  * Porting of the ISS detector in PCL.
  * Test the performances of the ISS detector.
.. * **Porting of the 3DGSS detector in PCL.** 

* Tools:
  
  * Develop a new point type to handle monochrome images.
  * Develop a PNG to PCD converter.


Click :ref:`here <gioia_roadmap>` for a detailed roadmap about my GSoC activities.


How to's
--------
* :ref:`how to compute the keypoint repeatability <gioia_repeatability>`
* :ref:`how to use the ISS 3D keypoint detector <gioia_iss>`


Detectors evaluation
--------------------
* See :ref:`Detectors evaluation: repeatability and time performances <gioia_tests>`.


Recent status updates
---------------------

.. blogbody::
  :author: gioia
  :nr_posts: 10



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:
 
  tests	
  iss
  repeatability	
  roadmap	
  status
