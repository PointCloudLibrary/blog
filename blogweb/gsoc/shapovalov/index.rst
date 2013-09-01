Roman Shapovalov
================

This is my personal page

:email: shapovalov@graphics.cs.msu.ru
:project: Geometric object recognition
:mentor: **Aitor Aldoma** (Radu B. Rusu [Zoltan-Csaba Marton, Vincent Rabaud, Nico Blodow, Michael Dixon])

About me
--------

I'm a student at `Moscow State University <http://msu.ru>`_.  

My hobbies include urban orienteereing, snowboarding and editing Wikipedia. See also `my homepage <http://shapovalov.ro/>`_.


Project summary / Roadmap
-------------------------

In this project we would like to implement new object descriptors and test 
them together with the already existent descriptors. Moreover, we are interested in the pose of the objects on the scene 
and ideally we would like to handle several level of occlusion.

We would like to focus on geometric characteristics of the surfaces/objects but
we can decide to incorporate other kind of information like color,etc.

For benchmarking, we might use different sources for training like CAD models
and training data obtained from real sensors like the Kinect. We might use 
existing databases like the one presented in the Perception Challenge from 
Willow Garage or create new data sets that better fit our specific needs.

Finally, one very important piece missing into PCL consists of a generic
descriptor/feature matcher that can be used to exchange the type of descriptor
being used for optimal benchmarking.

Generic Trainer / Matcher
-------------------------

Roadmap
-------

Here is a brief outline of my GSoC project milestones:

* Fill out this page
* Decide the appropiate benchmark data set for object reconition and pose.

  - CAD models.
  - Data obtained from the Kinect.

* Implement generic trainer

  - Given training data in a certain format, it should need to load the data and compute the specific descriptor on the given data.  Finally, make all the needed information persistent for the matcher.
  - Documentation.
  - Some descriptors might need specific subclasses to fulfill its specific needs.

* Implement generic matcher

  - Similar to the generic trainer but for matching purposes.
  - Needs to be able to interface with the data generated on training.

* Benchmarking of existing descriptors.
  
  - Eventually create a new training set.
  - Evaluate accuracy in object recognition and pose, time performance, etc.

* Implement new descriptors. 

Click :ref:`here <shapovalov_roadmap>` for a more detailed roadmap


Recent status updates
---------------------

.. blogbody::
   :author: shapovalov
   :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
   :hidden:

   roadmap
   status
