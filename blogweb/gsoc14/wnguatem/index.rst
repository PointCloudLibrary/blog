William Nguatem 
==========

:email: william.nguatem@googlemail.com	
:project: A Bayesian Approach for fitting Geometric Models in 3D Point Clouds
:mentor: Zoltan-Csaba Marton

About me
--------
I am a graduate student at the Bundeswehr University Munich.

Introduction
-------------
PCL provides a great frame work for fitting geometric primitives e.g Planes, Cylinders, e.t.c. Most of these algorithms are based on the RANSAC paradigm.
In this work however, we aim at solving the model fitting tasks using a Bayesian approach. Amongst others, this approach has the advantage of being very flexible and
generic i.e. both parametric and non-parametric (e.g. curves, surfaces using splines) models can be fitted.

Brainstorming
=============
This section contains some of the "Bits and Bytes" from the discussions I've had with my mentor Zoltan

Meeting 1-2
-----------

- Study the existing SampleConsensus and Tracking Module.
- What can be taken from these Modules ?
- The Pros and Cons of inheriting from these modules rather than re-inventing the wheel ? 
- Extending the posible model types for say Torrus/ellipsoids
- B-Spline curve fitting in 3D
- Knot/Control point placement problematic
- Lets try first an MCMC Sampling approach for already existing model types (Planes, Lines,Cylinders ...)
- B-Spline surface fitting

Meeting 3
---------
- Consider the case when data is sequentially comming in, i.e tracking.
- How to initialize a tracker ?
- How to make a random proposal to a direction vector ? Quarterions maybe helpful ?

Meeting 4
---------

Recent status updates
---------------------

.. blogbody::
  :author: William Nguatem
  :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:

  status
