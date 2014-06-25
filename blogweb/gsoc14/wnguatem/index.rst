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
PCL provides a greate frame work based on the ransac family of algorithms for fitting basic geometric primitives e.g Planes, Cylinders, e.t.c.
In this work however, we aim at solving the model fitting tasks using a Bayesian approach. Amongst others, this approach has the advantage of being very flexible and
generic i.e. both parametric and non-parametric (e.g. curves, surfaces using splines) models can be fitted.

Brainstorming with my Mentor
----------------------------
These are some of "Bits and Bytes" from the discussions I've had with my mentor Zoltan
-Study the existing SampleConsensus and Tracking Module.
-What can be taken from these Modules ?
-The Pros and Cons of inheriting from these modules rather than re-inventing the wheel ? 
-Extending the posible model types for say Torrus/ellipsoids
-B-Spline Fitting
-Knot/Control point placement problematic
-Lets try first an MCMC Sampling approach for already existing model types (Planes, Lines,Cylinders ...)



Recent status updates
---------------------

.. blogbody::
  :author: my_username
  :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:

  status
