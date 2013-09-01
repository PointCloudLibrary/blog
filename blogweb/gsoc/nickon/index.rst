Nick Vanbaelen
==============

Welcome to my personal, official PCL Google Summer of Code page.
I am excited to get working for PCL and you will find all project progress and specifics on this page!


:email: nickon (at) acm.org
:project: KdTree on GPU using CUDA
:mentor: **Marius Muja** (Stéphane Magnenat [Radu B. Rusu, Michael Dixon])

About me
--------

I'm a masters student at `Catholic University of Leuven (KULeuven) <http://www.kuleuven.be/>`_ and my field of study is `Civil Engineering: Computer science` with specialization `Artificial Intelligence`. My interests in computer science ranges from theoretical aspects to software design. In particular I'm interested in distributed and parallel computing and the optimisation of software for the underlying hardware platform.

Besides computer science, my hobby is carp fishing. I've been doing this since I was a kid and it is grown to be a passion of mine. It's a great way to relax and don't think about anything else than trying to outsmart the one giant carp which is wandering around in the lake.


Roadmap
-------

The main goal of my project is to firstly benchmark different NN libraries such as nnn, libnabo and FLANN (currently being used in PCL) and integrating the best parts of each into FLANN. When this integration is done I will be exploring possibilities to optimize FLANN for multi-core architectures, exploiting the maximum amount of parallelization possible for each underlying hardware platform. I will be working closely together with Marius Muja, which is the mentor for this project and also the developer of the FLANN library.

In the meantime I will also be working together with my GSoC colleague Andreas, which will be starting the development of a NN search on the GPU. The aim is that I will be involved in the decision making of the design and integration into FLANN of this NN GPU implementation, such that towards the end of the GSoC period (or if the timespan won't allow it, outside of the GSoC period) I can aid in developing this GPU implementation.

You can find the more detailed roadmap of my GSoC project by clicking :ref:`here <nickon_roadmap>`.


Recent status updates
---------------------

.. blogbody::
   :author: nickon
   :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
   :hidden:

   roadmap
   status
