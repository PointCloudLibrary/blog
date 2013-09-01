Alexander Velizhev and Sergey Ushakov
=====================================




:email: mine_all_mine@bk.ru 
:project: Automated feature extraction
:mentor: Alexander Velizhev

About me
--------

My name is Sergey and this page is about me, my mentor Alexander and the project on which I intend to work.
I'm a student at the `Lomonosov Moscow State University
<http://graphics.cs.msu.ru/en>`_. I'm engaged in object segmentation in 3D point clouds. In addition, I'm fond of casual game development and all that is relevant to computer graphics.
I was fortunate to become a participant of the TRCS, within which the growing region algorithm will be embeded. I am also going to implement the object segmentation algorithm based on graphcut(for more details see below).
During my work, I will cover here every stage of development and share with you my success, failure and experience.



Roadmap
-------


First of all I'm going to implement:

* a standard well-known region growing algorithm
* it's extension for smoothed surfaces(as described in `"Segmentation of point clouds using smoothness constraint" <http://www.isprs.org/proceedings/XXXVI/part5/paper/RABB_639.pdf>`_)

Then I'm going to implement a graphcut based object segmentation(as described in `"Min-Cut Based Segmentation of Point Clouds" <http://gfx.cs.princeton.edu/pubs/Golovinskiy_2009_MBS/index.php>`_). In short the idea is following:

* to construct a graph from points
* to segment object/background points for a given possible rough object center by energy minimization



Recent status updates
---------------------

.. blogbody::
  :author: Sergey
  :nr_posts: 5



.. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. 

.. toctree::
  :hidden:

  status
