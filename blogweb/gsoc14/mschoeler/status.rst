My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Markus

.. blogpost::
  :title: Implementation of the shape generator
  :author: Markus
  :date: 07-08-2014
  
  The work on the lccp implementation is mainly done and the pull request is awaiting its approvel. Thanks a lot for the help of the pcl community (especially Sergey and Victor). Thanks to your comments the lccp algorithm is now in a much better shape.
  Talking about shapes: The next milestone in this GSOC project is the implementation of a shape generator which can be used to create various labeled scenes which can be used to create unit tests or benchmarks for part and/or object segmentation algorithms. I have written a big part of the code already. Now the question is: To which module of pcl should this generator go? I think about putting it into the geometry module. Any comment on this is more than welcome! The next image shows an assembled animal-like object which has been generated from simple geometric shapes (mainly geons).
  
  .. image:: images/animal.png
    :align: center 
  
.. blogpost::
  :title: First post - LCCP algorithm implementation
  :author: Markus
  :date: 06-16-2014

  Hello everybody, this is my first blog post to this project. The last weeks I have been busy implementing the LCCP algorithm in PCL. The algorithm can be used to split a point cloud into regions which are isolated by concave boundaries to all other regions. It turns out that this is a highly valuable segmentation as it often retrieves (bottom-up!) nameable parts like handles, heads and so on. Especially robotic applications may find this useful. Together with Jeremie I will introduce it at this year's CVPR 2014 conference (S. C. Stein, M. Schoeler, J. Papon, F. Woergoetter: Object Partitioning using Local Convexity). We will have a poster next Tuesday afternoon. So if you are around, you are more than welcome to visit us there. In the meantime I hope I get the pull request through, so that everybody interested can play around with the algorithm. It will be located in the segmentation module. There is also an example pcl\_example\_lccp\_segmentation. To get you interested the following image shows an example of the segmentation. As you can see all parts can be easily named.
  
  .. image:: images/lccp_example.png
    :align: center
 
  That's it for the first post. Hope to see some of you at CVPR. Stay tuned for more to come. 
 