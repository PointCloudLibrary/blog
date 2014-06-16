My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Markus


.. blogpost::
  :title: First post - LCCP algorithm implementation
  :author: Markus
  :date: 16-06-2014

  Hello everybody, this is my first blog post to this project. The last weeks I have been busy implementing the LCCP algorithm in PCL. The algorithm can be used to split a point cloud into regions which are isolated by concave boundaries to all other regions. It turns out that this is a highly valuable segmentation as it often retrieves (bottom-up!) nameable parts like handles, heads and so on. Especially robotic applications may find this useful. Together with Jeremie I will introduce it at this year's CVPR 2014 conference (S. C. Stein, M. Schoeler, J. Papon, F. Woergoetter: Object Partitioning using Local Convexity). We will have a poster next Tuesday afternoon. So if you are around, you are more than welcome to visit us there. In the meantime I hope I get the pull request through, so that everybody interested can play around with the algorithm. It will be located in the segmentation module. There is also an example pcl\_example\_lccp\_segmentation. To get you interested the following image shows an example of the segmentation. As you can see all parts can be easily named.
  
  .. image:: images/lccp_example.png
    :align: center
 
  That's it for the first post. Hope to see some of you at CVPR. Stay tuned for more to come. 
 