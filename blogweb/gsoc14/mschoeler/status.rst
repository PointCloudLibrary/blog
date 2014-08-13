My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: Markus

.. blogpost::
  :title: Implementation of the shape generator finished
  :author: Markus
  :date: 08-13-2014
  
  I finished the work on the shape generator module. It became quite a versatile tool. Here is an overview what one can do:
  
  - Create simple shapes (Sphere, Cylinder, Cone, Wedge, Cuboid, Full Torus, Partial Torus).
  - Create any type of polygon shape (Just give it a list of vertices and edges). If you want to use the more advanced features like cutting the polygon needs to be convex.
  - The list of simple shapes can be easily extended using convex polygon.
  - Combine any number of shapes/objects into more complex objects.
  - Create cavities, holes, tunnels by cutting one shape/object by another.
  - Everything you generate will have normal information attached to it.
  - Full control over the labeling of shapes. Label each shape individually, label a group of shapes, give the full object one label .... This allows one to tailor groundtruth to what you actually want to benchmark.
  - One class for reading assembly instructions from human readable text files (recipes). You can access all features of the generator except the general polygon class. Recipes can also call/include other recipes. This makes it possible to create complex objects and reuse them in different settings. Another example would be the creation of several objects which can easily be arranged in various scenes.
  
  Here are some examples how we can create a table top scenario consisting of a table, a cup on top and a saw which cuts into the table using recipes (shown on the right side).
  First we make and compile a recipe file for a table leg:

  .. image:: images/TableLeg.png
    :align: center
    
  Next we will combine four legs plus a cylindrical plate to a full table (we set the variable Label to keep, which preserves different labels for the object's parts):
  
  .. image:: images/Table_keep.png
    :align: center
    
  If we set Label to unique it will tell the generator to give only the table parts unique labels (not the parts the parts consist of :) )
  
  .. image:: images/Table_unique.png
    :align: center
    
  This labeling would make sense if you want to test for instance part segmentation algorithms like LCCP.  
  
  Now let us combine the table and two objects. We have multiple options for the labeling now:
  First: Each object gets its own label:
  
  .. image:: images/Table+objects_unique.png
    :align: center
    
  Second: Give the parts of the objects their own label:
  
  .. image:: images/Table+objects_keep.png
    :align: center
    
  Or keep the labels of the parts and parts of parts unique :) : 
  
  .. image:: images/Table+objects_keep_keep.png
    :align: center
  
  Just to mention: All points have normal information attached:
  
  .. image:: images/Table+objects_keep_keep+normals.png
    :align: center  
 
 
 
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

  Hello everybody, this is my first blog post to this project. The last weeks I have been busy implementing the LCCP algorithm in PCL. The algorithm can be used to split a point cloud into regions which are isolated by concave boundaries to all other regions. It turns out that this is a highly valuable segmentation as it often retrieves (bottom-up!) nameable parts like handles, heads and so on. Especially robotic applications may find this useful. Together with Jeremie I will introduce it at this year's CVPR 2014 conference (S. C. Stein, M. Schoeler, J. Papon, F. Woergoetter: Object Partitioning using Local Convexity). We will have a poster next Tuesday afternoon. So if you are around, you are more than welcome to visit us there. In the meantime I hope I get the pull request through, so that everybody interested can play around with the algorithm. It will be located in the segmentation module. There is also an example pcl\_example\_lccp\_segmentation. To get you interested the following image shows an 
example of the segmentation. As you can see all parts can be easily named.
  
  .. image:: images/lccp_example.png
    :align: center
 
  That's it for the first post. Hope to see some of you at CVPR. Stay tuned for more to come. 
 