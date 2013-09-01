.. blogpost::
  :title: Porting SSD
  :author: fcalakli
  :date: 04-22-2012
  
  On Monday (4/16), I had a short discussion with my mentor about how to improve surface reconstruction code in PCL. We started porting our SSD software based on the work `SSD: Smooth Signed Distance Surface Reconstruction <http://mesh.brown.edu/ssd/>`_. The code hasn't been pushed to the trunk yet. Once this is ready, it will provide a base for us to make further improvements. I am going to make another blog post soon to share our improvement plans. 

.. blogpost::
  :title: Reimplementing SSD
  :author: fcalakli
  :date: 05-07-2012
  
  I am currently reimplementing SSD using PCL. It looks like I need additional datastructures defined over the octree, such as primal and dual graphs. I am investigating if the current octree version supports those graphs. Otherwise, I will implement these. As timing for this project, I am planning to deliver the software (and a report for the sponsors) by June 18. 
  
.. blogpost::
  :title: Initial results on (new) SSD 
  :author: fcalakli
  :date: 07-26-2012
  
  I have been able to reformulate the mathematical framework used in SSD for implementation in PCL, without having to explicitly allocate primal and dual graphs defined over the octree. Over the past few weeks, I have implemented this extended formulation and have done many experiments to figure out the range of regularization parameters involved in the optimization. The results look very good. 
  
  There are 3 independent tasks in the implementation:

    1. Given a point cloud with normals, construct an octree (Julius' extended octree does this)
    2. Given an octree, estimate scalar field per leaf (My extended SSD will do this)
    3. Given a scalar field defined defined over octree leaves, either render the volume or extract isolevel zero as a polygonal mesh ( for the latter there is a  need for implementation of Dual Marching cubes algorithm )
  
  .. image:: img/alternative_impl_results.png
	 :align: center
	 
  SSD with standard implementation (left) vs. proposed implementation (right). Given that the results obtained by using new formulation are quite comparable to ones obtained by using the standard implementation, I now proceed to incorporate this implementation into PCL. 
  
  