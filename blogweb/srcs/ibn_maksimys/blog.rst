.. blogpost::
  :title: My first step
  :author: ibn_maksimys
  :date: 03-21-2012

  Hello everybody! This is my first blog entry as a participant of PCL-SRCS. I filled out information about myself. Now we refine the algorithm that will be implemented in the PCL-SRCS.

.. blogpost::
  :title: The chosen algorithm
  :author: ibn_maksimys
  :date: 04-07-2012
  
  This week in Wednesday we chose algorithm for implementation: `Template Deformation for Point Cloud Fitting <http://www.mpi-inf.mpg.de/~stoll/paper/td.pdf>`_. I think that it will be first iteration for common algorithm: Template matching using 2D + 3D.
 
  Now I'm researching algorithm and thinking about its implementation. I will have consultation with my mentor next week. After this I will describe here basic steps of implementation and will begin realization.

.. blogpost::
  :title: Description of the algorithm
  :author: ibn_maksimys
  :date: 04-16-2012
  
  The steps of the `chosen algorithm <http://www.mpi-inf.mpg.de/~stoll/paper/td.pdf>`_:
  
  .. image:: img/algorithm.jpg
	 :align: center
	
  User-program interaction:

		1.	User must choose the most appropriate template for the input point cloud.
  
		2.	User must identifies and marks pairs of corresponding points on template and point data, defines a local frame for every marked point (Fig. 1 (a, b)).
  
  The program:
  
		1.	From the selected correspondences, we compute the initial deformation of the template. We compute Laplacian coordinates of the template and estimate local rotations for every pair of corresponding points.
  
		2.	Make initial deformation (Fig.1 (c)).
  
		3.	We estimate a global scaling factor for the template which is applied to the Laplacian coordinates, to consider the fact that template and input data may be (and generally are) scaled differently may distort the resulting shape in an unacceptable way . Make new deformation (Fig.1 (d)).
  
		4.	Iterative process moves the template nearer towards the data points guided by local correspondences which are established from simple heuristics. This local matching is motivated by iterative closest point (ICP) algorithms for finding (rigid) transformations for shape registration (Fig.1 (e-g)).
  
		5.	We improve the remaining regions of the deformed template for which no counterparts exist in the data (Fig.1 (h)).

  The first phase of the realization described algorithm, I will write code to calculate the initial approximation and estimate the coordinates of Laplace
	
.. blogpost::
  :title: Laplacian matrix
  :author: ibn_maksimys
  :date: 04-25-2012
  
  Last week, I wrote the program for calculation the Laplace matrix with uniform and cotangent weights (Described in the article `Laplacian Mesh Optimization <http://www.cs.jhu.edu/~misha/Fall07/Papers/Nealen06.pdf>`_). 
  Now I want to write a program for smoothing based on the Laplacian matrix.
  
.. blogpost::
  :title: Laplacian mesh editing: MATLAB realization
  :author: ibn_maksimys
  :date: 05-13-2012
  
  Now I try implement algorithm for 3D mesh editing base on Laplacian matrix. I decide to do it in MATLAB for verify my idea. If it will work I will implement it in C++.
	
.. blogpost::
  :title: Laplacian mesh editing: MATLAB realization
  :author: ibn_maksimys
  :date: 06-04-2012
  
  I almost not was working with the project the last two weeks. I was making report on my master's thesis. Almost completed work on it. Resume work on the project: now I will work on Matlab realization deformation of the surface.

.. blogpost::
  :title: Intermediate results
  :author: ibn_maksimys
  :date: 06-29-2012
  
  I'm graduated from my University. Now I have master degree and I have enough time to work at project.
  I figured out and implemented in Matlab surface editing based on the coordinates of Laplace. This conversion is the basis of the method that I have to implement in the Code Sprint.
  Implemented algorithm is Laplacian Mesh Editing consists of the following steps:
  
		1. 	Based on information about the relative positions of surface points computed the Laplacian operator of the mesh.
		2. 	Fix points whose position should not change - "static anchors".
		3.	Choose the point, whose position should be changed, and specify the offset of its origin - "handle anchors".
		4.	Based on available information, we construct the normal equations. For the static anchors we set big weight, and for the handle anchors little weight.
		5.	The new coordinates of the surface are calculated based on the method of least squares.
		
  The results of the program are presented below.
  
  .. image:: img/Man.png
	 :align: center
	 
	 
  .. image:: img/Circ.png
	 :align: center
	 
  Now I proceed to implement the algorithm from article "Template Deformation for Point Cloud Fitting". 



