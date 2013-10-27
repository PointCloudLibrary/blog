My status updates
=================

.. blogpost::
  :title: Final report
  :author: moerwald
  :date: 09-24-2012

  As a final blog post for this Trimble Code Sprint, I am attaching the final report I have written for the sponsors.

  .. raw:: html

    <iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Ftrcs%2Fmoerwald%2Ffiles%2Ffinal_report.pdf%3Fraw%3Dtrue&embedded=true" width="400" height="800" style="border: none;"></iframe>

.. blogpost::
  :title: Trimming the bunny
  :author: moerwald
  :date: 05-29-2012
  
  I've added NURBS curve fitting to the example of surface fitting. The curve is fitted to the point-cloud in the parametric domain of the NURBS surface (left images). During triangulation only vertices inside the curve are treated, borderline vertices are clamped to the curve (right images). 
  
  .. image:: img/nurbs_bunny_trim.png
    :align: center 

.. blogpost::
  :title: Fitting the bunny
  :author: moerwald
  :date: 05-22-2012
  
  The functions of NURBS fitting are documented within the header files. I've also added a test and example file in examples/surface where you can test the algorithms and try out to fit some pcd files (e.g. test/bunny.pcd). The result should look like the image below.
  
  Coming up next:
  
      * Trimming of the bunny using the B-Spline curve fitting algorithm.
  
  .. image:: img/nurbs_bunny.png
    :align: center 
  
.. blogpost::
  :title: NURBS fitting algorithms integrated
  :author: moerwald
  :date: 05-21-2012

  I've integrated all the NURBS fitting stuff (curve and surfaces) using PDM (point-distance-minimization), TDM (tangent-distance-minimization) and SDM (squared-distance-minimization). Therefore I've put openNURBS 5 into the repository as well. A good comparison of the fitting techniques PDM, TDM and SDM are described in "Fitting B-spline curves to point clouds by squared distance minimization" by W. Wang, H. Pottmann, and Y. Liu  (http://www.geometrie.tuwien.ac.at/ig/sn/2006/wpl_curves_06/wpl_curves_06.html)
  
  Coming up next:
  
      * Consistent documentation and code cleaning.
      * Examples for better understanding of the usage.
      * Conversion of NURBS to polygon meshes.

.. blogpost::
  :title: Publishing
  :author: moerwald
  :date: 04-12-2012

  Hi, I'm still working on some paper for submittion next week and another one in three weeks. After that I'll clean up the code and commit it to pcl.

.. blogpost::
  :title: Curve fitting results
  :author: moerwald
  :date: 03-31-2012

  Below I posted some results of the B-Spline curve fitting algorithm. I haven't found any existing code that can handle point-clouds like those in the image. The reasons for this are heavy clutter and noise inside and at the boundary (c,d), strong turns and concavities and regions where the boundary is quite close to another one (a,b).

  After finishing the work I'll have a talk with Radu to discuss about implementation issues regarding NURBS, which will define how the curve fitting algorithms will be implemented.

  .. image:: img/curve-fit.png
    :align: center


.. blogpost::
  :title: Ill last week
  :author: moerwald
  :date: 03-26-2012

  Hi, unfortunately I was ill last week and I'm struggeling a little bit with paper deadlines right now. I'll post some results of the paper this week and will integrate the stuff right after the deadline.

.. blogpost::
  :title: Fitting NURBS curves (real data)
  :author: moerwald
  :date: 03-19-2012

  Fitting NURBS curve to real data in a generic way is quite tricky. I've developed an algorithm to fit NURBS curves in an error-adaptive manner, with respect to knot insertion and concavity filling. This means that at regions where the error (distance from the curve to the closest point) is high, control points are added by knot insertion and simultaneously the concavity of the control point is increased bending the NURBS curve inwards. Not modeling this concavity constraint would lead to bridging of narrow gaps.

  Please have a look at the video "Curve fitting" on my homepage: http://users.acin.tuwien.ac.at/tmoerwald/?site=4#nurbs

.. blogpost::
  :title: Fitting NURBS curves
  :author: moerwald
  :date: 03-13-2012

  One of the geometric opperations on NURBS is to trim them. That is to cut out holes or define a boundary which has a different polynomial degree then the NURBS itself. Then, trimming is nothing else but defining regions on the NURBS where it is visible or invisible. In our case we need it for defining the boundary of arbitrary surfaces and furthermore for intersecting NURBS of different polynomial degree.
  
  To apply such boundaries to real data we want to fit Nurbs curves to a point-cloud boundary. In the image below you see some test case of a boundary with some outliers in the middle (think of typical kinect data). The curve is initialised with PCA with a radius equal to the first eigenvalue (iteration 0), and then successively refined and fitted. The fitting weight of the points (blue) is influenced by the distance (green) to the curve (red) using the gaussian function so that close points lead to high influence and vice verca.

  To account for higher frequence in the measurement, the degree of freedom of the curve is increased by knot insertion (iteration 1 and 2). After a few iterations the curve approximates data quite nice (iteration 5).

  .. image:: img/curve-lo.png
    :align: center

  The radius of the fitted curve is 0.5 with an overlayed sinusoidal of peak amplitude 0.05.

  Coming up next: Fitting of NURBS curves in the parametric domain of NURBS surfaces and trimming (on real data).


.. blogpost::
  :title: NURBS fitting on real data
  :author: moerwald
  :date: 03-08-2012

  I've made some experiments with cylindrical NURBS and NURBS patches on real data. I've uploaded two videos to my homepage: http://users.acin.tuwien.ac.at/tmoerwald/?site=4.

.. blogpost::
  :title: Cylindrical NURBS
  :author: moerwald
  :date: 03-04-2012

  UPDATE: The inversion of points for NURBS cylinders is now fixed and takes the same time as for NURBS patches

  This week I implemented fitting of periodical NURBS, namely cylinders. Unfortunately the inversion of points (i.e. finding the closest distance from a point to the NURBS surface) takes (for some reason I didn't find yet) much longer than for NURBS patches. In the image below you can see some data points in blue. The NURBS cylinder is initialized using PCA. Then for each point the distance to the closest point on the NURBS surface is calculated (red lines), and the linear equation for minimizing this distance is assembled and solved. As usual a regularisation term models 'stiffness' of the NURBS (magenta).

  .. image:: img/cylinder.png
    :align: center

.. blogpost::
  :title: submitted pcl/surface/nurbs
  :author: moerwald
  :date: 27-02-2012

  I've submitted the templated version of NURBS fitting to pcl trunk and tested it with a small program.

.. blogpost::
  :title: Global optimization
  :author: moerwald
  :date: 22-02-2012

  The last week I worked on global optimization of objects with several fitted NURBS surfaces. As mentioned in earlier posts there are several problems when fitting several NURBS to C^1 continuous regions of an object, like overlaping, gaps and other intersection and alignment combinations.

  Until now I was fitting C^1 continuous regions sequential in a linear equation. The key idea of global optimization of NURBS is to assemble all NURBS fitting equation of one object into one system of linear equations. This allows to define relationships between NURBS like the closing boundary constraint. This one basically defines that a point on one NURBS lies on a point on another NURBS. This is especially visible in the 'Boxes' videos available at http://users.acin.tuwien.ac.at/tmoerwald/?site=4.

  The points for closing the boundary between NURBS are also used for trimming them. Since those points are by definition the outline of the sub-point-cloud of the C^1 continuous region they are simply added to the triangulation algorithm.

  For Triangulation the established Delaunay Triangulation is used (http://www.sintef.no/Projectweb/Geometry-Toolkits/TTL/). The textures are captured from the RGB image by simply projecting them into the camera plane. This causes the problem that the texture of occluded area is the same as the one of the occluder. To solve this problem I want to implement a z-buffer to check which surface is the nearest.

  Coming up next:

    - Multiple views for complete models.


.. blogpost::
  :title: Closing the gaps
  :author: moerwald
  :date: 13-02-2012

  Hi, I've commited the code for NURBS fitting to svn last week. There's still some clean-up necessary but the code should work fine!

  I'm now starting with optimization methods for reconstructed NURBS surfaces: After fitting a set of NURBS to a point cloud, a couple of things have to be done to make a nice CAD model out of it. E.g. gaps have to be closed, intersecting NURBS need to be trimmed, ...

  To deal with this issues I'm working on a global optimization approach, where I solve a linear equation in the least square sense, so that the NURBS still approximates the point-cloud while meeting the constraints (formulated as weak) mentioned above.

.. blogpost::
  :title: Holiday + NURBS integration finished
  :author: moerwald
  :date: 27-01-2012

  I'll be on my snowboard for the next week, so don't be surprised if I don't respond until Monday 6th of February.

  I've got NURBS now fully functional integrated into pcl. But since I'm on holiday next week I will commit it to the svn-repo when I come back. So that I'm available just in case of problems.

.. blogpost::
  :title: Finally
  :author: moerwald
  :date: 24-01-2012

  A little late I also figured out the blogging system. Had a couple of exams last week, the last one yesterday, so I'm now ready to start.

  Regarding my plan for the next couple of days/weeks: I'm implementing NURBS basis functions so that afterwards I can modify my NURBS fitting functions to remove the dependency of openNURBS.

