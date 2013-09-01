.. blogpost::
   :title: Pencils Down
   :author: glong
   :date: 8-22-2011

   Today is the official "pencils down" date for GSoC 2011.  My project is now officially "over," and I'm writing this blog post as a final summary of my work this summer, and as a look into the future of surface reconstruction for PCL.

   This summer, I've completed a large majority of what I've set out to do.  I've reviewed the surface reconstruction API, implemented Marching Cubes, ported over Poisson Reconstruction, connected our surface reconstruction algorithms to VTK's surface smoothing techniques, and did a lot of further research into the world of surface reconstruction.  All in all, I'm pretty satisfied with the amount of progress that has been made over these past couple months.

   That said, I feel there's still a lot more work to be done.  Some tutorials should be written for these surface reconstructions, as well as unit tests and more detailed documentation.  The Poisson reconstruction code is basically a copy of Misha's, so this should be ported further, to utilize our own in house Octree and Marching Cubes code.  The Marching Cubes code should also be further extended to handle octree based voxel grids.  Finally, there are many other surface reconstruction algorithms out there that should be added to the library.

   I've had a great time this summer working on PCL.  Having come in with very little experience in professional or open source software development, it's been a great learning experience for me, teaching me a lot about efficient C++ coding as well as code management and design.  I've also found the topic of surface reconstruction very interesting, and will be continuing to work on improving PCL's surface reconstruction library in the future.  Thanks to all the PCL developers, and see you on the mailing list!
   
.. blogpost::
   :title: Poisson Reconstruciton Results
   :author: glong
   :date: 8-17-2011

   I've successfully ported over Misha's Poisson reconstruciton code, and here are the results:

   .. image:: images/poisson_horse1.png
   .. image:: images/poisson_horse2.png

   This model was generated with the sample data on Misha's site, which provides both points and normals.  In an ideal case, the results look extremely good.  When the points alone are ported in without any normal information, the image quality is a bit worse:

   .. image:: images/poisson_horse_nonorm.png

   These norms were generated in PCL, with a K-NN value of 10.  This is a great illustration of how important a good set of surface normals are for accurate reconstruction using implicit surface algorithms like Poisson reconstruction.

.. blogpost::
   :title: Misha's Poisson Reconstruction
   :author: glong
   :date: 8-11-2011

   I've successfully ported over Misha's Poisson reconstruction code into PCL.  As of now, the interface is still rough, and needs refinement, however the reconstruction algorithm does run.  Unfortunately, I was unable to code my own version of Poisson reconstruction, due to summer of code ending soon, but I think that an in-house implementation would be the best in the long term, since marching cubes is already implemented, and a more complex, octree based marching cubes algorithm could be then used in tandem with any manner of implicit surface algorithm.  This would allow future implementations of implicit surface reconstructions to reuse this code and make life that much easier for developers.  The way that the Poisson code is structured now, it is a standalone algorithm with its own implementation of marching cubes, matrices, etc.

   More details, along with pictures, coming soon!

.. blogpost::
   :title: VTK Smoothing Integration
   :author: glong
   :date: 8-1-2011

   I've finished integrating VTK smoothing algorithms into PCL.  There is now an object VtkSmoother, that takes in a PolygonMesh, and outputs a new PolygonMesh that is smoothed.  The actual algorithm first optionally subdivides the polygons into smaller triangles, using either a linear, loop, or butterfly subdivision filter.  It then runs either VTK's LaPlacian or WindowedSinc smoothing algorithm, and outputs it into another PolygonMesh.  To read more on these algorithms, the VTK documentation gives good information on them: http://www.vtk.org/doc/release/5.6/html/.

   I've spent most of my time analyzing the WindowedSinc smoothing algorithm, and the different parameters such as the number of iterations, and the size of the window.  Also, I've taken a look at the different subdivision methods.

   On the Bunny dataset, there aren't many differences between the Linear, Loop, and Butterfly subdivision methods:

   .. image:: images/bunny_smooth_linear.png
   .. image:: images/bunny_smooth_loop.png
   .. image:: images/bunny_smooth_butterfly.png

   The difference is very slight between them all, in both results and in computational time.

   The number of iterations affects how much smoothing is performed.  In general, the Windowed Sinc algorithm converges quickly, so the number of iterations doesn't affect the results much.  In general, 20 iterations is sufficient.  Here are the results after 5, 10, and 15 iterations:

   .. image:: images/bunny_smooth_5.png
   .. image:: images/bunny_smooth_10.png
   .. image:: images/bunny_smooth_15.png

   The window size also affects the amount of smoothing, and is the most effective way of changing the results.  In general, values between 0.01 and 0.001 work well.  Here are results with 0.1, 0.01, and 0.001 window size:

   .. image:: images/bunny_smooth_0p1.png
   .. image:: images/bunny_smooth_0p01.png
   .. image:: images/bunny_smooth_0p001.png

   One improvement is to include hole filling for the mesh.  Other work integrating more polygon mesh algorithms from VTK is also possible, and should be straightforward in the future.

.. blogpost::
   :title: VTK Smoothing Results
   :author: glong
   :date: 7-20-2011

   This week I've been working on integrating the VTK surface smoothing algorithms in to the surface reconstruction code.  I've successfully converted a pcl PolygonMesh into VTK format, and run the subdivision and surface smoothing algorithms on it.  I've been using vtkLinearSubdivisionFilter and vtkWindowedSincPolyDataFilter for these tasks.  The results for the gp3 bunny are very nice:

   .. image:: images/bunny_gp3_nosmooth.png
   .. image:: images/bunny_gp3_smooth_0p001.png

   There are also parameters that you can tweak for the surface smoothing filter.  Here I am changing the pass band from 0.001 to 0.1:

   .. image:: images/bunny_gp3_smooth_0p1.png

   More updates to come as I implement more smoothing features.

.. blogpost::
   :title: More Research on Surface Reconstruction
   :author: glong
   :date: 7-15-2011

   Doing some more digging on surface reconstruction, I found this nice summary of current surface reconstruction algorithms: http://stackoverflow.com/questions/838761/robust-algorithm-for-surface-reconstruction-from-3d-point-cloud.  

   In general, there are 2 categories of algorithms: Computational Geometry and Implicit Functions.  Computational Geometry includes algorithms that attempt to build the surface from the ground up, for example the ball pivot algorithm or our own greedy triangulation algorithm.  Implicit function algorithms create implicit functions for a point cloud, and uses something like marching cubes to extract a surface.  Examples of these are Poisson reconstruction, and my own naive method of voxelizing data.  These algorithms are typically more robust with respect to noisy data and varying density point clouds, and can also be implemented to process large inputs efficiently.  A drawback is that they require normals for all points - however, PCL's surface normal generation methods should take care of this problem :).

   Since implicit functions seem so widespread, I believe a more powerful version of marching cubes is needed.  Therefore, after integrating the VTK surface smoothing tools, and possibly Poisson as well, I will begin revamping my marching cubes code to make it as general and efficient as possible, so that any number of these implicit functions can be plugged in easily to the marching cubes base class.

.. blogpost::
   :title: Improved Dot Product Based Voxelization
   :author: glong
   :date: 7-15-2011

   I developed a new method of voxelizing point clouds, utilizing a dot product to eliminate the double surface problem.

   In the current naive voxelization method, if a point is found within a voxel, that voxel is filled, or set to 1, and neighboring voxels' vertices are filled accordingly.  Then, when the isosurface is extracted, the marching cubes algorithm marches with a value between 0 and 1.  This causes a double surface effect:

   .. image:: images/mc_double.png

   This is undesirable as it's not an accurate representation of the isosurface, and it uses twice as many polygons to draw.  A simple way of solving this is to somehow fill the inside of the isosurface, however for objects that aren't enclosed, this won't quite work, and it also requires all the voxels to be enumerated, reducing performance.  A quick fix is to do a dot product between the point's normal vector and the vector between each neighboring voxel's center and the point itself:

   .. image:: images/mc_dotproduct.png

   If the dot product is less than 0, then that voxel's vertices are not filled.

   This results in a single surface, but it also introduces holes into the surface:

   .. image:: images/bunny_mcdot1.png
   .. image:: images/bunny_mcdot2.png

   Not great, but a modest improvement.

.. blogpost::
   :title: A Comparison of Current Surface Reconstruction Algorithms Part 2
   :author: glong
   :date: 7-14-2011

   In this post, I'll go over Organized Fast Mesh, Concave/Convex Hull, and Poisson reconstruction algorithms, as well as Moving Least Squares and SurfelSmoothing.

   Poisson Reconstruction is a method of voxelizing data, combined with an octree-based implementation of Marching Cubes.  Code is available at http://www.cs.jhu.edu/~misha/Code/PoissonRecon/, and is being ported into PCL currently, however a more tightly integrated version could probably also be made as a child class of MarchingCubes, provided that the Marching Cubes class is extended to handle octree based volumes.  

   Organized Fast Mesh is a simple reconstruction algorithm for organized point clouds.  Neighboring points are connected to construct a triangular mesh.  This algorithm is fast, and very simple, but requires points to be ordered.

   Concave/Convex Hull are algorithms that construct a concave/convex hull defined by the input point cloud.  These algorithms are not based on the base reconstruction algorithm, but instead a separate implementation.  For more information on convex hulls, see http://en.wikipedia.org/wiki/Convex_hull.  For more information on the algorithm, see http://www.pointclouds.org/documentation/tutorials/hull_2d.php#hull-2d.

   Moving Least Squares is not a reconstruction algorithm, but instead a method of smoothing and resampling noisy data.  More information can be found at http://www.pointclouds.org/documentation/tutorials/resampling.php#moving-least-squares.  Finally, SurfelSmoothing is a tool for smoothing a cloud with normal vectors.


.. blogpost::
   :title: A Comparison of Current Surface Reconstruction Algorithms Part 1
   :author: glong
   :date: 7-13-2011

   As of now, PCL implements the following surface reconstruction algorithms:

   - Greedy Projection Triangulation
   - Grid Projection
   - Marching Cubes
   - Organized Fast Mesh
   - Concave/Convex Hull
   
   Additionally, Poisson reconstruction is being added as well.  There are other classes found in the library, Moving Least Squares and SurfelSmoothing, that do not actually perform reconstruction, but instead are tools to help improve surface reconstruction in some way.

   In this post, I'll do a comparison of these reconstruction methods, Greedy Projection Triangulation, Grid Projection, and Marching Cubes.

   Greedy Projection Triangulation
   The Greedy Projection Triangulation is a greedy algorithm that begins with a seed triangle, and connects subsequent triangles to it, repeating this until all points are connected.  The algorithm can be found in the paper "On Fast Surface Reconstruction Methods for Large and Noisy Datasets" by Marton, Rusu, and Beetz.  The paper can be found at http://files.rbrusu.com/publications/Marton09ICRA.pdf.  This algorithm runs quickly, and produces good quality images with the test data set.  One potential downside to the algorithm is that it requires several parameters, such as distance between connected points, and minimum and maximum angles, which may need to be tuned for any given dataset.

   .. image:: images/bunny_gp3.png

   The Grid Projection algorithm is a grid based surface reconstruction algorithm.  Points are first partitioned into voxels, and a vector field is constructed, where the vectors at any given point are directed at the nearest point.  A surface is then determined by examining where vectors with opposite directions point towards.  Edges in the voxels that this surface are reconstructed from are determined, and padding cells (cells neighboring the voxels containing the points) are also added.  The center points of each voxel are then projected based on the edge intersections, and the surface is reconstructed by connecting these center points.  A more detailed explanation can be found at http://www.pointclouds.org/news/surface-reconstruction-from-point-clouds.html.  The output is very high quality, and requires only 2 parameters, padding and resolution, however the algorithm takes significantly longer than Greedy Projection or Marching Cubes - 10s of seconds, as opposed to <1 second for the sample data set.

   .. image:: images/bunny_gridproj.png

   The Marching Cubes algorithm is a well known method of extracting an iso-surface from a set of voxels.  An explanation of the algorithm can be found at http://paulbourke.net/geometry/polygonise/.  This algorithm takes in a set of voxels, and produces a list of polygons, therefore a method of voxelizing the data must be performed first.  The current implementation of marching cubes contains a simple naive method of filling in the voxel in which a point lies.  This method is fast, however the image quality is significantly lower than other methods.  For more details on Marching Cubes image quality, see my previous blog post.  The only input required for this naive method is the resolution.  Image quality can also be improved with more clever means of voxelizing the data, as well as implementing dynamically scaled voxels with an octree data structure.

   .. image:: images/bunny_mc.png

   In my next post, I will cover the remaining reconstruction algorithms, and the other toolkit classes found in the surface reconstruction package.

.. blogpost::
   :title: Status Update
   :author: glong
   :date: 7-6-2011

   It's been a while since I've updated!  I was out of town for the 4th of July holiday, and therefore didn't accomplish a lot last week.  This week, I've been porting over the Poisson reconstruction algorithm, and also looking into the VTK smoothing algorithms, as well as playing with MeshLab to see what they have to offer.

   MeshLab is a program that manipulates polygonal meshes, built on the VCG Library (http://vcg.sourceforge.net/index.php/Main_Page).  One of its features is surface reconstruction of point clouds, so their implementations are good to look at to see what we may need to implement.  From what I can tell, the two main reconstruction algorithms are Marching Cubes and Poisson.  Since Poisson shouldn't really be a separate reconstruction aglorithm, I will look into the code and see exactly how marching cubes and Poisson differ in this case.

.. blogpost::
   :title: Poisson Reconstruction Summary
   :author: glong
   :date: 6-27-2011

   After a more detailed examination the Poisson reconstruction algorithm, I've discovered that it is actually a method for constructing a voxel grid, on which marching cubes is then run.  Specifically, the paper (http://research.microsoft.com/en-us/um/people/hoppe/poissonrecon.pdf) outlines a method using an octree.  Since Misha (http://www.cs.jhu.edu/~misha/Code/PoissonRecon/) has allowed us to integrate his code into PCL, I will most likely do a separate port of this code into PCL as its own class.  This is non-ideal, however.  I believe the best solution would be to use the existing marching cubes implementation to reconstruct the octree that is output by the code.  However, as I wish to add as many methods as possible, I will first implement the Poisson method in its entirety, and only later on update the marching cubes code to take octree inputs as well.  It could be that adding octree implementation to the marching cubes code would be simple, and I will keep this in mind and change plans accordingly depending on what I find.

.. blogpost::
   :title: VTK Surface Smoothing First Look
   :author: glong
   :date: 6-27-2011

   VTK has a couple different surface smoothing algorithms at its disposal: vtkSmoothPolydataFilter and vtkWindowedSincPolyDataFilter.  These methods take in a set of polygons in VTK format, and output a smoothed version of the same.  There are also algorithms that will subdivide existing surfaces, vtkLoopSubdivisionFilter and vtkButterflySubdivisionFilter.  These take in a set of triangles, and then split each triangle into a set of four triangles.  I can imagine situations when one would wish to first split their mesh into a finer set of triangles, and then apply smoothing.  I believe integrating these smoothing filters would be simple, simply a matter of converting the data to VTK format, running the filters, and converting back.

.. blogpost::
   :title: Weekly Status Report
   :author: glong
   :date: 6-24-2011

   This week, I accomplished the following:

   - finished Marching Cubes implementation
   - began research into better voxelization methods
   - started research on Poisson Reconstruction
   - started to decouple voxelization from Marching Cubes

   Next tasks include:

   - decide on a method to decouple voxelization from Marching Cubes
   - implement Poisson reconstruction
   - develop good unit tests for the new classes
   - look into other novel surface reconstruction algorithms

.. blogpost::
   :title: Poisson Reconstruction & Marching Cubes Code Structure
   :author: glong
   :date: 6-24-2011

   After doing some preliminary research into Poisson Reconstruction, it turns out that the algorithm is actually just a novel method of voxelizing data.  In their paper, Kazhdan et al state that they use a modified version of Marching Cubes, utilizing an octree.  Given this new finding, I will restructure my Marching Cubes code to be more generic in its voxelization methods.  I believe the best method would be to create a virtual voxelization function in the MarchingCubes class, and have child classes specify the voxelization technique.  This way, any type of voxelization can be combined with the Marching Cubes isosurface extraction.

   An even more general method of decoupling the voxelization from Marching Cubes is to create two separate classes, Voxelizer and MarchingCubes, and have the voxelizer create a data structure of voxels, that is passed into the MarchingCubes surface reconstructer.  This would require a standard Voxel data structure as well. 

.. blogpost::
   :title: Marching Cubes Results
   :author: glong
   :date: 6-22-2011

   I recently completed work on the marching cubes algorithm.  The marching cubes algorithm is an algorithm for constructing a polygonal isosurface from a set of volumetric data.  For a more in-depth overview of the algorithm, see Paul Bourke's excellent overview (and code) at http://paulbourke.net/geometry/polygonise/.  Working off of his generously supplied code, the actual algorithm was easy to implement.  The difficult part was actually creating the volumetric data to generate the surface from.

   In other applications, a volumetric data set, such as a CT scan, is provided, and creating the volume is trivial: each vertex in the voxel is simply a point from this data set.  In the case of point clouds, however, this set of voxels must first be created.  As a first cut, I implemented a simple method for creating the voxels.  For each point in the cloud, I found the voxel that contains it, and set every vertex in that voxel to be 1, and also updated all of the neighboring voxels so that the correct vertices are set to 1 as well.  All other vertices are set by default to 0.  Marching cubes is then run on the resulting voxel set, using an isovalue of 0.5.  One great side effect of this method is that many empty voxels can be filtered out, and not traversed over, saving both computation time and memory.  The results are as follows:

   .. image:: images/bunny_mc1.png
   .. image:: images/bunny_mc2.png

   These results can be contrasted with the greedy surface triangulation method:

   .. image:: images/bunny_gp31.png
   .. image:: images/bunny_gp32.png

   The algorithm does produce a smooth, continuous isosurface, with some caveats:

   - The isosurface looks "blocky," in that there are only sharp 45 degree angles to outline curves.  This can be contrasted with the greedy surface triangulation method which can produce more smooth looking surfaces.
   - The isosurface has holes in some areas, some of which are highlighted in the second picture.
   - Even though the model used should be a planar segment, the algorithm produces a shell-like triangle enclosure that surrounds the actual point cloud.  This can be seen more easily in the second image.
   - The size of the voxel must be chosen carefully.  If the size is too small, you will get another point cloud, except with polygonal spheres encapsulating your points.  If too large, you'll get degenerate shapes that don't convey any structure.  This is actually a difficult problem to solve automatically, and so far I've only been doing it heuristically.  The following images show some examples of poorly chosen leaf sizes.

   .. image:: images/bunny_mc3.png
   .. image:: images/bunny_mc4.png

   From these results, I think that more work should be done in the actual creation of the voxel grid, to make marching cubes behave better.  The marching cubes algorithm is straightforward; voxelization of data is the interesting part.  I'll continue to think about this and post any new discoveries.  From an implementation standpoint, I think that we should also split the code into 2 parts: voxelization and marching cubes.  This will enable the user to pick different methods of constructing the voxel grid, and use the one that works best for them.  For now, I think the easiest thing to do will be to add it as a flag when running the algorithm, however in the future we might consider creating a separate voxel class, and allowing the user to manually create the voxels themselves.

   Next tasks include:

   - Complete unit testing and push the new code to the trunk
   - Begin Poisson reconstruction
   - Find a more robust voxelization method

.. blogpost::
   :title: Coding inertia
   :author: glong
   :date: 6-17-2011

   As a newcomer to large scale coding projects, one interesting problem I've had is the amount of time it took me to begin writing actual code.  In the beginning, I spent a long time simply examining the code, trying to understand all the different functions and existing tools (e.g. boost shared pointers, eigen vectors, search trees, etc).  It was a bit frustrating in the beginning as I felt I was not productive, but in retrospect, all the time I spent reading the code, playing with simple examples, and just thinking really pay off now that I'm writing code.  I feel like after a slow start, I can finally begin building up some momentum and get some tangible results.

.. blogpost::
   :title: Weekly status update
   :author: glong
   :date: 6-17-2011

   This week, I completed the following tasks:

   - I completed a review of the surface reconstruction API.  Ultimately, I decided to leave it be, and closely monitor it in relation to the development of new surface reconstruction algorithms.
   - I successfully added and compiled a skeleton MarchingCubes class.  I've completed the code to voxelize the input point cloud and am beginning to code the actual surface reconstruction algorithm.

   Next tasks include:

   - complete coding marching cubes
   - debug and design unit tests for the marching cubes algorithm
   - begin Poisson reconstruction and research new, novel reconstruction algorithms

.. blogpost::
   :title: Voxelization thoughts
   :author: glong
   :date: 6-14-2011

   The marching cubes algorithm is an algorithm used on volumetric data to construct isosurfaces.  For example, CT scan data may have a spectrum of values from 0 to 255, and the user would pick a value that corresponds to something interesting (for example, 192 may correspond to bone), and then an isosurface would be extracted that traces that value.

   The implementation for PCL is a bit different because a point cloud is a discrete set of points, without a spectrum of values.  In fact, the data isn't even in voxel format, so the cloud must first be converted.  There are some parameters that must be tweaked first, though, such as the voxel size, and how to determine the value at each voxel.  After some thought, I've decided that the best way to do this would be to create a high resolution voxel grid, no larger than the smallest distance between points.  As a first cut, we'll iterate through the point cloud, and assign each point's nearest vertex a alue of 1.  There are other ways of computing the value of each vertex, such as a weighted average of each nearest point to a vertex based on distance.  Once the code is up and running, I'll have to run tests to see what the best method would be.

.. blogpost::
   :title: final API notes
   :author: glong
   :date: 6-13-2011

   After a discussion on the pcl-developers mailing list, I realized that my API design was a bit overcomplicated.  Now that I think about it, I guess that I should have realized that this is an API meant for more seasoned programmers that will be using PCL, as opposed to some new user that may break things easily.  Also in the interest of backwards compatibility, we decided to ultimately leave the API as is, and modify it later if needed for other surface reconstruction algorithms.

   Next up on my to do list:

   - Implement the Marching Cubes algorithm
   - Implement Poisson reconstruction
   - Do more research on surface reconstruction algorithms and implement some of them

.. blogpost::
   :title: Ruminations on API design
   :author: glong
   :date: 6-9-2011

   My first task for my GSoC project is to design an API for all surface reconstruction methods in PCL.  As I'm new to large scale software development and have very little background in software engineering practices in general, I thought it a good idea to first do a bit of research into API design.  Some googling turned up these two links:

   - How to Design a Good API and Why it Matters - Joshua Bloch, Google http://lcsd05.cs.tamu.edu/slides/keynote.pdf
   - On API Design Guidelines - neuroning.com http://neuroning.com/2006/11/19/on-api-design-guidelines

   In short, I've gotten this general impression of API design:

   - The API is just as simple as it sounds - the interface that users see to access the methods.  In this case, users will probably create some sort of object that will then have methods to construct a surface given certain parameters.
   - That being said, API design is not necessarily simple.  In general, however, the cardinal rule seems to be this: make it easy for users to use the API correctly, and hard for them to use incorrectly.

   My proposed API would be a class, SurfaceReconstruction, that should be as black box as possible - that is, parameters are set, and then reconstruct() is called, and you're done.  The input is a valid point cloud, and optional tuning parameters, and the output is a surface mesh.
   I believe planning is very important, especially in the early stages, but now of course come the hard (and fun) part: actually coding it up.

.. blogpost::
   :title: Some minor setbacks
   :author: glong
   :date: 6-7-2011

   As I'm on the quarter system, there's been quite a bit of delay on my part getting started on my project.  Thankfully, I've finished classes, and now I'm ready to get going!

   This week, I've successfully installed PCL on my Mac.  This was actually a non-trivial task, with a lot of building and tweaking in order for everything to go.  I've noted the process, and am going to try to post another blog entry on how to install on OS X 10.6, for those other unfortunate Mac users that don't want to resort to using a VM to develop.

   I've also set up an Eclipse environment to develop in, another non-trivial task for people like me who are used to coding in a simple editor like Emacs.  I'm also considering writing up a short post on how to set up Eclipse.

   Finally, I've worked through some of the tutorials.  While coding I encountered an interesting duality of sorts: the ease of following the tutorial code and coding up new examples of my own, and the difficulty in understanding exactly how to set up the tutorials in the first place.  Another final post might be my thoughts on the sort of mindset one needs to work on, and use PCL.  Using PCL is obviously different than coding for PCL, and I think it's important to remember the distinction of using it as a tool, vs actually improving that tool.  That being said, it's also important to understand both clearly, as the tool has to be useful for the user, or else it's a useless tool.

   Next steps include: coming up with a unified surface reconstruction API, and implementation of some totally sweet surface reconstruction algorithms.

.. blogpost::
   :title: My first status update
   :author: glong
   :date: 5-24-2011

   Today I learned how to add content to the developer blogs.

   Here's a code snippet:

   .. code-block:: c

      // This is a really boring block of code...
      int n = 10;
      for (int i = 0; i < n; ++i)
      {
        printf ("%d\n", i);
      }

   And here's an equation:

   .. math::

      ax^2 + bx + c = 0