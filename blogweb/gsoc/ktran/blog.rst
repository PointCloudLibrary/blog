.. blogpost::
   :title: Finished and back to school
   :author: ktran
   :date: 8-13-2011

   As Zoltan suggested, I have implemented extra step to remove the triangles in the overlapping region given by 2 meshes. The first version proposed by Zoltan is that we will go over the triangles in the first mesh and check if the its center has a nearest neighbor in the other cloud that is closer than the maximum distance from the center to one of the vertices and this nearest neighbor has to be in at least one triangle of other mesh, we will delete this triangle of the first mesh. To search for nearest neighbor, I have used k-nearest neighbor search with k =1. 

   To delete triangle from a mesh, we set a vertex to FREE if this triangle was only a triangle which the vertex was in. After that update SFN and FFN of other two vertices. If this was not the only triangle the vertex was in, it has to be set to FRINGE. Updating the sfn and ffn here is a bit more tricky. Check if on of the other triangles this vertex is in also has as another vertex one of the other points. Following Zoltan suggested I have implemented three more functions to complete this task.

   And here are results:

  
   The 1st mesh:
  	
   .. image:: images/mesh1.png
     :width: 800

   The 2st mesh which overlapps 1st mesh:
  	
   .. image:: images/mesh2-over_mesh1.png
     :width: 800

   The result after removing overlapped triangles and merging 2 meshes:
  	
   .. image:: images/mergedMesh.png
     :width: 800


.. blogpost::
   :title: Combine mesh update and texture mapping
   :author: ktran
   :date: 8-4-2011

   A small update on combining mesh update and texture mapping. I just created the function that will update the texture mesh when a new point cloud is added to the mesh. Now I will move to the final part of the project.

   .. image:: images/tex_mesh_update00.png
     :width: 800


.. blogpost::
   :title: Update the mesh without recreating the entire mesh
   :author: ktran
   :date: 8-3-2011

   Such a long time I haven't update my progress since finishing my 1st part. I am working on part II of my roadmap. I just finished implementing the surface mesh update when a new point cloud is added to an existing surface mesh, without recreating the entire mesh. Basically, the implementation of greedy triangular algorithms is based on the algorithm in paper "A fast and efficient projection-based approach for surface reconstruction". The terminology is that we will assign the data point at any given state of the algorithm as FREE, FRINGE, BOUNDARY and COMPLETED points. To update the mesh without recreating the entire mesh for a updated cloud, as Zoltan suggested, we assign the new points with FREE state, change the states of BOUNDARY points to FRINGE points and enter the triangulating loop which only keep the current mesh and updates new trianges. 

   To test this function, I have splited bunny point cloud dataset to 2 point clouds. 

   This figure show 2  meshes after using greedy triangular algorithm on 2 separated datasets.

   .. image:: images/bun_2meshes.png
     :width: 800

   And here is result of updated mesh when one point clouds is added to existing mesh.
	 
   .. image:: images/bun_updated.png
     :width: 800

   And here is result of the original mesh before splited.

   .. image:: images/bunny_original.png
     :width: 800



.. blogpost::
   :title: Finish part I of my roadmap
   :author: ktran
   :date: 7-12-2011

   Finally, I have finished part I in my roadmap:

	* Starting from the existing methods (greedy surface triangulation algorithm) to obtain triangle mesh, I have designed and implemented the API for texture mapping onto a mesh.	
	* I have finished implementing paper on texture synthesis from the paper "Texture Synthesis on Subdivision Surfaces".	
	* I have finished designing and implementing texture mapping and blending for two overlapping textures onto a surface mesh.

   Here are some snapshots of mapping 2 overlap textures to a mesh:

   .. image:: images/july1201.png
     :width: 800
   .. image:: images/july1200.png
     :width: 800 

   And here are examples of texture atlas for further processing.
	 
   .. image:: images/tex_00.jpg
     :width: 500
   .. image:: images/tex_10.jpg
     :width: 500 


.. blogpost::
   :title: Generating texture atlas for 2 overlapping textures onto a surface mesh
   :author: ktran
   :date: 7-6-2011

   I am working on generating texture atlas for 2 overlapping textures onto a surface mesh. This part basically pack all individual textures in the atlases. I am using OpenCV library to line all textures up horizontally. As I know, as common rending function requires that the width and the height of atlas should be less than 2048. So we may need multiple atlases in order to store  all the textures. My implementation is working fine except few unexpected texture mappings still need to be correct when I map the mesh with generated atlases. I will plot something after I fix the errors.

.. blogpost::
   :title: 2 overlapping textures onto a surface mesh
   :author: ktran
   :date: 6-27-2011

   Recently, I have generated an example of 2 overlapping textures onto a surface mesh. The texture mapping algorithm base on algorithm in "Texture Synthesis on Subdivision Surfaces" but I haven't implement the blending part yet.

   Here are some snapshots:

   .. image:: images/june_2700.png
     :width: 800 
   
   .. image:: images/june_27_201.png 
     :width: 800
           
.. blogpost::
   :title: 2st Texture mapping - Texture synthesis.
   :author: ktran
   :date: 6-24-2011

   For a long week, I haven't updated my progress. Here is my update for this week:

	* Implemented texture synthesis on the paper "Texture Synthesis on Subdivision Surfaces".
	* I have completed the texture parameterization part and the next step will be  blending texture and generating texture atlas.

   Here is my first result after texture parameterization step:

   .. image:: images/snapshot01.png
     :width: 1000 
   
   .. image:: images/snapshot02.png 
     :width: 1000
           

   The result still have few unexpected mapping and it is not smooth because:

	* I haven't found the optimal solution for selecting the edge which determines the angle (alpha) with vector field (face orientation) calculated on each face. 
	* Texture is not blended yet. I have started working on it.
	



.. blogpost::
   :title: 1st Texture mapping.
   :author: ktran
   :date: 6-13-2011

   This week I have done:

	* Created OBJ_IO class to save a 3D model with the texture as Wavefront (.OBJ) format. 
	* Designed texture data structure for mapping texture onto the mesh. (Just 1 texture material for a mesh right now)
	* Implement a basic texture mapping which is similar to OpenGL placing texture maps onto polygons. The texture coordinates are stored per-vertex. And texture is mapped in the Y plane.

   Here is my result after mapping texture onto a bunny:

   .. image:: images/bunny06.png
     :width: 1000 
   
   .. image:: images/bunny02.png 
     :width: 1000
           

   The result is not perfect because some reasons:

	* The mesh is not smoothed.
	* The texture is simply mapped in the Y plane. Texture coordinates are computed based on X and Z.
	   
	
   And here is my update :ref:`code <obj_io>`

   Next steps:

	* Implement another texture mapping to improve the result.
	* Work on overlapping textures onto a mesh. 
	
	
.. blogpost::
   :title: Working on 3D OBJ file format
   :author: ktran
   :date: 6-7-2011

   As I know, VTK file format is not much useful to represent 3D model with texture. So I decided to write the code to save a polygon mesh (further will be mesh with texture) as OBJ file which can better represent  3D model with texture compared with VTK format. Here is the link for OBJ file format: `<http://en.wikipedia.org/wiki/Wavefront_.obj_file>`_.

   Here is my :ref:`code <obj_io>`
      	
.. blogpost::
   :title: Khai Tran's status update on Week 1
   :author: ktran
   :date: 5-27-2011

   This week I have done:

	* Installed and built PCL library and its dependencies
	* Completed build and test some tutorials using PCL

   In progress:

	* Reading references on texture blending
	* Working on the surface reconstruction API for texture mapping onto a mesh 

   
.. blogpost::
   :title: Khai Tran's first status update
   :author: ktran
   :date: 5-23-2011

   This is the first update to the developer blogs.

   Here's a code snippet sample:

   .. code-block:: c++

      // Hello world
      for (int i = 0; i < 10; ++i)
      {
        cout << "Hello World" << endl;
      }

   And here's an equation:

   .. math::

     \sum^{n}_{i=1}\Phi_{i}


