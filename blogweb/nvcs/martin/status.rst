My status updates
=================

.. blogbody::
  :nr_days: 60
  :author: martin

.. blogpost::
  :title: Design considerations and a short example
  :author: martin
  :date: 07-28-2012

  **Underlying containers**

  The first important design choice for the mesh implementation is the data structure in which the individual mesh elements (vertices, half-edges, faces) are stored. The three most suitable containers are the vector, deque and list (arrays are only good for meshes with a known size). The list has the advantage that pointers to its elements are not invalidated when elements are inserted or erased. This makes it possible to store pointers (iterators) to the connected neighbors directly in each mesh element. No further steps are required if elements are removed from the mesh. The disadvantage is a slower traversal through the list which is a very common operation, for example when the mesh is transformed or rendered.

  The vector and deque provide very similar functionality while using a different memory management. For both it is not possible to directly store iterators in the mesh elements because these are invalidated when elements are inserted or erased in the middle of the container. Therefore it is necessary to exchange the iterators with indices to the connected neighbors. Since indices are not dereferencable one has to call a method of the mesh to access the element at the given index.

  Another problem is that if one element is removed in the middle of the container all subsequent indices have to be shifted and all elements that store an index to a shifted index have to be changed accordingly. This is a very time consuming operation. Therefore it is better to mark the mesh elements as deleted instead of erasing them. When the user has finished all desired deletions a clean-up method has to be called that adapts all indices in one run.

  In experiments that I did for my diploma thesis I noticed that the list is much slower compared to a vector for my intended application. I have to do further experiments comparing the vector with the deque so for now I will stick with the vector.

  **Topology & geometry**

  Although in a mesh the geometry is stored together with the topology I want to keep them separated for the implementation. I understand the mesh class as a container itself (a quite complex one) which provides methods for accessing and changing the topology while the user is responsible for the geometry: The mesh class should make no assumptions on, for example, the existence of a point position, normal, color, or any other possible data because they are irrelevant for the topology. It is even possible to define a mesh without any geometry at all. This might not be very useful for an application but it has the advantage that the definition of arbitrary user data is very easy::

    // Define the mesh traits
    struct MeshTraits
    {
      typedef pcl::PointXYZ         VertexData;
      typedef pcl::geometry::NoData HalfEdgeData;
      typedef int                   EdgeData;
      typedef pcl::Normal           FaceData;

      typedef boost::false_type IsManifold;
    };

    // Define the mesh
    typedef pcl::geometry::PolygonMesh <MeshTraits> Mesh;
    Mesh mesh;

  The example defines a non-manifold polygon mesh which stores a PointXYZ for the vertices, nothing for the half-edges, an integer for the edges and a normal for the faces. The data is stored in a pcl::PointCloud <T> and can be retrieved by the following methods::

      Mesh::VertexDataCloud&   cloud = mesh.getVertexDataCloud ();
      Mesh::HalfEdgeDataCloud& cloud = mesh.getHalfEdgeDataCloud (); // Note: Empty for the example above
      Mesh::EdgeDataCloud&     cloud = mesh.getEdgeDataCloud ();
      Mesh::FaceDataCloud&     cloud = mesh.getFaceDataCloud ();

  The topology is accesed through the mesh. Each method must give the respective index::

      // Vertex connectivity
      HalfEdgeIndex ind = mesh.getOutgoingHalfEdgeIndex (vertex_index);
      HalfEdgeIndex ind = mesh.getIncomingHalfEdgeIndex (vertex_index);

      // Half-edge connectivity
      VertexIndex   ind = mesh.getTerminatingVertexIndex (half_edge_index);
      VertexIndex   ind = mesh.getOriginatingVertexIndex (half_edge_index);
      HalfEdgeIndex ind = mesh.getOppositeHalfEdgeIndex  (half_edge_index);
      HalfEdgeIndex ind = mesh.getNextHalfEdgeIndex      (half_edge_index);
      HalfEdgeIndex ind = mesh.getPrevHalfEdgeIndex      (half_edge_index);
      FaceIndex     ind = mesh.getFaceIndex              (half_edge_index);
      FaceIndex     ind = mesh.getOppositeFaceIndex      (half_edge_index);

      // Face connectivity
      HalfEdgeIndex ind = mesh.getInnerHalfEdgeIndex (face_index);
      HalfEdgeIndex ind = mesh.getOuterHalfEdgeIndex (face_index);

  **Circulators**

  The mesh provides several circulators:

  * VertexAroundVertexCirculator (clockwise)
  * OutgoingHalfEdgeAroundVertexCirculator (clockwise)
  * IncomingHalfEdgeAroundVertexCirculator (clockwise)
  * FaceAroundVertexCirculator (clockwise)
  * VertexAroundFaceCirculator (counterclockwise)
  * InnerHalfEdgeAroundFaceCirculator (counterclockwise)
  * OuterHalfEdgeAroundFaceCirculator (counterclockwise)
  * FaceAroundFaceCirculator (counterclockwise)

  Incrementing the circulators around the vertex circulates clockwise while incrementing the circulators around the face circulates counterclockwise when looking at the mesh from the outside. The reason is that these operations don't need to access the previous half-edge which might become an optional part of the mesh in the future. The circulators around the face can also be used to move along the boundary of the mesh.

  Circulators are different compared to iterators in the aspect that they don't have a distinct begin and end position. It is valid to circulate through a sequence endlessly. Usually one wants to access all elements only once. This can be achieved by the following procedure::

    // NOTE: {...} stands for any of the circulators and (...) stands for the respective input

    {...}Circulator circ     = mesh.get{...}Circulator (...);
    {...}Circulator circ_end = circ;

    do
    {
      // do something
    } while (++circ != circ_end);

  **Example**

  I added a short example to `pcl-trunk/examples/geometry/example_half_edge_mesh.cpp <http://svn.pointclouds.org/pcl/trunk/examples/geometry/example_half_edge_mesh.cpp>`_. It defines a non-manifold mesh and creates a simple manifold topology which is traversed in different ways. Then it deletes two faces resulting in a non-manifold topology. If you change to a manifold mesh further faces are deleted in order to keep the mesh manifold.

  Please let me know if you find any bugs. And since the implementation of the mesh is still evolving I am also very happy about comments regarding the API.

.. blogpost::
  :title: Representation of manifold and non-manifold meshes
  :author: martin
  :date: 07-14-2012

  In this blog post I will talk about the representation of a mesh and how it is affected by the manifold property. Probably the most convenient way for the user to generate a mesh is to first add the vertices and then connect them by faces while the edges are created automatically. This way the mesh class can ensure that it holds only valid data. For example, if the user tries to add the same face twice then only the first insertion must be executed. Unconnected (isolated) vertices can be removed afterwards without changing the rest of the topology. The face is specified by a sequential list of vertices where the vertices are connected in a closed loop.

  **Manifold mesh**

  In a manifold mesh faces may be added only between boundary half-edges. We can make use of the property that each boundary vertex in a manifold mesh has exactly one incoming and one outgoing boundary half-edge. If a vertex does not lie on the boundary then all incoming and outgoing half-edges are not on the boundary as well. Therefore it is useful to ensure that the outgoing half-edge of a boundary vertex is a boundary half-edge. This way we can easily check if a face may be added without the need to circulate through all half-edges that are connected to each vertex in the new face. We can also check in constant time if an edge between two vertices is new or already contained in the mesh.

  There are two possible problems that might be a concern for certain applications. The first problem is only related to a triangle mesh: If there are several separate components in the mesh then it is not possible to join them by only one triangle without avoiding non-manifold vertices (illustrated in the image below): The dashed triangle may not be added because the encircled vertex would become non-manifold. This problem can be circumvented by adding triangles in pairs in such cases. If this is not desired then one should consider using a quad or polygon mesh or a mesh that can handle non-manifold vertices.

  .. image:: images/combining.png
    :alt: Combining two separated components
    :width: 855px
    :scale: 60
    :align: center

  The second problem appears when faces are removed from the mesh: The deletion of a face can result in non-manifold vertices. For example, if face number 5 is deleted from the mesh in the next image (left) the encircled vertex becomes non-manifold. This problem can be solved by deleting the neighboring faces until the vertex becomes manifold again (right): Starting from an edge in the deleted face (number 5) we circulate around the vertex and remove all faces (number 4 and 3) until the boundary is reached . This however, can result in further non-manifold vertices (encircled) which have to be removed iteratively until the whole mesh becomes manifold again. These operations are part of the implementation of the manifold mesh so the user does not have to worry about them. Again, if this behavior is not desired one should use a mesh that can handle non-manifold vertices.

  .. image:: images/removing_faces.png
    :alt: Removing faces
    :width: 1331px
    :scale: 60
    :align: center

  **Mesh with non-manifold vertices**

  There was a very nice description how to handle non-manifold vertices online however the site is currently down (please let me know if you see it online again). Although my implementation of the mesh is different in several aspects (mainly due to a different API) the basic idea is the same.

  * http://www.cgafaq.info/wiki/Geometric_data_structures

  In a mesh with non-manifold vertices it can be no longer guaranteed that every boundary vertex has exactly one incoming and outgoing boundary half-edge. However it is still useful to store one of the boundary half-edges as the outgoing half-edge for boundary vertices. This way it is possible to check if a vertex is on the boundary or not but it becomes necessary to circulate around all neighboring half-edges in order to check if two boundary vertices are already connected.

  The problem with non-manifold vertices is that the current connectivity around them might be incompatible with the insertion of a new face. However all possible consistent cases are topologically correct and the mesh does not know beforehand which faces are meant to be inserted next. Therefore it is necessary to make the half-edges adjacent before the insertion of a new face. The next image gives an example how the half-edges around a non-manifold vertex can be reconnected (the connectivity between the half-edges is represented as a continuous line). The insertion of a new face between the half-edges 3-0 and 0-2 (image on the left), for example can be done without any further intervention. However for an insertion of a new face between the half-edges 1-0 and 0-8 it becomes necessary to reconnect the half-edges around vertex 0 in such a way that the half-edges 1-0 and 0-8 become adjacent, as shown in the image on the right. Although in this case the operation resulted in other non-adjacent half-edges these can be corrected during the insertion of further faces.

  .. image:: images/make_adjacent.png
    :alt: Make adjacent with consistent orientation
    :height: 525px
    :scale: 60
    :align: center

  The next image shows an invalid configuration that happens for certain situations. For example if we would try to insert a face between the edges 3-0 and 0-4 then the half-edges around vertex 0 would be reconnected in such a way that the closed loop of half-edges is split up into two loops. The problem is that vertex 0 stores only one outgoing half-edge which can point into only one of the loops. The other loop is then "unknown" by the vertex because it can't be reached anymore by circulating around the vertex. This configuration is not supported by the half-edge data structure and must be avoided. Luckily all of these operations can be hidden inside the mesh implementation so the user does not have to worry about them.

  .. image:: images/make_adjacent_invalid.png
    :alt: Non-representable non-manifold vertex.
    :height: 525px
    :scale: 60
    :align: center

  **Mesh with non-manifold edges**

  The linked page describes how the half-edge data structure can be modified in order to handle non-manifold edges and non-orientable meshes:

  * http://jcae.sourceforge.net/amibe-doc/org/jcae/mesh/amibe/ds/AbstractHalfEdge.html

  The basic idea is to introduce virtual triangles between non-manifold edges. Virtual triangles are not exposed to the user but they allow keeping the mesh manifold internally. They also allow modeling non-orientable surfaces.

  My implementation currently does not support virtual triangles. I think that the implementation of a non-orientable mesh with non-manifold vertices and edges would become really messy if it is not done right and I would like to test the mesh with non-manifold vertices first before continuing with more elaborate representations. Furthermore it is also worth considering other mesh data structures which might be suited better for representing arbitrary meshes. If there is a need for it I can have a more in-depth look into virtual triangles after my code sprint. But for now I want to keep things simple. In my next post I will give a short API overview of my current implementation and add everything to the trunk.

.. blogpost::
  :title: Half-edge data structure
  :author: martin
  :date: 07-02-2012

  The half-edge data structure is a popular representation for a mesh in which each edge is split up into two half-edges with opposite directions. The main topology information is stored in the half-edges. The vertices and faces are not explicitly connected to each other because they can be reached through the respective half-edges. If one wants to find, for example all faces to which a vertex is connected to then it is necessary to go through all neighboring half-edges and refer to the faces through them. Compared to explicitly storing references from each vertex or face to all of their immediate neighbors this has the advantage that the storage space per vertex, half-edge and face is constant while it is still possible to access the neighborhood without an exhaustive search through the hole mesh. Another advantage is that the orientation of the faces is represented in the surrounding half-edges.

  **Half-edge connectivity:**

  .. image:: images/half_edge.png
    :alt: Half-edge
    :width: 845px
    :scale: 60
    :align: center

  The image above illustrates the connectivity information related to a specific half-edge (red), which explicitly stores indices to

  * the opposite half-edge (also called pair or twin)
  * the next half-edge
  * the previous half-edge (this is actually not a requirement of the half-edge data structure but it is useful for the implementation)
  * the terminating vertex
  * the face it belongs to

  Other elements in the mesh can be reached implicitly (gray)

  * originating vertex = opposite half-edge -> terminating vertex
  * opposite face = opposite half-edge -> face

  By using a convention in the implementation even the opposite half-edge can be accessed implicitly in constant time and its storage space can be saved: If half-edges are always added in pairs then their respective indices come right after each other. Given a half-edge index we can then derive the opposite index simply by checking if it is even or odd.

  **Vertex connectivity:**

  .. image:: images/vertex.png
    :alt: Vertex
    :width: 580px
    :scale: 60
    :align: center

  The image above illustrates the connectivity information related to a specific vertex (red), which explicitly stores an index to

  * one of its outgoing half-edges

  The corresponding incoming half-edge can be reached implicitly (gray)

  * incoming half-edge = outgoing half-edge -> opposite half-edge

  Although only one of the outgoing half-edges is stored the others can be reached by *circulating* around the vertex. Given any of the outgoing half-edges the next outgoing half-edge can be reached by

  * outgoing half-edge = outgoing half-edge -> previous half-edge -> opposite half-edge (counter-clockwise)
  * outgoing half-edge = outgoing half-edge -> opposite half-edge -> next half-edge (clockwise)

  This procedure has to be continued until a full loop around the vertex has been completed. Similarly it is possible to use the incoming half-edge for the circulation

  * incoming half-edge = incoming half-edge -> opposite half-edge -> previous half-edge (counter-clockwise)
  * incoming half-edge = incoming half-edge -> next half-edge -> opposite half-edge (clockwise)

  With a slight modification this allows us to access the vertices or faces as well (only shown in counter-clockwise order)

  #. vertex = outgoing half-edge -> terminating vertex
  #. outgoing half-edge = outgoing half-edge -> previous half-edge -> opposite half-edge

  or

  #. face = outgoing half-edge -> face
  #. outgoing half-edge = outgoing half-edge -> previous half-edge -> opposite half-edge

  This procedure is continued until a full loop is completed. Using these basic operations it is possible to find all neighbors around a vertex, which is also called the *one-ring neighborhood*.

  **Face connectivity**

  .. image:: images/face.png
    :alt: Vertex
    :width: 421px
    :scale: 60
    :align: center

  The image above illustrates the connectivity information related to a specific face (red), which explicitly stores an index to

  * one of the inner half-edges

  The corresponding outer half-edge can be reached implicitly (gray)

  * outer half-edge = inner half-edge -> opposite half-edge

  As for the vertices the other inner half-edges can be reached by circulating around the face

  * inner half-edge = inner half-edge -> next half-edge (counter-clockwise)
  * inner half-edge = inner half-edge -> previous half-edge (clockwise)

  Each outer half-edge or vertex is referenced from its corresponding inner half-edge.

  **Boundaries**

  Boundaries can be represented by an invalid face index in the half-edge. It is important that *both* half-edges are kept in the mesh because else it would be no longer possible to access all neighbors. A half-edge with an invalid face index is called *boundary half-edge*. Accordingly, a vertex that is connected to a boundary half-edge is called *boundary vertex*.

  One can circulate around the boundary with

  * boundary half-edge = boundary half-edge -> next half-edge (clockwise)
  * boundary half-edge = boundary half-edge -> previous half-edge (counter-clockwise)

  This is the same as circulating through the inner half-edges of a face with the only difference that the direction (clockwise, counter-clockwise) is reversed.

  If the mesh has a boundary then one has to be very careful when accessing the faces through the half-edges in order to avoid dereferencing an invalid face index.

  **Manifoldness**

  I will talk about manifoldness in my next blog post.

.. blogpost::
  :title: Mesh basics
  :author: martin
  :date: 06-27-2012

  It's been a while since my last blog post. I have been moving into a new home and started with a job. I hope I can find the right balance between this and the code sprint. Before starting with the half-edge data structure I would like to introduce a few mesh basics, just to make the terminology clear and consistent. Here are two links that should provide more in-depth information:

  * http://graphics.stanford.edu/courses/cs468-12-spring/ (Schedule & Assignments)

  * http://www.pmp-book.org/

  A polygon mesh is a data structure where the *geometry* is stored along with the *topology* (connectivity information). The points or nodes inside a mesh are called *vertices* and hold the geometry. An *edge* is a connection between two vertices. Three or more edges that form a closed loop are called *face*, i.e. a triangle, quadrilateral, pentagon, ..., polygon. The edges and faces define the topology. A mesh that consists only of triangles is called *triangle-mesh*, a mesh that consists only of quadrilaterals is called *quad-mesh* and so on.

  The mesh has no *boundary* if each edge is connected to exactly two faces. If any edge is connected to only one face then the mesh has a boundary, shown in the next image (taken from the Geometry Processing Algorithms lecture, `Basics <http://graphics.stanford.edu/courses/cs468-12-spring/LectureSlides/02_Basics.pdf>`_).

  .. image:: images/boundary.png
    :alt: Mesh with boundary
    :width: 640px
    :scale: 60
    :align: center

  If any edge is connected to more than two faces (left) or if two or more faces are connected by only one vertex (middle, right) then the mesh becomes *non-manifold*.

  .. image:: images/non_manifold.png
    :alt: Non-manifold configurations
    :width: 1146px
    :scale: 60
    :align: center

  Another property to consider when working with meshes is the orientation of the faces. If the faces in the mesh have a consistent orientation (all clockwise or all counter-clockwise) then the mesh is *orientable* (left). A non-orientable configuration is shown in the right. An example for a non-orientable surface is the `Möbius strip <http://en.wikipedia.org/wiki/Moebius_strip>`_.

  .. image:: images/orientation.png
    :alt: Orientability
    :width: 885px
    :scale: 60
    :align: center

.. blogpost::
  :title: Hello NVCS
  :author: martin
  :date: 06-09-2012

  Hi, this is my first blog post for NVCS so I would like to introduce myself shortly. My name is Martin Sälzle and I recently graduated at the Technische Universität München. I have been working with the Kinect for my diploma thesis on 3D in-hand scanning of small, texture-less objects. The work is based mainly on [Weise2009]_ with some personal flavors. I also concentrated on the implementation without performing an active loop closure because I first wanted to get the registration pipeline as robust as possible. A bundle adjustment would surely help in the future. Here is the scanning setup (left) and some results (right):

  .. image:: images/kinect_in_hand_scanner.jpg
    :alt: Kinect in-hand scanner & scanned objects
    :width: 2880px
    :scale: 33
    :align: center

  For an evaluation I downloaded the model of the Stanford Bunny (left) and 3D printed it at `www.shapeways.com <http://www.shapeways.com>`_ (middle). I scanned the real world object in again with the Kinect (right) and compared it to the original model.

  .. image:: images/stanford_bunny.jpg
    :alt: Stanford Bunny
    :width: 2880px
    :scale: 33
    :align: center

  I registered the scanned model to the original model and colored the original model according to the scanning error (Euclidean distance between corresponding points). A histogram is shown in the right.

  .. image:: images/evaluation.png
    :alt: Evaluation
    :width: 1920px
    :scale: 45
    :align: center

  My first task is to integrate the code into PCL. I am currently implementing the `half-edge data structure <http://en.wikipedia.org/wiki/Doubly_connected_edge_list>`_ because I reached the limits of the `face-vertex mesh <http://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes>`_ I used in my thesis. In the next post I will talk about that in more detail. Due to licensing issues we can't use `CGAL <http://www.cgal.org>`_ or `OpenMesh <http://www.openmesh.org>`_ in PCL.

  .. [Weise2009] Weise, Wismer, Leibe, Van Gool. In-hand scanning with online loop closure. In ICCV 2009, p.1630--1637.
