.. blogpost::
   :title: SimpleTree/SimpleForest - PCL tools for geometrical tree modeling
   :author: hackenberg 
   :date: 11-28-2015

        * **Introduction**


   Geometrical models of trees in the field of computational forestry are nowadays referred to as Quantitative Structure Models (QSMs).  The capability of those models exceeds pure volume estimation of trees. If the volume of a tree is multiplied with density values the above ground biomass (AGB) of a tree can be derived non destructively.

   Instead of being limited to predict the AGB like statistical or voxel based approaches QSMs give also insight into internal biomass distributions of a tree.

   The geometrical building blocks of such models are commonly cylinders fitted into terrrestrial laser scan point clouds.

   SimpleTree is an open source tool to build QSMs from TLS clouds `(SimpleTree homepage) 				<https://simpleforest.org/>`_. The released version is based on PCL 1.8.0.

        * **Update SimpleForest release**

   Since 2019 the here-on presented algorithm is embedded in a forestry plot processing tool named SimpleForest `(SimpleForest homepage) 				<https://simpleforest.org/>`_, `(SimpleForest code base) 				<https://gitlab.com/SimpleForest/pluginSimpleForest>`_.

   .. image:: img/plotAll.jpg
      :width: 300px
      :height: 200px
      :align: center

   .. image:: img/plotZoom.jpg
      :width: 300px
      :height: 200px
      :align: center

   
        * **The cylinder fitting method**

   Typically cylinders are fitted into point clouds with either RANSAC or NLS fitting routines. NLS needs initial estimates of cylinder parameters. Also RANSAC tends to produce errourness cylinders, if no initial segmentation of the point cloud is performed before.

   The method implemented in SimpleTree relies on search spheres to produce an initial tree model which is enriched afterwards.   A sphere with its center point located on the tree skeleton with a radius larger than the underlying represented branch or stem segment will cut the point cloud. All points located on the epsilon neighbourhood of the sphere will represent one or more circular cross sectional areas.

   .. image:: img/sphere1.png
      :width: 200px
      :height: 200px
      :align: center

   Into this subpoint cloud a circle can be fitted with RANSAC 		`(Module sample_consensus) 				<http://docs.pointclouds.org/trunk/group__sample__consensus.html>`_. The sphere center point, the circle center point and the circle radius serve as parameters of a preliminary detected cylinder.

   .. image:: img/sphere2.jpg
      :width: 200px
      :height: 200px
      :align: center

   If the circle is enlarged and transformed to a 3D sphere the procedure can be repeated recursively. 

   .. image:: img/sphere3.jpg
      :width: 200px
      :height: 200px
      :align: center

   To get fast access to the epsilon neighbourhood of a sphere a PCL search structure `(Module octree) 				<http://docs.pointclouds.org/trunk/group__octree.html>`_ is used. All points contained in a search sphere have to be removed as soon as the sphere is utilized to prevent the algorithm to jump back and forth infinitely. For the case the epsilon neighbourhood of a sphere contains multiple cross sectional areas - this occurs in branch junctions - a clustering `(Module segmentation) 				<http://docs.pointclouds.org/trunk/group__segmentation.html>`_ has to be performed. Into each cluster a circle is fitted

   .. image:: img/sphere_split.png
      :width: 200px
      :height: 150px
      :align: center

   the largest circle is processed first (marked in green), while the others are stored in a FIFO queue to be processed later (marked in yellow).

   .. image:: img/sphere_split2.png
      :width: 200px
      :height: 150px
      :align: center

   By the nature of the algorithm a detected cylinders end point will coincide with another cylinders start point and an informatics tree structure can be utilized to store the cylinders. Several non PCL related statistical post processing procedures will adjust the tree structure and the cylinder parameters. The fit quality is also improved by another RANSAC routine. Points are spatially allocated to their nearest cylinder and only on this sub group a 3D RANSAC cylinder fit is performed. Before cylinder RANSAC: 

   .. image:: img/improve1.png
      :width: 200px
      :height: 150px
      :align: center
 
   and after RANSAC:

   .. image:: img/improve2.png
      :width: 200px
      :height: 150px
      :align: center

   The final models are highly accurate and the informatics tree structure allows extraction of diameter classes

   .. image:: img/prunus2.jpg
      :width: 200px
      :height: 200px
      :align: center

   or the extraction of tree components like stem and branches:

   .. image:: img/prunus3.jpg
      :width: 200px
      :height: 200px
      :align: center

   Other possible parameters to extract are described in Hackenberg et al. 2015b.

        * **Crown representation**

   Two comprehensive crown representations are also computed with PCL. The crown can be modelled as a convex hull `(Module surface) 				<http://docs.pointclouds.org/trunk/group__surface.html>`_

   .. image:: img/hull1.jpg
      :width: 200px
      :height: 200px
      :align: center

   or a concave hull

   .. image:: img/hull2.jpg
      :width: 200px
      :height: 200px
      :align: center

   Only the convex hull's volume is for now written in Output files.

        * **ICP to align cloud of different years**

   ICP `(Module registration) 				<http://docs.pointclouds.org/trunk/group__registration.html>`_ can be used to allign scans of the same tree taken at different times, in the example one cloud represents the unprooned tree and the second cloud was taken after the prooning.

   Before:

   .. image:: img/ICP1.jpg
      :width: 200px
      :height: 140px
      :align: center

   After:

   .. image:: img/ICP4.jpg
      :width: 200px
      :height: 140px
      :align: center

   The initial allignment (semi automatic) is described in Hackenberg et al. 2015b. 

        * **References**

   The figures are taken from two peer-reviewed publications presenting the method and the software:

   Hackenberg, J.; Morhart, C.; Sheppard, J.; Spiecker, H.; Disney, M. Highly Accurate Tree Models Derived from Terrestrial Laser Scan Data: A Method Description. Forests 2014, 5, 1069-1105. 

   Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P. SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds. Forests 2015, 6, 4245-4294. 
