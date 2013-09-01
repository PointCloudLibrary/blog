.. blogpost::
   :title: An example blog post
   :author: aaldoma 
   :date: 6-18-2011

   This is a *very* **simple** example blog post.

.. blogpost::
   :title: CVFH
   :author: aaldoma 
   :date: 6-19-2011

   Object recognition and pose estimation for Kinect data trained on synthetic CAD models.

   .. image:: images/cvfh1.jpg	

   Image shows the recognized CAD models overlayed on the point cloud.

.. blogpost::
   :title: Virtual rendering of CAD models as pointcloud...
   :author: aaldoma 
   :date: 6-22-2011

   A couple of new functions into pcl::visualization to obtain pointclouds from CAD models. The first one is renderView() and the second one is renderViewTesselatedSphere(). We can add different geometries to the visualizer and once the virtual scene is created, we can just call the renderView() method to obtain a pointcloud of it seen from the virtual camera. The second one is mainly for object recognition methods that use synthetic models as training data and need to simulate views from the models seen from different viewpoints. Using a more rudimentary version of renderViewTesselatedSphere() is how the pointcloud version from the background robot at pointclouds.org was created :) after merging eighty views of it and applying some filters...

   .. code-block:: cpp

    int main(int argc, char ** argv) {
      vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
      readerQuery->SetFileName (argv[1]);
      vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
      polydata->Update();

      pcl::visualization::PCLVisualizer vis("Visualizer");

      vis.addModelFromPolyData (polydata, "mesh1", 0);

      vis.camera_.window_size ={480,480};
      vis.camera_.pos = {0,5,5};
      vis.camera_.focal = {0,0,0};
      vis.camera_.view = {0,1,0};

      vis.updateCamera();
      vis.resetCamera();

      vis.setRepresentationToSurfaceForAllActors();
      vis.spin();

      //call render in the visualizer to obtain a point cloud of the scene
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
      vis.renderView(256,256, cloud_out);

      pcl::io::savePCDFileASCII("scene.pcd",*cloud_out);
      
      return 0;  
    }

   And the results: you can see the CAD model and a two partial views generated from the same viewpoint with different resolution (the top one at 64x64 and the bottom one at 256x256).

   .. image:: images/virtual_rendering.jpg
