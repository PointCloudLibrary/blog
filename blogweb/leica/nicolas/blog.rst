.. blogpost::
   :title: Plane based compression of point clouds
   :author: nicolas
   :date: 07-11-2013


   Basically the approach fits a set of geometric primitives approximating the input point cloud. Although higher order proxies such as spheres or cylinders could be considered we focus here on planes. First, most man made environments are essentially piecewise planar. Second, curved objects allow piecewise linear approximations at the  cost of a higher number of proxies.


   Principles of our approach :
   i) Estimation of an oriented plane is done via a weighted PCA incorporating closeness of points and and local complexity using curvature information obtained from the eigenvalues of the PCA. We define an input point to be inlier whenever its distance is less then some threshold, it is visible on the positive side of the plane and the angle between the line of sight and the normal is bounded by some threshold.
   ii) A proxy consists of an oriented 3D plane and a connected subset of inliers encoding the spatial extent. Here two methods will be tested. First a RANSAC based approach and second a region growing approach.


   Currently I am implementing the region growing extraction. First results and images will be reported next week.

      

   We expect a very high compression with the proposed approach. Since a single plane can approximate a high number of points. Of course the compression is lossy and one cannot expect recovering the original point cloud. I further  believe that the integration of colour information can be done without to much overhead. 
