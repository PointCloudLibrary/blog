.. blogpost::
   :title: Setting up Linux tools
   :author: georgeLisca
   :date: 5-25-2011

   Some days ago I decided to move from Windows to Linux. I installed Linux tools  and got in touch with them. Now I'm compiling PCL. Today I learned how to add content to the developer blogs.

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

.. blogpost::
   :title: Starting ICP Registration
   :author: georgeLisca
   :date: 6-04-2011

   My achievements for last week include:

   - Successfully compiling of PCL and it's dependencies under Ubuntu
   - Running some examples
   - Reading references about ICP algorithm
   - Start coding the first point cloud registration prototype which use ICP algorithm

   My goals for next week are:

   - Read references about `GICP <http://stanford.edu/~avsegal/generalized_icp.html>`_
   - Get working the implementation of ICP based registration prototype
   - Get running already existing GICP implementation

.. blogpost::
   :title: ICP Registration
   :author: georgeLisca
   :date: 6-12-2011

   My achievements for last week include:

   - Reading more references about GICP
   - Successfully compiling and running already existing GICP implementation
   - Taking some PCDs from Tyzx stereo camera, but I have weak chances to use them because they contain too much noise

     - We should be able to distinguish some furniture objects from my room: 

       - Left - front view 
       - Right - top view 

   .. image:: images/tyzxOfficePCDFrontView.png
           :width: 500

   .. image:: images/tyzxOfficePCDTopView.png
           :width: 500 

   In progress working: 

   - Testing PCL filters in order to reduce PCD noise
   - Implementing the ICP based registration prototype
   - Waiting for arriving new Kinect sensor, which should happen no latest Wednesday

   My goals for next week are:

   - Read some references about detection / extraction of 3D global features
   - Install the new Kinect sensor
   - Start integrating of already existing GICP implementation into PCL

.. blogpost::
   :title: ICP Registration
   :author: georgeLisca
   :date: 6-19-2011

   My activity from last week includes:

   - Installing and taking first datasets from the new Kinect sensor
   - Testing ICP registration on some datasets
     
     - As input I used two point clouds which differ just a little, but even that unfortunately ICP gets blocked in local minim
     - The computed translation (top view -> right) is  good, but the orientation must be adjusted (front view -> left)

       - left - front view
       - right - top view

   .. image:: images/ICPRegistrationFrontView.png
           :width: 800

   .. image:: images/ICPRegistrationTopView.png
           :width: 800 

   - The computational time of ICP keeps him away of real time registration

   In progress working: 

   - In the next days is GICP turn - it looks promising

     - I have to watch closely already existing implementation

   - Integrating GICP into PCL
   - The registration topic turns to be sensitive

   My goals for next weeks are:

   - Think about the detection and matching of 3D global features

     - I think that 3D global features are the key solution of real time registration

   - Continue integrating GICP into PCL

.. blogpost::
   :title: GICP Registration
   :author: georgeLisca
   :date: 6-22-2011

   Short update regarding the progress of GICP integration :

   - Below are displayed two points clouds registered by GICP
     
     - As inputs, I used two point clouds taken from Kinect sensor.

       - green -> target
       - red -> source
       - blue -> source aligned to target
       - first -> front view
       - second -> top view

     - No initial guess for transformation -> identity matrix

   .. image:: images/GICPRegistrationFrontView.png
           :width: 800

   .. image:: images/GICPRegistrationTopView.png
           :width: 800 

   - Even the two point clouds look little different (target contains much more wall points) GICP succeed to align them
   - The computational time must be improved in order to try to use it for real time registration

.. blogpost::
   :title: GICP integration and a break for diploma presentation
   :author: georgeLisca
   :date: 7-05-2011

   Yesterday it was my diploma presentation. Last week was a crazy week. I had to finish and bind the written book and also refine the final results. I tried to do something for GSoC too.

   GICP current implementation depends on ANN and GSL libraries. PCL 0.7 became independent of ANN and now it relies on FLANN. Therefore this must happens in GICP case too. Also GSL and PCL have incompatible licenses and in order to integrate GICP into PCL it is necessarily to replace GSL functions with Eigen and CMINPACK equivalent functions.

   During last week I studied ANN, FLANN, GSL, Eigen and CMINPACK documentations. Regarding to ANN and GSL functions used by GICP I proposed their equivalences from FLANN, GSL and CMINPACK. You can find a draft which is open to new suggestions :download:`here <./files/ANN2FLANN_GSL2EigenCMINPACK.txt>` and if you would like to add something please post on PCL blog.

   My goals for next days are to apply the listed equivalences and add GICP to PCL without ANN and GSL.

   Thank you for understanding the delay introduced by my graduation!

.. blogpost::
   :title: Debugging GICP and visualization of registration process
   :author: georgeLisca
   :date: 7-20-2011

   It passed a lot of time from my last post. In the passed days I tested GICP implementation committed by Nizar and I tried to display the registration process of two point clouds. 

   In order to visualize the registration process I implemented a callback function. What remained is the synchronization of registration thread and visualization thread which is in progress now.

   For GICP I still have to test it more deeply. Now I am digging into code and I hope to get GICP running as soon as possible.

.. blogpost::
   :title: First version of registration visualizer
   :author: georgeLisca
   :date: 7-26-2011

   The first version of Registration Visualizer is ready and can be found in the visualization module. In order to be used, it requires to be set:

   - the registration method whose intermediate steps will be rendered
   - the number of correspondence pairs between intermediate point cloud and target point cloud. These pairs will be displayed as lines. Note that if the maximum correspondences is not set then the entire set of correspondences will be displayed and the most probably the visualization will freeze while the registration algorithm will progress. For the future I am intending to speed up the correspondence lines remove / update sequence.

   
   During the registration progress is opened a visualization window which has two ports. In the left port are displayed:

   - the initial positions of source (red) and target (blue) point clouds .

   In the right port are displayed:

   - the target point cloud (blue)
   - intermediate positions of source point cloud (yellow) obtained after applying the estimated rigid transformation.
   - the correspondences between the points of intermediate point cloud and the points of the target point cloud used for the estimation of the rigid transformation. These correspondences are represented with random colored lines.
   
   .. image:: images/RegistrationVisualizer.png
           :width: 800


   .. image:: images/RegistrationVisualizerBun04.png
           :width: 800
   
   Currently the RegistrationVisualizer class uses PCLVisualizer. Once I will succeed to run PCLVisualizer display in a separate thread, RegistrationVisualizer will be able to inherit PCLVisualizer.

   Here is some snipped code showing how to use this class:

   .. code-block:: c

     // Create the registration visualizer which will display points of type pcl::PointXYZ
     pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> registrationVisualizer;
     
     // Set the maximum number of correspondences which will be displayed. Note that if
     // this number is greater than 30 the visualization thread is slowed down and not all
     // intermediate results will be displayed
     registrationVisualizer.setMaximumDisplayedCorrespondences (30);
     
     // Start the display window
     registrationVisualizer.startDisplay();
     
     // Prepare the registration method to be visualized
     
     // ...
     
     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
     icp.setInputCloud(cloud_in);
     icp.setInputTarget(cloud_out);
     
     // ...
     
     // Register the registration method
     registrationVisualizer.setRegistration (icp);
     
     // Start running the registration method
     icp.align (source_aligned);
     
     // Print the registration results
     
     // ...
     
     // Use a loop to enable examination of final results
     while (true)
     {
       boost::this_thread::sleep (boost::posix_time::seconds (1));
     }
     
   Also I added a test unit to visualization tools from visualization module. I run it for *trunk/test/bun0.pcd* and *trunk/test/bun4.pcd* and obtained the second print screen.

.. blogpost::
   :title: The convergence of GICP based on Levenberg–Marquardt optimizer
   :author: georgeLisca
   :date: 8-14-2011

   I run many times already existing GICP implementation, but unfortunately I couldn't find out why the algorithm doesn't converge. This made me think that this happens because of some numerical issues infiltrated into the new code. Needing a ground truth for numerical comparisons we decided to start from original implementation based on ANN and GSL and compare the results of each intermediate algorithm's step from original implementation and PCL ported version. In what fallows I will try to summarize what I did and my conclusions:

   **1.** remodeled the original implementation in order to get it closer to PCL "registration" for an easier future integration
    
    - Break some data structures which made the data access harder and change the computation flow.

   **2.** replaced ANN searching with FLANN searching

    - I compared ANN vs FLANN results required by the computation of covariance matrix for each point and by the correspondence point searching. They are almost identical. For covariance matrices computation are used 20 neighbors. Both ANN and FLANN returned same neighbors. This allows me to go deeply with future comparisons because I was sure that I am comparing same points but retained in different containers (original and PCL). Notice here that this is not happening all the times, but in not happening case I tried to filter both point clouds and then it worked.

   **3.** reimplemented matrix and vector operations based on GSL with their Eigen equivalent. The GSL operations are using original point cloud containers while Eigen operations are using PCL point cloud containers.

    - I compared each intermediate result and all the differences between them is less than 1e-10, which I think is acceptable for beginning.

   **4.** reimplemented functions necessary for the evaluation of the function to be optimized and function's Jacobian for the pairs of corresponding points. Also here the GSL operations are using original point cloud containers while Eigen operations are using PCL point cloud containers.

    - Additionally from original implementation I called both new functions (evaluation function and it's Jacobian). After each call, I computed the differences between the values returned by the new implemented functions and their original versions. These differences are less than 1e-10 in both cases: function evaluation and jacobian evaluation.

   **5.** replaced the original optimization based on GSL with the new one based on LM. CMINPACK offers multiple `variants <http://www.math.utah.edu/software/minpack/>`_ for minimize the sum of the squares of M nonlinear functions in N variables.

    - **Using LMDER1**

     - Requires to be hand-computed both the function values and the function Jacobian.

     - In the first test scenario, the GICP optimization loop, additionally to original optimization based on GSL, optimizes the same transformation matrix using LMDER1 and the same set of correspondences. The numerical results are quite `different <http://svn.pointclouds.org/gsocweb/source/georgeLisca/files/gicp_original_optimized_by_LM.html>`_. The same transformation optimized by GSL converges to different values if it is optimized by LMDER1. 

     - In the second test scenario, the GICP optimization loop separately computes the correspondences for GSL optimization and LMDER1 optimization. As in previous scenario the `numerical results <http://svn.pointclouds.org/gsocweb/source/georgeLisca/files/gicp_parallel_convergence.html>`_ are quite different. Additionally it seems that LMDER1 get blocked into a local minim.
     
     - In conclusion I think that `LMDER1 convergence parameters <http://www.math.utah.edu/software/minpack/minpack/lmder1.html>`_ represent a critical point. I am intending to test in parallel the GSL optimization vs. LMDER1 optimization in order to verify the robustness of their convergences.

     - After listening Nizar suggestions I think that I tried to use LMDER1 somehow in and improper way because of reasons related the GICP mathematic model.

    - **Using LMDIF1**

     - Requires to be hand-computed only the function values. The function Jacobian is computed by a forward-difference approximation.

     - As in the second scenario of LMDER1, the GICP optimization loop separately computes the correspondences for GSL optimization and LMDIF1 optimization. The results are almost the same but still the translation components significantly `differ <http://svn.pointclouds.org/gsocweb/source/georgeLisca/files/gicp_parallel_convergence_lmdif1.html>`_.

    - **Using LMSTR1**

     - Requires to be hand-computed the function values and the Jacobian's rows.

     - As in the case of LMDIF1, the optimization converges to different `values <http://svn.pointclouds.org/gsocweb/source/georgeLisca/files/gicp_parallel_convergence_lmstr1.html>`_ than original implementation.



   
