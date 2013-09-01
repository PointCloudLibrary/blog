.. blogpost::
   :title: Final report and code committed
   :author: aaldoma 
   :date: 9-28-2012

   This week I committed the code to perform face detection in PCL trunk and wrote the final report summarizing the work done as well as how to use the module.

   .. raw:: html 

      <center><iframe src="http://docs.google.com/viewer?url=http://svn.pointclouds.org/hrcsweb/source/aaldoma/files/final_report.pdf&amp;embedded=true" style="border: none;" height="400" width="800"></iframe></center>
   
.. blogpost::
   :title: Some things on object recognition
   :author: aaldoma 
   :date: 9-4-2012

   In the last months I have been working on a new meta-global descriptor called OUR-CVFH (http://rd.springer.com/chapter/10.1007/978-3-642-32717-9_12) that as you can imagine is an extension to CVFH which is in its turn an extension to VFH (i am not very original at dubbing things). I have also commited some tools and pipelines into pcl/apps/3d_rec_framework (still unstable and not very well documented). 

   Tomorrow we are having a TV demo in the lab where we are showing recent work on recognition/classification and grasping of unknown objects. So, as happens usually, I had to finish some things for it and I would like to show how with OUR-CVFH is it possible to do scale invariant recognition and 6DOF pose estimation + scale. The training objects are in this case downloaded from 3d-net.org (unit scale, whatever unit is) and they usually do not fit the test objects accurately.

   .. image:: img/scale_invariant-our-cvfh.png
      :align: center

   Apart from this, I have also extended OUR-CVFH to use color information and integrate in the histogram. Basically, the reference frame obtained in OUR-CVFH is used to create color distributions depending on the spatial distribution of the points. To test the extension, I did some evaluations on the Willow Garage ICRA 11 Challenge dataset obtaining excellent results (about 99% precission and recall). The training dataset is composed of 35 objects and the test set with 40 sequences totalling 435 object instances. A 3D recognition pipeline method based on SIFT (keypoints projected to 3D) obtains about 70% in such a dataset (even though the objects present texture most of the time). Combining SIFT with SHOT and merging the hypotheses together, gets about 84% and the most recent paper on this dataset (Tang et al. from ICRA 2012) obtains about 90% recall at 99% precission. If you are not familiar with the dataset, here are some screenshots and the respective overlayed recognition and pose estimation.

   .. image:: img/T_02_01.png
      :align: center
   .. image:: img/T_05_01.png
      :align: center
   .. image:: img/T_06_01.png
      :align: center

   The color extension to OUR-CVFH and the hypotheses verification stage are not yet in PCL but I hope to commit them as soon as possible, probably after ICRA deadline and before ECCV. You can find the Willow ICRA challenge test dataset in PCD format at http://svn.pointclouds.org/data/ICRA_willow_challenge.

.. blogpost::
   :title: Random Forest changes
   :author: aaldoma 
   :date: 9-2-2012

   I am back from "holidays", conferences, etc. and today I started dealing with some of the concerns I pointed out in the last email, mainly regarding the memory footprint required to train. The easiest way to deal with that is to do bagging on each tree so that the training samples used at each tree are loaded before start and dismissed after training a specific tree. I implemented that by adding an abstract DataProvider class to the random forest implementation which is specialized depending on the problem. Then, when a tree is trained and a data provider is available, the tree requests training data to the provider, trains and discards the samples.

   I also realized that most of the training data I have for faces, contains a lot of NaNs except of the parts containing the face itself and other parts of the body (which are usually localized in the center of the image). So, to reduce further the data in memory, the specialization of the data provider crops the kinect frames discarding regions with only NaN values.

   With this two simple tricks, I am able to train each tree in the forest with 2000 random training samples (from each sample 10 positive and 10 negative patches are extracted) requiring only 3GB of RAM. In case that more training data is needed or the training samples become bigger, one might use a similar trick to design an out-of-core implementation where the data is not requested at tree level but at node level and only indices are kept into memory.

   I also found some silly bugs and now I am retraining... let's see what comes out.

.. blogpost::
   :title: Progress on head detection and pose estimation (II)
   :author: aaldoma 
   :date: 7-17-2012

   I have continued working on the face detection method and added the pose estimation part, including the clustering step mentioned on my last post. See the video for some results from our implementation (at the beginning is a bit slow due to the video recording software, then it gets better).

   .. raw:: html 

      <center><iframe width="420" height="315" src="http://www.youtube.com/embed/AhD6Z7JBBdo" frameborder="0" allowfullscreen></iframe></center>

   I fixed several bugs lately and even though the results start looking pretty good I am not yet completely satisfied. First I was facing some problems during training regarding what to do with patches where the features are invalid (division by zero), I ended up using a tree with three branches and that worked better although I am not yet sure which classification measure should be used then (working on that). The other things are: use of normal features which can be computed very fast with newer PCL versions on organized data and a modification on the way the random forest is trained. Right now it requires all training data to be available in memory (one integral image for each training frame or even four of them if normals are used). This ends up taking a lot of RAM and restricts the amount of training data that can be used to train the forest.

.. blogpost::
   :title: Progress on head detection and pose estimation
   :author: aaldoma 
   :date: 6-22-2012

   Hi again! 

   Has been some time since my last post. Was on vacation for some days, then sick and afterwards getting all stuff done after the inactivity. Anyway, I have resumed work on head detection + pose estimation reimplementing the approach from Fanelli at ETH. I implemented the regression part of the approach so that the trees provide information about the head location and orientation and did some improvements on the previous code. I used the purity criteria in order to activate regression which seemed the most straightforward.

   .. image:: img/face_votes.png
      :align: center

   The red spheres show the predicted head location after filtering sliding windows that reach leaves with high variance and therefore, are not accurate. As you can see there are several red spheres at non head locations. Nevertheless, the approach relies on a final bottom-up clustering to isolate the different heads in the image. The size of the clusters allows to threshold head detections and eventually, remove outliers.

   I hope to commit a working (and complete) version quite soon together with lots of other stuff regarding object recognition.

.. blogpost::
   :title: Decision Forests and Depth Image Features
   :author: aaldoma 
   :date: 5-25-2012

   Hi again! 

   Because of ICRA and the preparations for the conference I was quite inactive the last couple of weeks. This week I had to finish some school projects and yesterday I resumed work on face detection. Because the BoW approach using SHOT was "slow", I decided to give it a try to Decision Forests and Features extracted directly from the depth image (similar to http://www.vision.ee.ethz.ch/~gfanelli/head_pose/head_forest.html). Although not yet finished, I am already able to generate face responses over the depth map and the results look like this:

   .. image:: img/rf_faces.png
      :align: center

   Basically, the map accumulates at each pixel how many sliding windows with a probability of being a face higher than 0.9 include each specific pixel. This runs in real time thanks to the use of integral image for evaluating the features. For the machine learning part, I am using the decision forest implementation available in the ML module of PCL.

.. blogpost::
   :title: 3D Face Detection
   :author: aaldoma 
   :date: 5-02-2012

   Here are some results regarding 3D face detection:

   .. raw:: html 

      <center><iframe width="420" height="315" src="http://www.youtube.com/embed/6YhPRTK_d84" frameborder="0" allowfullscreen></iframe></center>

   To speed things up I switched to normal estimation based on integral images (20fps). I found and fixed some bugs regarding the OMP version of SHOT (the SHOT description was being OMP'ed but the reference frame computation was not) and now I am getting almost 1fps for the whole recognition pipeline. Was not convinced about the results with RNN so I decided to give Kmeans a try (no conclusion yet regarding this).

   Code is far from perfect (there are few hacks here and there) but the results start looking decent. Regarding further speed-ups, we should consider moving to GPU, however, porting SHOT might not be the lightest task. Next steps would be stabilizing the code, do some commits and then will see if we go for GPU or move forward to pose detection and hypotheses verification.

.. blogpost::
   :title: Face Detection with Bag Of (Geometric) Words 
   :author: aaldoma 
   :date: 4-24-2012

   Yesterday and today, I have been working on face detection and went ahead implementing a Bag Of Words approach. I will summarize briefly the steps I followed:

   1. Generate some synthetic training data of faces (as explained in past posts) and some data of other kind of objects.
   2. Computed a 3D feature (I tested FPFH and SHOT features) on the generated views.
   3. Codebook generation. I implemented a naive version of RNN clustering (Leibe 05). Compare to K-means, RNN does not require the user to input the length of the codebook. It is controlled by a similarity threshold indicating when two clusters are similar enough to be merged together.
   4. For the training views belonging to faces, compute the Bag Of Words (Sivic, Zisserman 03). Roughly, which codewords are activated by faces and do not activate for other objects.
   5. For recognition, compute the desired features on the Kinect cloud and find nearest neighbour word on the codebook.
   6. 2D sliding windows (after backprojecting the feature position if needed) and BoW of the feature falling into the sliding window.
   7. Compute similarity between the BoW and those BoWs from the training faces. Now I take the max and save it.
   8. Sort candidates by similarity and post-process how much you like.

   Right now, I am not doing any post-processing and just visualizing the first 5 candidates (those sliding windows with higher similarity). You can see some results in the following images. Red points indicate computed features (that are not found in the visualized candidates) and green spheres those that vote for a face within the candidates list. Next step will be some optimizations to speed up detection. Now is taking between 3 and 6s depending on the amount of data to be processed (ignore points far away from camera and downsampling).

   .. image:: img/face_detection3.png
      :align: center

   .. image:: img/face_detection4.png
      :align: center

   .. image:: img/face_detection5.png
      :align: center

.. blogpost::
   :title: Publication and face detection
   :author: aaldoma 
   :date: 4-20-2012

   This week I have been working on a publication which took away most of my time. However, we found some time to chat with Federico about face detection. We decided to try next week a 3D features classification approach based on a bag of words model. Such an approach should be able to gracefully deal with intraclass variations and deliver regions of interest with high probability of containing faces on which we can focus the most expensive recognition and pose estimation stage.

.. blogpost::
   :title: Object recognition framework (Global Pipeline II)
   :author: aaldoma 
   :date: 4-14-2012

   Hi again, first I need to correct myself regarding my last post where I claimed that this week I would be working on face detection. The experiments and tests I was doing on CVFH ended up taking most of my time but the good news are that I am getting good results and the changes increased the descriptiveness and robustness of the descriptor. 

   Mainly, a unique and repeatable coordinate frame is built at each CVFH smooth cluster (I also slightly modified how the clusters are computed) of an object enabling a spatial description of the object in respect to this coordinate frame. The other good news are that this coordinate frame is also repeatable under roll rotations and thus can substitute the camera roll histogram which in some situations was not accurate/resolutive enough yielding several roll hypotheses that need to be further postprocessed and inevitably slow down the recognition.

   This are some results using the first 10 nearest neighbours, pose refinement with ICP and hypotheses verification using the greedy approach. The recognition time varies between 500ms and 1s per object, where approx 70% of the time is spent on pose refinement. The training set contains 16 objects.
   
   .. image:: img/micvfh_icp_10nn.png
      :align: center

   A couple of scenes avoiding pose refinement stage where it can be observed that the pose obtained aligning the reference frame is accurate enough for the hypotheses verification to select a good hypothesis. In this case, the recognition time varies between 100ms and 300ms per object. 
   
   .. image:: img/micvfh_no_icp_10nn.png
      :align: center

   I am pretty enthusiatic about the modifications and believe that with some GPU optimizations (mainly regarding nearest neighbour searches for ICP and hypotheses verification) a real time (at least, almost) could be implemented.

   Regarding the local pipeline, I implemented a new training data source for registered views obtained with a depth device. In this case, the local pipeline can be used as usual without needing 3D meshes of the objects to recognize. The input is represented as pcd files (segmented views of the object) together with a transformation matrix that align a view to a common coordinate frame. This allows to easily train objects in our environment (Kinect + calibration pattern) and allow the use of RGB/texture cues (if available in the sensor) that were not available using 3D meshes. The next image shows an example of a fast experiment where four objects where scanned from different viewpoint using a Kinect and placed into a scene with some clutter in order to be recognized.

   .. image:: img/local_reg_views.png
      :align: center

   The red points represent the overlayed model after being recognized using SHOT, geometric correspondence grouping, SVD, ICP and Papazov's verification. The downside of not having a 3D mesh is that the results do not look so pretty :)  Notice that such an input could as well be used to train the global pipeline. Anyway, I will be doing a "massive" commit later next week with all these modifications. GPU optimizations will be postponed for a while but help is welcomed after the commits.

.. blogpost::
   :title: Object recognition framework (Global Pipeline)
   :author: aaldoma 
   :date: 4-05-2012

   This last week I have continued working on the recognition framework, focusing on the global pipeline. The global pipelines require segmentation to hypothesize about objects in the scene, each object is then encoded using a global feature (right now available in PCL are VFH, CVFH, ESF, ...) and matched against a training set which objects (their partial views) have been encoded using the same feature. The candidates obtained from the matching stage are post-processed with the Camera Roll Histogram (CRH) to obtain a full 6DOF pose. Finally, the pose can be refined and the best candidate selected by means of an hypotheses verification stage. I will also integrate Alex's work regarding real time segmentation and euclidean clustering to the global pipeline (see http://www.pointclouds.org/news/new-object-segmentation-algorithms.html).

   In summary, I committed the following things to PCL:
   
   1. KissFFT library to perform real or complex FFTs. KissFFT has been added to common/fft and therefore is available to all pcl modules.
   2. The Camera Roll Histogram feature and matching stage. The first can be found under pcl_features and the second one in pcl_recognition. Both contain examples on how to use the KissFFT library.
   3. A greedy hypotheses verification stage based on model inliers and outliers (in pcl_recognition).

   These are some results using CVFH, CRH, ICP and the greedy hypotheses verification:

   .. image:: img/cvfh_crh.png
      :align: center
   
   .. image:: img/cvfh_crh6.png
      :align: center

   I have as well been playing a bit with CVFH to solve some mirror invariances and in general, increase the descriptive power of the descriptor. Main challenge so far has been finding a semi-global unique and repeatable reference frame. I hope to finish at the beginning of next week with this extension and be able to cleanup the global pipeline so I can commit it. Regarding the main topic of the sprint, we will try some fast face detectors based on depth to efficiently retrieve regions of interest with high probability of containing faces. Another interesting approach that we will definetely try can be found here: http://www.vision.ee.ethz.ch/~gfanelli/head_pose/head_forest.html 
.. blogpost::
   :title: Face recognition based on 3D meshes (II)
   :author: aaldoma 
   :date: 3-29-2012

   Hi again, I integrated the generation of training data for faces into the recognition framework and use the standard recognition pipeline based on SHOT features, Geometric Consistency grouping + RANSAC, SVD to estimate the 6DOF pose and the hypotheses verification from Papazov. The results are pretty cool and encouraging...

   .. image:: img/face_recognition1.png
      :align: center

   Thats me in the first image (should go to the hairdresser...) and the next image is Hannes, a colleague from our lab.

   .. image:: img/face_recognition2.png
      :align: center

   The CAD model used in this case was obtained from http://face.turbosquid.com/ that contains some free 3D meshes of faces. Observe that despite of the training geometry being slightly different than those from the recognized subjects, the model is aligned quite good to the actual face. Notice also the amount of noise in the first image.

   I am having interesting conversations with Radu and Federico about how to proceed, so I will post a new entry soon with a concrete roadmap.

.. blogpost::
   :title: Face recognition based on 3D meshes (1)
   :author: aaldoma 
   :date: 3-27-2012

   Last week I have been working on a small framework for 3D object recognition/classification. It can be found on trunk under apps/3d_rec_framework but be aware that it is far from finished. 
   
   This is related to 3D face orientation project as face detection and orientation estimation might be approached using a classic object recognition approach: a training set of the objects to be detected (faces in this case) is available and salient features are computed on the training data. During recognition, the same feature can be computed on the input depth image / point cloud and matched against the training features yielding point-to-point correspondences from which a 3D pose can be estimated usually by means of RANSAC-like approaches.

   I am a big fan of using 3D meshes or CAD models for object recognition due to many reasons so I decided to do a small experiment regarding this for face detection. I took a random mesh of a face from the Princeton Shape Benchmark and aligned as depicted in the first image. Yaw, Pitch and Roll are usually used to define a coordinate system for faces.

   .. image:: img/face.png
      :align: center

   Because we would like to recognize a face from several orientations, the next step consists in simulating how the mesh would like when seen from different viewpoints using a depth sensor. So, basically we can discretize the yaw,pitch,roll space and render the mesh from the same viewpoint after being transformed using yaw,pitch,roll rotations.

   .. image:: img/face2.png
      :align: center

   The small window with red background is a VTK window used to render the mesh and the point cloud (red points overlapped with the mesh) is obtained by reading VTKs depth-buffer. The partial view is obtained with a yaw of -20° and a pitch of 10°. The next image is a screenshot of a multi-viewport display when varying the yaw in a range of [-45°,45°].

   .. image:: img/faces.png
      :align: center


   And maybe more interesting, varying pitch from [-45°,45°] with a 10° step. The points are colored according to their z-value.

   .. image:: img/faces2.png
      :align: center

   Basically, this allows to generate training data easily with known pose/orientation information which represents an interesting opportunity to solve our task. The idea would be to have a big training dataset so that variability among faces (scale, traits, ...) is captured. Same would apply for tracking applications were a single person is to be tracked. A mesh of the face could be generated in real-time (using KinFU) and use as only input for the recognizer. This is probably the next thing I am going to try using FPFH or SHOT for the feature matching stage.

.. blogpost::
   :title: Testing blog system
   :author: aaldoma 
   :date: 3-20-2012

   I was now on vacation for a couple of days and will be flying back tomorrow to Vienna. Used some spare time
   to update personal info and get familiar with the blogging system.

