All blog posts for Frits Florentinus
====================================

.. blogpost::
  :title: Update
  :author: florentinus
  :date: 05-25-2012

  I finished the preliminary report for ANF, which I will make available once Mattia is also finished, so that we can start to evaluate the work of this sprint with our mentors.

  In the meanwhile I have been struggling with getting my PC ready for PCL developer use again.
  For the next few weeks I will be finishing some other projects for school and will also be working on my PCL to do list:

  * Implement the MixedPixel class in PCL.
  * Work on the filters module clean up.
  * Finalize the LUM class implementation.

.. blogpost::
  :title: Testing, reporting and continuously learning new things
  :author: florentinus
  :date: 05-11-2012

  In the process of reporting for the sprint, Mattia and I have been working more in-depth on getting test results from the system and have been training a classifier.
  There have also been a few improvements to the system such as a new feature that should aid the distinguishment of leaves.

  I have furthermore been learning up on function pointers, functors, boost::bind, boost::function and lambda functions.
  They will be useful for the filters module clean up.

.. blogpost::
  :title: Reporting
  :author: florentinus
  :date: 05-04-2012

  During the last two weeks I have been working on the report for this sprint, which is becoming bigger and taking more time than I anticipated.
  I am also aiming to finish the filters module clean up which can be followed here: http://dev.pointclouds.org/issues/614.

.. blogpost::
  :title: Finishing up segmentation
  :author: florentinus
  :date: 04-19-2012

  I have been finishing up the segmentation steps of the system.
  The following is a typical result when used on the Trimble outdoor sets using the default parameters:

  .. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/florentinus/02.pcd&scale=0.004&psize=1" align="center" width="600" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>

  These results are downsampled from the original and the ground segmentation has already taken place.
  The clusters marked as red are not passed to the SVM classifier.
  They are either too small and will be marked as isolated and removed without the need for classification, or they are too large in which case they will be classified as background and are never removed.

  I upgraded the system with a very basic over-segmentation detection system that performs quite alright.
  Furthermore, most my time was spent getting the parameterization right: Making all parameters depend on as few global parameters as possible and still allowing to greatly and intuitively vary the clustering needs.
  Since we are having a chat with the mentors soon, I will discuss these topics in the report that I will write for that chat, which is what I will be working on for the next few days.

.. blogpost::
  :title: Improved segmentation step
  :author: florentinus
  :date: 04-17-2012

  I have worked on improving the segmentation steps of the system.
  Initially I wanted to do a separate sub clustering after the euclidean clustering.
  However, the version I have now performs the advanced segmentation simultaneously with the euclidean clustering.
  In the future I might add this as a new class to PCL since it could be useful for any type of conditional region growing.

  For our purposes this conditional region growing currently checks distance, intensity difference and curvature difference with respect to the candidate points, which results in better clustering than before.
  However, the balance between over- and under-segmentation is still as tricky as before.
  Ideally, the trees are clustered into separate leaves so that the SVM has a lot of different training candidates from one set.
  It is still quite impossible to get this clustering while not having too much clustering in the non-noise parts of the scene.
  I did manage to condense all the parameters for the segmentation step into one parameter, indicating clustering aggressiveness, so that a user could tweak this balance himself.

  An idea I still want to investigate is to use a pyramid or staged system that will only further sub-segment under certain conditions.
  I think these conditions will need to be more complex than what I am using now (intensity, curvature and location).
  Although making them too complex could limit the applicability of the system.

.. blogpost::
  :title: Full implementation
  :author: florentinus
  :date: 04-05-2012

  As we are implementing the ANF system, the pipeline has been slightly modified:

  .. image:: 10.png
    :width: 800 pt
    :height: 70 pt

  The ground and wall segmentation are now both performed in the planar segmentation block which can do this iteratively as long as large enough planes are being detected.

  The object segmentation has been split into two blocks:

  * Euclidean clustering will only use point location data to do segmentation and the parameters will be set to focus on not having over-segmentation.
    This will automatically make the step more prone to under-segmentation but this will be handled by the sub segmentation block.
    This step will also limit cluster sizes, automatically classifying isolated points.
  * The sub segmentation block will further divide the clusters found by Euclidean clustering and will use additional information to divide them into exactly 1 object per cluster as good as it can.

  Lastly, a feature estimation block is added that will be gathering the information needed to feed the SVM classifier.
  The sub segmentation block may be combined or interchanged with the feature estimator depending on what information will be needed to do the segmentation properly.

  The main interface for this pipeline and all of the stages are implemented with the exception of the sub-segmenter.
  What remains is refining some stages and most importantly: keeping the "automatedness" of the system.
  Each stage has its own parameters that need to be set dynamically depending on other information of the cloud.
  At the moment this is working for most cases.
  For those that it doesn't: I hope to build a feedback loop so that the system can rectify its own parameters.
  If that doesn't work out I will need to translate it to an intuitive parameter that needs to be input by the user.

  I have updated :ref:`my roadmap <florentinus_roadmap>` incorporating the latest developments.

.. blogpost::
  :title: Full implementation
  :author: florentinus
  :date: 04-04-2012

  Mattia and I are now working on a full implementation of the ANF system we have been doing for this sprint.
  We are writing the code in the same format as the tools in PCL's trunk/tools/ with the only difference being that we moved all stages of the pipeline to their own separate implementation file so that we could work on it simultaneously.
  The pipeline overview and the interface between the different stages is nearly finished.
  Tomorrow I will post a more elaborate blog update with an improved graphical overview of the pipeline and update of my roadmap.

.. blogpost::
  :title: Object clustering
  :author: florentinus
  :date: 03-30-2012

  I am testing the new embedded pcd viewer and showing the results of the objects clustering I have been working on.

  .. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/florentinus/01.pcd&scale=0.01&psize=1" align="center" width="600" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>

  The viewpoint of this pcd viewer does not auto correct which is annoying.
  When pressing R in the embedded pcd viewer it will reset the viewpoint to the one described in the .pcd file instead of calculate an optimal one.
  This means that you manually need to open the .pcd file and modify the viewpoint there so that the initial view is already valid.

  Edit: I spent 1.5 hours trying to do this... this quaternion system sucks big time. I will just ask Radu to update the embedded viewer :)

  As for the actual work: I implemented a basic ground, wall and object segmenter.
  The ground and wall surfaces are removed using SAC segmentation with a plane model.
  These steps are really fast and leave one fourth of the points left for the remaining object clustering.
  There I apply an octree representation and a Euclidean clustering to get to the clustering as shown in the picture.
  The octree is more or less used like a voxel grid filtering at the moment and speeds up the euclidian clustering significantly.
  It also holds the density information which will be useful information to pass on to the classifier.
  For the benchmark data set there were no issues with objects being very close to one another.
  Next week I will improve the Euclidean clustering to also take varying point densities into account and be able to distinguish adjacent objects.

.. blogpost::
  :title: Current pipeline idea
  :author: florentinus
  :date: 03-28-2012

  Here is a graphical overview of how the pipeline of the ANF system may be turning out:

  .. image:: 09.png
    :width: 720 pt
    :height: 72 pt

  This pipeline is fine-tuned for larger scale environments such as the Trimble data sets.
  For smaller scale environments the ground and wall segmentation steps could be omitted.

  The process from left to right can also be seen as an interactive information gathering process where the information up to that point is being used to refine the search for new information.
  This is useful for both the speed and accuracy of the final results.

  * The ground segmentation step will use very basic analysis of the scene and should be a fast step, removing a lot of points so that the other steps are processing faster as well.
    This step is likely going to be based on iterative plane fitting.
  * Similarly, the wall segmentation will also remove a lot of points, easing further processing steps.
    It will however be more difficult to distinguish walls from objects of interest so slightly more advanced analysis is required.
    This step is likely going to be based on a region growing algorithm only passing very large regions.
  * The object segmentation step is going to determine how the remaining points in the point cloud are making up objects.
    An important challenge is that it needs to be able to detect when two objects are very near to one another, where a region growing algorithm would fail.
    The step will also be gathering more information like point density, normals and curvature to use in its analysis.
  * For each of the segmented objects the classifier will determine what type of object it actually is.
    It will calculate features such as VFH and use almost all of the information already gathered.
    This step is already implemented by using a Support Vector Machine learning algorithm and is working quite accurately.
  * The final step is very specific to the application.
    For our current purposes we just need to remove the trees and moving objects from the scene.

  So the idea is to do the information gathering as late as possible in order to optimize speed (late in the pipeline means less points to apply the algorithm to).
  But don't move it too late: earlier in the pipeline is better for accuracy.

  For the next few days I will be focusing on the object segmentation step here.
  More specifically: I will investigate normalized cut clustering.

.. blogpost::
  :title: New papers, New ideas
  :author: florentinus
  :date: 03-22-2012

  After reading some more papers on segmentation, classification and recognition, Mattia and I had another talk on the ANF system.
  We are now investigating and adapting ideas from `this <http://gfx.cs.princeton.edu/pubs/Golovinskiy_2009_SRO/paper.pdf>`_ paper, which has a lot of similarities with what we are trying to achieve.
  We are thinking of splitting up the work as follows: I will work on the "information gathering" and Mattia will work on the "information processing".
  For instance, the machine learning based classification will be done by Mattia and I will work on providing the classifier with enough features to classify on.

  The new information gathering that I will be doing will likely belong in the pre-processor that I was working on already.
  However, the information that is useful to extract often requires other information and again a step-like system would develop.
  For example, the Trimble data sets start out with x, y, z and intensity information.
  This can then be used to calculate normals and curvatures.
  With this, simple features can be computed.
  After that, more sophisticated features, etc.

  I am currently investigating the features module of PCL more in-depth and am already finding a lot of useful things for this sprint.
  The eventual ANF system will probably turn out to use and interact with almost all modules in PCL :)

.. blogpost::
  :title: Tree segmentation progress
  :author: florentinus
  :date: 03-20-2012

  I have been discussing work on ANF with Mattia.
  Mattia will be building the more sophisticated segmentation system based on machine learning.
  I will be working on a more basic segmentation system and will focus on the interaction between different classes and levels in the hierarchy of the complete ANF system.
  I have updated :ref:`my roadmap <florentinus_roadmap>` incorporating the latest developments.

  Meanwhile, the TreeSegmentation class I was working on has improved further:

  .. image:: 07.png
    :width: 300 pt
    :height: 200 pt

  .. image:: 08.png
    :width: 300 pt
    :height: 200 pt

  It now also uses a very basic shape and size classification.
  Note that the results are actually in the form of a weighting to each point, the above screenshot depicts the points above a certain threshold for this weighting.

  I am not comfortable with the current method I use though.
  I want to look at an octree implementation where I can zoom through different levels of resolution and also make use of density information.
  Hopefully this will provide more accurate and faster results for the shape and size classification.

.. blogpost::
  :title: ANF progress
  :author: florentinus
  :date: 03-16-2012

  I wrote a very basic pre-processor for the ANF system.
  The idea is to gather commonly used data that the majority of the other steps of the ANF system will want to use anyway.
  For the Trimble data sets it currently only performs normal estimation and appends this information to the point clouds, resulting in PointXYZINormal type clouds.

  At the moment I am still working on the TreeSegmentation class, which will use almost all of the fields of PointXYZINormal.
  The classification steps for intensity and curvature are already finished, what remains are the steps for shape and size classification of clusters of points.

.. blogpost::
  :title: LUM class unit test
  :author: florentinus
  :date: 03-14-2012

  I intended to create a suitable unit test for the LUM class, however, I stumbled upon particular cases where the LUM class fails to give proper results.
  I ended up spending the last two days searching for the cause but was unable to find it.
  In order to satisfy http://dev.pointclouds.org/issues/623, I will now make the LUM class instantiate through a .cpp file instead.

  Tomorrow I will continue with the ANF project again.

.. blogpost::
  :title: Automated Segmentation and ANF
  :author: florentinus
  :date: 03-09-2012

  We are currently focusing on "binary noise", i.e. noise that is defined by the (binary) existence of points.
  For this type of noise, the challenges in Automated Noise Filtering can completely boil down to challenges in Automated Segmentation;
  If the AS is performing ideally, the only further step to get to ANF is to apply the ExtractIndices filter in PCL.
  Hence I have added my latest work to the segmentation module in PCL http://docs.pointclouds.org/trunk/group__segmentation.html.

  Currently there are the base class AutomatedSegmentation and one derived class AutomatedTreeSegmentation.
  Each derived class from AutomatedSegmentation is going to represent one step in the total system and focus on one particular type of entity to segment from the scene.
  These different steps can then be used in succession to get to a complete ANF system.
  However, I aim to build these classes so that they can interact with one another in more complex ways (like an iterative design or combination with registration -> SRAM).
  More information on the classes can be found in their documentation as soon as docs.pointclouds updates.

  Also, each of these classes/steps is built up from different substeps.
  For instance, The AutomatedTreeSegmentation performs intensity classification and curvature classification as substeps.
  I am still thinking if it could be interesting to separate these substeps into their own classes somehow.
  For these substeps it also holds that they may need to interact more complexly than just successive application.

  I am hoping to converse with Mattia or other people who are interested to see if this is the most interesting/useful implementation of an ANF system.
  If you are reading this and have suggestions or feedback (both positive and negative) about this, don't hesitate to drop me a line: f.j.florentinus@student.tue.nl.

  Meanwhile I will continue working on the implementation of AutomatedTreeSegmentation since is it not finished.
  I will also spend time on http://dev.pointclouds.org/issues/614 and other PCL related clean up things.

.. blogpost::
  :title: Vegetation ANF
  :author: florentinus
  :date: 03-06-2012

  The grand idea of the ANF system as discussed during last week is going to take a while to fully take shape.
  Following Jorge's suggestion, I am going to focus on the subproblem of vegetation first (trees in particular) and implement an easy system.
  The system will already be a multi-step system where the first two steps are:

  1 Intensity classification

    | Trees generally have a low intensity rating compared to most flat surface entities.
      Hopefully this step does not need any input parameters, i.e. all laser scanner type data sets have this property (TODO).
      This step is performed first for it is a very fast step and will leave a smaller exploration space for the other steps.

  2 Curvature classification

    | Leaves generally have a high curvature.
      This step will likely also need to analyze the size of the areas that have high curvatures and maybe their shape too.
      It could be useful to implement the size/shape detection as a separate step in ANF so other classifiers can also make use of this (TODO).
      This step has some parameters that need to be set but it is likely possible that in the end this can be done fully automatically.

  An interesting global input parameter / cost function would be to set the amount of trees that are in the point cloud.
  This is hopefully easily determined by a user and it allows for an iterative design where the ANF system could iterate and re-adjust its internal parameters in order to get to the appropriate answer.

  The system will also start using the weighting system where each step applies weighting to the points for how likely that point is part of a tree or not.

  I have also been working on http://dev.pointclouds.org/issues/614 since my last blog post.

.. blogpost::
  :title: Chat with Alexander and Mattia
  :author: florentinus
  :date: 03-02-2012

  Today I had a chat with Alexander and Mattia on segmentation.
  Alexander explained his work on Non-Associative Markov Networks for 3D Point Cloud Classification: http://graphics.cs.msu.ru/en/node/546.
  This type of classification could be very useful for our ANF system since we are working with quite specific noise types.
  These noise types would probably be adequately distinguishable through the machine learning that is used in this classification method.
  Alexander adds that the current implementation uses OpenCV, which would add a new dependency if it was implemented into PCL as such.

  While I am gathering information like this and thinking of how to combine it into one big ANF system, I will also be working on the following roadmap entries for the next couple of days:

  * Bugfix, patch and enhance the PCL filters module.

    - Make all filters that are about point removal derive from FilterIndices instead of Filter.
    - Move the getRemovedIndices() system from Filter to FilterIndices.
    - Implement the getRemovedIndices() system for all the derived filters.
    - Implement the MixedPixel class in PCL.

.. blogpost::
  :title: Chat with Jorge, Radu, Federico and Mattia
  :author: florentinus
  :date: 03-01-2012

  Last Monday I had a very useful meeting with Jorge, Radu, Federico and Mattia about the TRCS and ANF.
  A quick summary of the conversation:

  * The system setup for TRCS-ANF is more clear now:

    - The automated noise filtering system will perform analysis on the scene and use the results of the analysis to determine which filtering to apply and which parameters to use.
    - The system could have any number of these analysis and filtering steps, where each step has a particular focus. Steps should be minimized for the removal/alteration of non-noise points, which could limit the filter's ability in tackling noise. Hence the idea of having multiple steps: widen the overall applicability of the system.
    - Each step would have at most one settable parameter, like a slider that ranges from 0 to 1, indicating the "aggressiveness" of that step.
    - Ideally the number of sliders of the ANF system would approach zero. This would likely only happen if an adequate cost function can be devised. The cost function could also be used to enhance the performance of some of the steps in the sytem by allowing an iterative design.
    - The system can still be built in various ways, largely depending on what types of noise are in need to be tackled. For now we will focus on the noise of the Trimble data sets, namely: vegetation and moving objects noise.

  * Brainstorm on the analysis type steps:

    - Use segmentation to distinguish between different entities (both noise and non-noise). For instance:

      - Use parametric modeling to segment the ground.
      - Region growing for point clusters not connected to anything else, useful for moving objects noise.
      - Investigate MRF segmentation: http://vision.deis.unibo.it/fede/3Dsegm.html
      - Investigate region growing with smoothing constraints: http://www.pointclouds.org/blog/trcs/velizhev/index.php

    - Use properties of points and determine new properties based on surrounding points:

      - Make use of the intensity values of points in the Trimble data sets for distinguishment.
      - Compute normals, determine curvatures, useful for detecting trees.
      - Investigate AMN classification: http://graphics.cs.msu.ru/en/science/research/3dpoint/classification

    - Apply a very fast, simple conditional analysis that passes points that are definitely not noise or filters points that definitely are noise. The first steps of the system should be fast and simple like this. As the subset of unclassified points decreases, the complexity of the steps increases.
    - Instead of binary removal of points, apply weights to points, describing the certainty that the point is noise. Different steps in the pipeline alter this weight accordingly. At the end of the (sub-)pipeline apply hysteresis and/or use one of the main system sliders to determine the actual removal.
    - If possible: use change detection to further analyze the scene. Most interesting option: combine this with the registration process of the total system using an iterative design. Allows to link the intermediate results of noise weighting across different point clouds. Also ensures a symbiotic relation between registration and filtering: SRAM (Simultaneous Registration And Modeling).

  * Brainstorm on the filtering type steps:

    - The 3D bilateral filter and integral images in 2D and 3D would make powerful additions to PCL. Furhter investigation is needed however to determine their effectiveness on the Trimble data sets and this particular form of ANF.
    - The mixed pixel implementation can be easily implemented in PCL. It is also useful for this ANF since it focuses on shadow point removal. Link: https://code.ros.org/svn/ros-pkg/stacks/laser_pipeline/trunk/laser_filters/include/laser_filters/scan_shadows_filter.h

  Implementing all of the ideas above could become a huge project.
  Tomorrow I will discuss with Mattia how to properly split up the system into subsystems and determine priorities for each of the subsystems.
  The results from that conversation can be found on :ref:`my roadmap <florentinus_roadmap>`.

.. blogpost::
  :title: Filters module clean up
  :author: florentinus
  :date: 02-27-2012

  For the last couple of days I have been working on http://dev.pointclouds.org/issues/614.
  I have expanded the FilterIndices base class with the following new systems:

  * FilterNegative
  * RemovedIndices
  * KeepOrganized
  * Filtering for NaN and Inf

  The latter is not actually part of the base class, the derived classes may want to implement this if they so choose.
  The reason for that would be to give meaning to the difference between FilterNegative and RemovedIndices.
  FilterNegative only inverts the conditions of point removal for the real points.
  RemovedIndices also keeps track of the points removed because of NaN or Inf.

  For the next couple of days I will upgrade the filters that can use these new systems to do so.

.. blogpost::
  :title: Filters module analysis
  :author: florentinus
  :date: 02-23-2012

  The most eligible PCL filters for the specific noise removal in the Trimble data sets have already been discussed in previous blog posts by me and Mattia.
  The filters described in this blog post are not really suitable to be independently tested on the Trimble data sets, but a quick summary would be useful.
  While I was analyzing these filters I stumbled upon minor bugs, API inconsistencies and typos.
  The following analysis will summarize functionality, time complexity and possible improvements.

  1 Filter

    | Function: Base class for almost all filters.
      Inherits from PCLBase, which manages a point cloud.
    | Runtime: N/A
    | Notes: Currently manages removed_indices, which is only useful for filters that are about point removal, and only a few of those filters actually use this functionality.
      Better to move this functionality to the FilterIndices base class.

  2 FilterIndices

    | Function: Base class for filters that are about point removal.
      Inherits from Filter; the added functionality is being able to forward the indices of filtered points instead of points themselves.
    | Runtime: N/A
    | Notes: Some filters still inherit from Filter that could easily be upgraded to inherit from this class.

  3 Clipper3D

    | Function: Base class for BoxClipper3D and PlaneClipper3D.
    | Runtime: N/A
    | Notes: Not officially part of any API module.

  4 ExtractIndices

    | Function: Extracts a set of indices from a point cloud as a separate point cloud.
    | Runtime: :math:`O(n)`, iterates through all indices once, not performing any real computations.
    | Notes: Uses setNegative instead of the inherited getRemovedIndices for inversion purposes.
      May be upgraded to inherit from FilterIndices instead of Filter.

  5 PassThrough

    | Function: Pass certain elements of a point cloud based on constraints for one particular dimension.
      Can act on any dimension of any PointT, not just spatial dimensions.
      Only acts on one dimension at a time though.
    | Runtime: :math:`O(n)`, iterates through all indices once, not performing any real computations.
    | Notes: Has setFilterLimitsNegative and getRemovedIndices for inversion purposes.
      May be upgraded to inherit from FilterIndices instead of Filter.

  6 CropBox

    | Function: Pass certain elements of a point cloud based on contraints for their spatial dimensions.
      The constraint area is always a box but can be scaled, rotated and translated to any extent.
    | Runtime: :math:`O(n)`, iterates through all indices once, performing minor computations.
    | Notes: Does not use the inherited getRemovedIndices method and has no inversion system.

  7 CropHull

    | Function: Pass certain elements of a point cloud based on contraints for their spatial dimensions.
      The constraint area is defined by a polygon structure.
    | Runtime: :math:`O(n \cdot p)`, iterates through all indices and through all polygon points, performing medium computations.
    | Notes: Uses setCropOutside instead of the inherited getRemovedIndices for inversion purposes.

  8 PlaneClipper3D

    | Function: Check points on contraints for their spatial dimensions.
      The constraint area is a half-space, defined by a plane in 3D.
      Can be used on separate points, lines, planar polygons and point clouds.
    | Runtime: :math:`O(n)`, iterates through all indices once, performing minor computations.
    | Notes: Not part of any API module.

  9 BoxClipper3D

    | Function: Check points on contraints for their spatial dimensions.
      The constraint area is always a box but can be scaled, rotated and translated to any extent.
      Can be used on separate points as well as point clouds.
    | Runtime: :math:`O(n)`, iterates through all indices once, performing minor computations.
    | Notes: Not part of any API module.
      Two virtual methods are not implemented.
      The point cloud implementation is almost identical to the CropBox functionality.

  10 ProjectInliers

    | Function: Project certain elements of a point cloud to a predefined model.
      The possible models are defined in the sample_consensus module.
      Only moves points, does not remove points.
    | Runtime: :math:`O(n)`, iterates through all indices once, performing medium computations.

  11 RandomSample

    | Function: Downsamples a point cloud with uniform random sampling.
    | Runtime: :math:`O(n)`, iterates through all indices once, performing minimal computations.
    | Notes: Does not use the inherited getRemovedIndices method.
      Does not use the inherited setIndices method.

  For the next few days I will be tackling some of the typos and minor issues and will be adding the "bigger" issues on dev.pointclouds.

.. blogpost::
  :title: Bilateral filter analysis
  :author: florentinus
  :date: 02-17-2012

  The current bilateral filter in PCL acts only on the intensity values of the points in a point cloud and is therefore not interesting for the desired noise removal in the Trimble data sets.
  However, analysis of this implementation will help in understanding a new implementation and gives an indication for the time complexity that can be expected.

  The current version has :math:`O(n \cdot m)` complexity where :math:`n` is the number of points to iterate through and :math:`m` the average number of neighbors found for each point.

  For each point pair it will calculate weights based on distance and intensity difference using gaussian kernels.
  It uses this information to only alter the intensity values of the points.
  For more information: C. Tomasi and R. Manduchi. Bilateral Filtering for Gray and Color Images. In Proceedings of the IEEE International Conference on Computer Vision, 1998.

  The neighbors are found by a radius search (currently only tested with kdtree search) where the radius is :math:`2 \, \sigma_s`.
  The searching is the most time consuming part of the algorithm and the parameter :math:`\sigma_s` greatly determines runtime.

  =============================  =============================
  :math:`\boldsymbol{\sigma_s}`  Time (s)
  =============================  =============================
   1                              21
   5                              68
   10                             203
   15                             426
   25                             1116
  =============================  =============================

  Increasing :math:`\sigma_s` also increases the area of effect of the smoothing as can be seen in the following pictures:

  original,  :math:`\sigma_s` = 5:

  .. image:: 03.png
    :width: 276 pt
    :height: 249 pt

  .. image:: 04.png
    :width: 276 pt
    :height: 249 pt

  :math:`\sigma_s` = 15, :math:`\sigma_s` = 25:

  .. image:: 05.png
    :width: 276 pt
    :height: 249 pt

  .. image:: 06.png
    :width: 276 pt
    :height: 249 pt

  The above results all have a :math:`\sigma_r` of 1000.
  Reducing this number reduces the kernel effect and gives seemingly similar results as reducing :math:`\sigma_s`.
  The value of :math:`\sigma_r` has no effect on the computation time.

  This bilateral filter could already be considered useful for the Trimble data sets since the intensity does have some noise in it.
  With :math:`\sigma_s` = 15 the noise is significantly removed whilst not reducing detail on edges.
  The runtime is however very long.
  Hopefully a new searching algorithm would significantly reduce this.
  Furthermore it can be noted that the code can easily exploit computing parallelism since there is no conditional branching and few dependencies.

  In conclusion: The filter is very powerful and elegant, requiring few input parameters that are also easily understood.
  The upgrade to a spatial bilateral filter for 3D will likely be worhtwhile, although the drawback (for now) will be its runtime.

.. blogpost::
  :title: Quantitative filter analysis using benchmarks
  :author: florentinus
  :date: 02-15-2012

  Today I finished the quantitative filter analysis benchmark algorithm, which takes the resulting cloud of a filter, compares it against the target benchmark cloud using octree and returns the following result:

  :math:`Error = \dfrac{noise_{known}-noise_{removed}}{noise_{known}} \; + \; \dfrac{noise_{added}}{\frac{1}{10} \, desired} \; + \; \dfrac{desired_{removed}}{\frac{1}{10} \, desired}`

  The first term ranges from 0 to 1, denoting the amount of noise remaining, i.e. how good the filter is at removing the noise we want it to remove.
  The second term increases if additional noise is being generated because of the filter.
  The third term increases if non-noise/desired points are being removed because of the filter.
  If the resulting sum of these terms becomes equal or greater than 1, the filter is deemed useless.
  Because of this interrelationship the last two terms are scaled with a percentage of the total desired points in the cloud.
  This value (currently 10%) may still be changed after further analysis.
  Also, the checking for removed points is currently not taking point location into account, for instance:
  If the desired points were to be uniformly downsampled to 90% of the original, the error would be 1 although the actual result would not be that useless.

  For the next few days I will be finishing up on the PCL filter testing using these new metrics.

.. blogpost::
  :title: Quantitative filter analysis using benchmarks
  :author: florentinus
  :date: 02-14-2012

  Today I finished the benchmark target point cloud that has its noise manually removed.
  After discussing with Jorge, it was decided that we are currently only focussing on removal of points and not smoothing of points.

  .. image:: 01.png
    :width: 240 pt
    :height: 233 pt

  .. image:: 02.png
    :width: 240 pt
    :height: 233 pt

  Next I will be working on the comparison algorithm that will return a number describing the success of noise removal.
  I will level will Shaohui since this is a particular case of change detection.

.. blogpost::
  :title: Chat with Radu, Federico and Mattia
  :author: florentinus
  :date: 02-13-2012

  Last friday I had a very useful chat with Radu, Federico and Mattia about the TRCS and ANF.
  Unfortunately Jorge was not able to be present.

  A quick summary of the conversation:

  * For the time being there are 3 tasks at hand that Mattia and I can work on:

    - Test current PCL filters on Trimble data sets.
    - Bugfix and patch/modify filters that do not perform as they should.
    - Research and brainstorm for new filter implementations.

  * For further filter testing during this TRCS, a more quantifiable error metric is useful.
    Radu suggested to create a target point cloud that has the noise manually removed and compare that target with the filter results.
    This needs to be further discussed with Jorge since he will know best what noise is in need to be removed.
    Another topic of further discussion relates to currently defining noise solely as point removal, though point cloud smoothing is also interesting.
    Alexandru-Eugen Ichim is currently working on an algorithm also used for point cloud smoothing: http://www.pointclouds.org/blog/tocs/aichim/index.php.
  * Radu mentioned that shadow point removal is easily implementable and already being used on their PR2.
    Related papers: http://researchcommons.waikato.ac.nz/bitstream/handle/10289/3828/Mixed%20Pixel%20Return%20Separation.pdf and http://www.robotic.de/fileadmin/robotic/fuchs/TOFCamerasFuchsMay2007.pdf.
  * The current PCL bilateral filter only changes intensity.
    A new filter implementation based on the bilateral filter would act on the 3D coordinates.
    Federico is very knowledgeable in this field.
    A link that came up during the chat: http://people.csail.mit.edu/sparis/bf/
  * Federico mentioned the topic of integral images in 3D, which would be useful for the 3D bilteral filter and for fast filtering.
    Mattia has shown interest in working on this implementation.
  * For those filters that use a searching algorithm, which are the more powerful filters, the searching is currently the most time consuming aspect.
    Michael, Radu and Suat are discussing the possibility for adding a SphericalOrganizedNeighbor search, useful for LIDAR scans.
  * For vegetation removal; remove areas with high curvature; analyze neighborhoods differenly.
  * For LIDAR scans; take note that point cloud density is not uniform.
  * For exploiting GPU parallelism; implementations will stay in trunk till PCL 2.0

  I have updated :ref:`my roadmap <florentinus_roadmap>` and will start work on the new error metric; creating the benchmark target (segment from Statues_1.pcd) that has its noise manually removed.

.. blogpost::
  :title: Testing PCL filters on Trimble data sets
  :author: florentinus
  :date: 02-03-2012

  Mattia and I have extracted a noisy segment from Statues_1.pcd that we will use as a benchmark to test the different filters in PCL.
  We have also constructed a timing benchmark that allows us to measure the algorithm's speed more or less independent of platform.
  The desirable noise reduction and undesirable deformation are currently measured by our own (subjective) grading.
  In order to speed up the work, the extracted segment is deprived of NaNs and in the process also lost its organizing.

.. blogpost::
  :title: Testing PCL filters on Trimble data sets
  :author: florentinus
  :date: 01-31-2012

  I am currently downloading data sets that Jorge provided for further testing.
  For the next couple of days I will test the currently existing PCL filters on them and analyze the type of noise in the sets.
  I will also attempt to set up a more detailed roadmap for the next phase of the sprint.

.. blogpost::
  :title: Familiarizing with filters module in PCL
  :author: florentinus
  :date: 01-23-2012

  I have not spent a lot of time on TRCS since my last update; I am currently finishing up on some work here and will not be spending full-time on TRCS during upcoming week.
  I have discussed initial approaches for ANF with Mattia and Jorge and have slightly reworked my roadmap.
  Currently I am working on:

  * Get to know the filters module in PCL; the code structuring, the different filters in there, how to use them, when to use them, basic understanding of their workings.

.. blogpost::
  :title: First blog entry
  :author: florentinus
  :date: 01-18-2012

  Today I managed to get Sphinx running and have been updating my personal and roadmap pages.
  For the remainder of this week I will be working on the following entries of my roadmap:

  * Get familiar with the blogging system, commiting code, my mentor(s), my co-worker(s) and the other people of the TRCS.
  * Gather information on the latest/best/most interesting work in the field of ANF.

