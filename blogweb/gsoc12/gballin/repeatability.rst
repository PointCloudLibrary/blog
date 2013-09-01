How to compute the keypoint repeatability
=========================================
.. _gioia_repeatability:

This section of my blog is devoted to the keypoint repeatability computations. Since I had some misunderstanding while reading the articles on this subject, I thought it could be useful to clarify here the way in which the keypoint repeatability should be computed in the context of an evaluation framework. First of all, I need to cite the reference paper during my work on the evaluation framework:

* Salti, S.; Tombari, F.; Stefano, L.D.; , **"A Performance Evaluation of 3D Keypoint Detectors,"** 3D Imaging, Modeling, Processing, Visualization and Transmissio (3DIMPVT), 2011 International Conference on , vol., no., pp.236-243, 16-19 May 2011 (Best Paper Award Runner-up).

In order to compute the keypoint repeatability related to a specific keypoint detector at least two computational steps are needed (absolute repeatability). If we are interested also in computing the relative repeatability, the required steps will be three. In what follows, a brief description of all the three steps is given. We assume that we are dealing with a pair of meshes/point clouds: one represents the model and the other represents the scene in which an instance of the model at issue is contained. The three steps are:

1. **Calculus of the set of all the keypoints extracted on the model that are not occluded in the scene.**
     
   This set is estimated by aligning the keypoints extracted on the model according to the ground-truth rotation and translation and then checking for the presence of vertices in the scene in a small neighborhood (1 x *model_resolution* in my evaluators) of the transformed keypoints. If at least a vertex is present in the scene in such a neighborhood, the keypoint of the model is added to the set. Finally, the cardinality of the set is computed and returned.

2. **Calculus of the absolute repeatability.**

   The absolute repeatability is computed starting from the set of keypoints defined in step 1. Each keypoint of the model contained in the aforementioned set is extracted and transformed according to the ground-truth rotation and translation. The transformed keypoint is finally added to the set of repeatable points if the distance from its nearest neighbor in the set of keypoints extracted from the scene is less than a threshold ``epsilon`` (2 x *model_resolution* in my evaluators). Finally the cardinality of the set of repeatable points is computed and it represents the desired absolute repeatability measure.

3. **Calculus of the relative repeatability.**

   In order to compute the relative repeatability, all the required metrics have already been computed in point 1. and point 2. . Indeed, the relative repeatability measure is simply obtained by dividing the cardinality of the set of repeatable keypoints (point 2.) by the the cardinality of the set of all the keypoints extracted on the model that are not occluded in the scene (point 1.).


I hope that this brief section could be helpful to anyone who is approaching to the evaluation issues.
