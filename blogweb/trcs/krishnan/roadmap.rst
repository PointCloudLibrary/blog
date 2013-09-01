Roadmap
=======
.. _krishnan_roadmap:

The roadmap to begin with is to add more algorithms to the following modules in the Registration pipeline

* Correspondence Estimation
* Correspondence Rejection
* Transformation Validation

**Correspondence Estimation**

We are planning to add Normal shooting, Source point projection and Source point projection followed by search for correspondence estimation. Also planning to implement a weighting function that assigns weights to the computed correspondences. Assigning weights should help in the next stage of the pipeline (Correspondence Rejection).

**Correspondence Rejection**

Few techniques to be added here are (1) Rejection based on thresolding of weights computed in the previous step (2) Rejection of pairs that are inconsistent with the neighbours (in terms of distances / weights) (3) Rejection based on distances / weights being larger than some multiple of the standard deviation of distances / weights.

**Transformation Validation**

Need to do a literature survey on validating a transformation between two point clouds. This is very useful in the sense that it could be fed back into the registration pipeline or tell the user that the alignment failed.

The techniques listed above follow those mentioned in the paper "Efficient Variants of ICP". More techniques can be added during the course of the project.
