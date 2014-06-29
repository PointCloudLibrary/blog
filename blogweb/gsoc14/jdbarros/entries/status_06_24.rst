.. blogpost::
  :title: 1. Reduction of computational redundancy in cost aggregation in stereo matching.
  :author: jilliam
  :date: 21-06-2014

  **INTRODUCTION**
  
  A stereo image pair can be used to estimate the depth of a scene. To do so, it is necessary to perform pixel matching and find the correspondences in both images. Different methods for stereo correspondence have been proposed and they are classified in two classes:

  - Correlation-based algorithms: Produce a dense set of correspondences.

  - Feature-based algorithms: Produce a sparse set of correspondences.
  
  Additionally, correlation-based algorithms are usually classified in two main groups, local (window-based) or global algorithms. However, some methods do not fit into any group, and are classified in between them.

  The current work is based on correlation-based algorithms, more espefically local and window based-methods, intended for applications where a dense and fast output is required.

  The input of the algorithm are two calibrated images, i.e. the camera geometry is known. The images are also rectified in order to limit the correspondence to a 1D search.

  **CORRESPONDENCE ESTIMATION**
  
  The general methodology for stereo vision local approaches can be summarized as follows. An energy cost is computed for every pixel p by using the reference and d-shifted right images:

  .. math:: e \left(p,d \right) = min \left(|I_{l}(x,y)-I_{r}(x-d,y)|, \sigma \right)
  	:label: eq11

  Then, the aggregated cost is computed by an adaptive sum of the per-pixel cost:

  .. math:: E(p,d) = \dfrac{\displaystyle \sum_{q \in N(p)}w(p,q)e(q,d)}{\displaystyle \sum_{q \in N(p)}w(p,q)}
  	:label: eq12

  Finally, a Winner-Takes-All method is used to find the best of all the disparity hypothesis:

  .. math:: d(p) = argmin\{ E(p,d), d \in [ 0,..,D-1 ] \}
  	:label: eq13

  This whole process is complex and time consuming since it is repeated for every hypothesis d. 
  A representation of the conventional approaches can be observed in next figure [Min1]_.

  .. image:: figures/conventional_ca.png
	      :height: 240px
	      :align: center

  Min et al. [Min1]_ introduced a new methodology to reduce the complexity, by finding a compact representation of the per-pixel likelihood, assuming that low values do not provide really informative support. In this case, only a pre-defined number of disparity candidates per pixel are selected to perform the cost aggregation step. The subset of disparity hypotheses correspond to the local maxima points in the profile of the likelihood function, previously pre-filtered to reduce the noise, as shown in the following example:

  .. image:: figures/disp_cand_selection.png
	      :height: 240px
	      :align: center

  The disparity hypotheses estimation and cost aggregation processes proposed by Min et al. are depicted in the next figure, where Sc is the subset of disparity hypothesis with size Dc: 

  .. image:: figures/compact_ca.png
	      :height: 240px
	      :align: center

  .. [Min1] Min, D., Lu, J., & Do, M. N. "A revisit to cost aggregation in stereo matching: How far can we reduce its computational redundancy?." In IEEE International Conference on Computer Vision (ICCV), 2011 (pp. 1567-1574).
