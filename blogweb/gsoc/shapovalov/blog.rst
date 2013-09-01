.. blogpost::
   :title: Debugged Harris vertical detector 
   :author: shapovalov
   :date: 7-12-2011

    .. image:: images/cat-correct-dir.PNG
    
    Fig. 1. Psychodelic cat.

    It seems I finally debugged the detector and dominant orientation estimation. The color shows detector response at some scale (red is low, blue is high), spheres are keypoints at local maxima.

    Now I want to check its robustness to transformations such as rotation in horisontal axes, holes, subsampling etc. Unfortunately, SHREC page does not contain ground truth correspondences, so I emailed the organisers about that.

.. blogpost::
   :title: Sixth week 
   :author: shapovalov
   :date: 7-07-2011

    This week I returned to work on the project. I wanted to make sure the implementation of Harris vertical detector works correctly and tested it on artificial data. The result did not look okay. So I debugged the code and found one source of errors.

    It seems that in the new version of FLANN the interface changed and the indices returned by radius search were not sorted. So a lot of points were ignored during neighborhood analysis.

    I fixed that and the keypoints look great on the sphere:

    .. image:: images/sphere-space.PNG

    Here the colour codes the detector response. The dominant orientations are estimated incorrectly. I am going to fix it next weak and then compare the detector with some other ones.
    
.. blogpost::
   :title: Fifth week 
   :author: shapovalov
   :date: 6-25-2011

    As I described earlier, I had faced the problem of normal orientation. When the orientations are random, descriptor and detector could be estimated incorrectly. So I tried to make them invariant to individual normal orientations.

    Since my implementation of Harris detector uses only squares of projections, it does not require to have consistent orientations. So I needed to change implementation only for the dominant direction estimation. It attempts to estimate the direction where the most normals are directed (weighted window is used). So I decided to vote for the directions in pi instead of 2pi, which makes the method insensitive to orientations.

    I am leaving to the UK for a summer school in a few hours, so I won't be able to work during the next week. On returning I plan to test the repeatability of this detetor on SHREC dataset and look if the dominant direction is estimated robustly. Then I plan to use spin images to estimate the quality of the overall voting framework.
    
.. blogpost::
   :title: Lolcatz, or Keypoint detection 
   :author: shapovalov
   :date: 6-17-2011

    Everybody is posting fancy pictures, so as me. I implemented Harris height map keypoints, here is visualization.

    .. image:: images/cat-keypoints.PNG

    Note that I intensionally set high non-maxima suppression interval to make the picture visually plausible.

    The directions for voting are defined as described in my previous post. The problem is the normal map is discontinuous. For example, if all normals are directed towards the model centroid, for the points whose tangent goes through the centroid there are discontinuties. In such cases the dominant orientation cannot be estimated properly. 
    
    .. image:: images/cat-normals.PNG
    
    I tried to use MLS to get a consistent map of normals, but fixed-radius MLS failed on that model. Another solution is to use greedy triangulation, but it is unlikely to work with real-world scans. So I am going to ignore the orientation of normals and vote for two locations.


.. blogpost::
   :title: Third week 
   :author: shapovalov
   :date: 6-13-2011

    As I noted earlier, both keypoint detectors implemented in PCL are not suitable for monochrome unorganized clouds. So I needed to come up with a new one.

    Vertical world prior (though not always holds) might help in recognition. In many applications we know that the vertical axis of an object remains constant. For example, a car is usually oriented horizontally, while a pole is usually strictly vertical. On the opposite, car toy models might have any orientation. So, if the application allows it, the keypoint detector should be invariant only to horizontal rotations. Scale invariance (across octaves) often does not matter too, since e.g. car parts are usually of similar size.

    Therefore, it is natural to consider the cloud as a height map. Derivatives of this function along the horizontal axes are essentially the projections of the point normals to those axes. So, the Harris operator could be computed across the scale-space. The maximums of the operator gives (hopefully) reliable keypoints. Intuitively, keypoints are the points where the normal projection to the horizontal plane changes significantly.

    It is still unclear how to estimate the dominant orientation in that setting (the horizontal keypoint orientation may be required by the voting framework). Like in images, we can take the direction of maximum gradient, which means here the most horiontal normal (summed in gaussian weighted window). For horizontal planes there is no dominant orientation, but the Harris operator value is low there, so it fits to the defined keypoint detector. I implemented these ideas during the previous week and now ready to verify them experimentally.  
    
    Not connected to that, we also encountered a problem with FLANN radius search. We need that to estimate peaks in the ISM voting space. So we declare a point structure like this: 

    .. code-block:: c

        struct VoteISM 
        { 
          PCL_ADD_POINT4D;  
          union
          {
            struct
            {
              float votePower; 
            };
            float data_c[4];
          };

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
        } EIGEN_ALIGN16; 

        POINT_CLOUD_REGISTER_POINT_STRUCT (VoteISM,           // here we assume a XYZ + "test" (as fields)
          (float, x, x)
          (float, y, y)
          (float, z, z)
          (float, votePower, votePower)
        )

    ``pcl::KdTreeFLANN<VoteISM>::radiusSearch()`` works correctly only if the ``sorted_`` flag if false. If the result should be sorted, there appear repetitions in indices. Obviously, sorting does not work right:

    .. code-block:: c

        int n = 0;
        int* indices_ptr = NULL;
        DistanceType* dists_ptr = NULL;
        if (indices.cols > 0) {
            n = indices.cols;
            indices_ptr = indices[0];
            dists_ptr = dists[0];
        }
        RadiusResultSet<DistanceType> result_set(radius, indices_ptr, dists_ptr, n);
        nnIndex->findNeighbors(result_set, query[0], searchParams);
        size_t cnt = result_set.size();
        if (n > 0 && searchParams.sorted) {
            std::sort(make_pair_iterator(dists_ptr, indices_ptr),
                      make_pair_iterator(dists_ptr + cnt, indices_ptr + cnt),
                      pair_iterator_compare<DistanceType*, int*>());
        }

    I am not sure I defined the point type correctly, so may be those errors are due to alignment violations or something. I am going to try to analyze the problem and then submit a bug if needed.

    
    
.. blogpost::
   :title: Second week 
   :author: shapovalov
   :date: 6-08-2011

    During the latest week keypoint detection was in the focus of my attention. It is an important step in the object detection pipeline. 

    pcl::Keypoint is implemented by two classes: SIFTKeypoint and NarfKeypoint. The former needs a textured point cloud (and in fact ignores geometry), while the later could be applied only to range images. So none of them are suitable in general case. 

    There were some surveys of geometric keypoint detection in 3DIMPVT 2011. I probably need to implement some of them: [Salti2011]_, [Yu2011]_.

    In the meantime, a keypoint detector can be mocked by a simple subsampling. It can be used for prototyping as well as for the baseline for recognition method evaluation. The problem is there's no efficient subsampling method. For example, VoxelGrid does not preserve the cloud structure, and copy the cloud. It is quite inconvenient, since some descriptor implementations assume the points should belong to the original cloud. So I decided to implement a cloud-preserving variant of VoxelGrid.

    The concept of a cloud-preserving filter means it retains only the points that already belong to the cloud. There's no need to return the copy of those points, they also could be handled by their indices. So I prototyped the interface of a cloud-preserving filter and wait for the comments from the mentors. Such filters as PassThrough and xRemoval are already cloud-preserving, so it is easy to make them implement the ned interface.

    Also, PCL keypoints and filters both transform the cloud to some smaller cloud, sometimes a subcloud. As noted above, they could be used in similar cases. So probably we should unify those interfaces.

    In parallel, I work on the implementation of ISM-based recognition method, which will be hopefully added to PCL when/if it proves to work decently. 

    
    .. rubric:: References

    .. [Salti2011] A Performance Evaluation of 3D Keypoint Detectors. Samuele Salti, Federico Tombari, and Luigi Di Stefano.
    
    .. [Yu2011] An Evaluation of Volumetric Interest Points. Tsz-Ho Yu, Oliver J. Woodford, and Roberto Cipolla.

.. blogpost::
   :title: First week 
   :author: shapovalov
   :date: 5-30-2011

    I tried to build PCL before, but last week I decided to build the latest trunk version with all dependencies, including the optional ones, under Windows XP and MSVS 2010. I left only the install system, OpenNI and CUDA (since I don't have hardware for the two latest). The tutorial was useful, though there were few things where I got stuck. 

    **VTK**. First I downloaded Windows Installer, but it turned out I need to download sources to build the required libraries. VTK was being built for few hours (and few more for the debug build). Still I did not understand how to set the build variables TCL_LIBRARY and TK_LIBRARY. 

    **wxWidgets** were compiled successfully. Some libs were still missing, e.g. WX_mono. Also, I did not understand how to set the wxWidgets_wxrc_EXECUTABLE variable, since both CMake and me did not find wxrc executable. Overall, I am not sure if visualization works. Are there any variant of test_visualization?

    For **QHull**, in opposit to the tutorial, i used qhullstatic.lib, since there were no qhull.lib. It was not really intuitive, and first I tried qhull6.lib.
    
    There were few errors on the build, since I did not link **OpenNI**. It seems there is no way to turn it off in CMake configuration. There was only one failed test, i.e. pcl_keypoints, and, again, I am not sure about visualization.
    
.. blogpost::
   :title: Hello, world! 
   :author: shapovalov
   :date: 5-25-2011

