Roadmap
=======
.. _gioia_roadmap:

This is a detailed roadmap for my GSoC 2012 project. The boldface writings represent my current activities.

* Module keypoints:

  * First approach to the object recognition research field: documentation on techniques at the state of the art.
  * Test functionalities for the new 3D object recognition tutorial and the existing detectors in PCL.
  * Implement a framework for simple keypoint detection evaluation:

    * Determine detectors' limits and detectors' saliency.
    * Structured comparison between the existing keypoint detectors in PCL:
      
      * Detectors under testing: Harris 2D, 3D, 6D, SIFT, NARF, Uniform sampling.
      * Compute absolute repeatability.
      * Compute relative repeatability.

    * Visualize results by means of the PCLVisualizer.

  * Extend the framework in order to perform tests on complete datasets: 
    
    * Structured comparison between the existing keypoint detectors in PCL:
      
      * Detectors under testing: Harris 3D, 6D, SIFT, NARF, Uniform sampling.
      * Study of the detectors' parameters.
      * Compute average absoulute repeatability on the overall dataset at issue.
      * Compute average relative repeatability on the overall dataset at issue.
      * Perform an effective analysis about detectors' time performances:
	
        * Compute the extraction time for each keypoint detector. 

      * Summarize the collected data by means of appropriate tables and graphs.

    * Tests execution on two different datasets:

      * A kinect-based dataset (Dataset 5 - Kinect).

        * Detectors: Harris 3D, 6D, SIFT, NARF, Uniform sampling.

      * A synthetic dataset (Dataset 2 - Stanford).	

      * Detectors: Harris 3D, NARF, Uniform sampling.

      * Both the datasets can be found at `http://vision.deis.unibo.it/SHOT/ <http://vision.deis.unibo.it/SHOT/>`_ .

  * Porting of the ISS detector in PCL.
  
    * Read the documentation paper.
    * Take some knowledge about the code that has already been implemented.
    * Design of the ISSKeypoint3D class:

      * Define the input parameters of the detector.
      * Define the output parameters of the detector.
      * Define the first pipeline operations needed to compute the interest points.
      * Implement the skeleton of the class.
      * Implement the methods needed to obtain a basic functionality of the detector.

    * Refinement of the ISSKeypoint3D class:
	
      * Take knowledge about the time performances of the detector by means of a basic evaluation framework. 
      * Take knowledge about the repeatability performances of the detector by means of a basic evaluation framework.
      * Detect the bottleneck (if any).
      * Enhance the time performances of the detector by using appropriate data structures.
      * Enhance the time performances of the detector by using the OpenMP directives.
      * Add the boundary estimation if it is possible and it does not decrease the performances of the detector (both time performances and repeatability performances).

  * Test the performances of the ISS detector.

    * Use the skeleton of the evaluation frameworks developed at the beginning of the GSoC.
    * Test the repeatability performances.
    * Test the time performances.
    * Test the time performances related to the OpenMP optimization.
    * Compare the performances of ISS with those of the other dectors already tested.
  * Tests execution on two different datasets (as for the first part of the my GSoC roadmap).

* Tools

  * Develop a new point type to handle monochrome images.

    * Name: *pcl::Intensity*.
    * Number of fields: 1 (type: uint8_t).
  
  * Develop a PNG to PCD converter.

    * Input:

      * Name of the PNG input file.
      * Name of the PNG output file.

    * Output:

      * A PCD file that represents the conversion of the PNG input file.

.. * **Porting of the 3DGSS detector in PCL.**

..   * Read the documentation paper.
..   * **Take some knowledge about the code that has already been implemented.**
..   * **Re-design of the 3DGSS class**:

..    * Move the 3DGSS class to the pcl::keypoint module.
..    * Make the 3DGSS class extending the pcl::Keypoint class.
..    * Add some doxygen documentation.



