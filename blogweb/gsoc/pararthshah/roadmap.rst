.. _pararthshah_roadmap:

Point cloud features and registration - Project Roadmap
=======================================================

To aid in the development of new feature descriptors and registration algorithms, we plan to create a set of tools to 
automatically test different algorithms and compare their speed and effectiveness.  We will then implement several
new feature descriptors and registration techniques that have been proposed in recent scientific literature and 
evaluate them on a set of benchmark data sets.  All new code will be thoroughly documented and tested, and we will
create a series of tutorials to help new users understand how to apply these new features and registration algorithms 
in their own code.

Our initial plan for this project is described below.

Getting started
~~~~~~~~~~~~~~~

 * Get familiar with PCL

   * Create an account on dev.pointclouds.org

     * Send public SSH key to Radu
     * Download the PCL source tree

       * (``svn co svn+ssh://svn@svn.pointclouds.org/pcl/trunk``)

   * Read, compile, experiment with several tutorials

 * Set up the project website/status reports

   * Install Sphinx (instructions :ref:`here <How-to-install-Sphinx>`)
   * Fill out the :doc:`index <index>` page
   * Write an introductory status update (instructions :ref:`here <How-to-write-a-blog-post>`)

Quantitative evaluation framework
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The first milestone of this project is to design and implement a framework for
quantitatively evaluating the accuracy of different feature estimation and
registration algorithms.  We need a utility that will allow us to

* specify one or more benchmark data sets
* select an algorithm and a set of parameter values
* run a series of registration or feature correspondence trials
* compare the results to our ground truths to score the outcome
* summarize the results of multiple trials and report accuracy/runtime statistics

Here's a breakdown of the basic requirements:

Benchmark data sets
*******************

The benchmark data sets that we select for this task should reflect the typical use cases for registration algorithms.
One common application is the construction of a 3D panorama; in this case, the data is taken in a static environment 
from a moving 3D camera over a large range of rotation and a relatively small range of translations. For our benchmark
we have captured a series of point clouds from a Kinect camera mounted on top of our PR2's head as it scanned across an
conference room.  To construct a benchmark, this raw data must now be organized into a set of specific pairwise
registration "tests" that each algorithm can be evaluated on.   

Although we will start with only one data set, the framework should be designed to accomodate many different data sets
that will cover a range of different environments and tasks.

Evaluation routine
******************

The evaluation routine will comprise two kinds of tests:

In the **feature correspondence test**, the system will be given two clouds (*source* and *target*).  It will compute feature descriptors in each cloud using the specified algorithm and parameters. Each feature in the source cloud will be matched to its corresponding feature in the target cloud based on a nearest neighbor search in the *n*-D feature space.  For each point, the system will compare the 3D position of the estimated corresponding target point to the previously established ground truth position.  If the two points are close together (as determined by a user specified threshold), then the correspondence is marked as a *success*.  Otherwise, it is marked a *failure*.  The total number of successes and failures will be calculated for a range of different parameters and thresholds and stored for further analysis.

In the **registration test**, the system will be given two clouds (*source* and *target*).  It will estimate the rigid transformation that best maps points in the source cloud to their corresponding point in the target cloud.  The estimated transformation will be compared to the ground truth.  If the two transforms are sufficiently similar (as determined by a user specified threshold) it will be marked as a *success*.  Otherwise, it will be marked a *failure*.  Numerous trials will be performed over a range of different parameter settings and different intial conditions, and the total number of successful vs. unsuccessful alignments will be recorded for further analysis.

Results summary
***************

Results will be stored in a machine-readble format that can be used to create reports and visualizations.  A small set of tools will be developed to automatically generate graphs and publish a ReST or HTML formatted document that will allow users and developers to browse the results of the tests that have been performed.


Implement new point cloud feature descriptors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Implement, document, and test several new 3D feature descriptors

 * <Details forthcoming>

* Evaluate new algorithms on benchmarks

 * Select good default values
 * Document performance under different conditions
 * Summarize performance results in our online documentation

   * it will make life a lot easier for our end users if they can see some quantitative measures of
     how well our various algorithms perform on different kinds of data.

* Implement new initial alignment algorithm

 * <Details forthcoming>
