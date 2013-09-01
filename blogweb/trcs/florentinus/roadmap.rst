Full roadmap for Frits Florentinus
==================================

.. _florentinus_roadmap:

  * Familiarize myself with Automated Noise Filtering and the TRCS.

    - Get familiar with the blogging system, commiting code, my mentor(s), my co-worker(s) and the other people of the TRCS.
    - Get to know the filters module in PCL; the code structuring, the different filters in there, how to use them, when to use them, basic understanding of their workings.
    - Gather information on what PCL currently has implemented regarding ANF.
    - Gather information on the latest/best/most interesting work in the field of ANF.
    - Determine a more detailed set of goals to achieve during this TRCS regarding ANF.
    - Divide the work with Mattia and form a more detailed roadmap.

  * Test the current PCL filters on the Trimble data sets.

    - Work has been split with Mattia, I will work on: BilateralFilter, ProjectInliers, CropBox, CropHull, RandomSample, NormalSpaceSampling.
    - Analyze the filters for effectiveness in noise removal and time complexity.
    - Set up quantifiable metrics to measure the results.

  * Bugfix, patch and enhance the PCL filters module.

    - Make all filters that are about point removal derive from FilterIndices instead of Filter.
    - Move the getRemovedIndices() system from Filter to FilterIndices.
    - Implement the getRemovedIndices() system for all the derived filters.
    - Implement the MixedPixel class in PCL.

  * Give shape to the ANF system.

    - Aim to minimize the amount of input parameters of such a system.
    - Create an intuitive hierarchy for different levels of analysis (e.g. SRAM level, segmentation level, classification level).
    - Create an intuitive class structuring based on this hierarchy, but one that also focuses on useful interaction between the classes.
    - Implement the weighting system instead of binary classification.
    - Devise an understandble user interface.

  * Implementation of the ANF system.

    - Set up a ground and wall segmentation class.
    - Set up a point cloud clustering class.
    - Set up a working pipeline interface incorporating all of the ideas.
    - Refine all stages to be more robust and adaptive in its automatic parametrization.

  * Other things to do after full system is working:

    - Implement the MixedPixel class in PCL.
    - Work on the filters module clean up.
    - Look into the possibilities of a weighting system and SRAM.

