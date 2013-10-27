My status updates
=================

.. blogbody::
  :nr_days:  60
  :author: koenbuys

.. blogpost::
   :title: Final report and code committed
   :author: koenbuys
   :date: 3-1-2013

   Last two weeks I've finished the NPP implementation of the Haar Cascade detector, it is now fully ported to PCL. I've included Alex Trevor his organized multi-plane
   segmentation to people api as well, so that people can do background subtraction based on the planes found in the image.
   This week I wrote the final report summarizing the work done as well as how to use the module.

   .. raw:: html 

      <center><iframe src="http://docs.google.com/viewer?url=https%3A%2F%2Fgithub.com%2FPointCloudLibrary%2Fblog%2Fblob%2Fmaster%2Fblogweb%2Fnvcs%2Fkoenbuys%2Ffiles%2Ffinal_report.pdf%3Fraw%3Dtrue&embedded=true" width="400" height="800" style="border: none;"></iframe></center>


.. blogpost::
  :title: XML interface for people
  :author: koenbuys
  :date: 06-20-2012

  I've added an XML interface to load/save person specific configuration files. The generic XML file is also saved in trunk. This will allow people to tune the kinematic chain according to themselfs to improve on the tracking results. This XML interface is currently already in the process of beeing extended to an v0.2 which will also include the directed acyclic graph (DAG) description of the kinematic chain tree, allowing users to reconfigure this and place the root at a different position. The XML interface is now part of the PersonAttribs class and in the examples can be used with the -XML tag.
  For the summer I'll have a student to help me with the training process as we now got access to the AWS cluster to enhance our training facilities. We will also be looking into training on GPU. If anybody has good knowledge about how to achieve this (RDF training in CUDA) and is willing to point this out to me or to help out, please send me an email!

.. blogpost::
  :title: Face detection in PCL
  :author: koenbuys
  :date: 06-19-2012

  I've added the FaceDetector class framework and committed it, in the back I've also started porting the NPP face detector implementation from Nvidia to PCL. However as there is still some uncertainty on the license of this. I'm currently waiting for a reply from Nvidia before I will commit it to trunk. This includes a GPU Viola Jones implementation from Anton Obukhov, "Haar Classifiers for Object Detection with CUDA" explained in the GPU Computing Gems book.

.. blogpost::
  :title: Gaussian kernel convolution
  :author: koenbuys
  :date: 06-17-2012

  I've added a GPU Gaussian kernel convolution, this allows to add a system model to the human in order to have sensor fusion on the pixel labels. The prediction step will make use of the Gaussian uncertainty. I've also added the Gaussian kernel generation function.

.. blogpost::
  :title: Label histograms finished and working great
  :author: koenbuys
  :date: 06-11-2012

  I've finished the probabilistic label histograms in the people library, these are now part of the RDF detector class and will be added to the future detector classes as well, this will allow for easy probabilistic merging of the different detectors in order to improve the tracking results. Downside is that calculating them actually takes a lot of time, so I will look into doing them in NPP in the future and reorganising them from AOS to SOA architecture. With the SOA beeing image oriented.

.. blogpost::
  :title: Exams and stuff
  :author: koenbuys
  :date: 06-10-2012

  It's been a while since I wrote a blogpost because I'm currently very occupied with TA and correcting exams for this. In the process we started the discussion how to redesign the PCL GPU sublibrary for the next releases of PCL. The goal is to remove PCL CUDA sublibrary by PCL2.x and keep only the PCL GPU sublibrary. I'm also thinking about a possible redesign of the DeviceMemory and DeviceArray structures that are currently beeing used. Feel free to contact me or Anatoly for pointers on this.

.. blogpost::
  :title: Training pipeline
  :author: koenbuys
  :date: 05-31-2012

  I finally found the time to start putting the training pipeline code public, still the Makefiles of the public version need to be adapted, and this will most likely have multiple changes and API breaks within the next weeks, but the code can be found here: http://svn.pointclouds.org/people/. There is also a change that this will move to trunk/people in the future once this is fully in the PCL namespace.

.. blogpost::
  :title: Training pipeline
  :author: koenbuys
  :date: 05-29-2012

  I had a great time presenting my work at ICRA, but still some small bug was present in the public version, from time to time the background was displayed as the correct kinematic chain, this is now solved in the current trunk version. This was because it was displaying without a check for the correct tree beeing build.

.. blogpost::
  :title: Data
  :author: koenbuys
  :date: 04-25-2012

  I'm recording a one person test dataset today which has Vicon measurement data in there as well. I'll use the plug-in-gait model from Vicon to have something to compare the behavior of our algorithm against. As a first step this will just be a single person, in order to see how this data is best recorded and synchronised in order to have a more extensive experiment later on. 
  In the meanwhile with help of Anatoly's CUDA experience the peopleDetector now runs at a stable 6-8fps which can be considered adequate for online use. My hopes are that bringing in the prior fusion will only help speeding up this process.

.. blogpost::
  :title: Roadmap
  :author: koenbuys
  :date: 04-22-2012

  This is an overview of things I will focus on the next week:
    * Add a probabilistic DeviceArray into the RDF CUDA code
    * Expand each tree vote into a more probabilistic 1/NR_TREES = 25% vote in this DeviceArray
    * Add a discretisation function (in a CUDA kernel) for this array, giving the final votes

  This expansion allows the detector to do sensor fusion based on data that can be fetched from other detectors (like face-detector). A more theoretical explanation to this will follow.

.. blogpost::
  :title: First blog entry
  :author: koenbuys
  :date: 04-21-2012

  This is my first blog post. Past two weeks I've been integrating the code for run-time RDF labeling into PCL together with Anatoly. The code is now arranged in the PeopleDetector class and all the CUDA code 
  is converted to use the DeviceArray structures. For the Seeded Hue Segmentation (SHS) and Euclidean Labeled Clustering (ELEC) a custom function was implemented specific for our use case. For interested users:
  please keep on using the original version in trunk/segmentation and don't use the one in gpu/people as that one is currently written specific.
