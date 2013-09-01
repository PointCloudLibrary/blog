.. blogpost::
   :title: Progress
   :author: ioan
   :date: 7-04-2011	

   Last week after some discusions with Nico, I managed to correctly complie and install OpenCV with CUDA support. This part was messy for me. After that I recompiled the source code of PCL, WITH_CUDA, and compiled the kinect_ransac.cpp in order to get more familiar with the framework. During the weekend I started to implement the MSAC algorithm. I can say that here I have progressed a little. The code is not done yet, but is due tomorrow night.   Today I am planing to implement and test the sac model functions needed to be able to compute the MSAC. - this is inspired from pcl/sample_consensous/. The next post, probably tomorrow night will contain some results from the MSAC implementation with CUDA. 


.. blogpost::
   :title: Progress report
   :author: ioan
   :date: 6-20-2011	

   Due to some minor problems I was not able to work on gsoc last week. Today I downloaded (knowing that Nico had updated the Cuda code last week) and compiled the updates concerning the CUDA code. I started to get a general ideea on what is going on. The plan is that this night I will try to better understand what was done. And tomorrow to test some of the code on my machine. After this I have to get some guidance from Nico concerning what is to be done next.

.. blogpost::
   :title: Update
   :author: ioan
   :date: 6-08-2011

   Last week I have been very busy- reason for no posts. I started getting familiar with the THRUST library and with the project. As a quick todo, this days I have to get in contact with Nico and discuss more details about the roadmap. As a plus side of the last week+now: I am participating at a HPC computing ( Toward petaflop numerical simulation on parallel hybrid architectures - INRIA sophia antipolis) summer school where I have the great opportunity of listening to Jack Dongara and other interesting persons. 
   As far as gsoc concerns me, starting from this week I will have enough time to perform all the necessary tasks, as I finished with the work needed for my master thesis and for the internship. 

.. blogpost::
   :title: Progress report
   :author: ioan
   :date: 6-13-2011
   
   This week I started by reading the document provided along with M-estimator SAmple and Consensus. (MLESAC: A new robust estimator with application to estimating image geometry, by P. H. S. Torr and A. Zisserman.) I also started reading and testing the existing RandomSampleConsensus class.
   At the moment I am trying to figure out how to implement this MSAC using the existing CUDA code. As I progress I will post more details concerning the implementation.
