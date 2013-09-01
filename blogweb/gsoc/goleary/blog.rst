Blog
====
.. blogpost::
   :title: Gabe O'Leary's first status update
   :author: goleary
   :date: 5-23-2011

   Today I learned how to add content to the developer blogs.

   Here's a code snippet:

   .. code-block:: cpp

      // This is a really boring block of code...
      int n = 10;
      for (int i = 0; i < n; ++i)
      {
        printf ("%d\n", i);
      }

   And here's an equation:

   .. math::

      ax^2 + bx + c = 0

.. blogpost::
   :title: And now for something completely different
   :author: goleary
   :date: 5-24-2011

   I've spent a large amount of time installing tons of libraries and software on my Apple laptop, and after many many problems and headaches I decided to just install Ubuntu on my computer to make everything easier.  After doing a bunch of research on compatibility and some other issues having to do with the EFI firmware on my computer, I finally start to install it and I attempt to partition my hard drive.  For some god forsaken reason my computer is apparently unable of moving some of the files that are on my hard drive so it can't partition the hard drive.  After going to all this trouble Vincent (my mentor) just suggested that I use a virtual machine.  Why didn't I think of that in the first place.  Well anyway so now I'm installing Ubuntu on my virtual machine and then I'm going to have to reinstall all of the libraries and everything necessary for PCL as well as the software that Michael told everyone to install.

.. blogpost::
   :title: Gabe O'Leary's status update for Week 1
   :author: goleary
   :date: 5-29-2011

   So I finally got the pcl to compile this week, but that was using:

   .. code-block:: sh

      svn co http://svn.pointclouds.org/pcl/trunk pcl-trunk

   I had been unable to properly use ssh and svn to download the code.  This is because I had been trying to put the repository in my root folder and therefore had to use sudo which bypassed my regular public ssh key and used a different one.  Anyway all of that now works fine.  Once I downloaded the repository I attempted to compile and it only got to 5 percent before giving me a build error.  After discussion on the IRC channel we realized this was an error that had actually been committed already, but it was resolved.  My first milestone that I am now working on is going over the list of tutorials and finding out what is missing.  In order to do this I have created a file, located at gsocweb/source/goleary/tutorials.rst that will keep track of all components that exist in the PCL, and whether or not they currently have a tutorial.  You can view this list :doc:`here <tutorials>`.  I am going to be populating this list over the next few days, but I would appreciate any assistance as I am not hugely familiar with the Point Cloud Library and all of it's componenets as of yet.

.. blogpost::
   :title: Gabe O'Leary's status update for Week 2
   :author: goleary
   :date: 6-8-2011

   It has been a rather hectic past week for me as my house has flooded for the second time in 1 month and I had to meet with a lawyer because my landlord is refusing to pay for damages.  Anyway other than that I've been compiling a list of all "interesting" classes which can be found :doc:`here <interesting>`.  I will begin by making tutorials for modules that currently have none.  Most of the modules have atleast one tutorial made for them but several do not.  Keypoints, Octree and Registration are the only 3 modules which do not currently have any tutorials.  Vincent, my mentor, suggested that I go ahead and make a tutorial for the IterativeClosestPoint class in the Registration module, so that is going to be the next thing that I do.

.. blogpost::
   :title: First tutorial lives
   :author: goleary
   :date: 7-5-2011

   First off I'd like to apologize for the lack of posts as of late.  I've been dealing with several unexpected family issues that have required a significant amount of traveling, and my time and attention.  Regardless, my focus will be dedicated to this program soley for the rest of the summer.  I've more or less finished a turorial for the :doc:`IterativeClosestPoint <tutorials/iterative_closest_point>` class in the Registration module.  I'm still becoming comfortable with the PCL code, so if anyone has some suggestions or anything please shoot me an email.  I've also realized that there is an unconsistency between the tutorials about certain things like 'using namespace...'.  I think it could be beneficial to decided on a standard style that can be applied across all of the tutorials.

   Edit: I ended up also going ahead and working on a simple :doc:`tutorial <tutorials/concave_hull>` for the surface module.  It's for the  :doc:`ConcaveHull <tutorials/concave_hull>` class.  There is something wrong with the pcl code at the moment, because the final concave hull that gets returned is composed of 0 points.  This is also true for ConvexHull right now.  Other than that the tutorial is correct.

.. blogpost::
   :title: Filter & surface tutorials and merge
   :author: goleary
   :date: 7-9-2011

   So I made some progress with my project this week, all of the tutorials that I have done so far are now live on the `tutorial section of the website <http://www.pointclouds.org/documentation/tutorials/>`_.  I ended up merging both the ConvexHull and the ConcaveHull tutorials into one tutorial because of how much code, and how many explanations were similar or the same between the two of them.  I also wrote 2 more tutorials for the only two classes in the filter module that did not already have tutorials, specifically for the ConditionalRemoval and RadiusOutlierRemoval classes.  I realized how similar the code for both of these was to the StatisticalOutlierRemoval tutorial written by Radu, so I decided to combine all of them into one all encompassing tutorial on removing outliers using different methods.

.. blogpost::
   :title: Forward progress and set-backs
   :author: goleary
   :date: 7-13-2011

   I've been dedicating a lot more of my time to this organization lately, and one thing I really need to do is learn how to use the visual tools of the pcl so that I can create pretty pictures for use in my tutorials.  I've been attempting to run the PCLVisualizer demo, but I keep running into issues with the pcl-trunk Library installed.  Initially I was unable to even compile the demo source file provided with the tutorial, but Radu had me make a small change to the code that fixed that problem.  After that though, I ran into an even bigger problem with the compiled program seg faulting everytime a try running it no matter what command line argument was specified (except -h).  Radu and Vincent both told me it ran fine for them and neither of them were working with the trunk, so I'm currently compiling the pcl from branch 1.0.x.  Hopefully after I get that installed I will be able to complete the tutorial without issue.  On to other progress, I updated the layout of our `tutorial page <http://www.pointclouds.org/documentation/tutorials/>`_ this past week.  I added a table of contents to the beginning of the page, re-ordered the sections and made everything look a little bit cleaner.  I completed another merge, this time of the concatenate fields and concatenate points because they had so much (code and explanations) in common.  I am now working on a tutorial for the sample_consensus module, specifically for the RANSAC.

.. blogpost::
   :title: Random Sample Consensus tutorial
   :author: goleary
   :date: 7-19-2011

   So I finally got the PCLVisualizer demo to work.  I just ended up completely removing all of the Point Cloud Library components from my computer and re-compiling and installing the trunk.  I believe that the problem I had been encountering was because I had installed some pcl-developer tools by apt-get.  Anyway I'm now able to run the tutorial without any issues.  This is going to be helpful for making my tutorials becuase I'll be able to make pictures to include in my documents.  I have a lot of the code for my Random Sample Consensus tutorial written, but I'm running into a few issues with displaying it so that everything makes sense.  I think what I have written works and demonstrates the algorithm, I'm just working on some pictures that show what's going on.

.. blogpost::
   :title: RANSAC tutorial done moving on
   :author: goleary
   :date: 7-26-2011

   At last the Random Sample Consensus tutorial is done.  It has 2 seperate examples in it, one using the plane sample consensus model, and the other using the spherical sample consensus model.  I was trying to use a cylindrical model, but as I have not yet learned how to generate normals to surfaces I was unable to continue, so I just went with the spherical model instead.  Now I am moving on, and my mentor, Vincent, suggested that work on a tutorial for either the Range Images module or the KDTree module, so that is what I am going to begin working on.

.. blogpost::
   :title: KdTree search tutorial.
   :author: goleary
   :date: 8-2-2011

   At the moment I'm working on a tutorial for the KdTree module.  It is nearly done, I just need to finish up the theoretical primer and then it will be completly good to go.  I've also gotten several emails regarding the tutorials that I have already written which I need to sit down and respond to.  I believe one is about the RANSAC tutorial and the other has to do with my concave/convex hull tutorial.

.. blogpost::
   :title: KdTree Search tutorial complete
   :author: goleary
   :date: 8-4-2011

   So I finished up the KdTree tutorial, and added a theoretical primer to it to explain a little bit about whats going on.  This addition should be live on the tutorial page as soon as the website updates.

.. blogpost::
   :title: Updates
   :author: goleary
   :date: 8-18-2011

   I'm supposed to add an example to the Statistical removal tutorial that use pcl::PointXYZRGB instead of pcl::PointXYZ, but the input file provided contains no RGB data, so I need a different file to work with for the example.  I've also been requested to provide an example of finding the volume of irregularly-shaped objects from PointClouds (such as leaves on a bush), and am now just waiting on the files to be provided so that I can write the example.
