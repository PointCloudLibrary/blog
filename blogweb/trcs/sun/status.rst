My status updates
=================
.. blogpost::
  :title: Introduce a joint probability distribution for estimating changes
  :author: sun
  :date: 03-30-2012
  
  Recently, I came across a `paper <http://robots.stanford.edu/papers/kaestner.05.pdf>`_  in which a statistical model was proposed to detect changes between two range scans. I am going to well understand and implement it. 
  
  Currently, PCL has the capability to handle geometrical and intensity differences detection. The general criterion is the Euclidean distance. I wish to improve this by adopting the model mentioned in this paper.
  
  I am contacting the author about the mathematical derivations in the paper and will post the progress later.
  
.. blogpost::
  :title: Code revision and future plan
  :author: sun
  :date: 03-17-2012
  
  According to the feedback from Gordon, I revised the code I wrote. I will add more functionalities to handle RGB point cloud, as well as spatial+intensity change simultaneously.
  

.. blogpost::
  :title: Latest Result for Change Detection on Intensity Values
  :author: sun
  :date: 03-08-2012
  
  I continue working on http://dev.pointclouds.org/issues/630.
  
  I am still using Gordon's data, but at this time I did more trimming and converting work. I added one more data member to pcl::SegmentDifferences which controls the threshold that the users could specify when looking at the intensity values.
  
  By far, the new code has been working fine. 
  
  Reference
  
  .. image:: img/test3/reference01.png 
     :height: 300 pt 
     :width: 600 pt
  
  Target
  
  .. image:: img/test3/target.png
     :height: 300 pt 
     :width: 600 pt
  
  Result
  
  .. image:: img/test3/diff01.png 
     :height: 300 pt 
     :width: 600 pt
  
	 
.. blogpost::
  :title: pcl::SegmentDifferences Profiling Result
  :author: sun
  :date: 03-04-2012
  
  After talking with Radu, I decided to use Very Sleepy to profile the performance of pcl::SegmentDifferences. The computing time was extremely large to Trimble data. I could not wait till the program stopped before it used up the memory. 
  
  The basic statitics shows:
  
  Filename: D:\PCL_TRCS\pcl_ssh\bin\TRCS_test_debug.exe
  
  Duration: 31549.642000s
  
  Date: Sat Mar 03 23:29:50 2012
  
  Samples: 3619112
  
  I wish I could have made a figure but I haven't found a tool to convert the result to a reasonable graph. The output .csv file was not well generated. So, here I just show the screen shot from which you could clearly see which parts are the most time consuming parts.
  
  .. image:: img/prof/prof.png 
     :height: 500 pt 
     :width: 1000 pt
  
.. blogpost::
  :title: Add intensity differences detection to pcl::Segmentation module
  :author: sun
  :date: 02-28-2012
  
  I have opened up a new issue on http://dev.pointclouds.org/issues/630 and still working on it.
  
.. blogpost::
  :title: Final week for this quarter
  :author: sun
  :date: 02-15-2012
  
  It is approaching my final week in this quarter. I am in a little bit more pressure. Anyway I am planning to play with more data from trimble and try to gain some ideas from the noise removal project.
  
.. blogpost::
  :title: Preliminary Result for Change Detection on Intensity Values
  :author: sun
  :date: 02-11-2012
  
  I just came back to NY from a conference in CA. Currently, I am coding in order to test some thoughts. This time, I am looking at the intensity difference between two corresponding points in two inputs. In order to figure out the corresponding pairs, I adopted nearest neighbour searching method which was similarly used in pcl:SegmentDifferences. This time, I tried to ignore the location difference. So if the distance between them was too large, the two matching points were avoided any further processing. 
  
  I actually care about the two papers on the floor in target more. The result is the intensity difference between the two inputs. We could see the papers were successfully detected because they have higher values. We could some other major differences were detected. Please ignore the noise on the background this time.
  
  Reference
  
  .. image:: img/test2/reference00.png 
     :height: 300 pt 
     :width: 600 pt
  
  Target
  
  .. image:: img/test2/target00.png
     :height: 300 pt 
     :width: 600 pt
  
  Result
  
  .. image:: img/test2/diff00.png 
     :height: 300 pt 
     :width: 600 pt
	 
.. blogpost::
  :title: First Testing Result based on Current Octree-based Change Detector
  :author: sun
  :date: 02-02-2012
  
  I have set up an initial testing project for myself a while before. Finally, I got great testing datasets. Trimble provided us with more than 2G data and Gordon collected several point clouds of his own garage. Since there are a lot of NAN values in Trimble data that could be removed by centain PCL functionality but I am kind of 'lazy' and have other options :D, I decided to choose two of Gordon's data files of which details are shown as below to start.
  
   - reference.pcd: baseline scan
   - change.pcd: items on the floor moved a few cm, door moved, item moved from floor to table.
  
  I have been studying how octree-based change detector works. Basically, double buffer structure and XOR bit pattern were used to decide which points are in the target but not in the reference. If you want to know which points are in the reference but not in the target, you just need to switch their roles.
  
  Here are some first simple testing results. The major time-consuming part is not the stage of change detection, but the loading data part.
  
  Reference
  
  .. image:: img/test1/snapshotA00.png 
     :height: 300 pt 
     :width: 600 pt
  
  Target
  
  .. image:: img/test1/snapshotB00.png
     :height: 300 pt 
     :width: 600 pt
  
  Result
  
  .. image:: img/test1/snapshotBdiff00.png 
     :height: 300 pt 
     :width: 600 pt

.. blogpost::
  :title: Change Detection based on Hausdorff Distance
  :author: sun
  :date: 01-27-2012

  I have started looking at how to do change detection based on Hausdorff Distance. Basically what I wish to do is looking for the nearest neigbour in the reference 
  point cloud of each point in the target point cloud and then calculating the Euclidian distance. There is supposed to be a threshold that could be specified by 
  users to segment out the differences. The whole thought is pretty similar to what we have now in pcl:getPointCloudDifference. I want to inject this functionality to 
  the module pcl:octree.

.. blogpost::
  :title: Update on progress on change detection project
  :author: sun
  :date: 01-23-2012

  I am still running and hacking the current example on Octree-based detector. So far, I have not been quite sure where I should go with yet. Radu and Gordon gave me some suggestion on
  methods based on
  
   - change detection in geometry space (e.g., XYZ)
   - change detection in color space (e.g., RGB)
   - combination of both
   
  I was looking for some published literature on Octree-based method. Now according to the new suggestion, I would like to change my direction and find something new and interesting.

.. blogpost::
  :title: My first blog entry
  :author: sun
  :date: 01-18-2012

  Getting familiar with Sphinx blog system. 

