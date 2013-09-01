My status updates
=================


.. blogpost::
  :title: My first blog entry
  :author: Sergey
  :date: 01-24-2012

  Hi everybody. Today I finally figured out how to post blog entries. I also was able to install python and Sphinx on my PC. So from now on I don't have to commit the rst files to see the result.




.. blogpost::
  :title: First results
  :author: Sergey
  :date: 01-31-2012

  A few days ago I was thinking about how it would be better to organize the code and its structure, dependencies between classes. Because there will be several algorithms of segmentation, I have decided that it would be better if all of them will be inherited from some base class named Segmentation(or something like that). So based on these thoughts I have written the code and even managed to test it on some of the datasets I had.

  Synthetic clouds

  .. image:: images/Cube.png
     :target: http://s015.radikal.ru/i331/1202/fa/a579b3bb3796.png

  .. image:: images/2.png
     :target: http://s018.radikal.ru/i503/1202/0b/3614889e1048.png

  Noisy synthetic clouds

  .. image:: images/6.png
     :target: http://s018.radikal.ru/i500/1202/34/12431704c49d.png

  .. image:: images/4.png
     :target: http://s018.radikal.ru/i503/1202/0b/3614889e1048.png

  Real clouds

  .. image:: images/Cars.png
     :target: http://s57.radikal.ru/i156/1202/4e/4c57388d4003.png

  .. image:: images/5.png
     :target: http://s018.radikal.ru/i507/1202/5c/739d5c1d5ef4.png

  Point cloud that I found on the local forum

  .. image:: images/Office_1.png
     :target: http://s017.radikal.ru/i428/1202/ce/e3ec16057943.png

  .. image:: images/Office_2.png
     :target: http://s14.radikal.ru/i187/1202/27/7461065a6739.png

  I hope that at the end of this week I will be able to commit the code. But right now I'm gonna run more tests and will give the code a proper form.



.. blogpost::
  :title: Trimble data
  :author: Sergey
  :date: 02-12-2012

  Hi evreybody. I finally managed to test the algorithm on the Trimble datasets. So here are some results.

  .. image:: images/place_1.png
     :target: http://s018.radikal.ru/i507/1202/53/84ce72677907.png

  .. image:: images/place_2.png
     :target: http://s017.radikal.ru/i440/1202/a5/b86453bbe1d8.png

  .. image:: images/facade_1.png
     :target: http://s017.radikal.ru/i404/1202/47/ffec4938d88e.png

  .. image:: images/facade_2.png
     :target: http://s017.radikal.ru/i421/1202/a3/bd2f27f61f92.png

  .. image:: images/elephant_1.png
     :target: http://s017.radikal.ru/i424/1202/0f/b2cc1c58a7c5.png

  .. image:: images/elephant_2.png
     :target: http://s04.radikal.ru/i177/1202/8f/5955a3c9d13c.png

  .. image:: images/statues_1.png
     :target: http://s018.radikal.ru/i508/1202/f3/4d913b78bb7a.png

  .. image:: images/statues_2.png
     :target: http://s017.radikal.ru/i434/1202/28/48c93adfb13f.png

.. blogpost::
  :title: Source code
  :author: Sergey
  :date: 02-22-2012

  Hi everybody. I have commited the source code of the RegionGrowing algorithm. But right now it is disabled, because it needs to be fixed a little. But everyone who is interested can look at it.

  I wrote one more variant of this algorithm. It takes the color of the points into account. It also has a good approach for controlling under- and over- segmentation. The idea is very simple. If the segment has less points than the user wants then the algorithm finds the nearest neighbouring segment and merges them together. I want to test it a few more times. The detailed description can be found in the article "Color-based segmentation of point clouds" by Qingming Zhana, Yubin Liangb, Yinghui Xiaoa.

  One more interesting thing about the commited code. During testing I found it too slow. I was very surprised when my profiler said that the problem is in std::vector<bool>. I didn't knew that it packs booleans one per bit, causing loss of speed when accessing elements. Anyway, I solved this problem by simply changing the value type.

  My next step is to write some unit tests for the algorithm and make a tutorial.
  
.. blogpost::
  :title: New commit
  :author: Sergey
  :date: 02-27-2012

  Hi everybody. I've made it. Not without the help from Radu, but I finally committed the code. I found out how those gTests work and wrote some for my class. I also solved the problem with line endings(MSVC was using CR+LF instead of LF).

  There was one more interesting thing about the code that I wrote. I am using vector of lists to store indices of segmented points. It looks like std::vector<std::list<int>>. I was very surprised when Radu told me that there occured some errors during the compilation, because I have manually assembled the library on my PC. The cause of it was the missing space. GCC wasn't able to compile the std::vector<std::list<int>>. ">>" cannot be compiled on GCC.

  Right now I'm going to prepare the the second variant of the RegionGrowing algorithm taking into account all those difficulties that I met on my way. I think it would be easier and much faster because now I have more experience.
  
.. blogpost::
  :title: New test results
  :author: Sergey
  :date: 03-05-2012

  Hi everybody. I have tested the Region Growing algorithm that uses points color. So here are the results.

  .. image:: images/office4_original.png

  .. image:: images/office4_dist-10_rcdist-5_pcdist-6_pnum-600_.png

  .. image:: images/office2_original.png

  .. image:: images/office2_dist-10_rcdist-5_pcdist-6_pnum-200_88-segments_1067-sec_.png

  .. image:: images/office3_original.png

  .. image:: images/office3_dist-10_rcdist-5_pcdist-6_pnum-200_75-segments_1889-sec_.png

  .. image:: images/office1_original.png

  .. image:: images/office1_dist-10_rcdist-5_pcdist-6_pnum-200_87-segments_1247-sec__.png

  There are some more pictures in my folder in higher resolution. You can find them in "trcsweb\\source\\velizhev\\images\\COLOR"

.. blogpost::
  :title: Color-based segmentation
  :author: Sergey
  :date: 03-19-2012

  Hi everybody. I have commited the code for Region Growing algorithm that uses the points color. I also made some refactoring for the base algorithm, so now it works faster.
  Right now I'm going to implement the segmentation algorithm based on the graph cut.

.. blogpost::
  :title: Min-Cut Segmentation
  :author: Sergey
  :date: 04-02-2012

  Hi everybody. I have wrote the code for building the graph. Right now it uses triangulation algorithm from PCL. But I intend to write my own code for this later, because triangulation from PCL requires normals and their calculation can slow down the graph building.
  My variant will simply find K nearest points and add edges between initial point and its neighbours, as suggested in the article. Constructing the graph this way can cause existance of several connected components. But this problem can be solved easily by connecting points from different components with the edge.
  
  Right now I am inspecting Boost Graph Library for the necessary algorithms. I have found several algorithms for finding the maximum flow in the graph, but I think that I will use push_relabel_max_flow algorithm because it is the fastest one.
  I have never used BGL algorithms for finding maximum flow, so right now I am trying to run some simple examples for small graphs.
  
  One more important thing is that we have decided to generalize the algorithm. It will allow user to specify points that belong to backrgound/object. Not only one point that belongs to object as said in the base algorithm. The idea is based on the interactive application that was described in the artcile.

.. blogpost::
  :title: BGL usage
  :author: Sergey
  :date: 04-06-2012

  Finally! I have figured out how to use the push_ralabel_max_flow. Documentation of the BGL is not the best(ordinary users won't understand how all this works). But I have managed to launch a few simple examples and it looks great.
  
  So right now I will change the code a little so that it would work with the real point clouds. It won't take much time so I hope that in a few days I will be able to post some results of the segmentation using PCL web viewer.

.. blogpost::
  :title: Debugging and varying weights
  :author: Sergey
  :date: 04-24-2012

  Hi everybody. I have finally finished the work on the code for min cut segmentation. It took so long because of some bugs. The simplest mistakes are always hard to find. Any way. the code is ready and I even have some results of segmentation. The algorithm requires point clouds with cutted floor planes etc. So first of all I have deleted the floor plane. You can see the result in the point cloud viewer.

  .. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/velizhev/ggg.pcd&scale=1.0&psize=1" align="center" width="600" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>

  Right now I am trying to find the best unary and binary potentials(edge weights), because those that were mentioned in the article are not good enough. I hope that at the end of this week I wil be able to find them and upload the resulting code.

.. blogpost::
  :title: More results of segmentation
  :author: Sergey
  :date: 05-02-2012

  Hi everybody. I have ran some tests and results are terrific. The algorithm works well even for very noisy point clouds. Here are some results:

  .. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/velizhev/result_2.pcd&scale=10.0&psize=1&zoom=10" align="center" width="450" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>
	<iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/velizhev/result_3.pcd&scale=10.0&psize=1&zoom=10" align="center" width="450" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>
	<iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/velizhev/result_5.pcd&scale=10.0&psize=1&zoom=10" align="center" width="450" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>
	<iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=http://svn.pointclouds.org/trcsweb/source/velizhev/result_6.pcd&scale=10.0&psize=1&zoom=10" align="center" width="450" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>

  There were some problems with the min cut algorithm. The best known is the algorithm proposed by Yuri Boykov, Olga Veksler and Ramin Zabih(based on their article "Fast Approximate Energy Minimization via Graph Cuts"). But it has some license constraints. So I used it only for testing at the beginning of the work.
  My code is using boykov_kolmogorov_max_flow algorithm from boost graph library. At the beginning I tried to use push_relabel_max_flow from BGL but it works a little bit strange. For the same unary and binary potentials(data and smooth costs) push_relabel_max_flow gives worse results then boykov_kolmogorov_max_flow does. So I've decided to give preference to the last one.

  Right now I'm going to make some last changes in the code to fit the adopted rules.
  
  Forgot to tell there are some problems with Online 3D point cloud viewer. I have already wrote about it to the authors.
  The problem appears only in Mozilla Firefox, it works fine with Google Chrome. So I hope everybody is able to see my point clouds.

.. blogpost::
  :title: Code for Min Cut Segmentation
  :author: Sergey
  :date: 05-11-2012

  Hi everybody. I have committed the code for min cut segmentation. At the moment only basic functionality can be used. I wanted to make a generalization for adding points that are known to be points of the background. Right now this option is turned off. It works fine but I just wanted to run more tests for this functionality.

  So my next step will be to test this option. After that I will start to write tutorials for the code that I've added(RegionGrowing, RegionGrowingRGB, MinCutSegmentation). Exams are nearing so I will pay less time for working on the TRCS. But this is temporary. I hope that I will finish additional functionality, that I mentioned, before the beginning of the exams.