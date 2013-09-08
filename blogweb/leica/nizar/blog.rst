.. blogpost::
   :title: Start
   :author: nizar
   :date: 04-26-2013

   
   Started the data compression project.

.. blogpost::
   :title: PTX format reader
   :author: nizar
   :date: 05-01-2013

   
   Created class PTXReader to read content of leica PTX ASCII file and
   store data into a PCL cloud.
   Reader is pushed into https://github.com/nizar-sallem/pcl leica branch.

.. blogpost::
   :title: Leica proper data structures
   :author: nizar
   :date: 05-02-2013

   
   The reading speed is slower than the aim of the project. My aim is
   to speed it up by using proper data types.
	  
   First, we propose a new file structure for PTX where the
   coordinates can be separated from the image data. Header contains
   three extra fields:
     
     * *data_type*: indicates whether it is 
       
       #. ascii
       #. binary 
       #. binary compressed
	  
     * *image_offset*: if image data is to be separated from the
       coordinates than it starts at position in the
       file. image_offset should be set to *-1* if no RGB data is stored.

     * *image_encoding*: indicates how the image is stored
       # bgr8 indicates a binary pixel map
       # jp2k indicates a JPEG2000 compressed data image

   Second, leica::PointCloud class inherits from pcl::PointCloud with
   an additional transformation matrix.
    
   Third, sensor_msgs::PTXCloudData inherits from
   sensor_msgs::PointCloud2 with extra fields:
    
     #. image_offset
     #. image_encoding
     #. image_step

   Finally, adapted point types:
    
     #. PointXYZI without extra padding;
     #. PointXYZIRGB specific to leica contains both color and intensity data.

.. blogpost::
   :title: Improved compression and writing speed
   :author: nizar
   :date: 08-29-2013


   I implemented two compression methods :

     * LZF mainly a rewrite from PCDWriter::writeBinaryCompressed method
     * JP2K + LZF method which uses LZF to compress XYZ and intensity
       information while JP2K is used to compress RGB data.

   This choice is motivated by the need of a comparison basis and also
   by the fact that RGB data won't be compressed efficiently by LZF
   since it is a dictionary based algorithm.

   As for JP2K, it is an improvement of JPEG it is a wavelet based
   compression algorithm which claims higher compression rates with
   almost no data loss.

   The implementation I am using is the one provided by OpenJPEG. As
   version 1.3 seems to be the most common I picked it to run the
   tests.

   I spent the few past weeks trying to improve the data read/write
   speed by using leica centric point types which lead to better
   results.

   In the next weeks I will be essentially running tests and trying to
   enhance compression performances.

   For now loseless compression ratio is 0.27 using LZF + JP2K,
   ASCII data reading is 0.021 ms/point while LZF + JP2K data writing
   speed is 0.001 ms/point.

.. blogpost::
   :title: Performance analysis
   :author: nizar
   :date: 09-06-2013


   This part of the project is purely analytical where I compare
   compression rate/speed of several compression methods. 

   Below are compression rates on the test dataset. Tests were run on
   a personal laptop powered by a i7 CPU M 620 @ 2.67GHz. To be fair,
   I only compare loseless compression rates.

   ===========  =====  ======  ====  ==========
   File         ASCII  binary  LZF   LZF + JP2K
   ===========  =====  ======  ====  ==========
   indoor.ptx   480M   210M    169M  134M
   outdoor.ptx  212M   124M    124M  X                
   ===========  =====  ======  ====  ==========

   For PTX files with RGB data the joint LZF + JP2K compression is the
   most efficient, else I would recommend the simple binary format
   since LZF doesn't seem to do much compression on this data type.

   Main issue though is that the JP2K compression is not fast : it
   takes almost 10s on my laptop to perform for the indoor.ptx dataset
   but I believe it is acceptable given the gain in file size.

