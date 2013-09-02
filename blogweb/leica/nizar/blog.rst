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

   
   The reading speed is slower than the aim of the project. First
   attempt is to speed it up by using proper data types.  
	  
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

    * The compression ratio is now of 0.27 using lzf + JP2K hybrid compression
    * The reading rate of ascii data is 0.021 ms/point 
    * Writing using a hybrid compression lzf/JP2K speed is 0.001 ms/point

