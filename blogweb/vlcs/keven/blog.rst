.. blogpost::
   :title: Velodyne Laser Code Sprint (VLCS) Update
   :author: keven
   :date: 12-05-2012

   I'm pleased to announce that PCL now supports the Velodyne High Definition Laser (HDL) -32 and -64 lasers.  The interface is provided as a Grabber (HDL_Grabber) and accepts packets from the network (live) or PCAP file (recorded).  Two sample programs, pcl_hdl_grabber and pcl_hdl_viewer_simple are provided to demonstrate the capabilities of the system.  libpcap-devel is required to build the PCAP reading portions of code.  
   
   Sample PCAP Files are provided by Velodyne: http://velodyne.com/lidar/doc/Sample%20sets/HDL-32/
   
   The image below came from the Building-Side.pcap.
   
   .. image:: img/Building-Side.png
      :align: center
   

