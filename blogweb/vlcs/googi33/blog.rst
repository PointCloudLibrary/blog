.. blogpost::
   :title: Ubuntu and Win7 - VLCS HDL Viewer Test
   :author: googi33
   :date: 01-23-2013

   It has been fast time since VLCS starting. Keven announces “pcl_hdl_simple_viewer” in Dec. 2012. Now, it supports live mode (network) and recorded file mode (PCAP file) with several visualization format (XYZ, XYZI, XYZRGB). He mainly develops under Linux-Fedora. I have tested and supported under Linux-Ubuntu and Windows 7.

   For windows implementation, Hdl_simple_viewer needs WinPcap
 
   - WinPcap user version, WinPcap development version (with environment setting PCAPDIR). It will be explained on   document later.
   - WinPcap Windows Installer : http://www.winpcap.org/install/default.htm
   - WinPcap Developer’s Pack : http://www.winpcap.org/devel.htm

   Unfortunately, PCAP is not stable under Windows 64bits. It takes a long time to know about it. So, I build up 32-bit development environment.

   Ubuntu and Windows test image are shown as belows

   - Recording file mode(PCAP file) - road.pcap from velodyne website.

     Ubuntu XYZ / XYZI / XYZRGB

     Windows XYZ / XYZI / XYZRGB

   .. image:: img/filemode.png
      :align: center

   - Live mode (Live mode also provides three visualization format).

     Ubuntu / windows

   .. image:: img/livemode.png
      :align: center



