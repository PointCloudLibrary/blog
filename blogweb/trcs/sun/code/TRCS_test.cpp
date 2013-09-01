/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  \author Shaohui Sun (sxs4643@rit.edu)
 * */

/////////////////////////////////////////////////////////////////////////////
// MAIN
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/impl/segment_differences.hpp>
#include <iostream>
#include <time.h>

#define PointT pcl::PointXYZI


int
main (int argc, char** argv)
{
  srand ((unsigned int) time (NULL));
  time_t start, end;
  double dift;

  float resolution = 5.0f;

  pcl::SegmentDifferences<PointT> sd;
    
  pcl::PointCloud<PointT>::Ptr cloudA (new pcl::PointCloud<PointT>); 

  time(&start);
  cloudA->width = 128;
  cloudA->height = 1;
  cloudA->points.resize (cloudA->width * cloudA->height);  
  for (size_t i = 0; i < cloudA->points.size (); ++i)
  {
    cloudA->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudA->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudA->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
	cloudA->points[i].intensity = rand () / 255 + 1.0f;
  }
  time(&end);

  dift = difftime (end,start);
  std::cout<<"It took "<<dift<<" secs to read in the first point cloud"<<std::endl;
  
  pcl::PointCloud<PointT>::Ptr cloudB (new pcl::PointCloud<PointT>);

  time(&start);
  cloudB->width = 128;
  cloudB->height = 1;
  cloudB->points.resize (cloudB->width * cloudB->height);
  for (size_t i = 0; i < cloudB->points.size (); ++i)
  {
    cloudB->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudB->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudB->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
	cloudB->points[i].intensity = rand () / 255 + 1.0f;
  }
  time(&end);

  dift = difftime (end,start);
  std::cout<<"It took "<<dift<<" secs to read in the second point cloud"<<std::endl;
 

  pcl::PointCloud<PointT> cloudC;
  cloudC.width = 0;
  cloudC.height = 0;
  cloudC.points.resize (cloudC.width * cloudC.height);

  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
  kdtree->setInputCloud(cloudB);
  sd.setDistanceThreshold (resolution);
  sd.setInputCloud (cloudA);
  sd.setTargetCloud (cloudB);
  sd.setSearchMethod (kdtree);  
  sd.segmentInt (cloudC);

  std::cout<<"The points in cloudC are "<<std::endl;
  for (size_t i = 0; i < cloudC.points.size (); ++i)
  {
	std::cout<<cloudC.points[i].x<<" "<<cloudC.points[i].y<<" "<<cloudC.points[i].z<<" "<<cloudC.points[i].intensity<<std::endl;
  }
  system("pause");
}