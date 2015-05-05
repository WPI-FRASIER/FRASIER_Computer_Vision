/*
 * normal_components_image.h
 *
 *  Created on: Dec 16, 2014
 *      Author: Vanderlei Cunha Jr.
 */


#ifndef NORMAL_COMPONENTS_H_
#define NORMAL_COMPONENTS_H_

#include<Fixtures/virtual_fixtures.h>
#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <vector>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>


//ros include files
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/PointIndices.h>
#include <pcl/ros/conversions.h>
#include <image_transport/image_transport.h>

#include <pcl/filters/passthrough.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

//Segmentation and Indicies Extraction PCL libraries
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>


//#include <pcl/visualization/cloud_viewer.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Fixtures/virtual_fixtures.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <DepthSense.hxx>


#include<Fixtures/virtual_fixtures.h>
#include<Fixtures/Images/fixed_base_tool.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
using namespace ros;
using namespace pcl;

ros::Subscriber subscribeSegmentedObject;
ros::Publisher publishPreoperativeImage;

#endif /* NORMAL_COMPONENTS_H_ */
