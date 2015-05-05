#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <vector>
#include <cmath>
#include <ar_track_alvar/AlvarMarkers.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/package.h>

using namespace std;
using namespace ros;


Publisher pub_pose;

geometry_msgs::Pose ARTag_pose_1;
geometry_msgs::Pose ARTag_pose; //Relative to camera frame
tf::Transform artag_tf_1; //Relative to camera frame
tf::Transform artag_tf; //Relative to camera frame

void PoseTransform()
{

			
	tf::Vector3 point_vec(ARTag_pose.position.x, ARTag_pose.position.y, ARTag_pose.position.z);
	tf::Quaternion quat_vec(ARTag_pose.orientation.x, ARTag_pose.orientation.y, ARTag_pose.orientation.z, ARTag_pose.orientation.w);	
	artag_tf.setIdentity();
	artag_tf.setOrigin(point_vec);
	artag_tf.setRotation(quat_vec);


	tf::Vector3 point_vec_1(ARTag_pose_1.position.x, ARTag_pose_1.position.y, ARTag_pose_1.position.z);
	tf::Quaternion quat_vec_1(ARTag_pose_1.orientation.x, ARTag_pose_1.orientation.y, ARTag_pose_1.orientation.z, ARTag_pose_1.orientation.w);	
	artag_tf_1.setIdentity();
	artag_tf_1.setOrigin(point_vec_1);
	artag_tf_1.setRotation(quat_vec_1);
	
}
//callback for AR tag location
void ARtag_callback(const ar_track_alvar::AlvarMarkers& msg)
{
//pub_pose.publish(msg);
	//code in here for getting the transform to the tag on the table
	for (int i = 0; i < msg.markers.size(); i++){
		if (msg.markers[i].id == 0){
		
		
			ARTag_pose.position.x = msg.markers[i].pose.pose.position.x;
			ARTag_pose.position.y = msg.markers[i].pose.pose.position.y;
			ARTag_pose.position.z = msg.markers[i].pose.pose.position.z;
			ARTag_pose.orientation.x = msg.markers[i].pose.pose.orientation.x;
			ARTag_pose.orientation.y = msg.markers[i].pose.pose.orientation.y;
			ARTag_pose.orientation.z = msg.markers[i].pose.pose.orientation.z;
			ARTag_pose.orientation.w = msg.markers[i].pose.pose.orientation.w;

			//pub_pose.publish(ARTag_pose);
			
			
			//ROS_INFO("Pose of tag with id of 0: ", msg);
		
		
	}
}
PoseTransform();
}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "magic_node");
	ros::NodeHandle nh;

	ROS_INFO("Creating Publisher and Subscribers");
	
	//subscribe to AR tag 
	ros::Subscriber ARtag_sub = nh.subscribe("/ar_pose_marker", 1, &ARtag_callback);
	pub_pose = nh.advertise<ar_track_alvar::AlvarMarkers>("Transform_From_Tag",1);

	ros::Rate loop_rate(50);

	while (ok())
	{	
	        spinOnce();

		//sleep
		loop_rate.sleep();
	}
	
	return 0;
}
