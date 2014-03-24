/*
 * stereo_cam_trigger_node.cpp
 *
 *  Created on: Mar 17, 2014
 *      Author: lbarnett
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../../devel/include/stereo_cam/stereo_cam_trigger.h"
#include "../../../devel/include/stereo_cam/stereo_cam_capture.h"

bool left_image_rec = true;
bool right_image_rec = true;
int seq_no = -1;
int loopCounter = 0;

ros::Publisher trigger_pub;

void trigger() {
	//stereo_cam::stereo_cam_triggerPtr msg = boost::make_shared<stereo_cam::stereo_cam_trigger>();
	stereo_cam::stereo_cam_trigger msg;
	msg.trigger = true;
	//msg->trigger = true;

	if (seq_no == 65536)
		seq_no = 0;
	else
		seq_no++;

	//msg->seq_no = seq_no;
	msg.seq_no = seq_no;

	trigger_pub.publish(msg);


	ROS_DEBUG("Trigger sent");
}

void stereoCamCaptureCallback(const stereo_cam::stereo_cam_captureConstPtr msg) {

	ROS_DEBUG("Capture received");
	if (msg->seq_no == seq_no) {
		if (msg->left_image_received)
			left_image_rec = true;
		if (msg->right_image_received)
			right_image_rec = true;
	} else {
		left_image_rec = false;
		right_image_rec = false;

		trigger();
		loopCounter = 0;
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stereo_camera_trigger");

	ros::NodeHandle nh;
	trigger_pub = nh.advertise<stereo_cam::stereo_cam_trigger>("stereo_cam_trigger", 1);
	ros::Subscriber sub = nh.subscribe("stereo_cam_capture", 10, stereoCamCaptureCallback);

	ros::Rate loop_rate(100);

	while (ros::ok()) {
		ros::spinOnce();

		if (left_image_rec && right_image_rec){
			left_image_rec = false;
			right_image_rec = false;
			trigger();
			loopCounter = 0;
		} else
			loopCounter++;

		if (loopCounter == 100) {
			ROS_WARN("Timeout on capture...sending trigger message.");
			loopCounter = 0;
			trigger();
		}

		loop_rate.sleep();
	}

	return 0;
}
