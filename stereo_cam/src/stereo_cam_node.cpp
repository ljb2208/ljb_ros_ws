/*
 * stereo_cam_node.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: lbarnett
 */

#include "../include/stereo_cam_node.h"

void stereoCamTriggerCallback(const stereo_cam::stereo_cam_triggerConstPtr& msg) {
	ROS_DEBUG("Trigger received");
	seq_no_ = msg->seq_no;
	captureFrame();
}

bool openCamera() {
	ROS_INFO("Opening device: %s Width: %i Height: %i FPS: %i Left Cam: %i Mono Mode: %i", device_.c_str(), width_, height_, fps_, left_camera_, mono_mode_);
	v4l = new V4lVideo(device_.c_str(), width_, height_, fps_, gs_mode_);
	return true;
}

void closeCamera() {
	v4l->Stop();
}

bool captureFrame() {
	sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();

	v4l->GrabROSNewest(msg);

	imagePub.publish(msg);

	stereo_cam::stereo_cam_capture captureMsg;

	if (mono_mode_) {
		captureMsg.left_image_received = true;
		captureMsg.right_image_received = true;
	} else {
		if (left_camera_)
			captureMsg.left_image_received = true;
		else
			captureMsg.right_image_received = true;
	}
	captureMsg.seq_no = seq_no_;

	capturePub.publish(captureMsg);

	ROS_DEBUG("Capture message published.");

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stereo_camera_capture", ros::init_options::AnonymousName);

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("camera_id", camera_id_, int(0));
	nh_private.param("left_camera", left_camera_, bool(false));
	nh_private.param("mono_mode", mono_mode_, bool(false));
	nh_private.param("gs_mode", gs_mode_, bool(false));
	nh_private.param("width", width_, int(640));
	nh_private.param("height", height_, int(480));
	nh_private.param("fps", fps_, int(30));


	if (camera_id_ < 0)
		ROS_WARN("Invalid camera ID");

	std::stringstream dev;
	dev << device_ << camera_id_;
	device_ = dev.str();

	openCamera();

	image_transport::ImageTransport it(nh);

	if (left_camera_) {
		imagePub = it.advertise("camera/left_image", 1);
	}
	else {
		imagePub = it.advertise("camera/right_image", 1);
	}


	capturePub = nh.advertise<stereo_cam::stereo_cam_capture>("stereo_cam_capture", 1);
	ros::Subscriber sub = nh.subscribe("stereo_cam_trigger", 1, stereoCamTriggerCallback);

	ros::spin();
	/*
	ros::Rate r(100);

	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
	*/

	closeCamera();

	return 0;
}



