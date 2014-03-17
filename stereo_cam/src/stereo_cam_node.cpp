/*
 * stereo_cam_node.cpp
 *
 *  Created on: Mar 13, 2014
 *      Author: lbarnett
 */

#include "../include/stereo_cam_node.h"

void stereoCamTriggerCallback(const stereo_cam::stereo_cam_triggerConstPtr& msg) {
	ROS_INFO("Trigger received: %i", msg->trigger);
	captureFrame();
}

bool openCamera() {
	ROS_INFO("Opening device: %s Width: %i Height: %i FPS: %i", device_.c_str(), width_, height_, fps_);
	v4l = new V4lVideo(device_.c_str(), width_, height_, fps_);
	return true;
}

void closeCamera() {
	v4l->Stop();
}

bool captureFrame() {

	sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
	v4l->GrabROSNewest(msg);

	imagePub.publish(msg);

	return true;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stereo_camera_capture");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh.param("camera_id", camera_id_, int(0));
	nh.param("left_camera", left_camera_, bool(false));
	nh.param("width", width_, int(640));
	nh.param("height", height_, int(480));
	nh.param("fps", fps_, int(30));

	if (camera_id_ < 0)
		ROS_WARN("Invalid camera ID");

	std::stringstream dev;
	dev << device_ << camera_id_;
	device_ = dev.str();

	openCamera();

	image_transport::ImageTransport it(nh);

	if (left_camera_)
		imagePub = it.advertise("camera/left_image", 1);
	else
		imagePub = it.advertise("camera/right_image", 1);


	//ros::Subscriber sub = nh.subscribe("stereo_cam_trigger", 10, stereoCamTriggerCallback);

	while (ros::ok()) {
		ros::spinOnce();

		captureFrame();
	}

	closeCamera();

	return 0;
}



