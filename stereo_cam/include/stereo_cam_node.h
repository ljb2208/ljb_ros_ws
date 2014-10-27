/*
 * stereo_cam_node.h
 *
 *  Created on: Mar 13, 2014
 *      Author: lbarnett
 */

#ifndef STEREO_CAM_NODE_H_
#define STEREO_CAM_NODE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include "../../../devel/include/stereo_cam/stereo_cam_trigger.h"
#include "../../../devel/include/stereo_cam/stereo_cam_capture.h"
#include "v4l.h"
#include "../include/timing.h"

int camera_id_ = 0;
int width_ = 640;
int height_ = 480;
int fps_ = 30;
int scale_ = 1;
bool left_camera_ = false;
bool mono_mode_ = false;
bool gs_mode_ = false;
int seq_no_ = -1;
std::string ci_url_ = "";
image_transport::CameraPublisher imagePub;
ros::Publisher capturePub;
boost::shared_ptr<camera_info_manager::CameraInfoManager> camInfo;
V4lVideo* v4l;
std::string device_ = "/dev/video";
std::string frameId;

timing* captureTimer;

bool captureFrame(ros::Time stamp);
void closeCamera();
bool openCamera();

#endif /* STEREO_CAM_NODE_H_ */
