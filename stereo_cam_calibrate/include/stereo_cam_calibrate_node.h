/*
 * stereo_cam_calibrate_node.h
 *
 *  Created on: Mar 24, 2014
 *      Author: lbarnett
 */

#ifndef STEREO_CAM_CALIBRATE_NODE_H_
#define STEREO_CAM_CALIBRATE_NODE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "StereoCalibrator.h"

std::string left_image_topic_;
std::string right_image_topic_;

int num_corners_x_;
int num_corners_y_;
int window_size_;
int zero_zone_;
int max_iterations_;
double epsilon_;
int square_size_; // in cm

StereoCalibrator* stereo_calib;


#endif /* STEREO_CAM_CALIBRATE_NODE_H_ */
