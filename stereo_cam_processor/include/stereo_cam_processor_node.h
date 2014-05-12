/*
 * stereo_cam_processor_node.h
 *
 *  Created on: Mar 24, 2014
 *      Author: lbarnett
 */

#ifndef STEREO_CAM_PROCESSOR_NODE_H_
#define STEREO_CAM_PROCESSOR_NODE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "StereoCamProcessor.h"
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <boost/any.hpp>
#include "../../devel/include/stereo_cam_processor/stereo_cam_processorConfig.h"


std::string left_image_topic_;
std::string right_image_topic_;

int num_disparities_;
int sad_window_size_;
int preset_;
bool show_images_;

int min_disp_; //normally zero
int disp_smoothness1_; // penalty on the disparity change by plus or minus one between neighbour pixels
int disp_smoothness2_; // penalty on the disparity change by more than 1 between neighber pixels. this needs to be greater than above
int disp_smoothness_maxdiff_; // maximum allowed difference between left and right pixels. -1 to disable
int prefilter_cap_;
int uniqueness_ratio_; // 5 to 15 is normal
int speckle_window_size_; //maxmimum size of smooth disparity regions - set to 0 to disable otherwise 50 to 200
int speckle_range_; //Maximum disparity variation within each connected component, 1 to 2 normally
bool full_dp_; // normally false

bool use_bm_;
bool disp_smoothness_auto_;


StereoCamProcessor* stereoProc;


#endif /* STEREO_CAM_PROCESSOR_NODE_H_ */
