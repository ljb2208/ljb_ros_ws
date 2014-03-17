/*
 * lcmreceive.cpp
 *
 *  Created on: Nov 25, 2013
 *      Author: lbarnett
 */

#include "../include/lcmreceive.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.h"

lcmreceive::lcmreceive()
{

}

lcmreceive::lcmreceive(image_transport::Publisher* _image_pub)
{
	image_pub = _image_pub;
}

lcmreceive::~lcmreceive() {

}
/*
lcmreceive::lcmreceive(ros::NodeHandle* _nh)
{
	nh = _nh;
}
*/

void lcmreceive::handleMessage(const lcm::ReceiveBuffer* rbuf,
	                const std::string& chan,
	                const image_lcm::image_t* msg) {

	sensor_msgs::Image image;
	image.height = msg->height;
	image.width = msg->width;
	image.step = msg->width * msg->depth;// * msg->depth;

	image.data = msg->image_data;
	image.encoding = msg->encoding;

	ROS_WARN("received lcm message. Height %i Width %i Depth %i", msg->height, msg->width, msg->depth);

	image_pub->publish(image);
}

