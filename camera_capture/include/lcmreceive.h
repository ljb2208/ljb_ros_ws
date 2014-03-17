/*
 * lcmreceive.h
 *
 *  Created on: Nov 25, 2013
 *      Author: lbarnett
 */

#ifndef LCMRECEIVE_H_
#define LCMRECEIVE_H_

#include <lcm/lcm-cpp.hpp>
#include "../../../../development/lcm/lcm_capture/lcm_messages/image_lcm/image_t.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
//#include "ros/ros.h"

class lcmreceive {
public:
	lcmreceive();
	lcmreceive(image_transport::Publisher* _image_pub);
	virtual ~lcmreceive();


	void handleMessage(const lcm::ReceiveBuffer* rbuf,
	                const std::string& chan,
	                const image_lcm::image_t* msg);


private:
	//ros::NodeHandle* nh;
	image_transport::Publisher* image_pub;

};


#endif /* LCMRECEIVE_H_ */
