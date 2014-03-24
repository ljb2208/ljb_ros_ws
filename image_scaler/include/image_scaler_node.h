/*
 * image_scaler.h
 *
 *  Created on: Mar 23, 2014
 *      Author: lbarnett
 */

#ifndef IMAGE_SCALER_H_
#define IMAGE_SCALER_H_

#include "ros/ros.h"
#include <image_transport/image_transport.h>

std::string image_topic_;
int scaling_;
image_transport::Publisher imagePub;

struct Pixel
{
	Pixel() :b(0), g(0), r(0){}
	Pixel(int bgr) { *((int*)this) = bgr; }
	Pixel(unsigned char b, unsigned char g, unsigned char r) : b(b), g(g), r(r) {}

 unsigned char b, g, r;
};

#endif /* IMAGE_SCALER_H_ */
