/*
 * image_scaler.cpp
 *
 *  Created on: Mar 23, 2014
 *      Author: lbarnett
 */

#include "../include/image_scaler_node.h"

Pixel getPixel(unsigned char *img, float x, float y, int width) {
	 int px = (int)x; // floor of x
	 int py = (int)y; // floor of y
	 int stride = width;
	 Pixel* p0 = (Pixel*)img + px + py * stride; // pointer to first pixel

	 // load the four neighboring pixels
	 Pixel& p1 = p0[0];
	 Pixel& p2 = p0[1];
	 Pixel& p3 = p0[stride];
	 Pixel& p4 = p0[1 + 1 * stride];

	 // Calculate the weights for each pixel
	 float fx = x - px;
	 float fy = y - py;
	 float fx1 = 1.0f - fx;
	 float fy1 = 1.0f - fy;

	 int w1 = fx1 * fy1 * 256.0f;
	 int w2 = fx  * fy1 * 256.0f;
	 int w3 = fx1 * fy  * 256.0f;
	 int w4 = fx  * fy  * 256.0f;

	 // Calculate the weighted sum of pixels (for each color channel)
	 int outb = p1.b * w1 + p2.b * w2 + p3.b * w3 + p4.b * w4;
	 int outg = p1.g * w1 + p2.g * w2 + p3.g * w3 + p4.g * w4;
	 int outr = p1.r * w1 + p2.r * w2 + p3.r * w3 + p4.r * w4;

	 return Pixel(outb >> 8, outg >> 8, outr >> 8);
}

unsigned char getGSPixel(unsigned char *img, float x, float y, int width) {
	 int px = (int)x; // floor of x
	 int py = (int)y; // floor of y
	 int stride = width;
	 unsigned char* p0 = (unsigned char*)img + px + py * stride; // pointer to first pixel

	 // load the four neighboring pixels
	 unsigned char& p1 = p0[0];
	 unsigned char& p2 = p0[1];
	 unsigned char& p3 = p0[stride];
	 unsigned char& p4 = p0[1 + 1 * stride];

	 // Calculate the weights for each pixel
	 float fx = x - px;
	 float fy = y - py;
	 float fx1 = 1.0f - fx;
	 float fy1 = 1.0f - fy;

	 int w1 = fx1 * fy1 * 256.0f;
	 int w2 = fx  * fy1 * 256.0f;
	 int w3 = fx1 * fy  * 256.0f;
	 int w4 = fx  * fy  * 256.0f;

	 // Calculate the weighted sum of pixels (for single color channel)
	 int out = p1 * w1 + p2 * w2 + p3 * w3 + p4 * w4;

	 return (unsigned char)(out >> 8);
}

void bgr2thumbnail(unsigned char *BGR, unsigned char *TN, int height, int width) {
	int newWidth = width / scaling_;
	int newHeight = height / scaling_;
	int step = newWidth * 3;

	float dx, dy;

	int i, y;

	dy = scaling_ / 2;

	for (i=0; i < newHeight; i++) {
		y = 0;
		dx = scaling_ / 2;

		for (int z=0; z < newWidth; z++){
			Pixel pixel = getPixel(BGR, dx, dy, width);
			TN[i*step + y] = pixel.b;
			TN[i*step + y + 1] = pixel.g;
			TN[i*step + y + 2] = pixel.r;
			y += 3;
			dx += scaling_;
		}

		dy += scaling_;
	}
}

void gs2thumbnail(unsigned char *GS, unsigned char *TN, int height, int width) {
	int newWidth = width / scaling_;
	int newHeight = height / scaling_;
	int step = newWidth;

	int tx = (float) width /newWidth ;
	int ty =  (float)height / newHeight;

	float dx, dy;

	int i, y;

	dy = ty / 2;

	for (i=0; i < newHeight; i++) {
		y = 0;
		dx = tx / 2;

		for (int z=0; z < newWidth; z++){
			unsigned char pixel = getGSPixel(GS, dx, dy, width);
			TN[i*step + y] = pixel;
			y++;
			dx += tx;
		}

		dy += ty;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	sensor_msgs::ImagePtr msgTN = boost::make_shared<sensor_msgs::Image>();

	int newHeight = msg->height / scaling_;
	int newWidth = msg->width / scaling_;

	msgTN->width = newWidth;
	msgTN->height = newHeight;
	msgTN->is_bigendian = msg->is_bigendian;

	if (msg->encoding == "bgr8") {
		msgTN->encoding = "bgr8";
		msgTN->step = newWidth * 3;

	} else if (msg->encoding == "mono8") {
		msgTN->encoding = "mono8";
		msgTN->step = newWidth;
	}

	int charCount = msgTN->height * msgTN->step;
	msgTN->data.resize(charCount);
	unsigned char* tnImage = (unsigned char*) malloc(charCount);

	if (msg->encoding == "bgr8") {
		bgr2thumbnail((unsigned char*)(&msg->data[0]), tnImage, msg->height, msg->width);
	} else if (msg->encoding == "mono8") {
		gs2thumbnail((unsigned char*)(&msg->data[0]), tnImage, msg->height, msg->width);
	}

	memcpy((unsigned char*)(&msgTN->data[0]), tnImage, charCount);
	imagePub.publish(msgTN);
	free(tnImage);
}


int main(int argc, char **argv){

	ros::init(argc, argv, "image_scaler", ros::init_options::AnonymousName);

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("image_topic", image_topic_, std::string(""));
	nh_private.param("scaling", scaling_, int(4));

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub = it.subscribe(image_topic_, 1, imageCallback);

	image_topic_.append("_thumbnail");
	imagePub = it.advertise(image_topic_, 1);

	ros::spin();

	return 0;
}
