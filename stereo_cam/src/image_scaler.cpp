/*
 * image_scaler.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: lbarnett
 */

#include "../include/image_scaler.h"

image_scaler::image_scaler(int scale_) {
	scale = scale_;

}

image_scaler::~image_scaler() {
	// TODO Auto-generated destructor stub
}

Pixel image_scaler::getPixel(unsigned char *img, float x, float y, int width) {
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

unsigned char image_scaler::getGSPixel(unsigned char *img, float x, float y, int width) {
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

void image_scaler::bgr2thumbnail(unsigned char *BGR, unsigned char *TN, int height, int width) {
	int newWidth = width / scale;
	int newHeight = height / scale;
	int step = newWidth * 3;

	float dx, dy;

	int i, y;

	dy = scale / 2;

	for (i=0; i < newHeight; i++) {
		y = 0;
		dx = scale / 2;

		for (int z=0; z < newWidth; z++){
			Pixel pixel = getPixel(BGR, dx, dy, width);
			TN[i*step + y] = pixel.b;
			TN[i*step + y + 1] = pixel.g;
			TN[i*step + y + 2] = pixel.r;
			y += 3;
			dx += scale;
		}

		dy += scale;
	}
}

void image_scaler::gs2thumbnail(unsigned char *GS, unsigned char *TN, int height, int width) {
	int newWidth = width / scale;
	int newHeight = height / scale;
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

