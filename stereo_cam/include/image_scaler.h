/*
 * image_scaler.h
 *
 *  Created on: Jun 2, 2014
 *      Author: lbarnett
 */

#ifndef IMAGE_SCALER_H_
#define IMAGE_SCALER_H_

struct Pixel
{
	Pixel() :b(0), g(0), r(0){}
	Pixel(int bgr) { *((int*)this) = bgr; }
	Pixel(unsigned char b, unsigned char g, unsigned char r) : b(b), g(g), r(r) {}

 unsigned char b, g, r;
};

class image_scaler {
public:
	image_scaler(int scale);
	virtual ~image_scaler();

	void gs2thumbnail(unsigned char *GS, unsigned char *TN, int height, int width);
	void bgr2thumbnail(unsigned char *BGR, unsigned char *TN, int height, int width);
	void cvgs2thumbnail(unsigned char *GS, unsigned char *TN, int height, int width);
	void cvbgr2thumbnail(unsigned char *BGR, unsigned char *TN, int height, int width);
	unsigned char getGSPixel(unsigned char *img, float x, float y, int width);
	Pixel getPixel(unsigned char *img, float x, float y, int width);


private:
	int scale;
};

#endif /* IMAGE_SCALER_H_ */
