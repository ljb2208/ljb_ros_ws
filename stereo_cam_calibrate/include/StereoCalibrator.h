/*
 * StereoCalibrator.h
 *
 *  Created on: Mar 24, 2014
 *      Author: lbarnett
 */

#ifndef STEREOCALIBRATOR_H_
#define STEREOCALIBRATOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

static const std::string LEFT_WINDOW = "Left Image";
static const std::string RIGHT_WINDOW = "Right Image";

class StereoCalibrator {
public:
	StereoCalibrator(int num_corners_x, int num_corners_y, int square_size, int window_size, int zero_zone, int max_iterations, double epsilon);
	virtual ~StereoCalibrator();
	void setLeftImage(cv_bridge::CvImagePtr imgptr);
	void setRightImage(cv_bridge::CvImagePtr imgptr);
	int captureImage();
	void toggleDisplayCorners();
	void calibrateCameras();

private:
	cv_bridge::CvImagePtr leftImage;
	cv_bridge::CvImagePtr rightImage;

	CvSize corners;
	CvSize imageSize;
	CvSize windowSize;
	CvSize zeroZone;

	int max_iteratons;
	double epsilon;
	int square_size;

	std::vector<cv_bridge::CvImagePtr> leftImages;
	std::vector<cv_bridge::CvImagePtr> rightImages;

	std::vector<std::vector<cv::Point2f> > leftImagePoints;
	std::vector<std::vector<cv::Point2f> > rightImagePoints;

	cv::Mat leftCameraMatrix;
	cv::Mat leftDistCoeffs;
	cv::Mat rightCameraMatrix;
	cv::Mat rightDistCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	std::vector<cv::Mat> evecs;
	std::vector<cv::Mat> fvecs;

	cv::Mat displayCorners(cv_bridge::CvImagePtr image);
	int findCorners(cv_bridge::CvImagePtr image, std::vector<cv::Point2f> &points, bool store, bool leftImage);


	bool display_corners;
};

#endif /* STEREOCALIBRATOR_H_ */
