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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <fstream>
#include <time.h>
#include <string>

#include "ros/ros.h"

static const std::string LEFT_WINDOW = "Left Image";
static const std::string RIGHT_WINDOW = "Right Image";

class StereoCalibrator {
public:
	StereoCalibrator(int num_corners_x, int num_corners_y, float square_size, int window_size, int zero_zone, int max_iterations, double epsilon, double alpha);
	virtual ~StereoCalibrator();
	void setLeftImage(cv_bridge::CvImagePtr imgptr);
	void setRightImage(cv_bridge::CvImagePtr imgptr);
	int captureImage();
	void toggleDisplayCorners();
	void calibrateCameras();
	void saveCalibrationToFile(std::string fileName);
	void checkCalibration();


	double leftReprojectionError;
	double rightReprojectionError;
	double stereoReprojectionError;


private:
	cv::Mat leftImage;
	cv::Mat rightImage;

	CvSize corners;
	CvSize imageSize;
	CvSize windowSize;
	CvSize zeroZone;

	int max_iteratons;
	double epsilon;
	double stereoEpsilon;
	float square_size;

	std::string path;

	std::vector<cv::Mat> leftImages;
	std::vector<cv::Mat> rightImages;

	std::vector<std::vector<cv::Point2f> > leftImagePoints;
	std::vector<std::vector<cv::Point2f> > rightImagePoints;

	std::vector<cv::Point2f> leftImagePointsTemp;
	std::vector<cv::Point2f> rightImagePointsTemp;

	int leftResult;
	int rightResult;

	cv::Mat leftCameraMatrix;
	cv::Mat leftDistCoeffs;
	cv::Mat rightCameraMatrix;
	cv::Mat rightDistCoeffs;
	cv::Mat rvecs;
	cv::Mat tvecs;
	cv::Mat evecs;
	cv::Mat fvecs;

	//std::vector<std::vector<cv::Point3f>> testPoints;
	std::vector<std::vector<cv::Point3f> > objectPoints;

	cv::Mat displayCorners(cv_bridge::CvImagePtr image, bool leftImage);
	int findCorners(cv::Mat image, std::vector<cv::Point2f> &points, bool leftImage);
	double calibrateCamera(bool leftCamera);


	void checkCalibrationPerCamera(bool leftCamera, std::vector<cv::Vec3f> lines, int imageIndex);

	void checkIfFileExists(std::string fileName);
	std::string getFullPathName(std::string fileName);

	bool display_corners;

	cv::Mat r1;
	cv::Mat r2;
	cv::Mat p1;
	cv::Mat p2;
	cv::Mat q;

	cv::Size rectifiedImageSize;

	cv::Rect validPixROI1;
	cv::Rect validPixROI2;


	cv::Mat leftMap1;
	cv::Mat leftMap2;
	cv::Mat rightMap1;
	cv::Mat rightMap2;

	double alpha;
};

#endif /* STEREOCALIBRATOR_H_ */
