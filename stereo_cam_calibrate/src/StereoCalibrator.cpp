/*
 * StereoCalibrator.cpp
 *
 *  Created on: Mar 24, 2014
 *      Author: lbarnett
 */

#include "../include/StereoCalibrator.h"

StereoCalibrator::StereoCalibrator(int num_corners_x, int num_corners_y, int square_size, int window_size, int zero_zone, int max_iterations, double epsilon) {
	// TODO Auto-generated constructor stub
	cv::namedWindow(LEFT_WINDOW, CV_WINDOW_NORMAL);
	cv::namedWindow(RIGHT_WINDOW, CV_WINDOW_NORMAL);

	this->square_size = square_size;
	this->max_iteratons = max_iterations;
	this->epsilon = epsilon;

	windowSize.width = window_size;
	windowSize.height = window_size;

	zeroZone.width = zero_zone;
	zeroZone.height = zero_zone;

	corners.width = num_corners_x;
	corners.height = num_corners_y;

	display_corners = false;
}

StereoCalibrator::~StereoCalibrator() {
	// TODO Auto-generated destructor stub
}

void StereoCalibrator::setLeftImage(cv_bridge::CvImagePtr imgptr) {
	leftImage = imgptr;

	if (display_corners) {
		cv::Mat displayImage = displayCorners(imgptr);
		cv::imshow(LEFT_WINDOW, displayImage);
	}
	else
		cv::imshow(LEFT_WINDOW, imgptr->image);
}
void StereoCalibrator::setRightImage(cv_bridge::CvImagePtr imgptr) {
	rightImage = imgptr;

	if (display_corners) {
		cv::Mat displayImage = displayCorners(imgptr);
		cv::imshow(RIGHT_WINDOW, displayImage);
	}
	else
		cv::imshow(LEFT_WINDOW, imgptr->image);
}

int StereoCalibrator::captureImage() {
	leftImages.push_back(leftImage);
	rightImages.push_back(rightImage);

	std::vector<cv::Point2f> leftPoints, rightPoints;
	findCorners(leftImage, leftPoints, true, true);
	findCorners(rightImage, rightPoints, true, false);

	return leftImages.size();
}

cv::Mat StereoCalibrator::displayCorners(cv_bridge::CvImagePtr image) {

	std::vector<cv::Point2f> points;

	int result = findCorners(image, points, false, false);

	cv::Mat displayImage = cv::Mat(image->image);

	if (result > 0)
		cv::drawChessboardCorners(displayImage, corners, points, result);

	return displayImage;

}

void StereoCalibrator::calibrateCameras() {
	std::vector<std::vector<cv::Point2f> > objectPoints;
	objectPoints.resize(leftImagePoints.size());

	for (int i=0; i< leftImagePoints.size(); i++) {
		for (int x=0; x < corners.width; x++)
			for (int y=0; y < corners.height; y++)
				objectPoints.at(i).push_back(cv::Point2f(x +(x*corners.width), y + (y * corners.height)));
	}

	double reprojectionError = cv::stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints, leftCameraMatrix, leftDistCoeffs,
			rightCameraMatrix, rightDistCoeffs, imageSize, rvecs, tvecs, evecs, fvecs);
}

void StereoCalibrator::toggleDisplayCorners() {
	display_corners = !display_corners;
}

int StereoCalibrator::findCorners(cv_bridge::CvImagePtr image, std::vector<cv::Point2f> &points, bool store, bool leftImage) {

	cv::Mat gsImage;

	if (image->image.channels() > 1)
		gsImage = cv::Mat(image->image.rows, image->image.cols, CV_8UC1);
	else
		gsImage = cv::Mat(image->image);


	cv::cvtColor(image->image, gsImage,CV_BGR2GRAY);


	int result = cv::findChessboardCorners(image->image, corners, points,
											CV_CALIB_CB_ADAPTIVE_THRESH |
											CV_CALIB_CB_NORMALIZE_IMAGE);

	if (points.size() < 1)
		return result;

	cv::cornerSubPix(gsImage, points, windowSize, zeroZone, cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, max_iteratons, epsilon ));

	if (store)
		if (leftImage)
			leftImagePoints.push_back(points);
		else
			rightImagePoints.push_back(points);

	return result;

}
