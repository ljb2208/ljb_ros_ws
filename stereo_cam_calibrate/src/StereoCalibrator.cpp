/*
 * StereoCalibrator.cpp
 *
 *  Created on: Mar 24, 2014
 *      Author: lbarnett
 */

#include "../include/StereoCalibrator.h"

StereoCalibrator::StereoCalibrator(int num_corners_x, int num_corners_y, float square_size, int window_size, int zero_zone, int max_iterations, double epsilon, double alpha) {
	// TODO Auto-generated constructor stub
	cv::namedWindow(LEFT_WINDOW, CV_WINDOW_NORMAL);
	cv::namedWindow(RIGHT_WINDOW, CV_WINDOW_NORMAL);

	this->alpha = alpha;
	this->square_size = square_size;
	this->max_iteratons = max_iterations;
	this->epsilon = epsilon;
	stereoEpsilon = epsilon / 1000;

	windowSize.width = window_size;
	windowSize.height = window_size;

	zeroZone.width = zero_zone;
	zeroZone.height = zero_zone;

	corners.width = num_corners_x;
	corners.height = num_corners_y;

	display_corners = false;

	path = "/usr/local/.stereo_calibration/";

	leftReprojectionError = 0;
	rightReprojectionError = 0;
	stereoReprojectionError = 0;

	leftCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	rightCameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	leftResult = 0;
	rightResult = 0;
}

StereoCalibrator::~StereoCalibrator() {
	// TODO Auto-generated destructor stub
}

void StereoCalibrator::setLeftImage(cv_bridge::CvImagePtr imgptr) {
	 imgptr->image.copyTo(leftImage);

	if (display_corners) {
		cv::Mat displayImage = displayCorners(imgptr, true);
		cv::imshow(LEFT_WINDOW, displayImage);
	}
	else
		cv::imshow(LEFT_WINDOW, leftImage);

	//setRightImage(imgptr);
}
void StereoCalibrator::setRightImage(cv_bridge::CvImagePtr imgptr) {
	imgptr->image.copyTo(rightImage);

	if (display_corners) {
		cv::Mat displayImage = displayCorners(imgptr, false);
		cv::imshow(RIGHT_WINDOW, displayImage);
	}
	else
		cv::imshow(RIGHT_WINDOW, rightImage);
}

int StereoCalibrator::captureImage() {
	if (leftResult > 0 && rightResult > 0) {
		std::vector<cv::Point2f> left(leftImagePointsTemp);
		std::vector<cv::Point2f> right(rightImagePointsTemp);

		leftImagePoints.push_back(left);
		rightImagePoints.push_back(right);

		leftImages.push_back(leftImage);
		rightImages.push_back(rightImage);
	}

	return leftImages.size();
}

cv::Mat StereoCalibrator::displayCorners(cv_bridge::CvImagePtr image, bool leftImage) {

	int result = findCorners(image->image, (leftImage ? leftImagePointsTemp : rightImagePointsTemp), leftImage);

	cv::Mat displayImage = cv::Mat(image->image);

	if (result > 0)
		cv::drawChessboardCorners(displayImage, corners, (leftImage ? leftImagePointsTemp : rightImagePointsTemp), result);

	return displayImage;

}

void StereoCalibrator::calibrateCameras() {
	objectPoints.clear();

	for (int i=0; i< leftImagePoints.size(); i++) {
		std::vector<cv::Point3f> corner_points;

		  for( int i = 0; i < corners.height; i++ )
		  {
			for( int j = 0; j < corners.width; j++ )
			{
				corner_points.push_back(cv::Point3f(float(j*square_size),
										float(i*square_size), 0));
			}
		  }

		  objectPoints.push_back(corner_points);
	}

	imageSize.width = leftImages[0].cols;
	imageSize.height = leftImages[0].rows;

	leftReprojectionError = calibrateCamera(true);
	rightReprojectionError = calibrateCamera(false);

	rectifiedImageSize.operator =(imageSize);

	stereoReprojectionError = cv::stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints, leftCameraMatrix, leftDistCoeffs,
			rightCameraMatrix, rightDistCoeffs, imageSize, rvecs, tvecs, evecs, fvecs,
			cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, max_iteratons, stereoEpsilon), CV_CALIB_FIX_INTRINSIC + CV_CALIB_SAME_FOCAL_LENGTH);

	cv::stereoRectify(leftCameraMatrix, leftDistCoeffs, rightCameraMatrix, rightDistCoeffs, imageSize, rvecs, tvecs,
				r1, r2, p1, p2, q, CV_CALIB_ZERO_DISPARITY, alpha, rectifiedImageSize, &validPixROI1, &validPixROI2);

	cv::initUndistortRectifyMap(leftCameraMatrix, leftDistCoeffs, r1, p1, imageSize, CV_16SC2, leftMap1, leftMap2);
	cv::initUndistortRectifyMap(rightCameraMatrix, rightDistCoeffs, r2, p2, imageSize, CV_16SC2, rightMap1, rightMap2);

	ROS_INFO("Stereo calibration complete. Stereo Reprojection Error: %f. Left Reprojection Error: %f. Right Reprojection error: %f. Save file by pressing 's'.", stereoReprojectionError, leftReprojectionError, rightReprojectionError);
	ROS_INFO("POI. Left: %i Right: %i", validPixROI1.area(), validPixROI2.area());
}

double StereoCalibrator::calibrateCamera(bool leftCamera) {

	std::vector<cv::Mat> rvecs_temp;
	std::vector<cv::Mat> tvecs_temp;

	double reprojectionError = cv::calibrateCamera(objectPoints, (leftCamera ? leftImagePoints : rightImagePoints), imageSize,
			(leftCamera ? leftCameraMatrix : rightCameraMatrix), (leftCamera ? leftDistCoeffs : rightDistCoeffs), rvecs_temp, tvecs_temp);

	return reprojectionError;
}

void StereoCalibrator::toggleDisplayCorners() {
	display_corners = !display_corners;
}

int StereoCalibrator::findCorners(cv::Mat image, std::vector<cv::Point2f> &points, bool leftImage) {

	cv::Mat gsImage;

	if (image.channels() > 1) {
		gsImage = cv::Mat(image.rows, image.cols, CV_8UC1);
		cv::cvtColor(image, gsImage,CV_BGR2GRAY);
	}
	else
		gsImage = cv::Mat(image);

	int result = cv::findChessboardCorners(image, corners, points,
											CV_CALIB_CB_ADAPTIVE_THRESH |
											CV_CALIB_CB_NORMALIZE_IMAGE);

	if (result  > 0) {
		cv::cornerSubPix(gsImage, points, windowSize, zeroZone, cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, max_iteratons, epsilon ));
	}

	gsImage.release();

	if (leftImage)
		leftResult = result;
	else {
		rightResult = result;
	}

	return result;

}

void StereoCalibrator::checkCalibration() {
	std::vector<cv::Vec3f> lines[2];

	double error = 0;
	int npoints = 0;

	for (int i=0; i < leftImagePoints.size(); i++) {

		int npt = leftImagePoints[i].size();

		checkCalibrationPerCamera(true, lines[0], i);
		checkCalibrationPerCamera(false, lines[1], i);


		for( int j = 0; j < npt; j++ )
		{
			cv::Point2f pnt = leftImagePoints[i][j];
			cv::Vec3f v = lines[1][j];
			float val = v[0];
			float val1 = ((cv::Vec3f)(lines[1][j]))[0];

			double errij = std::fabs(leftImagePoints[i][j].x*((cv::Vec3f)(lines[1][j]))[0] +
									 leftImagePoints[i][j].y*((cv::Vec3f)(lines[1][j]))[1] +
									((cv::Vec3f)(lines[1][j]))[2]) +
							   std::fabs(rightImagePoints[i][j].x*((cv::Vec3f)(lines[0][j]))[0] +
									rightImagePoints[i][j].y*((cv::Vec3f)(lines[0][j]))[1] +
									((cv::Vec3f)(lines[0][j]))[2]);
			error += errij;

		}
		npoints += npt;
	}

	ROS_INFO("Average reprojection error: %f", error/npoints);
}

void StereoCalibrator::checkCalibrationPerCamera(bool leftCamera, std::vector<cv::Vec3f> lines, int imageIndex) {

	cv::Mat imgpt = cv::Mat((leftCamera ? leftImagePoints[imageIndex] : rightImagePoints[imageIndex]));

	cv::undistortPoints(imgpt, imgpt, (leftCamera ? leftCameraMatrix : rightCameraMatrix),
				(leftCamera ? leftDistCoeffs : rightDistCoeffs), cv::Mat(),
				(leftCamera ? leftCameraMatrix : rightCameraMatrix));

	ROS_INFO("Image points depth: %i", imgpt.depth());
	ROS_INFO("points check vector: %i", imgpt.checkVector(2));

	cv::computeCorrespondEpilines(imgpt, (leftCamera ? 1 : 2), fvecs, lines);
}

void StereoCalibrator::saveCalibrationToFile( std::string fileName) {
	std::string fullFileName;
	fullFileName.append(path);
	fullFileName.append(fileName);

	checkIfFileExists(fileName);

	cv::FileStorage store(fullFileName, cv::FileStorage::WRITE);

	store << "leftDistCoeffs" << leftDistCoeffs;
	store << "leftCameraMatrix" << leftCameraMatrix;
	store << "rightDistCoeffs" << rightDistCoeffs;
	store << "rightCameraMatrix" << rightCameraMatrix;
	store << "rvecs" << rvecs;
	store << "tvecs" << tvecs;
	store << "leftReprojectionError" << leftReprojectionError;
	store << "rightReprojectionError" << rightReprojectionError;
	store << "stereoReprojectionError" << stereoReprojectionError;
	store << "r1" << r1;
	store << "r2" << r2;
	store << "p1" << p1;
	store << "p2" << p2;
	store << "q" << q;
	store << "rectifiedImageSize" << rectifiedImageSize;
	store << "validPixROI1" << validPixROI1;
	store << "validPixROI2" << validPixROI2;
	store << "leftMap1" << leftMap1;
	store << "leftMap2" << leftMap2;
	store << "rightMap1" << rightMap1;
	store << "rightMap2" << rightMap2;

	store.release();

	ROS_INFO("Calibration file saved. %s", fullFileName.c_str());
}

void StereoCalibrator::checkIfFileExists(std::string fileName) {
	std::string name = getFullPathName(fileName);
	std::ifstream f(name.c_str());
	    if (f.good()) {
	        f.close();
	        long ticks = std::clock();
	        std::stringstream newFileName;
	        newFileName << getFullPathName(fileName);
	        newFileName << ticks;
	        std::rename(fileName.c_str(), newFileName.str().c_str());

	    } else {
	        f.close();
	    }
}

std::string StereoCalibrator::getFullPathName(std::string fileName) {
	std::string fullFileName;
	fullFileName.append(path);
	fullFileName.append(fileName);

	return fullFileName;
}

