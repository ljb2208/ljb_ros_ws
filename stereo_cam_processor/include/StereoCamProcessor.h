/*
 * StereoCamProcessor.h
 *
 *  Created on: Apr 3, 2014
 *      Author: lbarnett
 */

#ifndef STEREOCAMPROCESSOR_H_
#define STEREOCAMPROCESSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <fstream>
#include <time.h>
#include <string>

static const std::string LEFT_WINDOW = "Left Image";
static const std::string RIGHT_WINDOW = "Right Image";
static const std::string DISP_WINDOW = "Disp Image";
static const std::string DISP_WINDOW_GS = "Disp Image GS";
static const std::string THREED_WINDOW = "3d Image";
static const std::string LEFT_IMAGE_WINDOW = "Left U Image";


class StereoCamProcessor {
public:
	StereoCamProcessor(bool showImages, ros::Publisher pub);
	virtual ~StereoCamProcessor();
	void loadCalibration(std::string fileName);
	void setLeftImage(cv_bridge::CvImagePtr imgptr);
	void setRightImage(cv_bridge::CvImagePtr imgptr);

	void setPreset(int preset);
	void setNumDisp(int value);
	void setSadWindowSize(int value);
	void reInitialize();
	void setCurrentTransform(tf::Transform currentTransform);
	void setBlockMatchParams(bool useBM, int preset, int minDisp, int numDisp, int sadWindowSize, bool dispSmoothnessAuto, int dispSmoothness1,
			int dispSmoothness2, int dispSmoothnessMaxDiff,  int preFilterCap, int uniquenessRatio,
			int speckleWindowSize, int speckleRange, bool fullDP);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pubCloud;

private:

	std::string getFullPathName(std::string fileName);
	int numDisp; // must be divisble by 16
	int sadWindowSize; // must be odd number: normally between 3 and 11
	int preset;
	int minDisp; //normally zero
	bool dispSmoothnessAuto;
	int dispSmoothness1; // penalty on the disparity change by plus or minus one between neighbour pixels
	int dispSmoothness2; // penalty on the disparity change by more than 1 between neighber pixels. this needs to be greater than above
	int dispSmoothnessMaxDiff; // maximum allowed difference between left and right pixels. -1 to disable
	int preFilterCap;
	int uniquenessRatio; // 5 to 15 is normal
	int speckleWindowSize; //maxmimum size of smooth disparity regions - set to 0 to disable otherwise 50 to 200
	int speckleRange; //Maximum disparity variation within each connected component, 1 to 2 normally
	bool fullDP; // normally false

	cv::Mat leftCameraMatrix;
	cv::Mat leftDistCoeffs;
	cv::Mat rightCameraMatrix;
	cv::Mat rightDistCoeffs;
	cv::Mat rvecs;
	cv::Mat tvecs;
	cv::Mat q;

	cv::Mat leftImage;
	cv::Mat rightImage;

	cv::Mat leftImageGS;
	cv::Mat rightImageGS;

	cv::Mat leftImageGSRemap;
	cv::Mat rightImageGSRemap;


	std::string path;

	cv::Size imageSize;

	cv::StereoBM stereoBM;
	cv::StereoSGBM stereoSGBM;

	cv::Mat dispImage;
	cv::Mat dispImageGS;

	cv::Mat threeDImage;

	cv::Mat leftMap1;
	cv::Mat leftMap2;
	cv::Mat rightMap1;
	cv::Mat rightMap2;

	cv::Rect validPixROI1;
	cv::Rect validPixROI2;


	void calculateDisparity();
	void reprojectTo3D();
	void convertToPointCloud();
	void runBlockMatch();
	void constructBlockMatch();

	bool leftImageReceived;
	bool rightImageReceived;

	bool initialized;
	bool showImages;

	bool useBM;

	ros::Publisher pub;

	tf::TransformBroadcaster tfbroadcast;
	tf::Transform transform;



};

#endif /* STEREOCAMPROCESSOR_H_ */
