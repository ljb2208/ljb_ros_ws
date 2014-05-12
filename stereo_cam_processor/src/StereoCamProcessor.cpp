/*
 * StereoCamProcessor.cpp
 *
 *  Created on: Apr 3, 2014
 *      Author: lbarnett
 */

#include "../include/StereoCamProcessor.h"

StereoCamProcessor::StereoCamProcessor(bool showImages, ros::Publisher pub) {

	path = "/usr/local/.stereo_calibration/";

	leftImageReceived = false;
	rightImageReceived = false;

	initialized = false;

	if (showImages) {
		cv::namedWindow(LEFT_WINDOW, CV_WINDOW_NORMAL);
		cv::namedWindow(RIGHT_WINDOW, CV_WINDOW_NORMAL);
		cv::namedWindow(DISP_WINDOW, CV_WINDOW_NORMAL);
		cv::namedWindow(DISP_WINDOW_GS, CV_WINDOW_NORMAL);
		cv::namedWindow(THREED_WINDOW, CV_WINDOW_NORMAL);
		cv::namedWindow(LEFT_IMAGE_WINDOW, CV_WINDOW_NORMAL);
	}
}

StereoCamProcessor::~StereoCamProcessor() {
	// TODO Auto-generated destructor stub
}

void StereoCamProcessor::constructBlockMatch() {
	if (useBM) {
		stereoBM = cv::StereoBM(preset, numDisp * 16, (sadWindowSize * 2) + 1);
	}
	else {

		if (dispSmoothness2 <= dispSmoothness1) {
			ROS_WARN("dispSmoothness2 must be greater than dispSmoothness1.");
		}

		stereoSGBM = cv::StereoSGBM(minDisp, numDisp * 16, (sadWindowSize * 2) + 1, dispSmoothness1, dispSmoothness2, dispSmoothnessMaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, fullDP);
	}
}

void StereoCamProcessor::setBlockMatchParams(bool useBM, int preset, int minDisp, int numDisp, int sadWindowSize, bool dispSmoothnessAuto, int dispSmoothness1,
			int dispSmoothness2, int dispSmoothnessMaxDiff,  int preFilterCap, int uniquenessRatio,
			int speckleWindowSize, int speckleRange, bool fullDP) {

	this->numDisp = numDisp;
	this->sadWindowSize = sadWindowSize;
	this->preset = preset;
	this->showImages = showImages;
	this->pub = pub;
	this->minDisp = minDisp;
	this->dispSmoothness1 = dispSmoothness1;
	this->dispSmoothness2 = dispSmoothness2;
	this->dispSmoothnessMaxDiff = dispSmoothnessMaxDiff;
	this->preFilterCap = preFilterCap;
	this->uniquenessRatio = uniquenessRatio;
	this->speckleWindowSize = speckleWindowSize;
	this->speckleRange = speckleRange;
	this->fullDP = fullDP;
	this->useBM = useBM;
	this->dispSmoothnessAuto = dispSmoothnessAuto;

	constructBlockMatch();
	calculateDisparity();

}

void StereoCamProcessor::loadCalibration(std::string fileName) {
	std::string fullFileName = getFullPathName(fileName);

	cv::FileStorage store(fullFileName, cv::FileStorage::READ);

	cv::Mat qTemp;
	store["leftMap1"] >> leftMap1;
	store["leftMap2"] >> leftMap2;
	store["rightMap1"] >> rightMap1;
	store["rightMap2"] >> rightMap2;
	store["q"] >> qTemp;
	qTemp.convertTo(q, CV_32F);

	cv::FileNode rect =store["validPixROI1"];
	validPixROI1.x = (int) rect[0];
	validPixROI1.y = (int) rect[1];
	validPixROI1.width = (int) rect[2];
	validPixROI1.height = (int) rect[3];

	cv::FileNode rect2 =store["validPixROI2"];
	validPixROI2.x = (int) rect2[0];
	validPixROI2.y = (int) rect2[1];
	validPixROI2.width = (int) rect2[2];
	validPixROI2.height = (int) rect2[3];

	ROS_INFO("ValidPixROI1 Size: %i ", validPixROI1.area());
	ROS_INFO("ValidPixROI2 Size: %i ", validPixROI2.area());

	store.release();
}

std::string StereoCamProcessor::getFullPathName(std::string fileName) {
	std::string fullFileName;
	fullFileName.append(path);
	fullFileName.append(fileName);

	return fullFileName;
}

void StereoCamProcessor::setLeftImage(cv_bridge::CvImagePtr imgptr) {

	imgptr->image.copyTo(leftImage);

	if (imgptr->image.channels() == 3) {
		leftImageGS = cv::Mat(leftImage.rows, leftImage.cols, CV_8UC1);
		cv::cvtColor(leftImage, leftImageGS ,CV_BGR2GRAY);
	} else {
		leftImage.copyTo(leftImageGS);
	}

	leftImageGSRemap = cv::Mat(leftImage.rows, leftImage.cols, CV_8UC1);

	cv::remap(leftImageGS, leftImageGSRemap, leftMap1, leftMap2, CV_INTER_LINEAR, cv::BORDER_CONSTANT, 0);
	leftImageReceived = true;

	if (showImages) {
		cv::imshow(LEFT_WINDOW, leftImageGSRemap);
		cv::imshow(LEFT_IMAGE_WINDOW, leftImage);
	}

	calculateDisparity();
}

void StereoCamProcessor::setRightImage(cv_bridge::CvImagePtr imgptr) {

	imgptr->image.copyTo(rightImage);

	if (rightImage.channels() == 3) {
		rightImageGS = cv::Mat(rightImage.rows, rightImage.cols, CV_8UC1);
		cv::cvtColor(rightImage, rightImageGS ,CV_BGR2GRAY);
	} else {
		rightImage.copyTo(rightImageGS);
	}

	rightImageGSRemap = cv::Mat(rightImage.rows, rightImage.cols, CV_8UC1);
	cv::remap(rightImageGS, rightImageGSRemap, rightMap1, rightMap2, CV_INTER_LINEAR, cv::BORDER_CONSTANT,0);
	rightImageReceived = true;

	if (showImages)
		cv::imshow(RIGHT_WINDOW, rightImageGSRemap);

	calculateDisparity();
}

void StereoCamProcessor::calculateDisparity() {

	if (!leftImageReceived || !rightImageReceived)
		return;

	if (!initialized) {
		initialized = true;
		dispImage  = cv::Mat(leftImage.rows, leftImage.cols, CV_32F);
		dispImageGS  = cv::Mat(leftImage.rows, leftImage.cols, CV_8UC1);
		threeDImage = cv::Mat(leftImage.rows, leftImage.cols, CV_32FC3);
	}

	runBlockMatch();

	double minVal, maxVal;
	cv::minMaxLoc(dispImage, &minVal, &maxVal);

	//cv::normalize(dispImage, dispImageGS, 0, 255, CV_MINMAX, CV_8U);
	//cv::convertScaleAbs(dispImage, dispImageGS, 255 / maxVal);
	//dispImage.convertTo(dispImageGS, CV_8UC1);//, 255/(maxVal - minVal));
	dispImage.convertTo(dispImageGS, CV_8UC1, 255/(maxVal - minVal), -minVal* (maxVal - minVal));
	cv::Mat falseDispImage;
	cv::applyColorMap(dispImageGS, falseDispImage, cv::COLORMAP_AUTUMN);
	//dispImage.convertTo(dispImageGS, CV_8UC1, -255.0/maxVal, 255);

	if (showImages) {
		cv::imshow(DISP_WINDOW_GS, falseDispImage);
		cv::imshow(DISP_WINDOW, dispImage);
	}

	reprojectTo3D();

	leftImageReceived = false;
	rightImageReceived = false;
}

void StereoCamProcessor::runBlockMatch	() {
	if (useBM)
		stereoBM(leftImageGSRemap, rightImageGSRemap, dispImage, CV_32F);
	else
		stereoSGBM(leftImageGSRemap, rightImageGSRemap, dispImage);
}

void StereoCamProcessor::reprojectTo3D() {
	cv::reprojectImageTo3D(dispImage, threeDImage, q, 0, CV_32F);

	/*
	cv::Mat_<float> vec(4,1);

	for(int y=0; y<dispImage.rows; ++y) {
	    for(int x=0; x<dispImage.cols; ++x) {
	        vec(0)=x; vec(1)=y; vec(2)=dispImage.at<float>(y, x); vec(3)=1;

	        //if (vec(2) > 0)
	        //	ROS_INFO("Vec 2: %f", vec(2));
	        vec = q*vec;
	        //if (vec(2) > 0)
	        //	ROS_INFO("Vec 1: %f 2: %f 3: %f 4:%f" , vec(0), vec(1), vec(2), vec(3));
	        vec /= vec(3);
	        //if (vec(2) > 0)
	        //	ROS_INFO("Vec2 1: %f 2: %f 3: %f 4:%f" , vec(0), vec(1), vec(2), vec(3));
	        cv::Vec3f &point = threeDImage.at<cv::Vec3f>(y, x);
	        point[0] = vec(0);
	        point[1] = vec(1);
	        point[2] = vec(2);
	    }
	}
	*/

	if (showImages)
		cv::imshow(THREED_WINDOW, threeDImage);

	convertToPointCloud();
}

void StereoCamProcessor::convertToPointCloud() {
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.width = validPixROI1.width;
	cloud.height = validPixROI1.height;
	cloud.is_dense = true;
	cloud.points.resize(validPixROI1.area());
	cv::Mat color = leftImage;

	int counter = 0;
	int ignoreCounter = 0;

	float min, max;

	uint32_t minrgb, maxrgb;

	min = 1000;
	max = 0;

	for (size_t v = 0; v < cloud.height; v++)
	{
		for (size_t u = 0; u < cloud.width; u++)
		{
				cv::Vec3f p = threeDImage.at<cv::Vec3f> (v+validPixROI1.y, u+validPixROI1.x); //row,column axis, y,x
				p[2] *= -1;

				pcl::PointXYZRGB& cp = cloud(u, v);

				if (p[2] == 10000 || p[2] < 0)
				{
					cp.x = cp.y = cp.z = NAN;
					ignoreCounter++;
					continue;
				}

				const cv::Vec3b & colorp = color.at<cv::Vec3b> (v+validPixROI1.y, u+validPixROI1.x);
				uint32_t rgb = ((uint32_t)colorp(2) << 16 | (uint32_t)colorp(1) << 8 | (uint32_t)colorp(0));

				//memcpy(&cp.rgb, &colorp, sizeof(colorp));
				cp.rgb = *reinterpret_cast<float*>(&rgb);

				cp.x = p[0], cp.y = p[1], cp.z = p[2];

				if (cp.z < min)
					min = cp.z;

				if (cp.z > max)
					max = cp.z;

				counter++;
		}
	}

	if (counter > 0) {
		ROS_INFO("PCL created with depth. Points: %i Ignored Points: %i Min: %f Max: %f", counter, ignoreCounter, min, max);


		sensor_msgs::PointCloud2 msg;

		/*
		pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);

		*/
		cv::FileStorage store("dispImages.yml", cv::FileStorage::WRITE);

		store << "dispImage" << dispImage;
		store << "dispImageGS" << dispImageGS;
		store << "threeDImage" << threeDImage;

		store.release();

		pcl::toROSMsg(cloud, msg);
		msg.header.frame_id = "3dcam";
		msg.header.stamp = ros::Time::now();

		tfbroadcast.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "world", "3dcam"));
		//cloud.toMsg(&msg);

		pub.publish(msg);
	}
}

void StereoCamProcessor::setCurrentTransform(tf::Transform currentTransform) {
	transform = currentTransform;
}

void StereoCamProcessor::setNumDisp(int value) {
	numDisp = value;

}

void StereoCamProcessor::setSadWindowSize(int value) {
	sadWindowSize = value;
	stereoBM.init(preset, numDisp, sadWindowSize);
}

void StereoCamProcessor::setPreset(int value) {
	preset = value;
	stereoBM.init(preset, numDisp, sadWindowSize);
}

void StereoCamProcessor::reInitialize() {
	if (useBM)
		stereoBM.init(preset, numDisp, sadWindowSize);
}
