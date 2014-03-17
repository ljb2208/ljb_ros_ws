#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <lcm/lcm-cpp.hpp>
#include "../include/lcmreceive.h"
#include "../include/gspipe.h"

#include <sstream>

void captureImage(IplImage* image);

using namespace std;
using namespace cv;

//CvCapture* capture;
int camera_id_ = -1;
bool gray_scale_ = false;
string encoding_ = "rgb8";

int main(int argc, char **argv){
	ros::init(argc, argv, "camera_capture");

	ros::NodeHandle nh, nh_private("~");

	nh.param("camera_id", camera_id_, int(-1));
	nh.param("encoding", encoding_, string("rgb8"));
	nh.param("grayscale", gray_scale_, bool(true));

	image_transport::Publisher img_pub;
	image_transport::ImageTransport it(nh);
//	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

//	lcm::LCM lcm;

//	if (!lcm.good())
//		ROS_ERROR("Couldn't connect to lcm", NULL);
	//IplImage* image;
	//cv::Mat image;

//	lcmreceive _lcmreceive(&img_pub); //(&nh);
//	lcm.subscribe("CAM_IMAGE", &lcmreceive::handleMessage, &_lcmreceive);

	gspipe::GSPipe* pipe = new gspipe::GSPipe(nh, nh_private);
	pipe->run();
	//pipe->init_stream();

	//img_pub = it.advertise("/camera_capture/output_video", 1);

	/*while (ros::ok()) {
//		lcm.handle();
		//captureImage(image);

	//	cv::Mat greyMat, colorMat;

		//ROS_WARN("Camera encoding: %i", colorMat.type());


/*		if (gray_scale_) {
			cvtColor(colorMat, greyMat, CV_RGB2GRAY);
			greyMat.copyTo(cv_ptr->image);
			cv_ptr->encoding = "mono8";
		}
		else {*/
		//cv_bridge::toCvCopy(colorMat, vidCap.)

//		}

		/*
		if (gray_scale_) {
		//	colorMat.convertTo(greyMat, CV_8U);
			cvtColor(colorMat, greyMat, CV_RGB2GRAY);
			greyMat.copyTo(cv_ptr->image);
			cv_ptr->encoding = "mono8";
		} else {
			colorMat.copyTo(cv_ptr->image);
			cv_ptr->encoding = encoding_;
		}
		*/

		//img_pub.publish(cv_ptr->toImageMsg());

		//ros::spinOnce();
	//}

	//vidCap.release();

	return 0;
}

void captureImage(cv::Mat* image) {
	//vidCap::read(image);
	//image = cvQueryFrame(capture);
}
