#include "../include/stereo_cam_calibrate_node.h"

void leftImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	  cv_bridge::CvImagePtr cv_ptr;
	  try{
		  if (msg->encoding == "bgr8")
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  else
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

		  stereo_calib->setLeftImage(cv_ptr);
	  }
	  catch (cv_bridge::Exception& e)
	  {
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
	  }
}

void rightImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	  cv_bridge::CvImagePtr cv_ptr;
	  try{
		  if (msg->encoding == "bgr8")
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  else
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

		  stereo_calib->setRightImage(cv_ptr);
	  }
	  catch (cv_bridge::Exception& e)
	  {
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
	  }
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stereo_camera_calibrate");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("left_image_topic", left_image_topic_, std::string("/camera/left_image"));
	nh_private.param("right_image_topic", right_image_topic_, std::string("/camera/right_image"));

	nh_private.param("num_corners_x", num_corners_x_, int(8));
	nh_private.param("num_corners_y", num_corners_y_, int(6));

	nh_private.param("square_size", square_size_, int(2));
	nh_private.param("window_size", window_size_, int(11));
	nh_private.param("zero_zone", zero_zone_, int(-1));
	nh_private.param("max_iterations", max_iterations_, int(30));
	nh_private.param("epsilon", epsilon_, double(0.01));
	nh_private.param("max_iterations", max_iterations_, int(30));
	nh_private.param("alpha", alpha_, double(-1));


	image_transport::ImageTransport it(nh);
	image_transport::Subscriber leftSub = it.subscribe(left_image_topic_, 1, leftImageCallback);
	image_transport::Subscriber rightSub = it.subscribe(right_image_topic_, 1, rightImageCallback);

	stereo_calib = new StereoCalibrator(num_corners_x_, num_corners_y_, square_size_, window_size_, zero_zone_, max_iterations_, epsilon_, alpha_);

	while (ros::ok()) {
		ros::spinOnce();
		int keyPress = cv::waitKey(30);

		if (keyPress == (int) 'c') {
			int count = stereo_calib->captureImage();
			ROS_INFO("Captured image set. Count: %i", count);
		} else if (keyPress == (int) 'a') {
			stereo_calib->calibrateCameras();
			ROS_INFO("Reprojection errors. Left: %f Right: %f Stereo %f", stereo_calib->leftReprojectionError, stereo_calib->rightReprojectionError,
					stereo_calib->stereoReprojectionError);
		} else if (keyPress == (int) 'd') {
			stereo_calib->toggleDisplayCorners();
		} else if (keyPress == (int) 's') {
			stereo_calib->saveCalibrationToFile("stereo_calib.yml");
		}
		else if (keyPress == (int) 'q') {
			break;
		}
	}

}
