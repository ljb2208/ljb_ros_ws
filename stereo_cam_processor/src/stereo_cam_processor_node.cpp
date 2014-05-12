#include "../include/stereo_cam_processor_node.h"

void leftImageCallback(const sensor_msgs::ImageConstPtr& msg) {
	  cv_bridge::CvImagePtr cv_ptr;
	  try{
		  if (msg->encoding == "bgr8")
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  else
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

		  stereoProc->setLeftImage(cv_ptr);
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

		  stereoProc->setRightImage(cv_ptr);
	  }
	  catch (cv_bridge::Exception& e)
	  {
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
	  }
}

void reconfigureCallback(stereo_cam_processor::stereo_cam_processorConfig &config, uint32_t level) {
	min_disp_ = config.minDisp;
	sad_window_size_ = config.sadWindowSize;
	min_disp_ = config.minDisp;
	num_disparities_ = config.numDisp;
	preset_ = config.preset;
	speckle_window_size_ = config.speckleWindowSize;
	speckle_range_ = config.speckleRange;
	uniqueness_ratio_ = config.uniquenessRatio;
	full_dp_ = config.fullDP;
	disp_smoothness1_ = config.dispSmoothness1;
	disp_smoothness2_ = config.dispSmoothness2;
	disp_smoothness_maxdiff_ = config.dispSmoothnessMaxDiff;
	prefilter_cap_ = config.prefilterCap;
	use_bm_ = config.useBM;
	disp_smoothness_auto_ = config.autoSetDispSmoothness;

	if (stereoProc != NULL) {
		stereoProc->setBlockMatchParams(use_bm_, preset_, min_disp_, num_disparities_, sad_window_size_,
				disp_smoothness_auto_, disp_smoothness1_, disp_smoothness2_, disp_smoothness_maxdiff_, prefilter_cap_, uniqueness_ratio_,
				speckle_window_size_, speckle_range_, full_dp_);

	}
	ROS_INFO("Reconfigure Request");
}

int main(int argc, char **argv){

	ros::init(argc, argv, "stereo_camera_processor");

	dynamic_reconfigure::Server<stereo_cam_processor::stereo_cam_processorConfig> server;
	dynamic_reconfigure::Server<stereo_cam_processor::stereo_cam_processorConfig>::CallbackType f;

	f = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(f);

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("left_image_topic", left_image_topic_, std::string("/camera/left_image"));
	nh_private.param("right_image_topic", right_image_topic_, std::string("/camera/right_image"));

	nh_private.param("num_disparities", num_disparities_, int(3));
	nh_private.param("sad_window_size", sad_window_size_, int(15));
	nh_private.param("preset", preset_, int(31));
	nh_private.param("show_images", show_images_, bool(true));
	nh_private.param("min_disp", min_disp_, int(0));
	nh_private.param("disp_smoothness1", disp_smoothness1_, int(1));
	nh_private.param("disp_smoothness2", disp_smoothness2_, int(2));
	nh_private.param("disp_smoothness_maxdiff", disp_smoothness_maxdiff_, int(-1));
	nh_private.param("prefilter_cap", prefilter_cap_, int(1));
	nh_private.param("uniqueness_ratio", uniqueness_ratio_, int(10));
	nh_private.param("speckle_window_size", speckle_window_size_, int(100));
	nh_private.param("speckle_range", speckle_range_, int(2));
	nh_private.param("full_dp", full_dp_, bool(false));
	nh_private.param("use_bm", use_bm_, bool(false));
	nh_private.param("disp_smoothness_auto", disp_smoothness_auto_, bool(false));

	int min_disp_; //normally zero
	int disp_smoothness1_; // penalty on the disparity change by plus or minus one between neighbour pixels
	int disp_smoothness2_; // penalty on the disparity change by more than 1 between neighber pixels. this needs to be greater than above
	int disp_smoothness_maxdiff_; // maximum allowed difference between left and right pixels. -1 to disable
	int prefilter_cap_;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber leftSub = it.subscribe(left_image_topic_, 1, leftImageCallback);
	image_transport::Subscriber rightSub = it.subscribe(right_image_topic_, 1, rightImageCallback);

	ros::Publisher pub;
	pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

	stereoProc = new StereoCamProcessor(show_images_, pub);
	stereoProc->setBlockMatchParams(use_bm_, preset_, min_disp_, num_disparities_, sad_window_size_,
			disp_smoothness_auto_, disp_smoothness1_, disp_smoothness2_, disp_smoothness_maxdiff_, prefilter_cap_, uniqueness_ratio_,
			speckle_window_size_, speckle_range_, full_dp_);

	stereoProc->loadCalibration("stereo_calib.yml");

	// for testing - publish transformation
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation(tf::Quaternion(0, 0, 0));

	stereoProc->setCurrentTransform(transform);
	//tf::Quaternion q;
	//q.setRPY(0, 0, 0);
	//ransform.setRotation(q);

	while (ros::ok()) {
		ros::spinOnce();
		int keyPress = cv::waitKey(30);
	}
}
