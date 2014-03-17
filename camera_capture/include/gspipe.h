/*
 * gspipe.h
 *
 *  Created on: Nov 30, 2013
 *      Author: lbarnett
 */

#ifndef GSPIPE_H_
#define GSPIPE_H_


extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <stdexcept>


namespace gspipe {
	class GSPipe {
	public:
		GSPipe(ros::NodeHandle nh_camera, ros::NodeHandle nh_private);
		~GSPipe();

		bool configure();
		bool init_stream();
		void publish_stream();
		void cleanup_stream();

		void run();

	private:

		// General gstreamer configuration
		std::string gsconfig_;
		std::string pipeline_str;

		std::string frame_id_;
		int width_, height_;

		// Gstreamer structures
		GstElement *pipeline_;
		GstElement *sink_;

		// Appsink configuration
		bool sync_sink_;
		bool preroll_;
		bool reopen_on_eof_;

		std::string image_encoding_;

		// ROS Inteface
		ros::NodeHandle nh_, nh_private_;
		image_transport::ImageTransport image_transport_;
		camera_info_manager::CameraInfoManager camera_info_manager_;
		image_transport::CameraPublisher camera_pub_;


	};
}


#endif /* GSPIPE_H_ */
