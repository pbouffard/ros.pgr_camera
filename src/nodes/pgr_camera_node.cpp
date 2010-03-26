// ROS communication
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <polled_camera/publication_server.h>
#include <pgr_camera/pgr_camera.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include "pgr_camera/PGRCameraConfig.h"

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>

class PGRCameraNode {
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher streaming_pub_;
	polled_camera::PublicationServer poll_srv_;
	ros::ServiceServer set_camera_info_srv_;

	// Camera
	boost::scoped_ptr<pgr_camera::Camera> cam_;
	bool running;

	// ROS messages
	sensor_msgs::Image img_;
	sensor_msgs::CameraInfo cam_info_;

	// Diagnostics
	int count_;

public:
	PGRCameraNode(const ros::NodeHandle& node_handle) :
		nh_(node_handle), it_(nh_), cam_(NULL), running(false), count_(0) {
		// Two-stage initialization: in the constructor we open the requested camera. Most
		// parameters controlling capture are set and streaming started in configure(), the
		// callback to dynamic_reconfig.
		pgr_camera::init();
		if (pgr_camera::numCameras() == 0)
			ROS_WARN("Found no cameras");

		//ros::NodeHandle local_nh("~");

		cam_.reset(new pgr_camera::Camera());
		cam_->initCam();
	}

	void configure(pgr_camera::PGRCameraConfig& config, uint32_t level) {
		ROS_INFO("Reconfigure request received");

		if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
			stop();

		cam_->SetVideoModeAndFramerate(config.width, config.height,
				config.format, config.frame_rate);

		// TF frame
		img_.header.frame_id = cam_info_.header.frame_id = config.frame_id;

		if (level >= (uint32_t) driver_base::SensorLevels::RECONFIGURE_STOP)
			start();
	}

	void start() {
		if (running)
			return;

		cam_->setFrameCallback(boost::bind(&PGRCameraNode::publishImage, this,
				_1));
		streaming_pub_ = it_.advertiseCamera("image_raw", 1);
		cam_->start();
		running = true;
	}

	void stop() {
		if (!running)
			return;

		cam_->stop(); // Must stop camera before streaming_pub_.
		poll_srv_.shutdown();
		streaming_pub_.shutdown();

		running = false;
	}

	static bool frameToImage(FlyCapture2::Image* frame,
			sensor_msgs::Image &image) {

		// NOTE: 16-bit and Yuv formats not supported
		static const char* BAYER_ENCODINGS[] = { "none", "bayer_rggb8",
				"bayer_grbg8", "bayer_gbrg8", "bayer_bggr8", "unknown" };
		std::string encoding;

		//unsigned int bpp = frame->GetBitsPerPixel();
		FlyCapture2::BayerTileFormat bayerFmt;
		bayerFmt = frame->GetBayerTileFormat();
		//ROS_INFO("bayer is %u", bayerFmt);
		if (bayerFmt == FlyCapture2::NONE) {
			encoding = sensor_msgs::image_encodings::MONO8;
		} else {
			encoding = BAYER_ENCODINGS[bayerFmt];
		}

		//uint32_t step = frame->GetDataSize() / frame->GetRows(); // TODO: really need to compute this every time?
		return sensor_msgs::fillImage(image, encoding, frame->GetRows(),
				frame->GetCols(), frame->GetStride(), frame->GetData());
	}

	bool processFrame(FlyCapture2::Image* frame, sensor_msgs::Image &img,
			sensor_msgs::CameraInfo &cam_info) {
		/// @todo Use time from frame?
		img.header.stamp = cam_info.header.stamp = ros::Time::now();

		if (!frameToImage(frame, img))
			return false;
		cam_info.height = img.height;
		cam_info.width = img.width;
		//frame->GetDimensions(&cam_info.height, &cam_info.width);
		//cam_info.width = frame->GetCols();
		//
		//		cam_info.roi.x_offset = frame->RegionX;
		//		cam_info.roi.y_offset = frame->RegionY;
		//		cam_info.roi.height = frame->Height;
		//		cam_info.roi.width = frame->Width;

		count_++;
		//ROS_INFO("count = %d", count_);
		return true;
	}

	void publishImage(FlyCapture2::Image* frame) {
		if (processFrame(frame, img_, cam_info_))
			streaming_pub_.publish(img_, cam_info_);
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "pgr_camera");

	typedef dynamic_reconfigure::Server<pgr_camera::PGRCameraConfig> Server;
	Server server;

	try {
		ros::NodeHandle nh("camera");
		boost::shared_ptr<PGRCameraNode> pn(new PGRCameraNode(nh));

		Server::CallbackType f = boost::bind(&PGRCameraNode::configure, pn, _1,
				_2);
		server.setCallback(f);

		ros::spin();

	} catch (std::runtime_error &e) {
		ROS_FATAL("Uncaught exception: '%s', aborting.", e.what());
		ROS_BREAK();
	}

	return 0;

}
