// Grabber for the xtion kernel driver
// Author: Max Schwarz <max.schwarz@online.de>

#ifndef XTION_GRABBER_H
#define XTION_GRABBER_H

#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <linux/videodev2.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <camera_info_manager/camera_info_manager.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <ros/publisher.h>

#include "pool.h"
#include "generate_pointcloud.h"
#include "depth_filler.h"

const int NUM_BUFS = 31;

namespace xtion_grabber
{

class XtionGrabber : public nodelet::Nodelet
{
public:
	XtionGrabber();
	virtual ~XtionGrabber();

	virtual void onInit();
private:
	////////////////////////////////////////////////////////////////////////////
	// Parameters
	int m_depthWidth;
	int m_depthHeight;
	int m_colorWidth;
	int m_colorHeight;

	double m_depthFocalLength;
	double m_colorFocalLength;

	std::string m_deviceName;
	std::string m_frameName;

	////////////////////////////////////////////////////////////////////////////
	// Camera info
	void setupRGBInfo();
	void setupDepthInfo();

	////////////////////////////////////////////////////////////////////////////
	// Depth channel
	bool setupDepth(const std::string& device);
	void stopDepth();

	sensor_msgs::ImagePtr createDepthImage();

	struct DepthBuffer
	{
		sensor_msgs::ImagePtr image;
		v4l2_buffer buf;
	};
	DepthBuffer m_depth_buffers[NUM_BUFS];

	int m_depth_fd;

	boost::shared_ptr<image_transport::ImageTransport> m_depth_it;
	sensor_msgs::CameraInfo m_depth_info;
	image_transport::CameraPublisher m_pub_depth;
	utils::Pool<sensor_msgs::Image>::Ptr m_depth_pool;

	boost::shared_ptr<camera_info_manager::CameraInfoManager> m_depth_infoMgr;

	////////////////////////////////////////////////////////////////////////////
	// Color channel
	bool setupColor(const std::string& device);
	void stopColor();

	struct ColorBuffer
	{
		v4l2_buffer buf;
		std::vector<uint8_t> data;
	};
	ColorBuffer m_color_buffers[NUM_BUFS];

	int m_color_fd;

	boost::shared_ptr<image_transport::ImageTransport> m_color_it;
	sensor_msgs::CameraInfo m_color_info;
	image_transport::CameraPublisher m_pub_color;
	utils::Pool<sensor_msgs::Image>::Ptr m_color_pool;

	boost::shared_ptr<camera_info_manager::CameraInfoManager> m_color_infoMgr;

	////////////////////////////////////////////////////////////////////////////
	// PointCloud generation
	uint64_t m_lastColorSeq;
	sensor_msgs::ImageConstPtr m_lastColorImage;

	uint64_t m_lastDepthSeq;
	sensor_msgs::ImageConstPtr m_lastDepthImage;

	accel::PointCloudGenerator m_cloudGenerator;
	accel::PointCloudGenerator m_filledCloudGenerator;
	ros::Publisher m_pub_cloud;
	ros::Publisher m_pub_filledCloud;
	utils::Pool<sensor_msgs::PointCloud2>::Ptr m_pointCloudPool;

	DepthFiller m_depthFiller;
	void publishPointCloud(const sensor_msgs::ImageConstPtr& depth,
	                       accel::PointCloudGenerator* generator,
	                       ros::Publisher* dest);

	void read_thread();

	boost::thread m_thread;
	bool m_shouldExit;
};

}

#endif
