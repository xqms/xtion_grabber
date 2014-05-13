// Grabber for the xtion kernel driver
// Author: Max Schwarz <max.schwarz@online.de>

#include "xtion_grabber.h"

#include <pluginlib/class_list_macros.h>

#include <boost/make_shared.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/photo/photo.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <libyuv.h>

namespace xtion_grabber
{

XtionGrabber::XtionGrabber()
 : m_lastColorSeq(-1)
 , m_lastDepthSeq(-2)
 , m_shouldExit(false)
{
	m_color_pool.reset(new utils::Pool<sensor_msgs::Image>);
	m_depth_pool.reset(new utils::Pool<sensor_msgs::Image>);
	m_pointCloudPool.reset(new utils::Pool<sensor_msgs::PointCloud2>);
}

XtionGrabber::~XtionGrabber()
{
	m_shouldExit = true;
	m_thread.join();

	stopColor();
	stopDepth();

	close(m_color_fd);
	close(m_depth_fd);
}

bool XtionGrabber::setupDepth(const std::string& device)
{
	m_pub_depth = getMTPrivateNodeHandle().advertise<sensor_msgs::Image>("depth", 1);

	m_depth_fd = open(device.c_str(), O_RDONLY);
	if(m_depth_fd < 0)
	{
		perror("Could not open depth device");
		return false;
	}

	struct v4l2_requestbuffers reqbuf;
	memset(&reqbuf, 0, sizeof(reqbuf));
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_USERPTR;
	reqbuf.count = NUM_BUFS;

	if(ioctl(m_depth_fd, VIDIOC_REQBUFS, &reqbuf) != 0)
	{
		perror("Could not request buffers");
		return false;
	}

	for(size_t i = 0; i < NUM_BUFS; ++i)
	{
		DepthBuffer* buffer = &m_depth_buffers[i];

		buffer->image = createDepthImage();

		memset(&buffer->buf, 0, sizeof(buffer->buf));
		buffer->buf.index = i;
		buffer->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer->buf.memory = V4L2_MEMORY_USERPTR;
		buffer->buf.m.userptr = (long unsigned int)buffer->image->data.data();
		buffer->buf.length = buffer->image->data.size();

		NODELET_INFO("buffer data: index %d, m.userptr: %p, length: %u",
			buffer->buf.index, (void*)buffer->buf.m.userptr, buffer->buf.length
		);

		if(ioctl(m_depth_fd, VIDIOC_QBUF, &buffer->buf) != 0)
		{
			perror("Could not queue buffer");
			return false;
		}
	}

	if(ioctl(m_depth_fd, VIDIOC_STREAMON, &reqbuf.type) != 0)
	{
		perror("Could not start streaming");
		return false;
	}

	return true;
}

void XtionGrabber::stopDepth()
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(m_depth_fd, VIDIOC_STREAMOFF, &type) != 0)
		perror("Could not stop streaming");
}

bool XtionGrabber::setupColor(const std::string& device)
{
	m_pub_color = getMTPrivateNodeHandle().advertise<sensor_msgs::Image>("color", 1);

	m_color_fd = open(device.c_str(), O_RDONLY);
	if(m_color_fd < 0)
	{
		perror("Could not open color device");
		return false;
	}

	struct v4l2_format fmt;
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = m_colorWidth;
	fmt.fmt.pix.height = m_colorHeight;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;

	if(ioctl(m_color_fd, VIDIOC_S_FMT, &fmt) != 0)
	{
		perror("Could not set image format");
		return false;
	}

	m_colorWidth = fmt.fmt.pix.width;
	m_colorHeight = fmt.fmt.pix.height;

	struct v4l2_streamparm parms;
	parms.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parms.parm.capture.timeperframe.numerator = 1;
	parms.parm.capture.timeperframe.denominator = 30;

	if(ioctl(m_color_fd, VIDIOC_S_PARM, &parms) != 0)
	{
		perror("Could not set image interval");
		return false;
	}

	/* Enable flicker filter for 50 Hz */
	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(ctrl));
	ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
	ctrl.value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;

	if(ioctl(m_color_fd, VIDIOC_S_CTRL, &ctrl) != 0)
	{
		perror("Could not set flicker control");
		return false;
	}

	struct v4l2_requestbuffers reqbuf;
	memset(&reqbuf, 0, sizeof(reqbuf));
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_USERPTR;
	reqbuf.count = NUM_BUFS;

	if(ioctl(m_color_fd, VIDIOC_REQBUFS, &reqbuf) != 0)
	{
		perror("Could not request buffers");
		return false;
	}

	for(size_t i = 0; i < NUM_BUFS; ++i)
	{
		ColorBuffer* buffer = &m_color_buffers[i];

		buffer->data.resize(m_colorWidth*m_colorHeight*2);

		memset(&buffer->buf, 0, sizeof(buffer->buf));
		buffer->buf.index = i;
		buffer->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer->buf.memory = V4L2_MEMORY_USERPTR;
		buffer->buf.m.userptr = (long unsigned int)buffer->data.data();
		buffer->buf.length = buffer->data.size();

		if(ioctl(m_color_fd, VIDIOC_QBUF, &buffer->buf) != 0)
		{
			perror("Could not queue buffer");
			return false;
		}
	}

	if(ioctl(m_color_fd, VIDIOC_STREAMON, &reqbuf.type) != 0)
	{
		perror("Could not start streaming");
		return false;
	}

	return true;
}

void XtionGrabber::stopColor()
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(ioctl(m_color_fd, VIDIOC_STREAMOFF, &type) != 0)
		perror("Could not stop streaming");
}

void XtionGrabber::onInit()
{
	ros::NodeHandle& nh = getMTPrivateNodeHandle();

	std::string depthDevice;
	std::string colorDevice;

	if(!nh.getParam("depth_device", depthDevice)
		|| !nh.getParam("color_device", colorDevice))
	{
		ROS_FATAL("depth_device and color_device parameters are mandatory!");
		throw std::runtime_error("depth_device and color_device parameters are mandatory!");
	}

	nh.param("depth_width", m_depthWidth, 640);
	nh.param("depth_height", m_depthHeight, 480);
	nh.param("color_width", m_colorWidth, 1280);
	nh.param("color_height", m_colorHeight, 1024);

	if(!setupColor(colorDevice))
		throw std::runtime_error("Could not setup color channel");

	if(!setupDepth(depthDevice))
		throw std::runtime_error("Could not setup depth channel");

	m_depthFocalLength = 525.0f * m_depthWidth / 640;
	m_colorFocalLength = 525.0f * m_colorWidth / 640;

	m_cloudGenerator.init(m_depthWidth, m_depthHeight, m_depthFocalLength);
	m_filledCloudGenerator.init(1280, 960, 525.0f * 1280 / 640);

	setupCameraInfo();

	m_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
	m_pub_filledCloud = nh.advertise<sensor_msgs::PointCloud2>("cloud_filled", 1);

	m_thread = boost::thread(boost::bind(&XtionGrabber::read_thread, this));
}

sensor_msgs::ImagePtr XtionGrabber::createDepthImage()
{
	sensor_msgs::ImagePtr img = m_depth_pool->create();

	img->encoding = sensor_msgs::image_encodings::MONO16;
	img->width = m_depthWidth;
	img->height = m_depthHeight;
	img->step = img->width * 2;
	img->data.resize(img->step * img->height);

	return img;
}

ros::Time timeFromTimeval(const struct timeval& tv)
{
	struct timespec current_monotonic;
	clock_gettime(CLOCK_MONOTONIC, &current_monotonic);

	ros::Time current_time = ros::Time::now();

	int64_t nsec_diff = (current_monotonic.tv_sec - tv.tv_sec) * 1000000000L + current_monotonic.tv_nsec - tv.tv_usec/1000L;

	return ros::Time(
		current_time.sec + nsec_diff / 1000000000L,
		(current_time.nsec + nsec_diff) % 1000000000L
	);
}

void XtionGrabber::read_thread()
{
	fd_set fds;

	while(!m_shouldExit)
	{
		FD_ZERO(&fds);
		FD_SET(m_depth_fd, &fds);
		FD_SET(m_color_fd, &fds);

		int ret = select(std::max(m_depth_fd, m_color_fd)+1, &fds, 0, 0, 0);

		if(ret < 0)
		{
			perror("Could not select()");
			return;
		}

		if(FD_ISSET(m_color_fd, &fds))
		{
			v4l2_buffer buf;
			memset(&buf, 0, sizeof(buf));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;

			if(ioctl(m_color_fd, VIDIOC_DQBUF, &buf) != 0)
			{
				if(errno == EAGAIN)
					continue;
				perror("Could not dequeue buffer");
				return;
			}

			ColorBuffer* buffer = &m_color_buffers[buf.index];

			sensor_msgs::ImagePtr img = m_color_pool->create();
			img->width = m_colorWidth;
			img->height = m_colorHeight;
			img->step = img->width * 4;
			img->data.resize(img->step * img->height);
			img->header.stamp = timeFromTimeval(buf.timestamp);

			img->encoding = sensor_msgs::image_encodings::BGRA8;

			libyuv::ConvertToARGB(
				buffer->data.data(), buffer->data.size(),
				img->data.data(),
				m_colorWidth*4, 0, 0, m_colorWidth, m_colorHeight, m_colorWidth, m_colorHeight,
				libyuv::kRotate0, libyuv::FOURCC_UYVY
			);

			m_lastColorImage = img;
			m_lastColorSeq = buf.sequence;
			m_pub_color.publish(img);

			if(ioctl(m_color_fd, VIDIOC_QBUF, &buffer->buf) != 0)
			{
				perror("Could not queue buffer");
				return;
			}
		}

		if(FD_ISSET(m_depth_fd, &fds))
		{
			v4l2_buffer buf;
			memset(&buf, 0, sizeof(buf));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;

			if(ioctl(m_depth_fd, VIDIOC_DQBUF, &buf) != 0)
			{
				if(errno == EAGAIN)
					continue;
				perror("Could not dequeue buffer");
				return;
			}

			DepthBuffer* buffer = &m_depth_buffers[buf.index];

			buffer->image->header.stamp = timeFromTimeval(buf.timestamp);

			m_lastDepthImage = buffer->image;
			m_lastDepthSeq = buf.sequence;
			m_pub_depth.publish(buffer->image);

			buffer->image.reset();

			buffer->image = createDepthImage();
			buffer->buf.m.userptr = (long unsigned int)buffer->image->data.data();

			if(ioctl(m_depth_fd, VIDIOC_QBUF, &buffer->buf) != 0)
			{
				perror("Could not queue buffer");
				return;
			}
		}

		if(m_lastColorSeq == m_lastDepthSeq)
		{
			if(m_pub_cloud.getNumSubscribers() != 0)
				publishPointCloud(m_lastDepthImage, &m_cloudGenerator, &m_pub_cloud);

			if(m_pub_filledCloud.getNumSubscribers() != 0)
				publishPointCloud(fillDepth(), &m_filledCloudGenerator, &m_pub_filledCloud);
		}
	}

	fprintf(stderr, "read thread exit now\n");
}

sensor_msgs::ImageConstPtr XtionGrabber::fillDepth()
{
	cv_bridge::CvImageConstPtr cv_depth_input = cv_bridge::toCvShare(m_lastDepthImage, "mono16");

	cv::Mat_<uint16_t> cv_depth(960, 1280);
	for(int y = 0; y < cv_depth.rows; ++y)
	{
		for(int x = 0; x < cv_depth.cols; ++x)
		{
			cv_depth(y,x) = cv_depth_input->image.at<uint16_t>(
				y / (cv_depth.rows / cv_depth_input->image.rows),
				x / (cv_depth.cols / cv_depth_input->image.cols)
			);
		}
	}

	cv::Mat_<uint8_t> cv_depth8(cv_depth.size());
	cv::Mat_<uint8_t> mask(cv_depth.size(), (uint8_t)0);

	// Fill invalid depth with cv::inpaint
	for(int y = 0; y < cv_depth.rows; ++y)
	{
		for(int x = 0; x < cv_depth.cols; ++x)
		{
			cv_depth8(y,x) = cv_depth(y,x) / 10;

			if(cv_depth(y, x) == 0)
				mask(y,x) = 1;
		}
	}

	cv::Mat_<uint8_t> inpaintedDepth(cv_depth.size(), cv_depth.type());
	cv::inpaint(cv_depth8, mask, inpaintedDepth, 5.0, cv::INPAINT_TELEA);

	cv_bridge::CvImage filledDepth;

	filledDepth.image = cv::Mat(cv_depth.size(), CV_16UC1);
	filledDepth.encoding = "mono16";
	filledDepth.header = m_lastDepthImage->header;

	for(int y = 0; y < cv_depth.rows; ++y)
	{
		for(int x = 0; x < cv_depth.cols; ++x)
		{
			if(mask(y,x))
				filledDepth.image.at<uint16_t>(y,x) = inpaintedDepth(y,x) * 10;
			else
				filledDepth.image.at<uint16_t>(y,x) = cv_depth(y,x);
		}
	}

	sensor_msgs::ImageConstPtr out = filledDepth.toImageMsg();

	NODELET_ERROR("filled depth image has dim %dx%d", (int)out->width, (int)out->height);

	return out;
}

void XtionGrabber::publishPointCloud(const sensor_msgs::ImageConstPtr& depth,
                                     accel::PointCloudGenerator* generator,
                                     ros::Publisher* dest)
{
	if(!m_lastColorImage || !depth)
		return;

	sensor_msgs::PointCloud2::Ptr cloud = m_pointCloudPool->create();

	cloud->header.stamp = m_lastColorImage->header.stamp;
	cloud->header.frame_id = "/camera_optical";

	cloud->fields.resize(4);
	cloud->fields[0].name = "x";
	cloud->fields[0].offset = 0;
	cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	cloud->fields[0].count = 1;
	cloud->fields[1].name = "y";
	cloud->fields[1].offset = 4;
	cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	cloud->fields[1].count = 1;
	cloud->fields[2].name = "z";
	cloud->fields[2].offset = 8;
	cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	cloud->fields[2].count = 1;
	cloud->fields[3].name = "rgb";
	cloud->fields[3].offset = 16;
	cloud->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
	cloud->fields[3].count = 1;

	cloud->width = depth->width;
	cloud->height = depth->height;
	cloud->is_bigendian = 0;
	cloud->is_dense = 1;
	cloud->point_step = 32;
	cloud->row_step = cloud->point_step * cloud->width;
	cloud->data.resize(cloud->row_step * cloud->height);

	timespec start;
	timespec end;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

	int colorOffset = 0;
	if(m_colorHeight == 1024)
	{
		// SXGA is the true sensor resolution. The xtion can only sense depth
		// in VGA resolution, which has a wider aspect ratio.
		// => skip the first 32 rows in the color data to get the same aspect
		//    ratio.
		colorOffset = 32 * m_colorWidth;
	}

	generator->generatePointCloud(
		(uint16_t*)depth->data.data(),
		(uint32_t*)m_lastColorImage->data.data() + colorOffset,
		m_colorWidth / depth->width,
		cloud->data.data()
	);

	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);

	dest->publish(cloud);
}

void XtionGrabber::setupCameraInfo()
{
	sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

	/* We are reporting information about the *color* sensor here. */

	info->width = m_colorWidth;
	info->height = m_colorHeight;

	// No distortion
	info->D.resize(5, 0.0);
	info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	// Simple camera matrix: square pixels (fx = fy), principal point at center
	info->K.assign(0.0);
	info->K[0] = info->K[4] = m_colorFocalLength;
	info->K[2] = (m_colorWidth /2.0) - 0.5;
	info->K[5] = (m_colorHeight/2.0) - 0.5;
	info->K[8] = 1.0;

	// No separate rectified image plane, so R = I
	info->R.assign(0.0);
	info->R[0] = info->R[4] = info->R[8] = 1.0;

	// Then P=K(I|0) = (K|0)
	info->P.assign(0.0);
	info->P[0] = info->P[5] = m_colorFocalLength; // fx, fy
	info->P[2] = info->K[2]; // cx
	info->P[6] = info->K[5]; // cy
	info->P[10] = 1.0;

	m_pub_info = getPrivateNodeHandle().advertise<sensor_msgs::CameraInfo>("camera_info", 1, true);
	m_pub_info.publish(info);
}

}

PLUGINLIB_EXPORT_CLASS(xtion_grabber::XtionGrabber, nodelet::Nodelet)
