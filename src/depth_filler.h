// Fills holes in the depth image in a hopefully smart way
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DEPTH_FILLER_H
#define DEPTH_FILLER_H

#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>

class DepthFiller
{
public:
	DepthFiller();
	virtual ~DepthFiller();

	sensor_msgs::ImagePtr fillDepth(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& color);
private:
	typedef cv::Mat_<uint16_t> NeighborMap;

	NeighborMap m_leftNeighbors;
	NeighborMap m_rightNeighbors;
	NeighborMap m_topNeighbors;
	NeighborMap m_bottomNeighbors;
};

#endif
