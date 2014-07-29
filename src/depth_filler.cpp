// Fills holes in the depth image in a hopefully smart way
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "depth_filler.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

DepthFiller::DepthFiller()
{
}

DepthFiller::~DepthFiller()
{
}

sensor_msgs::ImagePtr DepthFiller::fillDepth(const sensor_msgs::ImageConstPtr& depth,
                                             const sensor_msgs::ImageConstPtr& color)
{
	cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvShare(depth);
	cv_bridge::CvImageConstPtr cv_color_raw = cv_bridge::toCvShare(color);

	cv::Mat cv_color;
	if(color->height == 1024)
		cv_color = cv_color_raw->image(cv::Rect(0, 32, 1280, 960));
	else
		cv_color = cv_color_raw->image;

	cv::Mat_<uint16_t> output(cv_color.size(), (uint16_t)0);

	int xincr = output.cols / cv_depth->image.cols;
	int yincr = output.rows / cv_depth->image.rows;

	cv::Mat_<uint8_t> edgeDetInput(cv_depth->image.size());
	for(int y = 0; y < edgeDetInput.rows; ++y)
	{
		uint8_t* row = edgeDetInput[y];
		const uint16_t* depth = cv_depth->image.ptr<uint16_t>(y);

		for(int x = 0; x < edgeDetInput.cols; ++x)
			row[x] = std::min(255, depth[x] / 10);
	}
	cv::Mat_<uint8_t> edges(edgeDetInput.size());
	cv::Canny(edgeDetInput, edges, 10, 3 * 10, 3);

	cv::Mat_<uint8_t> dilatedEdges;

	const int dilation_size = 1;
	cv::Mat element = cv::getStructuringElement(2, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                  cv::Point(dilation_size, dilation_size));     // dilation_type = MORPH_ELLIPSE
	cv::dilate(edges, dilatedEdges, element);

	cv::imshow("edges", dilatedEdges);
	cv::waitKey(100);

	// Fill in original data
	for(int y = 0; y < output.rows; y += yincr)
	{
		for(int x = 0; x < output.cols; x += xincr)
		{
			if(dilatedEdges(y/yincr,x/xincr) == 0)
				output(y,x) = cv_depth->image.at<uint16_t>(y / yincr, x / xincr);
		}
	}

	m_leftNeighbors.create(output.size());
	m_rightNeighbors.create(output.size());
	m_topNeighbors.create(output.size());
	m_bottomNeighbors.create(output.size());

	// Row-wise neighbors of invalid pixels
	for(int y = 0; y < output.rows; ++y)
	{
		uint16_t* row = output[y];
		uint16_t* lRow = m_leftNeighbors[y];
		uint16_t* rRow = m_rightNeighbors[y];

		uint16_t distance = 1;

		for(int x = 0; x < output.cols; ++x, ++distance)
		{
			if(row[x] != 0)
				distance = 0;
			else
				lRow[x] = distance;
		}

		distance = 1;
		for(int x = output.cols - 1; x != -1; --x, ++distance)
		{
			if(row[x] != 0)
				distance = 0;
			else
				rRow[x] = distance;
		}
	}

	// Col-wise neighbors
	for(int x = 0; x < output.cols; ++x)
	{
		uint16_t distance = 1;

		for(int y = 0; y < output.rows; ++y, ++distance)
		{
			if(output(y,x) != 0)
				distance = 0;
			else
				m_topNeighbors(y,x) = distance;
		}

		distance = 1;
		for(int y = output.rows-1; y != -1; --y, ++distance)
		{
			if(output(y,x) != 0)
				distance = 0;
			else
				m_bottomNeighbors(y,x) = distance;
		}
	}

	cv::imwrite("/tmp/leftNeighbors.png", m_leftNeighbors);
	cv::imwrite("/tmp/rightNeighbors.png", m_rightNeighbors);

#if 0
	// And fill in from the best available source
	for(int y = 0; y < output.rows; ++y)
	{
		uint16_t* row = output[y];
		const uint32_t* colorRow = cv_color.ptr<uint32_t>(y);

		uint16_t* lRow = m_leftNeighbors[y];
		uint16_t* rRow = m_rightNeighbors[y];
		uint16_t* tRow = m_topNeighbors[y];
		uint16_t* bRow = m_bottomNeighbors[y];

		for(int x = 0; x < output.cols; ++x)
		{
			if(row[x] != 0)
				continue;

			uint32_t rgb = colorRow[x];
			int32_t r = (rgb >>  0) & 0xFF;
			int32_t g = (rgb >>  8) & 0xFF;
			int32_t b = (rgb >> 16) & 0xFF;

			cv::Point2i p(x, y);

			int distances[] = {
				lRow[x],
				rRow[x],
				tRow[x],
				bRow[x]
			};
			cv::Point2i neighbors[4] = {
				p + cv::Point2i(-lRow[x], 0),
				p + cv::Point2i(rRow[x], 0),
				p + cv::Point2i(0, -tRow[x]),
				p + cv::Point2i(0, bRow[x])
			};

			uint32_t min_dist = 0;
			int best_idx = -1;

			for(int i = 0; i < 4; ++i)
			{
				const cv::Point2i& pos = neighbors[i];

				if(pos.x < 0 || pos.x >= output.cols || pos.y < 0 || pos.y >= output.rows || distances[i] > 32)
					continue;

				uint32_t xrgb = cv_color.at<uint32_t>(pos.y, pos.x);
				int32_t xr = (xrgb >>  0) & 0xFF;
				int32_t xg = (xrgb >>  8) & 0xFF;
				int32_t xb = (xrgb >> 16) & 0xFF;

				uint32_t dr = abs(xr - r);
				uint32_t dg = abs(xg - g);
				uint32_t db = abs(xb - b);

				uint32_t dist = dr*dr + dg*dg + db*db;

				if(best_idx == -1 || dist < min_dist)
				{
					min_dist = dist;
					best_idx = i;
				}
			}

			if(best_idx != -1)
			{
				const cv::Point2i& pos = neighbors[best_idx];
				row[x] = output(pos.y, pos.x);
			}
		}
	}
#endif

	cv::imshow("filled", output);

	return cv_bridge::CvImage(depth->header, depth->encoding, output).toImageMsg();
}
