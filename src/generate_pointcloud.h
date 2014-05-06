// Generate pointcloud from depth & rgb image
// Author: Max Schwarz <max.schwarz@online.de>

#ifndef ACCEL_GENERATE_POINTCLOUD_H
#define ACCEL_GENERATE_POINTCLOUD_H

#include <stdint.h>

#include <Eigen/Core>

namespace accel
{

class PointCloudGenerator
{
public:
	PointCloudGenerator();

	void init(int width, int height, float f);

	void generatePointCloud(const uint16_t* __restrict depth, const uint32_t* __restrict color, unsigned int color_step, uint8_t* __restrict output);
private:
	std::vector<float, Eigen::aligned_allocator<float>> m_xlut;
	std::vector<float, Eigen::aligned_allocator<float>> m_ylut;

	int m_width;
	int m_height;
	float m_constant;
};

// void generatePointCloud(const uint16_t* __restrict depth, const uint32_t* __restrict color, uint8_t* __restrict output, unsigned int width, unsigned int height, float f);

}

#endif
