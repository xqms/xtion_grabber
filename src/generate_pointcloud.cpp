// Generate pointcloud from depth & rgb image
// Author: Max Schwarz <max.schwarz@online.de>

/* FIXME: Currently all optimized variants produce a pointcloud layout which
 * is not compatible with PCL (see generate_pointcloud.s). Therefore, we use
 * the dumb version for now. */

#include "generate_pointcloud.h"

#include <limits>

// #include <smmintrin.h>
// #include <emmintrin.h>
// #include <immintrin.h>
#include <vector>

#include <cmath>
#include <stdio.h>

namespace accel
{

PointCloudGenerator::PointCloudGenerator()
{
}

void PointCloudGenerator::init(int width, int height, float f)
{
	m_width = width;
	m_height = height;

	m_xlut.resize(width);
	m_ylut.resize(height);

	m_constant = 1.0f / 1000.0f / f;

	fprintf(stderr, "Filling XLUT\n");
	for(int x = 0; x < width; ++x)
	{
		m_xlut[x] = (x - (0.5f*width - 0.5f)) * m_constant;
		int cls = std::fpclassify(m_xlut[x]);
		if(cls != FP_NORMAL)
		{
			fprintf(stderr, "Denormalized number in xlut at x=%d: %f (cls %d)\n", x, m_xlut[x], cls);
			if(cls == FP_ZERO)
				fprintf(stderr, "(was zero)\n");
		}
	}

	for(int y = 0; y < height; ++y)
		m_ylut[y] = (y - (0.5f*height - 0.5f)) * m_constant;
}

#if 0

extern "C"
{
extern void __generatePointCloud_AVX2(const uint16_t* __restrict depth, const uint32_t* __restrict color, uint8_t* __restrict output, const float* xlut, const float* ylut, uint64_t width, uint64_t height);
}

void PointCloudGenerator::generatePointCloud(const uint16_t* __restrict depth, const uint32_t* __restrict color, uint8_t* __restrict output)
{
	__generatePointCloud_AVX2(depth, color, output, m_xlut.data(), m_ylut.data(), m_width, m_height);
}

#elif 0 && (__SSE4_1__ || 1)

// SSE implementation
void PointCloudGenerator::generatePointCloud(const uint16_t* __restrict depth, const uint32_t* __restrict color, uint8_t* __restrict output)
{
	if((reinterpret_cast<size_t>(depth) & 0xf) != 0)
	{
		abort();
	}

	if((reinterpret_cast<size_t>(color) & 0xf) != 0)
	{
		abort();
	}

	if((reinterpret_cast<size_t>(output) & 0xf) != 0)
	{
		abort();
	}

	__m128i DEPTH_INT;
	__m128i INT_ZERO = _mm_setzero_si128();

	__m128 DEPTH;
	__m128 X;
	__m128 Y;
	__m128 Z;

	__m128 YLUT;

	float depth_mm_to_m = 0.001f;

#if __AVX__
	__m128 DEPTH_MM_TO_M = _mm_broadcast_ss(&depth_mm_to_m);
#else
#warning No AVX
	__m128 DEPTH_MM_TO_M = _mm_set_ps1(0.001f);
#endif

	const float* xlut;
	const float* ylut = m_ylut.data();

	for(int y = 0; y < m_height; y += 1)
	{
		YLUT = _mm_set_ps(m_ylut[y], m_ylut[y], m_ylut[y], m_ylut[y]);

		xlut = m_xlut.data();

		for(int x = 0; x < m_width; x += 8)
		{
			// Load 8 depth words
			__m128i DEPTH_INT_PACKED = _mm_load_si128((const __m128i*)depth);

			// Unpack the first 4 words to 32 bit
			DEPTH_INT = _mm_unpacklo_epi16(DEPTH_INT_PACKED, INT_ZERO);

			// And convert to float
			DEPTH = _mm_cvtepi32_ps(DEPTH_INT);

			X = _mm_mul_ps(_mm_load_ps(xlut), DEPTH);
			Y = _mm_mul_ps(YLUT, DEPTH);
			Z = _mm_mul_ps(DEPTH, DEPTH_MM_TO_M);

			// And write!
			*((int*)(output + 0)) = _mm_extract_ps(X, 0);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 0);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 0);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			*((int*)(output + 0)) = _mm_extract_ps(X, 1);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 1);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 1);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			*((int*)(output + 0)) = _mm_extract_ps(X, 2);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 2);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 2);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			*((int*)(output + 0)) = _mm_extract_ps(X, 3);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 3);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 3);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			xlut += 4;

			// Unpack the second 4 words to 32 bit
			DEPTH_INT = _mm_unpackhi_epi16(DEPTH_INT_PACKED, INT_ZERO);

			// And convert to float
			DEPTH = _mm_cvtepi32_ps(DEPTH_INT);

			X = _mm_mul_ps(_mm_load_ps(xlut), DEPTH);
			Y = _mm_mul_ps(YLUT, DEPTH);
			Z = _mm_mul_ps(DEPTH, DEPTH_MM_TO_M);

			// And write!
			*((int*)(output + 0)) = _mm_extract_ps(X, 0);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 0);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 0);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			*((int*)(output + 0)) = _mm_extract_ps(X, 1);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 1);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 1);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			*((int*)(output + 0)) = _mm_extract_ps(X, 2);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 2);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 2);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			*((int*)(output + 0)) = _mm_extract_ps(X, 3);
			*((int*)(output + 4)) = _mm_extract_ps(Y, 3);
			*((int*)(output + 8)) = _mm_extract_ps(Z, 3);
			*((uint32_t*)(output + 12)) = *color;
			output += 16;
			color++;

			depth += 8;
			xlut += 4;
		}

		ylut += 4;
	}
}

#else
#warning No SSE available, using slow version

// Dumb implementation
void PointCloudGenerator::generatePointCloud(const uint16_t* __restrict depth_ptr, const uint32_t* __restrict color, unsigned int color_step, uint8_t* __restrict output)
{
	for(int y = 0; y < m_height; ++y)
	{
		float ylut = m_ylut[y];

		for(int x = 0; x < m_width; ++x)
		{
			float px;
			float py;
			float pz;

			if(*depth_ptr == 0)
			{
				px = py = pz = NAN;
			}
			else
			{
				float depth = *depth_ptr;
				px = m_xlut[x] * depth;
				py = ylut * depth;
				pz = depth / 1000.0;
			}

			*((float*)(output + 0)) = px;
			*((float*)(output + 4)) = py;
			*((float*)(output + 8)) = pz;
			*((uint32_t*)(output + 16)) = *color;

			output += 32;
			depth_ptr++;
			color += color_step;
		}

		if(color_step > 1)
			color += (color_step-1) * m_width * color_step;
	}
}

#endif

}
