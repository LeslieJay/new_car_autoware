#pragma once
#include "device_launch_parameters.h"
#include <cuda_runtime.h>

// define undistort params
struct UndistParams
{
    int src_img_size = 0;
    int dst_img_size = 0;
    int src_img_width = 0;
    int src_img_height = 0;
    int dst_img_width = 0;
    int dst_img_height = 0;
};
// define undistort function
void distort_CUDA(const UndistParams &params, const float *MapX, const float *MapY,
                  const unsigned char *d_src, unsigned char *d_dst, cudaStream_t stream);