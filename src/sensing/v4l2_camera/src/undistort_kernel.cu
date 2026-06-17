#include <iostream>
#include <stdio.h>
#include "undistort_kernel.h"
#include <chrono>

// CUDA 核函数：实现畸变校正，RGBRGBRGB排布方式
__global__ void remap_Kernel(const unsigned char *src, unsigned char *dst, const float *MapX, const float *MapY,
                             const int src_width, const int src_height, const int dst_width, const int dst_height)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    if (x < dst_width && y < dst_height)
    {
        float newX = MapX[y * dst_width + x];
        float newY = MapY[y * dst_width + x];

        // 计算映射后的整数坐标
        int src_x = static_cast<int>(newX);
        int src_y = static_cast<int>(newY);
        // 边界检查：确保坐标在源图像范围内
        if (src_x >= 0 && src_x < src_width && src_y >= 0 && src_y < src_height) {
            int dstIndex = (y * dst_width + x) * 3;
            int srcIndex = (src_y * src_width + src_x) * 3;
            
            // 安全访问源图像
            dst[dstIndex] = src[srcIndex];
            dst[dstIndex + 1] = src[srcIndex + 1];
            dst[dstIndex + 2] = src[srcIndex + 2];
        }
        else {
            // 将越界像素设为黑色
            int dstIndex = (y * dst_width + x) * 3;
            dst[dstIndex] = 0;        // R
            dst[dstIndex + 1] = 0;    // G
            dst[dstIndex + 2] = 0;    // B
        }
    }
}

// RGBRGBRGB 排布方式，双线性插值
__global__ void remap_Kernel_RGB(const unsigned char *src, unsigned char *dst, 
                               const float *MapX, const float *MapY,
                               const int src_width, const int src_height, 
                               const int dst_width, const int dst_height)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    
    if (x < dst_width && y < dst_height)
    {
        float newX = MapX[y * dst_width + x];
        float newY = MapY[y * dst_width + x];
        
        // 确保坐标在有效范围内
        newX = max(0.0f, min(newX, static_cast<float>(src_width - 1)));
        newY = max(0.0f, min(newY, static_cast<float>(src_height - 1)));
        
        // 计算四个最近邻点的坐标
        int x1 = static_cast<int>(newX);
        int y1 = static_cast<int>(newY);
        int x2 = min(x1 + 1, src_width - 1);
        int y2 = min(y1 + 1, src_height - 1);
        
        // 计算插值权重
        float fx = newX - x1;
        float fy = newY - y1;
        float w1 = (1.0f - fx) * (1.0f - fy);
        float w2 = fx * (1.0f - fy);
        float w3 = (1.0f - fx) * fy;
        float w4 = fx * fy;
        
        // 计算目标索引
        int dstIndex = (y * dst_width + x) * 3;
        
        // 计算源图像中四个点的索引
        int idx1 = (y1 * src_width + x1) * 3;
        int idx2 = (y1 * src_width + x2) * 3;
        int idx3 = (y2 * src_width + x1) * 3;
        int idx4 = (y2 * src_width + x2) * 3;
        
        // 对每个通道进行双线性插值
        for (int c = 0; c < 3; ++c) {
            dst[dstIndex + c] = static_cast<unsigned char>(
                w1 * src[idx1 + c] + w2 * src[idx2 + c] + 
                w3 * src[idx3 + c] + w4 * src[idx4 + c]
            );
        }
    }
}

// RRRGGGBBB排布方式
__global__ void remap_Kernel_RRR(const unsigned char *src, unsigned char *dst, 
                                  const float *MapX, const float *MapY,
                                  const int src_width, const int src_height, 
                                  const int dst_width, const int dst_height)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    
    if (x < dst_width && y < dst_height)
    {
        float newX = MapX[y * dst_width + x];
        float newY = MapY[y * dst_width + x];
        
        // 确保坐标在有效范围内
        newX = max(0.0f, min(newX, static_cast<float>(src_width - 1)));
        newY = max(0.0f, min(newY, static_cast<float>(src_height - 1)));
        
        // 计算四个最近邻点的坐标
        int x1 = static_cast<int>(newX);
        int y1 = static_cast<int>(newY);
        int x2 = min(x1 + 1, src_width - 1);
        int y2 = min(y1 + 1, src_height - 1);
        
        // 计算插值权重
        float fx = newX - x1;
        float fy = newY - y1;
        float w1 = (1.0f - fx) * (1.0f - fy);
        float w2 = fx * (1.0f - fy);
        float w3 = (1.0f - fx) * fy;
        float w4 = fx * fy;
        
        // 计算目标索引（RRRGGGBBB 布局）
        int dstIndexR = y * dst_width + x;
        int dstIndexG = dst_width * dst_height + dstIndexR;
        int dstIndexB = 2 * dst_width * dst_height + dstIndexR;
        
        // 计算源图像中四个点的索引（假设源图像也是 RRRGGGBBB 布局）
        int idx1 = y1 * src_width + x1;
        int idx2 = y1 * src_width + x2;
        int idx3 = y2 * src_width + x1;
        int idx4 = y2 * src_width + x2;
        
        // 对每个通道进行双线性插值
        dst[dstIndexR] = static_cast<unsigned char>(
            w1 * src[idx1] + w2 * src[idx2] + w3 * src[idx3] + w4 * src[idx4]
        );
        
        // 注意 G 和 B 通道在源图像中的偏移
        int offsetG = src_width * src_height;
        int offsetB = 2 * src_width * src_height;
        
        dst[dstIndexG] = static_cast<unsigned char>(
            w1 * src[offsetG + idx1] + w2 * src[offsetG + idx2] + 
            w3 * src[offsetG + idx3] + w4 * src[offsetG + idx4]
        );
        
        dst[dstIndexB] = static_cast<unsigned char>(
            w1 * src[offsetB + idx1] + w2 * src[offsetB + idx2] + 
            w3 * src[offsetB + idx3] + w4 * src[offsetB + idx4]
        );
    }
}

void distort_CUDA(const UndistParams &params, const float *MapX, const float *MapY, const unsigned char *d_src, unsigned char *d_dst, cudaStream_t stream=nullptr)
{
    // 线程和块的配置
    dim3 blockDim(32, 32);
    dim3 gridDim((params.dst_img_width + blockDim.x - 1) / blockDim.x, (params.dst_img_height + blockDim.y - 1) / blockDim.y);
    // 调用 CUDA 内核
    remap_Kernel_RGB<<<gridDim, blockDim, 0, stream>>>(d_src, d_dst, MapX, MapY, params.src_img_width, params.src_img_height, params.dst_img_width, params.dst_img_height);
}