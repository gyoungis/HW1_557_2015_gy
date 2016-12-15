#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDARationalDivideKernel(float4* surfPts, float4* dervPts, int uNum, int vNum, float3* output)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		int index = j*uNum+i;
		float4 point = surfPts[index];
		float4 derv	 = dervPts[index];
		float3 ratDerv;
		ratDerv.x = (derv.x - derv.w*point.x/point.w)/point.w;
		ratDerv.y = (derv.y - derv.w*point.y/point.w)/point.w;
		ratDerv.z = (derv.z - derv.w*point.z/point.w)/point.w;
		output[index] = ratDerv;
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDARationalDivide(float4* surfPts, float4* dervPts, int uNum, int vNum, float3* output)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
    CUDARationalDivideKernel<<<grid, block>>>(surfPts, dervPts, uNum, vNum, output);
	return 1;
}