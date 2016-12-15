#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDAExactNormalEvalKernel(float3* uDervGPUData, float3* vDervGPUData, int uNum, int vNum, float3* normalData)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		int index = j*uNum+i;
		float3 uDerv = uDervGPUData[index];
		float3 vDerv = vDervGPUData[index];
		if (length(uDerv) < DELTA)
		{
			int offsetX,offsetY;
			if(i == 0)
				offsetX = 1;
			else
				offsetX = -1;
			if(j == 0)
				offsetY = 1;
			else
				offsetY = -1;
			int index2 = (j+offsetY)*uNum + (i+offsetX);
			uDerv = uDervGPUData[index2];
		}
		if (length(vDerv) < DELTA)
		{
			int offsetX,offsetY;
			if(i == 0)
				offsetX = 1;
			else
				offsetX = -1;
			if(j == 0)
				offsetY = 1;
			else
				offsetY = -1;
			int index2 = (j+offsetY)*uNum + (i+offsetX);
			vDerv = vDervGPUData[index2];
		}
//		normalData[index] = normalize(cross(uDerv, vDerv));
		normalData[index] = cross(uDerv, vDerv);
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDAExactNormalEvaluate(float3* uDervGPUData, float3* vDervGPUData, int uNum, int vNum, float3* normal)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
    CUDAExactNormalEvalKernel<<<grid, block>>>(uDervGPUData, vDervGPUData, uNum, vNum, normal);
	return 1;
}