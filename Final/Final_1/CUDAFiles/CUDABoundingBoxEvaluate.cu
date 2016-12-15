#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDABoundingBoxKernel(float4* surfPoints, int uNum, int vNum, float magFactor, float3* minData, float3* maxData)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		int index0 = j*(uNum+1)+i;
		int index1 = j*(uNum+1)+(i+1);
		int index2 = (j+1)*(uNum+1)+i;
		int index3 = (j+1)*(uNum+1)+(i+1);

		float4 v0p = surfPoints[index0];
		float4 v1p = surfPoints[index1]; 
		float4 v2p = surfPoints[index2];
		float4 v3p = surfPoints[index3]; 

		// Perspective divide
		float3 v0 = make_float3(v0p/v0p.w);
		float3 v1 = make_float3(v1p/v1p.w);
		float3 v2 = make_float3(v2p/v2p.w);
		float3 v3 = make_float3(v3p/v3p.w);

		float3 cornerPointMin = fminf(fminf(v0,v1),fminf(v2,v3));
		cornerPointMin = cornerPointMin - magFactor;

		float3 cornerPointMax = fmaxf(fmaxf(v0,v1),fmaxf(v2,v3));
		cornerPointMax = cornerPointMax + magFactor;

		int index = j*uNum+i;
		minData[index] = cornerPointMin;
		maxData[index] = cornerPointMax;
	}
}

__global__ void CUDABoundingBoxHierarchyKernel(float3* minPoints, float3* maxPoints, int uNum, int vNum, int currentSize, int yReadPos)
{
    unsigned int writei = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int writej = blockIdx.y*blockDim.y + threadIdx.y;
	unsigned int i = 2*writei;
	unsigned int j = 2*writej;

	int index0 = j*(uNum)+i;
	int index1 = j*(uNum)+(i+1);
	int index2 = (j+1)*(uNum)+i;
	int index3 = (j+1)*(uNum)+(i+1);

	float3 min0 = minPoints[index0];
	float3 min1 = minPoints[index1]; 
	float3 min2 = minPoints[index2];
	float3 min3 = minPoints[index3]; 

	float3 max0 = maxPoints[index0];
	float3 max1 = maxPoints[index1]; 
	float3 max2 = maxPoints[index2];
	float3 max3 = maxPoints[index3]; 

	float3 pointMin = fminf(fminf(min0,min1),fminf(min2,min3));
	float3 pointMax = fmaxf(fmaxf(max0,max1),fmaxf(max2,max3));

	int index = (writej+currentSize*2)*(uNum)+(writei);
	minPoints[index] = pointMin;
	maxPoints[index] = pointMax;
}


// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDABoundingBoxEvaluate(float4* surfPts, int uNum, int vNum, float magFactor, float3* bBoxMinData, float3* bBoxMaxData)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
    CUDABoundingBoxKernel<<<grid, block>>>(surfPts, uNum, vNum, magFactor, bBoxMinData, bBoxMaxData);
	return 1;
}

extern "C" int CUDABoundingBoxHierarchy(float3* minPts, float3* maxPts, int uNum, int vNum, int currentSize, int yReadPos)
{
    // execute the kernel
    dim3 block(GetBlockSize(currentSize,16), GetBlockSize(currentSize,16), 1);
	dim3 grid(GetGridSize(currentSize,block.x), GetGridSize(currentSize,block.y), 1);
    CUDABoundingBoxHierarchyKernel<<<grid, block>>>(minPts, maxPts, uNum, vNum, currentSize, yReadPos);
	return 1;
}