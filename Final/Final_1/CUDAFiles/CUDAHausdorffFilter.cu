#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"
#define EPSILON 1e-5

__global__ void CUDAHausdorffFilter1Kernel(float* minData, float* maxData, float minHDVal, int texSize, int* output)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;
	if (i < texSize && j < texSize)
	{
		int index = j*texSize+i;
		int outVal = 1;
		if ((maxData[index]-minHDVal) < EPSILON || minData[index] < 0)
			outVal = 0;
		output[index] = outVal;
	}
}

__global__ void CUDAHausdorffFilter2Kernel(float* minData, int* rowData, float* minRowMaxData, int* filterRowData, int texSize, int* output)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;
	if (i < texSize && j < texSize)
	{
		int index = j*texSize+i;
		int outVal = 1;
		float minVal = minData[index];
		int rowNumber = rowData[index];

		if (rowNumber < 0)
		{
			outVal = 0;
			rowNumber = 0;
		}
		float minRowMaxVal	= minRowMaxData[rowNumber];
		int filterRowVal = filterRowData[rowNumber];
		if (minVal > minRowMaxVal || filterRowVal < 0.5 || minVal < 0)
			outVal = 0;
		output[index] = outVal;
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDAHausdoffFilter1(float* minData, float* maxData, float minHDVal, int texSize, int* output)
{
    // execute the kernel
    dim3 block(GetBlockSize(texSize,16), GetBlockSize(texSize,16), 1);
	dim3 grid(GetGridSize(texSize,block.x), GetGridSize(texSize,block.y), 1);
    CUDAHausdorffFilter1Kernel<<<grid, block>>>(minData, maxData, minHDVal, texSize, output);
	return 1;
}

extern "C" int CUDAHausdoffFilter2(float* minData, int* rowData, float* minRowMaxData, int* filterRowData, int texSize, int* output)
{
    // execute the kernel
    dim3 block(GetBlockSize(texSize,16), GetBlockSize(texSize,16), 1);
	dim3 grid(GetGridSize(texSize,block.x), GetGridSize(texSize,block.y), 1);
    CUDAHausdorffFilter2Kernel<<<grid, block>>>(minData, rowData, minRowMaxData, filterRowData, texSize, output);
	return 1;
}