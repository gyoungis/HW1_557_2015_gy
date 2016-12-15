#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDAExpansionFactorEvalKernel(float3* dx2duuData, float3* dx2duvData, float3* dx2dvuData, float3* dx2dvvData, int uNum, int vNum, float* k1Data, float* k2Data, float* k3Data, float* k4Data)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum+1 && j < vNum+1)
	{
		int index = j*(uNum+1)+i;

		float dx2duuLen = length(dx2duuData[index]);
		float dx2duvLen = length(dx2duvData[index]);
		float dx2dvuLen = length(dx2dvuData[index]);
		float dx2dvvLen = length(dx2dvvData[index]);
		
		k1Data[index] = dx2duuLen/(8.0*uNum*uNum);
		k2Data[index] = dx2duvLen/(8.0*uNum*vNum);
		k3Data[index] = dx2dvuLen/(8.0*uNum*vNum);
		k4Data[index] = dx2dvvLen/(8.0*vNum*vNum);
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDAExpansionFactorEvaluate(float3* secondDerivatives[4], int uNum, int vNum, float* k1CUDAData, float* k2CUDAData, float* k3CUDAData, float* k4CUDAData)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum+1,16), GetBlockSize(vNum+1,16), 1);
	dim3 grid(GetGridSize(uNum+1,block.x), GetGridSize(vNum+1,block.y), 1);
    CUDAExpansionFactorEvalKernel<<<grid, block>>>(secondDerivatives[0], secondDerivatives[1], secondDerivatives[2], secondDerivatives[3], uNum, vNum, k1CUDAData, k2CUDAData, k3CUDAData, k4CUDAData);
	return 1;
}