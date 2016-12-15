#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDATransformKernel(float4* surfPts, int uNum, int vNum, float4 TCol1, float4 TCol2, float4 TCol3, float4 TCol4, float4* output)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		int index = j*uNum+i;
		float4 a = surfPts[index];
		float4 transformedPoint;
		transformedPoint.x = dot(TCol1,a) + TCol4.x;
		transformedPoint.y = dot(TCol2,a) + TCol4.y;
		transformedPoint.z = dot(TCol3,a) + TCol4.z;
		transformedPoint.w = TCol4.w*a.w;
		output[index] = transformedPoint;
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDATransformEvaluatedPoints(float4* surfPts, int uNum, int vNum, float T[16], float4* output)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
	float4 TCol1 = make_float4(T[0], T[4], T[8], T[3]);
	float4 TCol2 = make_float4(T[1], T[5], T[9], T[7]);
	float4 TCol3 = make_float4(T[2], T[6], T[10], T[11]);
	float4 TCol4 = make_float4(T[12], T[13], T[14], T[15]);

    CUDATransformKernel<<<grid, block>>>(surfPts, uNum, vNum, TCol1, TCol2, TCol3, TCol4, output);
	return 1;
}