#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDANormalEvalKernel(float4* surfPts, int uNum, int vNum, float3* normal)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		int index = j*uNum+i;
		float4 c1 = surfPts[index];

		// Perspective divide
		float3 d1 = make_float3(c1.x, c1.y, c1.z)/c1.w ;
		float3 d2 = make_float3(0,0,0);
		float3 d4 = make_float3(0,0,0);
		float3 d6 = make_float3(0,0,0);
		float3 d8 = make_float3(0,0,0);
			
		if (i != uNum-1)
		{
			float4 c2 = surfPts[index + 1];
			float3 d = make_float3(c2.x, c2.y, c2.z)/c2.w;
			d2 = normalize(d-d1);
		}
			
		if (j != 0)
		{
			float4 c4 = surfPts[index - uNum];
			float3 d = make_float3(c4.x, c4.y, c4.z)/c4.w;
			d4 = normalize(d-d1);
		}

		if (j != vNum-1)
		{
			float4 c8 = surfPts[index + uNum];
			float3 d = make_float3(c8.x, c8.y, c8.z)/c8.w;
			d8 = normalize(d-d1);
		}

		if (i != 0)
		{
			float4 c6 = surfPts[index - 1];
			float3 d = make_float3(c6.x, c6.y, c6.z)/c6.w;
			d6 = normalize(d-d1);
		}
			

		float3 norm = (cross(d2,d4) + cross(d4,d6) + cross(d6,d8) + cross(d8,d2));
		normal[j*uNum+i] = -normalize(norm);
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDANormalEvaluate(float4* surfPts, int uNum, int vNum, float3* normal)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
    CUDANormalEvalKernel<<<grid, block>>>(surfPts, uNum, vNum, normal);
	return 1;
}