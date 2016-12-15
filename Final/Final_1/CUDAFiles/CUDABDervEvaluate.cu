#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDABasisDervEvalKernel(float* knotData, int kVal, int evalNum, float* basisData)
{
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i >= 0 && i <= kVal && j < evalNum)
	{
		int index = j*kVal + i;
		float d = 0;
		float e = 0;
		int knotIndex = j*(2*kVal+1) + i;
		//float u			= knotData[knotIndex - i];
		float prevI		= basisData[index];
		float prevI1	= 0.0;
		if (i < kVal-1)
			prevI1 = basisData[index + 1];
		
		float knotI		= knotData[knotIndex + 1];
		float knotIK1	= knotData[knotIndex + kVal];
		float knotIK = knotData[knotIndex + kVal + 1];
		float knotI1 = knotData[knotIndex + 2];
		if ( prevI!= 0 && knotIK1 != knotI)
			d = ((kVal-1)*prevI)/(knotIK1-knotI);
		if ( prevI1!= 0 && knotIK != knotI1)
			e = ((kVal-1)*prevI1)/(knotIK-knotI1);
		basisData[index] = d - e;
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDABasisDervEvaluate(float* knotData, int kVal, int evalNum, float* basisData)
{
    // execute the kernel
    dim3 block(GetBlockSize(kVal,4), GetBlockSize(evalNum,64), 1);
	dim3 grid(GetGridSize(kVal,block.x), GetGridSize(evalNum,block.y), 1);
	if ((block.x*block.y < 512) && (grid.x*grid.y < 65536))
		CUDABasisDervEvalKernel<<<grid, block>>>(knotData, kVal, evalNum, basisData);
	else
		return 0;
	return 1;
}