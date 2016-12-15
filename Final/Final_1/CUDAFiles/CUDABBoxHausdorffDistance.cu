#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDABBoxHausdorffDistanceKernel(float3* bBoxMinA, float3* bBoxMaxA, float3* bBoxMinB, float3* bBoxMaxB, int* referenceListAx, int* referenceListAy, int* referenceListBx, int* referenceListBy, int* rowNumbers, int texSize, int levelOriginY, int uNum, float* minDistance, float* maxDistance, int* rowNumberOut)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < texSize && j < texSize)
	{
		int index = j*texSize + i;
		int indexA = i%4;
		int indexB = j%4;

		int referenceListIndex = floor(j/4.0)*texSize/4 + floor(i/4.0);
		int indexOffsetAx = referenceListAx[referenceListIndex];
		int indexOffsetAy = referenceListAy[referenceListIndex];
		int indexOffsetBx = referenceListBx[referenceListIndex];
		int indexOffsetBy = referenceListBy[referenceListIndex];
		int rowNumber = rowNumbers[referenceListIndex];

		int currentRowNumber = -1;
		float minBBoxDist = 1e6;
		float maxBBoxDist = 1e6;

		if (indexOffsetAx >= 0)
		{
			currentRowNumber = rowNumber*4 + indexB;

			int texIndexAi = indexOffsetAx + indexA%2;
			int texIndexAj = levelOriginY + indexOffsetAy + floor(indexA/2.0f);
			int bBoxIndexA = texIndexAj*uNum + texIndexAi;

			int texIndexBi = indexOffsetBx + indexB%2;
			int texIndexBj = levelOriginY + indexOffsetBy + floor(indexB/2.0f);
			int bBoxIndexB = texIndexBj*uNum + texIndexBi;

			float3 minPointA = bBoxMinA[bBoxIndexA];
			float3 maxPointA = bBoxMaxA[bBoxIndexA];
			float3 minPointB = bBoxMinB[bBoxIndexB];
			float3 maxPointB = bBoxMaxB[bBoxIndexB];

			float3 boxCenterA	= (maxPointA + minPointA) * 0.5f;
			float3 boxExtentA	= (maxPointA - minPointA) * 0.5f;

			float3 boxCenterB	= (maxPointB + minPointB) * 0.5f;
			float3 boxExtentB	= (maxPointB - minPointB) * 0.5f;

			float3 centerDistAB = fabs(boxCenterB - boxCenterA);
			float3 maxDistAB = centerDistAB + boxExtentA + boxExtentB;
			float3 minDistAB = fmaxf(centerDistAB - boxExtentA - boxExtentB, make_float3(0, 0, 0));

			if (length(boxExtentA) > 0 && length(boxExtentB) > 0)
			{
				minBBoxDist = length(minDistAB); 
				maxBBoxDist = length(maxDistAB);
			}
		}
		minDistance[index] = minBBoxDist;
		maxDistance[index] = maxBBoxDist;
		rowNumberOut[index] = currentRowNumber;
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDABBoxHausdorffDistance(float3* bBoxMinA, float3* bBoxMaxA, float3* bBoxMinB, float3* bBoxMaxB, int* referenceListAx, int* referenceListAy, int* referenceListBx, int* referenceListBy, int* rowNumbers, int texSize, int levelOriginY, int uNum, float* minDistance, float* maxDistance, int* rowNumberOut)
{
    // execute the kernel
    dim3 block(GetBlockSize(texSize,16), GetBlockSize(texSize,16), 1);
	dim3 grid(GetGridSize(texSize,block.x), GetGridSize(texSize,block.y), 1);
    CUDABBoxHausdorffDistanceKernel<<<grid, block>>>(bBoxMinA, bBoxMaxA, bBoxMinB, bBoxMaxB, referenceListAx, referenceListAy, referenceListBx, referenceListBy, rowNumbers, texSize, levelOriginY, uNum, minDistance, maxDistance, rowNumberOut);
	return 1;
}
