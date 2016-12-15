#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDATriTriHausdorffDistanceKernel(float4* surface1Points, float4* surface2Points, int* referenceListAx, int* referenceListAy, int* referenceListBx, int* referenceListBy, int* rowNumbers, int referenceListSize, int numOverlaps, int uNum, float* hausdorffDistance, float3* surface1HDPoint, float3* surface2HDPoint)
{
    unsigned int indexI = blockIdx.x*blockDim.x + threadIdx.x;
	unsigned int indexJ = blockIdx.y*blockDim.y + threadIdx.y;
	int k = indexJ*referenceListSize + indexI;

	if (indexI < referenceListSize && indexJ < referenceListSize && k < numOverlaps)
	{
		int uIndex1 = (int) referenceListAx[k]/2.0f;
		int vIndex1 = (int) referenceListAy[k]/2.0f;
		int uIndex2 = (int) referenceListBx[k]/2.0f;
		int vIndex2 = (int) referenceListBy[k]/2.0f;

		float3 A[4];
		float3 B[4];
		A[0] = make_float3(surface1Points[(vIndex1+0)*uNum+(uIndex1+0)]/surface1Points[(vIndex1+0)*uNum+(uIndex1+0)].w);
		A[1] = make_float3(surface1Points[(vIndex1+0)*uNum+(uIndex1+1)]/surface1Points[(vIndex1+0)*uNum+(uIndex1+1)].w);
		A[2] = make_float3(surface1Points[(vIndex1+1)*uNum+(uIndex1+1)]/surface1Points[(vIndex1+1)*uNum+(uIndex1+1)].w);
		A[3] = make_float3(surface1Points[(vIndex1+1)*uNum+(uIndex1+0)]/surface1Points[(vIndex1+1)*uNum+(uIndex1+0)].w);

		B[0] = make_float3(surface2Points[(vIndex2+0)*uNum+(uIndex2+0)]/surface2Points[(vIndex2+0)*uNum+(uIndex2+0)].w);
		B[1] = make_float3(surface2Points[(vIndex2+0)*uNum+(uIndex2+1)]/surface2Points[(vIndex2+0)*uNum+(uIndex2+1)].w);
		B[2] = make_float3(surface2Points[(vIndex2+1)*uNum+(uIndex2+1)]/surface2Points[(vIndex2+1)*uNum+(uIndex2+1)].w);
		B[3] = make_float3(surface2Points[(vIndex2+1)*uNum+(uIndex2+0)]/surface2Points[(vIndex2+1)*uNum+(uIndex2+0)].w);
	
		float hausdorffDist = -1;
		float3 surf1HDPoint;
		float3 surf2HDPoint;
		for (int j = 0; j < 4; j++)
		{
			float minDist = -1;
			float3 tempPoint1, tempPoint2;
			for (int i = 0; i < 4; i++)
			{
				float dist = length(A[i] - B[j]);

				if ((dist < minDist || minDist == -1))
				{
					tempPoint1 = A[i];
					tempPoint2 = B[j];
					minDist = dist;
				}
			}
			if (minDist > hausdorffDist || hausdorffDist == -1)
			{
				hausdorffDist = minDist;
				surf1HDPoint = tempPoint1;
				surf2HDPoint = tempPoint2;
			}
		}
		hausdorffDistance[k] = hausdorffDist;
		surface2HDPoint[k] = surf1HDPoint;
		surface1HDPoint[k] = surf2HDPoint;
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDATriTriHausdorffDistance(float4* surface1Points, float4* surface2Points, int* referenceListAx, int* referenceListAy, int* referenceListBx, int* referenceListBy, int* rowNumbers, int referenceListSize, int numOverlaps, int uNum, float* hausdorffDistance, float3* surface1HDPoint, float3* surface2HDPoint)
{
    // execute the kernel
	dim3 block(GetBlockSize(referenceListSize,8), GetBlockSize(referenceListSize,8), 1);
	dim3 grid(GetGridSize(referenceListSize,block.x), GetGridSize(referenceListSize,block.y), 1);
	CUDATriTriHausdorffDistanceKernel<<<grid, block>>>(surface1Points, surface2Points, referenceListAx, referenceListAy, referenceListBx, referenceListBy, rowNumbers, referenceListSize, numOverlaps, uNum, hausdorffDistance, surface1HDPoint, surface2HDPoint);
	return 1;
}