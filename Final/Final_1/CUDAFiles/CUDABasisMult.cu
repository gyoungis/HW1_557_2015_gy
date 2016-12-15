#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

__global__ void CUDABasisMultKernel(float* ctrlPts, int* uCtrlData, int* vCtrlData, float* uBasisData, float* vBasisData, float4* surfPt, int ku, int kv, int nu, int uNum, int vNum, int uPass, int vPass)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		float uBasis	= uBasisData[i*ku + uPass];
		float vBasis	= vBasisData[j*kv + vPass];
		int uCtrlIndexPos = uCtrlData[i]+uPass;
		int vCtrlIndexPos = vCtrlData[j]+vPass;
		float ctrlPtx	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 0];
		float ctrlPty	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 1];
		float ctrlPtz	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 2];
		float ctrlPtw	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 3];

		float4 surfacePt = make_float4(0,0,0,0);
		
		surfacePt.w = ctrlPtw*uBasis*vBasis;
		surfacePt.x = ctrlPtx*surfacePt.w;
		surfacePt.y = ctrlPty*surfacePt.w;
		surfacePt.z = ctrlPtz*surfacePt.w;
		
		if (uPass==0 && vPass==0)
			surfPt[j*uNum+i] = surfacePt;
		else
			surfPt[j*uNum+i] += surfacePt;
	}
}

__global__ void CUDABasisMultKernelNoPP(float* ctrlPts, int* uCtrlData, int* vCtrlData, float* uBasisData, float* vBasisData, float4* surfPt, int ku, int kv, int nu, int uNum, int vNum)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		for(int vPass=0; vPass<kv; vPass++)
		{
			for(int uPass=0; uPass<ku; uPass++)
			{
				float uBasis	= uBasisData[i*ku + uPass];
				float vBasis	= vBasisData[j*kv + vPass];
				int uCtrlIndexPos = uCtrlData[i]+uPass;
				int vCtrlIndexPos = vCtrlData[j]+vPass;
				float ctrlPtx	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 0];
				float ctrlPty	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 1];
				float ctrlPtz	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 2];
				float ctrlPtw	= ctrlPts[(vCtrlIndexPos*nu + uCtrlIndexPos)*4 + 3];

				float4 surfacePt = make_float4(0,0,0,0);
				
				surfacePt.w = ctrlPtw*uBasis*vBasis;
				surfacePt.x = ctrlPtx*surfacePt.w;
				surfacePt.y = ctrlPty*surfacePt.w;
				surfacePt.z = ctrlPtz*surfacePt.w;
				
				if (uPass==0 && vPass==0)
					surfPt[j*uNum+i] = surfacePt;
				else
					surfPt[j*uNum+i] += surfacePt;
			}
		}
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDABasisMult(float* ctrlPts, int* uCtrlData, int* vCtrlData, float* uBasisData, float* vBasisData, float4* surfPt, int uNum, int vNum, int nu, int nv, int ku, int kv, int uPass, int vPass)
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
	if (uPass >= 0 && vPass >= 0)
	    CUDABasisMultKernel<<<grid, block>>>(ctrlPts, uCtrlData, vCtrlData, uBasisData, vBasisData, surfPt, ku, kv, nu, uNum, vNum, uPass, vPass);
	else
	    CUDABasisMultKernelNoPP<<<grid, block>>>(ctrlPts, uCtrlData, vCtrlData, uBasisData, vBasisData, surfPt, ku, kv, nu, uNum, vNum);
	return 1;
}