#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"

texture<float4, 2, cudaReadModeElementType> ctrlPointsGPUTexture;
texture<float, 2, cudaReadModeElementType> uBasisGPUTexture;
texture<float, 2, cudaReadModeElementType> vBasisGPUTexture;
texture<int, 1, cudaReadModeElementType> uCtrlDataGPUTexture;
texture<int, 1, cudaReadModeElementType> vCtrlDataGPUTexture;

__global__ void CUDABasisMultKernelTextures(float4* surfPt, int ku, int kv, int nu, int uNum, int vNum, int uPass, int vPass)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		float uBasis		= tex2D(uBasisGPUTexture, uPass, i);
		float vBasis		= tex2D(vBasisGPUTexture, vPass, j);
		int uCtrlIndexPos	= tex1D(uCtrlDataGPUTexture,i) + uPass;
		int vCtrlIndexPos	= tex1D(vCtrlDataGPUTexture,j) + vPass;
		float4 ctrlPt		= tex2D(ctrlPointsGPUTexture, uCtrlIndexPos, vCtrlIndexPos);

		float4 surfacePt = make_float4(0,0,0,0);
		
		surfacePt.w = ctrlPt.w*uBasis*vBasis;
		surfacePt.x = ctrlPt.x*surfacePt.w;
		surfacePt.y = ctrlPt.y*surfacePt.w;
		surfacePt.z = ctrlPt.z*surfacePt.w;
		
		if (uPass==0 && vPass==0)
			surfPt[j*uNum+i] = surfacePt;
		else
			surfPt[j*uNum+i] += surfacePt;
	}
//	surfPt[j*uNum+i] = 
//		make_float4(ctrlPtx,ctrlPty,ctrlPtz,ctrlPtw);
//		make_float4(uBasis,vBasis,uCtrlIndexPos,vCtrlIndexPos);
}

__global__ void CUDABasisMultKernelTexturesNoPP(float4* surfPt, int ku, int kv, int nu, int uNum, int vNum)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		for(int vPass=0; vPass<kv; vPass++)
		{
			for(int uPass=0; uPass<ku; uPass++)
			{
				float uBasis		= tex2D(uBasisGPUTexture, uPass, i);
				float vBasis		= tex2D(vBasisGPUTexture, vPass, j);
				int uCtrlIndexPos	= tex1D(uCtrlDataGPUTexture,i) + uPass;
				int vCtrlIndexPos	= tex1D(vCtrlDataGPUTexture,j) + vPass;
				float4 ctrlPt		= tex2D(ctrlPointsGPUTexture, uCtrlIndexPos, vCtrlIndexPos);

				float4 surfacePt = make_float4(0,0,0,0);
				
				surfacePt.w = ctrlPt.w*uBasis*vBasis;
				surfacePt.x = ctrlPt.x*surfacePt.w;
				surfacePt.y = ctrlPt.y*surfacePt.w;
				surfacePt.z = ctrlPt.z*surfacePt.w;
				
				if (uPass==0 && vPass==0)
					surfPt[j*uNum+i] = surfacePt;
				else
					surfPt[j*uNum+i] += surfacePt;
			}
		}
	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDABasisMultTextures(cudaArray* ctrlPtsArray, cudaArray* uCtrlData, cudaArray* vCtrlData, cudaArray* uBasisData, cudaArray* vBasisData, float4* surfPt, int uNum, int vNum, int nu, int nv, int ku, int kv, int uPass, int vPass)
{
	// Allocate array and copy Texture data
    cudaChannelFormatDesc float4ChannelDesc = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
    cudaChannelFormatDesc float1ChannelDesc	= cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
    cudaChannelFormatDesc int1ChannelDesc	= cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindSigned);

    // Set texture parameters
    ctrlPointsGPUTexture.addressMode[0] = cudaAddressModeClamp;
    ctrlPointsGPUTexture.addressMode[1] = cudaAddressModeClamp;
    ctrlPointsGPUTexture.filterMode = cudaFilterModePoint;
    ctrlPointsGPUTexture.normalized = false;

    uBasisGPUTexture.addressMode[0] = cudaAddressModeClamp;
    uBasisGPUTexture.addressMode[1] = cudaAddressModeClamp;
    uBasisGPUTexture.filterMode = cudaFilterModePoint;
    uBasisGPUTexture.normalized = false;

    vBasisGPUTexture.addressMode[0] = cudaAddressModeClamp;
    vBasisGPUTexture.addressMode[1] = cudaAddressModeClamp;
    vBasisGPUTexture.filterMode = cudaFilterModePoint;
    vBasisGPUTexture.normalized = false;

    uCtrlDataGPUTexture.addressMode[0] = cudaAddressModeClamp;
    uCtrlDataGPUTexture.filterMode = cudaFilterModePoint;
    uCtrlDataGPUTexture.normalized = false;

	vCtrlDataGPUTexture.addressMode[0] = cudaAddressModeClamp;
    vCtrlDataGPUTexture.filterMode = cudaFilterModePoint;
    vCtrlDataGPUTexture.normalized = false;

	// Bind the array to the texture
   cudaBindTextureToArray(ctrlPointsGPUTexture, ctrlPtsArray, float4ChannelDesc);
   cudaBindTextureToArray(uBasisGPUTexture, uBasisData, float1ChannelDesc);
   cudaBindTextureToArray(vBasisGPUTexture, vBasisData, float1ChannelDesc);
   cudaBindTextureToArray(uCtrlDataGPUTexture, uCtrlData, int1ChannelDesc);
   cudaBindTextureToArray(vCtrlDataGPUTexture, vCtrlData, int1ChannelDesc);

	// execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
	if (uPass >= 0 && vPass >= 0)
	    CUDABasisMultKernelTextures<<<grid, block>>>(surfPt, ku, kv, nu, uNum, vNum, uPass, vPass);
	else
	    CUDABasisMultKernelTexturesNoPP<<<grid, block>>>(surfPt, ku, kv, nu, uNum, vNum);
	return 1;
}