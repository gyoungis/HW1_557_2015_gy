#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"


// xx	d2	xx
// d4	d1	d8
// xx	d6	xx
__global__ void CUDASecondDerivativeEvalKernel(float3* uSurfDerv, float3* vSurfDerv, int uNum, int vNum, float uInterval, float vInterval, float3* dx2duuData, float3* dx2duvData, float3* dx2dvuData, float3* dx2dvvData)
{
    unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
    unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	if (i < uNum && j < vNum)
	{
		float3 dx2duu, dx2duv, dx2dvu, dx2dvv;
		int index1 = j*uNum+i;

		if (i == uNum-1)
		{
			int index4 = j*uNum+i-1;
			float3 du4 = uSurfDerv[index4];
			float3 du1 = uSurfDerv[index1];
			float3 dv4 = vSurfDerv[index4];
			float3 dv1 = vSurfDerv[index1];
			dx2duu = uInterval*(du1-du4);
			dx2duv = uInterval*(dv1-dv4);
		}
		else if (i == 0)
		{
			int index8 = j*uNum+i+1;
			float3 du8 = uSurfDerv[index8];
			float3 du1 = uSurfDerv[index1];
			float3 dv8 = vSurfDerv[index8];
			float3 dv1 = vSurfDerv[index1];
			dx2duu = uInterval*(du8-du1);
			dx2duv = uInterval*(dv8-dv1);
		}
		else
		{
			int index8 = j*uNum+i+1;
			int index4 = j*uNum+i-1;
			float3 du8 = uSurfDerv[index8];
			float3 du4 = uSurfDerv[index4];
			float3 dv8 = vSurfDerv[index8];
			float3 dv4 = vSurfDerv[index4];
			dx2duu = (uInterval/2.0)*(du8-du4);
			dx2duv = (uInterval/2.0)*(dv8-dv4);
		}

		if (j == vNum-1)
		{
			int index6 = (j-1)*uNum+i;
			float3 du6 = uSurfDerv[index6];
			float3 du1 = uSurfDerv[index1];
			float3 dv6 = vSurfDerv[index6];
			float3 dv1 = vSurfDerv[index1];
			dx2dvv = vInterval*(dv1-dv6);
			dx2dvu = vInterval*(du1-du6);
		}
		else if (j == 0)
		{
			int index2 = (j+1)*uNum+i;
			float3 du1 = uSurfDerv[index1];
			float3 du2 = uSurfDerv[index2];
			float3 dv1 = vSurfDerv[index1];
			float3 dv2 = vSurfDerv[index2];
			dx2dvv = vInterval*(dv2-dv1);
			dx2dvu = vInterval*(du2-du1);
		}
		else
		{
			int index6 = (j-1)*uNum+i;
			int index2 = (j+1)*uNum+i;
			float3 du6 = uSurfDerv[index6];
			float3 du2 = uSurfDerv[index2];
			float3 dv6 = vSurfDerv[index6];
			float3 dv2 = vSurfDerv[index2];
			dx2dvv = (vInterval/2.0)*(dv2-dv6);
			dx2dvu = (vInterval/2.0)*(du2-du6);
		}

		dx2duuData[index1] = dx2duu;
		dx2duvData[index1] = dx2duv;
		dx2dvuData[index1] = dx2dvu;
		dx2dvvData[index1] = dx2dvv;

//		dx2duvData[index1] = make_float3(length(dx2duv)-length(dx2dvu),0,0);
//		dx2dvuData[index1] = make_float3(length(dx2dvu),0,0);

	}
}

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDASecondDerivativeEvaluate(float3* uSurfDerv, float3* vSurfDerv, int uNum, int vNum, float uInterval, float vInterval, float3* secondDerivatives[4])
{
    // execute the kernel
    dim3 block(GetBlockSize(uNum,16), GetBlockSize(vNum,16), 1);
	dim3 grid(GetGridSize(uNum,block.x), GetGridSize(vNum,block.y), 1);
    CUDASecondDerivativeEvalKernel<<<grid, block>>>(uSurfDerv, vSurfDerv, uNum, vNum, uInterval, vInterval, secondDerivatives[0], secondDerivatives[1], secondDerivatives[2], secondDerivatives[3]);
	return 1;
}