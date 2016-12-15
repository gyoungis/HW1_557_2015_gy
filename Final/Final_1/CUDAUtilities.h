// Copyright (c) 2009 by Adarsh Krishnamurthy and The Regents of the University of 
// California. All rights reserved.
//
// Permission to use, copy, modify, and distribute this software and its
// documentation for non-profit use, without fee, and without written agreement is
// hereby granted, provided that the above copyright notice and the following
// two paragraphs appear in all copies of this software.
//
// IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
// DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
// OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
// CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
// ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

#pragma once
#include "Includes.h"
#include "GLParameters.h"
#include "NURBS.h"

#ifdef CUDA
inline int CUTGetMaxGflopsDeviceID()
{
	int device_count = 0;
	cudaGetDeviceCount( &device_count );

	cudaDeviceProp device_properties;
	int max_gflops_device = 0;
	int max_gflops = 0;
	
	int current_device = 0;
	cudaGetDeviceProperties( &device_properties, current_device );
	max_gflops = device_properties.multiProcessorCount * device_properties.clockRate;
	++current_device;

	while( current_device < device_count )
	{
		cudaGetDeviceProperties( &device_properties, current_device );
		int gflops = device_properties.multiProcessorCount * device_properties.clockRate;
		if( gflops > max_gflops )
		{
			max_gflops        = gflops;
			max_gflops_device = current_device;
		}
		++current_device;
	}

	return max_gflops_device;
}

void InitializeCUDA();
void CUDACheckErrors(char* label = NULL);
Float4 CPUFindMax(int w, int h, float* k1CUDAPointer, float* k2CUDAPointer, float* k3CUDAPointer, float* k4CUDAPointer);
Float4 CPUFindAvg(int w, int h, float* k1CUDAPointer, float* k2CUDAPointer, float* k3CUDAPointer, float* k4CUDAPointer);

#ifdef THRUST
Float4 THRUSTFindMax(int w, int h, float* k1CUDAPointer, float* k2CUDAPointer, float* k3CUDAPointer, float* k4CUDAPointer);
extern "C" float THRUSTDeviceFindMax(float* dataCUDAPointer, int w, int h);
extern "C" float THRUSTDeviceFindMaxFloat4(float4* dataCUDAPointer, int numValues, float4* maxVal, int* maxLoc);
//extern "C" void THRUSTDeviceMinMaxSort(float* minDataCUDAPointer, float* maxDataCUDAPointer, int* rowDataCUDAPointer, int n);
extern "C" void THRUSTDeviceMinMaxSort(float* minDataCUDAPointer, float* maxDataCUDAPointer, int* rowDataCUDAPointer, int n, float* minData, float* maxData, int* rowNumbers);
#endif

extern "C" int CUDABasisMult(float* ctrlPts, int* uCtrlData, int* vCtrlData, float* uBasisData, float* vBasisData, float4* surfPt, int uNum, int vNum, int nu, int nv, int ku, int kv, int uPass, int vPass);
extern "C" int CUDABasisMultTextures(cudaArray* ctrlPts, cudaArray* uCtrlData, cudaArray* vCtrlData, cudaArray* uBasisData, cudaArray* vBasisData, float4* surfPt, int uNum, int vNum, int nu, int nv, int ku, int kv, int uPass, int vPass);
extern "C" int CUDABasisEvaluate(float* knotData, int pass, int kVal, int evalNum, float* basisData);
extern "C" int CUDABasisDervEvaluate(float* knotData, int kVal, int evalNum, float* basisData);
extern "C" int CUDANormalEvaluate(float4* surfPts, int uNum, int vNum, float3* normal);
extern "C" int CUDASecondDerivativeEvaluate(float3* uSurfDerv, float3* vSurfDerv, int uNum, int vNum, float uInterval, float vInterval, float3* secondDerivatives[4]);
extern "C" int CUDAExactNormalEvaluate(float3* uDervGPUData, float3* vDervGPUData, int uNum, int vNum, float3* normal);
extern "C" int CUDARationalDivide(float4* surfPts, float4* dervPts, int uNum, int vNum, float3* output);
extern "C" int CUDAExpansionFactorEvaluate(float3* secondDerivatives[4], int uNum, int vNum, float* k1CUDAData, float* k2CUDAData, float* k3CUDAData, float* k4CUDAData);
extern "C" int CUDATransformEvaluatedPoints(float4* surfPts, int uNum, int vNum, float T[16], float4* output);
extern "C" int CUDABoundingBoxEvaluate(float4* surfPts, int uNum, int vNum, float magFactor, float3* bBoxMinData, float3* bBoxMaxData);
extern "C" int CUDABoundingBoxHierarchy(float3* minPts, float3* maxPts, int uNum, int vNum, int currentSize, int xReadPos);
extern "C" int CUDABBoxHausdorffDistance(float3* bBoxMinA, float3* bBoxMaxA, float3* bBoxMinB, float3* bBoxMaxB, int* referenceListAx, int* referenceListAy, int* referenceListBx, int* referenceListBy, int* rowNumbers, int texSize, int levelOriginY, int uNum, float* minDistance, float* maxDistance, int* rowNumberOut);
extern "C" int CUDAHausdoffFilter1(float* minData, float* maxData, float minHDVal, int texSize, int* output);
extern "C" int CUDAHausdoffFilter2(float* minData, int* rowData, float* minRowMaxData, int* filterRowData, int texSize, int* output);
extern "C" int CUDATriTriHausdorffDistance(float4* surface1Points, float4* surface2Points, int* referenceListAx, int* referenceListAy, int* referenceListBx, int* referenceListBy, int* rowNumbers, int referenceListSize, int numOverlaps, int uNum, float* hausdorffDistance, float3* surface1HDPoint, float3* surface2HDPoint);

//iddo:
//extern "C" int CUDAHausdorffIddo(float4* surf1Pts, float4* prevDists, int uNum1, int vNum1, float4* surface2Cps, int ordU, int ordV, float4* oPoint);
extern "C" int CUDAHausdorffIddo(float4* surf1Pts, float4* prevDists, float* prevConvStatus, int uNum1, int vNum1, float4* surface2Cps, int ordU, int ordV, float kBStopTolerance, float4* oPoint, float* convStatus);

#endif
