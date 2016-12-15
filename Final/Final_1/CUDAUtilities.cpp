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

#include "CUDAUtilities.h"
#include "GPUUtilities.h"
#include "NURBS.h"

#ifdef CUDA
void CUDACheckErrors(char* label)
{
#ifdef CHECKCUDAERRORS
	// Need to synchronise first to catch errors due to
	// asynchroneous operations that would otherwise
	// potentially go unnoticed

	cudaError_t error;
	error = cudaThreadSynchronize();
	error = cudaGetLastError();
	if (error != cudaSuccess)
	{
		char *errStr = (char*) cudaGetErrorString(error);
		cout << "CUDA Error: " << label << " " << errStr << endl;
	}
#endif
}

void InitializeCUDA()
{
	cudaSetDevice(CUTGetMaxGflopsDeviceID());
	CUDACheckErrors(" Init ");
}

Float4 CPUFindMax(int w, int h, float* k1CUDAPointer, float* k2CUDAPointer, float* k3CUDAPointer, float* k4CUDAPointer)
{
	float* tempK1Data = new float[w*h];
	cudaMemcpy(tempK1Data, k1CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);
	float* tempK2Data = new float[w*h];
	cudaMemcpy(tempK2Data, k2CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);
	float* tempK3Data = new float[w*h];
	cudaMemcpy(tempK3Data, k3CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);
	float* tempK4Data = new float[w*h];
	cudaMemcpy(tempK4Data, k4CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);

#ifdef DUMP
	WriteToFile(tempKData, uNum+1, vNum+1, 0);
#endif

	Float4 maxVal = Float4(tempK1Data[0], tempK2Data[0], tempK3Data[0], tempK4Data[0]);
	for (int j=0; j < h; j++)
	{
		for (int i=0; i < w; i++)
		{
			if (tempK1Data[j*w+i] > maxVal[0])
				maxVal[0] = tempK1Data[j*w+i];
			if (tempK2Data[j*w+i] > maxVal[1])
				maxVal[1] = tempK2Data[j*w+i];
			if (tempK3Data[j*w+i] > maxVal[2])
				maxVal[2] = tempK3Data[j*w+i];
			if (tempK4Data[j*w+i] > maxVal[3])
				maxVal[3] = tempK4Data[j*w+i];
		}
	}

	delete [] tempK1Data;
	delete [] tempK2Data;
	delete [] tempK3Data;
	delete [] tempK4Data;
	return maxVal;
}

Float4 CPUFindAvg(int w, int h, float* k1CUDAPointer, float* k2CUDAPointer, float* k3CUDAPointer, float* k4CUDAPointer)
{
	float* tempK1Data = new float[w*h];
	cudaMemcpy(tempK1Data, k1CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);
	float* tempK2Data = new float[w*h];
	cudaMemcpy(tempK2Data, k2CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);
	float* tempK3Data = new float[w*h];
	cudaMemcpy(tempK3Data, k3CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);
	float* tempK4Data = new float[w*h];
	cudaMemcpy(tempK4Data, k4CUDAPointer, w*h*sizeof(float), cudaMemcpyDeviceToHost);

#ifdef DUMP
	WriteToFile(tempKData, uNum+1, vNum+1, 0);
#endif

	Float4 maxVal = Float4(0, 0, 0, 0);
	for (int j=0; j < h; j++)
	{
		for (int i=0; i < w; i++)
		{
			maxVal[0] += tempK1Data[j*w+i];
			maxVal[1] += tempK2Data[j*w+i];
			maxVal[2] += tempK3Data[j*w+i];
			maxVal[3] += tempK4Data[j*w+i];
		}
	}

	delete [] tempK1Data;
	delete [] tempK2Data;
	delete [] tempK3Data;
	delete [] tempK4Data;
	return maxVal/(w*h);
}

#ifdef THRUST
Float4 THRUSTFindMax(int w, int h, float* k1CUDAPointer, float* k2CUDAPointer, float* k3CUDAPointer, float* k4CUDAPointer)
{
	Float4 maxVal = Float4(-1, -1, -1, -1);

	maxVal[0] = THRUSTDeviceFindMax(k1CUDAPointer, w, h);
	maxVal[1] = THRUSTDeviceFindMax(k2CUDAPointer, w, h);
	maxVal[2] = THRUSTDeviceFindMax(k3CUDAPointer, w, h);
	maxVal[3] = THRUSTDeviceFindMax(k4CUDAPointer, w, h);
	return maxVal;
}
#endif
#endif

