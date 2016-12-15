#include "Hausdorff.h"
#include "GPUUtilities.h"
#include "CUDAUtilities.h"
#ifdef CUDA
// Hausdorff Distance Computation
//bool Index2SortPredicate(Index2& d1, Index2& d2)
//{
//	return d1 < d2;
//}

int CPUFindRenderLocation(int* texCUDAData, int* numHits, int texWidth, int texHeight, vector<int>* locations)
{
	int* positionData = new int[texWidth*texHeight];
	cudaMemcpy(positionData, texCUDAData, texWidth*texHeight*sizeof(int), cudaMemcpyDeviceToHost);
	for (int vCount=0; vCount<texHeight; vCount++)
		for (int uCount=0; uCount<texWidth; uCount++)
			if (positionData[vCount*(texWidth)+uCount] == 1)
					locations->push_back(vCount*(texWidth)+uCount);
	*numHits = locations->size();
	delete [] positionData;
	return 0;
}

void CPURowFilterTexture(int* filter1CUDAData, int* rowCUDAData, int* filter1RowCUDAData, int texWidth, int outTexSize)
{
	int texHeight = texWidth;

	// Read back to CPU
	int* filter1Values = new int[texWidth*texHeight];
	int* filter1RowValues = new int[texWidth*texHeight];

	cudaMemcpy(filter1Values, filter1CUDAData, texWidth*texHeight*sizeof(int), cudaMemcpyDeviceToHost);
	cudaMemcpy(filter1RowValues, rowCUDAData, texWidth*texHeight*sizeof(int), cudaMemcpyDeviceToHost);
	CUDACheckErrors(" Data Copy ");


	// Reduce all values and write to out texture
	float* filterOutValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
		filterOutValues[i] = 1;
	for (int i = 0; i < texWidth*texHeight; i++)
	{
		int currentRowValue = filter1RowValues[i];
		int currentFilterValue = filter1Values[i];
		if (currentRowValue >= 0 && currentFilterValue == 0)
			filterOutValues[currentRowValue] = 0;
	}

	// Upload data to GPU
	cudaMemcpy(filter1RowCUDAData, filterOutValues, outTexSize*outTexSize*sizeof(int), cudaMemcpyHostToDevice);
	CUDACheckErrors(" Data Copy ");
	// WriteToFile(minMaxValues, outTexSize, outTexSize, 1);

	delete[] filter1Values;
	delete[] filter1RowValues;
	delete[] filterOutValues;
}

Float4 CPUFindRowMinMaxBBoxExtent(float* minCUDAData, float* maxCUDAData, int* rowNumbersCUDAData, int numValues, int outTexSize, float* minMaxRowMaxCUDAData)
{
	// Read back to CPU
	float* minValues = new float[numValues];
	float* maxValues = new float[numValues];
	int* rowNumValues = new int[numValues];

	cudaMemcpy(minValues, minCUDAData, numValues*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(maxValues, maxCUDAData, numValues*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(rowNumValues, rowNumbersCUDAData, numValues*sizeof(int), cudaMemcpyDeviceToHost);

	// Local Upper Bound. Smallest Max value for each row
	float* minMaxRowMaxValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
		minMaxRowMaxValues[i] = -1;

	for (int i = 0; i < numValues; i++)
	{
		int currentRowValue = rowNumValues[i];
		if (currentRowValue >= 0)
		{
			assert(currentRowValue < outTexSize*outTexSize);
			float currentMinValue = minValues[i];
			float currentMaxValue = maxValues[i];
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			//(x2.y < x1.y && x2.x > 0) || x1.x < 0)
			if ((currentMaxValue < minMaxRowMaxValues[currentRowValue] && currentMinValue > 0) || minMaxRowMaxValues[currentRowValue] < 0)
				minMaxRowMaxValues[currentRowValue] = currentMaxValue;
		}
	}

	// Upload data to GPU
	cudaMemcpy(minMaxRowMaxCUDAData, minMaxRowMaxValues, outTexSize*outTexSize*sizeof(float), cudaMemcpyHostToDevice);
	CUDACheckErrors(" Row Min Max Upload ");

	//WriteToFile(minMaxRowMaxValues, outTexSize, outTexSize);

	// Global Lower Bound
	// Find Min Min value for each row
	float* minMinRowMinValues = new float[outTexSize*outTexSize];
	float* minMinRowMaxValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		minMinRowMinValues[i] = -1;
		minMinRowMaxValues[i] = -1;
	}
	for (int i = 0; i < numValues; i++)
	{
		int currentRowValue = rowNumValues[i];
		float currentMinValue = minValues[i];
		float currentMaxValue = maxValues[i];
		if (currentRowValue >= 0)
		{
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			// ((x2.x < x1.x && x2.x > 0) || (x2.x == x1.x && x2.y > x1.y) || x1.x < 0)
			if ((currentMinValue < minMinRowMinValues[currentRowValue] && currentMinValue > 0) || (currentMinValue == minMinRowMinValues[currentRowValue] && currentMaxValue > minMinRowMaxValues[currentRowValue]) || minMinRowMinValues[currentRowValue] < 0)
			{
				minMinRowMinValues[currentRowValue] = currentMinValue;
				minMinRowMaxValues[currentRowValue] = currentMaxValue;
			}
		}
	}

	//WriteToFile(minMinRowMinValues, 1, outTexSize*outTexSize);

	// Reduce to find Max Value of MinMin distances
	Float4 maxMinMinValue = Float4(minMinRowMinValues[0], minMinRowMaxValues[0], 0, -1);
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		if (minMinRowMinValues[i] > maxMinMinValue[0])
		{
			maxMinMinValue[0] = minMinRowMinValues[i];
			maxMinMinValue[1] = minMinRowMaxValues[i];
			maxMinMinValue[2] = i;
		}
	}

	delete[] minMaxRowMaxValues;

	delete[] minMinRowMinValues;
	delete[] minMinRowMaxValues;

	delete[] minValues;
	delete[] maxValues;
	delete[] rowNumValues;

	return maxMinMinValue;
}

#ifdef THRUST
Float4 THRUSTFindRowMinMaxBBoxExtent(float* minCUDAData, float* maxCUDAData, int* rowNumbersCUDAData, int numValues, int outTexSize, float* minMaxRowMinCUDAData, float* minMaxRowMaxCUDAData)
{
	float* minValues = new float[numValues];
	float* maxValues = new float[numValues];
	int* rowNumValues = new int[numValues];

	cudaMemcpy(minValues, minCUDAData, numValues*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(maxValues, maxCUDAData, numValues*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(rowNumValues, rowNumbersCUDAData, numValues*sizeof(int), cudaMemcpyDeviceToHost);

//	WriteToFileInt(rowNumValues, numValues, 1);

	THRUSTDeviceMinMaxSort(minCUDAData, maxCUDAData, rowNumbersCUDAData, numValues, minValues, maxValues, rowNumValues);

	WriteToFileInt(rowNumValues, numValues, 1);




	float* minMaxRowMinValues = new float[outTexSize*outTexSize];
	float* minMaxRowMaxValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		minMaxRowMinValues[i] = -1;
		minMaxRowMaxValues[i] = -1;
	}

	// Find Min Max Value
	for (int i = 0; i < numValues; i++)
	{
		int currentRowValue = rowNumValues[i];
		if (currentRowValue >= 0)
		{
			assert(currentRowValue < outTexSize*outTexSize);
			float currentMinValue = minValues[i];
			float currentMaxValue = maxValues[i];
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			//(x2.y < x1.y && x2.x > 0) || x1.x < 0)
			if ((currentMaxValue < minMaxRowMaxValues[currentRowValue] && currentMinValue > 0) || minMaxRowMinValues[currentRowValue] < 0)
			{
				minMaxRowMinValues[currentRowValue] = currentMinValue;
				minMaxRowMaxValues[currentRowValue] = currentMaxValue;
			}
		}
	}

	// Upload data to GPU
	cudaMemcpy(minMaxRowMinCUDAData, minMaxRowMinValues, outTexSize*outTexSize*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(minMaxRowMaxCUDAData, minMaxRowMaxValues, outTexSize*outTexSize*sizeof(float), cudaMemcpyHostToDevice);
	CUDACheckErrors(" Row Min Max Upload ");

	//WriteToFile(minMaxRowMinValues, outTexSize, outTexSize);
	//WriteToFile(minMaxRowMaxValues, outTexSize, outTexSize);

	// Reduce to find Min Min value for each row
	float* minMinRowMinValues = new float[outTexSize*outTexSize];
	float* minMinRowMaxValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		minMinRowMinValues[i] = -1;
		minMinRowMaxValues[i] = -1;
	}
	for (int i = 0; i < numValues; i++)
	{
		int currentRowValue = rowNumValues[i];
		float currentMinValue = minValues[i];
		float currentMaxValue = maxValues[i];
		if (currentRowValue >= 0)
		{
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			// ((x2.x < x1.x && x2.x > 0) || (x2.x == x1.x && x2.y > x1.y) || x1.x < 0)
			if ((currentMinValue < minMinRowMinValues[currentRowValue] && currentMinValue > 0) || (currentMinValue == minMinRowMinValues[currentRowValue] && currentMaxValue > minMinRowMaxValues[currentRowValue]) || minMinRowMinValues[currentRowValue] < 0)
			{
				minMinRowMinValues[currentRowValue] = currentMinValue;
				minMinRowMaxValues[currentRowValue] = currentMaxValue;
			}
		}
	}

	//WriteToFile(minMinRowMinValues, 1, outTexSize*outTexSize);

	// Reduce to find Max Value of MinMin distances
	Float4 maxMinMinValue = Float4(minMinRowMinValues[0], minMinRowMaxValues[0], 0, -1);
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		if (minMinRowMinValues[i] > maxMinMinValue[0])
		{
			maxMinMinValue[0] = minMinRowMinValues[i];
			maxMinMinValue[1] = minMinRowMaxValues[i];
			maxMinMinValue[2] = i;
		}
	}

	delete[] minMaxRowMinValues;
	delete[] minMaxRowMaxValues;

	delete[] minMinRowMinValues;
	delete[] minMinRowMaxValues;

	delete[] minValues;
	delete[] maxValues;
	delete[] rowNumValues;

	return maxMinMinValue;
}
#endif

float CUDAHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, bool readBack, bool renderTextureOutput)
{
	bool operationFailed = false;
	bool timing = false;
	int startingLevel = 2;
	Timer timer1, timer2;
	float distTime = 0, findTime1 = 0, findTime2 = 0, finalCalcTime = 0, totalTime = 0;
	if (timing)
		timer1.Start();

	int initialReference = 0;
	int initialRow = 0;
	int referenceListSize = 1;
	int numOverlaps = 1;

	int* referenceListAxData = new int[1];
	int* referenceListAyData = new int[1];
	int* referenceListBxData = new int[1];
	int* referenceListByData = new int[1];
	int* rowNumbersListData  = new int[1];

	referenceListAxData[0]  = initialReference;
	referenceListAyData[0]  = initialReference;
	referenceListBxData[0]  = initialReference;
	referenceListByData[0]  = initialReference;
	rowNumbersListData[0]	= initialRow;

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;
	int maxLevel = GetExponent2(uNum);

	assert(maxLevel < 10);
	
	float hausdorffMinVal = 0;
	for (int level = 1; level <= maxLevel && !operationFailed; level++)
	{
		if (timing)
			timer2.Start();

		int levelOriginY = int((pow(2.0,maxLevel-level)-1)*pow(2.0,level+1));
		int distanceArraySize = referenceListSize*4;

		if (distanceArraySize < 8100 && distanceArraySize > 0)
		{
			// Create a texture that is a power of two texture
			float* minDistanceCUDAData;
			float* maxDistanceCUDAData;
			int* rowNumberOutCUDAData;
			cudaMalloc((void**)&(minDistanceCUDAData), distanceArraySize*distanceArraySize*sizeof(float));
			cudaMalloc((void**)&(maxDistanceCUDAData), distanceArraySize*distanceArraySize*sizeof(float));
			cudaMalloc((void**)&(rowNumberOutCUDAData), distanceArraySize*distanceArraySize*sizeof(int));

			int* referenceListAxCUDAData;
			int* referenceListAyCUDAData;
			int* referenceListBxCUDAData;
			int* referenceListByCUDAData;
			int* rowNumbersCUDAData;

			cudaMalloc((void**)&(referenceListAxCUDAData), referenceListSize*referenceListSize*sizeof(int));
			cudaMalloc((void**)&(referenceListAyCUDAData), referenceListSize*referenceListSize*sizeof(int));
			cudaMalloc((void**)&(referenceListBxCUDAData), referenceListSize*referenceListSize*sizeof(int));
			cudaMalloc((void**)&(referenceListByCUDAData), referenceListSize*referenceListSize*sizeof(int));
			cudaMalloc((void**)&(rowNumbersCUDAData),	   referenceListSize*referenceListSize*sizeof(int));

			// Transfer Reference List Data
			cudaMemcpy(referenceListAxCUDAData, referenceListAxData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
			cudaMemcpy(referenceListAyCUDAData, referenceListAyData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
			cudaMemcpy(referenceListBxCUDAData, referenceListBxData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
			cudaMemcpy(referenceListByCUDAData, referenceListByData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
			cudaMemcpy(rowNumbersCUDAData,		rowNumbersListData,	 referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
			CUDACheckErrors(" Data Upload ");

			float3* bBoxMinA = surface1->evalParams->bBoxMinCUDAData;
			float3* bBoxMaxA = surface1->evalParams->bBoxMaxCUDAData;
			float3* bBoxMinB = surface2->evalParams->bBoxMinCUDAData;
			float3* bBoxMaxB = surface2->evalParams->bBoxMaxCUDAData;
			
			CUDABBoxHausdorffDistance(bBoxMinA, bBoxMaxA, bBoxMinB, bBoxMaxB, referenceListAxCUDAData, referenceListAyCUDAData, referenceListBxCUDAData, referenceListByCUDAData, rowNumbersCUDAData, distanceArraySize, levelOriginY, uNum, minDistanceCUDAData, maxDistanceCUDAData, rowNumberOutCUDAData);

			CUDACheckErrors(" BBox Distance ");


#ifdef DUMP
			float* tempData = new float[distanceArraySize*distanceArraySize];
			cudaMemcpy(tempData, minDistanceCUDAData, distanceArraySize*distanceArraySize*sizeof(float), cudaMemcpyDeviceToHost);
			WriteToFile(tempData, distanceArraySize, distanceArraySize);

			//cudaMemcpy(tempData, maxDistanceCUDAData, distanceArraySize*distanceArraySize*sizeof(float), cudaMemcpyDeviceToHost);
			//WriteToFile(tempData, distanceArraySize, distanceArraySize);

			int* tempDataInt = new int[distanceArraySize*distanceArraySize];
			cudaMemcpy(tempDataInt, rowNumberOutCUDAData, distanceArraySize*distanceArraySize*sizeof(int), cudaMemcpyDeviceToHost);
			WriteToFileInt(tempDataInt, distanceArraySize, distanceArraySize);

			delete [] tempData;
			delete [] tempDataInt;
#endif

			cudaFree(referenceListAxCUDAData);
			cudaFree(referenceListAyCUDAData);
			cudaFree(referenceListBxCUDAData);
			cudaFree(referenceListByCUDAData);
			cudaFree(rowNumbersCUDAData);

			if (timing)
			{
				timer2.Stop();
				distTime += timer2.GetElapsedTimeInMicroSec();
				timer2.Start();
			}

			// Find the minimum max extent of each row and minimum min extent of each row
			int minMinTextureSize = int(pow(2.0, level));
			float* minMaxRowMaxCUDAData;
			cudaMalloc((void**)&(minMaxRowMaxCUDAData), minMinTextureSize*minMinTextureSize*sizeof(float));

#ifndef THRUST
			Float4 hausdorffVal = CPUFindRowMinMaxBBoxExtent(minDistanceCUDAData, maxDistanceCUDAData, rowNumberOutCUDAData, numOverlaps*16, minMinTextureSize, minMaxRowMaxCUDAData);
#else
			Float4 hausdorffVal = CPUFindRowMinMaxBBoxExtent(minDistanceCUDAData, maxDistanceCUDAData, rowNumberOutCUDAData, numOverlaps*16, minMinTextureSize, minMaxRowMaxCUDAData);
			//Float4 hausdorffVal = THRUSTFindRowMinMaxBBoxExtent(minDistanceCUDAData, maxDistanceCUDAData, rowNumberOutCUDAData, numOverlaps*16, minMinTextureSize, minMaxRowMaxCUDAData);
#endif
			CUDACheckErrors(" Row Min Max ");

			hausdorffMinVal = hausdorffVal[0];

			if  (timing)
			{
				timer2.Stop();
				findTime1 += timer2.GetElapsedTimeInMicroSec();
				timer2.Start();
			}

			//Apply the filter
			// Apply the first filter
			int* filter1CUDAData;
			cudaMalloc((void**)&(filter1CUDAData), distanceArraySize*distanceArraySize*sizeof(int));
			CUDACheckErrors(" Filter 1 Allocation ");

			CUDAHausdoffFilter1(minDistanceCUDAData, maxDistanceCUDAData, hausdorffMinVal, distanceArraySize, filter1CUDAData);
			CUDACheckErrors(" Filter 1 ");

#ifdef DUMP
			int* tempData = new int[distanceArraySize*distanceArraySize];
			cudaMemcpy(tempData, filter1CUDAData, distanceArraySize*distanceArraySize*sizeof(int), cudaMemcpyDeviceToHost);
			WriteToFileInt(tempData, distanceArraySize, distanceArraySize);
#endif
		
			// Reduce the output texture to cull all values of rows that get culled
			int* filter1RowCUDAData;
			cudaMalloc((void**)&(filter1RowCUDAData), minMinTextureSize*minMinTextureSize*sizeof(int));

			CPURowFilterTexture(filter1CUDAData, rowNumberOutCUDAData, filter1RowCUDAData, distanceArraySize, minMinTextureSize);
			CUDACheckErrors(" Filter 1 Row ");

			int* filter2CUDAData;
			cudaMalloc((void**)&(filter2CUDAData), distanceArraySize*distanceArraySize*sizeof(int));

			CUDAHausdoffFilter2(minDistanceCUDAData, rowNumberOutCUDAData, minMaxRowMaxCUDAData, filter1RowCUDAData, distanceArraySize, filter2CUDAData);
			CUDACheckErrors(" Filter 2 ");

#ifdef DUMP
			tempData = new int[distanceArraySize*distanceArraySize];
			cudaMemcpy(tempData, filter2CUDAData, distanceArraySize*distanceArraySize*sizeof(int), cudaMemcpyDeviceToHost);
			WriteToFileInt(tempData, distanceArraySize, distanceArraySize);
#endif

			vector<int> positions;
			CPUFindRenderLocation(filter2CUDAData, &numOverlaps, distanceArraySize, distanceArraySize, &positions);

			//assert(numOverlaps > 0);

			cudaFree(minMaxRowMaxCUDAData);
			cudaFree(filter1CUDAData);
			cudaFree(filter1RowCUDAData);
			cudaFree(filter2CUDAData);
			CUDACheckErrors(" Delete Data ");

			if (timing)
			{
				timer2.Stop();
				findTime2 += timer2.GetElapsedTimeInMicroSec();
			}

			if (timing)
				cout <<"Level : " << level << " Pairs : " << numOverlaps << endl;
			
			int newReferenceListSize = ceil(sqrt(numOverlaps*1.0));
			int* newReferenceListAxData = new int[newReferenceListSize*newReferenceListSize];
			int* newReferenceListAyData = new int[newReferenceListSize*newReferenceListSize];
			int* newReferenceListBxData = new int[newReferenceListSize*newReferenceListSize];
			int* newReferenceListByData = new int[newReferenceListSize*newReferenceListSize];
			int* newRowNumbersListData	= new int[newReferenceListSize*newReferenceListSize];
			for (int j = 0; j < newReferenceListSize; j++)
			{
				for (int i = 0; i < newReferenceListSize; i++)
				{
					int overlap = j*newReferenceListSize + i;
					if (overlap < numOverlaps)
					{
						int pos = positions[overlap];
						int xPos = pos%distanceArraySize;
						int yPos = int(pos/distanceArraySize);
						int referenceListLocationX = int(xPos/4.0);
						int referenceListLocationY = int(yPos/4.0);
						int boxLocationAx = ((xPos%4)%2		+ referenceListAxData[referenceListLocationY*referenceListSize + referenceListLocationX])*2;
						int boxLocationAy = (int((xPos%4)/2)+ referenceListAyData[referenceListLocationY*referenceListSize + referenceListLocationX])*2;
						int boxLocationBx = ((yPos%4)%2		+ referenceListBxData[referenceListLocationY*referenceListSize + referenceListLocationX])*2;
						int boxLocationBy = (int((yPos%4)/2)+ referenceListByData[referenceListLocationY*referenceListSize + referenceListLocationX])*2;
						int newRowNumber =					4*rowNumbersListData[referenceListLocationY*referenceListSize + referenceListLocationX] + (yPos%4);
						newReferenceListAxData[overlap] = boxLocationAx;
						newReferenceListAyData[overlap] = boxLocationAy;
						newReferenceListBxData[overlap] = boxLocationBx;
						newReferenceListByData[overlap] = boxLocationBy;
						newRowNumbersListData[overlap]	= newRowNumber;
					}
					else
					{
						newReferenceListAxData[overlap] = -1;
						newReferenceListAyData[overlap] = -1;
						newReferenceListBxData[overlap] = -1;
						newReferenceListByData[overlap] = -1;
						newRowNumbersListData[overlap]	= -1;
					}
				}
			}

			delete[] referenceListAxData;
			delete[] referenceListAyData;
			delete[] referenceListBxData;
			delete[] referenceListByData;
			delete[] rowNumbersListData;

			referenceListSize = newReferenceListSize;
			referenceListAxData  = newReferenceListAxData;
			referenceListAyData  = newReferenceListAyData;
			referenceListBxData  = newReferenceListBxData;
			referenceListByData  = newReferenceListByData;
			rowNumbersListData	 = newRowNumbersListData;

			cudaFree(minDistanceCUDAData);
			cudaFree(maxDistanceCUDAData);
			cudaFree(rowNumberOutCUDAData);
			CUDACheckErrors(" Delete Data ");

		}// For if >4096
		else
		{
			operationFailed = true;
			cout << "HD Failed " << endl; 
		}

	}
	// For each level loop ends

	float hausdorffDistance = -1;
	if (renderTextureOutput)
	{
		surface1->numIntersectingBoxes = numOverlaps;
		surface2->numIntersectingBoxes = numOverlaps;
		surface1->intersectingBoxes = new float[numOverlaps*4];
		surface2->intersectingBoxes = new float[numOverlaps*4];
		float uInterval1 = (surface1->evalParams->uMax - surface1->evalParams->uMin)/(uNum*1.0);
		float vInterval1 = (surface1->evalParams->vMax - surface1->evalParams->vMin)/(vNum*1.0);
		float uInterval2 = (surface2->evalParams->uMax - surface2->evalParams->uMin)/(uNum*1.0);
		float vInterval2 = (surface2->evalParams->vMax - surface2->evalParams->vMin)/(vNum*1.0);
		for (int k = 0; k < numOverlaps ; k++)
		{
			int uIndex1 = (int) referenceListAxData[k]/2.0;
			int vIndex1 = (int) referenceListAyData[k]/2.0;
			int uIndex2 = (int) referenceListBxData[k]/2.0;
			int vIndex2 = (int) referenceListByData[k]/2.0;

			surface1->intersectingBoxes[k*4+0] = (uIndex1+0)*uInterval1;
			surface1->intersectingBoxes[k*4+1] = (uIndex1+1)*uInterval1;
			surface1->intersectingBoxes[k*4+2] = (vIndex1+0)*vInterval1;
			surface1->intersectingBoxes[k*4+3] = (vIndex1+1)*vInterval1;

			surface2->intersectingBoxes[k*4+0] = (uIndex2+0)*uInterval2;
			surface2->intersectingBoxes[k*4+1] = (uIndex2+1)*uInterval2;
			surface2->intersectingBoxes[k*4+2] = (vIndex2+0)*vInterval2;
			surface2->intersectingBoxes[k*4+3] = (vIndex2+1)*vInterval2;
		}
	}
	if (!operationFailed && referenceListSize > 0)
	{
		Float3 hausdorffSurf1Point, hausdorffSurf2Point;
		if (timing)
			timer2.Start();

		int* referenceListAxCUDAData;
		int* referenceListAyCUDAData;
		int* referenceListBxCUDAData;
		int* referenceListByCUDAData;
		int* rowNumbersCUDAData;

		cudaMalloc((void**)&(referenceListAxCUDAData), referenceListSize*referenceListSize*sizeof(int));
		cudaMalloc((void**)&(referenceListAyCUDAData), referenceListSize*referenceListSize*sizeof(int));
		cudaMalloc((void**)&(referenceListBxCUDAData), referenceListSize*referenceListSize*sizeof(int));
		cudaMalloc((void**)&(referenceListByCUDAData), referenceListSize*referenceListSize*sizeof(int));
		cudaMalloc((void**)&(rowNumbersCUDAData),	   referenceListSize*referenceListSize*sizeof(int));

		// Transfer Reference List Data
		cudaMemcpy(referenceListAxCUDAData, referenceListAxData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(referenceListAyCUDAData, referenceListAyData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(referenceListBxCUDAData, referenceListBxData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(referenceListByCUDAData, referenceListByData, referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
		cudaMemcpy(rowNumbersCUDAData,		rowNumbersListData,	 referenceListSize*referenceListSize*sizeof(int), cudaMemcpyHostToDevice);
		CUDACheckErrors(" Data Upload ");

		float4* surface1CUDAData;
		float4* surface2CUDAData;
		if (surface1->identityTransformation)
			surface1CUDAData = surface1->evalParams->evaluatedCUDAPoints;
		else
			surface1CUDAData = surface1->evalParams->transformedCUDAPoints;

		if (surface2->identityTransformation)
			surface2CUDAData = surface2->evalParams->evaluatedCUDAPoints;
		else
			surface2CUDAData = surface2->evalParams->transformedCUDAPoints;

		float* hausdorffDistanceCUDAData;
		float3* surface1HDPointCUDAData;
		float3* surface2HDPointCUDAData;
		cudaMalloc((void**)&(hausdorffDistanceCUDAData), referenceListSize*referenceListSize*sizeof(float));
		cudaMalloc((void**)&(surface1HDPointCUDAData), referenceListSize*referenceListSize*sizeof(float3));
		cudaMalloc((void**)&(surface2HDPointCUDAData), referenceListSize*referenceListSize*sizeof(float3));
		CUDACheckErrors(" Data Allocation ");

		CUDATriTriHausdorffDistance(surface1CUDAData, surface2CUDAData, referenceListAxCUDAData, referenceListAyCUDAData, referenceListBxCUDAData, referenceListByCUDAData, rowNumbersCUDAData, referenceListSize, numOverlaps, uNum+1, hausdorffDistanceCUDAData, surface1HDPointCUDAData, surface2HDPointCUDAData);
		CUDACheckErrors(" Tri Tri Hausdorff Distance ");

		cudaFree(referenceListAxCUDAData);
		cudaFree(referenceListAyCUDAData);
		cudaFree(referenceListBxCUDAData);
		cudaFree(referenceListByCUDAData);
		cudaFree(rowNumbersCUDAData);

		float* hausdorffDistanceData = new float[referenceListSize*referenceListSize];
		float* surface1HDPointData = new float[referenceListSize*referenceListSize*3];
		float* surface2HDPointData = new float[referenceListSize*referenceListSize*3];

		cudaMemcpy(surface1HDPointData, surface1HDPointCUDAData, referenceListSize*referenceListSize*sizeof(float3), cudaMemcpyDeviceToHost);
		cudaMemcpy(surface2HDPointData, surface2HDPointCUDAData, referenceListSize*referenceListSize*sizeof(float3), cudaMemcpyDeviceToHost);
		cudaMemcpy(hausdorffDistanceData, hausdorffDistanceCUDAData, referenceListSize*referenceListSize*sizeof(float), cudaMemcpyDeviceToHost);

		cudaFree(surface1HDPointCUDAData);
		cudaFree(surface2HDPointCUDAData);
		cudaFree(hausdorffDistanceCUDAData);

		float* rowMinDistances = new float[uNum*vNum];
		Float3* surf1Points = new Float3[uNum*vNum];
		Float3* surf2Points = new Float3[uNum*vNum];
		for (int k = 0; k < uNum*vNum; k++)
			rowMinDistances[k] = -1;
		for (int k = 0; k < numOverlaps ; k++)
		{
			float tempHausdorffDist = hausdorffDistanceData[k];
			int uIndex1 = (int) referenceListAxData[k]/2.0;
			int vIndex1 = (int) referenceListAyData[k]/2.0;
			int uIndex2 = (int) referenceListBxData[k]/2.0;
			int vIndex2 = (int) referenceListByData[k]/2.0;
			int indexVal2 = vIndex2*uNum + uIndex2;
			if (indexVal2 > 0) // Potential Bug
			{
				if (rowMinDistances[indexVal2] == -1 || tempHausdorffDist < rowMinDistances[indexVal2])
				{
					rowMinDistances[indexVal2] = tempHausdorffDist;
					surf1Points[indexVal2] = Float3(surface1HDPointData[k*3+0], surface1HDPointData[k*3+1], surface1HDPointData[k*3+2]);
					surf2Points[indexVal2] = Float3(surface2HDPointData[k*3+0], surface2HDPointData[k*3+1], surface2HDPointData[k*3+2]);
				}
			}
		}
		
		delete[] hausdorffDistanceData;
		delete[] surface1HDPointData;
		delete[] surface2HDPointData;

		hausdorffDistance = -1;
		for (int k = 0; k < uNum*vNum; k++)
		{
			if (rowMinDistances[k]> hausdorffDistance)
			{
				hausdorffDistance	= rowMinDistances[k];
				hausdorffSurf1Point	= surf1Points[k];
				hausdorffSurf2Point	= surf2Points[k];
			}
		}

		if (hausdorffDistance >= 0)
		{
			point1[0] = hausdorffSurf1Point[0];
			point1[1] = hausdorffSurf1Point[1];
			point1[2] = hausdorffSurf1Point[2];
			point2[0] = hausdorffSurf2Point[0];
			point2[1] = hausdorffSurf2Point[1];
			point2[2] = hausdorffSurf2Point[2];
		}
		delete [] rowMinDistances;
		delete [] surf1Points;
		delete [] surf2Points;
	}

	delete[] referenceListAxData;
	delete[] referenceListAyData;
	delete[] referenceListBxData;
	delete[] referenceListByData;
	delete[] rowNumbersListData;

	if (timing)
	{
		timer1.Stop();
		timer2.Stop();
		finalCalcTime = timer2.GetElapsedTimeInMicroSec();
		totalTime = timer1.GetElapsedTimeInMicroSec();
		cout << endl;
		cout << "Distance Time        : " << int(distTime)*0.001 << endl;
		cout << "Range Time           : " << int(findTime1)*0.001 << endl;
		cout << "Filter Time          : " << int(findTime2)*0.001 << endl;
		cout << "Trianguled Time      : " << int(finalCalcTime)*0.001 << endl << endl;
		cout << "Traversal Total Time : " << int(totalTime)*0.001 << endl << endl;
	}

	if (!operationFailed)
		return hausdorffDistance;
	else
		return -1;

}

float GetCUDASurfaceHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, int intRes, GLParameters* glParam, bool forcedEvaluate)
{
	glParam->hausdorff = true;
	int uNum = intRes;
	int vNum = intRes;

	bool timing = false;
	Timer timer1, timer2;
	float initTime = 0, evaluationTime = 0, bBoxTime = 0, hausdorffTime = 0;
	if (timing)
	{
		timer1.Start();
		timer2.Start();
	}

	bool evaluate = true;
	if(surface1->initializedEvalParams && surface2->initializedEvalParams && !forcedEvaluate)
	{
		if (surface1->evalParams->uMin == 0.0  &&
			surface1->evalParams->uMax == 1.0  &&
			surface1->evalParams->vMin == 0.0  &&
			surface1->evalParams->vMax == 1.0  &&
			surface1->evalParams->uNum == uNum &&
			surface1->evalParams->vNum == vNum &&
			surface2->evalParams->uMin == 0.0  &&
			surface2->evalParams->uMax == 1.0  &&
			surface2->evalParams->vMin == 0.0  &&
			surface2->evalParams->vMax == 1.0  &&
			surface2->evalParams->uNum == uNum &&
			surface2->evalParams->vNum == vNum)
				evaluate = false;
	}

	if (!evaluate && forcedEvaluate)
		evaluate = true;

	if (evaluate)
	{
		if (surface1->initializedEvalParams)
			delete surface1->evalParams;
		surface1->evalParams = new EvaluationParams();
		surface1->initializedEvalParams = true;

		if (surface2->initializedEvalParams)
			delete surface2->evalParams;
		surface2->evalParams = new EvaluationParams();
		surface2->initializedEvalParams = true;

		surface1->evalParams->uNum	= uNum;
		surface1->evalParams->vNum	= vNum;
		surface2->evalParams->uNum	= uNum;
		surface2->evalParams->vNum	= vNum;

		surface1->evalParams->uMin = 0.0;
		surface1->evalParams->uMax = 1.0;
		surface1->evalParams->vMin = 0.0;
		surface1->evalParams->vMax = 1.0;

		surface2->evalParams->uMin = 0.0;
		surface2->evalParams->uMax = 1.0;
		surface2->evalParams->vMin = 0.0;
		surface2->evalParams->vMax = 1.0;
	}
	else
	{
		if (!surface1->identityTransformation)
		{
			cudaFree(surface1->evalParams->secondDervCUDAData[0]);
			cudaFree(surface1->evalParams->secondDervCUDAData[1]);
			cudaFree(surface1->evalParams->secondDervCUDAData[2]);
			cudaFree(surface1->evalParams->secondDervCUDAData[3]);

			cudaFree(surface1->evalParams->bBoxMinCUDAData);
			cudaFree(surface1->evalParams->bBoxMaxCUDAData);
			cudaFree(surface1->evalParams->evaluatedCUDANormals);
			cudaFree(surface1->evalParams->transformedCUDAPoints);
		}

		if (!surface2->identityTransformation)
		{
			cudaFree(surface2->evalParams->secondDervCUDAData[0]);
			cudaFree(surface2->evalParams->secondDervCUDAData[1]);
			cudaFree(surface2->evalParams->secondDervCUDAData[2]);
			cudaFree(surface2->evalParams->secondDervCUDAData[3]);

			cudaFree(surface2->evalParams->bBoxMinCUDAData);
			cudaFree(surface2->evalParams->bBoxMaxCUDAData);
			cudaFree(surface2->evalParams->evaluatedCUDANormals);
			cudaFree(surface2->evalParams->transformedCUDAPoints);
		}

		CUDACheckErrors(" Reuse Data ");
	}

	if (timing)
	{
		timer2.Stop();
		initTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}
	if (evaluate)
	{
//		if (glParam->readBack)
//		{
//			surface1->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
//			surface2->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
//		}
		bool readBack = false;
		bool secondDerivatives = glParam->exactNormals;
		surface1->CUDASurfaceEvaluation(glParam->exactNormals, secondDerivatives, readBack, false);
		surface2->CUDASurfaceEvaluation(glParam->exactNormals, secondDerivatives, readBack, false);
	}
	if (timing)
	{
		timer2.Stop();
		evaluationTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}
	if (evaluate || !surface1->identityTransformation)
	{
		surface1->CUDASurfaceTransformation(glParam->readBack);
		surface1->CUDASurfaceBoundingBoxEvaluation(true, glParam->expandBoundingBox, glParam->readBack);
	}
	if (evaluate || !surface2->identityTransformation)
	{
		surface2->CUDASurfaceTransformation(glParam->readBack);
		surface2->CUDASurfaceBoundingBoxEvaluation(true, glParam->expandBoundingBox, glParam->readBack);
	}
	if (timing)
	{
		timer2.Stop();
		bBoxTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}

	bool renderTextureOutput = false;
	float hausdorffDist = CUDAHausdorffDistance(surface1, surface2, point1, point2, glParam->readBack, renderTextureOutput);

	if (renderTextureOutput)
	{
		surface1->GenerateIntersectionOverlayTexture(surface1->intersectingBoxes, surface1->numIntersectingBoxes, uNum, glParam);
		surface2->GenerateIntersectionOverlayTexture(surface2->intersectingBoxes, surface2->numIntersectingBoxes, uNum, glParam);
	}
	glParam->hausdorff = false;

	if (timing)
	{
		timer2.Stop();
		hausdorffTime = timer2.GetElapsedTimeInMicroSec();
		timer1.Stop();

		cout<<"Init Time         : "<<(0.001*initTime)<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationTime)<<endl;
		cout<<"BBox Time         : "<<(0.001*bBoxTime)<<endl;
		cout<<"Traversal Time    : "<<(0.001*hausdorffTime)<<endl<<endl;
		cout<<"Hausdorff Time    : "<<(0.001*timer1.GetElapsedTimeInMicroSec())<<endl<<endl;
	}
	return hausdorffDist;
}
#endif


/*	if (!operationFailed)
	{
		if (timing)
			timer2.Start();
	
		Float3 hausdorffSurf1Point, hausdorffSurf2Point;
		float* rowMinDistances = new float[uNum*vNum];
		Float3* surf1Points = new Float3[uNum*vNum];
		Float3* surf2Points = new Float3[uNum*vNum];
		for (int k = 0; k < uNum*vNum; k++)
			rowMinDistances[k] = -1;

		for (int k = 0; k < numOverlaps; k++)
		{
			Float3 surf1Point;
			Float3 surf2Point;

			int uIndex1 = (int)(referenceListAxData[k]/2.0);
			int vIndex1 = (int)(referenceListAyData[k]/2.0);
			int uIndex2 = (int)(referenceListBxData[k]/2.0);
			int vIndex2 = (int)(referenceListByData[k]/2.0);

			int indexVal1 = vIndex1*uNum + uIndex1;
			int indexVal2 = vIndex2*uNum + uIndex2;

			float tempHausdorffDist = -1;

			tempHausdorffDist = HausdorffData(surface1, surface2, uIndex1, vIndex1, uIndex2, vIndex2, &surf1Point, &surf2Point);

			if (rowMinDistances[indexVal2] == -1 || tempHausdorffDist < rowMinDistances[indexVal2])
			{
				rowMinDistances[indexVal2] = tempHausdorffDist;
				surf1Points[indexVal2] = surf1Point;
				surf2Points[indexVal2] = surf2Point;
			}
		}

		hausdorffDistance = -1;
		for (int k = 0; k < uNum*vNum; k++)
		{
			if (rowMinDistances[k]> hausdorffDistance)
			{
				hausdorffDistance	= rowMinDistances[k];
				hausdorffSurf1Point	= surf1Points[k];
				hausdorffSurf2Point	= surf2Points[k];
			}
		}

		if (hausdorffDistance >= 0)
		{
			point1[0] = hausdorffSurf1Point[0];
			point1[1] = hausdorffSurf1Point[1];
			point1[2] = hausdorffSurf1Point[2];
			point2[0] = hausdorffSurf2Point[0];
			point2[1] = hausdorffSurf2Point[1];
			point2[2] = hausdorffSurf2Point[2];
		}
		delete [] rowMinDistances;
		delete [] surf1Points;
		delete [] surf2Points;
	}

Float4 CPUFindRowMinMaxBBoxExtent(float* minCUDAData, float* maxCUDAData, int* rowNumbersCUDAData, int numValues, int outTexSize, float* minMaxRowMinCUDAData, float* minMaxRowMaxCUDAData)
{
	int texHeight = texWidth;

	// Read back to CPU
	float* minValues = new float[texWidth*texHeight];
	float* maxValues = new float[texWidth*texHeight];
	int* rowNumValues = new int[texWidth*texHeight];

	cudaMemcpy(minValues, minCUDAData, texWidth*texHeight*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(maxValues, maxCUDAData, texWidth*texHeight*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(rowNumValues, rowNumbersCUDAData, texWidth*texHeight*sizeof(int), cudaMemcpyDeviceToHost);

	float* minMaxRowMinValues = new float[outTexSize*outTexSize];
	float* minMaxRowMaxValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		minMaxRowMinValues[i] = -1;
		minMaxRowMaxValues[i] = -1;
	}

	// Find Min Max Value
	for (int i = 0; i < texWidth*texHeight; i++)
	{
		int currentRowValue = rowNumValues[i];
		if (currentRowValue >= 0)
		{
			assert(currentRowValue < outTexSize*outTexSize);
			float currentMinValue = minValues[i];
			float currentMaxValue = maxValues[i];
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			//(x2.y < x1.y && x2.x > 0) || x1.x < 0)
			if ((currentMaxValue < minMaxRowMaxValues[currentRowValue] && currentMinValue > 0) || minMaxRowMinValues[currentRowValue] < 0)
			{
				minMaxRowMinValues[currentRowValue] = currentMinValue;
				minMaxRowMaxValues[currentRowValue] = currentMaxValue;
			}
		}
	}

	// Upload data to GPU
	cudaMemcpy(minMaxRowMinCUDAData, minMaxRowMinValues, outTexSize*outTexSize*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(minMaxRowMaxCUDAData, minMaxRowMaxValues, outTexSize*outTexSize*sizeof(float), cudaMemcpyHostToDevice);
	CUDACheckErrors(" Row Min Max Upload ");

	//WriteToFile(minMaxRowMinValues, outTexSize, outTexSize);
	//WriteToFile(minMaxRowMaxValues, outTexSize, outTexSize);

	// Reduce to find Min Min value for each row
	float* minMinRowMinValues = new float[outTexSize*outTexSize];
	float* minMinRowMaxValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		minMinRowMinValues[i] = -1;
		minMinRowMaxValues[i] = -1;
	}
	for (int i = 0; i < texWidth*texHeight; i++)
	{
		int currentRowValue = rowNumValues[i];
		float currentMinValue = minValues[i];
		float currentMaxValue = maxValues[i];
		if (currentRowValue >= 0)
		{
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			// ((x2.x < x1.x && x2.x > 0) || (x2.x == x1.x && x2.y > x1.y) || x1.x < 0)
			if ((currentMinValue < minMinRowMinValues[currentRowValue] && currentMinValue > 0) || (currentMinValue == minMinRowMinValues[currentRowValue] && currentMaxValue > minMinRowMaxValues[currentRowValue]) || minMinRowMinValues[currentRowValue] < 0)
			{
				minMinRowMinValues[currentRowValue] = currentMinValue;
				minMinRowMaxValues[currentRowValue] = currentMaxValue;
			}
		}
	}

	//WriteToFile(minMinRowMinValues, 1, outTexSize*outTexSize);

	// Reduce to find Max Value of MinMin distances
	Float4 maxMinMinValue = Float4(minMinRowMinValues[0], minMinRowMaxValues[0], 0, -1);
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		if (minMinRowMinValues[i] > maxMinMinValue[0])
		{
			maxMinMinValue[0] = minMinRowMinValues[i];
			maxMinMinValue[1] = minMinRowMaxValues[i];
			maxMinMinValue[2] = i;
		}
	}

	delete[] minMaxRowMinValues;
	delete[] minMaxRowMaxValues;

	delete[] minMinRowMinValues;
	delete[] minMinRowMaxValues;

	delete[] minValues;
	delete[] maxValues;
	delete[] rowNumValues;

	return maxMinMinValue;
}

*/