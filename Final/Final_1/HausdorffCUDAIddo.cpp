#include "HausdorffCUDAIddo.h"
#include "Hausdorff.h"
#include "GPUUtilities.h"
#include "CUDAUtilities.h"

#ifdef CUDA

//Iddo CUDA Hausdorff Code:
#include "NURBSGeomUtils.h" //for splitting into flat Bezier patches

//Loop over all textures of flat Beziers in surf2 (=inTexs) and perform gpu frag prog.
float IddoCUDAHausdorffDistance(NURBS* surface1, NURBS* surface2, const std::vector<float4*>& inTexs, float4* fboTexs[2], float* convergenceTexs[2], float kBStopTolerance, float* point1, float* point2, float* convergenceStatus, bool readBack, bool renderTextureOutput)
{
	bool timing = false;

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;

	int uOrder2 = surface2->uOrder;
	int vOrder2 = surface2->vOrder;

	// After each iteration fboTexs[k%2] holds the output fbo.
	// In the end of the iterations fboTexs[(numOfSurfaces-1)%2] will hold the final output.
	unsigned numOfSurfaces = inTexs.size();

	for (int k=0; k < numOfSurfaces; ++k) //index of patch we "render"
	{

#ifdef DUMP
		//debug - reading current fbo
		float* tempData=new float[(uNum+1)*(vNum+1)*4];
		//cudaMemcpy(tempData, surface1->evalParams->evaluatedCUDAPoints, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
		cudaMemcpy(tempData, fboTexs[(k+1)%2], (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
		//cudaMemcpy(tempData, inTexs[k], (uOrder2)*(vOrder2)*4*sizeof(float), cudaMemcpyDeviceToHost);
		//WriteToFileAll(tempData, uNum+1, vNum+1, 4);
		delete [] tempData;
#endif
		float4* surface1EvalPoints; 
		if (surface1->identityTransformation)
			surface1EvalPoints = surface1->evalParams->evaluatedCUDAPoints;
		else
			surface1EvalPoints = surface1->evalParams->transformedCUDAPoints;

		float4* prevDistTex = fboTexs[(k+1)%2];
		float4* currentDistTex =  fboTexs[k%2];
		float* prevConvStatus = convergenceTexs[(k+1)%2];
		float* currentConvStatus = convergenceTexs[k%2];
		CUDAHausdorffIddo(surface1EvalPoints, prevDistTex, prevConvStatus, uNum+1, vNum+1, inTexs[k], uOrder2, vOrder2, kBStopTolerance, currentDistTex, currentConvStatus);

	} //for k loop

#ifdef DUMP
	//iddo: debug - getting the final output texture
	float* tempData=new float[(uNum+1)*(vNum+1)*4];
	cudaMemcpy(tempData, fboTexs[(numOfSurfaces-1)%2], (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFileAll(tempData, uNum+1, vNum+1, 4); //print rgba
	delete[] tempData;
#endif
#ifdef DUMP
	// Convergence
	int* tempConvData = new int[(uNum+1)*(vNum+1)];
	cudaMemcpy(tempConvData, convergenceTexs[(numOfSurfaces-1)%2], (uNum+1)*(vNum+1)*sizeof(int), cudaMemcpyDeviceToHost);
	WriteToFileInt(tempConvData, uNum+1, vNum+1);
	delete[] tempConvData;
#endif

	//Get the maximal distance from the results:
	int iMax, jMax;

	Timer timerReduce;
	if (timing)
		timerReduce.Start();

	float maxDist = GetMaxDistanceInTexture(fboTexs[(numOfSurfaces-1)%2], uNum, vNum, iMax, jMax, point2);

	if (timing)
	{
		timerReduce.Stop();
		cout<<"Reduction Time     : "<<(0.001*timerReduce.GetElapsedTimeInMicroSec())<<endl;
	}


	//iddo: in CUDA without readback - surface1->evalParams->evaluatedPoints is not initialized.. currently we copy from device to host
	//TODO: do it on host mempory..
	//float * evaluatedPoints = surface1->evalParams->evaluatedPoints;

	//adarsh: Copying only the maxIndex value from the GPU to the CPU
	unsigned int maxIndex = (jMax*(uNum+1) + iMax);
	assert(maxIndex < (uNum+1)*(vNum+1));

	float4* evaluatedPoints = new float4;
	if (surface1->identityTransformation)
		cudaMemcpy(evaluatedPoints, surface1->evalParams->evaluatedCUDAPoints+maxIndex, sizeof(float4), cudaMemcpyDeviceToHost);
	else
		cudaMemcpy(evaluatedPoints, surface1->evalParams->transformedCUDAPoints+maxIndex, sizeof(float4), cudaMemcpyDeviceToHost);
	
	cudaMemcpy(convergenceStatus,convergenceTexs[(numOfSurfaces-1)%2]+maxIndex, sizeof(float), cudaMemcpyDeviceToHost);

	float wij = evaluatedPoints->w;
	point1[0] = evaluatedPoints->x/wij;
	point1[1] = evaluatedPoints->y/wij;
	point1[2] = evaluatedPoints->z/wij;
	delete evaluatedPoints;

	
	//iddo: @@ for debug rendering - copy output texture to surface2->evalParams->evaluatedPoints
#ifdef DEBUG
	float* tempData = new float[(uNum+1)*(vNum+1)*4];
	if (surface2->evalParams->evaluatedPoints != NULL)
		delete [] (surface2->evalParams->evaluatedPoints);
	surface2->evalParams->evaluatedPoints = new GLfloat[(uNum+1)*(vNum+1)*4];

	cudaMemcpy(tempData, fboTexs[(numOfSurfaces-1)%2], (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	//WriteToFileAll(tempData, uNum+1, vNum+1, 4);

	for (int i=0; i<(uNum+1)*(vNum+1)*4; ++i) 
	{
		surface2->evalParams->evaluatedPoints[i] = tempData[i];
		if (i%4 == 3)
			surface2->evalParams->evaluatedPoints[i] = 1.0; //the w coordinates holds the distance..
	}

	//iddo: in CUDA without readback - surface1->evalParams->evaluatedPoints is not initialized.. we copy the debug data there
	if (surface1->evalParams->evaluatedPoints != NULL)
		delete [] (surface1->evalParams->evaluatedPoints);
	surface1->evalParams->evaluatedPoints = new GLfloat[(uNum+1)*(vNum+1)*4];
	cudaMemcpy(surface1->evalParams->evaluatedPoints, surface1->evalParams->evaluatedCUDAPoints, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	//WriteToFileAll(tempData, uNum+1, vNum+1, 4);
	delete [] tempData;
#endif

	//return the hausdorff distance	
	return maxDist;
}


float IddoGetCUDASurfaceHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, float* convergenceStatus, int intRes, float kb, GLParameters* glParam, bool forcedEvaluate)
{
	glParam->hausdorff = true;
	int uNum = intRes;
	int vNum = intRes;

	bool timing = false;
	Timer timer1, timer2;
	float initTime = 0, evaluationTime = 0, bezPatchTime = 0, hausdorffTime = 0;
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
			surface1->evalParams->vNum == vNum)
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

		surface1->evalParams->uNum	= uNum;
		surface1->evalParams->vNum	= vNum;

		surface1->evalParams->uMin = 0.0;
		surface1->evalParams->uMax = 1.0;
		surface1->evalParams->vMin = 0.0;
		surface1->evalParams->vMax = 1.0;
	}
	else
	{
		/*
		if (!surface1->identityTransformation)
		{
		cudaFree(surface1->evalParams->secondDervCUDAData[0]);
		cudaFree(surface1->evalParams->secondDervCUDAData[1]);
		cudaFree(surface1->evalParams->secondDervCUDAData[2]);
		cudaFree(surface1->evalParams->secondDervCUDAData[3]);

		cudaFree(surface1->evalParams->bBoxMinCUDAData);
		cudaFree(surface1->evalParams->bBoxMaxCUDAData);
		cudaFree(surface1->evalParams->evaluatedCUDANormals);
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
		}

		CUDACheckErrors(" Reuse Data ");
		*/
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
		surface1->CUDASurfaceEvaluation(false, false, readBack, false); //need just the points not exact normals and second derivs
	}
	if (timing)
	{
		timer2.Stop();
		evaluationTime = timer2.GetElapsedTimeInMicroSec(); 
		timer2.Start();
	}  

	// Construct the input textures (flat Bezier patches) - iddo: possibly store them in surface2 to save recomputation?
	double flatnessTol = -40.0; //negative flatnessTol means splitting only into Bezier (no check of flatness criterion)
	if (surface2->m_bezCUDAPatches.empty()) 
	{
		ConstructBezDataTex(surface2, flatnessTol);
	}

	if (timing)
	{
		timer2.Stop();
		bezPatchTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}

	// Handling Surface1 transformation
	if (!surface1->identityTransformation)
	{
		bool readBack = false;
		if (surface1->evalParams->transformedCUDAPoints != NULL)
			cudaFree(surface1->evalParams->transformedCUDAPoints);
		surface1->CUDASurfaceTransformation(readBack);
	}

	// Handling surface2 transformation
	if (!surface2->identityTransformation)
	{
		//transform bezPatches.
		//iddo: is there a flag that says when a transform has changed? this can save recomputation time (although it seems this does not take a lot of time).
		if (surface2->m_bezCUDAPatchesTransformed.empty())
		{
			surface2->m_bezCUDAPatchesTransformed.resize(surface2->m_bezCUDAPatches.size());
			for (int i=0; i<surface2->m_bezCUDAPatches.size(); ++i)
				cudaMalloc((void**)&(surface2->m_bezCUDAPatchesTransformed[i]), surface2->uOrder*surface2->vOrder*sizeof(float4));
		}
		for (int i=0; i<surface2->m_bezCUDAPatches.size(); ++i)
		{
			CUDATransformEvaluatedPoints(surface2->m_bezCUDAPatches[i], surface2->uOrder, surface2->vOrder, surface2->transformationMatrix, surface2->m_bezCUDAPatchesTransformed[i]);
			CUDACheckErrors(" Transformation Bez Patches");
		}
	}

	float4* fboTexs[2]; // Initialize textures fbo will copy to (in pingpong method)
	float* convergenceTexs[2]; // Textures to store convergence results
	InitializeFBOTextures(fboTexs, convergenceTexs, uNum, vNum);

	bool renderTextureOutput = false;

	//iddo test - bbox values for defining kb stop criterion
	//BBox needed for kb value, assuming bbox of surface has already been computed
	Float3 bMin2 = surface2->bBoxMin;
	Float3 bMax2 = surface2->bBoxMax;
	float bDiag = sqrt((bMax2[0]-bMin2[0])*(bMax2[0]-bMin2[0])+ 
		(bMax2[1]-bMin2[1])*(bMax2[1]-bMin2[1])+
		(bMax2[2]-bMin2[2])*(bMax2[2]-bMin2[2]));

	float hausdorffDist;
	if (surface2->identityTransformation)
	{
		//surface2 not transformed
		hausdorffDist = IddoCUDAHausdorffDistance(surface1, surface2, surface2->m_bezCUDAPatches, fboTexs, convergenceTexs, kb, point1, point2, convergenceStatus, glParam->readBack, renderTextureOutput);
	}
	else
	{
		hausdorffDist = IddoCUDAHausdorffDistance(surface1, surface2, surface2->m_bezCUDAPatchesTransformed, fboTexs, convergenceTexs, kb, point1, point2, convergenceStatus, glParam->readBack, renderTextureOutput);
	}

	glParam->hausdorff = false;

	if (timing)
	{
		timer2.Stop();
		hausdorffTime = timer2.GetElapsedTimeInMicroSec();
		timer1.Stop();

		cout<<"Init Time                 : "<<(0.001*initTime)<<endl;
		cout<<"Evaluation Time           : "<<(0.001*evaluationTime)<<endl;
		cout<<"Bezier Patch Time         : "<<(0.001*bezPatchTime)<<endl;
		cout<<"HD Time (incl. reduction) : "<<(0.001*hausdorffTime)<<endl<<endl;
		cout<<"Total Hausdorff Time      : "<<(0.001*timer1.GetElapsedTimeInMicroSec())<<endl<<endl;
	}


	//freeing CUDA memory allocated by cudaMalloc
	cudaFree(fboTexs[0]);
	cudaFree(fboTexs[1]);
	cudaFree(convergenceTexs[0]);
	cudaFree(convergenceTexs[1]);

	return hausdorffDist;
}


// Getting max distance in the texture.
// iddo: current implementation just goes over the texture, in the future use reduction.
#ifndef THRUST
float GetMaxDistanceInTexture(float4* cudaRes, int uNum, int vNum, int& iMax, int& jMax, float point2[3])
{
	//reading output texture
	float* surf2Pts=new float[(uNum+1)*(vNum+1)*4];
	cudaMemcpy(surf2Pts, cudaRes, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);

	float maxCurr = 0.0;
	int iMaxCurr = 0;
	int jMaxCurr = 0;

	for(int j=0; j < vNum+1; j++)
	{
		for(int i=0; i < uNum+1; i++)
		{
			if (surf2Pts[4*(j*(uNum+1)+i)+3] > maxCurr)
			{
				maxCurr = surf2Pts[4*(j*(uNum+1)+i)+3];
				iMaxCurr = i;
				jMaxCurr = j;
			}
		}
	}

	iMax = iMaxCurr;
	jMax = jMaxCurr;
	int maxIndex = 4*(jMax*(uNum+1)+iMax);
	point2[0] = surf2Pts[maxIndex+0];
	point2[1] = surf2Pts[maxIndex+1];
	point2[2] = surf2Pts[maxIndex+2];

	delete[] surf2Pts;

	return maxCurr;
}
#else
float GetMaxDistanceInTexture(float4* cudaRes, int uNum, int vNum, int& iMax, int& jMax, float point2[3])
{

	float4 maxVal;
	int maxLoc;

	float maxCurr  = THRUSTDeviceFindMaxFloat4(cudaRes, (uNum+1)*(vNum+1), &maxVal, &maxLoc);

	point2[0] = maxVal.x;
	point2[1] = maxVal.y;
	point2[2] = maxVal.z;

	jMax = int(maxLoc/(uNum+1.0));
	iMax = maxLoc - (jMax)*(uNum+1);

	return maxCurr;
}
#endif

// Initialize and construct a vector of cuda arrays (inTexs) representing flat Bezier patches of surface2
//void ConstructBezDataTex(NURBS* surface2, std::vector<float4*>& inTexs, double flatnessTol)
void ConstructBezDataTex(NURBS* surface2, double flatnessTol)
{
	SplitSurfaceIntoFlatBezPatches(surface2, surface2->m_bezPatches, flatnessTol);
	surface2->CUDASetBezPatches();
}


void InitializeFBOTextures(float4* fboTexs[2], float* convTexs[2], int uNum, int vNum)
{
	cudaMalloc((void**)&(fboTexs[0]), (uNum+1)*(vNum+1)*sizeof(float4));	
	cudaMalloc((void**)&(fboTexs[1]), (uNum+1)*(vNum+1)*sizeof(float4));	
	cudaMalloc((void**)&(convTexs[0]), (uNum+1)*(vNum+1)*sizeof(float));	
	cudaMalloc((void**)&(convTexs[1]), (uNum+1)*(vNum+1)*sizeof(float));	


	// Construct second texture and initialize with initial large distance value.
	//iddo: TODO - maybe have an additional initialized texture for first pass only (instead of the memory copying here, use this texture)
	static float *maxDist = NULL; //method for not reallocating each time
	static float *convStatus = NULL; //method for not reallocating each time
	static unsigned allocSize = 0;
	if (allocSize < (uNum+1)*(vNum+1)*4)
	{
		allocSize = (uNum+1)*(vNum+1);
		maxDist = new float[allocSize*4];
		convStatus = new float[allocSize];
		for (unsigned i=0; i<allocSize; ++i)
		{
			//can probably do it with memset or std::fill or something..
			maxDist[4*i+0] = 10000.0;
			maxDist[4*i+1] = 10000.0;
			maxDist[4*i+2] = 10000.0;
			maxDist[4*i+3] = 10000.0;
			convStatus[i] = -1.0;
		}
		//iddo: need to set an initial value based on bboxes of surfaces (e.g., max(max1-min2,max2-min1)!
		//other (more robust) possibility is -1 and in shader check if positive
	}
	cudaMemcpy(fboTexs[1], maxDist, (uNum+1)*(vNum+1)*sizeof(float4), cudaMemcpyHostToDevice);	
	cudaMemcpy(convTexs[1], convStatus, (uNum+1)*(vNum+1)*sizeof(float), cudaMemcpyHostToDevice);	
}


#endif
