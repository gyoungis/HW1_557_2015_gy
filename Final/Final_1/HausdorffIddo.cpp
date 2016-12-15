#include "Hausdorff.h"
#include "GPUUtilities.h"

#include "NURBSGeomUtils.h" //for splitting into flat Bezier patches

//Iddo CG Hausdorff Code:

float GetMaxDistanceInTexture(GLuint outTex, int uNum, int vNum, int& iMax, int& jMax, float point2[3]);
void ConstructBezDataTex(NURBS* surface2, std::vector<GLuint>& inTexs, double flatnessTol);
void InitializeFBOTextures(GLuint fboTexs[2], int uNum, int vNum);


//Loop over all textures of flat Beziers in surf2 (=inTexs) and perform gpu frag prog.
float IddoGPUHausdorffDistance(NURBS* surface1, NURBS* surface2, const std::vector<GLuint>& inTexs, GLuint fboTexs[2], float* point1, float* point2, GLParameters* glParam)
{
	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;

	int uOrder2 = surface2->uOrder;
	int vOrder2 = surface2->vOrder;

	// Set up GPU computations
	StartGPUComputation(glParam);

	//setting cg program I/O
	cgGLEnableProfile(glParam->cgFragmentProfile);

	cgGLBindProgram(glParam->hausdorffIddoProgram); 

	// Initializing parameters:
	int k=0; //index of patch we "render"

	cgGLSetTextureParameter(glParam->hausdorffIddoSurface1Param, surface1->evalParams->evaluatedSurfTexture);
	cgGLEnableTextureParameter(glParam->hausdorffIddoSurface1Param);

	cgGLSetTextureParameter(glParam->hausdorffIddoSurface2Param, inTexs[k]);
	cgGLEnableTextureParameter(glParam->hausdorffIddoSurface2Param);

	cgGLSetTextureParameter(glParam->hausdorffIddoPrevDistsParam, fboTexs[(k+1)%2]);
	//iddo: if we want to have a constant texture for initialization (e.g., has 1000 in all entries)
	//we can use that here instead, and fboTexs[1] will not have to be initialized..
	cgGLEnableTextureParameter(glParam->hausdorffIddoPrevDistsParam);

	cgGLSetParameter1f(glParam->hausdorffIddoSurface2OrdUParam,uOrder2);
	cgGLSetParameter1f(glParam->hausdorffIddoSurface2OrdVParam,vOrder2);

	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, fboTexs[(k+1)%2]); //other fbo texture is now an input parameter
	//iddo: see comment above on initial constant texture
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inTexs[k]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->evaluatedSurfTexture);

	// Specifying dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, fboTexs[k%2], 0);
	glEnable(GL_COLOR_ATTACHMENT0_EXT);

	//gluOrtho2D(0,1,0,1);
	glViewport(0, 0, uNum+1, vNum+1);
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//After each iteration fboTexs[k%2] holds the output fbo.
	//In the end of the iterations fboTexs[(numOfSurfaces-1)%2] will hold the final output.
	unsigned numOfSurfaces = inTexs.size();
	//unsigned numOfSurfaces = 3; //debug
	for (k=0; k<numOfSurfaces; ++k)
	{
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, fboTexs[k%2], 0);

		cgGLSetTextureParameter(glParam->hausdorffIddoSurface2Param, inTexs[k]);
		cgGLEnableTextureParameter(glParam->hausdorffIddoSurface2Param);

		// Setting the prevDist texture shader parameter (when fboTexs[0] is the fbo output, fboTexs[1] is input and vice versa)
		cgGLSetTextureParameter(glParam->hausdorffIddoPrevDistsParam, fboTexs[(k+1)%2]);
		cgGLEnableTextureParameter(glParam->hausdorffIddoPrevDistsParam);
		glActiveTextureARB(GL_TEXTURE2_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, fboTexs[(k+1)%2]); //other fbo texture is now an input parameter

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inTexs[k]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->evaluatedSurfTexture);

		DrawQuad(uNum+1, vNum+1);
		glFlush(); 
		glFinish(); //iddo: is this needed? if not remove..

#ifdef DUMP
	{
	//debug - reading current fbo
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*4];

	//reading the frame buffer to file
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo); 
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, uNum+1, vNum+1, GL_RGBA, GL_FLOAT, retVal);
	//WriteToFileAll(retVal, uNum+1, vNum+1, 4);
    delete[] retVal;
    }
#endif 
	} //for k loop

#ifdef DUMP
	//iddo: debug - getting the final output texture
	{
	//reading output texture
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*4];
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,fboTexs[(numOfSurfaces-1)%2]);
	glGetTexImage( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, GL_FLOAT, retVal);
	WriteToFileAll(retVal, uNum+1, vNum+1, 4); //print rgba
	delete[] retVal;
	}
#endif

	int iMax, jMax;
	float maxDist = 
		GetMaxDistanceInTexture(fboTexs[(numOfSurfaces-1)%2], uNum, vNum, iMax, jMax, point2);
	int maxIndex = 4*(jMax*(uNum+1)+iMax);
	float * evaluatedPoints = surface1->evalParams->evaluatedPoints;
	float wij = evaluatedPoints[maxIndex+3];      
	point1[0] = evaluatedPoints[maxIndex+0]/wij;
	point1[1] = evaluatedPoints[maxIndex+1]/wij;
	point1[2] = evaluatedPoints[maxIndex+2]/wij;


	//iddo: for debug rendering - copy output texture to surface2->evalParams->evaluatedPoints
	/**
	{
		GLfloat* surf2Pts=new GLfloat[(uNum+1)*(vNum+1)*4];
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,fboTexs[(numOfSurfaces-1)%2]);
		glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, GL_FLOAT, surf2Pts);

		if (surface2->evalParams->evaluatedPoints != NULL)
			delete [] (surface2->evalParams->evaluatedPoints);
		surface2->evalParams->evaluatedPoints = new GLfloat[(uNum+1)*(vNum+1)*4];
		for (int i=0; i<(uNum+1)*(vNum+1)*4; ++i) {
			surface2->evalParams->evaluatedPoints[i] = surf2Pts[i];
		if (i%4 == 3)
			surface2->evalParams->evaluatedPoints[i] = 1.0; //the w coordinates holds the distance..
		}
	}
	/**/

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	//return the hausdorff distance	
	return maxDist;
}


float IddoGetSurfaceHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, int intRes, GLParameters* glParam, bool forcedEvaluate)
{
	glParam->hausdorff = true;
	int uNum = intRes;
	int vNum = intRes;
	bool timing = false;

	bool evaluate = true;
	if(surface1->initializedEvalParams && surface2->initializedEvalParams && !forcedEvaluate)
	{
		if (surface1->evalParams->uMin == 0.0  &&
			surface1->evalParams->uMax == 1.0  &&
			surface1->evalParams->vMin == 0.0  &&
			surface1->evalParams->vMax == 1.0  &&
			surface1->evalParams->uNum == uNum &&
			surface1->evalParams->vNum == vNum
        )
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

	if (evaluate)
	{
		if (glParam->readBack)
		{
			surface1->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
		}
		surface1->SurfaceEvaluation(glParam, false,false); //iddo: now surface1->evalParams->evaluatedPoints is filled
	}

#ifdef DUMP
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*4];

	//reading the frame buffer to file
	//glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo); //fbo holds the surface evaluated points (possibly we don't need to read them into surface1->evalParams->evaluatedPoints
	//glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	//glReadPixels(0, 0, uNum+1, vNum+1, GL_RGBA, GL_FLOAT, retVal);

	//reading a texture to file
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,surface1->evalParams->evaluatedSurfTexture);
	glGetTexImage( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, GL_FLOAT, retVal);


	WriteToFileAll(retVal, uNum+1, vNum+1, 4);
	delete[] retVal;
#endif

	//construct the input textures (flat Bezier patches) - possibly store them in surface2?
	double flatnessTol = -20.0; //negative flatnessTol means splitting only into Bezier (no check of flatness criterion)
	std::vector<GLuint> inTexs;
	ConstructBezDataTex(surface2, inTexs, flatnessTol); //don't forget to delete[] before dtr

	GLuint fboTexs[2]; //initialize textures fbo will copy to (in pingpong method)
	InitializeFBOTextures(fboTexs, uNum, vNum);

	assert(surface2->identityTransformation); //iddo: we have not implemented surface2 transformations in cg (only in CUDA)

	float hausdorffDist = IddoGPUHausdorffDistance(surface1, surface2, inTexs, fboTexs, point1, point2, glParam);

	// Cleanup initialized fboTexes
	glDeleteTextures(2, fboTexs); 

	//Cleanup the inTexs
	//iddo: probably store these inTexes in surface2 (they are the flat bez patches) so they
	//can be reused and perform this cleanup only when surface2 is destructed.
	for (unsigned i=0; i<inTexs.size(); ++i) {
		glDeleteTextures(1,&inTexs[i]);
	}

	glParam->hausdorff = false;

	printf("Iddo Finished!\n");
	return hausdorffDist;
}




// Getting max distance in the texture.
// iddo: current implementation just goes over the texture, in the future use reduction.
float GetMaxDistanceInTexture(GLuint outTex, int uNum, int vNum, int& iMax, int& jMax, float point2[3])
{
	//reading output texture
	GLfloat* surf2Pts=new GLfloat[(uNum+1)*(vNum+1)*4];
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,outTex);
	glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, GL_FLOAT, surf2Pts);

	float maxCurr = 0.0;
	int iMaxCurr = 0;
	int jMaxCurr = 0;

	for(int j=0; j < vNum+1; j++)
	{
		for(int i=0; i < uNum+1; i++)
		{
			if (surf2Pts[4*(j*(uNum+1)+i)+3] > maxCurr) {
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

// Initialize and construct a vector of textures (inTexs) representing flat Bezier patches of surface2
void ConstructBezDataTex_SAVE_ORIG(NURBS* surface2, std::vector<GLuint>& inTexs, double flatnessTol)
{
	//iddo: for now assuming surface2 is a Bezier-type surface and no splitting,
	//in the future will call the NURBSGeom functions to split into flat patches

	//iddo: generating input texture from control points of surface2 
	//(in this test we assume surface2 is a Bezier-type surface)
	GLuint inTex;
	glGenTextures(1,&inTex);
	int uNum2 = surface2->uPoints;
	int vNum2 = surface2->vPoints;
	InitTexture(&inTex, RGBA_FLOAT_FORMAT, uNum2, vNum2, GL_RGBA);

	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, uNum2, vNum2, 0, GL_RGBA, GL_FLOAT, surface2->cntlPoints);
	inTexs.push_back(inTex);
}

// Initialize and construct a vector of textures (inTexs) representing flat Bezier patches of surface2
void ConstructBezDataTex(NURBS* surface2, std::vector<GLuint>& inTexs, double flatnessTol)
{
	int i;
	std::vector<float*> bezTexData;
	SplitSurfaceIntoFlatBezPatches(surface2, bezTexData, flatnessTol);

	//iddo: generating input texture from control points of surface2 
	//(in this test we assume surface2 is a Bezier-type surface)
	for (i=0; i<bezTexData.size(); ++i) {
		GLuint inTex;
		glGenTextures(1,&inTex);
		int uNum2 = surface2->uOrder;
		int vNum2 = surface2->vOrder;
		InitTexture(&inTex, RGBA_FLOAT_FORMAT, uNum2, vNum2, GL_RGBA);

		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, uNum2, vNum2, 0, GL_RGBA, GL_FLOAT, bezTexData[i]);
		inTexs.push_back(inTex);
	}

	//deleting bezTexData pointers
	for (i=0; i<bezTexData.size(); ++i) {
		delete [] bezTexData[i]; 
	}
}


void InitializeFBOTextures(GLuint fboTexs[2], int uNum, int vNum)
{
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2,fboTexs);
	InitTexture(fboTexs+0, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA); 

	// Construct second texture and initialize with initial large distance value.
	//iddo: TODO - maybe have an additional initialized texture for first pass only (instead of the memory copying here, use this texture)
	static float *maxDist = NULL; //method for not reallocating each time
	static unsigned allocSize = 0;
	if (allocSize < (uNum+1)*(vNum+1)*4) {
		allocSize = (uNum+1)*(vNum+1)*4;
		maxDist = new float[allocSize];
		for (unsigned i=0; i<allocSize; ++i)
			maxDist[i] = 1000.0; //can probably do it with memset or std::fill or something..
			//iddo: need to set an initial value based on bboxes of surfaces (e.g., max(max1-min2,max2-min1)!
			//other (more robust) possibility is -1 and in shader check if positive
	}
	//Initialize texture with maxDist..
	InitTexture(fboTexs+1, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA, maxDist);
}


