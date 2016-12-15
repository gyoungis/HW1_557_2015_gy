#include "GeometricMoments.h"
#include "GPUUtilities.h"
Timer timer1, timer2;

// Moment Computation
float ApproxTriangleVolume(Float3 v0, Float3 v1, Float3 v2)
{
	Float3 midPoint = (v0 + v1 + v2)/3.0;
	Float3 v0v1 = v1 - v0;
	Float3 v0v2 = v2 - v0;
	Float3 n = VectorCrossProduct(v0v1, v0v2);
	float triangleArea = 0.5*VectorMagnitude(n);
	VectorNormalize(n);
	float approxTriVolume = midPoint[2]*n[2]*triangleArea;
	return approxTriVolume;
}

Float4 ApproxTriangleMoment(Float3 v0, Float3 v1, Float3 v2, Float3 n0, Float3 n1, Float3 n2)
{
	Float3 midPoint = (v0 + v1 + v2)/3.0;
//	Float3 averageNormal = (n0 + n1 + n2)/3.0;

	Float4 momentFunction = Float4(1.0, midPoint[0], midPoint[1], midPoint[2]*.5);

	Float3 v0v1 = v1 - v0;
	Float3 v0v2 = v2 - v0;
	Float3 n = VectorCrossProduct(v0v1, v0v2);
//	float triangleArea = 0.5*VectorMagnitude(n);
//	float dMoment  =  averageNormal[2]*midPoint[2]*triangleArea;
	float dMoment  =  n[2]*midPoint[2]*0.5;
	Float4 approxTriMoment = momentFunction*dMoment;
	return approxTriMoment;
}

Float4 ApproxTriangleMoment(Float3 v0, Float3 v1, Float3 v2)
{
	Float3 midPoint = (v0 + v1 + v2)/3.0;

	Float4 momentFunction = Float4(1.0, midPoint[0], midPoint[1], midPoint[2]*.5);

	Float3 v0v1 = v1 - v0;
	Float3 v0v2 = v2 - v0;
	Float3 n = VectorCrossProduct(v0v1, v0v2);
	float dMoment  =  n[2]*midPoint[2]*0.5;
	Float4 approxTriMoment = momentFunction*dMoment;
	return approxTriMoment;
}

float ApproxQuadMoment(Float3 v0, Float3 v1, Float3 v2, Float3 v3, Float3 n0, Float3 n1, Float3 n2, Float3 n3, int uNum, int vNum, int momentType)
{
	Float3 midPoint			= (v0 + v1 + v2 + v3)/4.0;
	Float3 averageNormal	= (n0 + n1 + n2 + n3)/4.0;
	float momentFunction	= 1.0;

	if (momentType == 1)
		momentFunction = midPoint[0];
	else if (momentType == 2)
		momentFunction = midPoint[1];
	else if (momentType == 3)
		momentFunction = midPoint[2]*.5;
	
	float quadArea = 1.0/(uNum*vNum);
	float dMoment  =  averageNormal[2]*midPoint[2]*quadArea;
	float approxQuadMoment = momentFunction*dMoment;
	return approxQuadMoment;
}

Float4 GetTriangulatedMoment(Face* face)
{
	bool timing = false;
	if (timing)
		timer1.Start();

	Float4 signedMoment = Float4(0, 0, 0, 0);
	for (int j = 0; j < face->triangles.size(); j++)
	{
		// Add the vertex points if not already added
		Float3 vertex0 = face->triangles[j].vertices[0].point;
		Float3 vertex1 = face->triangles[j].vertices[1].point;
		Float3 vertex2 = face->triangles[j].vertices[2].point;

		Float3 normal0 = face->triangles[j].vertices[0].normal;
		Float3 normal1 = face->triangles[j].vertices[1].normal;
		Float3 normal2 = face->triangles[j].vertices[2].normal;

		Float4 moment = ApproxTriangleMoment(vertex0, vertex1, vertex2, normal0, normal1, normal2);
		signedMoment += moment;
	}

 	if (timing)
	{
		timer1.Stop();
		cout << "Volume : " << signedMoment[0] << endl;
		cout << "Volume Computation Time : " << timer1.GetElapsedTimeInMicroSec()*0.001 << endl << endl;
	}
	return signedMoment;
}


float CPUComputeSurfaceMoment(NURBS* surface, int momentType, GLParameters* glParam)
{
	int uNum = surface->evalParams->uNum;
	int vNum = surface->evalParams->vNum;
	float* evaluatedPoints = surface->evalParams->evaluatedPoints;
	float* evaluatedNormals = surface->evalParams->evaluatedNormals;
	float moment = 0;

	for (int vIndex = 0; vIndex < vNum; vIndex++)
	{
		for (int uIndex = 0; uIndex < uNum; uIndex++)
		{
			Float3 A[4], N[4];

			A[0][0] = evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+0]/evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+3];
			A[0][1] = evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+1]/evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+3];
			A[0][2] = evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+2]/evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+3];

			A[1][0] = evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+0]/evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+3];
			A[1][1] = evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+1]/evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+3];
			A[1][2] = evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+2]/evaluatedPoints[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+3];

			A[2][0] = evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+0]/evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+3];
			A[2][1] = evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+1]/evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+3];
			A[2][2] = evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+2]/evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+3];

			A[3][0] = evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+0]/evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+3];
			A[3][1] = evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+1]/evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+3];
			A[3][2] = evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+2]/evaluatedPoints[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+3];

			N[0][0] = evaluatedNormals[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+0];
			N[0][1] = evaluatedNormals[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+1];
			N[0][2] = evaluatedNormals[(vIndex+0)*(uNum+1)*4+(uIndex+0)*4+2];

			N[1][0] = evaluatedNormals[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+0];
			N[1][1] = evaluatedNormals[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+1];
			N[1][2] = evaluatedNormals[(vIndex+0)*(uNum+1)*4+(uIndex+1)*4+2];

			N[2][0] = evaluatedNormals[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+0];
			N[2][1] = evaluatedNormals[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+1];
			N[2][2] = evaluatedNormals[(vIndex+1)*(uNum+1)*4+(uIndex+1)*4+2];

			N[3][0] = evaluatedNormals[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+0];
			N[3][1] = evaluatedNormals[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+1];
			N[3][2] = evaluatedNormals[(vIndex+1)*(uNum+1)*4+(uIndex+0)*4+2];

			float trimFactor = 1.0;
			if (surface->trimmed)
			{
				float T[4];
				T[0] = surface->evalParams->trimData[(vIndex+0)*(uNum+1)+(uIndex+0)];
				T[1] = surface->evalParams->trimData[(vIndex+0)*(uNum+1)+(uIndex+1)];
				T[2] = surface->evalParams->trimData[(vIndex+1)*(uNum+1)+(uIndex+1)];
				T[3] = surface->evalParams->trimData[(vIndex+1)*(uNum+1)+(uIndex+0)];
				trimFactor = (T[0]+T[1]+T[2]+T[3])/4.0;
			}
			moment += trimFactor*ApproxQuadMoment(A[0], A[1], A[2], A[3], N[0], N[1], N[2], N[3], uNum, vNum, momentType);
		}
	}
	return moment;
}

Float4 CPUComputeSurfaceMomentQuadrature(NURBS* surface, int uNumPatches, int vNumPatches, int numQuadraturePoints, GLParameters* glParam)
{
	int uNum = surface->evalParams->uNum;
	int vNum = surface->evalParams->vNum;
	float* evaluatedPoints = surface->evalParams->evaluatedPoints;
	float* evaluatedNormals = surface->evalParams->evaluatedNormals;
	Float4 moment = Float4(0,0,0,0);

	float dS = 0.25/((uNumPatches)*(vNumPatches));
	int uNumTrim = surface->evalParams->uNumTrim;
	int vNumTrim = surface->evalParams->vNumTrim;
	int uNumTrimPointsPerPatch = (uNumTrim+1)/uNumPatches;
	int vNumTrimPointsPerPatch = (vNumTrim+1)/vNumPatches;

	for (int vPatchIndex = 0; vPatchIndex < vNumPatches; vPatchIndex++)
	{
		for (int uPatchIndex = 0; uPatchIndex < uNumPatches; uPatchIndex++)
		{
			Float4 quadMoment = Float4(0,0,0,0);
			for (int vPoint = 0; vPoint < numQuadraturePoints; vPoint++)
			{
				float vWeight = 1.0;
				if (numQuadraturePoints == 3)
				{
					if (vPoint%3 == 1)
						vWeight = float(8.0/9.0);
					else
						vWeight = float(5.0/9.0);
				}

				for (int uPoint = 0; uPoint < numQuadraturePoints; uPoint++)
				{
					float uWeight = 1.0;
					if (numQuadraturePoints == 3)
					{
						if (uPoint%3 == 1)
							uWeight = float(8.0/9.0);
						else
							uWeight = float(5.0/9.0);
					}

					int uIndex = uPatchIndex*numQuadraturePoints+uPoint;
					int vIndex = vPatchIndex*numQuadraturePoints+vPoint;

					Float3 surfPoint, normal;

					surfPoint[0] = evaluatedPoints[(vIndex)*(uNum+1)*4+(uIndex)*4+0]/evaluatedPoints[(vIndex)*(uNum+1)*4+(uIndex)*4+3];
					surfPoint[1] = evaluatedPoints[(vIndex)*(uNum+1)*4+(uIndex)*4+1]/evaluatedPoints[(vIndex)*(uNum+1)*4+(uIndex)*4+3];
					surfPoint[2] = evaluatedPoints[(vIndex)*(uNum+1)*4+(uIndex)*4+2]/evaluatedPoints[(vIndex)*(uNum+1)*4+(uIndex)*4+3];

					normal[0] = evaluatedNormals[(vIndex)*(uNum+1)*4+(uIndex)*4+0];
					normal[1] = evaluatedNormals[(vIndex)*(uNum+1)*4+(uIndex)*4+1];
					normal[2] = evaluatedNormals[(vIndex)*(uNum+1)*4+(uIndex)*4+2];

					Float4 momentFunction = Float4(1.0, surfPoint[0], surfPoint[1], surfPoint[2]*.5);
					quadMoment[0] += uWeight*vWeight*momentFunction[0]*normal[2]*surfPoint[2]*dS;
					quadMoment[1] += uWeight*vWeight*momentFunction[1]*normal[2]*surfPoint[2]*dS;
					quadMoment[2] += uWeight*vWeight*momentFunction[2]*normal[2]*surfPoint[2]*dS;
					quadMoment[3] += uWeight*vWeight*momentFunction[3]*normal[2]*surfPoint[2]*dS;
				}
			}


			float trimFactor = 1.0;
			if (surface->trimmed)
			{
				float sumTrims = 0;
				int numTrimPoints = 0;
				for (int vPoint = 0; vPoint < vNumTrimPointsPerPatch; vPoint++)
				{
					for (int uPoint = 0; uPoint < uNumTrimPointsPerPatch; uPoint++)
					{
						int uIndex = uPatchIndex*uNumTrimPointsPerPatch + uPoint;
						int vIndex = vPatchIndex*vNumTrimPointsPerPatch + vPoint;
						sumTrims += surface->evalParams->trimData[(vIndex)*(uNumTrim+1)+(uIndex)];
						numTrimPoints++;
					}
				}
				trimFactor = sumTrims/(1.0*numTrimPoints);
			}

			moment[0] += trimFactor*quadMoment[0];
			moment[1] += trimFactor*quadMoment[1];
			moment[2] += trimFactor*quadMoment[2];
			moment[3] += trimFactor*quadMoment[3];
		}
	}
	return moment;
}

Float4 GPUComputeSurfaceMomentQuadrature(NURBS* surface, int uNumPatches, int vNumPatches, Float4* surfaceMomentError, GLParameters* glParam)
{
	bool operationFailed = false;
	bool timing = false;
	float momentTime = 0, trimTime = 0, errorTime = 0, reduceTime = 0, totalTime = 0;
	float momentError[4] = {0, 0, 0, 0};

	if (timing)
	{
		timer1.Start();
		timer2.Start();
	}

	int uNum = surface->evalParams->uNum;
	int vNum = surface->evalParams->vNum;

	int uNumTrim = surface->evalParams->uNumTrim;
	int vNumTrim = surface->evalParams->vNumTrim;

	int numQuadraturePoints = surface->evalParams->numQuadraturePoints;

	float dS = surface->normalSign*0.25/((uNumPatches)*(vNumPatches));
	if (numQuadraturePoints == 1)
		dS *= 4;

	// Set up GPU computations
	StartGPUComputation(glParam);

	//Set up for evaluation
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ZERO);

	GLuint tempTex[5];
	GLuint errorTempTex;
	glGenTextures(5,tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);

	if (numQuadraturePoints == 1 || numQuadraturePoints == 2)
	{
		cgGLBindProgram(glParam->moment2PointProgram);

		cgGLEnableTextureParameter(glParam->moment2PointSurfaceTextureParam);
		cgGLSetTextureParameter(glParam->moment2PointSurfaceTextureParam, surface->evalParams->evaluatedSurfTexture);
		cgGLEnableTextureParameter(glParam->moment2PointNormalTextureParam);
		cgGLSetTextureParameter(glParam->moment2PointNormalTextureParam, surface->evalParams->evaluatedSurfNormTexture);

		cgGLSetParameter1f(glParam->moment2PointMultConstantParam, dS);
	}
	else if (numQuadraturePoints == 3)
	{
		cgGLBindProgram(glParam->moment3PointProgram);

		cgGLEnableTextureParameter(glParam->moment3PointSurfaceTextureParam);
		cgGLSetTextureParameter(glParam->moment3PointSurfaceTextureParam, surface->evalParams->evaluatedSurfTexture);
		cgGLEnableTextureParameter(glParam->moment3PointNormalTextureParam);
		cgGLSetTextureParameter(glParam->moment3PointNormalTextureParam, surface->evalParams->evaluatedSurfNormTexture);

		cgGLSetParameter1f(glParam->moment3PointMultConstantParam, dS);
	}

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface->evalParams->evaluatedSurfNormTexture);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface->evalParams->evaluatedSurfTexture);

	glViewport(0, 0, uNum+1, vNum+1);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

	DrawQuad(uNum+1, vNum+1);
	glFlush();
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*4];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, uNum+1, vNum+1, GL_RGBA, GL_FLOAT, retVal);
	WriteToFile(retVal, uNum+1, vNum+1, 0);
#endif

	if (numQuadraturePoints == 1 || numQuadraturePoints == 2)
	{
		cgGLDisableTextureParameter(glParam->moment2PointSurfaceTextureParam);
		cgGLDisableTextureParameter(glParam->moment2PointNormalTextureParam);
	}
	else if (numQuadraturePoints == 3)
	{
		cgGLDisableTextureParameter(glParam->moment3PointSurfaceTextureParam);
		cgGLDisableTextureParameter(glParam->moment3PointNormalTextureParam);
	}

	GLuint momentOutputTex = tempTex[0];

	// Reduce based on number of quadrature points
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, uNumPatches, vNumPatches, GL_RGBA);
	if (numQuadraturePoints == 2)
	{
		Reduce4Sum(tempTex[0], tempTex[1], uNum+1, vNum+1, glParam);
		momentOutputTex = tempTex[1];
	}
	else if (numQuadraturePoints == 3)
	{
		Reduce9Sum(tempTex[0], tempTex[1], uNum+1, vNum+1, glParam);
		momentOutputTex = tempTex[1];
	}

	if (timing)
	{
		timer2.Stop();
		momentTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}

	// Reduce trim texture to number of patches
	int uNumTrimPointsPerPatch = (uNumTrim+1)/uNumPatches;
	int vNumTrimPointsPerPatch = (vNumTrim+1)/vNumPatches;
	GLuint outputTrimTex = surface->evalParams->trimTexture;
	if (surface->trimmed)
	{
		if (uNumTrimPointsPerPatch == 2 || uNumTrimPointsPerPatch == 4)
		{
			InitTexture(tempTex+2, LUMINANCE_FLOAT_FORMAT, (uNumTrim+1)/2, (vNumTrim+1)/2, GL_LUMINANCE);
			Reduce4Sum(outputTrimTex, tempTex[2], uNumTrim+1, vNumTrim+1, glParam);
			outputTrimTex = tempTex[2];	

			if (uNumTrimPointsPerPatch==4)
			{
				InitTexture(tempTex+3, LUMINANCE_FLOAT_FORMAT, (uNumTrim+1)/4, (vNumTrim+1)/4, GL_LUMINANCE);
				Reduce4Sum(outputTrimTex, tempTex[3], (uNumTrim+1)/2, (vNumTrim+1)/2, glParam);
				outputTrimTex = tempTex[3];
			}
		}
		else if (uNumTrimPointsPerPatch == 3)
		{
			InitTexture(tempTex+2, LUMINANCE_FLOAT_FORMAT, (uNumTrim+1)/3, (vNumTrim+1)/3, GL_LUMINANCE);
			Reduce9Sum(outputTrimTex, tempTex[2], uNumTrim+1, vNumTrim+1, glParam);	
			outputTrimTex = tempTex[2];
		}

		// Multiply trim texture and moment values
		InitTexture(tempTex+4, RGBA_FLOAT_FORMAT, uNumPatches, vNumPatches, GL_RGBA);
		cgGLBindProgram(glParam->momentTrimMultProgram);

		cgGLEnableTextureParameter(glParam->momentTrimMultMomentTextureParam);
		cgGLSetTextureParameter(glParam->momentTrimMultMomentTextureParam, momentOutputTex);
		cgGLEnableTextureParameter(glParam->momentTrimMultTrimTextureParam);
		cgGLSetTextureParameter(glParam->momentTrimMultMomentTextureParam, outputTrimTex);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, outputTrimTex);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, momentOutputTex);

		glViewport(0, 0, uNumPatches, vNumPatches);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[4], 0);

		DrawQuad(uNumPatches, vNumPatches);
		glFlush();
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[uNumPatches*vNumPatches*4];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, uNumPatches, vNumPatches, GL_RGBA, GL_FLOAT, retVal);
		WriteToFile(retVal, uNumPatches, vNumPatches, 0);
#endif
		cgGLDisableTextureParameter(glParam->momentTrimMultTrimTextureParam);
		cgGLDisableTextureParameter(glParam->momentTrimMultMomentTextureParam);
		momentOutputTex = tempTex[4];
	}

	if (timing)
	{
		timer2.Stop();
		trimTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}

	// Perform Error Analysis
	if (surface->evalParams->computeMomentError && numQuadraturePoints > 1)
	{
		glGenTextures(1,&errorTempTex);
		InitTexture(&errorTempTex, RGBA_FLOAT_FORMAT, uNumPatches, vNumPatches, GL_RGBA);

		cgGLBindProgram(glParam->momentErrorProgram);

		cgGLEnableTextureParameter(glParam->momentErrorTrimTextureParam);
		cgGLSetTextureParameter(glParam->momentErrorTrimTextureParam, outputTrimTex);
		cgGLEnableTextureParameter(glParam->momentErrorSurfaceTextureParam);
		cgGLSetTextureParameter(glParam->momentErrorSurfaceTextureParam, surface->evalParams->evaluatedSurfTexture);
		cgGLEnableTextureParameter(glParam->momentErrorExpansionTextureParam);
		cgGLSetTextureParameter(glParam->momentErrorExpansionTextureParam, surface->evalParams->expansionFactorTexture);
		cgGLEnableTextureParameter(glParam->momentErrorMomentTextureParam);
		cgGLSetTextureParameter(glParam->momentErrorMomentTextureParam, momentOutputTex);
		
		float numTrimPoints = 0;
		if (surface->trimmed)
			numTrimPoints = uNumTrimPointsPerPatch*vNumTrimPointsPerPatch;
		cgGLSetParameter1f(glParam->momentErrorTrimPointsParam, numTrimPoints);
		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE3_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, outputTrimTex);
		glActiveTextureARB(GL_TEXTURE2_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, momentOutputTex);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface->evalParams->expansionFactorTexture);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface->evalParams->evaluatedSurfTexture);

		glViewport(0, 0, uNumPatches, vNumPatches);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, errorTempTex, 0);

		DrawQuad(uNumPatches, vNumPatches);
		glFlush();
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[uNumPatches*vNumPatches*4];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, uNumPatches, vNumPatches, GL_RGBA, GL_FLOAT, retVal);
		WriteToFile(retVal, uNumPatches, vNumPatches, 0);
#endif
		cgGLDisableTextureParameter(glParam->momentErrorSurfaceTextureParam);
		cgGLDisableTextureParameter(glParam->momentErrorExpansionTextureParam);
		cgGLDisableTextureParameter(glParam->momentErrorMomentTextureParam);
		cgGLDisableTextureParameter(glParam->momentErrorTrimTextureParam);

		ReduceMomentSum(errorTempTex, uNumPatches, vNumPatches, glParam, momentError);
	}

	// Reduce to moment value
	if (timing)
	{
		timer2.Stop();
		errorTime = timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}

	float momentData[4];
	ReduceMomentSum(momentOutputTex, uNumPatches, vNumPatches, glParam, momentData);
	if (surface->trimmed)
	{
		momentData[0] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
		momentData[1] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
		momentData[2] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
		momentData[3] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);

		if (surface->evalParams->computeMomentError)
		{
			momentError[0] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
			momentError[1] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
			momentError[2] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
			momentError[3] /= (uNumTrimPointsPerPatch*vNumTrimPointsPerPatch);
		}
	}
	(*surfaceMomentError) = Float4(momentError[0], momentError[1], momentError[2], momentError[3]);

	glDeleteTextures(5, tempTex);
	if (surface->evalParams->computeMomentError && surface->trimmed)
		glDeleteTextures(1, &errorTempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (timing)
	{
		timer2.Stop();
		reduceTime = timer2.GetElapsedTimeInMicroSec();
	}

	if (timing)
	{
		timer1.Stop();
		totalTime = timer1.GetElapsedTimeInMicroSec();
		cout << "Moment Time : " << momentTime*0.001 << endl;
		cout << "Trim   Time : " << trimTime*0.001 << endl;
		cout << "Error  Time : " << errorTime*0.001 << endl;
		cout << "Reduce Time : " << reduceTime*0.001 << endl;
		cout << "Total  Time : " << totalTime*0.001 << endl << endl;
	}
	if (!operationFailed)
		return Float4(momentData[0], momentData[1], momentData[2], momentData[3]);
	else
		return Float4(0,0,0,0);
}

void PrepareEvaluationIntervalsEdge(float pMin, float pMax, int numPatches, int numQuadraturePoints, float* paramPoints)
{
	float patchLength = (pMax - pMin)/(1.0*numPatches-2);
	paramPoints[0] = 0;
	paramPoints[1] = 0;
	if (numQuadraturePoints == 3)
		paramPoints[2] = 0;
	for (int patchIndex = 1; patchIndex < numPatches-1; patchIndex++)
	{
		float patchMidPoint = float((patchIndex-0.5)*patchLength + pMin);
		if (numQuadraturePoints == 2)
		{
			float param1 = float(patchMidPoint - (0.5*patchLength/sqrt(3.0)));
			float param2 = float(patchMidPoint + (0.5*patchLength/sqrt(3.0)));
			paramPoints[patchIndex*2 + 0] = param1;
			paramPoints[patchIndex*2 + 1] = param2;
		}
		else if (numQuadraturePoints == 3)
		{
			float param1 = float(patchMidPoint - (0.5*patchLength*sqrt(0.6)));
			float param2 = patchMidPoint;
			float param3 = float(patchMidPoint + (0.5*patchLength*sqrt(0.6)));
			paramPoints[patchIndex*3 + 0] = param1;
			paramPoints[patchIndex*3 + 1] = param2;
			paramPoints[patchIndex*3 + 2] = param3;
		}
	}
	paramPoints[(numPatches-1)*numQuadraturePoints+0] = 1.0;
	paramPoints[(numPatches-1)*numQuadraturePoints+1] = 1.0;
	if (numQuadraturePoints == 3)
		paramPoints[(numPatches-1)*numQuadraturePoints+2] = 1.0;
}

void PrepareEvaluationIntervals(float pMin, float pMax, int numPatches, int numQuadraturePoints, float* paramPoints)
{
	float patchLength = (pMax - pMin)/(1.0*numPatches);
	for (int patchIndex = 0; patchIndex < numPatches; patchIndex++)
	{
		float patchMidPoint = float((patchIndex+0.5)*patchLength + pMin);
		if (numQuadraturePoints == 2)
		{
			float param1 = float(patchMidPoint - (0.5*patchLength/sqrt(3.0)));
			float param2 = float(patchMidPoint + (0.5*patchLength/sqrt(3.0)));
			paramPoints[patchIndex*2 + 0] = param1;
			paramPoints[patchIndex*2 + 1] = param2;
		}
		else if (numQuadraturePoints == 3)
		{
			float param1 = float(patchMidPoint - (0.5*patchLength*sqrt(0.6)));
			float param2 = patchMidPoint;
			float param3 = float(patchMidPoint + (0.5*patchLength*sqrt(0.6)));
			paramPoints[patchIndex*3 + 0] = param1;
			paramPoints[patchIndex*3 + 1] = param2;
			paramPoints[patchIndex*3 + 2] = param3;
		}
	}
}


Float4 GetSurfaceMoment(NURBS* surface, int uNumPatches, int vNumPatches, int numQuadraturePoints, int numTrimPointsPerPatch, GLParameters* glParam, Float4* surfaceMomentError = NULL, bool errorAnalysis = false, bool forcedEvaluate = false)
{
	glParam->computingMoment = true;
	bool timing = false;
	float evaluationTime = 0;
	float momentTime = 0;

	int uNum	 = uNumPatches*numQuadraturePoints - 1;
	int vNum	 = vNumPatches*numQuadraturePoints - 1;
	int uNumTrim = uNumPatches*numTrimPointsPerPatch - 1;
	int vNumTrim = vNumPatches*numTrimPointsPerPatch - 1;

	if (timing)
	{
		timer1.Start();
		timer2.Start();
	}

	bool evaluate = true;
	if(surface->initializedEvalParams && !forcedEvaluate)
	{
		if (surface->evalParams->uMin == 0.0  &&
			surface->evalParams->uMax == 1.0  &&
			surface->evalParams->vMin == 0.0  &&
			surface->evalParams->vMax == 1.0  &&
			surface->evalParams->uNum == uNum &&
			surface->evalParams->vNum == vNum)
				evaluate = false;
	}

	if (!evaluate && forcedEvaluate)
		evaluate = true;

	if (evaluate)
	{
		if (surface->initializedEvalParams)
			delete surface->evalParams;
		surface->evalParams = new EvaluationParams();
		surface->initializedEvalParams = true;

		//surface->evalParams->evaluatedPoints = new float[(uNum+1)*(vNum+1)*4];
		//surface->evalParams->evaluatedNormals = new float[(uNum+1)*(vNum+1)*4];
		//surface->evalParams->trimData = new float[(uNum+1)*(vNum+1)];

		surface->evalParams->uNum		= uNum;
		surface->evalParams->vNum		= vNum;
		surface->evalParams->uNumTrim	= uNumTrim;
		surface->evalParams->vNumTrim	= vNumTrim;

		surface->evalParams->uMin = 0.0;
		surface->evalParams->uMax = 1.0;
		surface->evalParams->vMin = 0.0;
		surface->evalParams->vMax = 1.0;

		surface->evalParams->numQuadraturePoints = numQuadraturePoints;
		surface->evalParams->computeMomentError = errorAnalysis;
	}

	if (evaluate)
	{
		if (numQuadraturePoints >= 2)
		{
			// Initialize the gaussian evaluation grid
			surface->evalParams->uEvalParamPoints = new float[uNum+1];
			float uMin = surface->evalParams->uMin;
			float uMax = surface->evalParams->uMax;

			PrepareEvaluationIntervals(uMin, uMax, uNumPatches, numQuadraturePoints, surface->evalParams->uEvalParamPoints);

			surface->evalParams->vEvalParamPoints = new float[vNum+1];
			float vMin = surface->evalParams->vMin;
			float vMax = surface->evalParams->vMax;

			PrepareEvaluationIntervals(vMin, vMax, vNumPatches, numQuadraturePoints, surface->evalParams->vEvalParamPoints);

			surface->SurfaceEvaluation(glParam, false, true);
		}
		else
			surface->SurfaceEvaluation(glParam, false, false);

		surface->GPUExactNormalEvaluation(glParam);

		if (surface->evalParams->computeMomentError)
			surface->SurfaceExpansionFactorEvaluation(glParam);

		if (surface->trimmed)
			surface->GenerateTrimData(uNumTrim, vNumTrim, glParam);
	}

	if (timing)
	{
		timer2.Stop();
		evaluationTime += timer2.GetElapsedTimeInMicroSec();
		timer2.Start();
	}

	Float4 surfaceMoment = GPUComputeSurfaceMomentQuadrature(surface, uNumPatches, vNumPatches, surfaceMomentError, glParam);
//	Float4 surfaceMoment = CPUComputeSurfaceMomentQuadrature(surface, uNumPatches, vNumPatches, numQuadraturePoints, glParam);
//	float surfaceMoment = CPUComputeSurfaceMoment(surface, momentType, glParam);

	if (timing)
	{
		timer2.Stop();
		momentTime = timer2.GetElapsedTimeInMicroSec();
	}

	if (timing)
	{
		timer1.Stop();
		cout<<"Total Time        : "<<(0.001*timer1.GetElapsedTimeInMicroSec())<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationTime)<<endl;
		cout<<"Moment Time       : "<<(0.001*momentTime)<<endl;
	}

	glParam->computingMoment = false;
	return surfaceMoment;
}

Float4 GetGPUTriangulatedMoment(float* triangleVertex1Data, float* triangleVertex2Data, float* triangleVertex3Data, int numTriangles, int texSize, GLParameters* glParam)
{
	float moment[4];
	
	// Set up GPU computations
	StartGPUComputation(glParam);

	GLuint tempTex[4];
	glGenTextures(4, tempTex);
	//0 - Triangle 1
	//1 - Triangle 2
	//2 - Triangle 3
	//3 - Moment Tex

	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, texSize, texSize, GL_RGBA, triangleVertex1Data);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, texSize, texSize, GL_RGBA, triangleVertex2Data);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, texSize, texSize, GL_RGBA, triangleVertex3Data);
	InitTexture(tempTex+3, RGBA_FLOAT_FORMAT, texSize, texSize, GL_RGBA);

	cgGLBindProgram(glParam->momentTriangleProgram);

	cgGLEnableTextureParameter(glParam->momentTriangleVertex1TextureParam);
	cgGLSetTextureParameter(glParam->momentTriangleVertex1TextureParam, tempTex[0]);
	cgGLEnableTextureParameter(glParam->momentTriangleVertex2TextureParam);
	cgGLSetTextureParameter(glParam->momentTriangleVertex2TextureParam, tempTex[1]);
	cgGLEnableTextureParameter(glParam->momentTriangleVertex3TextureParam);
	cgGLSetTextureParameter(glParam->momentTriangleVertex3TextureParam, tempTex[2]);
	
	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

	glViewport(0, 0, texSize, texSize);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[3], 0);

	DrawQuad(texSize, texSize);
	glFlush();
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[texSize*texSize];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texSize, texSize, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, texSize, texSize);
#endif
	cgGLDisableTextureParameter(glParam->momentTriangleVertex1TextureParam);
	cgGLDisableTextureParameter(glParam->momentTriangleVertex2TextureParam);
	cgGLDisableTextureParameter(glParam->momentTriangleVertex3TextureParam);

	ReduceMomentSum(tempTex[3], texSize, texSize, glParam, moment);

	glDeleteTextures(4, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	return Float4(moment[0], moment[1], moment[2], moment[3]);
}

Float4 GetGPUObjectMoment(Object* object, int baseResMult, int numQuadraturePoints, int numTrimPointsPerPatch, GLParameters* glParam, Float4* momentError, bool errorAnalysis = false, bool forcedEvaluate = false, bool timing = false, bool triGPUMoments = false)
{
	float surfaceMomentTime = 0, triangulatedMomentTime = 0;
	if (timing)
		timer1.Start();

	int maxuNum = 0, maxvNum = 0;
	int totalPatches = 0;
	Float4 moment = Float4(0, 0, 0, 0);
	(*momentError) = Float4(0, 0, 0, 0);

	int numTriangles = object->totalNumTriangles;
	int triangleTexSize = int(sqrt(numTriangles*1.0)) + 1;
	assert(triangleTexSize*triangleTexSize >= numTriangles);
	float* triangleVertex1Data;
	float* triangleVertex2Data;
	float* triangleVertex3Data;
	if (triGPUMoments)
	{
		triangleVertex1Data = new float[triangleTexSize*triangleTexSize*4];
		triangleVertex2Data = new float[triangleTexSize*triangleTexSize*4];
		triangleVertex3Data = new float[triangleTexSize*triangleTexSize*4];
	}

	int currentTriCount = 0;
	for (int p = 0; p < object->faces.size(); p++)
	{
		Face* face = object->faces[p];
		if (face->isNURBS)
		{
			if (timing)
				timer2.Start();

			int uPatches = 400*baseResMult;
			int vPatches = 400*baseResMult;
//			int uPatches = (face->surface->uPoints+face->surface->uOrder)*baseResMult;
//			int vPatches = (face->surface->vPoints+face->surface->vOrder)*baseResMult;
			int minPoints = 4;
			int maxPoints = 2048;
			if (uPatches < minPoints)
				uPatches = minPoints;
			if (vPatches < minPoints)
				vPatches = minPoints;
			if (uPatches > maxPoints)
				uPatches = maxPoints;
			if (vPatches > maxPoints)
				vPatches = maxPoints;

			if (uPatches > maxuNum)
				maxuNum = uPatches;
			if (vPatches > maxvNum)
				maxvNum = vPatches;
			totalPatches += uPatches*vPatches;
			Float4 surfMomentError;
			Float4 surfMoment = GetSurfaceMoment(face->surface, uPatches, vPatches, numQuadraturePoints, numTrimPointsPerPatch, glParam, &surfMomentError, errorAnalysis, forcedEvaluate);

			if (timing)
			{
				timer2.Stop();
				surfaceMomentTime += timer2.GetElapsedTimeInMicroSec();
			}

			moment += surfMoment;

			face->surface->moment0		= surfMoment[0];
			face->surface->moment1[0]	= surfMoment[1];
			face->surface->moment1[1]	= surfMoment[2];
			face->surface->moment1[2]	= surfMoment[3];

			if (errorAnalysis)
			{
				(*momentError) += surfMomentError;
				face->surface->moment0Error		= surfMomentError[0];
				face->surface->moment1Error[0]	= surfMomentError[1];
				face->surface->moment1Error[1]	= surfMomentError[2];
				face->surface->moment1Error[2]	= surfMomentError[3];
			}

#ifdef DUMP
			Float4 surfMomentTriangle =  GetTriangulatedMoment(face);
			if (face->surface->trimmed)
				cout << "Trimmed" << endl;
			cout << "Triangulated : " << surfMomentTriangle[0] << endl;
			cout << "NURBS        : " << surfMoment[0] << endl << endl;
#endif
		}
		else
		{
			if (timing)
				timer2.Start();

			if (!triGPUMoments)
			{
				Float4 surfMoment = GetTriangulatedMoment(face);
				moment += surfMoment;

				face->moment0		= surfMoment[0];
				face->moment1[0]	= surfMoment[1];
				face->moment1[1]	= surfMoment[2];
				face->moment1[2]	= surfMoment[3];
			}
			else
			{
				for (int k = 0; k <  face->triangles.size(); k++)
				{
					triangleVertex1Data[currentTriCount*4 + 0] = face->triangles[k][0].point[0];
					triangleVertex1Data[currentTriCount*4 + 1] = face->triangles[k][0].point[1];
					triangleVertex1Data[currentTriCount*4 + 2] = face->triangles[k][0].point[2];
					triangleVertex1Data[currentTriCount*4 + 3] = 0;
					triangleVertex2Data[currentTriCount*4 + 0] = face->triangles[k][1].point[0];
					triangleVertex2Data[currentTriCount*4 + 1] = face->triangles[k][1].point[1];
					triangleVertex2Data[currentTriCount*4 + 2] = face->triangles[k][1].point[2];
					triangleVertex2Data[currentTriCount*4 + 3] = 0;
					triangleVertex3Data[currentTriCount*4 + 0] = face->triangles[k][2].point[0];
					triangleVertex3Data[currentTriCount*4 + 1] = face->triangles[k][2].point[1];
					triangleVertex3Data[currentTriCount*4 + 2] = face->triangles[k][2].point[2];
					triangleVertex3Data[currentTriCount*4 + 3] = 0;
					currentTriCount++;
				}
			}
			if (timing)
			{
				timer2.Stop();
				triangulatedMomentTime += timer2.GetElapsedTimeInMicroSec();
			}
		}
	}

	if (triGPUMoments)
	{
		if (timing)
			timer2.Start();
		for (int k = currentTriCount; k < triangleTexSize*triangleTexSize; k++)
		{
			triangleVertex1Data[k*4 + 0] = 0;
			triangleVertex1Data[k*4 + 1] = 0;
			triangleVertex1Data[k*4 + 2] = 0;
			triangleVertex1Data[k*4 + 3] = 0;
			triangleVertex2Data[k*4 + 0] = 0;
			triangleVertex2Data[k*4 + 1] = 0;
			triangleVertex2Data[k*4 + 2] = 0;
			triangleVertex2Data[k*4 + 3] = 0;
			triangleVertex3Data[k*4 + 0] = 0;
			triangleVertex3Data[k*4 + 1] = 0;
			triangleVertex3Data[k*4 + 2] = 0;
			triangleVertex3Data[k*4 + 3] = 0;
		}

		if (currentTriCount > 0)
			moment += GetGPUTriangulatedMoment(triangleVertex1Data, triangleVertex2Data, triangleVertex3Data, numTriangles, triangleTexSize, glParam);;
		//	Float4 moment2 = GetGPUTriangulatedMoment(triangleVertex1Data, triangleVertex2Data, triangleVertex3Data, numTriangles, triangleTexSize, glParam);;

		delete [] triangleVertex1Data;
		delete [] triangleVertex2Data;
		delete [] triangleVertex3Data;
		if (timing)
		{
			timer2.Stop();
			triangulatedMomentTime += timer2.GetElapsedTimeInMicroSec();
		}
	}

	if (timing)
	{
		timer1.Stop();

		cout<< endl;
		cout<<"Total Time          : "<<(0.001*timer1.GetElapsedTimeInMicroSec())<<endl;
		cout<<"Surface Time        : "<<(0.001*surfaceMomentTime)<<endl;
		cout<<"Triangle Time       : "<<(0.001*triangulatedMomentTime)<<endl<<endl;
		cout<<"Moment              : "<< moment[0] << endl;
		cout<<"ACIS Moment         : "<< object->ACISVolume << endl;
		cout<<"Ratio               : "<< fabs(moment[0])/object->ACISVolume << endl;
		cout<<"Error               : "<< (*momentError)[0]/moment[0] << endl;
		cout<<"Size                : "<< maxuNum << "x" << maxvNum << endl;
		cout<<"Total Patches       : "<< totalPatches << endl;
	}
	return moment;

}

Float4 UpdateObjectMoment(Object* object, int faceNum, Float4* momentError, GLParameters* glParam)
{
	int baseResMult = 2;
	int numQuadraturePoints = 3;
	int numTrimPointsPerPatch = 4;
	bool errorAnalysis = false;
	bool timing = false;

	clock_t beginMomentTime, beginSurfaceMomentTime, beginTriangulatedMomentTime;
	unsigned int surfaceMomentTime = 0, triangulatedMomentTime = 0;
	int maxuNum = 0, maxvNum = 0;
	Float4 moment = Float4(0, 0, 0, 0);
	(*momentError) = Float4(0, 0, 0, 0);

	if (timing)
		beginMomentTime = clock();

	for (int p = 0; p < object->faces.size(); p++)
	{
		Face* face = object->faces[p];
		if (face->isNURBS && faceNum == p)
		{
			if (timing)
				beginSurfaceMomentTime = clock();

//			int uPatches = 50*baseResMult;
//			int vPatches = 50*baseResMult;
			int uPatches = (face->surface->uPoints+face->surface->uOrder)*baseResMult;
			int vPatches = (face->surface->vPoints+face->surface->vOrder)*baseResMult;
			if (uPatches < 8)
				uPatches = 8;
			if (vPatches < 8)
				vPatches = 8;
			if (uPatches > 512)
				uPatches = 512;
			if (vPatches > 512)
				vPatches = 512;

			if (uPatches > maxuNum)
				maxuNum = uPatches;
			if (vPatches > maxvNum)
				maxvNum = vPatches;
			Float4 surfMomentError;
			Float4 surfMoment = GetSurfaceMoment(face->surface, uPatches, vPatches, numQuadraturePoints, numTrimPointsPerPatch, glParam, &surfMomentError, errorAnalysis, false);

			if (timing)
				surfaceMomentTime += clock() - beginSurfaceMomentTime;
 
			moment += surfMoment;

			face->surface->moment0		= surfMoment[0];
			face->surface->moment1[0]	= surfMoment[1];
			face->surface->moment1[1]	= surfMoment[2];
			face->surface->moment1[2]	= surfMoment[3];

			if (errorAnalysis)
			{
				(*momentError) += surfMomentError;
				face->surface->moment0Error		= surfMomentError[0];
				face->surface->moment1Error[0]	= surfMomentError[1];
				face->surface->moment1Error[1]	= surfMomentError[2];
				face->surface->moment1Error[2]	= surfMomentError[3];
			}

#ifdef DUMP
			Float4 surfMomentTriangle =  GetTriangulatedMoment(face);
			if (face->surface->trimmed)
				cout << "Trimmed" << endl;
			cout << "Triangulated : " << surfMomentTriangle[0] << endl;
			cout << "NURBS        : " << surfMoment[0] << endl << endl;
#endif
		}
		else
		{
			if (face->isNURBS) 
			{
				Float4 surfMoment;
				surfMoment[0]	= face->surface->moment0;
				surfMoment[1]	= face->surface->moment1[0];
				surfMoment[2]	= face->surface->moment1[1];
				surfMoment[3]	= face->surface->moment1[2];
				moment += surfMoment;
			}
			else
			{
				Float4 surfMoment;
				surfMoment[0]	= face->moment0;
				surfMoment[1]	= face->moment1[0];
				surfMoment[2]	= face->moment1[1];
				surfMoment[3]	= face->moment1[2];
				moment += surfMoment;
			}
		}
	}

	if (timing)
	{
		clock_t endTime = clock();

		cout<<"Moment               : "<< moment[0] << endl;
		cout<<"Error                : "<< (*momentError)[0]/moment[0] << endl;
		cout<<"ACIS Moment          : "<< object->ACISVolume << endl;
		cout<<"Ratio                : "<< fabs(moment[0])/object->ACISVolume << endl;
		cout<<"Size                 : "<< maxuNum << "x" << maxvNum << endl;
		cout<<"Total Time           : "<<(0.001*(endTime - beginMomentTime))<<endl;
		cout<<"Surface Time         : "<<(0.001*(surfaceMomentTime))<<endl;
	}
	return moment;
}

