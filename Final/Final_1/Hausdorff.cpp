#include "Hausdorff.h"
#include "GPUUtilities.h"

// Hausdorff Distance Computation
bool Index2SortPredicate(Index2& d1, Index2& d2)
{
	return d1 < d2;
}

float HausdorffData(NURBS* surface1, NURBS* surface2, int uIndex1, int vIndex1, int uIndex2, int vIndex2, Float3 *point1, Float3 *point2)
{
	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;

	Float3 A[4];
	Float3 B[4];

	A[0][0] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[0][1] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[0][2] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];

	A[1][0] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[1][1] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[1][2] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];

	A[2][0] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[2][1] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[2][2] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];

	A[3][0] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[3][1] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[3][2] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];

	B[0][0] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[0][1] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[0][2] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	
	B[1][0] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[1][1] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[1][2] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];

	B[2][0] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[2][1] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[2][2] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];

	B[3][0] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[3][1] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[3][2] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];

	float hausdorffDist = -1;
	for (int j = 0; j < 4; j++)
	{
		float minDist = -1;
		Float3 tempPoint1, tempPoint2;
		for (int i = 0; i < 4; i++)
		{
			float dist = Distance(A[i],B[j]);

/*			int uIndex1add, vIndex1add, uIndex2add, vIndex2add;
			vIndex1add = (i/2.0);
			vIndex2add = (j/2.0);

			if (i==0)
				uIndex1add = 0;
			else if (i==1)
				uIndex1add = 1;
			else if (i==2)
				uIndex1add = 1;
			else if (i==3)
				uIndex1add = 0;

			if (j==0)
				uIndex2add = 0;
			else if (j==1)
				uIndex2add = 1;
			else if (j==2)
				uIndex2add = 1;
			else if (j==3)
				uIndex2add = 0;

			Float2 tempUV1 = Float2(uIndex1 + uIndex1add, vIndex1 + vIndex1add);
			Float2 tempUV2 = Float2(uIndex2 + uIndex2add, vIndex2 + vIndex2add);
*/
//			if ((dist < minDist || minDist == -1) && surface1->TrimInOut(tempUV1[0], tempUV1[1], uNum, vNum) && surface2->TrimInOut(tempUV2[0], tempUV2[1], uNum, vNum) )
			if ((dist < minDist || minDist == -1))
			{
				tempPoint1 = A[i];
				tempPoint2 = B[j];
				minDist = dist;
			}
		}
		if (minDist > hausdorffDist || hausdorffDist == -1)
		{
			*point1 = tempPoint1;
			*point2 = tempPoint2;
			hausdorffDist = minDist;
		}
	}

//	*point1 = (A[0] + A[1] + A[2] + A[3])/4.0;
//	*point2 = (B[0] + B[1] + B[2] + B[3])/4.0;

	return hausdorffDist;
}

float GPUHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, GLParameters* glParam)
{
	bool operationFailed = false;
	bool timing = false;
	int startingLevel = 2;
	clock_t beginTime, beginFindTime, beginDistTime, distTime, findTime1, findTime2, beginFinalCalcTime, finalCalcTime, totalTime;
	if (timing)
	{
		beginTime = clock();
		distTime  = 0;
		findTime1 = 0;
		findTime2 = 0;
	}

	// Set up GPU computations
	StartGPUComputation(glParam);

	//Set up for evaluation
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ZERO);

	vector<Float4> referenceList;
	referenceList.clear();
	Float4 initialReference = Float4(0,0,0,0);
	referenceList.push_back(initialReference);
	
	vector<float> rowNumberList;
	rowNumberList.clear();
	float initialRow = 0;
	rowNumberList.push_back(initialRow);

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;
	int maxLevel = GetExponent2(uNum);
	
	GLuint tempTex[7];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	//0 - Min/Max Texture
	//1 - Reference List Texture
	//2 - Row number texture
	//3 - MinMax distances per row
	//4 - Boxes that get selected in a pass after first filter
	//5 - First filter per row
	//6 - Boxes that get selected in a pass after second filter
	glGenTextures(7, tempTex);

	//Enable surface generation program
	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);


	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[6]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

	int numOverlaps = 1;
	float hausdorffMinVal = 0;
	for (int level = 1; level <= maxLevel && !operationFailed; level++)
	{
		if (timing)
			beginDistTime = clock();

		int levelOriginX = int((pow(2.0,maxLevel-level)-1)*pow(2.0,level+1));
		int referenceListSize = ceil(sqrt(referenceList.size()*1.0));

		int distanceArraySize = referenceListSize*4;

		if (distanceArraySize < 4000)
		{
			// Create a texture that is a power of two texture
			float* referenceListData = new float[referenceListSize*referenceListSize*4];
			float* rowNumbersData = new float[referenceListSize*referenceListSize];
			for (int j = 0; j < referenceListSize; j++)
			{
				for (int i = 0; i < referenceListSize; i++)
				{
					if (j*referenceListSize + i < referenceList.size())
					{
						referenceListData[(j*referenceListSize + i)*4 + 0] = referenceList[j*referenceListSize + i][0];
						referenceListData[(j*referenceListSize + i)*4 + 1] = referenceList[j*referenceListSize + i][1];
						referenceListData[(j*referenceListSize + i)*4 + 2] = referenceList[j*referenceListSize + i][2];
						referenceListData[(j*referenceListSize + i)*4 + 3] = referenceList[j*referenceListSize + i][3];
						rowNumbersData[j*referenceListSize+i]	= rowNumberList[j*referenceListSize + i];
					}
					else
					{
						referenceListData[(j*referenceListSize + i)*4 + 0] = -1;
						referenceListData[(j*referenceListSize + i)*4 + 1] = -1;
						referenceListData[(j*referenceListSize + i)*4 + 2] = -1;
						referenceListData[(j*referenceListSize + i)*4 + 3] = -1;
						rowNumbersData[j*referenceListSize+i]	= -1;
					}
				}
			}

			InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, distanceArraySize, distanceArraySize, GL_RGBA);
			InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, referenceListSize, referenceListSize, GL_RGBA, referenceListData);
			InitTexture(tempTex+2, LUMINANCE_FLOAT_FORMAT, referenceListSize, referenceListSize, GL_LUMINANCE, rowNumbersData);

			cgGLBindProgram(glParam->bBoxHausdorffDistProgram);

			cgGLEnableTextureParameter(glParam->bBoxHausdorffDistMinATextureParam);
    		cgGLEnableTextureParameter(glParam->bBoxHausdorffDistMaxATextureParam);
			cgGLEnableTextureParameter(glParam->bBoxHausdorffDistMinBTextureParam);
			cgGLEnableTextureParameter(glParam->bBoxHausdorffDistMaxBTextureParam);

			cgGLSetTextureParameter(glParam->bBoxHausdorffDistMinATextureParam, surface1->evalParams->bBoxTextures[0]);
			cgGLSetTextureParameter(glParam->bBoxHausdorffDistMaxATextureParam, surface1->evalParams->bBoxTextures[1]);
			cgGLSetTextureParameter(glParam->bBoxHausdorffDistMinBTextureParam, surface2->evalParams->bBoxTextures[0]);
			cgGLSetTextureParameter(glParam->bBoxHausdorffDistMaxBTextureParam, surface2->evalParams->bBoxTextures[1]);

			cgGLSetTextureParameter(glParam->bBoxHausdorffDistReferenceListTextureParam, tempTex[1]);
			cgGLSetTextureParameter(glParam->bBoxHausdorffDistRowNumberTextureParam, tempTex[2]);

			cgGLEnableTextureParameter(glParam->bBoxHausdorffDistReferenceListTextureParam);
			cgGLEnableTextureParameter(glParam->bBoxHausdorffDistRowNumberTextureParam);

			cgGLSetParameter1f(glParam->bBoxHausdorffDistLevelOriginParam, levelOriginX);

			// Bind the src textures
			glActiveTextureARB(GL_TEXTURE5_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[1]);
			glActiveTextureARB(GL_TEXTURE4_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[0]);
			glActiveTextureARB(GL_TEXTURE3_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[1]);
			glActiveTextureARB(GL_TEXTURE2_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[0]);
			glActiveTextureARB(GL_TEXTURE1_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);

			glViewport(0, 0, distanceArraySize, distanceArraySize);

			// Specify the dst texture
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

			glClearColor(1e5,1e5,-1,-1);
			glClear(GL_COLOR_BUFFER_BIT);
			glFlush();
		
			DrawQuad(distanceArraySize, distanceArraySize);
			glFlush();
			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[distanceArraySize*distanceArraySize*4];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0, 0, distanceArraySize, distanceArraySize, GL_RGBA, GL_FLOAT, retVal);
			WriteToFile(retVal, distanceArraySize, distanceArraySize, 0);
			WriteToFile(retVal, distanceArraySize, distanceArraySize, 1);
			WriteToFileInt(retVal, distanceArraySize, distanceArraySize, 2);
#endif

			cgGLDisableTextureParameter(glParam->bBoxHausdorffDistMinATextureParam);
			cgGLDisableTextureParameter(glParam->bBoxHausdorffDistMaxATextureParam);
			cgGLDisableTextureParameter(glParam->bBoxHausdorffDistMinBTextureParam);
			cgGLDisableTextureParameter(glParam->bBoxHausdorffDistMaxBTextureParam);
			cgGLDisableTextureParameter(glParam->bBoxHausdorffDistReferenceListTextureParam);
			cgGLDisableTextureParameter(glParam->bBoxHausdorffDistRowNumberTextureParam);

			// Find the minimum max extent of each row and minimum min extent of each row
			if (timing)
			{
				beginFindTime = clock();
				distTime += (beginFindTime - beginDistTime);
			}

			int minMinTextureSize = int(pow(2.0, level));
			Float4 hausdorffVal = FindRowMinMaxBBoxExtent(tempTex[0], tempTex[3], distanceArraySize, minMinTextureSize, glParam);
			hausdorffMinVal = hausdorffVal[0];

			if (timing)
				findTime1 += (clock() - beginFindTime);
			if (timing)
				beginFindTime = clock();
			
			//Apply the filter
			{
				// Apply the first filter
				cgGLBindProgram(glParam->bBoxHausdorffFilter1Program);

				InitTexture(tempTex+4, RGBA_FLOAT_FORMAT, distanceArraySize, distanceArraySize, GL_RGBA);

				glActiveTextureARB(GL_TEXTURE0_ARB);
				glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

				cgGLSetTextureParameter(glParam->bBoxHausdorffFilter1MinMaxTextureParam, tempTex[0]);
				cgGLEnableTextureParameter(glParam->bBoxHausdorffFilter1MinMaxTextureParam);

				cgGLSetParameter1f(glParam->bBoxHausdorffFilter1MinValParam, hausdorffMinVal);

				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[4], 0);
				glViewport(0, 0, distanceArraySize, distanceArraySize);

				glClearColor(0,0,0,0);
				glClear(GL_COLOR_BUFFER_BIT);

				DrawQuad(distanceArraySize, distanceArraySize);
				glFlush();
				glFinish();

#ifdef DUMP
				GLfloat* retVal=new GLfloat[distanceArraySize*distanceArraySize];
				glReadBuffer(GL_COLOR_AiTTACHMENT0_EXT);
				glReadPixels(0, 0, distanceArraySize, distanceArraySize, GL_RED, GL_FLOAT, retVal);
				WriteToFile(retVal, distanceArraySize, distanceArraySize);
#endif
				cgGLDisableTextureParameter(glParam->bBoxHausdorffFilter1MinMaxTextureParam);

				// Reduce the output texture to cull all values of rows that get culled
				RowFilterTexture(tempTex[4], tempTex[5], distanceArraySize, minMinTextureSize, glParam);

				// Apply second filter using both row texture and row filter
				cgGLBindProgram(glParam->bBoxHausdorffFilter2Program);

				glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[6]);
				glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, distanceArraySize, distanceArraySize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

				glActiveTextureARB(GL_TEXTURE0_ARB);
				glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

				cgGLSetTextureParameter(glParam->bBoxHausdorffFilter2MinMaxTextureParam, tempTex[0]);
				cgGLEnableTextureParameter(glParam->bBoxHausdorffFilter2MinMaxTextureParam);

				cgGLSetTextureParameter(glParam->bBoxHausdorffFilter2MinRowTextureParam, tempTex[3]);
				cgGLEnableTextureParameter(glParam->bBoxHausdorffFilter2MinRowTextureParam);

				cgGLSetTextureParameter(glParam->bBoxHausdorffFilter2FilterRowTextureParam, tempTex[5]);
				cgGLEnableTextureParameter(glParam->bBoxHausdorffFilter2FilterRowTextureParam);

				cgGLSetParameter1f(glParam->bBoxHausdorffFilter2TexSizeParam, minMinTextureSize);
		
				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[6], 0);
				glViewport(0, 0, distanceArraySize, distanceArraySize);

				glClearColor(0,0,0,0);
				glClear(GL_COLOR_BUFFER_BIT);

				DrawQuad(distanceArraySize, distanceArraySize);
				glFlush();
				glFinish();

#ifdef DUMP
				GLfloat* retVal=new GLfloat[distanceArraySize*distanceArraySize];
				glReadBuffer(GL_COLOR_AiTTACHMENT0_EXT);
				glReadPixels(0, 0, distanceArraySize, distanceArraySize, GL_RED, GL_FLOAT, retVal);
				WriteToFile(retVal, distanceArraySize, distanceArraySize);
#endif
				cgGLDisableTextureParameter(glParam->bBoxHausdorffFilter2MinMaxTextureParam);
				cgGLDisableTextureParameter(glParam->bBoxHausdorffFilter2MinRowTextureParam);
				cgGLDisableTextureParameter(glParam->bBoxHausdorffFilter2FilterRowTextureParam);
			}


			vector<int> positions;
			FindRenderLocation(tempTex[6], &numOverlaps, distanceArraySize, distanceArraySize, &positions, glParam);

			if (timing)
				findTime2 += (clock() - beginFindTime);

			numOverlaps = positions.size();
			vector<Float4> newReferenceList;
			vector<float> newRowNumberList;
			
			if (timing)
				cout <<"Level : " << level << " Pairs : " << numOverlaps << endl;

			for (int overlap = 0; overlap < numOverlaps; overlap++)
			{
				int pos = positions[overlap];
				int xPos = pos%distanceArraySize;
				int yPos = int(pos/distanceArraySize);
				int referenceListLocationX = int(xPos/4.0);
				int referenceListLocationY = int(yPos/4.0);
				int boxLocationAx = ((xPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][0]))*2;
				int boxLocationAy = (int((xPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][1]))*2;
				int boxLocationBx = ((yPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][2]))*2;
				int boxLocationBy = (int((yPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][3]))*2;
				Float4 refCoords = Float4(boxLocationAx, boxLocationAy, boxLocationBx, boxLocationBy);
				int newRowNumber = int(rowNumberList[referenceListLocationY*referenceListSize + referenceListLocationX])*4 + (yPos%4);
				newReferenceList.push_back(refCoords);
				newRowNumberList.push_back(float(newRowNumber));
			}
			referenceList.clear();
			referenceList = newReferenceList;
			delete[] referenceListData;

			rowNumberList.clear();
			rowNumberList = newRowNumberList;
			delete[] rowNumbersData;
		}// For if >4096
		else
			operationFailed = true;
	}
	// For each level loop ends

	float hausdorffDistance = -1;
	if (!operationFailed)
	{
		if (timing)
			beginFinalCalcTime = clock();
	
		Float3 hausdorffSurf1Point, hausdorffSurf2Point;
		float* rowMinDistances = new float[uNum*vNum];
		Float3* surf1Points = new Float3[uNum*vNum];
		Float3* surf2Points = new Float3[uNum*vNum];
		for (int k = 0; k < uNum*vNum; k++)
			rowMinDistances[k] = -1;

		for (int k = 0; k < referenceList.size(); k++)
		{
			Float3 surf1Point;
			Float3 surf2Point;

			int uIndex1 = (int) referenceList[k][0]/2.0;
			int vIndex1 = (int) referenceList[k][1]/2.0;
			int uIndex2 = (int) referenceList[k][2]/2.0;
			int vIndex2 = (int) referenceList[k][3]/2.0;

			int indexVal1 = vIndex1*uNum + uIndex1;
			int indexVal2 = vIndex2*uNum + uIndex2;

			float tempHausdorffDist = -1;
			if (glParam->readBack)
				tempHausdorffDist = HausdorffData(surface1, surface2, uIndex1, vIndex1, uIndex2, vIndex2, &surf1Point, &surf2Point);
			else
			{
				float uMin1 = float(uIndex1*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
				float vMin1 = float(vIndex1*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);
				float uMax1 = float((uIndex1+1)*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
				float vMax1 = float((vIndex1+1)*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);
			
				float uMin2 = float(uIndex2*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
				float vMin2 = float(vIndex2*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);
				float uMax2 = float((uIndex2+1)*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
				float vMax2 = float((vIndex2+1)*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);

				surf1Point = surface1->Evaluate((uMin1+uMax1)*.5,(vMin1+vMax1)*.5);
				surf2Point = surface2->Evaluate((uMin2+uMax2)*.5,(vMin2+vMax2)*.5);
				tempHausdorffDist = Distance(surf1Point, surf2Point);
			}
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

	glDeleteTextures(7, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (timing)
	{
		finalCalcTime  = clock() - beginFinalCalcTime;
		totalTime = clock() - beginTime;
		cout << "Traversal Total Time : " << totalTime*0.001 << endl;
		cout << "Distance Time        : " << distTime*0.001 << endl;
		cout << "Reduction Time       : " << findTime1*0.001 << endl;
		cout << "BBox Filter Time     : " << findTime2*0.001 << endl;
		cout << "Trianguled Time      : " << finalCalcTime*0.001 << endl << endl;
	}
	if (!operationFailed)
		return hausdorffDistance;
	else
		return -1;
}

float GetSurfaceHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, int intRes, GLParameters* glParam, bool forcedEvaluate)
{
	glParam->hausdorff = true;
	int uNum = intRes;
	int vNum = intRes;
//	int uNum = 32;
//	int vNum = 32;
	bool timing = false;
	long unsigned int evaluationTime = 0;
	long unsigned int bBoxTime = 0;
	long unsigned int hausdorffTime = 0;
	clock_t beginTime = 0;
	clock_t beginEvalTime = 0;
	clock_t beginBBoxTime = 0;
	clock_t beginHausdorffTime = 0;

	if (timing)
		beginTime = clock();

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

	if (timing)
		beginEvalTime = clock();
	if (evaluate)
	{
		if (glParam->readBack)
		{
			surface1->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
			surface2->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
		}
		surface1->SurfaceEvaluation(glParam, false, false);
		surface2->SurfaceEvaluation(glParam, false, false);
/*		if (surface1->trimmed)
		{
			surface1->evalParams->trimData = new float[(uNum+1)*(vNum+1)];
			surface1->GenerateTrimData(uNum, vNum, glParam);
		}
		if (surface2->trimmed)
		{
			surface2->evalParams->trimData = new float[(uNum+1)*(vNum+1)];
			surface2->GenerateTrimData(uNum, vNum, glParam);
		}
*/	}
	if (timing)
	{
		beginBBoxTime = clock();
		evaluationTime += beginBBoxTime - beginEvalTime;
	}
	if (evaluate || !surface1->identityTransformation)
	{
//		if (surface1->trimmed)
//			surface1->SurfaceBoundingBoxTrimmedEvaluation(glParam);
//		else
			surface1->SurfaceBoundingBoxEvaluation(glParam);
	}
	if (evaluate || !surface2->identityTransformation)
	{
//		if (surface2->trimmed)
//			surface2->SurfaceBoundingBoxTrimmedEvaluation(glParam);
//		else
			surface2->SurfaceBoundingBoxEvaluation(glParam);
	}
	if (timing)
	{
		beginHausdorffTime = clock();
		bBoxTime += beginHausdorffTime - beginBBoxTime;
	}

	float hausdorffDist = GPUHausdorffDistance(surface1, surface2, point1, point2, glParam);

	if (timing)
		hausdorffTime += clock() - beginHausdorffTime;

	glParam->hausdorff = false;
	if (timing)
	{
		clock_t endTime = clock();
		cout<<"Hausdorff Time    : "<<(0.001*(endTime - beginTime))<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationTime)<<endl;
		cout<<"BBox Time         : "<<(0.001*bBoxTime)<<endl;
		cout<<"Traversal Time    : "<<(0.001*hausdorffTime)<<endl<<endl;
	}
	return hausdorffDist;
}



