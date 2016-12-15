#include "Object.h"
#include "GPUUtilities.h"

// copy constructor, *this = that
Object::Object(const Object &that)
{
}

// overloaded = operator
Object &Object::operator=(const Object &that)
{
	return *this;
}

// load an object from obj file fname
void Object::ReadObject2(char *fname)
{
	ifstream in(fname, ios::in);
	char c;
	Float3 pt;
	Float3 n(0, 0, 0);
	Float2 tx;
	Triangle t;
	Index3 vertexIndex;

	vector<Float2> tempTexture;
	vector<Vertex> tempVertices;

	if (!in.good())
	{
		cerr << "Unable to open file \"" << fname << "\"" << endl;
		abort();
	}

	while (in.good())
	{
		in >> c;
		if (!in.good()) break;
		if (c == 'v')
		{
			in >> pt[0] >> pt[1] >> pt[2];
			Vertex vertex;
			vertex.point = pt;
			vertex.normal = n;
			tempVertices.push_back(vertex);
		}
		else if (c == 't')
		{
			faces[0]->trimmed = true;
			in >> tx[0] >> tx[1];
			tempTexture.push_back(tx);
		}
		else if (c == 'f')
		{
			in >> vertexIndex[0] >> vertexIndex[1] >> vertexIndex[2];
			vertexIndex[0] -= 1; vertexIndex[1] -= 1; vertexIndex[2] -= 1;
			t.vertexIndex = vertexIndex;
			t[0] = tempVertices[vertexIndex[0]];
			t[1] = tempVertices[vertexIndex[1]];
			t[2] = tempVertices[vertexIndex[2]];
			if (faces[0]->trimmed)
			{
				t[0].texCoords = tempTexture[vertexIndex[0]];
				t[1].texCoords = tempTexture[vertexIndex[1]];
				t[2].texCoords = tempTexture[vertexIndex[2]];
			}

			Float3 side1 = t[1].point - t[0].point;
			Float3 side2 = t[2].point - t[0].point;
			t.faceNormal = VectorCrossProduct(side1, side2);
			double area = 0.5*VectorMagnitude(t.faceNormal);
			VectorNormalize(t.faceNormal);
			tempVertices[vertexIndex[0]].normal += area*t.faceNormal;
			tempVertices[vertexIndex[1]].normal += area*t.faceNormal;
			tempVertices[vertexIndex[2]].normal += area*t.faceNormal;

			faces[0]->triangles.push_back(t);
		}
	}

	for (int j = 0; j < tempVertices.size(); j++)
		VectorNormalize(tempVertices[j].normal);
	if (faces[0]->trimmed)
	{
		faces[0]->trimWidth = 256;
		faces[0]->trimHeight = 256;
	}

	tempTexture.clear();
	tempVertices.clear();
}

int Face::GetCommonFace(int vertexIndex1, int vertexIndex2, int face)
{
	int commonFaceIndex = -1;
	for (int p = 0; p < this->vertexFaces[vertexIndex1].size(); p++)
	{
		int faceIndex1 = this->vertexFaces[vertexIndex1][p];
		if (p != face)
		{
			for (int q = 0; q < this->vertexFaces[vertexIndex2].size(); q++)
			{
				int faceIndex2 = this->vertexFaces[vertexIndex2][q];
				if (faceIndex1 == faceIndex2)
					commonFaceIndex = faceIndex1;
			}
		}
	}
	return commonFaceIndex;
}

void Object::ReadObject(char *fname)
{
	ifstream in(fname, ios::in);
	string firstword;
	string comment;
	Float3 pt;
	Float3 n(0, 0, 0);
	Float2 tx;
	Triangle t;
	Index3 vertexIndex;
	int vertexNum = 0;

	vector<Vertex> tempVertices;
	vector<Float2> tempTexture;

	if (!in.good())
	{
		cerr << "Unable to open file \"" << fname << "\"" << endl;
		abort();
	}

	while (in.good())
	{
		in >> firstword;
		if (!in.good()) break;

		if (firstword[0] == 'v' && firstword.length() == 1)
		{
			in >> pt[0] >> pt[1] >> pt[2];
			Vertex vertex;
			vertex.point = pt;
			vertex.normal = n;
			tempVertices.push_back(vertex);
			vector<int> faceIndices;
			this->faces[0]->vertexFaces.push_back(faceIndices);
			if (vertexNum == 0)
			{
				this->faces[0]->bBoxMin = vertex.point;
				this->faces[0]->bBoxMax = vertex.point;
			}
			else
			{
				this->faces[0]->bBoxMin = MinFloat3(this->faces[0]->bBoxMin, vertex.point);
				this->faces[0]->bBoxMax = MaxFloat3(this->faces[0]->bBoxMax, vertex.point);
			}
			vertexNum++;
		}
		else if (firstword[0] == 'v' && firstword[1] == 'n')
		{
			in >> n[0] >> n[1] >> n[2];
			//tempNormals.push_back(n);
		}
		else if (firstword[0] == 'v' && firstword[1] == 't')
		{
			faces[0]->trimmed = true;
			in >> tx[0] >> tx[1];
			tempTexture.push_back(tx);
		}
		else if (firstword[0] == 'f')
		{
			char tempChar;
			int tempNormal;
			in >> vertexIndex[0];
			in >> vertexIndex[1];
			in >> vertexIndex[2];
			//in >> vertexIndex[0] >> tempChar >> tempChar >> tempNormal;
			//in >> vertexIndex[1] >> tempChar >> tempChar >> tempNormal;
			//in >> vertexIndex[2] >> tempChar >> tempChar >> tempNormal;
			vertexIndex[0] -= 1; vertexIndex[1] -= 1; vertexIndex[2] -= 1;
			t.vertexIndex = vertexIndex;
			t[0] = tempVertices[vertexIndex[0]];
			t[1] = tempVertices[vertexIndex[1]];
			t[2] = tempVertices[vertexIndex[2]];
			if (faces[0]->trimmed)
			{
				t[0].texCoords = tempTexture[vertexIndex[0]];
				t[1].texCoords = tempTexture[vertexIndex[1]];
				t[2].texCoords = tempTexture[vertexIndex[2]];
			}

			Float3 side1 = t[1].point - t[0].point;
			Float3 side2 = t[2].point - t[0].point;
			t.faceNormal = VectorCrossProduct(side1, side2);
			double area = 0.5*VectorMagnitude(t.faceNormal);
			VectorNormalize(t.faceNormal);
			tempVertices[vertexIndex[0]].normal += area*t.faceNormal;
			tempVertices[vertexIndex[1]].normal += area*t.faceNormal;
			tempVertices[vertexIndex[2]].normal += area*t.faceNormal;

			int triangleNum = faces[0]->triangles.size();
			faces[0]->vertexFaces[vertexIndex[0]].push_back(triangleNum);
			faces[0]->vertexFaces[vertexIndex[1]].push_back(triangleNum);
			faces[0]->vertexFaces[vertexIndex[2]].push_back(triangleNum);
			faces[0]->triangles.push_back(t);

		}
	}

	for (int j = 0; j < tempVertices.size(); j++)
		VectorNormalize(tempVertices[j].normal);
	for (int k = 0; k < faces[0]->triangles.size(); k++)
	{
		Index3 vertexIndex = faces[0]->triangles[k].vertexIndex;
		faces[0]->triangles[k][0].normal = tempVertices[vertexIndex[0]].normal;
		faces[0]->triangles[k][1].normal = tempVertices[vertexIndex[1]].normal;
		faces[0]->triangles[k][2].normal = tempVertices[vertexIndex[2]].normal;

		faces[0]->triangles[k].adjacentFaceIndex[0] = faces[0]->GetCommonFace(vertexIndex[0], vertexIndex[1], k);
		faces[0]->triangles[k].adjacentFaceIndex[1] = faces[0]->GetCommonFace(vertexIndex[1], vertexIndex[2], k);
		faces[0]->triangles[k].adjacentFaceIndex[0] = faces[0]->GetCommonFace(vertexIndex[2], vertexIndex[0], k);
	}

	if (faces[0]->trimmed)
	{
		faces[0]->trimWidth = 256;
		faces[0]->trimHeight = 256;
	}

	faces[0]->CalculateStructureTensor();
	tempVertices.clear();
	tempTexture.clear();

	double modelSize = VectorMagnitude(this->faces[0]->bBoxMax - this->faces[0]->bBoxMin);
	float offset = 0.001*modelSize;

	this->faces[0]->bBoxMax += Float3(offset, offset, offset);
	this->faces[0]->bBoxMin -= Float3(offset, offset, offset);

	this->bBoxMax = this->faces[0]->bBoxMax;
	this->bBoxMin = this->faces[0]->bBoxMin;
	this->maxModelSize = __max((this->bBoxMax[0] - this->bBoxMin[0]), __max((this->bBoxMax[1] - this->bBoxMin[1]), (this->bBoxMax[2] - this->bBoxMin[2])));
}

void Object::CreateDisplayLists(GLParameters* glParam)
{
	for (int i = 0; i < this->faces.size(); i++)
	{
		this->faces[i]->dlid = glGenLists(1);
		glNewList(this->faces[i]->dlid, GL_COMPILE);
		this->faces[i]->DrawFace(glParam, false, 1.0);
		glEndList();
	}
}

void Object::CreateDisplayListObject(GLParameters* glParam)
{
	this->dlid = glGenLists(1);
	glNewList(this->dlid, GL_COMPILE);
	for (int i = 0; i < this->faces.size(); i++)
		this->faces[i]->DrawFace(glParam, false, 1.0);
	glEndList();
}

void Object::DrawSceneObject(GLParameters* glParam, bool computeVisibility, float transparency)
{
	// This is where you call apply transformations remember to push and pop your matrices
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);
	this->DrawObject(glParam, computeVisibility, transparency);

	if (this->massCenterComputed)
		this->DrawCOM();
	glPopMatrix();
}

void Object::DrawObject(GLParameters* glParam, bool computeVisibility, float transparency)
{
	for (int i = 0; i < this->faces.size(); i++)
	{
		if (computeVisibility)
		{
			glPushAttrib(GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);
			glBeginQueryARB(GL_SAMPLES_PASSED_ARB, glParam->occlusionQuery);
		}

#ifdef DISPLAYLISTS
		if ((this->faces[i]->visibilityFactor >= glParam->surfVisibilityCutOff && this->visibilityFactor >= glParam->surfVisibilityCutOff) || computeVisibility)
		{
			if (this->faces[i]->isNURBS && glParam->displayNURBS)
				this->faces[i]->surface->DrawNURBS(glParam, transparency);
			else if (transparency != 1)
				this->faces[i]->DrawFace(glParam, computeVisibility, transparency);
			else
				glCallList(this->faces[i]->dlid);
		}
#else
		if ( (this->faces[i]->visibilityFactor >= glParam->surfVisibilityCutOff && this->visibilityFactor >= glParam->surfVisibilityCutOff) ||  computeVisibility)
		{
			if (this->faces[i]->isNURBS && glParam->displayNURBS)
				this->faces[i]->surface->DrawNURBS(glParam, transparency);
			else
				this->faces[i]->DrawFace(glParam, computeVisibility, transparency);
		}
#endif
		if (computeVisibility)
		{
			glEndQueryARB(GL_SAMPLES_PASSED_ARB);
			GLuint fragmentCount;
			glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &fragmentCount);
			float coverageRatio = fragmentCount;
			this->faces[i]->visibilityFactor += coverageRatio;
			glPopAttrib();
		}
	}
}

void Object::ClassifyInOut(GLParameters* glParam)
{
	bool timing = true;
	clock_t initialTime, initTime, sliceTime, totalTime;
	if (timing)
		initialTime = clock();

	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	float gridSizeZ = this->voxelData->gridSizeZ;
	inOutDType* level1InOut = this->voxelData->level1InOut;

	//Set up texture
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	float viewMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(this->bBoxMin[0], this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[1], this->bBoxMin[2] - 200, this->bBoxMax[2] + 200);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
	glDisable(GL_CULL_FACE);

	glViewport(0, 0, numDivX, numDivY);

#ifdef VISUALDEBUG
	glDrawBuffer(GL_FRONT);
#else
	glEnable(GL_COLOR_ATTACHMENT0_EXT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
#endif

	//Initialize the texture
	GLuint tempTex;
	glGenTextures(1, &tempTex);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, numDivX, numDivY, 0, GL_RGBA, GL_FLOAT, 0);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);

#ifdef STENCILBUFFERMETHOD
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, numDivX, numDivY);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);

	CheckFrameBuffer();
#endif

	cgGLEnableProfile(glParam->cgFragmentProfile);
	cgGLBindProgram(glParam->passiveProgram);

	if (timing)
		initTime = clock() - initialTime;

	for (int slice = 0; slice < numDivZ; slice++)
	{
		double equation[4] = { 0, 0, -1, (slice + 0.5)*gridSizeZ + this->bBoxMin[2] };
		glClipPlane(GL_CLIP_PLANE0, equation);
		glEnable(GL_CLIP_PLANE0);

#ifdef STENCILBUFFERMETHOD
		glClearColor(0, 0, 0, 0);
		glClearStencil(0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 0x11111111);
		glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
#else
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);
		glBlendFunc(GL_ONE_MINUS_DST_COLOR,GL_ZERO);	// Set The Blending Function For Translucency
		glEnable(GL_BLEND);
		glColor4f(1,1,1,1);
#endif

#ifdef MSAA
		//glEnable(GL_MULTISAMPLE_ARB);
#endif

		this->DrawObject(glParam, false, 1.0);

		glDisable(GL_CLIP_PLANE0);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_STENCIL_TEST);

#ifdef VISUALDEBUG
		glEnable(GL_COLOR_MATERIAL);
		glStencilFunc(GL_EQUAL, 3, 0x11111111);
		Float4 size = Float4(glParam->bBoxMin[0], glParam->bBoxMax[0], glParam->bBoxMin[1], glParam->bBoxMax[1]);
		DrawPlane(equation,size);

#endif

		//		glFlush();

		// Read back
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
#ifdef STENCILBUFFERMETHOD
		//inOutDType* 
		inOutDType* readStartPointer = level1InOut + (slice*numDivX*numDivY);
		glReadPixels(0, 0, numDivX, numDivY, GL_STENCIL_INDEX, INOUTDFORMAT, readStartPointer);
#else
		glReadPixels(0, 0, numDivX, numDivY, GL_RED, GL_FLOAT, level1InOut + slice*numDivX*numDivY);
#endif

#ifdef OUTPUTTEXTFILES
		char* fileName = new char[11];
		if (slice < 10)
			sprintf(fileName, "Slice0%d.txt", slice);
		else
			sprintf(fileName, "Slice%d.txt", slice);
		WriteToFileInt(level1InOut + slice*numDivX*numDivY, fileName, numDivX, numDivY);
#endif
	}

	if (timing)
		sliceTime = clock() - initTime - initialTime;

	//	WriteToFileInt(level1InOut, numDivX*numDivY, numDivZ);

	// Unbind the Framebuffer object
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	cgGLDisableProfile(glParam->cgFragmentProfile);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);
	glDeleteTextures(1, &tempTex);

	// Restore the previous views
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();

	if (timing)
	{
		totalTime = clock() - initialTime;
		cout << numDivX << " x " << numDivY << " x " << numDivZ << " = " << numDivX*numDivY*numDivZ << endl << endl;
		cout << "Init  Time         : " << initTime*0.001 << endl;
		cout << "Slice Time         : " << sliceTime*0.001 / numDivZ << endl;
		cout << "InOut Total Time   : " << totalTime*0.001 << endl << endl;
	}
}

void Object::ClassifyInOut2xLevel2(GLParameters* glParam, int voxelIndex, int indexLocation) //Add level2 index location as input
{
	int numSamples = 2;
	bool timing = true;
	clock_t initialTime, initTime, sliceTime, totalTime;
	if (timing)
		initialTime = clock();

	int numDivX = 5;
	int numDivY = 5;
	int numDivZ = 5;
	float gridSizeZ = this->voxelData->gridSizeZ;
	inOutDType* inOutLocal= this->voxelData->level2InOut + indexLocation * 125;
	//float inOutLocal[125] = {}; // Local Array for inOut values
	//inOutDType* inOutLocal;

	if (timing)
		initTime = clock() - initialTime;

	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	float viewMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	//glOrtho(this->bBoxMin[0], this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[1], this->bBoxMin[2] - 200, this->bBoxMax[2] + 200);
	glOrtho(this->voxelData->bBox[voxelIndex].minPoint[0], this->voxelData->bBox[voxelIndex].maxPoint[0], this->voxelData->bBox[voxelIndex].minPoint[1], this->voxelData->bBox[voxelIndex].maxPoint[1], this->bBoxMin[2] - 200, this->bBoxMax[2] + 200);

	glEnable(GL_COLOR_ATTACHMENT0_EXT);

	glEnable(GL_COLOR_ATTACHMENT0_EXT);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);

	// Disable depth testing to render all pixels
	glDisable(GL_DEPTH_TEST);

	//glEnable(GL_DEPTH_TEST);
	// The Type Of Depth Testing To Do
	//glDepthFunc(GL_LESS);

	// Disable culling to render back faces
	glDisable(GL_CULL_FACE);

	cgGLEnableProfile(glParam->cgFragmentProfile);
	cgGLBindProgram(glParam->passiveProgram);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);

	//Initialize the texture
	GLuint tempTex;
	glGenTextures(1, &tempTex);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, numSamples*numDivX, numSamples*numDivY, 0, GL_RGBA, GL_FLOAT, 0);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, numSamples*numDivX, numSamples*numDivY);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);

#ifdef DEBUG
	CheckFrameBuffer();
#endif

	// Rotate object around axis to test for off axis rendering
	//glMatrixMode(GL_MODELVIEW);
	//glRotatef(10,1,0,0);

	for (int slice = 0; slice < numDivZ; slice++)
	{
		glViewport(0, 0, numSamples*numDivX, numSamples*numDivY);

		double equation[4] = { 0, 0, -1, (slice + 0.5)*gridSizeZ + this->bBoxMin[2] };

		glClipPlane(GL_CLIP_PLANE0, equation);
		glEnable(GL_CLIP_PLANE0);

		glClearColor(0, 0, 0, 0);
		glClearStencil(0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 0x11111111);
		glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);

		this->DrawObject(glParam, false, 1.0);

		glDisable(GL_CLIP_PLANE0);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_STENCIL_TEST);

		glFlush();

		// Read back
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		inOutDType* stencilData2x = new inOutDType[numDivX*numDivY*numSamples*numSamples];
		glReadPixels(0, 0, numSamples*numDivX, numSamples*numDivY, GL_STENCIL_INDEX, INOUTDFORMAT, stencilData2x);

		int count = 0; // Location in Array for InOut values

		for (int j = 0; j < numDivY; j++)
		{
			for (int i = 0; i < numDivX; i++)
			{
				int currentIndex = (slice*numDivX*numDivY) + j*numDivX + i;
				int numInsideCount = 0;
				for (int l = 0; l < numSamples; l++)
				{
					for (int m = 0; m < numSamples; m++)
					{
						inOutDType sData = stencilData2x[(numSamples*j + l)*(numSamples*numDivX) + (numSamples*i + m)];
						if (int(sData) % 2 == 1)
							numInsideCount++;
					}
				}
				if (numInsideCount > numSamples*numSamples / 2.0)
				{
					inOutLocal[currentIndex] = inOutDType(1);
					//this->voxelData->level2InOut[indexLocation + currentIndex] = inOutLocal[currentIndex];
				}
				else
				{
					inOutLocal[currentIndex] = inOutDType(0);
					//this->voxelData->level2InOut[indexLocation + currentIndex] = inOutLocal[currentIndex];
				}
			}
		}

		
		//delete[] stencilData2x;

		

#ifdef OUTPUTTEXTFILES
		char* fileName = new char[11];
		if (slice < 10)
			sprintf(fileName, "Slice0%d.txt", slice);
		else
			sprintf(fileName, "Slice%d.txt", slice);
		WriteToFileInt(level1InOut + slice*numDivX*numDivY, fileName, numDivX, numDivY);
#endif

	}

	
	// Unbind the Framebuffer object
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	cgGLDisableProfile(glParam->cgFragmentProfile);
	glDeleteTextures(1, &tempTex);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	// Restore the previous views
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();


	if (timing)
		sliceTime = clock() - initTime - initialTime;

	if (timing)
	{
		totalTime = clock() - initialTime;
		cout << numDivX << " x " << numDivY << " x " << numDivZ << " = " << numDivX*numDivY*numDivZ << endl << endl;
		cout << "Init  Time         : " << initTime*0.001 << endl;
		cout << "Slice Time         : " << sliceTime*0.001 / numDivZ << endl;
		cout << "InOut Total Time   : " << totalTime*0.001 << endl << endl;
	}
}


void Object::ClassifyInOut2x(GLParameters* glParam)
{
	int numSamples = 2;
	bool timing = true;
	clock_t initialTime, initTime, sliceTime, totalTime;
	if (timing)
		initialTime = clock();

	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	float gridSizeZ = this->voxelData->gridSizeZ;
	inOutDType* level1InOut = this->voxelData->level1InOut;

	if (timing)
		initTime = clock() - initialTime;

	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	float viewMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(this->bBoxMin[0], this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[1], this->bBoxMin[2] - 200, this->bBoxMax[2] + 200);
	
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);

	// Disable depth testing to render all pixels
	glDisable(GL_DEPTH_TEST);

	//glEnable(GL_DEPTH_TEST);
	// The Type Of Depth Testing To Do
	//glDepthFunc(GL_LESS);

	// Disable culling to render back faces
	glDisable(GL_CULL_FACE);

	cgGLEnableProfile(glParam->cgFragmentProfile);
	cgGLBindProgram(glParam->passiveProgram);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);

	//Initialize the texture
	GLuint tempTex;
	glGenTextures(1, &tempTex);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, numSamples*numDivX, numSamples*numDivY, 0, GL_RGBA, GL_FLOAT, 0);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8_EXT, numSamples*numDivX, numSamples*numDivY);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_STENCIL_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);

#ifdef DEBUG
	CheckFrameBuffer();
#endif

	// Rotate object around axis to test for off axis rendering
	//glMatrixMode(GL_MODELVIEW);
	//glRotatef(10,1,0,0);

	for (int slice = 0; slice < numDivZ; slice++)
	{
		glViewport(0, 0, numSamples*numDivX, numSamples*numDivY);

		double equation[4] = { 0, 0, -1, (slice + 0.5)*gridSizeZ + this->bBoxMin[2] };

		glClipPlane(GL_CLIP_PLANE0, equation);
		glEnable(GL_CLIP_PLANE0);

		glClearColor(0, 0, 0, 0);
		glClearStencil(0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 0x11111111);
		glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);

		this->DrawObject(glParam, false, 1.0);

		glDisable(GL_CLIP_PLANE0);
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDisable(GL_STENCIL_TEST);

		glFlush();

		// Read back
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		inOutDType* stencilData2x = new inOutDType[numDivX*numDivY*numSamples*numSamples];
		glReadPixels(0, 0, numSamples*numDivX, numSamples*numDivY, GL_STENCIL_INDEX, INOUTDFORMAT, stencilData2x);

		for (int j = 0; j < numDivY; j++)
		{
			for (int i = 0; i < numDivX; i++)
			{
				int currentIndex = (slice*numDivX*numDivY) + j*numDivX + i;
				int numInsideCount = 0;
				for (int l = 0; l < numSamples; l++)
				{
					for (int m = 0; m < numSamples; m++)
					{
						inOutDType sData = stencilData2x[(numSamples*j + l)*(numSamples*numDivX) + (numSamples*i + m)];
						if (int(sData) % 2 == 1)
							numInsideCount++;
					}
				}
				if (numInsideCount > numSamples*numSamples / 2.0)
					level1InOut[currentIndex] = inOutDType(1);
				else
					level1InOut[currentIndex] = inOutDType(0);
			}
		}
		delete[] stencilData2x;

#ifdef OUTPUTTEXTFILES
		char* fileName = new char[11];
		if (slice < 10)
			sprintf(fileName, "Slice0%d.txt", slice);
		else
			sprintf(fileName, "Slice%d.txt", slice);
		WriteToFileInt(level1InOut + slice*numDivX*numDivY, fileName, numDivX, numDivY);
#endif

	}

	// Unbind the Framebuffer object
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	cgGLDisableProfile(glParam->cgFragmentProfile);
	glDeleteTextures(1, &tempTex);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	// Restore the previous views
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();


	if (timing)
		sliceTime = clock() - initTime - initialTime;

	if (timing)
	{
		totalTime = clock() - initialTime;
		cout << numDivX << " x " << numDivY << " x " << numDivZ << " = " << numDivX*numDivY*numDivZ << endl << endl;
		cout << "Init  Time         : " << initTime*0.001 << endl;
		cout << "Slice Time         : " << sliceTime*0.001 / numDivZ << endl;
		cout << "InOut Total Time   : " << totalTime*0.001 << endl << endl;
	}
}

void Object::DrawInOutPoints()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);

	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	float gridSizeX = this->voxelData->gridSizeX;
	float gridSizeY = this->voxelData->gridSizeY;
	float gridSizeZ = this->voxelData->gridSizeZ;
	inOutDType* level1InOut = this->voxelData->level1InOut;
	glEnable(GL_COLOR_MATERIAL);
	glPointSize(3.0);
	glBegin(GL_POINTS);    // Specify point drawing
	for (int k = 0; k < numDivZ; k++)
	{
		for (int j = 0; j < numDivY; j++)
		{
			for (int i = 0; i < numDivX; i++)
			{
				if ((int(level1InOut[k*numDivX*numDivY + j*numDivX + i])) % 2 == 1)
					glColor3f(1.0, 0.0, 0.0);		// set vertex color to red					
				else
					glColor3f(0.0, 0.0, 0.0);		// set vertex color to black					
				if ((int(level1InOut[k*numDivX*numDivY + j*numDivX + i])) % 2 == 1)
					glVertex3f(float((i + 0.5)*gridSizeX + this->bBoxMin[0]), float((j + 0.5)*gridSizeY + this->bBoxMin[1]), float((k + 0.5)*gridSizeZ + this->bBoxMin[2]));
			}
		}
	}
	glEnd();
	glPopMatrix();
	glPopAttrib();
}

void Object::DrawCOM()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//	glDisable(GL_DEPTH_TEST);
	//	glDisable(GL_BLEND);
	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE2);
	glDisable(GL_CLIP_PLANE3);
	//	glBlendFunc(GL_ONE_MINUS_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_COLOR_MATERIAL);
	glPointSize(6.0);
	glColor4d(0, 1, 0, 1.0);

	glBegin(GL_POINTS);
	glVertex3f(this->massCenter[0], this->massCenter[1], this->massCenter[2]);
	glEnd();

	glPopAttrib();
}

void Object::DrawOBB()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);
	// set vertex color to green	
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_QUAD_STRIP);
	//Quads 1 2 3 4
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[2]);
	glEnd();
	glBegin(GL_QUADS);
	//Quad 5
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMin[2]);
	//Quad 6
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[2]);
	glEnd();

	glPopMatrix();
	glPopAttrib();
}


void Object::DrawVoxels(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	glEnable(GL_COLOR_MATERIAL);
	//	glEnable(GL_CULL_FACE);
	glLineWidth(2);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	Float4 color;
	if (this->objID < 1)
	{
		color[0] = 0.5;
		color[1] = 0.1;
		color[2] = 0.2;
	}
	else
	{
		color[0] = 0.1;
		color[1] = 0.5;
		color[2] = 0.2;
	}
	color[3] = 0.4;

	int numBoxes = this->voxelData->numDivX * this->voxelData->numDivY * this->voxelData->numDivZ;
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);

	for (int i = 0; i < numBoxes; i++)
	{
		Float3 v1 = Float3(this->voxelData->bBox[i].minPoint[0], this->voxelData->bBox[i].minPoint[1], this->voxelData->bBox[i].minPoint[2]);
		Float3 v2 = Float3(this->voxelData->bBox[i].maxPoint[0], this->voxelData->bBox[i].maxPoint[1], this->voxelData->bBox[i].maxPoint[2]);
		bool drawBox = false;

		if (this->voxelData->bBox[i].solid == 1)
			drawBox = true;

		//		if (this->voxelData->bBox[i].intersecting)
		//			drawBox = true;

		// ----------------- Determines Size of 2nd Level Voxels --------------------
		float sections = 5;
		float subDivX = (this->voxelData->bBox[i].maxPoint[0] - this->voxelData->bBox[i].minPoint[0]) / sections;
		float subDivY = (this->voxelData->bBox[i].maxPoint[1] - this->voxelData->bBox[i].minPoint[1]) / sections;
		float subDivZ = (this->voxelData->bBox[i].maxPoint[2] - this->voxelData->bBox[i].minPoint[2]) / sections;
		// --------------------------------------------------------------------------

		if (drawBox){
			DrawBox(v1, v2, color);

			// ------------- Draw the 2nd Layer of Voxels -----------------------
			for (int p = 0; p < sections; p++)
				for (int q = 0; q < sections; q++)
					for (int r = 0; r < sections; r++)
					{
						float minX = this->voxelData->bBox[i].minPoint[0] + subDivX * p;
						float minY = this->voxelData->bBox[i].minPoint[1] + subDivY * q;
						float minZ = this->voxelData->bBox[i].minPoint[2] + subDivZ * r;
						Float4 color2 = Float4(1, 1, 1, 0.4);
						Float3 minVec = Float3(minX, minY, minZ);
						Float3 maxVec = Float3(minX + subDivX, minY + subDivY, minZ + subDivZ);
						DrawBox(minVec, maxVec, color2);
					}
			// -------------------- End ----------------------------------------
			
		}

	}
	glPopMatrix();
	glPopAttrib();
}

void Object::GenVoxelsDisplayLists(GLParameters* glParam)
{
	this->voxelDLid = glGenLists(1);
	glNewList(this->voxelDLid, GL_COMPILE);
	this->DrawVoxels(glParam);
	glEndList();
}

void Object::DrawFaceBoundingBoxes(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	glEnable(GL_COLOR_MATERIAL);
	//	glEnable(GL_CULL_FACE);
	glLineWidth(2);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	Float4 color;
	if (this->objID < 1)
	{
		color[0] = 0.1;
		color[1] = 0.5;
		color[2] = 0.2;
	}
	else
	{
		color[0] = 0.5;
		color[1] = 0.1;
		color[2] = 0.2;
	}
	color[3] = 0.4;

	int numBoxes = this->faces.size();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);

	for (int i = 0; i < numBoxes; i++)
	{
		Float3 v1 = Float3(this->faces[i]->bBoxMin[0], this->faces[i]->bBoxMin[1], this->faces[i]->bBoxMin[2]);
		Float3 v2 = Float3(this->faces[i]->bBoxMax[0], this->faces[i]->bBoxMax[1], this->faces[i]->bBoxMax[2]);

		if (this->faces[i]->isMarked)
			DrawBox(v1, v2, color);

	}
	glPopMatrix();
	glPopAttrib();
}

void Object::DrawVoxelHierarchy(GLParameters* glParam)
{
	if (this->voxelData->numLevels == 0)
		return;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	glEnable(GL_COLOR_MATERIAL);
	//	glEnable(GL_CULL_FACE);
	glLineWidth(2);
	Float4 color;
	if (this->objID < 1)
	{
		color[0] = 0.5;
		color[1] = 0.1;
		color[2] = 0.2;
	}
	else
	{
		color[0] = 0.1;
		color[1] = 0.5;
		color[2] = 0.2;
	}
	color[3] = 0.4;

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);
	int totalBoxes = this->voxelData->numDivX * this->voxelData->numDivY * this->voxelData->numDivZ;
	int numLevelBoxes = totalBoxes / 2;
	int levelStartIndex = 0;
	for (int level = 1; level <= this->voxelData->numLevels; level++)
	{
		if (level == glParam->displayLevel)
		{
			for (int i = 0; i < numLevelBoxes; i++)
			{
				int index = levelStartIndex + i;
				assert(index < totalBoxes - 1);
				Float3 v1 = Float3(this->voxelData->bBoxHierarchy[index].minPoint[0], this->voxelData->bBoxHierarchy[index].minPoint[1], this->voxelData->bBoxHierarchy[index].minPoint[2]);
				Float3 v2 = Float3(this->voxelData->bBoxHierarchy[index].maxPoint[0], this->voxelData->bBoxHierarchy[index].maxPoint[1], this->voxelData->bBoxHierarchy[index].maxPoint[2]);
				bool drawBox = false;
				drawBox = true;

				if (drawBox)
					DrawBox(v1, v2, color);
			}
		}
		levelStartIndex += numLevelBoxes;
		numLevelBoxes /= 2;
	}
	glPopMatrix();
	glPopAttrib();
}

void Object::ClassifyTessellation(GLParameters* glParam)
{
	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;

	for (int i = 0; i < this->faces.size(); i++)
	{
		for (int j = 0; j < this->faces[i]->triangles.size(); j++)
		{
			// Add the vertex points if not already added
			Float3 vertex0 = this->faces[i]->triangles[j].vertices[0].point;
			Float3 vertex1 = this->faces[i]->triangles[j].vertices[1].point;
			Float3 vertex2 = this->faces[i]->triangles[j].vertices[2].point;

			int boxNumX0 = int((vertex0[0] - this->bBoxMin[0]) / (this->bBoxMax[0] - this->bBoxMin[0]) * numDivX);
			int boxNumY0 = int((vertex0[1] - this->bBoxMin[1]) / (this->bBoxMax[1] - this->bBoxMin[1]) * numDivY);
			int boxNumZ0 = int((vertex0[2] - this->bBoxMin[2]) / (this->bBoxMax[2] - this->bBoxMin[2]) * numDivZ);
			if (boxNumX0 == numDivX && vertex0[0] == this->bBoxMax[0])
				boxNumX0--;
			if (boxNumY0 == numDivY && vertex0[1] == this->bBoxMax[1])
				boxNumY0--;
			if (boxNumZ0 == numDivZ && vertex0[2] == this->bBoxMax[2])
				boxNumZ0--;
			AddPointToVoxel(boxNumX0, boxNumY0, boxNumZ0, this->faces[i]->surfID, this->faces[i]->triangles[j].triangleID);

			int boxNumX1 = int((vertex1[0] - this->bBoxMin[0]) / (this->bBoxMax[0] - this->bBoxMin[0]) * numDivX);
			int boxNumY1 = int((vertex1[1] - this->bBoxMin[1]) / (this->bBoxMax[1] - this->bBoxMin[1]) * numDivY);
			int boxNumZ1 = int((vertex1[2] - this->bBoxMin[2]) / (this->bBoxMax[2] - this->bBoxMin[2]) * numDivZ);
			if (boxNumX1 == numDivX && vertex1[0] == this->bBoxMax[0])
				boxNumX1--;
			if (boxNumY1 == numDivY && vertex1[1] == this->bBoxMax[1])
				boxNumY1--;
			if (boxNumZ1 == numDivZ && vertex1[2] == this->bBoxMax[2])
				boxNumZ1--;
			AddPointToVoxel(boxNumX1, boxNumY1, boxNumZ1, this->faces[i]->surfID, this->faces[i]->triangles[j].triangleID);

			int boxNumX2 = int((vertex2[0] - this->bBoxMin[0]) / (this->bBoxMax[0] - this->bBoxMin[0]) * numDivX);
			int boxNumY2 = int((vertex2[1] - this->bBoxMin[1]) / (this->bBoxMax[1] - this->bBoxMin[1]) * numDivY);
			int boxNumZ2 = int((vertex2[2] - this->bBoxMin[2]) / (this->bBoxMax[2] - this->bBoxMin[2]) * numDivZ);
			if (boxNumX2 == numDivX && vertex2[0] == this->bBoxMax[0])
				boxNumX2--;
			if (boxNumY2 == numDivY && vertex2[1] == this->bBoxMax[1])
				boxNumY2--;
			if (boxNumZ2 == numDivZ && vertex2[2] == this->bBoxMax[2])
				boxNumZ2--;
			AddPointToVoxel(boxNumX2, boxNumY2, boxNumZ2, this->faces[i]->surfID, this->faces[i]->triangles[j].triangleID);

			// Check if vertices lie inside the same voxel.
			// If not set all the voxels the triangle passes thorugh
			int minBoxX = __min(boxNumX0, __min(boxNumX1, boxNumX2));
			int minBoxY = __min(boxNumY0, __min(boxNumY1, boxNumY2));
			int minBoxZ = __min(boxNumZ0, __min(boxNumZ1, boxNumZ2));

			int maxBoxX = __max(boxNumX0, __max(boxNumX1, boxNumX2));
			int maxBoxY = __max(boxNumY0, __max(boxNumY1, boxNumY2));
			int maxBoxZ = __max(boxNumZ0, __max(boxNumZ1, boxNumZ2));


			for (int p = minBoxX; p <= maxBoxX && p < numDivX; p++)
				for (int q = minBoxY; q <= maxBoxY && q < numDivY; q++)
					for (int r = minBoxZ; r <= maxBoxZ && r < numDivZ; r++)
						if (!(p == boxNumX0 &&  q == boxNumY0 && r == boxNumZ0)
							&& !(p == boxNumX1 &&  q == boxNumY1 && r == boxNumZ1)
							&& !(p == boxNumX2 &&  q == boxNumY2 && r == boxNumZ2))
							if (TestTriBox(p, q, r, &(this->faces[i]->triangles[j])))
							{
								AddPointToVoxel(p, q, r, this->faces[i]->surfID, this->faces[i]->triangles[j].triangleID);
								this->voxelData->level2Index.push_back(r*numDivY*numDivX+q*numDivX+p);
								//AddNormal(this->faces[i]->surfID, this->faces[i]->triangles[j].triangleID);
							}
		}
	}
}

bool Object::TestTriBox(int x, int y, int z, Triangle* t)
{


	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	BBoxData* currentBoxPtr = &(this->voxelData->bBox[z*(numDivY)*(numDivX)+y*(numDivX)+x]);

	float gridSizeX = (this->bBoxMax[0] - this->bBoxMin[0]) / (this->voxelData->numDivX*1.0);
	float gridSizeY = (this->bBoxMax[1] - this->bBoxMin[1]) / (this->voxelData->numDivY*1.0);
	float gridSizeZ = (this->bBoxMax[2] - this->bBoxMin[2]) / (this->voxelData->numDivZ*1.0);

	int index = z*(numDivY)*(numDivX)+y*(numDivX)+x;

	/*
	float minX = x*gridSizeX + this->bBoxMin[0];
	float minY = y*gridSizeY + this->bBoxMin[1];
	float minZ = z*gridSizeZ + this->bBoxMin[2];
	this->voxelData->bBox[index].minPoint = Float3(minX, minY, minZ);
	this->voxelData->bBox[index].maxPoint = Float3(minX + gridSizeX, minY + gridSizeY, minZ + gridSizeZ);
	this->voxelData->bBox[index].index = index;
	this->voxelData->bBox[index].objID = this->objID;
	*/

	float boxCenter[3];
	boxCenter[0] = (currentBoxPtr->minPoint[0] + currentBoxPtr->maxPoint[0]) / 2.0;
	boxCenter[1] = (currentBoxPtr->minPoint[1] + currentBoxPtr->maxPoint[1]) / 2.0;
	boxCenter[2] = (currentBoxPtr->minPoint[2] + currentBoxPtr->maxPoint[2]) / 2.0;

	float boxHalfSize[3];
	boxHalfSize[0] = (currentBoxPtr->maxPoint[0] - currentBoxPtr->minPoint[0]) / 2.0;
	boxHalfSize[1] = (currentBoxPtr->maxPoint[1] - currentBoxPtr->minPoint[1]) / 2.0;
	boxHalfSize[2] = (currentBoxPtr->maxPoint[2] - currentBoxPtr->minPoint[2]) / 2.0;

	assert(boxHalfSize[0] > 0 && boxHalfSize[1] > 0 && boxHalfSize[2] > 0);

	float triVerts[3][3] =
	{
		{ t->vertices[0].point[0], t->vertices[0].point[1], t->vertices[0].point[2] },
		{ t->vertices[1].point[0], t->vertices[1].point[1], t->vertices[1].point[2] },
		{ t->vertices[2].point[0], t->vertices[2].point[1], t->vertices[2].point[2] }
	};

	bool intersection = TriBoxOverlap(boxCenter, boxHalfSize, triVerts);

	return intersection;
}

void Object::AddPointToVoxel(int x, int y, int z, int surfID, int triangleID)
{
	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	BBoxData* currentBoxPtr = &(this->voxelData->bBox[z*(numDivY)*(numDivX)+y*(numDivX)+x]);
	currentBoxPtr->solid = 1;
	bool alreadyAdded = false;
	for (int i = 0; i < currentBoxPtr->surfaces.size(); i++)
		if (currentBoxPtr->surfaces[i] == surfID)
			alreadyAdded = true;
	if (!alreadyAdded){
		currentBoxPtr->surfaces.push_back(surfID);
		this->voxelData->boundaryVoxels.push_back(z*(numDivY)*(numDivX)+y*(numDivX)+x); // Adds boundary voxel index
	}

	// Test for repeated triangles
	bool repeat = false;
	for (int i = 0; i < currentBoxPtr->triangles.size(); i++)
		if (currentBoxPtr->triangles[i] == triangleID && currentBoxPtr->faceIDs[i] == surfID)
			repeat = true;
	if (!repeat)
	{
		currentBoxPtr->triangles.push_back(triangleID);
		currentBoxPtr->faceIDs.push_back(surfID);
	}
	//delete[] currentBoxPtr;
	//currentBoxPtr = NULL;
}

void Object::AddNormal(int surfID, int triangleID)
{
	Triangle* triPtr = &(this->faces[surfID]->triangles[triangleID]);

	Float3 edge1 = triPtr->vertices[1].point - triPtr->vertices[0].point;
	Float3 edge2 = triPtr->vertices[2].point - triPtr->vertices[0].point;
	Float3 faceNormal = VectorCrossProduct(edge1, edge2);
	VectorNormalize(faceNormal);
	triPtr->faceNormal = faceNormal;

	//delete[] triPtr;
	//triPtr = NULL;
}

void CombineBBox(BBoxData* box1, BBoxData* box2, BBoxData* combinedBox)
{
	Float3 maxVal = Float3(0, 0, 0);
	Float3 minVal = Float3(0, 0, 0);
	if (box1->solid == 0 && box2->solid == 0)
		combinedBox->solid = 0;
	else if (box1->solid == 0)
	{
		combinedBox->solid = 1;
		maxVal = box2->maxPoint;
		minVal = box2->minPoint;
	}
	else if (box2->solid == 0)
	{
		combinedBox->solid = 1;
		maxVal = box1->maxPoint;
		minVal = box1->minPoint;
	}
	else
	{
		combinedBox->solid = 1;
		maxVal = Float3(max(box1->maxPoint[0], box2->maxPoint[0]), max(box1->maxPoint[1], box2->maxPoint[1]), max(box1->maxPoint[2], box2->maxPoint[2]));
		minVal = Float3(min(box1->minPoint[0], box2->minPoint[0]), min(box1->minPoint[1], box2->minPoint[1]), min(box1->minPoint[2], box2->minPoint[2]));
	}
	combinedBox->maxPoint = maxVal;
	combinedBox->minPoint = minVal;
	combinedBox->childIndex1 = box1->index;
	combinedBox->childIndex2 = box2->index;
}

void Object::BuildHierarchy(GLParameters* glParam)
{
	int totalNumBoxes = this->voxelData->numDivX * this->voxelData->numDivY * this->voxelData->numDivZ;
	int numLevels = this->voxelData->numLevels = GetExponent2(totalNumBoxes);
	voxelData->bBoxHierarchy = new BBoxData[totalNumBoxes - 1];
	// Init hieararchy data

	for (int i = 0; i < totalNumBoxes - 1; i++)
	{
		this->voxelData->bBoxHierarchy[i].minPoint = Float3(0, 0, 0);
		this->voxelData->bBoxHierarchy[i].maxPoint = Float3(0, 0, 0);
		this->voxelData->bBoxHierarchy[i].solid = 0;
		this->voxelData->bBoxHierarchy[i].objID = this->objID;
		this->voxelData->bBoxHierarchy[i].intersecting = 0;
	}

	int numLevelBoxes = totalNumBoxes / 2;
	for (int i = 0; i < numLevelBoxes; i++)
	{
		BBoxData* box1 = this->voxelData->bBox + 2 * i;
		BBoxData* box2 = this->voxelData->bBox + 2 * i + 1;
		BBoxData* combinedBox = this->voxelData->bBoxHierarchy + i;
		CombineBBox(box1, box2, combinedBox);
		combinedBox->index = i;
	}

	int numDivX = this->voxelData->numDivX / 2;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	int prevLevelIndex = 0;
	int levelIndex = numLevelBoxes;
	numLevelBoxes /= 2;
	for (int level = 2; level < numLevels + 1; level++)
	{
		int iSkip = (level % 3 == 1 && numDivX > 1) ? 2 : 1;
		int jSkip = (level % 3 == 2 && numDivY > 1) ? 2 : 1;
		int kSkip = (level % 3 == 0 && numDivZ > 1) ? 2 : 1;
		if (iSkip == 1 && jSkip == 1 && kSkip == 1)
			if (numDivX > 1)
				iSkip = 2;
			else if (numDivY > 1)
				jSkip = 2;
			else if (numDivZ > 1)
				kSkip = 2;
			for (int k = 0; k < numDivZ; k += kSkip)
			{
				for (int j = 0; j < numDivY; j += jSkip)
				{
					for (int i = 0; i < numDivX; i += iSkip)
					{
						int index1 = (k)*(numDivY)*(numDivX)+j*(numDivX)+i;
						int index2 = (k / kSkip)*(numDivY / jSkip)*(numDivX / iSkip) + (j / jSkip)*(numDivX / iSkip) + (i / iSkip);
						int skip = (kSkip - 1)*(numDivY)*(numDivX)+(jSkip - 1)*(numDivX)+(iSkip - 1);
						assert(levelIndex + index2 < totalNumBoxes - 1);
						BBoxData* box1 = this->voxelData->bBoxHierarchy + prevLevelIndex + index1;
						BBoxData* box2 = this->voxelData->bBoxHierarchy + prevLevelIndex + index1 + skip;
						BBoxData* combinedBox = this->voxelData->bBoxHierarchy + levelIndex + index2;
						CombineBBox(box1, box2, combinedBox);
						combinedBox->index = levelIndex + index2;
					}
				}
			}
			if (iSkip == 2)
				numDivX /= 2;
			if (jSkip == 2)
				numDivY /= 2;
			if (kSkip == 2)
				numDivZ /= 2;
			prevLevelIndex += numLevelBoxes * 2;
			levelIndex += numLevelBoxes;
			numLevelBoxes /= 2;
	}
}

void Object::SaveVoxelization(GLParameters* glParam)
{
	// Make Individual Files for each Normal Value
	ofstream normalxFile;
	ofstream normalyFile;
	ofstream normalzFile;
	ofstream inoutFile;

	// Level 2 Files
	/*ofstream lvl2NormalxFile;
	ofstream lvl2NormalyFile;
	ofstream lvl2NormalzFile;*/
	/*ofstream lvl2Indexes;
	ofstream lvl2InOutFile;*/

	normalxFile.open("xNormals.raw");
	normalyFile.open("yNormals.raw");
	normalzFile.open("zNormals.raw");
	inoutFile.open("inouts.raw");

	/*lvl2Indexes.open("lvl2Indexes");
	lvl2InOutFile.open("lvl2InOut.raw");*/

	if (!normalxFile.good() || !normalyFile.good() || !normalzFile.good() || !inoutFile.good())
	{
		cerr << "Unable to open output file for writing" << endl;
		abort();
	}

	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;

	// Initialize 3D array of points to be classified
	// Initalize arrays for x,y,z's of normals
	int voxelDataSize = (numDivX)*(numDivY)*(numDivZ);
	//int lvl2voxelDataSize = this->voxelData->level2Index.size;

	unsigned char* surfxNormData = new unsigned char[voxelDataSize];
	unsigned char* surfyNormData = new unsigned char[voxelDataSize];
	unsigned char* surfzNormData = new unsigned char[voxelDataSize];
	unsigned char* inoutData = new unsigned char[voxelDataSize];

	//unsigned char* lvl2Index = new unsigned char[lvl2voxelDataSize];
	//unsigned char* lvl2InOutData = new unsigned char[lvl2voxelDataSize*125];

	for (int k = 0; k < voxelDataSize; k++)
	{
		int numTri = this->voxelData->bBox[k].triangles.size();
		Float3 sumNorm = Float3(0, 0, 0);
		Float3 avgNorm = Float3(0, 0, 0);
		this->voxelData->bBox[k].avgNorm = avgNorm;
		// Sum up the normals of all the triangles in specific voxel
		if (numTri != 0)
		{
			for (int i = 0; i < numTri; i++)
			{
				int faceID = this->voxelData->bBox[k].faceIDs[i];
				int triangleID = this->voxelData->bBox[k].triangles[i];
				Float3 normal = this->faces[faceID]->triangles[triangleID].faceNormal;
				sumNorm += this->faces[faceID]->triangles[triangleID].faceNormal;
			}
			// Average of normals in voxel
			avgNorm = sumNorm / numTri;
			VectorNormalize(avgNorm);
			this->voxelData->bBox[k].avgNorm = avgNorm;
		}

		// Put Coordinates into their respective arrays
		//surfxNormData[k] = unsigned char(sqrt(avgNorm[0] * avgNorm[0] + avgNorm[1] * avgNorm[1] + avgNorm[2] * avgNorm[2]) * 255);
		surfxNormData[k] = unsigned char(avgNorm[0] * (256.0 / 3.0) + 127.0);
		surfyNormData[k] = unsigned char(avgNorm[1] * (256.0 / 3.0) + 127.0);
		surfzNormData[k] = unsigned char(avgNorm[2] * (256.0 / 3.0) + 127.0);
		inoutData[k] = unsigned char(this->voxelData->level1InOut[k] * 255);
	}

	// Add level 2
	/*for (int j = 0; j < lvl2voxelDataSize; j++)
	{
		lvl2Indexes[j] = unsigned char(this->voxelData->level2Index[j]);
		for (int i = 0; i < 125; i++)
		{
			lvl2InOutData[j * 125] = unsigned char((this->voxelData->level2InOut[j * 125 + i]) * 255);
		}
	}*/

	// Write sufNormData into respective files
	normalxFile.write((char*)surfxNormData, voxelDataSize);
	normalyFile.write((char*)surfyNormData, voxelDataSize);
	normalzFile.write((char*)surfzNormData, voxelDataSize);
	inoutFile.write((char*)inoutData, voxelDataSize);

	delete[] surfxNormData;
	delete[] surfyNormData;
	delete[] surfzNormData;
	delete[] inoutData;

	normalxFile.close();
	normalyFile.close();
	normalzFile.close();
	inoutFile.close();
}

void Object::PerformVoxelizationLevel2(GLParameters* glParam)
{
	// Initialize Global level 2 Inout.  Size = Sizeof Level2Index * 2nd level dimensions
	int numBoundary = this->voxelData->level2Index.size();
	this->voxelData->level2InOut= new inOutDType[numBoundary * 125];
	// Add for loop for boundary voxels
	for (int i = 0; i < numBoundary; i++)
	{
		// Call ClassifyInOUt Level 2
		ClassifyInOut2xLevel2(glParam, this->voxelData->level2Index[i], i);
	}
	
}

void Object::PerformVoxelization(GLParameters* glParam)
{
	bool timing = true;
	clock_t initialTime, inOutTime, classifyTime, classifyInitTime, totalTime;
	if (timing)
		initialTime = clock();

	this->voxelData = new VoxelData();
	this->voxelData->storeBoxData = true;

	float nominalGridSize = this->maxModelSize / (1.0*(glParam->voxelCount));
	//	this->voxelData->numDivX = GetNextPower2(int((this->bBoxMax[0] - this->bBoxMin[0])/nominalGridSize));
	//	this->voxelData->numDivY = GetNextPower2(int((this->bBoxMax[1] - this->bBoxMin[1])/nominalGridSize));
	//	this->voxelData->numDivZ = GetNextPower2(int((this->bBoxMax[2] - this->bBoxMin[2])/nominalGridSize));
	this->voxelData->numDivX = (int((this->bBoxMax[0] - this->bBoxMin[0]) / nominalGridSize));
	this->voxelData->numDivY = (int((this->bBoxMax[1] - this->bBoxMin[1]) / nominalGridSize));
	this->voxelData->numDivZ = (int((this->bBoxMax[2] - this->bBoxMin[2]) / nominalGridSize));
	this->voxelData->numDivX = GetNextDiv4(this->voxelData->numDivX);
	this->voxelData->numDivY = GetNextDiv4(this->voxelData->numDivY);
	this->voxelData->numDivZ = GetNextDiv4(this->voxelData->numDivZ);
	float gridSizeX = (this->bBoxMax[0] - this->bBoxMin[0]) / (this->voxelData->numDivX*1.0);
	float gridSizeY = (this->bBoxMax[1] - this->bBoxMin[1]) / (this->voxelData->numDivY*1.0);
	float gridSizeZ = (this->bBoxMax[2] - this->bBoxMin[2]) / (this->voxelData->numDivZ*1.0);
	this->voxelData->gridSizeX = gridSizeX;
	this->voxelData->gridSizeY = gridSizeY;
	this->voxelData->gridSizeZ = gridSizeZ;

	int numDivX = this->voxelData->numDivX;
	int numDivY = this->voxelData->numDivY;
	int numDivZ = this->voxelData->numDivZ;
	// Initialize 3D array of points to be classified
	this->voxelData->level1InOut = new inOutDType[(numDivX)*(numDivY)*(numDivZ)];

	// Classify inside outside using rendering
	this->ClassifyInOut2x(glParam);
	//this->ClassifyInOut(glParam);

	if (timing)
		inOutTime = clock();

	if (this->voxelData->storeBoxData)
	{
		this->voxelData->bBox = new BBoxData[(numDivX)*(numDivY)*(numDivZ)];
		// Initialize voxels
		for (int k = 0; k < numDivZ; k++)
		{
			for (int j = 0; j < numDivY; j++)
			{
				for (int i = 0; i < numDivX; i++)
				{
					float minX = i*gridSizeX + this->bBoxMin[0];
					float minY = j*gridSizeY + this->bBoxMin[1];
					float minZ = k*gridSizeZ + this->bBoxMin[2];
					this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].minPoint = Float3(minX, minY, minZ);
					this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].maxPoint = Float3(minX + gridSizeX, minY + gridSizeY, minZ + gridSizeZ);
					this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].index = k*(numDivY)*(numDivX)+j*(numDivX)+i;
					this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].objID = this->objID;
					
					if (this->voxelData->level1InOut != NULL)
						this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].solid = int(this->voxelData->level1InOut[k*(numDivY)*(numDivX)+j*(numDivX)+i]) % 2;
					else
						this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].solid = 0;

					this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].avgNorm = Float3(0, 0, 0);
					this->voxelData->bBox[k*(numDivY)*(numDivX)+j*(numDivX)+i].intersecting = 0;
				}
			}
		}

		if (timing)
			classifyInitTime = clock();

		this->ClassifyTessellation(glParam);
		if (timing)
			classifyTime = clock();

		this->PerformVoxelizationLevel2(glParam);
		//this->BuildHierarchy(glParam);
		this->SaveVoxelization(glParam);

	}

	if (timing)
	{
		totalTime = clock();
		cout << "InOut Time         : " << (inOutTime - initialTime)*0.001 << endl;
		if (this->voxelData->storeBoxData)
		{
			cout << "Classify Init Time : " << (classifyInitTime - inOutTime)*0.001 << endl;
			cout << "Classify Tri Time  : " << (classifyTime - classifyInitTime)*0.001 << endl;
			cout << "Save Time          : " << (totalTime - classifyTime)*0.001 << endl;
		}
		cout << "Total Time         : " << (totalTime - initialTime)*0.001 << endl;

		// Count Voxels
		int numInsideVoxels = 0;
		for (int k = 0; k < numDivZ; k++)
			for (int j = 0; j < numDivY; j++)
				for (int i = 0; i < numDivX; i++)
					if (int(this->voxelData->level1InOut[k*(numDivY)*(numDivX)+j*(numDivX)+i]) % 2 == 1)
						numInsideVoxels++;
		cout << "Inside Voxels      : " << numInsideVoxels << endl << endl;

	}
#ifdef DISPLAYLISTS
	if (this->voxelData->storeBoxData)
		this->GenVoxelsDisplayLists(glParam);
#endif
}

float Object::CalculateVolume(bool timing)
{
	clock_t initialTime, totalTime;
	if (timing)
		initialTime = clock();

	float signedVolume = 0;
	Float3 origin = Float3(0, 0, 0);
	for (int i = 0; i < this->faces.size(); i++)
	{
		for (int j = 0; j < this->faces[i]->triangles.size(); j++)
		{
			// Add the vertex points if not already added
			Float3 vertex0 = this->faces[i]->triangles[j].vertices[0].point;
			Float3 vertex1 = this->faces[i]->triangles[j].vertices[1].point;
			Float3 vertex2 = this->faces[i]->triangles[j].vertices[2].point;

			float volume = TetVolume(vertex0, vertex1, vertex2, origin);
			signedVolume += volume;
		}
	}
	if (timing)
	{
		totalTime = clock() - initialTime;
		cout << "Volume Computation Time : " << totalTime*0.001 << endl << endl;
		cout << "Volume : " << fabs(signedVolume) << endl;
	}
	return signedVolume;
}

void Face::CalculateStructureTensor()
{
	vector <Float4> colorValues;
	for (int k = 0; k < this->vertexFaces.size(); k++)
	{
		vector<int> faceIndices = this->vertexFaces[k];
		Float4 color = Float4(0, 1, 0, 1);



		for (int j = 0; j < faceIndices.size(); j++)
		{
			int faceNum = faceIndices[j];
			if (this->triangles[faceNum].vertexIndex[0] == k)
				this->triangles[faceNum].vertices[0].color = color;
			else if (this->triangles[faceNum].vertexIndex[1] == k)
				this->triangles[faceNum].vertices[1].color = color;
			else if (this->triangles[faceNum].vertexIndex[2] == k)
				this->triangles[faceNum].vertices[2].color = color;
		}
	}
	colorValues.clear();
	this->isVertexColored = true;
}

void Face::DrawFace(GLParameters* glParam, bool computeVisibility, float transparency)
{
	// Set material properties and normals as well as actually draw the triangles
	GLfloat mat_ambient[] = { float(this->ka*this->kdColor[0]), float(this->ka*this->kdColor[1]), float(this->ka*this->kdColor[2]), transparency };
	GLfloat mat_diffuse[] = { float(this->kdColor[0]), float(this->kdColor[1]), float(this->kdColor[2]), transparency };
	GLfloat mat_specular[] = { float(this->ksColor[0]), float(this->ksColor[1]), float(this->ksColor[2]), transparency };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, this->shininess);

	cgGLEnableProfile(glParam->cgFragmentProfile);
	cgGLBindProgram(glParam->passiveProgram);
	if (this->trimmed)
	{
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->trimTexture);
	}
	for (int i = 0; i < this->triangles.size(); i++)
	{

		Float4 color0;
		Float4 color1;
		Float4 color2;

		Float3 normal0 = this->triangles[i].vertices[0].normal;
		Float3 normal1 = this->triangles[i].vertices[1].normal;
		Float3 normal2 = this->triangles[i].vertices[2].normal;

		Float3 vertex0 = this->triangles[i].vertices[0].point;
		Float3 vertex1 = this->triangles[i].vertices[1].point;
		Float3 vertex2 = this->triangles[i].vertices[2].point;

		Float2 texCoord0 = this->triangles[i].vertices[0].texCoords;
		Float2 texCoord1 = this->triangles[i].vertices[1].texCoords;
		Float2 texCoord2 = this->triangles[i].vertices[2].texCoords;

		if (this->isVertexColored)
		{
			color0 = this->triangles[i].vertices[0].color;
			color1 = this->triangles[i].vertices[1].color;
			color2 = this->triangles[i].vertices[2].color;
			glEnable(GL_COLOR_MATERIAL);
		}
		float offset = glParam->offsetDistance;
		if (offset > 0)
		{
			vertex0 += (normal0*offset);
			vertex1 += (normal1*offset);
			vertex2 += (normal2*offset);
		}

		//		Float3 midVertex = (vertex0 + vertex1 + vertex2)/3.0;
		//		Float3 normalVertex = midVertex + 1*this->triangles[i].faceNormal;

		glBegin(GL_TRIANGLES);
		glNormal3f(normal0[0], normal0[1], normal0[2]);
		if (this->trimmed)
			glTexCoord2f(texCoord0[0], texCoord0[1]);
		if (this->isVertexColored)
			glColor4f(color0[0], color0[1], color0[2], color0[3]);
		glVertex3f(vertex0[0], vertex0[1], vertex0[2]);

		glNormal3f(normal1[0], normal1[1], normal1[2]);
		if (this->isVertexColored)
			glColor4f(color1[0], color1[1], color1[2], color1[3]);
		if (this->trimmed)
			glTexCoord2f(texCoord1[0], texCoord1[1]);
		glVertex3f(vertex1[0], vertex1[1], vertex1[2]);

		glNormal3f(normal2[0], normal2[1], normal2[2]);
		if (this->isVertexColored)
			glColor4f(color2[0], color2[1], color2[2], color2[3]);
		if (this->trimmed)
			glTexCoord2f(texCoord2[0], texCoord2[1]);
		glVertex3f(vertex2[0], vertex2[1], vertex2[2]);
		glEnd();
		//		glBegin(GL_LINES);
		//			glVertex3f(midVertex[0], midVertex[1], midVertex[2]);
		//			glVertex3f(normalVertex[0], normalVertex[1], normalVertex[2]);
		//		glEnd();

		if (this->isVertexColored)
			glDisable(GL_COLOR_MATERIAL);
	}
	if (this->trimmed)
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
	cgGLDisableProfile(glParam->cgFragmentProfile);
}

float Face::GetDistanceFromSphere(float* viewMatrix)
{
	float dist = 0;
	for (int i = 0; i < this->triangles.size(); i++)
	{
		Float3 v1 = Float3(this->triangles[i][0].point[0], this->triangles[i][0].point[1], this->triangles[i][0].point[2]);
		Float3 v2 = Float3(this->triangles[i][1].point[0], this->triangles[i][1].point[1], this->triangles[i][1].point[2]);
		Float3 v3 = Float3(this->triangles[i][2].point[0], this->triangles[i][2].point[1], this->triangles[i][2].point[2]);
		Float3 v = (v1 + v2 + v3) / 3.0;
		Float3 transformedPoint = TransformPoint(v, viewMatrix);
		float triangleDist = VectorMagnitude(transformedPoint);
		if (triangleDist > dist)
			dist = triangleDist;
	}
	return dist;
}

float GetFaceFaceClosestPoint(Face* face1, Face* face2, float* tMatrix1, float* tMatrix2, float* point1, float* point2)
{
	float minDist = -1;
	Float3 closestPoint1;
	Float3 closestPoint2;
	for (int i = 0; i < face1->triangles.size(); i++)
	{
		for (int j = 0; j < face2->triangles.size(); j++)
		{
			for (int p = 0; p < 3; p++)
			{
				for (int q = 0; q < 3; q++)
				{
					Float3 translate1 = Float3(tMatrix1[12], tMatrix1[13], tMatrix1[14]);
					Float3 translate2 = Float3(tMatrix2[12], tMatrix2[13], tMatrix2[14]);
					Float3 vertex1 = Float3(face1->triangles[i].vertices[p].point[0], face1->triangles[i].vertices[p].point[1], face1->triangles[i].vertices[p].point[2]) + translate1;
					Float3 vertex2 = Float3(face2->triangles[j].vertices[q].point[0], face2->triangles[j].vertices[q].point[1], face2->triangles[j].vertices[q].point[2]) + translate2;
					float dist = Distance(vertex1, vertex2);
					if (dist < minDist || minDist < 0)
					{
						closestPoint1 = vertex1;
						closestPoint2 = vertex2;
						minDist = dist;
					}
				}
			}
		}
	}
	point1[0] = closestPoint1[0];
	point1[1] = closestPoint1[1];
	point1[2] = closestPoint1[2];
	point2[0] = closestPoint2[0];
	point2[1] = closestPoint2[1];
	point2[2] = closestPoint2[2];
	return minDist;
}

void Face::DrawOBB()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	//	glMultMatrixf(this->transformationMatrix);
	// set vertex color to green	
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0, 1.0, 0.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_QUAD_STRIP);
	//Quads 1 2 3 4
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[2]);
	glEnd();
	glBegin(GL_QUADS);
	//Quad 5
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMax[1], this->bBoxMin[2]);
	//Quad 6
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMax[2]);
	glVertex3f(this->bBoxMin[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMin[2]);
	glVertex3f(this->bBoxMax[0], this->bBoxMin[1], this->bBoxMax[2]);
	glEnd();

	glPopMatrix();
	glPopAttrib();
}


