#include "Object.h"
#include "GeometricOperations.h"
#include "GeometricMoments.h"
#include "Hausdorff.h"
#include "HausdorffCUDA.h"
#include "GPUUtilities.h"
#include "CUDAUtilities.h"
#include "Mesh.h"

#ifdef ACISOBJ
#include "CheckOutcome.h"
#endif

//iddo:
#include "HausdorffIddo.h"
#include "HausdorffCUDAIddo.h"
#include "NURBSGeomUtils.h"
#define NRKB 0.01 //for now

double NRKA = 0; //iddo: since it is relatively expensive to compute, we want to compute it only once
bool recomputeNRKA = false;
bool computeNRHD = false;

//Test Variables
int numDucky1 = 0;
int numDucky2 = 0;
int numTeapot1 = 0;
int numTeapot2 = 0;
int numFlags = 0;
int numTestNURBS = 0;

int intSurf1 = 0;
int intSurf2 = 1;

int surf1Num = 5;;//4,1,5,21,17;
int surf2Num = 4;//10,12,10,23;24,18;

//Resolution
int intRes = 64;
int silRes = 300;
bool selfInt = false;

// Lighting Variables
float objShininess = 10;
float objTransparency = 1.0;
float objKA = 0.5;

#ifdef COMPUTEDISTANCEMETRICS
bool computeDistanceMetrics = true;
bool computeInteractiveRun = false;
#else
bool computeDistanceMetrics = false;
bool computeInteractiveRun = false;
#endif

bool trimmedSurfaces = true;

//Move object 
//#define MOVEOBJ
//#define MOVESURF
float xPos = 0; 
float yPos = 0;
float zPos = 250;
//float xPos = -975; 
//float yPos = 100;
//float zPos = 100;

// Fixed view
bool fixedView = false;
float fixedViewMatrix[16] = {0.97453058,
-0.46680558,
1.1623085,
0,
1.2426617,
0.54494876,
-0.82304496,
0,
-0.1570241,
1.4155214,
0.70015824,
0,
98.893898,
-208.47809,
3.1728468,
1};


// View Variables
vector<Object*> objects;
vector<Light> lights;
Camera camera;
Viewport viewport;

// Program UI variables
bool lButtonDown		= false;
bool rButtonDown		= false;
bool mButtonDown		= false;
bool lShiftButtonDown	= false;
bool rShiftButtonDown	= false;
bool lCtrlButtonDown	= false;
float zoomFactor		= 1;
float modelSize			= 3;
int pickedPointIndex	= -1;
int standardView		= 0;
Float2 clickPoint		= Float2(0,0);
Float2 currentPoint		= Float2(0,0);
float sceneSize			= 0;
float sceneVolume		= 0;

float silhouetteArrowFactor = 0.7;
Float3 silhouettePoint		= Float3(0,modelSize*silhouetteArrowFactor,0);
Float3 silhouetteVector		= Float3(0,-1,0);
//CG and GL variables
GLParameters* glParam;

Timer timer;
Timer timerNR;
Timer timerBB;

#ifdef ANIMATE
int angle=0;
#endif

#ifdef INTERACTIVETIMING
ofstream ofs;
#endif

#ifdef INTERACTIVECOLLISION
int numCollisionRuns = 0;
float collisionTime;
#endif

int numRuns = 0;
float evaluationTime;
float closestPointTimeNR;
float closestPointTimeBB;

vector<NURBS*> nurbsSurfaces;

bool evaluated;
bool cleared;
int backgroundColor		= 0;
bool animate;
bool drawSilhouette		= false;
int reflectionPlane		= 0;
bool clipping			= false;
float clipPlaneDistX	= 2.8;
float clipPlaneDistY	= 0;
float clipPlaneDistZ	= -1.7;
int selectedObject1		= 0;
int selectedObject2		= 1;

void ParseInputFile(char *fname)
{
	ifstream in(fname, ios::in);
	Object obj;
	char str[80];
	int numObjects,numLights;
	int i;

	if (!in.good())
	{
		cerr<<"Unable to open file \""<<fname<<"\""<<endl;
		abort();
	}

	in >> numObjects;	
	for (i=0; i<numObjects; i++) 
	{
		Object* tempObject = new Object();
		Face* face = new Face();
		tempObject->faces.push_back(face);
		tempObject->faces[0]->dlid = (GLuint) i+1;
		tempObject->faces[0]->trimmed=false;
		in >> tempObject->faces[0]->kdColor[0] >> tempObject->faces[0]->kdColor[1] >> tempObject->faces[0]->kdColor[2];
		float ka, kd, ks;
		in >> tempObject->faces[0]->ka >> kd >> ks;
		in >> str;
		tempObject->faces[0]->ksColor = tempObject->faces[0]->kdColor*ks;
		tempObject->ReadObject(str);
		objects.push_back(tempObject);
	}

	// read lights
	in >> numLights;
	lights.resize(numLights);

	for (i=0; i<numLights; i++)
	{
		in >> lights[i].ambient[0] >> lights[i].ambient[1] >> lights[i].ambient[2] >>
			lights[i].ambient[3];
		in >> lights[i].diffuse[0] >> lights[i].diffuse[1] >> lights[i].diffuse[2] >>
			lights[i].diffuse[3];
		in >> lights[i].specular[0] >> lights[i].specular[1] >> lights[i].specular[2] >>
			lights[i].specular[3];
		in >> lights[i].pos[0] >> lights[i].pos[1] >> lights[i].pos[2] >> lights[i].pos[3];
		lights[i].id=GL_LIGHT0+i;
	}
}

void ReadOBJFile(char* fName, int dlID)
{
	Object* tempObject = new Object();
	Face* face = new Face();
	face->dlid = dlID;
	face->trimmed=false;
	face->kdColor = Float3( 0.768628, 0.462745, 0.137255);
	face->ksColor = Float3( 0.9, 0.9, 0.9);
	face->ka = 0.11;
	face->shininess = 50;
	face->surfID = 0;
	tempObject->faces.push_back(face);
	tempObject->ReadObject(fName);
	objects.push_back(tempObject);
}

void CALLBACK NURBSError()
{
	const GLubyte *estring;
	GLenum errorCode = 0;
	estring = gluErrorString(errorCode);
	fprintf (stderr, "Nurbs Error: %s\n", estring);
	exit (0);
}

void CGErrorCallback()
{
	CGerror lastError = cgGetError();

	if(lastError)
	{
		printf("%s\n\n", cgGetErrorString(lastError));
		printf("%s\n", cgGetLastListing(glParam->cgContext));
		printf("Cg error!\n");
	}
}

void ClearSurface(NURBS* surface)
{
	//Clear the trim texture
	surface->pickedBoxes.clear();
	if (surface->initializedEvalParams)
	{
		delete surface->evalParams;
		surface->initializedEvalParams = false;
	}
#ifndef VBO
	//Clear the trim texture and previous surface 
	surface->pickedBoxes.clear();
	delete[] surface->evaluatedPointsDisplay;
	delete[] surface->evaluatedNormalsDisplay;
#else
	delete[] surface->indexArray;
	if (surface->evaluatedVBO)
	{
		glDeleteBuffersARB(1,&(surface->ptVBO));
		glDeleteBuffersARB(1,&(surface->normVBO));
		glDeleteBuffersARB(1,&(surface->texVBO));
		surface->evaluatedVBO = false;
	}
#endif
}

void EvaluateSurface(NURBS* surface, float lodFactor, bool forcedEvaluation = true)
{
	int uNum = int(surface->uBaseNum*lodFactor);
	int vNum = int(surface->vBaseNum*lodFactor);
	//	uNum = GetNextPower2(uNum);
	//	vNum = GetNextPower2(vNum);

	surface->evalParams = new EvaluationParams();
	surface->initializedEvalParams = true;
	surface->evalParams->rendering = true;

	surface->evalParams->uNum		= uNum;
	surface->evalParams->vNum		= vNum;
	surface->uNumDisplay			= uNum;
	surface->vNumDisplay			= vNum;
	surface->trimWidth				= int(20*lodFactor*surface->uBaseNum);
	surface->trimHeight				= int(20*lodFactor*surface->vBaseNum);
	int maxSize = 4096;
	int minSize = 32;
	silRes = __max(surface->trimHeight, surface->trimWidth);

	if (surface->trimWidth < minSize)
		surface->trimWidth = minSize;
	if (surface->trimWidth > maxSize)
		surface->trimWidth = maxSize;

	if (surface->trimHeight < minSize)
		surface->trimHeight = minSize;
	if (surface->trimHeight > maxSize)
		surface->trimHeight = maxSize;

	if (TRIM_TEXTURE_FORMAT == GL_TEXTURE_2D)
	{
		surface->trimWidth = GetNextPower2(surface->trimWidth);
		surface->trimHeight = GetNextPower2(surface->trimHeight);
		surface->trimWidth = surface->trimHeight = __max(surface->trimHeight, surface->trimWidth);
	}

	surface->trimCurveLOD	= lodFactor;

	if (surface->trimmed)
		surface->GenerateTrimTexture(glParam);

	if (glParam->drawBBox)
	{
		surface->minPointsDisplay	= new float[(uNum)*(vNum)*4];
		surface->maxPointsDisplay	= new float[(uNum)*(vNum)*4];
	}
#ifndef VBO
	surface->evaluatedPointsDisplay		= new float[(uNum+1)*(vNum+1)*4];
	surface->evaluatedNormalsDisplay	= new float[(uNum+1)*(vNum+1)*3];
	surface->SurfaceEvaluation(glParam, true, false);
#else
	glGenBuffersARB(1,&(surface->ptVBO));
	glGenBuffersARB(1,&(surface->normVBO));
	glGenBuffersARB(1,&(surface->texVBO));

	GLfloat* texCoord = new GLfloat[(uNum+1)*(vNum+1)*2];
	GLuint* indexArray =  new GLuint[uNum*vNum*4];
	for (int vCount=0; vCount<=vNum; vCount++)
	{
		for (int uCount=0; uCount<=uNum; uCount++)
		{
			int width  = surface->trimWidth;
			int height = surface->trimHeight;
			if (TRIM_TEXTURE_FORMAT == GL_TEXTURE_2D)
				width = height = 1;
			float uTex = float(uCount/(uNum*1.0))*width;
			float vTex = float(vCount/(vNum*1.0))*height;
			texCoord[vCount*(uNum+1)*2+uCount*2+0] = uTex;
			texCoord[vCount*(uNum+1)*2+uCount*2+1] = vTex;

			if (uCount!=uNum && vCount!=vNum)
			{
				indexArray[vCount*(uNum)*4+uCount*4+0] =  vCount   *(uNum+1)+ uCount;
				indexArray[vCount*(uNum)*4+uCount*4+1] =  vCount   *(uNum+1)+(uCount+1);
				indexArray[vCount*(uNum)*4+uCount*4+2] = (vCount+1)*(uNum+1)+(uCount+1);
				indexArray[vCount*(uNum)*4+uCount*4+3] = (vCount+1)*(uNum+1)+ uCount;
			}
		}
	}
	surface->indexArray = indexArray;
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, surface->texVBO);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, (uNum+1)*(vNum+1)*2*sizeof(float), texCoord, GL_STATIC_DRAW);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
	delete[] texCoord;

#ifdef CUDA
	//	struct cudaGraphicsResource* pointVBOResource;
	//	struct cudaGraphicsResource* normalVBOResource;

	glBindBufferARB(GL_ARRAY_BUFFER, surface->ptVBO);
	glBufferDataARB(GL_ARRAY_BUFFER, (uNum+1)*(vNum+1)*4*sizeof(float), NULL, GL_DYNAMIC_DRAW);
	glBindBufferARB(GL_ARRAY_BUFFER, 0);
	cudaGLRegisterBufferObject(surface->ptVBO);
	//	cudaGraphicsGLRegisterBuffer(&(pointVBOResource), surface->ptVBO, cudaGraphicsMapFlagsNone);
	CUDACheckErrors(" VBO Registration ");

	glBindBufferARB(GL_ARRAY_BUFFER_ARB, surface->normVBO);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, (uNum+1)*(vNum+1)*3*sizeof(float), NULL, GL_DYNAMIC_DRAW);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
	cudaGLRegisterBufferObject(surface->normVBO);
	//	cudaGraphicsGLRegisterBuffer(&(normalVBOResource), surface->normVBO, cudaGraphicsMapFlagsNone);
	CUDACheckErrors(" VBO Registration ");

#ifdef CUDATEXTURES
	surface->CUDASurfaceEvaluationTextures(glParam->exactNormals, false, glParam->readBack, true);
#else
	surface->CUDASurfaceEvaluation(glParam->exactNormals, false, glParam->readBack, true);
#endif


	//	cudaGraphicsUnregisterResource(pointVBOResource);
	//	cudaGraphicsUnregisterResource(normalVBOResource);
	cudaGLUnregisterBufferObject(surface->ptVBO);
	cudaGLUnregisterBufferObject(surface->normVBO);
	CUDACheckErrors(" Unregister VBO ");

#else
	surface->SurfaceEvaluation(glParam, true, false);
#endif
	surface->evaluatedVBO = true;
#endif
}

#ifdef BREP
void EvaluateSurfaces(int objectNum =-1, int surfNum = -1, bool forcedEvaluation = true)
{
	bool timing = true;
	if (timing)
		timer.Start();

	// Store the ModelViewMatrix
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX, glParam->modelViewMatrix);
	double lodFactor = GetLOD();

	// Clear data if it is not cleared before
	if (objectNum == -1 && surfNum == -1)
	{
		if (!cleared)
		{
			for (int objID = 0; objID < objects.size(); objID++)
			{
				Object* object = objects[objID];
				for (int i = 0; i < object->faces.size(); i++)
				{
					if (object->faces[i]->isNURBS)
					{
						NURBS* surface = object->faces[i]->surface;
						ClearSurface(surface);
					}
				}
			}
			cleared = true;
		}

		for (int objID = 0; objID < objects.size(); objID++)
		{
			Object* object = objects[objID];
			for (int i = 0; i < object->faces.size(); i++)
			{
				if (object->faces[i]->isNURBS)
				{
					NURBS* surface = object->faces[i]->surface;
					EvaluateSurface(surface, lodFactor, forcedEvaluation);
					if (surface->evaluatedIntersection)
						surface->GenerateIntersectionOverlayTexture(surface->intersectingBoxes, surface->numIntersectingBoxes, intRes, glParam);
				}
			}
		}
	}
	else
	{
		Face* face = objects[objectNum]->faces[surfNum];
		if (face->isNURBS)
		{
			NURBS* surface = face->surface;
			if (!cleared)
				ClearSurface(surface);
			cleared = true;

			EvaluateSurface(surface, lodFactor, forcedEvaluation);
			if (surface->evaluatedIntersection)
				surface->GenerateIntersectionOverlayTexture(surface->intersectingBoxes, surface->numIntersectingBoxes, intRes, glParam);
		}
	}

	if (timing)
	{
		timer.Stop();
		cout << "Evaluation Time : " << timer.GetElapsedTimeInMilliSec() << endl;
	}
}
#else
void EvaluateSurfaces(int surfNum = -1, int objNum = -1, bool forcedEvaluation = true)
{
	bool timing = true;
	float initTime;
	if (timing)
		timer.Start();

	int startSurfNum = 0;
	int endSurfNum = nurbsSurfaces.size();
	if (surfNum >= 0)
	{
		startSurfNum = surfNum;
		endSurfNum = surfNum+1;
	}

	// Store the ModelViewMatrix
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX, glParam->modelViewMatrix);
	double lodFactor = GetLOD();

	// Clear data if it is not cleared before
	if (!cleared)
	{
		for (int i = startSurfNum; i < endSurfNum; i++)
		{
			NURBS* surface = nurbsSurfaces[i];
			ClearSurface(surface);
		}
		cleared = true;
	}

	if (timing)
	{
		timer.Stop();
		initTime = timer.GetElapsedTimeInMilliSec();
		timer.Start();
	}

	for (int i = startSurfNum; i < endSurfNum; i++)
	{
		NURBS* surface = nurbsSurfaces[i];
		EvaluateSurface(surface, lodFactor, forcedEvaluation);

		if (surface->evaluatedIntersection)
			surface->GenerateIntersectionOverlayTexture(surface->intersectingBoxes, surface->numIntersectingBoxes, intRes, glParam);
	}

	if (timing)
	{
		timer.Stop();
		cout << "Total Time      : " << timer.GetElapsedTimeInMilliSec() + initTime << endl;
		cout << "Evaluation Time : " << timer.GetElapsedTimeInMilliSec() << endl << endl;
	}
}
#endif

void InitGLEW(void)
{
	// Initialize GLEW
	GLenum ret = glewInit();
	if (ret != GLEW_OK)
	{
		// Problem: glewInit failed, something is seriously wrong.
		fprintf(stderr, "Error: %s\n", glewGetErrorString(ret));
	}
	if(!GLEW_EXT_framebuffer_object)
	{
		fprintf(stderr,"EXT_framebuffer_object is not supported!\n\n");
		exit(EXIT_FAILURE);
	}
	else if(!GLEW_ARB_occlusion_query)
	{
		fprintf(stderr,"Occlusion Query is not supported!\n\n");
		exit(EXIT_FAILURE);
	}
}

void InitGL(void) 
{
	// setup camera info
	camera.nearcp	= -50*modelSize*max(viewport.h/2.0,viewport.w/2.0)/100.0;
	camera.farcp	=  50*modelSize*max(viewport.h/2.0,viewport.w/2.0)/100.0;
	camera.leftcp	= -modelSize*viewport.w/200.0;
	camera.rightcp	=  modelSize*viewport.w/200.0;
	camera.bottomcp = -modelSize*viewport.h/200.0;
	camera.topcp	=  modelSize*viewport.h/200.0;

	camera.fov = 30;
	camera.eye  = Float3(0,0,1);
	camera.center  = Float3(0,0,0);
	camera.up = Float3(0,0,0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Set up the projection using glOrtho and gluLookAt
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (camera.leftcp,camera.rightcp,camera.bottomcp,camera.topcp,camera.nearcp,camera.farcp);

#ifdef DISPLAYLISTS
	// Generate Display Lists
	for (int i=0; i<objects.size(); i++)
		objects[i]->CreateDisplayLists(glParam);
#endif
}

// reshape viewport if the window is resized
void ReSize(int w, int h)
{
	viewport.w = w;
	viewport.h = h;

	camera.nearcp	= -50*modelSize*max(viewport.h/2.0,viewport.w/2.0)/100.0;
	camera.farcp	=  50*modelSize*max(viewport.h/2.0,viewport.w/2.0)/100.0;
	camera.leftcp	= -modelSize*viewport.w/200.0;
	camera.rightcp	=  modelSize*viewport.w/200.0;
	camera.bottomcp = -modelSize*viewport.h/200.0;
	camera.topcp	=  modelSize*viewport.h/200.0;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (camera.leftcp,camera.rightcp,camera.bottomcp,camera.topcp,camera.nearcp,camera.farcp);
	glutPostRedisplay();
}

void DrawSilhouetteDirection()
{
	float cylHeight = modelSize*.1;
	float cylWidth  = cylHeight/10.0;
	Float3 endPoint = silhouettePoint + silhouetteVector*cylHeight;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushMatrix();
	glEnable(GL_COLOR_MATERIAL);
	glColor4d(1, 0.1, 0.4, 0.4);

	glPushMatrix();
	glTranslated(silhouettePoint[0], silhouettePoint[1], silhouettePoint[2]);
	glScalef(2,2,2);
	GLUquadricObj* quadric = gluNewQuadric();

	Float3 basePerp1 = GetPerpendicular(silhouetteVector);
	Float3 basePerp2 = VectorCrossProduct(silhouetteVector,basePerp1);
	VectorNormalize(basePerp2);

	double localMatrix[]={	basePerp1[0]		, basePerp1[1]			, basePerp1[2]			, 0,
		basePerp2[0]		, basePerp2[1]			, basePerp2[2]			, 0,
		silhouetteVector[0]	, silhouetteVector[1]	, silhouetteVector[2]	, 0,
		0					, 0						, 0						, 1 };
	glMultMatrixd(localMatrix);
	gluCylinder(quadric, 2*cylWidth, 0, 4*cylWidth, 20, 3);
	gluDisk(quadric, 0, 2*cylWidth, 20, 1);
	double localMatrix2[]={	0 , 1 , 0 , 0,
		1 ,	0 , 0 , 0,
		0 ,	0 ,-1 , 0,
		0 ,	0 , 0 , 1};
	glMultMatrixd(localMatrix2);
	gluCylinder(quadric, cylWidth, cylWidth, cylHeight, 20, 3);
	glTranslated(0, 0, cylHeight);
	gluDisk(quadric, 0, cylWidth, 20, 1);

	glPopMatrix();
	glPopAttrib();
}

void DrawReflection()
{
	// Draw floor with stencil test
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	float rotX = 0;
	float rotY = 0;
	float rotZ = 0;
	if (reflectionPlane == 1)
		rotZ = 1;
	else if (reflectionPlane == 2)
		rotX = -1;
	else if (reflectionPlane == 3)
		rotY = 1;

	glRotatef(90,rotX,rotY,rotZ);

	DrawFloor(-5000, 5000, -5000, 5000, 0, true);
	glPopMatrix();

	// Now, only render where stencil is set to 1
	glStencilFunc(GL_EQUAL, 1, 0xffffffff);  // draw if stencil == 1
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

	if (clipping)
	{
		double equation[4] = {0,0,-1,clipPlaneDistZ};	
		glClipPlane(GL_CLIP_PLANE0, equation);
		glEnable(GL_CLIP_PLANE0);
	}
	// Draw reflected objects where stencil test passes
	glPushMatrix();
	if (reflectionPlane == 1)
		glScalef(1.0, 1.0, -1.0);
	else if (reflectionPlane == 2)
		glScalef(1.0, -1.0, 1.0);
	else if (reflectionPlane == 3)
		glScalef(-1.0, 1.0, 1.0);

	for (int i=0; i<lights.size(); i++)
		lights[i].Apply();

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);
	for (int i=0; i<nurbsSurfaces.size(); i++)
		if (nurbsSurfaces[i]->visibilityFactor >= glParam->surfVisibilityCutOff)
			nurbsSurfaces[i]->DrawNURBS(glParam, 1.0);

	glPopMatrix();

	if (clipping)
		glDisable(GL_CLIP_PLANE0);

	// Draw actual floor
	glPushMatrix();
	glRotatef(90,rotX,rotY,rotZ);
	DrawFloor(-5000, 5000, -5000, 5000, 0, false);
	glPopMatrix();

	glDisable(GL_STENCIL_TEST);
}

void ClipObjects(double equation[4], GLuint clipPlaneID)
{
	glEnable(clipPlaneID);

	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 0x1, 0x1);
	//	glStencilOpSeparate(GL_FRONT, GL_KEEP, GL_KEEP, GL_DECR);
	//	glStencilOpSeparate(GL_BACK, GL_KEEP, GL_KEEP, GL_INCR);
	glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	if (evaluated)
	{
		for (int i=0; i<nurbsSurfaces.size(); i++)
			if (nurbsSurfaces[i]->visibilityFactor >= glParam->surfVisibilityCutOff)
				nurbsSurfaces[i]->DrawNURBS(glParam, 1.0);
	}
	glDisable(clipPlaneID);
#ifdef DUMP
	float* retVal=new float[viewport.h*viewport.w];
	glReadBuffer(GL_FRONT);
	glReadPixels(0,0,viewport.w,viewport.h,GL_STENCIL_INDEX,GL_FLOAT,retVal);
	WriteToFileInt(retVal,viewport.w,viewport.h);
#endif
	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
	glColor3f(.5,.3,0);
	glEnable(GL_COLOR_MATERIAL);
	Float4 size = Float4(2*camera.leftcp, 2*camera.rightcp, 2*camera.bottomcp, 2*camera.topcp);

	glStencilFunc(GL_NOTEQUAL, 0, 1);
	DrawPlane(equation,size);

	glDisable(GL_STENCIL_TEST);
	glDisable(GL_COLOR_MATERIAL);
	//	glEnable(clipPlaneID);
}


void ClipQuarterObject()
{
	double equation0[4] = {-1, 0, 0, 0};
	double equation1[4] = {1, 0, 0, 0};
	double equation2[4] = {0, 0, -1, 0};	
	double equation3[4] = {0, 0, 1, 0};	

	glPushMatrix();
	glTranslated(clipPlaneDistX, 0, 0);
	glClipPlane(GL_CLIP_PLANE0, equation0);
	glPopMatrix();

	glPushMatrix();
	glTranslated(clipPlaneDistX, 0, 0);
	glClipPlane(GL_CLIP_PLANE1, equation1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ);
	glClipPlane(GL_CLIP_PLANE2, equation2);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ);
	glClipPlane(GL_CLIP_PLANE3, equation3);
	glPopMatrix();

	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 0x1, 0x1);
	glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE3);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE3);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
	glColor3f(.5,.4,0);

	GLfloat mat_ambient[]  = {objKA*0.5, objKA*0.4, 0.0, 1.0 };
	GLfloat mat_diffuse[]  = {0.5, 0.4, 0.0, 1.0 };
	GLfloat mat_specular[] = {0.5, 0.4, 0.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT , mat_ambient);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, objShininess);	

#ifdef BREP
	Float3 bBoxMin = objects[0]->bBoxMin;
	Float3 bBoxMax = objects[0]->bBoxMax;
#else
	Float3 bBoxMin = Float3(-sceneSize, -sceneSize, -sceneSize);
	Float3 bBoxMax = Float3(sceneSize, sceneSize, sceneSize);
#endif

	glStencilFunc(GL_NOTEQUAL, 0, 1);

	//	glEnable(GL_POLYGON_OFFSET_FILL);
	//	glPolygonOffset(1,1);

	float clipPlaneZ = clipPlaneDistZ + 0.00;
	float clipPlaneX = clipPlaneDistX - 0.00;
	glBegin(GL_QUADS);
	glNormal3f(1,0,0);
	glVertex3f(clipPlaneX, bBoxMin[1], clipPlaneZ);
	glVertex3f(clipPlaneX, bBoxMin[1], bBoxMax[2]);
	glVertex3f(clipPlaneX, bBoxMax[1], bBoxMax[2]);
	glVertex3f(clipPlaneX, bBoxMax[1], clipPlaneZ);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0,0,1);
	glVertex3f(clipPlaneX , bBoxMin[1], clipPlaneZ);
	glVertex3f(bBoxMax[0] , bBoxMin[1], clipPlaneZ);
	glVertex3f(bBoxMax[0] , bBoxMax[1], clipPlaneZ);
	glVertex3f(clipPlaneX , bBoxMax[1], clipPlaneZ);
	glEnd();

	//	glDisable(GL_POLYGON_OFFSET_FILL);

	glDisable(GL_STENCIL_TEST);
	glDisable(GL_COLOR_MATERIAL);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE3);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE3);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE3);

	// Set The Blending Function For Translucency
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_ALPHA);	

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, glParam->displayLevel/20.0);

	glDisable(GL_BLEND);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE3);


	// Draw actual floor
	//glPushMatrix();
	//glRotatef(90,-1,0,0);
	//DrawFloor(-5000, 5000, -5000, 5000, bBoxMin[2], false);
	//glPopMatrix();
	
}


/*
void ClipQuarterObject()
{
	double equation0[4] = {-1, 0, 0, 0};
	double equation1[4] = {1, 0, 0, 0};
	double equation2[4] = {0, 0, -1, 0};	
	double equation3[4] = {0, 0, 1, 0};	

	glPushMatrix();
	glTranslated(clipPlaneDistX, 0, 0);
	glClipPlane(GL_CLIP_PLANE0, equation0);
	glPopMatrix();

	glPushMatrix();
	glTranslated(clipPlaneDistX, 0, 0);
	glClipPlane(GL_CLIP_PLANE1, equation1);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ-10);
	glClipPlane(GL_CLIP_PLANE0, equation2);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ);
	glClipPlane(GL_CLIP_PLANE1, equation3);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ);
	glClipPlane(GL_CLIP_PLANE2, equation2);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ);
	glClipPlane(GL_CLIP_PLANE3, equation3);
	glPopMatrix();

	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS, 0x1, 0x1);
	glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE3);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE3);
	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
	glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
	glColor3f(.5,.4,0);
	//glColor3f(1,1,1);

//	GLfloat mat_ambient[]  = {objKA*0.5, objKA*0.4, 0.0, 1.0 };
	GLfloat mat_ambient[]  = {1.0, 0.8, 0.0, 1.0 };
//	GLfloat mat_diffuse[]  = {0.5, 0.4, 0.0, 1.0 };
//	GLfloat mat_specular[] = {0.5, 0.4, 0.0, 1.0 };
	GLfloat mat_diffuse[]  = {0.5, 0.4, 0.0, 1.0 };
	GLfloat mat_specular[] = {1.0, 1.0, 0.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT , mat_ambient);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, objShininess);	

#ifdef BREP
	Float3 bBoxMin = objects[0]->bBoxMin;
	Float3 bBoxMax = objects[0]->bBoxMax;
#else
	Float3 bBoxMin = Float3(-sceneSize, -sceneSize, -sceneSize);
	Float3 bBoxMax = Float3(sceneSize, sceneSize, sceneSize);
#endif

	glStencilFunc(GL_NOTEQUAL, 0, 1);

	//	glEnable(GL_POLYGON_OFFSET_FILL);
	//	glPolygonOffset(1,1);

	float clipPlaneZ = clipPlaneDistZ + 0.00;
	float clipPlaneX = clipPlaneDistX - 0.00;
	glBegin(GL_QUADS);
	glNormal3f(1,0,0);
	glVertex3f(bBoxMin[0], bBoxMin[1], clipPlaneZ-40);
	glVertex3f(bBoxMax[0], bBoxMin[1], clipPlaneZ-40);
	glVertex3f(bBoxMax[0], bBoxMax[1], clipPlaneZ-40);
	glVertex3f(bBoxMin[0], bBoxMax[1], clipPlaneZ-40);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0,0,1);
	glVertex3f(bBoxMin[0] , bBoxMin[1], clipPlaneZ);
	glVertex3f(bBoxMax[0] , bBoxMin[1], clipPlaneZ);
	glVertex3f(bBoxMax[0] , bBoxMax[1], clipPlaneZ);
	glVertex3f(bBoxMin[0] , bBoxMax[1], clipPlaneZ);
	glEnd();

	//	glDisable(GL_POLYGON_OFFSET_FILL);

	glDisable(GL_STENCIL_TEST);
	glDisable(GL_COLOR_MATERIAL);
/*
	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE3);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE3);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE3);

	// Set The Blending Function For Translucency
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_ALPHA);	

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, glParam->displayLevel/20.0);

	glDisable(GL_BLEND);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE3);

}

*/
/*
void ClipQuarterObject()
{
	double equation0[4] = {0, 0, -1, 0};	
	double equation1[4] = {0, 0, 1, 0};	

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ+10);
	glClipPlane(GL_CLIP_PLANE0, equation0);
	glPopMatrix();

	glPushMatrix();
	glTranslated(0, 0, clipPlaneDistZ);
	glClipPlane(GL_CLIP_PLANE1, equation1);
	glPopMatrix();

	//glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
	//glDisable(GL_DEPTH_TEST);
	//glEnable(GL_STENCIL_TEST);
	//glStencilFunc(GL_ALWAYS, 0x1, 0x1);
	//glStencilOp(GL_KEEP, GL_KEEP, GL_INVERT);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE1);
	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE1);

	//glEnable(GL_DEPTH_TEST);
	//glClear(GL_DEPTH_BUFFER_BIT);
	//glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);
	glColor3f(.5,.4,0);
	//glColor3f(1,1,1);

	//GLfloat mat_ambient[]  = {1.0, 0.8, 0.0, 1.0 };
	//GLfloat mat_diffuse[]  = {0.5, 0.4, 0.0, 1.0 };
	//GLfloat mat_specular[] = {1.0, 1.0, 0.0, 1.0 };
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , mat_diffuse);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT , mat_ambient);
	//glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, objShininess);	

#ifdef BREP
	Float3 bBoxMin = objects[0]->bBoxMin;
	Float3 bBoxMax = objects[0]->bBoxMax;
#else
	Float3 bBoxMin = Float3(-sceneSize, -sceneSize, -sceneSize);
	Float3 bBoxMax = Float3(sceneSize, sceneSize, sceneSize);
#endif

	//glStencilFunc(GL_NOTEQUAL, 0, 1);

	//	glEnable(GL_POLYGON_OFFSET_FILL);
	//	glPolygonOffset(1,1);
/*
	float clipPlaneZ = clipPlaneDistZ + 0.00;
	float clipPlaneX = clipPlaneDistX - 0.00;
	glBegin(GL_QUADS);
	glNormal3f(1,0,0);
	glVertex3f(bBoxMin[0], bBoxMin[1], clipPlaneZ-40);
	glVertex3f(bBoxMax[0], bBoxMin[1], clipPlaneZ-40);
	glVertex3f(bBoxMax[0], bBoxMax[1], clipPlaneZ-40);
	glVertex3f(bBoxMin[0], bBoxMax[1], clipPlaneZ-40);
	glEnd();

	glBegin(GL_QUADS);
	glNormal3f(0,0,1);
	glVertex3f(bBoxMin[0] , bBoxMin[1], clipPlaneZ);
	glVertex3f(bBoxMax[0] , bBoxMin[1], clipPlaneZ);
	glVertex3f(bBoxMax[0] , bBoxMax[1], clipPlaneZ);
	glVertex3f(bBoxMin[0] , bBoxMax[1], clipPlaneZ);
	glEnd();

	//	glDisable(GL_POLYGON_OFFSET_FILL);

	glDisable(GL_STENCIL_TEST);
	glDisable(GL_COLOR_MATERIAL);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE0);
	glEnable(GL_CLIP_PLANE3);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_CLIP_PLANE3);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE2);

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE2);

	glEnable(GL_CLIP_PLANE1);
	glEnable(GL_CLIP_PLANE3);

	// Set The Blending Function For Translucency
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_DST_ALPHA);
	//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_ALPHA);	

	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, glParam->displayLevel/20.0);

	glDisable(GL_BLEND);

	glDisable(GL_CLIP_PLANE1);
	glDisable(GL_CLIP_PLANE3);

}


*/

void DrawVolume(float volume, float maxVolume)
{
	float overlaySizeX = 25;
	float overlaySizeY = 100;
	float volumeFraction = (volume - sceneVolume)/ (sceneVolume*1.25);


	Float4 pos = Float4(viewport.w - overlaySizeX - 50, viewport.h - overlaySizeY - 50, overlaySizeX, overlaySizeY);
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glViewport(pos[0], pos[1], pos[2], pos[3]);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_COLOR_MATERIAL);

	glBegin(GL_QUADS);
	glNormal3f(0,0,-1);
	glColor4f(.9, .9, .9, 1.0);
	glVertex3f(-1, -1, 1);
	glVertex3f( 1, -1, 1);
	glVertex3f( 1,  1, 1);
	glVertex3f(-1,  1, 1);

	glColor4f(.6, .3, .5, 1.0);
	glVertex3f(-1, -1, 1);
	glVertex3f( 1, -1, 1);
	glVertex3f( 1,  volumeFraction, 1);
	glVertex3f(-1,  volumeFraction, 1);

	glEnd();

	// Restore the previous views
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

void Display()
{
	if (!evaluated && glParam->displayNURBS)
	{
		EvaluateSurfaces(-1, -1, false);
		evaluated = true;
		cleared = false;
	}

	// Generate Surface IDs
	//	GLuint surfaceIDTex;
	//	glGenTextures(1, &surfaceIDTex);
	//	GenerateSurfaceID(nurbsSurfaces, objects, camera, viewport, surfaceIDTex, glParam);


	//Setup the viewport
	glViewport (0, 0, viewport.w, viewport.h);

	float backgroundTransparency = 1-glParam->displayLevel/20.0;
	glClearColor(0,0,0,0);
	if (backgroundColor == 1)
		glClearColor(1, 1, 1, backgroundTransparency);

	// Clear the background
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);	// Clear Screen And Depth Buffer


	Float4 bottomColor	= Float4(0.9, 0.9, 0.9 ,backgroundTransparency);
	Float4 topColor		= Float4(.35, 0.7, 1.0 ,backgroundTransparency);
	if (backgroundColor == 2)
		DrawBackground(bottomColor,topColor);

	glEnable(GL_DEPTH_TEST);
	// The Type Of Depth Testing To Do
	glDepthFunc(GL_LEQUAL);	

	if (glParam->smooth)
		glShadeModel(GL_SMOOTH);
	else
		glShadeModel(GL_FLAT);

	if (glParam->wireframe)
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	else
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

	// Apply all the lights
	glEnable(GL_LIGHTING);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glEnable(GL_NORMALIZE);
//	glEnable(GL_RESCALE_NORMAL);

	// If fixed view
	if (fixedView)
	{
		glLoadIdentity();
		glMultMatrixf(fixedViewMatrix);
	}

	if (reflectionPlane > 0)
		DrawReflection();

	// Draw all the objects
	for (int i=0; i<lights.size(); i++)
		lights[i].Apply();

#ifdef DRAWEDGES
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1,1);
#endif

#ifdef MSAA
	glEnable(GL_MULTISAMPLE_ARB);
#endif

	if (clipping)
		ClipQuarterObject();
	else
	{
		for (int i=0; i<objects.size(); i++)
			objects[i]->DrawSceneObject(glParam, false, 1.0);
	}

#ifdef COMPUTEMOMENTS
	if (objects.size() > 0)
		if (objects[0]->massCenterComputed)
			DrawVolume(objects[0]->volume, pow(sceneSize/2.0, 3));
#endif

#ifdef DRAWEDGES
	glDisable(GL_POLYGON_OFFSET_FILL);
	//Draw Outline
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	glEnable(GL_COLOR_MATERIAL);
	glColor4f(0,1,0,1);
	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);
	glDisable(GL_COLOR_MATERIAL);
#endif

	if (evaluated)
	{
		for (int i=0; i<nurbsSurfaces.size(); i++)
			if (nurbsSurfaces[i]->visibilityFactor >= glParam->surfVisibilityCutOff)
				nurbsSurfaces[i]->DrawNURBS(glParam, 1.0);
	}

	if (glParam->closestPointComputed)
	{
		Float3 markColor = Float3(1,0,0);
		MarkClosestPoints(glParam->point1, glParam->point2, markColor, true);
	}


#ifdef DEBUG
	//iddo: @@ for debug rendering - projected points were copied into evaluatedPoints2
	if (glParam->closestPointComputed) 
	{
		float * evaluatedPoints1 = nurbsSurfaces[0]->evalParams->evaluatedPoints;
		float * evaluatedPoints2 = nurbsSurfaces[1]->evalParams->evaluatedPoints;
		int uNum = int(intRes*pow(2.0,glParam->displayLevel));
		int vNum = uNum;

		for(int j=0; j < vNum+1; j++)
		{
			for(int i=0; i < uNum+1; i++)
			{
				float wij = evaluatedPoints1[4*(j*(uNum+1)+i)+3];
				Float3 point1(evaluatedPoints1[4*(j*(uNum+1)+i)+0]/wij,
					evaluatedPoints1[4*(j*(uNum+1)+i)+1]/wij,
					evaluatedPoints1[4*(j*(uNum+1)+i)+2]/wij);
				Float3 point2(evaluatedPoints2[4*(j*(uNum+1)+i)+0],
					evaluatedPoints2[4*(j*(uNum+1)+i)+1],
					evaluatedPoints2[4*(j*(uNum+1)+i)+2]);
				Float3 markColor = Float3(0,1,0);
				MarkClosestPoints(point1, point2, markColor);
			}
		}
	}
#endif

	if (evaluated && nurbsSurfaces.size()>0)
	{
		int overlaySize = 128;

		if (nurbsSurfaces.size() > intSurf1 && nurbsSurfaces.size() > intSurf2)
		{
			if (nurbsSurfaces[intSurf1]->evaluatedIntersection || nurbsSurfaces[intSurf2]->evaluatedIntersection)
			{
				Float4 pos = Float4(viewport.w - overlaySize, viewport.h - overlaySize - 50, overlaySize, overlaySize);
				if (nurbsSurfaces[intSurf1]->textured)
					nurbsSurfaces[intSurf1]->DrawOverlayTrimTexture(glParam,pos);
				pos = Float4(viewport.w - overlaySize, viewport.h - 2*overlaySize - 100, overlaySize, overlaySize);
				if (nurbsSurfaces[intSurf2]->textured)
					nurbsSurfaces[intSurf2]->DrawOverlayTrimTexture(glParam,pos);
			}
		}
		else if (glParam->pickedSurfNum!=-1)
		{
			Float4 pos = Float4(viewport.w - overlaySize, viewport.h - overlaySize - 50, overlaySize, overlaySize);
			nurbsSurfaces[glParam->pickedSurfNum]->DrawOverlayTrimTexture(glParam,pos);
		}
		else if (nurbsSurfaces[0]->textured)
		{
			Float4 pos = Float4(viewport.w - overlaySize, viewport.h - overlaySize - 50, overlaySize, overlaySize);
			nurbsSurfaces[0]->DrawOverlayTrimTexture(glParam,pos);
		}
	}
	if (drawSilhouette)
		DrawSilhouetteDirection();

	if (glParam->drawVoxels)
	{
		for (int i=0; i<objects.size(); i++)
		{
#ifdef DRAWFACEBBOX
			objects[i]->DrawFaceBoundingBoxes(glParam);
#endif
			if (objects[i]->voxelData!=NULL)
			{
				if (glParam->displayLevel == 0)
				{
					if (objects[i]->voxelData->storeBoxData)
					{
#ifdef DISPLAYLISTS
						glCallList(objects[i]->voxelDLid);
#else
						objects[i]->DrawVoxels(glParam);
#endif
					}
					else
						objects[i]->DrawInOutPoints();
				}
				else
				{
					objects[i]->DrawVoxelHierarchy(glParam);
					//objects[i]->DrawOBB();
					//objects[i]->DrawInOutPoints();
				}
			}
		}
	}

	// Finish drawing, update the frame buffer
	// Swap the buffers (remember we are using double buffers)
	//	glFlush();
	glutSwapBuffers();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
}

void StopAnimation()
{
	for (int k=0; k<nurbsSurfaces.size(); k++)
	{
		nurbsSurfaces[k]->identityTransformation = true;;
		MakeIdentityMatrix(nurbsSurfaces[k]->transformationMatrix);
	}

}

void InitializeAnimation()
{
	for (int k=0; k<nurbsSurfaces.size(); k++)
	{
		glParam->collision = true;
		nurbsSurfaces[k]->evalParams = new EvaluationParams();
		nurbsSurfaces[k]->initializedEvalParams = true;
		int uNum = intRes;
		int vNum = intRes;
		nurbsSurfaces[k]->evalParams->uNum	= uNum;
		nurbsSurfaces[k]->evalParams->vNum	= vNum;
		nurbsSurfaces[k]->evalParams->uMin	= 0.0;
		nurbsSurfaces[k]->evalParams->uMax	= 1.0;
		nurbsSurfaces[k]->evalParams->vMin	= 0.0;
		nurbsSurfaces[k]->evalParams->vMax	= 1.0;

		float randScale = 400.0/(RAND_MAX*1.0);
		nurbsSurfaces[k]->transformationMatrix[12] = randScale*rand()-200;
		nurbsSurfaces[k]->transformationMatrix[13] = randScale*rand()-200;
		nurbsSurfaces[k]->transformationMatrix[14] = randScale*rand()-200;
		nurbsSurfaces[k]->identityTransformation = false;;

		glParam->collision = false;
	}
}

void PerformAnimationStep()
{
	for (int k=0; k<nurbsSurfaces.size(); k++)
	{
		nurbsSurfaces[k]->kdColor = Float3(0,1,0);
		float randScale = 400.0/(RAND_MAX*1.0);
		nurbsSurfaces[k]->transformationMatrix[12] = randScale*rand()-200;
		nurbsSurfaces[k]->transformationMatrix[13] = randScale*rand()-200;
		nurbsSurfaces[k]->transformationMatrix[14] = randScale*rand()-200;
	}

	for (int j=0; j<nurbsSurfaces.size(); j++)
	{
		for (int i=0; i<nurbsSurfaces.size(); i++)
		{
			if (i!=j)
			{
				bool collision = DetectCollision(nurbsSurfaces[i],nurbsSurfaces[j],intRes,glParam,true);
				if (collision)
				{
					nurbsSurfaces[i]->kdColor = Float3(1,0,0);
					nurbsSurfaces[j]->kdColor = Float3(1,0,0);
				}
			}
		}
	}

}

void EvaluateSilhouette()
{
	bool fitSilhouette = false;
	bool timing = false;
	if (timing)
		timer.Start();

	for (int i = 0; i < 1; i++)
	{
		silhouetteVector = Float3(glParam->modelViewMatrix[2], glParam->modelViewMatrix[6], glParam->modelViewMatrix[10]);
		silhouetteVector = Float3(glParam->modelViewMatrix[1], glParam->modelViewMatrix[5], glParam->modelViewMatrix[9]);
		if (glParam->pickedSurfNum > -1)
			nurbsSurfaces[glParam->pickedSurfNum]->SurfaceSilhouetteEvaluation(silhouetteVector, glParam, silRes, fitSilhouette);
		else
			for (int j=0; j<nurbsSurfaces.size(); j++)
				nurbsSurfaces[j]->SurfaceSilhouetteEvaluation(silhouetteVector, glParam, silRes, fitSilhouette);

		//		if (glParam->pickedSurfNum > -1)
		//			nurbsSurfaces[glParam->pickedSurfNum]->SurfaceCurvatureEvaluation(silRes, glParam);
		//		else
		//			for (int j=0; j<nurbsSurfaces.size(); j++)
		//				nurbsSurfaces[j]->SurfaceCurvatureEvaluation(silRes, glParam);
	}

	if (timing)
	{
		timer.Stop();
		int silhouetteTime = timer.GetElapsedTimeInMilliSec();
		cout << "Silhouette Time : " << silhouetteTime*0.001 << endl << endl;
	}
}

void AutomatedTranslationHD()
{
	ofstream ofs;
	ofs.open("Timings.txt");
	if (!ofs.good())
		cerr<<"Unable to open file Timings.txt"<<endl;

	float point1[3], point2[3];
	point1[0]=point1[1]=point1[2]=0;
	point2[0]=point2[1]=point2[2]=0;
	float hausdorffDist = -1;
	float K1 = 0;
	float K2 = 0;
	float K  = 0;
	NRKA = 0;
	float totalTime = 0;
	float converged = 0;

	nurbsSurfaces[intSurf2]->identityTransformation = false;
#ifdef TEAPOTHD
	nurbsSurfaces[5]->identityTransformation = false;
	nurbsSurfaces[6]->identityTransformation = false;
	nurbsSurfaces[7]->identityTransformation = false;
#endif
#ifdef DUCKYHD
	nurbsSurfaces[4]->identityTransformation = false;
	nurbsSurfaces[5]->identityTransformation = false;
#endif

	float currentXPos = -1250;
	float currentYPos = -100;
	float currentZPos = 100;
	int currentResolution = int(intRes*pow(2.0,glParam->displayLevel));

#ifdef CUDA
#ifdef SURFACEHD
	if (computeNRHD)
		hausdorffDist = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, &converged, currentResolution, NRKB, glParam, false);
	else
		hausdorffDist = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, currentResolution, glParam, false);

	K1 = nurbsSurfaces[intSurf1]->evalParams->maxExpansionFactor;
	K2 = nurbsSurfaces[intSurf2]->evalParams->maxExpansionFactor;
	K = K1+K2;
	if (recomputeNRKA)
	{
		float resol = intRes*pow(2.0,glParam->displayLevel);
		float du,dv;
		du = dv = 1.0/resol; //actually should be knotSpan/resol
		NRKA = ComputeKsBound(nurbsSurfaces[intSurf1], du, dv);
	}
#endif
#ifdef DUCKYHD
		float tempMinDist[3];
		tempMinDist[0] = -1; 
		tempMinDist[1] = -1; 
		tempMinDist[2] = -1; 
		float pointHD1[3][3], pointHD2[3][3];
		float convergedHD[3];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				float tempPoint1[3], tempPoint2[3];
				float tempHD;
				float tempConvergence = 0;
				if (computeNRHD)
					tempHD = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[j+3], nurbsSurfaces[i], tempPoint1, tempPoint2, &tempConvergence, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
				else
					tempHD = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[i], nurbsSurfaces[j+3], tempPoint1, tempPoint2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);

				if (tempMinDist[j] == -1 || tempHD < tempMinDist[j])
				{
					tempMinDist[j] = tempHD;
					convergedHD[j] = tempConvergence;
					pointHD1[j][0] = tempPoint1[0];
					pointHD1[j][1] = tempPoint1[1];
					pointHD1[j][2] = tempPoint1[2];
					pointHD2[j][0] = tempPoint2[0];
					pointHD2[j][1] = tempPoint2[1];
					pointHD2[j][2] = tempPoint2[2];
				}
			}
		}
		for (int i = 0; i < 3; i++)
		{
			if (recomputeNRKA)
			{
				float resol = intRes*pow(2.0,glParam->displayLevel);
				float du,dv;
				du = dv = 1.0/resol; //actually should be knotSpan/resol
				NRKA = __max(NRKA, ComputeKsBound(nurbsSurfaces[i], du, dv));
			}
			K1 = __max(K1, nurbsSurfaces[i]->evalParams->maxExpansionFactor);  
			K2 = __max(K2, nurbsSurfaces[i+3]->evalParams->maxExpansionFactor);  
			if (hausdorffDist == -1 || tempMinDist[i] > hausdorffDist)
			{
				hausdorffDist = tempMinDist[i];
				converged = convergedHD[i];
				point1[0] = pointHD1[i][0];
				point1[1] = pointHD1[i][1];
				point1[2] = pointHD1[i][2];
				point2[0] = pointHD2[i][0];
				point2[1] = pointHD2[i][1];
				point2[2] = pointHD2[i][2];
			}
		}
#endif
#ifdef TEAPOTHD
		float tempMinDist[4];
		tempMinDist[0] = -1; 
		tempMinDist[1] = -1; 
		tempMinDist[2] = -1; 
		tempMinDist[3] = -1; 
		float pointHD1[4][3], pointHD2[4][3];
		float convergedHD[4];
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				float tempPoint1[3], tempPoint2[3];
				float tempHD;
				float tempConvergence = 0;
				if (computeNRHD)
					tempHD = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[j+4], nurbsSurfaces[i], tempPoint1, tempPoint2, &tempConvergence, currentResolution, NRKB, glParam, false);
				else	
					tempHD = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[i], nurbsSurfaces[j+4], tempPoint1, tempPoint2, currentResolution, glParam, false);

				if (tempMinDist[j] == -1 || tempHD < tempMinDist[j])
				{
					tempMinDist[j] = tempHD;
					convergedHD[j] = tempConvergence;
					pointHD1[j][0] = tempPoint1[0];
					pointHD1[j][1] = tempPoint1[1];
					pointHD1[j][2] = tempPoint1[2];
					pointHD2[j][0] = tempPoint2[0];
					pointHD2[j][1] = tempPoint2[1];
					pointHD2[j][2] = tempPoint2[2];
				}
			}
		}
		for (int i = 0; i < 4; i++)
		{
			if (recomputeNRKA)
			{
				float resol = intRes*pow(2.0,glParam->displayLevel);
				float du,dv;
				du = dv = 1.0/resol; //actually should be knotSpan/resol
				NRKA = __max(NRKA, ComputeKsBound(nurbsSurfaces[i], du, dv));
			}
			K1 = __max(K1, nurbsSurfaces[i]->evalParams->maxExpansionFactor);  
			K2 = __max(K2, nurbsSurfaces[i+4]->evalParams->maxExpansionFactor);  
			if (hausdorffDist == -1 || tempMinDist[i] > hausdorffDist)
			{
				hausdorffDist = tempMinDist[i];
				converged = convergedHD[i];
				point1[0] = pointHD1[i][0];
				point1[1] = pointHD1[i][1];
				point1[2] = pointHD1[i][2];
				point2[0] = pointHD2[i][0];
				point2[1] = pointHD2[i][1];
				point2[2] = pointHD2[i][2];
			}
		}
#endif
#else
	hausdorffDist = GetSurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, currentResolution, glParam, false);
#endif

	for (int numRuns = 0; numRuns < 100; numRuns++)
	{
		hausdorffDist = -1;
		float randXPos = rand()*2500.0/(RAND_MAX*1.0)-1250;
		float randYPos = rand()*2500.0/(RAND_MAX*1.0)-1250;
		float randZPos = rand()*2500.0/(RAND_MAX*1.0)-1250;
		nurbsSurfaces[intSurf2]->transformationMatrix[12] = currentXPos;
		nurbsSurfaces[intSurf2]->transformationMatrix[13] = currentYPos;
		nurbsSurfaces[intSurf2]->transformationMatrix[14] = currentZPos;
		timer.Start();

#ifdef CUDA
#ifdef SURFACEHD
		if (computeNRHD)
			hausdorffDist = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, &converged, currentResolution, NRKB, glParam, false);
		else
			hausdorffDist = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, currentResolution, glParam, false);
#endif
#ifdef TEAPOTHD
		nurbsSurfaces[5]->transformationMatrix[12] = currentXPos;
		nurbsSurfaces[5]->transformationMatrix[13] = currentYPos;
		nurbsSurfaces[5]->transformationMatrix[14] = currentZPos;

		nurbsSurfaces[6]->transformationMatrix[12] = currentXPos;
		nurbsSurfaces[6]->transformationMatrix[13] = currentYPos;
		nurbsSurfaces[6]->transformationMatrix[14] = currentZPos;

		nurbsSurfaces[7]->transformationMatrix[12] = currentXPos;
		nurbsSurfaces[7]->transformationMatrix[13] = currentYPos;
		nurbsSurfaces[7]->transformationMatrix[14] = currentZPos;

		float tempMinDist[4];
		tempMinDist[0] = -1; 
		tempMinDist[1] = -1; 
		tempMinDist[2] = -1; 
		tempMinDist[3] = -1; 
		float pointHD1[4][3], pointHD2[4][3];
		float convergedHD[4];
		timer.Start();
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				float tempPoint1[3], tempPoint2[3];
				float tempHD;
				float tempConvergence = 0;
				if (computeNRHD)
					tempHD = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[j+4], nurbsSurfaces[i], tempPoint1, tempPoint2, &tempConvergence, currentResolution, NRKB, glParam, false);
				else
					tempHD = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[i], nurbsSurfaces[j+4], tempPoint1, tempPoint2, currentResolution, glParam, false);

				if (tempMinDist[j] == -1 || tempHD < tempMinDist[j])
				{
					tempMinDist[j] = tempHD;
					convergedHD[j] = tempConvergence;
					pointHD1[j][0] = tempPoint1[0];
					pointHD1[j][1] = tempPoint1[1];
					pointHD1[j][2] = tempPoint1[2];
					pointHD2[j][0] = tempPoint2[0];
					pointHD2[j][1] = tempPoint2[1];
					pointHD2[j][2] = tempPoint2[2];
				}
			}
		}
		for (int i = 0; i < 4; i++)
		{
			if (hausdorffDist == -1 || tempMinDist[i] > hausdorffDist)
			{
				hausdorffDist = tempMinDist[i];
				converged = convergedHD[i];
				point1[0] = pointHD1[i][0];
				point1[1] = pointHD1[i][1];
				point1[2] = pointHD1[i][2];
				point2[0] = pointHD2[i][0];
				point2[1] = pointHD2[i][1];
				point2[2] = pointHD2[i][2];
			}
		}
#endif
#ifdef DUCKYHD
		nurbsSurfaces[4]->transformationMatrix[12] = currentXPos;
		nurbsSurfaces[4]->transformationMatrix[13] = currentYPos;
		nurbsSurfaces[4]->transformationMatrix[14] = currentZPos;

		nurbsSurfaces[5]->transformationMatrix[12] = currentXPos;
		nurbsSurfaces[5]->transformationMatrix[13] = currentYPos;
		nurbsSurfaces[5]->transformationMatrix[14] = currentZPos;

		float tempMinDist[3];
		tempMinDist[0] = -1; 
		tempMinDist[1] = -1; 
		tempMinDist[2] = -1; 
		float pointHD1[3][3], pointHD2[3][3];
		float convergedHD[3];
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				float tempPoint1[3], tempPoint2[3];
				float tempHD;
				float tempConvergence = 0;
				if (computeNRHD)
					tempHD = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[j+3], nurbsSurfaces[i], tempPoint1, tempPoint2, &tempConvergence, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
				else
					tempHD = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[i], nurbsSurfaces[j+3], tempPoint1, tempPoint2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);

				if (tempMinDist[j] == -1 || tempHD < tempMinDist[j])
				{
					tempMinDist[j] = tempHD;
					convergedHD[j] = tempConvergence;
					pointHD1[j][0] = tempPoint1[0];
					pointHD1[j][1] = tempPoint1[1];
					pointHD1[j][2] = tempPoint1[2];
					pointHD2[j][0] = tempPoint2[0];
					pointHD2[j][1] = tempPoint2[1];
					pointHD2[j][2] = tempPoint2[2];
				}
			}
		}
		for (int i = 0; i < 3; i++)
		{
			if (hausdorffDist == -1 || tempMinDist[i] > hausdorffDist)
			{
				hausdorffDist = tempMinDist[i];
				converged = convergedHD[i];
				point1[0] = pointHD1[i][0];
				point1[1] = pointHD1[i][1];
				point1[2] = pointHD1[i][2];
				point2[0] = pointHD2[i][0];
				point2[1] = pointHD2[i][1];
				point2[2] = pointHD2[i][2];
			}
		}
#endif
#else
		hausdorffDist = GetSurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, currentResolution, glParam, false);
		//iddo:
		//hausdorffDist = GetSurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, currentResolution, glParam, false);
#endif

		timer.Stop();
		//ofs << timer.GetElapsedTimeInMilliSec()  << endl;
		//cout << randXPos << ", " << randYPos << ", " << randZPos << endl;
		if (computeNRHD)
			ofs << timer.GetElapsedTimeInMilliSec()  << " " << hausdorffDist << " " << converged << endl;
		else
			ofs << timer.GetElapsedTimeInMilliSec()  << " " << hausdorffDist << endl;
		//ofs << currentResolution << " " << currentXPos << " " << hausdorffDist << " "  <<  timer.GetElapsedTimeInMilliSec() << endl;
		currentXPos+=25;
	}

	glParam->closestPointComputed = true;
	glParam->point1 = Float3(point1[0], point1[1], point1[2]);
	glParam->point2 = Float3(point2[0], point2[1], point2[2]);

	cout<<"Hausdorff Distance  : "<< hausdorffDist << endl;
	cout<<"Resolution          : "<< int(intRes*pow(2.0,glParam->displayLevel)) << endl;
	cout<<"BBox K Tolerance    : "<< 0.000001 * int((K)*1000000) << endl;
	cout<<"NR KA Tolerance     : "<< NRKA << endl;
	cout<<"NR KB Tolerance     : "<< NRKB << endl;
	cout<<"Point1              : "<< glParam->point1[0] << ", " << glParam->point1[1] << ", " << glParam->point1[2] << endl;
	cout<<"Point2              : "<< glParam->point2[0] << ", " << glParam->point2[1] << ", " << glParam->point2[2] << endl;
	cout<<"Translation         : "<< nurbsSurfaces[intSurf2]->transformationMatrix[12] << ", " << nurbsSurfaces[intSurf2]->transformationMatrix[13] << ", " << nurbsSurfaces[intSurf2]->transformationMatrix[14] << endl  << endl;	

	ofs.close();
}

void ComputeMinDistance()
{
	float point1[3], point2[3];
	float minDist = -1;
	float K;
	timer.Start();
#ifdef SURFACEMETRICS
	if (nurbsSurfaces.size() > 1)
		minDist = GetSurfaceClosestPoint(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, intRes, glParam, false);
#else
	if (!objects[selectedObject2]->identityTransformation)
	{
		for (int i = 0; i < objects[selectedObject2]->faces.size(); i++)
		{
			if (objects[selectedObject2]->faces[i]->isNURBS)
			{
				CopyTransformationMatrix(objects[selectedObject2]->transformationMatrix, objects[selectedObject2]->faces[i]->surface->transformationMatrix);
				objects[selectedObject2]->faces[i]->surface->identityTransformation = false;
			}
		}
	}
	minDist = GetObjectClosestPoint(objects[selectedObject1], objects[selectedObject2], point1, point2, &K, intRes, glParam, true);
#endif
	timer.Stop();

	float K1 = nurbsSurfaces[intSurf1]->evalParams->maxExpansionFactor;
	float K2 = nurbsSurfaces[intSurf2]->evalParams->maxExpansionFactor;
	K = K1 + K2;
	float closestPointTime = int(timer.GetElapsedTimeInMicroSec())*0.001;

#ifdef INTERACTIVETIMING
	ofs << closestPointTime << " " << minDist << " " << K << endl;
#endif

	glParam->closestPointComputed = true;
	glParam->point1 = Float3(point1[0], point1[1], point1[2]);
	glParam->point2 = Float3(point2[0], point2[1], point2[2]);

	cout<<"Computation Time   : "<< closestPointTime << endl;
	cout<<"Minimum Distance   : "<< minDist << endl;
	cout<<"Distance Tolerance : "<< 0.000001 * int((K)*1000000) << endl;
	cout<<"Position Tolerance : "<< 0.001 * int(sqrt(minDist*K)*1000) << endl;
	cout<<"Distance Check     : "<< Distance(glParam->point1, glParam->point2) << endl;
	cout<<"Point1             : "<< glParam->point1[0] << ", " << glParam->point1[1] << ", " << glParam->point1[2] << endl;
	cout<<"Point2             : "<< glParam->point2[0] << ", " << glParam->point2[1] << ", " << glParam->point2[2] << endl;
#ifdef SURFACEMETRICS
	cout<<"Translation        : "<< nurbsSurfaces[intSurf2]->transformationMatrix[12] << ", " << nurbsSurfaces[intSurf2]->transformationMatrix[13] << ", " << nurbsSurfaces[intSurf2]->transformationMatrix[14] << endl  << endl;	
#else
	cout<<"Translation        : "<< objects[selectedObject2]->transformationMatrix[12] << ", " << objects[selectedObject2]->transformationMatrix[13] << ", " << objects[selectedObject2]->transformationMatrix[14] << endl  << endl;
#endif
}

void ComputeHausdorffDistance()
{
	if (nurbsSurfaces.size() <= 1)
		return;
	
	float point1[3], point2[3];
	point1[0]=point1[1]=point1[2]=0;
	point2[0]=point2[1]=point2[2]=0;
	float hausdorffDist = -1;
	float K = 0;
	float K1 = 0;
	float K2 = 0;
	NRKA = 0;
	float converged = 0;

#ifdef DUCKYHD
	timer.Start();
	float tempMinDist[3];
	tempMinDist[0] = -1; 
	tempMinDist[1] = -1; 
	tempMinDist[2] = -1; 
	float pointHD1[3][3], pointHD2[3][3];
	float convergedHD[3];
	for (int j = 0; j < 3; j++)
	{
		for (int i = 0; i < 3; i++)
		{
			float tempPoint1[3], tempPoint2[3];
			float tempHD;
			float tempConvergence;
			if (computeNRHD)
				tempHD = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[i], nurbsSurfaces[j+3], tempPoint1, tempPoint2, &tempConvergence, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
			else
				tempHD = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[j+3], nurbsSurfaces[i], tempPoint1, tempPoint2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);

			if (tempMinDist[i] == -1 || tempHD < tempMinDist[i])
			{
				tempMinDist[i] = tempHD;
				convergedHD[i] = tempConvergence;
				pointHD1[i][0] = tempPoint1[0];
				pointHD1[i][1] = tempPoint1[1];
				pointHD1[i][2] = tempPoint1[2];
				pointHD2[i][0] = tempPoint2[0];
				pointHD2[i][1] = tempPoint2[1];
				pointHD2[i][2] = tempPoint2[2];
			}
		}
	}
	for (int i = 0; i < 3; i++)
	{
		if (recomputeNRKA)
		{
			float resol = intRes*pow(2.0,glParam->displayLevel);
			float du,dv;
			du = dv = 1.0/resol; //actually should be knotSpan/resol
			NRKA = __max(NRKA, ComputeKsBound(nurbsSurfaces[i], du, dv));
		}
		K1 = __max(K1, nurbsSurfaces[i]->evalParams->maxExpansionFactor);  
		K2 = __max(K2, nurbsSurfaces[i+3]->evalParams->maxExpansionFactor);  
		if (hausdorffDist == -1 || tempMinDist[i] > hausdorffDist)
		{
			hausdorffDist = tempMinDist[i];
			converged = convergedHD[i];
			point1[0] = pointHD1[i][0];
			point1[1] = pointHD1[i][1];
			point1[2] = pointHD1[i][2];
			point2[0] = pointHD2[i][0];
			point2[1] = pointHD2[i][1];
			point2[2] = pointHD2[i][2];
		}
	}
	timer.Stop();
	float closestPointTime = int(timer.GetElapsedTimeInMicroSec())*0.001;

	cout<<"Computation Time    : "<< closestPointTime << endl;

#endif
#ifdef TEAPOTHD
	timer.Start();
	float tempMinDist[4];
	tempMinDist[0] = -1; 
	tempMinDist[1] = -1; 
	tempMinDist[2] = -1; 
	tempMinDist[3] = -1; 
	float pointHD1[4][3], pointHD2[4][3];
	float convergedHD[4];
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			float tempPoint1[3], tempPoint2[3];
			float tempHD;
			float tempConvergence;
			if (computeNRHD)
				tempHD = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[j+4], nurbsSurfaces[i], tempPoint1, tempPoint2, &tempConvergence, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
			else
				tempHD = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[i], nurbsSurfaces[j+4], tempPoint1, tempPoint2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);

			if (tempMinDist[j] == -1 || tempHD < tempMinDist[j])
			{
				tempMinDist[j] = tempHD;
				convergedHD[j] = tempConvergence;
				pointHD1[j][0] = tempPoint1[0];
				pointHD1[j][1] = tempPoint1[1];
				pointHD1[j][2] = tempPoint1[2];
				pointHD2[j][0] = tempPoint2[0];
				pointHD2[j][1] = tempPoint2[1];
				pointHD2[j][2] = tempPoint2[2];
			}
		}
	}
	for (int i = 0; i < 4; i++)
	{
		if (recomputeNRKA)
		{
			float resol = intRes*pow(2.0,glParam->displayLevel);
			float du,dv;
			du = dv = 1.0/resol; //actually should be knotSpan/resol
			NRKA = __max(NRKA, ComputeKsBound(nurbsSurfaces[i], du, dv));
		}
		K1 = __max(K1, nurbsSurfaces[i]->evalParams->maxExpansionFactor);  
		K2 = __max(K2, nurbsSurfaces[i+4]->evalParams->maxExpansionFactor);  
		if (hausdorffDist == -1 || tempMinDist[i] > hausdorffDist)
		{
			hausdorffDist = tempMinDist[i];
			converged = convergedHD[i];
			point1[0] = pointHD1[i][0];
			point1[1] = pointHD1[i][1];
			point1[2] = pointHD1[i][2];
			point2[0] = pointHD2[i][0];
			point2[1] = pointHD2[i][1];
			point2[2] = pointHD2[i][2];
		}
	}
	timer.Stop();
	float closestPointTime = int(timer.GetElapsedTimeInMicroSec())*0.001;

	cout<<"Computation Time    : "<< closestPointTime << endl;

#endif
#ifdef SURFACEHD
#ifdef CUDA
	timerBB.Start();
	hausdorffDist = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
	timerBB.Stop();
	cout << "Hausdorff Dist BB   : " << hausdorffDist << endl;
	// iddo:
	// Compute Bounds
	K1 = nurbsSurfaces[intSurf1]->evalParams->maxExpansionFactor;
	K2 = nurbsSurfaces[intSurf2]->evalParams->maxExpansionFactor;
	if (recomputeNRKA)
	{
		float resol = intRes*pow(2.0,glParam->displayLevel);
		float du,dv;
		du = dv = 1.0/resol; //actually should be knotSpan/resol
		NRKA = ComputeKsBound(nurbsSurfaces[intSurf1], du, dv);
	}

	if (computeNRHD)
	{
		timerNR.Start();
		hausdorffDist = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, &converged, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
		timerNR.Stop();
		cout << "Hausdorff Dist NR   : " << hausdorffDist << endl << endl;
	}

#else
	hausdorffDist = GetSurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
	//iddo: the call below is the call that gives the same results as my function (notice the parameter swapped order)
	hausdorffDist = IddoGetSurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
#endif
#ifdef TWOSIDEDHAUSDORFF
		float point3[3], point4[3];
#ifdef CUDA
		float hausdorffDIst2;
		int converged2
		if (computeNRHD)
			hausdorffDist2 = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point3, point4, &converged2, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
		else
			hausdorffDist2 = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point3, point4, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
#else
		float hausdorffDist2 = GetSurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point3, point4, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
#endif
		if (hausdorffDist2 > hausdorffDist)
		{
			point1[0] = point3[0];
			point1[1] = point3[1];
			point1[2] = point3[2];
			point2[0] = point4[0];
			point2[1] = point4[1];
			point2[2] = point4[2];
			minDist = minDist2;
			converged = converged2;
		}
	}
#endif

	closestPointTimeNR = int(timerNR.GetElapsedTimeInMicroSec())*0.001;
	closestPointTimeBB = int(timerBB.GetElapsedTimeInMicroSec())*0.001;
	cout<<"Computation Time BB : "<< closestPointTimeBB << endl;
	cout<<"Computation Time NR : "<< closestPointTimeNR << endl;
#endif

	K = K1 + K2;
	//cout <<  -NRKB << " = -KB <= HD - computed_HD <= KA = " << NR_Ka << endl;

	glParam->closestPointComputed = true;
	glParam->point1 = Float3(point1[0], point1[1], point1[2]);
	glParam->point2 = Float3(point2[0], point2[1], point2[2]);

	cout<<"Converged           : " << converged << endl;
	cout<<"Resolution          : "<< int(intRes*pow(2.0,glParam->displayLevel)) << endl;
	cout<<"BBox K Tolerance    : "<< 0.000001 * int((K)*1000000) << endl;
	cout<<"NR KA Tolerance     : "<< NRKA << endl;
	cout<<"NR KB Tolerance     : "<< NRKB << endl;
	cout<<"Point1              : "<< glParam->point1[0] << ", " << glParam->point1[1] << ", " << glParam->point1[2] << endl;
	cout<<"Point2              : "<< glParam->point2[0] << ", " << glParam->point2[1] << ", " << glParam->point2[2] << endl;
	cout<<"Translation         : "<< nurbsSurfaces[intSurf2]->transformationMatrix[12] << ", " << nurbsSurfaces[intSurf2]->transformationMatrix[13] << ", " << nurbsSurfaces[intSurf2]->transformationMatrix[14] << endl  << endl;	
}



void TransformObjects(Float2 disp, bool rotate)
{
	glMatrixMode(GL_MODELVIEW);
	float viewMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

	Float3 xAxis = Float3(viewMatrix[0], viewMatrix[4], viewMatrix[8]);
	Float3 yAxis = Float3(viewMatrix[1], viewMatrix[5], viewMatrix[9]);
	VectorNormalize(xAxis);
	VectorNormalize(yAxis);

	Float3 localTranslate=(xAxis*disp[0] - yAxis*disp[1])*(modelSize/(100.0*zoomFactor));
	Float3 localAxis = yAxis*disp[0] + xAxis*disp[1];
	VectorNormalize(localAxis);

	glPushMatrix();
	glLoadIdentity();
	if (rotate)
		glRotated(VectorMagnitude(disp),localAxis[0], localAxis[1], localAxis[2]);
	else
		glTranslated(localTranslate[0],localTranslate[1],localTranslate[2]);
#ifdef DYNAMICOBJECTMOVE
	glMultMatrixf(objects[selectedObject2]->transformationMatrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, objects[selectedObject2]->transformationMatrix);	// get the viewMatrix
	objects[selectedObject2]->identityTransformation = false;
	for (int i = 0; i < objects[selectedObject2]->faces.size(); i++)
		if (objects[selectedObject2]->faces[i]->isNURBS)
		{
			CopyTransformationMatrix(objects[selectedObject2]->transformationMatrix, objects[selectedObject2]->faces[i]->surface->transformationMatrix);
			objects[selectedObject2]->faces[i]->surface->identityTransformation = false;
		}
#else
	glMultMatrixf(nurbsSurfaces[intSurf2]->transformationMatrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, nurbsSurfaces[intSurf2]->transformationMatrix);	// get the viewMatrix
	nurbsSurfaces[intSurf2]->identityTransformation = false;
	if (numDucky2 > 0)
	{
		glGetFloatv(GL_MODELVIEW_MATRIX, nurbsSurfaces[4]->transformationMatrix);	// get the viewMatrix
		glGetFloatv(GL_MODELVIEW_MATRIX, nurbsSurfaces[5]->transformationMatrix);	// get the viewMatrix
		nurbsSurfaces[4]->identityTransformation = false;
		nurbsSurfaces[5]->identityTransformation = false;
	}
	if (numTeapot2 > 0)
	{
		glGetFloatv(GL_MODELVIEW_MATRIX, nurbsSurfaces[5]->transformationMatrix);	// get the viewMatrix
		glGetFloatv(GL_MODELVIEW_MATRIX, nurbsSurfaces[6]->transformationMatrix);	// get the viewMatrix
		glGetFloatv(GL_MODELVIEW_MATRIX, nurbsSurfaces[7]->transformationMatrix);	// get the viewMatrix
		nurbsSurfaces[5]->identityTransformation = false;
		nurbsSurfaces[6]->identityTransformation = false;
		nurbsSurfaces[7]->identityTransformation = false;
	}
#endif
	glPopMatrix();

#ifdef INTERACTIVECOLLISION
	timer.Start();
	ObjectCollision(objects[0], objects[1], glParam);
	timer.Stop();
	collisionTime += (timer.GetElapsedTimeInMilliSec())*0.001;
	numCollisionRuns++;
	if (numCollisionRuns%10 == 0)
	{
		cout << " Collision average time : " << collisionTime*.1 << endl;
		numCollisionRuns = 0;
		collisionTime =0;
	}
#endif

	Timer timerInteractive;
	timerInteractive.Start();
	if (computeDistanceMetrics)
	{
#ifdef SURFACEMETRICS
#ifdef COMPUTEHAUSDORFFDISTANCE
		ComputeHausdorffDistance();
#else
		ComputeMinDistance();
#endif
#endif
	}
	else
	{
#ifdef BREP
		if (objects.size() >= 2)
			if (objects[0]->faces[0]->isNURBS && objects[1]->faces[0]->isNURBS)
				CalculateSurfaceIntersection(objects[0]->faces[0]->surface, objects[1]->faces[0]->surface, intRes, glParam, false);
#else
		//for (int j=0; j<nurbsSurfaces.size(); j++)
		//	nurbsSurfaces[j]->SurfaceCurvatureEvaluation(intRes, glParam);
		CalculateSurfaceIntersection(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intRes, glParam, false);
#endif
	}
	timerInteractive.Stop();

	float currentEvaluationTime = int(timer.GetElapsedTimeInMicroSec())*0.001;
	evaluationTime += currentEvaluationTime;
	cout<<"Interactive Time    : "<< currentEvaluationTime << endl << endl;

	numRuns++;
	if (numRuns%10 == 0)
	{
		cout << "Current Average Time: " << evaluationTime*.1 << endl << endl;
		numRuns = 0;
		evaluationTime = 0;
	}
}

// handle ascii keyboard events
void SpecialKeys(int key, int x, int y)
{
	glMatrixMode(GL_MODELVIEW);
	float viewMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

	Float3 xAxis = Float3(viewMatrix[0], viewMatrix[4], viewMatrix[8]);
	Float3 yAxis = Float3(viewMatrix[1], viewMatrix[5], viewMatrix[9]);
	VectorNormalize(xAxis);
	VectorNormalize(yAxis);
	Float3 localTranslate;

	int keyMod=glutGetModifiers();
	switch(key)
	{
	case GLUT_KEY_UP:
		if (keyMod==GLUT_ACTIVE_SHIFT) 
		{
			localTranslate=(yAxis*10);
			glTranslated(localTranslate[0],localTranslate[1],localTranslate[2]);
		}
		else if (keyMod==GLUT_ACTIVE_CTRL)
		{
			TransformObjects(Float2(0,-5), false);
		}
		else
		{
			glLoadIdentity();
			glRotated(-1,1,0,0);
			glMultMatrixf(viewMatrix);
		}
		break;
	case GLUT_KEY_DOWN:
		if (keyMod==GLUT_ACTIVE_SHIFT) 
		{
			localTranslate=-(yAxis*10);
			glTranslated(localTranslate[0],localTranslate[1],localTranslate[2]);
		}
		else if (keyMod==GLUT_ACTIVE_CTRL)
		{
			TransformObjects(Float2(0,5), false);
		}
		else
		{
			glLoadIdentity();
			glRotated(1,1,0,0);
			glMultMatrixf(viewMatrix);
		}
		break;
	case GLUT_KEY_LEFT:
		if (keyMod==GLUT_ACTIVE_SHIFT) 
		{
			localTranslate=-(xAxis*10);
			glTranslated(localTranslate[0],localTranslate[1],localTranslate[2]);
		}
		else if (keyMod==GLUT_ACTIVE_CTRL)
		{
			TransformObjects(Float2(-20,0), false);
		}
		else
		{
			glLoadIdentity();
			glRotated(-1,0,1,0);
			glMultMatrixf(viewMatrix);
		}
		break;
	case GLUT_KEY_RIGHT:
		if (keyMod==GLUT_ACTIVE_SHIFT)
		{
			localTranslate=(xAxis*10);
			glTranslated(localTranslate[0],localTranslate[1],localTranslate[2]);
		}
		else if (keyMod==GLUT_ACTIVE_CTRL)
		{
			TransformObjects(Float2(20,0), false);
		}
		else
		{
			glLoadIdentity();
			glRotated(1,0,1,0);
			glMultMatrixf(viewMatrix);
		}
		break;
	default:
		cerr<<"That key is not recognized"<<endl;
		break;
	}
	if (drawSilhouette)
		EvaluateSilhouette();
	glutPostRedisplay();
}

void SurfaceCollision()
{
	float randScale = 400.0/(RAND_MAX*1.0);

	bool colliding = DetectCollision(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intRes, glParam, true);
	if (colliding)
	{
		nurbsSurfaces[intSurf1]->kdColor = Float3(1,0,0);
		nurbsSurfaces[intSurf2]->kdColor = Float3(1,0,0);
		//		cout << "Collision Detected" << endl << endl;
	}
	else
	{
		nurbsSurfaces[intSurf1]->kdColor = Float3(0,1,0);
		nurbsSurfaces[intSurf2]->kdColor = Float3(0,1,0);
		//		cout << "No Collision" << endl << endl;
	}
}

#ifdef COMPUTEMOMENTS
void PerformPicking(bool alreadyPicked)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	float invertY = viewport[3]-currentPoint[1];
	float xratio = 1.*currentPoint[0]/viewport[2];
	float yratio = 1.*invertY/viewport[3];
	float tempx = (camera.rightcp - camera.leftcp  )*xratio + camera.leftcp;
	float tempy = (camera.topcp   - camera.bottomcp)*yratio + camera.bottomcp;
	GLfloat *pt = new GLfloat[4];
	pt[0] = tempx;
	pt[1] = tempy;
	pt[3] = pt[4] = 0;
	glParam->pickedPoint = pt;

	if(glParam->controlMesh && glParam->pickedObjectNum >= 0 && glParam->pickedFaceNum >= 0)
	{
		if (alreadyPicked && glParam->pickedControlPointNum>=0)
		{
			Float3 xAxis;
			Float3 yAxis;
			Float3 localTranslate;

			double viewMatrix[16];
			glGetDoublev(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

			xAxis[0] = viewMatrix[0];
			xAxis[1] = viewMatrix[4];
			xAxis[2] = viewMatrix[8];

			yAxis[0] = viewMatrix[1];
			yAxis[1] = viewMatrix[5];
			yAxis[2] = viewMatrix[9];

			VectorNormalize(xAxis);
			VectorNormalize(yAxis);
			Float2 move = (currentPoint-clickPoint);
			Float3 translate=move[0]*xAxis - move[1]*yAxis ;

			float* cntlPoints		= objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->cntlPoints;
			int uPoints				= objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->uPoints;
			int vPoints				= objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->vPoints;
			int uPickedPointIndex	= glParam->pickedControlPointNum%uPoints;
			int vPickedPointIndex	= (glParam->pickedControlPointNum-uPickedPointIndex)/uPoints;
			int endBuffer = 1;
			for (int j = -1; j <= 1; j++)
			{
				for (int i = -1; i <= 1; i++)
				{
					if ( uPickedPointIndex + i >= endBuffer && uPickedPointIndex + i < uPoints - endBuffer && vPickedPointIndex + j >= endBuffer && vPickedPointIndex + j < vPoints - endBuffer )
					{
						cntlPoints[glParam->pickedControlPointNum*4+(j*uPoints+i)*4+0] += translate[0]*modelSize/(zoomFactor*100);
						cntlPoints[glParam->pickedControlPointNum*4+(j*uPoints+i)*4+1] += translate[1]*modelSize/(zoomFactor*100);
						cntlPoints[glParam->pickedControlPointNum*4+(j*uPoints+i)*4+2] += translate[2]*modelSize/(zoomFactor*100);
					}
				}
			}
			EvaluateSurfaces(glParam->pickedObjectNum, glParam->pickedFaceNum, false);
		}
		else
		{
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			float projectionMatrix[16], modelViewMatrix[16];
			glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrix);
			glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix);
			glParam->pickedControlPointNum = objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->PickControlPoint(currentPoint[0], currentPoint[1], projectionMatrix, modelViewMatrix);
			glPopMatrix();
		}
	}
	else if (glParam->pickedObjectNum >= 0 && glParam->pickedFaceNum >= 0 && glParam->drawingON)
	{
		Float3 xAxis;
		Float3 yAxis;
		Float3 localTranslate;

		double viewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

		xAxis[0] = viewMatrix[0];
		xAxis[1] = viewMatrix[4];
		xAxis[2] = viewMatrix[8];

		yAxis[0] = viewMatrix[1];
		yAxis[1] = viewMatrix[5];
		yAxis[2] = viewMatrix[9];

		VectorNormalize(xAxis);
		VectorNormalize(yAxis);
		Float2 move = (currentPoint-clickPoint);
		Float3 translate=move[0]*xAxis - move[1]*yAxis ;


		NURBS* surface			= objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface;
		Float2 paramPoint		= surface->pickedParamPoints[0];
		float* cntlPoints		= surface->cntlPoints;
		float* uKnotVector		= surface->uKnotVector;
		float* vKnotVector		= surface->vKnotVector;
		int nU					= surface->uPoints;
		int nV					= surface->vPoints;
		int kU					= surface->uOrder;
		int kV					= surface->vOrder;

		int p = 0;
		while ( p < nU+kU && uKnotVector[p] < paramPoint[0])
			p++;
		int uPickedPointIndex = p - kU;

		int q = 0;
		while ( q < nV+kV && vKnotVector[q] < paramPoint[1])
			q++;
		int vPickedPointIndex = q - kV;

		int endBuffer = 1;
		int uSpread = 10; 
		int vSpread = 10;
		for (int j = vPickedPointIndex; j <= vPickedPointIndex + uSpread; j++)
		{
			for (int i = uPickedPointIndex; i <= uPickedPointIndex + vSpread; i++)
			{
				if ( i >= endBuffer && i < nU - endBuffer && j >= endBuffer && j < nV - endBuffer )
				{
					cntlPoints[(j*nU+i)*4+0] += translate[0]*modelSize/(zoomFactor*100);
					cntlPoints[(j*nU+i)*4+1] += translate[1]*modelSize/(zoomFactor*100);
					cntlPoints[(j*nU+i)*4+2] += translate[2]*modelSize/(zoomFactor*100);
				}
			}
		}
		EvaluateSurfaces(glParam->pickedObjectNum, glParam->pickedFaceNum, false);
		Float4 momentError;
		Float4 moments = UpdateObjectMoment(objects[glParam->pickedObjectNum], glParam->pickedFaceNum, &momentError, glParam);

		objects[glParam->pickedObjectNum]->volume  = moments[0];
		Float3 firstMoment;
		firstMoment[0] = moments[1];
		firstMoment[1] = moments[2];
		firstMoment[2] = moments[3];
		objects[glParam->pickedObjectNum]->massCenter = firstMoment/objects[glParam->pickedObjectNum]->volume;
		objects[glParam->pickedObjectNum]->massCenterComputed = true;
	}
	else
	{
		glParam->picked	= true;
		GetClickParametricPoint(&(objects), camera.nearcp, glParam);
		glParam->picked	= false;
	}
}
#else
void PerformPicking(bool alreadyPicked)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	float invertY = viewport[3]-currentPoint[1];
	float xratio = 1.*currentPoint[0]/viewport[2];
	float yratio = 1.*invertY/viewport[3];
	float tempx = (camera.rightcp - camera.leftcp  )*xratio + camera.leftcp;
	float tempy = (camera.topcp   - camera.bottomcp)*yratio + camera.bottomcp;
	GLfloat *pt = new GLfloat[4];
	pt[0] = tempx;
	pt[1] = tempy;
	pt[3] = pt[4] = 0;
	glParam->pickedPoint = pt;
	
//	glParam->pickedSurfNum = 0;
	if(glParam->controlMesh && (glParam->pickedSurfNum>=0 || glParam->pickedFaceNum >= 0) )
	//if(glParam->controlMesh)
	{
		if (alreadyPicked && glParam->pickedControlPointNum>=0)
		{
			Float3 xAxis;
			Float3 yAxis;
			Float3 localTranslate;

			double viewMatrix[16];
			glGetDoublev(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

			xAxis[0] = viewMatrix[0];
			xAxis[1] = viewMatrix[4];
			xAxis[2] = viewMatrix[8];

			yAxis[0] = viewMatrix[1];
			yAxis[1] = viewMatrix[5];
			yAxis[2] = viewMatrix[9];

			VectorNormalize(xAxis);
			VectorNormalize(yAxis);
			Float2 move = (currentPoint-clickPoint);
			Float3 translate=move[0]*xAxis - move[1]*yAxis ;

			timer.Start();

#ifdef BREP
			objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->cntlPoints[glParam->pickedControlPointNum*4+0] += translate[0]*modelSize/(zoomFactor*100);
			objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->cntlPoints[glParam->pickedControlPointNum*4+1] += translate[1]*modelSize/(zoomFactor*100);
			objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->cntlPoints[glParam->pickedControlPointNum*4+2] += translate[2]*modelSize/(zoomFactor*100);
			EvaluateSurfaces(glParam->pickedObjectNum, glParam->pickedFaceNum, true);
#else
			nurbsSurfaces[glParam->pickedSurfNum]->cntlPoints[glParam->pickedControlPointNum*4+0] += translate[0]*modelSize/(zoomFactor*100);
			nurbsSurfaces[glParam->pickedSurfNum]->cntlPoints[glParam->pickedControlPointNum*4+1] += translate[1]*modelSize/(zoomFactor*100);
			nurbsSurfaces[glParam->pickedSurfNum]->cntlPoints[glParam->pickedControlPointNum*4+2] += translate[2]*modelSize/(zoomFactor*100);
			EvaluateSurfaces(glParam->pickedSurfNum, true);
#endif


#ifdef DYNAMICHD
			float point1[3], point2[3];

#ifdef CUDA
			float minDist;
			if (computeNRHD)
				minDist = IddoGetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), NRKB, glParam, false);
			else
				minDist = GetCUDASurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
#else
			float minDist;
			if (computeNRHD)
				minDist = IddoGetSurfaceHausdorffDistance(nurbsSurfaces[intSurf2], nurbsSurfaces[intSurf1], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
			else
				minDist = GetSurfaceHausdorffDistance(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], point1, point2, int(intRes*pow(2.0,glParam->displayLevel)), glParam, false);
#endif

			timer.Stop();
			float currentRunTime = timer.GetElapsedTimeInMilliSec();
#ifdef INTERACTIVETIMING
			ofs << currentRunTime  << endl;
#endif
			glParam->closestPointComputed = true;
			glParam->point1 = Float3(point1[0], point1[1], point1[2]);
			glParam->point2 = Float3(point2[0], point2[1], point2[2]);
#endif
		}
		else
		{
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			float projectionMatrix[16], modelViewMatrix[16];
			glGetFloatv(GL_PROJECTION_MATRIX, projectionMatrix);
			glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix);
#ifdef BREP
			glParam->pickedControlPointNum = objects[glParam->pickedObjectNum]->faces[glParam->pickedFaceNum]->surface->PickControlPoint(currentPoint[0], currentPoint[1], projectionMatrix, modelViewMatrix);
#else
			glParam->pickedControlPointNum = nurbsSurfaces[glParam->pickedSurfNum]->PickControlPoint(currentPoint[0], currentPoint[1], projectionMatrix, modelViewMatrix);
#endif
			glPopMatrix();
		}
	}
	else
	{
		glParam->picked	= true;
#ifdef BREP
		GetClickParametricPoint(&(objects), camera.nearcp, glParam);
#else
		GetClickParametricPoint(&(nurbsSurfaces), camera.nearcp, glParam);
#endif
		glParam->picked	= false;
	}
}
#endif

void SilhoetteOrbitControl(Float2 disp)
{
	glMatrixMode(GL_MODELVIEW);
	float viewMatrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

	Float3 xAxis = Float3(viewMatrix[0], viewMatrix[4], viewMatrix[8]);
	Float3 zAxis = Float3(viewMatrix[2], viewMatrix[6], viewMatrix[10]);
	VectorNormalize(xAxis);
	VectorNormalize(zAxis);

	Float3 localAxis = -disp[0]*zAxis + disp[1]*xAxis;
	VectorNormalize(localAxis);

	glPushMatrix();
	glLoadIdentity();
	glRotated(VectorMagnitude(disp),localAxis[0], localAxis[1], localAxis[2]);
	glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);	// get the viewMatrix
	glPopMatrix();

	silhouettePoint = TransformPoint(silhouettePoint, viewMatrix);
	silhouetteVector = silhouettePoint;
	VectorNormalize(silhouetteVector);
	silhouetteVector *= -1.0;
	silhouettePoint = silhouetteVector*(-modelSize*silhouetteArrowFactor);

	EvaluateSilhouette();
}

void KeyPress(unsigned char key, int x, int y)
{
	float scale;
	float viewMatrix[16];
	switch(key)
	{
	case 'q':
	case 'Q':
		//Quit
		exit(0);
		break;
	case 's':
	case 'S':
		// Toggle the smooth shading
		glParam->smooth=!glParam->smooth;
		break;
	case 'g':
	case 'G':
		// Toggle the background color
		backgroundColor = (backgroundColor+1)%3;
		break;
	case 'w':
	case 'W':
		// Toggle wireframe
		glParam->wireframe=!glParam->wireframe;
		break;
	case 'c':
	case 'C':
		// Toggle controlMesh
		glParam->controlMesh=!glParam->controlMesh;
		break;
	case 'b':
	case 'B':
		// Toggle bounding box visibility
		for (int i=0; i<nurbsSurfaces.size();i++)
		{
			if (glParam->drawBBox)
			{
				delete [] nurbsSurfaces[i]->minPointsDisplay;
				delete [] nurbsSurfaces[i]->maxPointsDisplay;
			}
			else
			{
				nurbsSurfaces[i]->minPointsDisplay	= new float[1];
				nurbsSurfaces[i]->maxPointsDisplay	= new float[1];
			}
		}
		glParam->drawBBox = !glParam->drawBBox;
		evaluated = false;
		break;
	case 'x':
	case 'X':
		// Expand bounding box
		glParam->expandBoundingBox = !glParam->expandBoundingBox;
		evaluated = false;
		break;
	case '.':
	case '>':
		//Zoom in
		glMatrixMode(GL_MODELVIEW);
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix
		glLoadIdentity();
		scale = 1.25;
		glScaled(scale, scale, scale);
		glMultMatrixf(viewMatrix);
		zoomFactor *= scale;
		//		evaluated = false;
		break;
	case ',':
	case '<':
		//Zoom out
		glMatrixMode(GL_MODELVIEW);
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix
		glLoadIdentity();
		scale = 0.8;
		glScaled(scale, scale, scale);
		glMultMatrixf(viewMatrix);
		zoomFactor *= scale;
		//		evaluated = false;
		break;
	case 'e':
	case 'E':
		//Evaluate all surfaces
		evaluated = false;
		break;
	case 'a':
	case 'A':
		// Increase Mesh Density
		for (int i=0; i<nurbsSurfaces.size();i++)
		{
			NURBS* surface = nurbsSurfaces[i];
			surface->uBaseNum = (int(surface->uBaseNum*1.11)!= surface->uBaseNum) ? int(surface->uBaseNum*1.11) : surface->uBaseNum+1;
			surface->vBaseNum = (int(surface->vBaseNum*1.11)!= surface->vBaseNum) ? int(surface->vBaseNum*1.11) : surface->vBaseNum+1;
		}
		for (int k=0; k<objects.size(); k++)
		{
			Object* object = objects[k];
			for (int i=0; i<object->faces.size(); i++)
			{
				if (object->faces[i]->isNURBS)
				{
					NURBS* surface = object->faces[i]->surface;
					surface->uBaseNum = (int(surface->uBaseNum*1.11)!= surface->uBaseNum) ? int(surface->uBaseNum*1.11) : surface->uBaseNum+1;
					surface->vBaseNum = (int(surface->vBaseNum*1.11)!= surface->vBaseNum) ? int(surface->vBaseNum*1.11) : surface->vBaseNum+1;
				}
			}
		}
		if (intSurf1 < nurbsSurfaces.size() && intSurf2 < nurbsSurfaces.size())
		{
			if (nurbsSurfaces[intSurf1]->evaluatedIntersection)
				nurbsSurfaces[intSurf1]->GenerateIntersectionOverlayTexture(nurbsSurfaces[intSurf1]->intersectingBoxes, nurbsSurfaces[intSurf1]->numIntersectingBoxes, intRes, glParam);
			if (nurbsSurfaces[intSurf2]->evaluatedIntersection)
				nurbsSurfaces[intSurf2]->GenerateIntersectionOverlayTexture(nurbsSurfaces[intSurf2]->intersectingBoxes, nurbsSurfaces[intSurf2]->numIntersectingBoxes, intRes, glParam);
		}
		evaluated = false;
		break;
	case 'z':
	case 'Z':
		// Decrease Mesh Density
		for (int i=0; i<nurbsSurfaces.size();i++)
		{
			NURBS* surface = nurbsSurfaces[i];
			surface->uBaseNum = (int(surface->uBaseNum*0.9)>1) ? int(surface->uBaseNum*0.9) : 1;
			surface->vBaseNum = (int(surface->vBaseNum*0.9)>1) ? int(surface->vBaseNum*0.9) : 1;
		}
		for (int k=0; k<objects.size(); k++)
		{
			Object* object = objects[k];
			for (int i=0; i<object->faces.size(); i++)
			{
				if (object->faces[i]->isNURBS)
				{
					NURBS* surface = object->faces[i]->surface;
					surface->uBaseNum = (int(surface->uBaseNum*0.9)>1) ? int(surface->uBaseNum*0.9) : 1;
					surface->vBaseNum = (int(surface->vBaseNum*0.9)>1) ? int(surface->vBaseNum*0.9) : 1;
				}
			}
		}
		if (intSurf1 < nurbsSurfaces.size() && intSurf2 < nurbsSurfaces.size())
		{
			if (nurbsSurfaces[intSurf1]->evaluatedIntersection)
				nurbsSurfaces[intSurf1]->GenerateIntersectionOverlayTexture(nurbsSurfaces[intSurf1]->intersectingBoxes, nurbsSurfaces[intSurf1]->numIntersectingBoxes, intRes, glParam);
			if (nurbsSurfaces[intSurf2]->evaluatedIntersection)
				nurbsSurfaces[intSurf2]->GenerateIntersectionOverlayTexture(nurbsSurfaces[intSurf2]->intersectingBoxes, nurbsSurfaces[intSurf2]->numIntersectingBoxes, intRes, glParam);
		}
		evaluated = false;
		break;
	case 'm':
	case 'M':
		// Calculate Moments
		if (objects.size() > 0)
		{
			Float4 moments;
			Float4 momentError;
			//			for (int quadPoints = 1; quadPoints <=3; quadPoints++)
			//			{
			//				for (int evalLevel = 0;evalLevel <=9; evalLevel++)
			//				{
			int trimPoints = 4;
			int quadPoints = 2;
			int evalLevel  = glParam->displayLevel;
			bool forcedEval = true;
			bool triangleGPUMoment = true;
			bool timing = false;
			timer.Start();
			moments = GetGPUObjectMoment(objects[0], (evalLevel+1), quadPoints, trimPoints, glParam, &momentError, glParam->expandBoundingBox, forcedEval, timing, triangleGPUMoment);
			timer.Stop();
			objects[0]->volume  = moments[0];
			Float3 firstMoment;
			firstMoment[0] = moments[1];
			firstMoment[1] = moments[2];
			firstMoment[2] = moments[3];
			objects[0]->massCenter = firstMoment/objects[0]->volume;
			objects[0]->massCenterComputed = true;
			sceneVolume = objects[0]->volume;
			//					cout << quadPoints << " " << evalLevel+1 << " " << objects[0]->volume << " " << (momentError[0])/objects[0]->volume << endl;
			//						<< " "  << moments[1]/moments[0] << " " << moments[2]/moments[0] << " " << moments[3]/moments[0] << endl;
			//				}
			//			}
			cout << "Moment Time         : " << timer.GetElapsedTimeInMicroSec()*0.001 << endl;
			cout << "Grid Size           : " << (evalLevel+1)*512 << endl;
			cout << "Volume              : " << objects[0]->volume << endl;
			cout << "ACIS Volume         : " << objects[0]->ACISVolume << endl;
			cout << "Volume Error        : " << (momentError[0])/objects[0]->volume << endl;
			cout << "Center of Mass      : " << objects[0]->massCenter[0] << " " << objects[0]->massCenter[1] << " " << objects[0]->massCenter[2] << endl;
			cout << "Center of Mass ACIS : " << objects[0]->ACISMassCenter[0] << " " << objects[0]->ACISMassCenter[1] << " " << objects[0]->ACISMassCenter[2] << endl << endl;
		}
		else if (nurbsSurfaces.size()>0)
		{
			Float4 momentError;
			int numQuadraturePoints = 3;
			int numTrimPointsPerPatch = 4;
			int uPatches = (glParam->displayLevel+1)*32;
			int vPatches = (glParam->displayLevel+1)*32;
			Float4 moments = GetSurfaceMoment(nurbsSurfaces[0], uPatches, vPatches, numQuadraturePoints, numTrimPointsPerPatch, glParam, &momentError, true, true);
			Float3 firstMoment;
			firstMoment[0] = moments[1];
			firstMoment[1] = moments[2];
			firstMoment[2] = moments[3];
			Float3 massCenter = firstMoment/moments[0];
			cout << uPatches << " " << moments[0] << " " << massCenter[0] << " " << massCenter[1] << " " << massCenter[2] << endl ;
		}
		break;
	case 'k':
	case 'K':
		// Save Mesh
		for (int i=0; i<nurbsSurfaces.size(); i++)
		{
			//char* intStr = new char[3];
			//itoa(i,intStr,10);
			string surfNum = static_cast<ostringstream*>( &(ostringstream() << i) )->str();
			string fileName = "Mesh" + surfNum + ".obj";
			nurbsSurfaces[i]->SaveMesh(fileName.c_str(), glParam);
			//delete[] intStr;
		}

/*
		if (nurbsSurfaces.size() >= 1)
			if (glParam->pickedSurfNum > 0)
				nurbsSurfaces[glParam->pickedSurfNum]->SaveMesh("Mesh.obj", glParam);
			else
				nurbsSurfaces[0]->SaveMesh("Mesh.obj", glParam);
*/
		break;
	case 'l':
	case 'L':
		// Interactive surface LOD Computation
		InteractiveLOD(nurbsSurfaces, objects, camera, glParam);
		break;
	case '{':
	case '[':
		// Interactive surface LOD
		if (glParam->surfVisibilityCutOff > 0)
			glParam->surfVisibilityCutOff-=1.0/(128*128*36*1.0);
		break;
	case '}':
	case ']':
		// Interactive surface LOD
		if (glParam->surfVisibilityCutOff < 1)
			glParam->surfVisibilityCutOff+=1.0/(128*128*36*1.0);
		break;
	case '-':
	case '_':
		// Interactive surface LOD
		if (glParam->surfVisibilityCutOff > 0)
			glParam->surfVisibilityCutOff-=0.0001;
		break;
	case '=':
	case '+':
		// Interactive surface LOD
		if (glParam->surfVisibilityCutOff < 1)
			glParam->surfVisibilityCutOff+=0.0001;
		break;
	case ';':
	case ':':
		// Interactive surface LOD
		if (glParam->objVisibilityCutOff > 0)
			glParam->objVisibilityCutOff-=1.0/(128*128*36*1.0);
		break;
	case '\'':
	case '"':
		// Interactive surface LOD
		if (glParam->objVisibilityCutOff < 1)
			glParam->objVisibilityCutOff+=1.0/(128*128*36*1.0);
		break;
	case '9':
	case '(':
		// Interactive surface LOD
		if (glParam->objVisibilityCutOff > 0)
			glParam->objVisibilityCutOff-=0.001;
		break;
	case '0':
	case ')':
		// Interactive surface LOD
		if (glParam->objVisibilityCutOff < 1)
			glParam->objVisibilityCutOff+=0.001;
		break; 
	case 'i':
	case 'I':
		if (computeDistanceMetrics)
		{
#ifdef SURFACEMETRICS
#ifdef COMPUTEHAUSDORFFDISTANCE
	if (computeInteractiveRun)
		AutomatedTranslationHD();
	else
		ComputeHausdorffDistance();
#else
		ComputeMinDistance();
#endif
#endif
		}
		else
		{
#ifdef BREP
		if (objects.size() >= 2)
			if (objects[0]->faces[0]->isNURBS && objects[1]->faces[0]->isNURBS)
				CalculateSurfaceIntersection(objects[0]->faces[0]->surface, objects[1]->faces[0]->surface, intRes, glParam, false);
#else
		//for (int j=0; j<nurbsSurfaces.size(); j++)
		//	nurbsSurfaces[j]->SurfaceCurvatureEvaluation(intRes, glParam);
		CalculateSurfaceIntersection(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intRes, glParam, false);
#endif
		}
		break;
	case 'u':
	case 'U':
		// Calculate Forced Intersection
		if (nurbsSurfaces.size() > 1)
			CalculateSurfaceIntersection(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intRes, glParam, true);
		break;
	/*	
	case '*':
	case '8':
		// Intersection Demo
		if (nurbsSurfaces.size() > 1)
		{
			TranslateNURBS(nurbsSurfaces[intSurf1],25);
			CalculateSurfaceIntersection(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intRes, glParam, true);
			evaluated = false;
		}
		break;
	case '&':
	case '7':
		// Intersection Demo
		if (nurbsSurfaces.size() > 1)
		{
			TranslateNURBS(nurbsSurfaces[intSurf1],-25);
			CalculateSurfaceIntersection(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intRes, glParam, true);
			evaluated = false;
		}
		break;
	*/	
	case 'n':
	case 'N':
		// Exact Normals
		glParam->exactNormals = !glParam->exactNormals;
		evaluated = false;
		break;
	case 'h':
	case 'H':
		// Silhouette curve
		drawSilhouette = !drawSilhouette;
	case 'f':
	case 'F':
		// Cut thickened lines
		if (glParam->drawingON && glParam->pickedSurfNum!=-1)
			nurbsSurfaces[glParam->pickedSurfNum]->UpdateTrimCurves(glParam);
		glParam->drawingON	= false;
		break;
	case '`':
	case '~':
		// Standard views
		glMatrixMode(GL_MODELVIEW);
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix
		zoomFactor = 1;
		glLoadIdentity();
		standardView = (standardView+1)%3;
		if (standardView == 1)
			glRotated(90,1,0,0);
		else if (standardView == 2)
			glRotated(90,0,1,0);
		break;
	case 'v':
	case 'V':
		glParam->drawVoxels = !(glParam->drawVoxels);
		break;
	case '1':
	case '!':
		// Interactive surface Offset
		if (glParam->offsetDistance > 0)
			glParam->offsetDistance-=0.001;
		break;
	case '2':
	case '@':
		// Interactive surface Offset
		if (glParam->offsetDistance < 1)
			glParam->offsetDistance+=0.001;
		break;
	case 'd':
	case 'D':
		// Dump Screen
		DumpScreenToFile("Screen.txt", viewport.w, viewport.h, 0);
		break;
	case 'j':
	case 'J':
		// Animate Collision
		if (!animate)
		{
			InitializeAnimation();
			animate = true;
		}
		else
		{
			StopAnimation();
			animate = false;
		}
		break;
	case 'r':
	case 'R':
		reflectionPlane = (reflectionPlane+1)%4;
		break;
	case 'p':
	case 'P':
		clipping = !clipping;
		break;
	case '3':
	case '#':
		clipPlaneDistZ += 0.1;
		break;
	case '4':
	case '$':
		clipPlaneDistZ -= 0.1;
		break;
	case '5':
	case '%':
		clipPlaneDistX += 0.01;
		break;
	case '6':
	case '^':
		clipPlaneDistX -= 0.01;
		break;
	case '7':
	case '&':
		glParam->displayLevel++;
		break;
	case '8':
	case '*':
		glParam->displayLevel--;
		break;
	case 't':
	case 'T':
		//glParam->voxelCount = 100;
		for (int i=0; i<objects.size(); i++)
			objects[i]->PerformVoxelization(glParam);
		break;
	default:
		cerr<<"Key "<<key<<" not supported"<<endl;

	}
	glutPostRedisplay();
}


// This functions is called whenever the mouse is pressed or released
// button is a number 0 to 2 designating the button
// state is 1 for release 0 for press event
void MouseClick(int button,int state,int x,int y)
{
	currentPoint[0] = x;
	currentPoint[1] = y;
	Float2 midPoint;
	float aspectRatio = (viewport.h*1.0)/(viewport.w*1.0);
	midPoint[0] = viewport.w/2.0;
	midPoint[1] = viewport.h/2.0;
	int keyMod=glutGetModifiers();
	if (button == 3 && state == GLUT_DOWN )
	{
		float viewMatrix[16];
		glMatrixMode(GL_MODELVIEW);
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

		float scale = .926;
		Float2 move = (currentPoint-midPoint)*((1.0-1.0/scale)*modelSize/100.0);

		glLoadIdentity();
		glScaled(scale, scale, scale);
		glTranslated(-move[0],move[1],0);
		glMultMatrixf(viewMatrix);

		zoomFactor *= scale;
		//		evaluated = false;
		Display();
	}
	else if (button == 4 && state == GLUT_DOWN )
	{
		float viewMatrix[16];
		glMatrixMode(GL_MODELVIEW);
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

		float scale = 1.08;
		Float2 move = (currentPoint-midPoint)*((1.0-1.0/scale)*modelSize/100.0);

		glLoadIdentity();
		glScaled(scale, scale, scale);
		glTranslated(-move[0],move[1],0);
		glMultMatrixf(viewMatrix);

		zoomFactor *= scale;
		//		evaluated = false;
		Display();
	}
	else if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON && keyMod == GLUT_ACTIVE_SHIFT)
	{
		lShiftButtonDown	= true;
		clickPoint[0]		= x;
		clickPoint[1]		= y;
		currentPoint		= clickPoint; 
	}
	else if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON && keyMod == GLUT_ACTIVE_SHIFT)
	{
		rShiftButtonDown	= true;
		clickPoint[0]		= x;
		clickPoint[1]		= y;
		currentPoint		= clickPoint; 
	}
	else if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON && keyMod == GLUT_ACTIVE_CTRL)
	{
		lCtrlButtonDown	= true;
		clickPoint[0]		= x;
		clickPoint[1]		= y;
		currentPoint		= clickPoint; 
	}
	else if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON)
	{
		lButtonDown		= true;
		clickPoint[0]	= x;
		clickPoint[1]	= y;
		currentPoint	= clickPoint; 
	}
	else if (state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON)
	{
		rButtonDown		= true;
		clickPoint[0]	= x;
		clickPoint[1]	= y;
		currentPoint	= clickPoint; 
		PerformPicking(false);
		glParam->drawingON	= true;
		Display();
		//		glutPostRedisplay();
	}
	else if (state == GLUT_DOWN && button == GLUT_MIDDLE_BUTTON)
	{
		mButtonDown		= true;
		clickPoint[0]	= x;
		clickPoint[1]	= y;
		currentPoint	= clickPoint;
	}
	else if(state = GLUT_UP)
	{
		if (glParam->drawingON && glParam->pickedSurfNum!=-1)
			nurbsSurfaces[glParam->pickedSurfNum]->UpdateTrimCurves(glParam);
		rButtonDown			= false;
		lButtonDown			= false;
		mButtonDown			= false;
		lShiftButtonDown	= false;
		rShiftButtonDown	= false;
		lCtrlButtonDown		= false;
		glParam->picked		= false;
		glParam->drawingON	= false;
		//		if (objects.size() > 0)
		//			selectedObject	= (selectedObject+1)%(int)(objects.size());
		Display();
	}
}

void MouseMove(int x,int y)
{
	currentPoint[0] = x;
	currentPoint[1] = y;
	if (rShiftButtonDown)
	{
		TransformObjects(currentPoint - clickPoint, true);
		clickPoint = currentPoint;
	}
	else if (lShiftButtonDown)
	{
		TransformObjects(currentPoint - clickPoint, false);
		clickPoint = currentPoint;
	}
	else if (lCtrlButtonDown && drawSilhouette)
	{
		SilhoetteOrbitControl(currentPoint - clickPoint);
		clickPoint = currentPoint;
	}
	else if (lButtonDown)
	{
		glMatrixMode(GL_MODELVIEW);
		float viewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

		Float3 xAxis = Float3(1, 0, 0);
		Float3 yAxis = Float3(0, 1, 0);

		Float2 angle = currentPoint-clickPoint;
		Float3 localAxis = angle[0]*yAxis + angle[1]*xAxis;
		VectorNormalize(localAxis);
		glLoadIdentity();
		glRotated(VectorMagnitude(angle),localAxis[0], localAxis[1], localAxis[2]);
		glMultMatrixf(viewMatrix);

		clickPoint = currentPoint;
	}
	else if (mButtonDown)
	{
		glMatrixMode(GL_MODELVIEW);
		float viewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);	// get the viewMatrix

		Float3 xAxis = Float3(1, 0, 0);
		Float3 yAxis = Float3(0, 1, 0);

		Float2 move = (currentPoint-clickPoint);
		Float3 localTranslate=(move[0]*xAxis - move[1]*yAxis)*modelSize/100.0;

		glLoadIdentity();
		glTranslated(localTranslate[0],localTranslate[1],localTranslate[2]);
		glMultMatrixf(viewMatrix);

		clickPoint = currentPoint;
	}
	else if (rButtonDown)
	{
		PerformPicking(true);
		clickPoint = currentPoint;
	}
	if (mButtonDown || rButtonDown || lButtonDown || lShiftButtonDown || rShiftButtonDown || lCtrlButtonDown)
		Display();
}

void Idle()
{
	if (animate)
	{
		PerformAnimationStep();
		glutPostRedisplay(); 
	}
}

void CloseWindow(void)
{
	delete glParam;
	objects.clear();
	nurbsSurfaces.clear();

#ifdef ACISOBJ
	// Terminate ACIS
	api_terminate_constructors();
	api_terminate_faceter();
	api_terminate_booleans();
	api_stop_modeller();
#endif

#ifdef INTERACTIVETIMING
	ofs.close();
#endif
}


#ifdef ACISOBJ
int ExtractNURBSData(bs3_surface bs3Surf, FACE* face, NURBS* nurbsSurf, Float3* tempObjColor = NULL, int surfID = -1)
{
	nurbsSurf->uPoints	= bs3_surface_ncu(bs3Surf);
	nurbsSurf->vPoints	= bs3_surface_ncv(bs3Surf);
	nurbsSurf->uOrder	= bs3_surface_degree_u(bs3Surf)+1;
	nurbsSurf->vOrder	= bs3_surface_degree_v(bs3Surf)+1;
	nurbsSurf->trimmed	= false;

	rgb_color entityColor;
	int inherit=1,isColored;
	api_rh_get_entity_rgb(face,entityColor,inherit,isColored);
	if (tempObjColor == NULL)
	{
		float specularFactor	= 1;
		nurbsSurf->kdColor		= Float3( entityColor.red(), entityColor.green(), entityColor.blue());
		nurbsSurf->ksColor		= Float3( entityColor.red()*specularFactor, entityColor.green()*specularFactor, entityColor.blue()*specularFactor);
		nurbsSurf->ka			= objKA;
		nurbsSurf->shininess	= objShininess;
	}
	else
	{
		nurbsSurf->kdColor		= Float3((*tempObjColor)[0], (*tempObjColor)[1], (*tempObjColor)[2]);
		nurbsSurf->ksColor		= Float3((*tempObjColor)[0]*.25, (*tempObjColor)[1]*.25, (*tempObjColor)[2]*.25);
		nurbsSurf->ka			= objKA;
		nurbsSurf->shininess	= objShininess;
	}

	nurbsSurf->cntlPoints	= new float[nurbsSurf->uPoints*nurbsSurf->vPoints*4];
	nurbsSurf->uKnotVector	= new float[nurbsSurf->uPoints+nurbsSurf->uOrder];
	nurbsSurf->vKnotVector	= new float[nurbsSurf->vPoints+nurbsSurf->vOrder];

	for (int v = 0; v < nurbsSurf->vPoints ; v++)
	{
		for (int u = 0; u < nurbsSurf->uPoints; u++)
		{
			SPAposition cntlPtPos;
			int dim;
			double weight;
			int isRational;
			bs3_surface_ij_ctrlpt(bs3Surf,u,v,cntlPtPos,weight,isRational,dim);

			nurbsSurf->cntlPoints[v*nurbsSurf->uPoints*4+u*4+0] = cntlPtPos.x();
			nurbsSurf->cntlPoints[v*nurbsSurf->uPoints*4+u*4+1] = cntlPtPos.y();
			nurbsSurf->cntlPoints[v*nurbsSurf->uPoints*4+u*4+2] = cntlPtPos.z();
			if (isRational)
				nurbsSurf->cntlPoints[v*nurbsSurf->uPoints*4+u*4+3] = weight;
			else
				nurbsSurf->cntlPoints[v*nurbsSurf->uPoints*4+u*4+3] = 1;

		}
	}

	double* tempuKnots;
	double* tempvKnots;
	int numuKnots,numvKnots;
	bs3_surface_knots_u(bs3Surf,numuKnots,tempuKnots);
	bs3_surface_knots_v(bs3Surf,numvKnots,tempvKnots);
	for (int u = 0; u < nurbsSurf->uPoints + nurbsSurf->uOrder; u++)
		nurbsSurf->uKnotVector[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[numuKnots-1]-tempuKnots[0]);
	for (int v = 0; v < nurbsSurf->vPoints + nurbsSurf->vOrder; v++)
		nurbsSurf->vKnotVector[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[numvKnots-1]-tempvKnots[0]);

	nurbsSurf->uParamOffset = tempuKnots[0];
	nurbsSurf->vParamOffset = tempvKnots[0];
	nurbsSurf->uParamRange  = tempuKnots[numuKnots-1]-tempuKnots[0];
	nurbsSurf->vParamRange  = tempvKnots[numvKnots-1]-tempvKnots[0];

	nurbsSurf->surfaceID = surfID;
	delete [] tempuKnots;
	delete [] tempvKnots;

	// Set up the size factor based on the bounding box size fo the control polygon
	float size = nurbsSurf->CalculateBoundingBoxSize();
	//float size = 2500;
	nurbsSurf->sizeFactor = size;

	nurbsSurf->uBaseNum = int(nurbsSurf->sizeFactor*(nurbsSurf->uPoints+nurbsSurf->uOrder));
	nurbsSurf->vBaseNum = int(nurbsSurf->sizeFactor*(nurbsSurf->vPoints+nurbsSurf->vOrder));

	int maxNum = 25;
	int minNum = 10;

	if (nurbsSurf->uBaseNum >= maxNum)
		nurbsSurf->uBaseNum = maxNum;
	if (nurbsSurf->vBaseNum >= maxNum)
		nurbsSurf->vBaseNum = maxNum;

	if (nurbsSurf->uBaseNum<= minNum)
		nurbsSurf->uBaseNum = minNum;
	if (nurbsSurf->vBaseNum<= minNum)
		nurbsSurf->vBaseNum = minNum;

	// Read in the Trim Curves
	ENTITY_LIST loopList;
	api_get_loops((ENTITY*)face,loopList,PAT_CAN_CREATE,NULL);
	int numLoops = loopList.count();
	ENTITY* loop = (ENTITY*)loopList.first();
	int startSegmentID = 0;
	for (int loopID = 0; loopID < numLoops; loopID++)
	{
		ENTITY_LIST coedgeList;
		api_get_coedges(loop,coedgeList,PAT_CAN_CREATE,NULL);
		COEDGE* coedge = (COEDGE*)coedgeList.first();
		int numCoedges = coedgeList.count();
		if (numLoops != 1 && numCoedges > 4)
			nurbsSurf->trimmed = true;
		startSegmentID = nurbsSurf->trimCurves.size();

		for (int coedgeID = 0; coedgeID < numCoedges; coedgeID++)
		{
			pcurve pSplCurve = coedge->geometry()->equation();
			bs2_curve bs2Curve = pSplCurve.cur();
			BSpline* bSplineCurve = new BSpline();
			int dim, degree, isRational, numKnots;
			SPAposition* ctrlPtPos;
			double *weights;
			double *tempKnots;
			bs2_curve_to_array(bs2Curve, dim, degree, isRational, bSplineCurve->nPoints, ctrlPtPos, weights, numKnots, tempKnots);

			bSplineCurve->cntlPoints = new float[bSplineCurve->nPoints*3];
			bSplineCurve->knotVector = new float[numKnots];
			bSplineCurve->order = degree+1;
			if (degree != 1)
				nurbsSurf->trimmed = true;
			for (int u = 0; u < bSplineCurve->nPoints; u++)
			{
				bSplineCurve->cntlPoints[u*3+0] = (ctrlPtPos[u].x()-nurbsSurf->uParamOffset)/nurbsSurf->uParamRange;
				bSplineCurve->cntlPoints[u*3+1] = (ctrlPtPos[u].y()-nurbsSurf->vParamOffset)/nurbsSurf->vParamRange;
				if (isRational)
					bSplineCurve->cntlPoints[u*3+2] = weights[u];
				else
					bSplineCurve->cntlPoints[u*3+2] = 1.0;
			}
			for (int u = 0; u < numKnots; u++)
				bSplineCurve->knotVector[u] = (tempKnots[u]-tempKnots[0])/(tempKnots[numKnots-1]-tempKnots[0]);

			delete [] weights;
			delete [] tempKnots;
			delete [] ctrlPtPos;

			nurbsSurf->trimCurves.push_back(bSplineCurve);
			coedge = (COEDGE*)coedgeList.next();
		}

		for (int coedgeID = 0; coedgeID < numCoedges; coedgeID++)
		{
			int segmentID1, segmentID2;
			segmentID1 = startSegmentID+coedgeID;
			if (coedgeID == numCoedges-1)
				segmentID2 = startSegmentID;
			else
				segmentID2 = startSegmentID+coedgeID+1;

			Float2 segment1Point1 = nurbsSurf->trimCurves[segmentID1]->Evalutate(0);
			Float2 segment1Point2 = nurbsSurf->trimCurves[segmentID1]->Evalutate(1);
			Float2 segment2Point1 = nurbsSurf->trimCurves[segmentID2]->Evalutate(0);
			Float2 segment2Point2 = nurbsSurf->trimCurves[segmentID2]->Evalutate(1);

			float dist11 = Distance(segment1Point1,segment2Point1);
			float dist12 = Distance(segment1Point1,segment2Point2);
			float dist21 = Distance(segment1Point2,segment2Point1);
			float dist22 = Distance(segment1Point2,segment2Point2);

			if (dist11 > DELTA && dist12 > DELTA && dist21 > DELTA && dist22 > DELTA)
			{
				BSpline* bSplineCurveFix = new BSpline();
				bSplineCurveFix->nPoints = 2;
				bSplineCurveFix->order = 2;
				bSplineCurveFix->cntlPoints = new float[6];
				bSplineCurveFix->knotVector = new float[4];
				bSplineCurveFix->knotVector[0] = 0;
				bSplineCurveFix->knotVector[1] = 0;
				bSplineCurveFix->knotVector[2] = 1;
				bSplineCurveFix->knotVector[3] = 1;
				bSplineCurveFix->cntlPoints[0] = segment1Point2[0];
				bSplineCurveFix->cntlPoints[1] = segment1Point2[1];
				bSplineCurveFix->cntlPoints[2] = 1;
				bSplineCurveFix->cntlPoints[3] = segment2Point1[0];
				bSplineCurveFix->cntlPoints[4] = segment2Point1[1];
				bSplineCurveFix->cntlPoints[5] = 1;
				nurbsSurf->trimCurves.push_back(bSplineCurveFix);
			}
		}
		loop = (ENTITY*)loopList.next();
	}
	return nurbsSurf->uBaseNum*nurbsSurf->vBaseNum;
}

void CalculateACISSurfaceSign(Face* face, FACE* faceACIS,  ENTITY* entity, float objectSize)
{
	float positionDELTA = 1e-3*objectSize;
	BODY* body = (BODY*)entity;
	NURBS* surface = face->surface;
	// Find a point on the surface
	bool pointFound = false;
	float uVal = 0.01;
	float vVal = 0.01;
	Float3 surfacePoint;
	int iterationCount = 0;
	while (!pointFound && iterationCount<100)
	{
		surfacePoint = surface->Evaluate(uVal, vVal);
		SPAposition testPointACIS = SPAposition(surfacePoint[0], surfacePoint[1], surfacePoint[2]);
		SPApar_pos initialGuess = SPApar_pos(uVal, vVal);
		point_face_containment testResult;
		SPAtransf faceTransformation; 
		api_point_in_face(testPointACIS, faceACIS, faceTransformation, testResult, initialGuess);
		if (testResult != point_outside_face)
			pointFound = true;
		else
		{
			uVal += rand()*0.02/(RAND_MAX*1.0);
			if (uVal>1.0)
				uVal = 1.0;
			vVal += rand()*0.02/(RAND_MAX*1.0);
			if (vVal>1.0)
				vVal = 1.0;
		}
		iterationCount++;
	}
	if (!pointFound)
		cout<<"Warning: Point not found for face " << face->surfID <<endl;

	// Calculate the sign of the normals at a surface point and check if they are outward pointing or not
	//	SPApar_pos paramPoint = SPApar_pos(surface->uParamOffset+surface->uParamRange*uVal, surface->vParamOffset+surface->vParamRange*vVal);
	//	SPAunit_vector normalACISoriginal = faceACIS->geometry()->equation().eval_normal(paramPoint);
	//	Float3 normalACIS = Float3(normalACISoriginal.x(), normalACISoriginal.y(), normalACISoriginal.z());
	Float3 normal = surface->EvaluateNormal(uVal, vVal);
	VectorNormalize(normal);
	Float3 testPoint = surfacePoint + positionDELTA*normal;
	point_containment solidTestResult;
	SPAposition pointACIS = SPAposition(testPoint[0], testPoint[1], testPoint[2]);
	api_point_in_body(pointACIS, body, solidTestResult);
	if (solidTestResult == point_inside)
		surface->normalSign = -1;
	else
		surface->normalSign = 1;
}
int GetTrianglesFromFacetedFace(FACE* face, vector<Triangle>* faceTri)
{
	// Find the attribute for facets attached to the face. This is the mesh.
	MESH * face_mesh = NULL;
	af_query((ENTITY*)face, IDX_MESH_APP, IDX_MESH_ID, face_mesh);
	INDEXED_MESH * mesh = (INDEXED_MESH*)face_mesh;

	if (mesh && mesh->get_type_of_mesh() != INDEXED_MESH_TYPE)
		mesh = NULL;

	if (mesh == NULL)
	{
		std::cout << "No indexed mesh available for face" << endl;
		return 0;
	}
	
	// Get the number of nodes of the whole face
	int nodeCount = mesh->get_num_vertex();

	// Get number of facets for this face
	int polygonCount = mesh->get_num_polygon();

	// Loop through each facet and copy the data to triangle data structure
	int triangleID = 0;
	for (int polygonIndex = 0; polygonIndex < polygonCount; polygonIndex++)
	{
		// Get the facet's data as an indexed_polygon
		indexed_polygon* poly = mesh->get_polygon(polygonIndex);

		if (poly)
		{
			// Get the number of nodes in this facet
			int nodeCount = poly->num_vertex();
			// Make sure it is a triangle
			if (nodeCount != 3)
				std::cout << "Face has non-triangualar facets" << endl;
			else
			{
				Triangle t;
				// Print the node indices of this facet
				for (int polyNodeIndex = 0; polyNodeIndex < nodeCount; polyNodeIndex++)
				{
					polygon_vertex* vertex = poly->get_vertex(polyNodeIndex);
					int vertexIndex = mesh->get_vertex_index(vertex);
					SPAposition vertexPos(mesh->get_position(vertexIndex));
					SPAunit_vector vertexNor(mesh->get_normal(vertexIndex));
					SPApar_pos uv(mesh->get_uv_as_entered(vertexIndex));
					t[polyNodeIndex].point		= Float3(vertexPos.x(), vertexPos.y(), vertexPos.z());
					t[polyNodeIndex].normal		= Float3(vertexNor.x(), vertexNor.y(), vertexNor.z());
					t[polyNodeIndex].texCoords	= Float2((float)uv.u, (float) uv.v);
				}
				Float3 side1 = t[1].point - t[0].point;
				Float3 side2 = t[2].point - t[0].point;
				t.faceNormal = VectorCrossProduct(side1, side2);
				VectorNormalize(t.faceNormal);
				// If shared edge info ins needed
				// find which edge is shared with the next facet
				//int ishare_edge = poly->get_share_info();
				t.triangleID = triangleID;  // Get Triangle ID
				triangleID++;
				faceTri->push_back(t);
			}
		}
	}
	int numTriangles = polygonCount;
	return numTriangles;
}

void ReadACISObject(char* fileName, bool trimmed)
{
	FILE *fp;
	fopen_s(&fp,fileName, "r");
	ENTITY_LIST entityList;
	if (fp != NULL)
		api_restore_entity_list(fp,TRUE,entityList);
	else
	{
		cout << "Unable to open file!" << endl;
		return;
	}
	fclose(fp);

	int totalNumQuads = 0;  
	int totalNumNURBS = 0;
	int totalNumTrimmed = 0;
	int totalNumTriangles = 0;
	int totalNumFaces = 0;
	unsigned int volumeACISTime = 0;
	bool randomFaceColors = false;
	bool randomEntityColors = false;
	// Get header info from sat file 
	//	FileInfo info;	// Create FileInfo Object
	//	api_get_file_info(info);

	//Cycle through entities to identify them
	for (int entityID = 0; entityID < entityList.count(); entityID++)
	//for (int entityID = 3; entityID < 4; entityID++)
	{
		//if (entityID == 0 || (entityID >= 2 && entityID <= 7))
		//continue;

		int totalObjTriangles = 0;
		Float3 tempObjColor = Float3(rand()/(RAND_MAX*1.0), rand()/(RAND_MAX*1.0), rand()/(RAND_MAX*1.0));
		if (is_BODY(entityList[entityID]))
		{
			// Apply transformations
			BODY* currentBody = (BODY*)entityList[entityID];
			if (!checkOutcome(api_change_body_trans(currentBody, NULL, 0)))
				cout << " Removing transformations failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;

#ifdef CONVERTTONURBS
			convert_to_spline_options convertOptions;
			convertOptions.set_do_edges(true);
			convertOptions.set_do_faces(true);
			convertOptions.set_in_place(true);
			outcome result = api_convert_to_spline(entityList[entityID], &convertOptions, NULL);
			if (result.ok())
				cout << "Entity " << entityID << " converted. " << endl;
			else
				cout << "Entity " << entityID << " conversion FAILED. " << endl;
#endif

			ENTITY_LIST faceEntityList;
			if (!checkOutcome(api_get_faces(entityList[entityID], faceEntityList, PAT_CAN_CREATE, NULL)))
				cout << " Get Faces failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;

			faceEntityList.init();
			FACE* face = (FACE*)faceEntityList.first();
			int numFaces = faceEntityList.count();
			totalNumFaces += numFaces;

			//Set up the ACIS object
			Object* tempObj = new Object();
			tempObj->dlid = entityID*1000;
			tempObj->objID = entityID;

#ifdef COMPUTEMOMENTS
			// Compute object volume and center of mass
			mass_props massProperties;
			mass_props_options massPropsOptions;
			SPAposition planeRoot = SPAposition(0,0,0);
			SPAvector planeNormal = SPAvector(0,1,0);
			massPropsOptions.set_req_rel_accy(.01);
			//			massPropsOptions.set_plane_info(planeRoot, planeNormal);
			//			massPropsOptions.set_use_plane_info(TRUE);

			timer.Start();
			BODY* tempBody = (BODY*)entityList[entityID];
			api_body_mass_props(tempBody, massProperties, &massPropsOptions, NULL);

			volumeACISTime += timer.GetElapsedTimeInMicroSec();
			cout << "Volume Computed" << endl;
			tempObj->ACISVolume = float(massProperties.get_volume());
			tempObj->ACISMassCenter = Float3(massProperties.get_centroid().x(), massProperties.get_centroid().y(), massProperties.get_centroid().z());
#endif

			// Get bounding box to calculate triangulation tolerance
			SPAposition minPt, maxPt;
			SPAboxing_options boxOptions;
			boxOptions.set_mode(SPAboxing_tight);
			api_get_entity_box(entityList[entityID], minPt, maxPt, &boxOptions, NULL);
			double ACISModelSize = distance_to_point(minPt, maxPt);
			float offset = 0.001*ACISModelSize;

			tempObj->bBoxMax = Float3(maxPt.x() + offset,	maxPt.y() + offset, maxPt.z() + offset);
			tempObj->bBoxMin = Float3(minPt.x() - offset,	minPt.y() - offset, minPt.z() - offset);

			tempObj->maxModelSize = __max((tempObj->bBoxMax[0] - tempObj->bBoxMin[0]),__max((tempObj->bBoxMax[1] - tempObj->bBoxMin[1]),(tempObj->bBoxMax[2] - tempObj->bBoxMin[2])));

			float modelDiag = Distance(tempObj->bBoxMax, tempObj->bBoxMin);
			modelSize = modelDiag/2.0;
			silhouettePoint = Float3(0,silhouetteArrowFactor*modelSize,0);
			if (modelDiag > sceneSize)
				sceneSize = modelDiag;

			// Specify the degree of refinement
			REFINEMENT* objRefinement;
			if (!checkOutcome(api_create_refinement(objRefinement)))
				cout << "Creating object refinement failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;
			objRefinement->set_triang_mode(AF_TRIANG_ALL);
			//objRefinement->set_surface_tol(-1);
			objRefinement->set_surface_tol(0.05*modelDiag);
			objRefinement->set_normal_tol(30.0);
			objRefinement->set_edge_length(0.05*modelDiag);

			// Set the refinement to the entity
			if (!checkOutcome(api_set_entity_refinement(entityList[entityID], objRefinement, TRUE)))
				cout << "Setting refinement to entity failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;

			// Specify the information to be recorded at each Vertex
			parameter_token ptoken[3];
			ptoken[0] = POSITION_TOKEN;
			ptoken[1] = NORMAL_TOKEN;
			ptoken[2] = UV_TOKEN;
			VERTEX_TEMPLATE* vTemplate = new VERTEX_TEMPLATE (3, ptoken);
			if (!checkOutcome(api_set_default_vertex_template(vTemplate)))
				cout << "Setting vertex template failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;

			facet_options_expert facetExpert;
			facetExpert.set_tolerance_specification_mode(af_surface_tolerance_from_refinement);
			//facetExpert.set_surface_tolerance_as_fraction_of_body_diagonal(0.01);

			// Calculate the sign of the surfaces
			for (int faceID = 0; faceID < numFaces; faceID++)
			{
#ifndef TESSELLATE
#ifndef BREP
				if (face->geometry()->identity()==SPLINE_TYPE)
				{
					const spline &splSurf = (spline&)face->geometry()->equation();
					if (splSurf.sur_present())
					{
						bs3_surface bs3Surf = splSurf.sur();
						if (!bs3_surface_closed_u(bs3Surf) && !bs3_surface_closed_v(bs3Surf))
						{
							NURBS* nurbsSurf = new NURBS();
							totalNumQuads += ExtractNURBSData(bs3Surf, face, nurbsSurf);
							if (nurbsSurf->trimmed && !trimmed)
								nurbsSurf->trimmed = false;
							nurbsSurfaces.push_back(nurbsSurf);
							cout << "Face " << faceID << " added to NURBS surface list" << endl;
							totalNumNURBS++;
							if (nurbsSurf->trimmed)
								totalNumTrimmed++;
						}
					}
				}
#endif
#endif
				Face* objFace = new Face();
				objFace->dlid = entityID*1000 + faceID + 1;
				objFace->surfID = faceID;
				objFace->parentObjID = entityID;
				objFace->trimmed = false;

				rgb_color entityColor;
				int inherit=1,isColored;
				api_rh_get_entity_rgb(face,entityColor,inherit,isColored);
				if (isColored && !randomEntityColors)
				{
					float specularFactor	= 1;
					objFace->kdColor		= Float3(entityColor.red(), entityColor.green(), entityColor.blue());
					objFace->ksColor		= Float3(entityColor.red()*specularFactor, entityColor.green()*specularFactor, entityColor.blue()*specularFactor);
					objFace->ka				= objKA;
					objFace->shininess		= objShininess;
				}
				else
				{
					//objFace->kdColor	= Float3(0.48, 0.24, 0.64);
					//objFace->ksColor	= Float3(0.12, 0.06, 0.16);
					//objFace->ka			= 0.7;
					//objFace->shininess	= 100;
					Float3 tempFaceColor = Float3(tempObjColor);
					if (randomFaceColors)
						tempFaceColor = Float3(rand() / (RAND_MAX*1.0), rand() / (RAND_MAX*1.0), rand() / (RAND_MAX*1.0));
					objFace->kdColor = Float3(tempFaceColor[0], tempFaceColor[1], tempFaceColor[2]);
					objFace->ksColor = Float3(tempFaceColor[0] * 0.25, tempFaceColor[1] * 0.25, tempFaceColor[2] * 0.25);
					objFace->ka = objKA;
					objFace->shininess = objShininess;
				}

				// Get bounding box data
				SPAposition faceMinPt, faceMaxPt;
				SPAboxing_options faceBoxOptions;
				boxOptions.set_mode(SPAboxing_tight);
				api_get_entity_box(face, faceMinPt, faceMaxPt, &faceBoxOptions, NULL);
				float offset = 20;

				float faceOffset = 0;// modelSize/100;
				objFace->bBoxMax = Float3(faceMaxPt.x() + faceOffset,	faceMaxPt.y() + faceOffset, faceMaxPt.z() + faceOffset);
				objFace->bBoxMin = Float3(faceMinPt.x() - faceOffset,	faceMinPt.y() - faceOffset, faceMinPt.z() - faceOffset);

				// Create a mesh manager for the face
				INDEXED_MESH_MANAGER *faceMeshManager = new INDEXED_MESH_MANAGER();
				if (!checkOutcome(api_set_mesh_manager(faceMeshManager)))
					cout << "Setting mesh manager failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;

				if (!checkOutcome(api_facet_entity(face, &facetExpert)))
					cout << "Faceting face " << faceID << " failed : " << spa::CheckOutcome::getLastErrorMessage() << endl;
							
				int faceTriangles = GetTrianglesFromFacetedFace(face, &(objFace->triangles));

#ifdef BREP
				if (face->geometry()->identity()==SPLINE_TYPE)
				{
					const spline &splSurf = (spline&)face->geometry()->equation();
					if (splSurf.sur_present())
					{
						bs3_surface bs3Surf = splSurf.sur();
						int surfID = entityID*10000 + faceID;
						NURBS* nurbsSurf = new NURBS();

						if (!isColored)
							totalNumQuads += ExtractNURBSData(bs3Surf, face, nurbsSurf, &tempObjColor, surfID);
						else
							totalNumQuads += ExtractNURBSData(bs3Surf, face, nurbsSurf, NULL, surfID);
						objFace->isNURBS = true;
						if (nurbsSurf->trimmed && !trimmed)
							nurbsSurf->trimmed = false;
						objFace->surface = nurbsSurf;
						//nurbsSurfaces.push_back(nurbsSurf);
						totalNumNURBS++;
						if (nurbsSurf->trimmed)
							totalNumTrimmed++;
					}
				}
				else
				{
					totalObjTriangles += faceTriangles;
					cout << "Face " << faceID << " is a triangulated surface" << endl;
				}
#endif
				tempObj->faces.push_back(objFace);
				cout << "Face " << faceID << " facetted with " << faceTriangles << " polygons" << endl;
				totalNumTriangles += faceTriangles;
				delete faceMeshManager;
				face = (FACE*)faceEntityList.next();
			}

#ifndef BREP
			FACE* faceACIS = (FACE*)faceEntityList.first();
			for (int k = 0; k < tempObj->faces.size(); k++)
			{
				Face* face = tempObj->faces[k];
				if (face->isNURBS)
					CalculateACISSurfaceSign(face, faceACIS, entityList[entityID], tempObj->maxModelSize);
				
				//else
				//	face->kdColor = Float3(.8,0,0);

				faceACIS = (FACE*) faceEntityList.next();
			}
#endif
			tempObj->totalNumTriangles = totalObjTriangles;
			//if (entityID > 3)
			//	tempObj->transformationMatrix[14] = 20;
			if (tempObj->faces.size() > 0)
				objects.push_back(tempObj);
			cout << "Entity " << entityID + 1 << " of " << entityList.count() << " processed" << endl;
		}
		else // If isBody
		{
			cout << "Entity " << entityID + 1 << " of " << entityList.count() << " not processed" << endl;
		}
	}

	cout << "Scene Size : " << sceneSize << endl;
	cout << "Total number of surfaces       : " << totalNumFaces << endl; 
	cout << "Total number of NURBS          : " << totalNumNURBS << endl;
	cout << "Total number of trimmed NURBS  : " << totalNumTrimmed << endl;
	cout << "Total number of quads          : " << totalNumQuads << endl;
	cout << "Total number of triangles      : " << totalNumTriangles << endl << endl;
#ifdef COMPUTEMOMENTS
	cout << "Volume computations time       : " << volumeACISTime*.001 << endl << endl;
#endif
}
#endif


void ReadHermiteMesh()
{
	//ifstream nodesFile("../../Models/Hermite/Nodes.txt", ios::in);
	//ifstream elemsFile("../../Models/Hermite/Elements.txt", ios::in);
	//ifstream sfmFile("../../Models/Hermite/SFM.xls", ios::in);
	//ifstream nodesFile("../../Models/Hermite/FaceENNodes.xls", ios::in);
	//ifstream elemsFile("../../Models/Hermite/FaceENElems.xls", ios::in);
	//ifstream sfmFile("../../Models/Hermite/FaceENSFM2.xls", ios::in);
	ifstream nodesFile("../../Models/Hermite/LVENEpiNodes.xls", ios::in);
	ifstream elemsFile("../../Models/Hermite/LVENEpiElements.xls", ios::in);
	ifstream sfmFile("../../Models/Hermite/LVENEpiSFM.xls", ios::in);

	Hermite2DPatchMesh Hermite2DPatchMesh;

	if (!nodesFile.good() || !elemsFile.good())
	{
		cerr<<"Unable to open input files" << endl;
		abort();
	}

	string firstLine;
	getline(nodesFile,firstLine); 
	while (nodesFile.good())
	{
		Node2D* node = new Node2D();
		nodesFile >> node->vertexVal[0] >> node->vertexDu[0] >> node->vertexDv[0] >> node->vertexDuDv[0];
		nodesFile >> node->vertexVal[1] >> node->vertexDu[1] >> node->vertexDv[1] >> node->vertexDuDv[1];
		nodesFile >> node->vertexVal[2] >> node->vertexDu[2] >> node->vertexDv[2] >> node->vertexDuDv[2];
		nodesFile >> node->label >> node->nodeNumber;
		if (node->nodeNumber != -1)
			Hermite2DPatchMesh.nodes.push_back(node);
		else
			delete node;
	}

	getline(elemsFile,firstLine); 
	while (elemsFile.good())
	{
		Element2D* element = new Element2D();
		elemsFile >> element->nodes[0] >> element->nodes[1] >> element->nodes[2] >> element->nodes[3];
		element->nodes[0]--;
		element->nodes[1]--;
		element->nodes[2]--;
		element->nodes[3]--;
		elemsFile >> element->label >> element->elementNumber;
		if (element->elementNumber != -1)
			Hermite2DPatchMesh.elements.push_back(element);
		else
			delete element;
	}

	getline(sfmFile,firstLine); 
	while (sfmFile.good())
	{
		SFM2D* sfmLine = new SFM2D();
		Float3 val, du, dv, dudv;
		int coord, elemNodeNum, elemNum;
		sfmFile >> val[0] >> du[0] >> dv[0] >> dudv[0] >> coord >> elemNodeNum >> elemNum;
		sfmFile >> val[1] >> du[1] >> dv[1] >> dudv[1] >> coord >> elemNodeNum >> elemNum;
		sfmFile >> val[2] >> du[2] >> dv[2] >> dudv[2] >> coord >> elemNodeNum >> elemNum;
		sfmLine->elemNodeNumber = elemNodeNum;
		sfmLine->elementNumber = elemNum;
		sfmLine->val = val;
		sfmLine->du = du;
		sfmLine->dv = dv;
		sfmLine->dudv = dudv;
		Hermite2DPatchMesh.sfm.push_back(sfmLine);
	}

	Hermite2DPatchMesh.initialized = true;
	nodesFile.close();
	elemsFile.close();
	sfmFile.close();

	for (int k = 0; k < Hermite2DPatchMesh.elements.size(); k++)
	{
		Hermite2DPatch* patch = new Hermite2DPatch();
		patch->identityTransformation = true;
		patch->kdColor = Float3( 0.8, 0.3, 0.3);
		patch->ksColor = Float3( 0.3, 0.3, 0.3);
		patch->ka = 0.01;
		patch->shininess = 5;
		for (int j = 0; j < 4; j++)
		{
			patch->vertexVal[j]		= Hermite2DPatchMesh.nodes[Hermite2DPatchMesh.elements[k]->nodes[j]]->vertexVal;
			//patch->vertexDu[j]	= Hermite2DPatchMesh.nodes[Hermite2DPatchMesh.elements[k]->nodes[j]]->vertexDu;
			//patch->vertexDv[j]	= Hermite2DPatchMesh.nodes[Hermite2DPatchMesh.elements[k]->nodes[j]]->vertexDv;
			//patch->vertexDuDv[j]	= Hermite2DPatchMesh.nodes[Hermite2DPatchMesh.elements[k]->nodes[j]]->vertexDuDv;
			
			patch->vertexDu[j]		= Hermite2DPatchMesh.sfm[k*4+j]->du;
			patch->vertexDv[j]		= Hermite2DPatchMesh.sfm[k*4+j]->dv;
			patch->vertexDuDv[j]	= Hermite2DPatchMesh.sfm[k*4+j]->dudv;;


		}
		patch->initialized = true;
		NURBS* surface = new NURBS();
		patch->ConvertToNURBS(surface);
		surface->surfaceID = k;
		nurbsSurfaces.push_back(surface);
		delete patch;
	}
}

void ReadHermite3DMesh()
{

	ifstream nodesFile;
	ifstream elemsFile;
	ifstream sfmFile;
	ifstream seFile;
	ifstream dispFile;


	bool lvModel = false;
	if (lvModel)
	{
		nodesFile.open("../../Models/Hermite/LVENNodes3D.xls", ios::in);
		elemsFile.open("../../Models/Hermite/LVENElements3D.xls", ios::in);
		sfmFile.open("../../Models/Hermite/LVENSFM3D.xls", ios::in);
		seFile.open("../../Models/Hermite/LVENSE3D.xls", ios::in);
		dispFile.open("../../Models/Hermite/LVENDispElems.xls", ios::in);
	}
	else
	{
		nodesFile.open("../../Models/Hermite/BiV162ENNodes3D.xls", ios::in);
		elemsFile.open("../../Models/Hermite/BiV162ENElements3D.xls", ios::in);
		sfmFile.open("../../Models/Hermite/BiV162ENSFM3D.xls", ios::in);
		seFile.open("../../Models/Hermite/BiV162ENSE3D.xls", ios::in);
		dispFile.open("../../Models/Hermite/BiV162ENDispElems.xls", ios::in);
	}
	Hermite3DMesh hermite3DMesh;

	if (!nodesFile.good() || !elemsFile.good() || !sfmFile.good() || !seFile.good() || !dispFile.good())
	{
		cerr<<"Unable to open input files" << endl;
		abort();
	}

	string firstLine;
	getline(nodesFile,firstLine); 
	while (nodesFile.good())
	{
		Node3D* node = new Node3D();
		nodesFile >> node->vertexVal[0] >> node->vertexDu[0] >> node->vertexDv[0] >> node->vertexDuDv[0];
		nodesFile >> node->vertexDw[0] >> node->vertexDvDw[0] >> node->vertexDwDu[0] >> node->vertexDuDvDw[0];
		nodesFile >> node->vertexVal[1] >> node->vertexDu[1] >> node->vertexDv[1] >> node->vertexDuDv[1];
		nodesFile >> node->vertexDw[1] >> node->vertexDvDw[1] >> node->vertexDwDu[1] >> node->vertexDuDvDw[1];
		nodesFile >> node->vertexVal[2] >> node->vertexDu[2] >> node->vertexDv[2] >> node->vertexDuDv[2];
		nodesFile >> node->vertexDw[2] >> node->vertexDvDw[2] >> node->vertexDwDu[2] >> node->vertexDuDvDw[2];
		nodesFile >> node->label >> node->nodeNumber;
		if (node->nodeNumber != -1)
			hermite3DMesh.nodes.push_back(node);
		else
			delete node;
	}

	getline(elemsFile,firstLine); 
	while (elemsFile.good())
	{
		Element3D* element = new Element3D();
		elemsFile >> element->nodes[0] >> element->nodes[1] >> element->nodes[2] >> element->nodes[3];
		elemsFile >> element->nodes[4] >> element->nodes[5] >> element->nodes[6] >> element->nodes[7];
		element->nodes[0]--;
		element->nodes[1]--;
		element->nodes[2]--;
		element->nodes[3]--;
		element->nodes[4]--;
		element->nodes[5]--;
		element->nodes[6]--;
		element->nodes[7]--;
		elemsFile >> element->label >> element->elementNumber;
		if (element->elementNumber != -1)
			hermite3DMesh.elements.push_back(element);
		else
			delete element;
	}

	getline(sfmFile,firstLine); 
	while (sfmFile.good())
	{
		LocalDOF* sfmLine = new LocalDOF();
		Float3 val, du, dv, dudv, dw, dwdu, dvdw, dudvdw;
		int coord, elemNodeNum;
		int elemNum = -1;
		sfmFile >> val[0] >> du[0] >> dv[0] >> dudv[0] >> dw[0] >> dwdu[0] >> dvdw[0] >> dudvdw[0] >> coord >> elemNodeNum >> elemNum;
		sfmFile >> val[1] >> du[1] >> dv[1] >> dudv[1] >> dw[1] >> dwdu[1] >> dvdw[1] >> dudvdw[1] >> coord >> elemNodeNum >> elemNum;
		sfmFile >> val[2] >> du[2] >> dv[2] >> dudv[2] >> dw[2] >> dwdu[2] >> dvdw[2] >> dudvdw[2] >> coord >> elemNodeNum >> elemNum;
		sfmLine->elemNodeNumber = elemNodeNum;
		sfmLine->elementNumber = elemNum;
		sfmLine->val = val;
		sfmLine->du = du;
		sfmLine->dv = dv;
		sfmLine->dudv = dudv;
		sfmLine->dw = dw;
		sfmLine->dwdu = dwdu;
		sfmLine->dvdw = dvdw;
		sfmLine->dudvdw = dudvdw;
		if (elemNum != -1)
			hermite3DMesh.localDOF.push_back(sfmLine);
		else
			delete sfmLine;
	}


	while (seFile.good())
	{
		Local2GlobalMap* local2GlobalMap = new Local2GlobalMap();
		int elemNodeNum;
		int elemNum = -1;
		float* der1Map = local2GlobalMap->der1Map;
		float* der2Map = local2GlobalMap->der2Map;
		seFile >> der1Map[0] >> der1Map[1] >> der1Map[2] >> der1Map[3] >> der1Map[4] >> der1Map[5] >> der1Map[6] >> der1Map[7] >> der1Map[8];
		seFile >> der2Map[0] >> der2Map[1] >> der2Map[2] >> der2Map[3] >> der2Map[4] >> der2Map[5] >> der2Map[6] >> der2Map[7] >> der2Map[8];
		seFile >> local2GlobalMap->der3Map >> elemNum >> elemNodeNum;

		local2GlobalMap->elemNodeNumber = elemNodeNum+1;
		local2GlobalMap->elementNumber = elemNum+1;
		if (elemNum != -1)
			hermite3DMesh.meshGlobal2LocalMap.push_back(local2GlobalMap);
		else
			delete local2GlobalMap;
	}

	getline(dispFile, firstLine);
	while (dispFile.good())
	{
		int elemNum = -1;
		int faceNum;
		dispFile >> elemNum >> faceNum;

		if (elemNum != -1)
		{
			hermite3DMesh.displayElements.push_back(elemNum);
			hermite3DMesh.displayFaces.push_back(faceNum);

		}
	}



	hermite3DMesh.initialized = true;
	nodesFile.close();
	elemsFile.close();
	sfmFile.close();
	seFile.close();
	dispFile.close();

	// Initialize Element DOF
	hermite3DMesh.ConvertGlobal2Local();


	/*
	// Initialize Local to Global Map
	for (int k = 0; k < hermite3DMesh.elements.size(); k++)
	{
		for (int j = 0; j < 8; j++)
		{
			int sfmLineNum = k * 8 + j;
			Local2GlobalMap* local2GlobalMap = new Local2GlobalMap();
			Float3 localDer1Vec[3];
			Float3 globalDer1Vec[3];
			localDer1Vec[0] = hermite3DMesh.LocalDOF[sfmLineNum]->du;
			localDer1Vec[1] = hermite3DMesh.LocalDOF[sfmLineNum]->dv;
			localDer1Vec[2] = hermite3DMesh.LocalDOF[sfmLineNum]->dw;

			globalDer1Vec[0] = hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[j]]->vertexDu;
			globalDer1Vec[1] = hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[j]]->vertexDv;
			globalDer1Vec[2] = hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[j]]->vertexDw;

			local2GlobalMap->GenerateMap(localDer1Vec, globalDer1Vec, local2GlobalMap->der1Map);

			Float3 localDer2Vec[3];
			Float3 globalDer2Vec[3];
			localDer2Vec[0] = hermite3DMesh.LocalDOF[sfmLineNum]->dudv;
			localDer2Vec[1] = hermite3DMesh.LocalDOF[sfmLineNum]->dvdw;
			localDer2Vec[2] = hermite3DMesh.LocalDOF[sfmLineNum]->dwdu;

			globalDer2Vec[0] = hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[j]]->vertexDuDv;
			globalDer2Vec[1] = hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[j]]->vertexDvDw;
			globalDer2Vec[2] = hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[j]]->vertexDwDu;
			float mag = VectorMagnitude(globalDer2Vec[0]);
			float mag1 = VectorMagnitude(localDer2Vec[0]);

			local2GlobalMap->GenerateMap(localDer2Vec, globalDer2Vec, local2GlobalMap->der2Map);
		}
	}

	*/

	int faceIndex[6][4]		= {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 2, 4, 6}, {1, 3, 5, 7}};
	for (int i = 0; i < hermite3DMesh.displayElements.size(); i++)
	{
		int k = hermite3DMesh.displayElements[i] - 1;
		int face = hermite3DMesh.displayFaces[i] - 1;
		
		Hermite2DPatch* patch = new Hermite2DPatch();
		patch->identityTransformation = true;
		patch->kdColor = Float3( 0.8, 0.3, 0.3);
		patch->ksColor = Float3( 0.3, 0.3, 0.3);
		patch->ka = 0.01;
		patch->shininess = 5;
		for (int j = 0; j < 4; j++)
		{
			patch->vertexVal[j]		= hermite3DMesh.nodes[hermite3DMesh.elements[k]->nodes[faceIndex[face][j]]]->vertexVal;
			if (face == 0 || face == 1)
			{
				patch->vertexDu[j]		= hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->du;
				patch->vertexDv[j]		= hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dv;
				patch->vertexDuDv[j]	= hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dudv;
			}
			else if (face == 2 || face == 3)
			{
				patch->vertexDu[j] = hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->du;
				patch->vertexDv[j] = hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dw;
				patch->vertexDuDv[j] = hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dwdu;
			}
			else if (face == 4 || face == 5)
			{
				patch->vertexDu[j] = hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dv;
				patch->vertexDv[j] = hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dw;
				patch->vertexDuDv[j] = hermite3DMesh.localDOF[k * 8 + faceIndex[face][j]]->dvdw;
			}
		}
		patch->initialized = true;
		hermite3DMesh.displayPatches.push_back(patch);
		NURBS* surface = new NURBS();
		patch->ConvertToNURBS(surface);
		surface->surfaceID = i;
		nurbsSurfaces.push_back(surface);
		hermite3DMesh.nurbsPatches.push_back(surface);
	}


	// Save Bezier Control Points
	//char* outputFile = "../../Models/Hermite/BiV162ENBezierNodes.xls";
	//hermite3DMesh.SaveMeshDisplayFaces(outputFile);

}


void main(int argc, char *argv[])
{
	// Initialize Variables
	viewport.w=800;//480;//640;
	viewport.h=600;//852;//480;
	glParam = new GLParameters();
	evaluated = false;
	cleared = true;
	animate = false;

	// Initialize GLUT
	glutInit(&argc, argv);
#ifdef MSAAA
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA | GLUT_STENCIL | GLUT_MULTISAMPLE);
#else
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA | GLUT_STENCIL);
#endif
	glutInitWindowSize(viewport.w,viewport.h);
	glutInitWindowPosition(0,0);
	glutCreateWindow(argv[0]);

	glutDisplayFunc(Display);
	glutIdleFunc(Idle);
	glutReshapeFunc(ReSize);
	glutMouseFunc(MouseClick);
	glutMotionFunc(MouseMove); 
	glutKeyboardFunc(KeyPress);
	glutSpecialFunc(SpecialKeys);
	atexit(&CloseWindow);
#ifdef USEFREEGLUT
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
#endif

	// Setup Cg
#ifndef CPURENDER
	cgSetErrorCallback(CGErrorCallback);
	InitGLEW();
#endif

	ParseInputFile("default.scene");

	//Read in NURBS surfaces
	for (int j=0;j<numDucky1;j++)
	{
		NURBS* nurbsSurf1 = new NURBS();
		nurbsSurf1->InitNURBS(1);
		NURBS* nurbsSurf2 = new NURBS();
		nurbsSurf2->InitNURBS(2);
		NURBS* nurbsSurf3 = new NURBS();
		nurbsSurf3->InitNURBS(3);
		if (numDucky1 > 1)
		{
			nurbsSurf1->transformationMatrix[12] = cos(j*2*PI/(numDucky1*1.0))*numDucky1/2.0-1;
			nurbsSurf1->transformationMatrix[13] = sin(j*2*PI/(numDucky1*1.0))*numDucky1/2.0-1;
			nurbsSurf1->transformationMatrix[14] = 1;

			nurbsSurf2->transformationMatrix[12] = cos(j*2*PI/(numDucky1*1.0))*numDucky1/2.0-1;
			nurbsSurf2->transformationMatrix[13] = sin(j*2*PI/(numDucky1*1.0))*numDucky1/2.0-1;
			nurbsSurf2->transformationMatrix[14] = 1;

			nurbsSurf2->transformationMatrix[12] = cos(j*2*PI/(numDucky1*1.0))*numDucky1/2.0-1;
			nurbsSurf2->transformationMatrix[13] = sin(j*2*PI/(numDucky1*1.0))*numDucky1/2.0-1;
			nurbsSurf2->transformationMatrix[14] = 1;
		}
		nurbsSurf1->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf1);
		nurbsSurf2->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf2);
		nurbsSurf3->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf3);
	}

	for (int j=0;j<numDucky2;j++)
	{
		NURBS* nurbsSurf1 = new NURBS();
		nurbsSurf1->InitNURBS(12);
		NURBS* nurbsSurf2 = new NURBS();
		nurbsSurf2->InitNURBS(2);
		NURBS* nurbsSurf3 = new NURBS();
		nurbsSurf3->InitNURBS(3);
		if (numDucky2 > 1)
		{
			nurbsSurf1->transformationMatrix[12] = cos(j*2*PI/(numDucky2*1.0))*numDucky2/2.0-1;
			nurbsSurf1->transformationMatrix[13] = sin(j*2*PI/(numDucky2*1.0))*numDucky2/2.0-1;
			nurbsSurf1->transformationMatrix[14] = 1;

			nurbsSurf2->transformationMatrix[12] = cos(j*2*PI/(numDucky2*1.0))*numDucky2/2.0-1;
			nurbsSurf2->transformationMatrix[13] = sin(j*2*PI/(numDucky2*1.0))*numDucky2/2.0-1;
			nurbsSurf2->transformationMatrix[14] = 1;

			nurbsSurf2->transformationMatrix[12] = cos(j*2*PI/(numDucky2*1.0))*numDucky2/2.0-1;
			nurbsSurf2->transformationMatrix[13] = sin(j*2*PI/(numDucky2*1.0))*numDucky2/2.0-1;
			nurbsSurf2->transformationMatrix[14] = 1;
		}
		nurbsSurf1->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf1);
		nurbsSurf2->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf2);
		nurbsSurf3->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf3);
	}

	for (int j=0;j<numTeapot1;j++)
	{
		NURBS* nurbsSurf1 = new NURBS();
		nurbsSurf1->InitNURBS(21);
		NURBS* nurbsSurf2 = new NURBS();
		nurbsSurf2->InitNURBS(22);
		NURBS* nurbsSurf3 = new NURBS();
		nurbsSurf3->InitNURBS(23);
		NURBS* nurbsSurf4 = new NURBS();
		nurbsSurf4->InitNURBS(24);

		nurbsSurf1->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf1);
		nurbsSurf2->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf2);
		nurbsSurf3->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf3);
		nurbsSurf4->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf4);
	}

	for (int j=0;j<numTeapot2;j++)
	{
		NURBS* nurbsSurf1 = new NURBS();
		nurbsSurf1->InitNURBS(21);
		NURBS* nurbsSurf2 = new NURBS();
		nurbsSurf2->InitNURBS(22);
		NURBS* nurbsSurf3 = new NURBS();
		nurbsSurf3->InitNURBS(23);
		NURBS* nurbsSurf4 = new NURBS();
		nurbsSurf4->InitNURBS(24);

		nurbsSurf1->kdColor = Float3(0.90, 0.60, 0.80);
		nurbsSurf1->ksColor = Float3(0.12, 0.06, 0.12);
		nurbsSurf2->kdColor = Float3(0.90, 0.60, 0.80);
		nurbsSurf2->ksColor = Float3(0.12, 0.06, 0.12);
		nurbsSurf3->kdColor = Float3(0.90, 0.60, 0.80);
		nurbsSurf3->ksColor = Float3(0.12, 0.06, 0.12);
		nurbsSurf4->kdColor = Float3(0.90, 0.60, 0.80);
		nurbsSurf4->ksColor = Float3(0.12, 0.06, 0.12);


		nurbsSurf1->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf1);
		nurbsSurf2->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf2);
		nurbsSurf3->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf3);
		nurbsSurf4->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf4);
	}

	for (int j=0;j<numFlags;j++)
	{
		NURBS* nurbsSurf = new NURBS();
		nurbsSurf->InitNURBS(j+5);
		if (j==0)
		{
			nurbsSurf->transformationMatrix[12] = 0;
			nurbsSurf->transformationMatrix[13] = 5;
			nurbsSurf->transformationMatrix[14] = 7;
			nurbsSurf->trimmed=true;
		}
		else
		{
			nurbsSurf->transformationMatrix[0]  = 10;
			nurbsSurf->transformationMatrix[5]  = 10;
			nurbsSurf->transformationMatrix[10] = 10;
			nurbsSurf->ka			=.1;
			nurbsSurf->shininess	= 10;
			nurbsSurf->kdColor		= Float3(.35,.21,.63);
			nurbsSurf->ksColor		= Float3(0.1,0.1,0.1);
		}
		nurbsSurfaces.push_back(nurbsSurf);
	}

	// Test NURBS surface
	if (selfInt)
	{
#ifndef ACISOBJ
		NURBS* nurbsSurf1 = new NURBS();
		nurbsSurf1->InitNURBS(6);
		nurbsSurf1->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf1);
		NURBS* nurbsSurf2 = new NURBS();
		nurbsSurf2->InitNURBS(6);
		nurbsSurf2->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf2);
#endif
		glParam->selfIntersection = true;
	}
	if (numTestNURBS >= 1)
	{
		NURBS* nurbsSurf1 = new NURBS();
		nurbsSurf1->InitNURBS(surf1Num);
		nurbsSurf1->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf1);
	}
	if (numTestNURBS >= 2)
	{
		NURBS* nurbsSurf2 = new NURBS();
		nurbsSurf2->InitNURBS(surf2Num);
		nurbsSurf2->surfaceID = nurbsSurfaces.size();
		nurbsSurfaces.push_back(nurbsSurf2);
	}
	if (numTestNURBS > 2)
	{
		for (int k = 3; k < numTestNURBS; k++)
		{
			NURBS* nurbsSurf = new NURBS();
			nurbsSurf->InitNURBS(4);
			nurbsSurf->surfaceID = nurbsSurfaces.size();
			nurbsSurfaces.push_back(nurbsSurf);
		}
	}

	//	if (intSurf1 < nurbsSurfaces.size())
	//		TransformNURBSCntlPts(nurbsSurfaces[intSurf1],30);

#ifndef CPURENDER
	if (!glParam->initialized)
		glParam->InitializeCG();
#endif

#ifdef CARDIACMODEL
	ReadHermite3DMesh();
#endif

	//	GLint max_buffers;
	//	glGetIntegerv(GL_MAX_DRAW_BUFFERS_ATI, &max_buffers);
	//	cout << "Max draw buffers = " << max_buffers << endl;

#ifdef ACISOBJ
	if (argv[1] == NULL)
	{
		cout << "File not specified!" << endl;
		return;
	}
	// Initialization of the modeller, must be done before any other calls.
	api_start_modeller(0);

	if (UnlockACIS() != 1)
		return;

	if (!checkOutcome(api_initialize_faceter()))
		cout << "Facetter Initialization failed: " << spa::CheckOutcome::getLastErrorMessage() << endl;

	for (int i = 1; i < argc; i++)
	{
		if (argv[i]!=NULL)
			ReadACISObject(argv[i], trimmedSurfaces);
	}
#else
#ifdef OBJFILE
	if (argv[1] == NULL)
		cout << "File not specified!" << endl;
	for (int i = 1; i < argc; i++)
	{
		if (argv[i]!=NULL)
			ReadOBJFile(argv[i], i+1);
	}
#endif
#endif
#ifdef NURBSFILE
	if (argv[1] == NULL)
		cout << "File not specified!" << endl;
	for (int i = 1; i < argc; i++)
	{
		if (argv[i]!=NULL)
		{
			NURBS* nurbsSurf = new NURBS();
			nurbsSurf->ReadNURBS(argv[i]);
			nurbsSurf->surfaceID = nurbsSurfaces.size();
			nurbsSurfaces.push_back(nurbsSurf);
		}
	}
#endif

	// Initialize GL and Display Lists
	InitGL();

	// Initialize CUDA
#ifdef CUDA
	InitializeCUDA();
#endif

#ifdef AUTOTIMING
	float intResVal[5] = {64, 128, 256, 512, 1024};
	float avgTimeVal[5], minTimeVal[5], maxTimeVal[5];
	int maxRuns = 10;
	for(int j = 0; j < 5; j++)
	{
		int runTime = 0;
		int maxRunTime = 0;
		int minRunTime = 1e6;
		int numRuns = 0;
		for (int numRun = 0; numRun < maxRuns; numRun++)
		{
			int intResValue = intResVal[j];

			long unsigned int intersectiontime = 0;
			timer.Start();

			CalculateSurfaceIntersection(nurbsSurfaces[intSurf1], nurbsSurfaces[intSurf2], intResValue, glParam, true);

			if (numRun > 0)
			{
				timer.Stop();
				int currentRunTime = timer.GetElapsedTimeInSec();
				runTime += currentRunTime;
				numRuns++;
				if ( currentRunTime > maxRunTime)
					maxRunTime = currentRunTime;
				if ( currentRunTime < minRunTime)
					minRunTime = currentRunTime;
			}
			cout << "Run : " << j << " " << numRun << endl;

		}
		avgTimeVal[j] = 0.001*int(runTime/(numRuns*1.0));
		minTimeVal[j] = (0.001*minRunTime);
		maxTimeVal[j] = (0.001*maxRunTime);

	}

	ofstream ofs("Timings.txt");
	if (!ofs.good())
		cerr<<"Unable to open file Timings.txt"<<endl;
	else
	{
		ofs<<"Average Time : "<< avgTimeVal[0] << " "  << avgTimeVal[1] << " "  << avgTimeVal[2] << " "  << avgTimeVal[3] << " "  << avgTimeVal[4] <<endl;
		ofs<<"Min Time     : "<< minTimeVal[0] << " "  << minTimeVal[1] << " "  << minTimeVal[2] << " "  << minTimeVal[3] << " "  << minTimeVal[4] <<endl;
		ofs<<"Max Time     : "<< maxTimeVal[0] << " "  << maxTimeVal[1] << " "  << maxTimeVal[2] << " "  << maxTimeVal[3] << " "  << maxTimeVal[4] <<endl;
	}
	cout << "Timings saved in Timings.txt" << endl;
	ofs.close();
#endif
#ifdef INTERACTIVETIMING
	ofs.open("Timings.txt");
	if (!ofs.good())
		cerr<<"Unable to open file Timings.txt"<<endl;
#endif

#ifdef MOVEOBJ
	objects[1]->transformationMatrix[12] = xPos;
	objects[1]->transformationMatrix[13] = yPos;
	objects[1]->transformationMatrix[14] = zPos;
	objects[1]->identityTransformation = false;
#endif
#ifdef MOVESURF
	nurbsSurfaces[intSurf2]->transformationMatrix[12] = xPos;
	nurbsSurfaces[intSurf2]->transformationMatrix[13] = yPos;
	nurbsSurfaces[intSurf2]->transformationMatrix[14] = zPos;
	nurbsSurfaces[intSurf2]->identityTransformation = false;
#ifdef DUCKYHD
	nurbsSurfaces[4]->transformationMatrix[12] = xPos;
	nurbsSurfaces[4]->transformationMatrix[13] = yPos;
	nurbsSurfaces[4]->transformationMatrix[14] = zPos;
	nurbsSurfaces[4]->identityTransformation = false;

	nurbsSurfaces[5]->transformationMatrix[12] = xPos;
	nurbsSurfaces[5]->transformationMatrix[13] = yPos;
	nurbsSurfaces[5]->transformationMatrix[14] = zPos;
	nurbsSurfaces[5]->identityTransformation = false;

#endif
#ifdef TEAPOTHD
	nurbsSurfaces[5]->transformationMatrix[12] = xPos;
	nurbsSurfaces[5]->transformationMatrix[13] = yPos;
	nurbsSurfaces[5]->transformationMatrix[14] = zPos;
	nurbsSurfaces[5]->identityTransformation = false;

	nurbsSurfaces[6]->transformationMatrix[12] = xPos;
	nurbsSurfaces[6]->transformationMatrix[13] = yPos;
	nurbsSurfaces[6]->transformationMatrix[14] = zPos;
	nurbsSurfaces[6]->identityTransformation = false;

	nurbsSurfaces[7]->transformationMatrix[12] = xPos;
	nurbsSurfaces[7]->transformationMatrix[13] = yPos;
	nurbsSurfaces[7]->transformationMatrix[14] = zPos;
	nurbsSurfaces[7]->identityTransformation = false;
#endif
#endif

	glutMainLoop();
}
