#include "NURBS.h"
#include "GPUUtilities.h"
#include "CUDAUtilities.h"

BSpline::BSpline()
{
	this->cntlPoints = NULL;
	this->knotVector = NULL;
	this->order = 0;
	this->nPoints = 0;
}

float BSpline::Basis(int i,int k, float u)
{
	float b;
	if (k>0)
	{
		float n1 = (u-knotVector[i])*Basis(i,k-1,u);
		float d1 = knotVector[i+k] - knotVector[i];
		float n2 = (knotVector[i+k+1] - u) * Basis(i+1,k-1,u);
		float d2 = knotVector[i+k+1] - knotVector[i+1];
		float a2,b2;
		if (d1 > DELTA || d1 < -DELTA)
			a2 = n1 / d1;
		else
			a2 = 0;
		if (d2 > DELTA || d2 < -DELTA)
			b2 = n2 / d2;
		else
			b2 = 0;
		b = a2 + b2;
	}
	else
	{
		if (u >= knotVector[i] && u < knotVector[i+1])
			b = 1;
		else
			b = 0;
	}
	return b;
}

Float2 BSpline::Evalutate(float u)
{
	Float2 c;
	float r = 0;
	int n = this->nPoints;
	int k = this->order;

	int uPos = 0;
	while((u>=knotVector[uPos]) && (uPos<n+k))
		uPos = uPos+1;

	if (u==1)
		uPos=uPos-2*k;
	else
		uPos=uPos-k;

	float* basisFunc=new float[k+1];
	
	BSplineBasis(k-1, u, uPos, this->knotVector, basisFunc);
	for (int i=0; i<k; i++)
	{
		c[0] += cntlPoints[3*(uPos+i)+0] * basisFunc[i] * cntlPoints[3*(uPos+i)+2];
		c[1] += cntlPoints[3*(uPos+i)+1] * basisFunc[i] * cntlPoints[3*(uPos+i)+2];
		r	 += basisFunc[i] * cntlPoints[3*(uPos+i)+2];
	}
	if (r!= 0)
	{
		c[0]/=r;
		c[1]/=r;
	}
	delete[] basisFunc;
	return c;
}

EvaluationParams::EvaluationParams()
{
	this->rendering = false;

	this->uNum	= 0;
	this->vNum	= 0; 

	this->uMin	= 0.0;
	this->vMin	= 0.0;
	this->uMax	= 1.0;
	this->vMax	= 1.0;

	glGenTextures(1, &(this->ctrlPointsTexture));
	glGenTextures(1, &(this->uCtrlPointIndexTexture));
	glGenTextures(1, &(this->vCtrlPointIndexTexture));
	glGenTextures(1, &(this->evaluatedSurfNormTexture));
	glGenTextures(1, &(this->evaluatedSurfTexture));

	glGenTextures(1, &(this->uBasisFuncDervTexture));
	glGenTextures(1, &(this->vBasisFuncDervTexture));

	this->bBoxTextures = new GLuint[2];
	glGenTextures(2, this->bBoxTextures);
	this->secondDervTextures = new GLuint[4];
	glGenTextures(4,this->secondDervTextures);

	glGenTextures(1, &(this->trimTexture));
	glGenTextures(1, &(this->expansionFactorTexture));

#ifdef CUDA
	this->uBasisFuncCUDAData = NULL;
	this->vBasisFuncCUDAData = NULL;
	this->ctrlPointsCUDAData = NULL;
	this->uBasisDervCUDAData = NULL;
	this->vBasisDervCUDAData = NULL;
	this->uCtrlPointIndexCUDAData = NULL;
	this->vCtrlPointIndexCUDAData = NULL;

	this->evaluatedCUDAPoints	= NULL;
	this->evaluatedCUDANormals	= NULL;
	this->transformedCUDAPoints = NULL;

	this->secondDervCUDAData[0] = NULL;
	this->secondDervCUDAData[1] = NULL;
	this->secondDervCUDAData[2] = NULL;
	this->secondDervCUDAData[3] = NULL;
	this->bBoxMinCUDAData		= NULL;
	this->bBoxMaxCUDAData		= NULL;
#endif

	this->maxExpansionFactor = 0.0;
	this->minPoints			= NULL;
	this->maxPoints			= NULL;
	this->evaluatedPoints	= NULL;
	this->trimData			= NULL;
	this->expansionFactors	= NULL;
	this->evaluatedNormals	= NULL;

	this->uEvalParamPoints	= NULL;
	this->vEvalParamPoints	= NULL;

	this->srcTexIndex = 0;
	this->dstTexIndex = 1;

	this->computeMomentError = false;
}

EvaluationParams::~EvaluationParams()
{
	glDeleteTextures(1, &(this->ctrlPointsTexture));
	glDeleteTextures(1, &(this->uCtrlPointIndexTexture));
	glDeleteTextures(1, &(this->vCtrlPointIndexTexture));
	glDeleteTextures(1, &(this->evaluatedSurfNormTexture));
	glDeleteTextures(1, &(this->evaluatedSurfTexture));

	glDeleteTextures(1, &(this->uBasisFuncTexture));
	glDeleteTextures(1, &(this->vBasisFuncTexture));
	glDeleteTextures(1, &(this->uBasisFuncDervTexture));
	glDeleteTextures(1, &(this->vBasisFuncDervTexture));

	glDeleteTextures(2, this->bBoxTextures);
	delete[] this->bBoxTextures;
	glDeleteTextures(4, this->secondDervTextures);

	glDeleteTextures(1, &(this->trimTexture));
	glDeleteTextures(1, &(this->expansionFactorTexture));

	delete[] this->secondDervTextures;

#ifdef CUDA
	cudaFree(this->uBasisFuncCUDAData);
	cudaFree(this->vBasisFuncCUDAData);
	cudaFree(this->ctrlPointsCUDAData);
	cudaFree(this->uCtrlPointIndexCUDAData);
	cudaFree(this->vCtrlPointIndexCUDAData);
	cudaFree(this->uBasisDervCUDAData);
	cudaFree(this->vBasisDervCUDAData);

	cudaFree(this->secondDervCUDAData[0]);
	cudaFree(this->secondDervCUDAData[1]);
	cudaFree(this->secondDervCUDAData[2]);
	cudaFree(this->secondDervCUDAData[3]);

	cudaFree(this->bBoxMinCUDAData);
	cudaFree(this->bBoxMaxCUDAData);

	if (!this->rendering)
	{
		cudaFree(this->evaluatedCUDAPoints);
		cudaFree(this->evaluatedCUDANormals);
	}
	if (this->transformedCUDAPoints != NULL)
		cudaFree(this->transformedCUDAPoints);
	CUDACheckErrors(" Clear Data " );
#endif

	if (this->evaluatedPoints != NULL)
		delete [] this->evaluatedPoints;

	if (this->evaluatedNormals != NULL)
		delete [] this->evaluatedNormals;

	if (this->expansionFactors != NULL)
		delete [] this->expansionFactors;

	if (this->minPoints != NULL)
		delete [] this->minPoints;

	if (this->maxPoints != NULL)
		delete [] this->maxPoints;

	if (this->trimData != NULL)
		delete [] this->trimData;

	if (this->uEvalParamPoints != NULL)
		delete [] this->uEvalParamPoints;

	if (this->vEvalParamPoints != NULL)
		delete [] this->vEvalParamPoints;
};

NURBS::NURBS()
{
	this->uOrder		= 0;
	this->vOrder		= 0; 
	this->uPoints		= 0; 
	this->vPoints		= 0;
	this->uBaseNum		= 10;
	this->vBaseNum		= 10;

	this->trimmed		= false;
	this->trimWidth		= 256;
	this->trimHeight	= 256;
	this->trimCurveLOD	= 1.0;
	glGenTextures(1, &this->trimTexture);

	this->textured	= false;
	glGenTextures(1, &this->texture);

	this->visibilityFactor	= 0;

	this->initializedEvalParams	= false;	

#ifdef VBO
	this->evaluatedVBO			= false;			
#endif

	this->evaluatedIntersection	= false;	
	this->numIntersectingBoxes = 0;

	this->evaluatedSilhouette		= false;
	this->identityTransformation	= true;
	MakeIdentityMatrix(this->transformationMatrix);

	this->moment0 = 0;
	this->moment1[0] = 0;
	this->moment1[1] = 0;
	this->moment1[2] = 0;

	this->normalSign = 1;
};

NURBS::~NURBS()
{
	if (this->cntlPoints != NULL)
		delete[] this->cntlPoints; 
	delete[] this->uKnotVector; 
	delete[] this->vKnotVector;

	glDeleteTextures(1, &this->trimTexture);
	glDeleteTextures(1, &this->texture);

//	this->transforms.clear();
	this->trimCurves.clear();

	if (this->numIntersectingBoxes!=0)
		delete [] this->intersectingBoxes;

	this->silhouettePointIndex.clear();

#ifdef VBO
	if (this->evaluatedVBO)
	{
		delete[] this->indexArray;
		glDeleteBuffersARB(1,&(this->ptVBO));
		glDeleteBuffersARB(1,&(this->normVBO));
		glDeleteBuffersARB(1,&(this->texVBO));
	}
#endif

#ifdef CUDA
	// Freeing Bezier Patches
	for (int i=0; i<m_bezPatches.size(); ++i)
		delete [] m_bezPatches[i]; 
	//freeing CUDA memory for bezPatches
	for (int i=0; i<m_bezCUDAPatches.size(); ++i)
		cudaFree(m_bezCUDAPatches[i]);
	for (int j=0; j<m_bezCUDAPatchesTransformed.size(); ++j)
		cudaFree(m_bezCUDAPatchesTransformed[j]);
#endif //CUDA
};

float NURBS::Basis(int i,int k, float u, int dir)
{
	float b;
	float* knotVector;
	if (dir==0)
		knotVector=this->uKnotVector;
	else
		knotVector=this->vKnotVector;

	if (k>0)
	{
		float n1 = (u-knotVector[i])*Basis(i,k-1,u,dir);
		float d1 = knotVector[i+k] - knotVector[i];
		float n2 = (knotVector[i+k+1] - u) * Basis(i+1,k-1,u,dir);
		float d2 = knotVector[i+k+1] - knotVector[i+1];
		float a2,b2;
		if (d1 > DELTA || d1 < -DELTA)
			a2 = n1 / d1;
		else
			a2 = 0;
		if (d2 > DELTA || d2 < -DELTA)
			b2 = n2 / d2;
		else
			b2 = 0;
		b = a2 + b2;
	}
	else
	{
		if (u >= knotVector[i] && u < knotVector[i+1])
			b = 1;
		else
			b = 0;
	}
	return b;
}

void NURBS::Evaluate(float u,float v, float point[3])
{
	Float3 c;
	float r = 0;
	int nU = this->uPoints;
	int kU = this->uOrder;
	int nV = this->vPoints;
	int kV = this->vOrder;
	int uPos = 0;
	int vPos = 0;
	while((u>=this->uKnotVector[uPos]) && (uPos<nU+kU))
		uPos++;
	while((v>=this->vKnotVector[vPos]) && (vPos<nV+kV))
		vPos++;

	if (u==1)
		uPos-=2*kU;
	else
		uPos-=kU;

	if (v==1)
		vPos-=2*kV;
	else
		vPos-=kV;

	float* uBasisFunc=new float[kU+1];
	float* vBasisFunc=new float[kV+1];
	BSplineBasis(kU-1, u, uPos, this->uKnotVector, uBasisFunc);
	BSplineBasis(kV-1, v, vPos, this->vKnotVector, vBasisFunc);
	for (int j=0;j<kV;j++)
	{
		for (int i=0;i<kU;i++)
		{
			float bU = uBasisFunc[i];
			float bV = vBasisFunc[j];
			c[0] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 0] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[1] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 1] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[2] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 2] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			r += bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
		}
	}
	if (r!=0)
	{
		c[0] /= r;
		c[1] /= r;
		c[2] /= r;
	}
	delete [] uBasisFunc;
	delete [] vBasisFunc;

	point[0] = c[0];
	point[1] = c[1];
	point[2] = c[2];
}

Float3 NURBS::Evaluate(float u,float v)
{
	Float3 c;
	float r = 0;
	int nU = this->uPoints;
	int kU = this->uOrder;
	int nV = this->vPoints;
	int kV = this->vOrder;
	int uPos = 0;
	int vPos = 0;
	while((u>=this->uKnotVector[uPos]) && (uPos<nU+kU))
		uPos++;
	while((v>=this->vKnotVector[vPos]) && (vPos<nV+kV))
		vPos++;

	if (u==1)
		uPos-=2*kU;
	else
		uPos-=kU;

	if (v==1)
		vPos-=2*kV;
	else
		vPos-=kV;

	float* uBasisFunc=new float[kU+1];
	float* vBasisFunc=new float[kV+1];
	BSplineBasis(kU-1, u, uPos, this->uKnotVector, uBasisFunc);
	BSplineBasis(kV-1, v, vPos, this->vKnotVector, vBasisFunc);
	for (int j=0;j<kV;j++)
	{
		for (int i=0;i<kU;i++)
		{
			float bU = uBasisFunc[i];
			float bV = vBasisFunc[j];
			c[0] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 0] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[1] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 1] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[2] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 2] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			r += bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
		}
	}
	if (r!=0)
	{
		c[0] /= r;
		c[1] /= r;
		c[2] /= r;
	}
	delete [] uBasisFunc;
	delete [] vBasisFunc;
	return c;
}

Float3 NURBS::EvaluateNormal(float u,float v)
{
	Float4 point = this->Evaluate4(u, v);
	Float4 uDerv = this->Evaluate4uDerv(u, v);
	Float4 vDerv = this->Evaluate4vDerv(u, v);

	Float3 uDervRational = (Float3(uDerv[0], uDerv[1], uDerv[2]) - (uDerv[3]/point[3])*Float3(point[0],point[1],point[2]))/point[3];
	Float3 vDervRational = (Float3(vDerv[0], vDerv[1], vDerv[2]) - (vDerv[3]/point[3])*Float3(point[0],point[1],point[2]))/point[3];
	
	Float3 normal = VectorCrossProduct(uDervRational, vDervRational);
	return normal;
}

Float4 NURBS::Evaluate4(float u,float v)
{
	Float4 c;
	int nU = this->uPoints;
	int kU = this->uOrder;
	int nV = this->vPoints;
	int kV = this->vOrder;
	int uPos = 0;
	int vPos = 0;
	while((u>=this->uKnotVector[uPos]) && (uPos<nU+kU))
		uPos++;
	while((v>=this->vKnotVector[vPos]) && (vPos<nV+kV))
		vPos++;

	if (u==1)
		uPos-=2*kU;
	else
		uPos-=kU;

	if (v==1)
		vPos-=2*kV;
	else
		vPos-=kV;

	float* uBasisFunc=new float[kU+1];
	float* vBasisFunc=new float[kV+1];
	BSplineBasis(kU-1, u, uPos, this->uKnotVector, uBasisFunc);
	BSplineBasis(kV-1, v, vPos, this->vKnotVector, vBasisFunc);
	for (int j=0;j<kV;j++)
	{
		for (int i=0;i<kU;i++)
		{
			float bU = uBasisFunc[i];
			float bV = vBasisFunc[j];
			c[0] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 0] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[1] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 1] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[2] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 2] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[3] += bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
		}
	}
	delete [] uBasisFunc;
	delete [] vBasisFunc;
	return c;
}

Float4 NURBS::Evaluate4uDerv(float u,float v)
{
	Float4 c;
	int nU = this->uPoints;
	int kU = this->uOrder;
	int nV = this->vPoints;
	int kV = this->vOrder;
	int uPos = 0;
	int vPos = 0;
	while((u>=this->uKnotVector[uPos]) && (uPos<nU+kU))
		uPos++;
	while((v>=this->vKnotVector[vPos]) && (vPos<nV+kV))
		vPos++;

	if (u==1)
		uPos-=2*kU;
	else
		uPos-=kU;

	if (v==1)
		vPos-=2*kV;
	else
		vPos-=kV;

	float* uBasisFunc=new float[kU+1];
	float* vBasisFunc=new float[kV+1];
	BSplineBasisDerv(kU-1, u, uPos, this->uKnotVector, uBasisFunc);
	BSplineBasis(kV-1, v, vPos, this->vKnotVector, vBasisFunc);
	for (int j=0;j<kV;j++)
	{
		for (int i=0;i<kU;i++)
		{
			float bU = uBasisFunc[i];
			float bV = vBasisFunc[j];
			c[0] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 0] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[1] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 1] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[2] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 2] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[3] += bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
		}
	}
	delete [] uBasisFunc;
	delete [] vBasisFunc;
	return c;
}

Float4 NURBS::Evaluate4vDerv(float u,float v)
{
	Float4 c;
	int nU = this->uPoints;
	int kU = this->uOrder;
	int nV = this->vPoints;
	int kV = this->vOrder;
	int uPos = 0;
	int vPos = 0;
	while((u>=this->uKnotVector[uPos]) && (uPos<nU+kU))
		uPos++;
	while((v>=this->vKnotVector[vPos]) && (vPos<nV+kV))
		vPos++;

	if (u==1)
		uPos-=2*kU;
	else
		uPos-=kU;

	if (v==1)
		vPos-=2*kV;
	else
		vPos-=kV;

	float* uBasisFunc=new float[kU+1];
	float* vBasisFunc=new float[kV+1];
	BSplineBasis(kU-1, u, uPos, this->uKnotVector, uBasisFunc);
	BSplineBasisDerv(kV-1, v, vPos, this->vKnotVector, vBasisFunc);
	for (int j=0;j<kV;j++)
	{
		for (int i=0;i<kU;i++)
		{
			float bU = uBasisFunc[i];
			float bV = vBasisFunc[j];
			c[0] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 0] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[1] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 1] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[2] += cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 2] * bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
			c[3] += bU * bV * cntlPoints[(vPos+j)*uPoints*4 + (uPos+i)*4 + 3];
		}
	}
	delete [] uBasisFunc;
	delete [] vBasisFunc;
	return c;
}

#ifndef VBO
void NURBS::DrawNURBS(GLParameters* glParam, float transparency)
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(this->transformationMatrix);

	GLfloat mat_ambient[]  = {float(this->ka*this->kdColor[0]),float(this->ka*this->kdColor[1]),float(this->ka*this->kdColor[2]), 1.0 };
	GLfloat mat_diffuse[]  = {float(this->kdColor[0]),float(this->kdColor[1]),float(this->kdColor[2]), 1.0 };
	GLfloat mat_specular[] = {float(this->ksColor[0]),float(this->ksColor[1]),float(this->ksColor[2]), 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT , mat_ambient);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, this->shininess);	
	
	int uNum = this->uNumDisplay;
	int vNum = this->vNumDisplay;

	cgGLEnableProfile(glParam->cgFragmentProfile);
	if (this->trimmed && this->textured)
	{
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
		cgGLBindProgram(glParam->trimOverlayProgram);
		cgGLSetTextureParameter(glParam->trimOverlayTrimTextureParam, this->trimTexture);
		cgGLEnableTextureParameter(glParam->trimOverlayTrimTextureParam);
		cgGLSetTextureParameter(glParam->trimOverlayTextureParam, this->texture);
		cgGLEnableTextureParameter(glParam->trimOverlayTextureParam);

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,this->texture);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,this->trimTexture);
	}
	else if (this->trimmed)
	{
		cgGLBindProgram(glParam->trimProgram);
		cgGLSetTextureParameter(glParam->trimTextureParam, this->trimTexture);
		cgGLEnableTextureParameter(glParam->trimTextureParam);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,this->trimTexture);
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
	}
	else if (this->textured)
	{
		cgGLBindProgram(glParam->passiveTextureProgram);
		cgGLSetTextureParameter(glParam->passiveTextureParam, this->texture);
		cgGLEnableTextureParameter(glParam->passiveTextureParam);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,this->texture);
		glEnable(GL_TEXTURE_RECTANGLE_ARB);
	}
	else
		cgGLBindProgram(glParam->passiveProgram);

	for (int vCount=0; vCount<vNum; vCount++)
	{
		for (int uCount=0; uCount<uNum; uCount++)
		{
			float uTex0 = float(uCount/(uNum*1.0))*this->trimWidth;
			float vTex0 = float(vCount/(vNum*1.0))*this->trimHeight;
			float uTex1 = float((uCount+1)/(uNum*1.0))*this->trimWidth;
			float vTex1 = float((vCount+1)/(vNum*1.0))*this->trimHeight;

			//float uTex0 = float(uCount*this->uParamRange/uNum) + this->uParamOffset;
			//float vTex0 = float(vCount*this->vParamRange/vNum) + this->vParamOffset;
			//float uTex1 = float((uCount+1)*this->uParamRange/uNum) + this->uParamOffset;
			//float vTex1 = float((vCount+1)*this->vParamRange/vNum) + this->vParamOffset;

			Float4 cornerVertex1 = Float4(this->evaluatedPointsDisplay[vCount*(uNum+1)*4+uCount*4+0],			this->evaluatedPointsDisplay[vCount*(uNum+1)*4+uCount*4+1],			this->evaluatedPointsDisplay[vCount*(uNum+1)*4+uCount*4+2],this->evaluatedPointsDisplay[vCount*(uNum+1)*4+uCount*4+3]);
			Float4 cornerVertex2 = Float4(this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+uCount*4+0],		this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+uCount*4+1],		this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+uCount*4+2],this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+uCount*4+3]);
			Float4 cornerVertex3 = Float4(this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+(uCount+1)*4+0],	this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+(uCount+1)*4+1],	this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+(uCount+1)*4+2],this->evaluatedPointsDisplay[(vCount+1)*(uNum+1)*4+(uCount+1)*4+3]);
			Float4 cornerVertex4 = Float4(this->evaluatedPointsDisplay[vCount*(uNum+1)*4+(uCount+1)*4+0],		this->evaluatedPointsDisplay[vCount*(uNum+1)*4+(uCount+1)*4+1],		this->evaluatedPointsDisplay[vCount*(uNum+1)*4+(uCount+1)*4+2],this->evaluatedPointsDisplay[vCount*(uNum+1)*4+(uCount+1)*4+3]);

			Float3 cornerNormal1 = Float3(-this->evaluatedNormalsDisplay[vCount*(uNum+1)*3+uCount*3+0],			-this->evaluatedNormalsDisplay[vCount*(uNum+1)*3+uCount*3+1],			-this->evaluatedNormalsDisplay[vCount*(uNum+1)*3+uCount*3+2]);
			Float3 cornerNormal2 = Float3(-this->evaluatedNormalsDisplay[(vCount+1)*(uNum+1)*3+uCount*3+0],		-this->evaluatedNormalsDisplay[(vCount+1)*(uNum+1)*3+uCount*3+1],		-this->evaluatedNormalsDisplay[(vCount+1)*(uNum+1)*3+uCount*3+2]);
			Float3 cornerNormal3 = Float3(-this->evaluatedNormalsDisplay[(vCount+1)*(uNum+1)*3+(uCount+1)*3+0],	-this->evaluatedNormalsDisplay[(vCount+1)*(uNum+1)*3+(uCount+1)*3+1],	-this->evaluatedNormalsDisplay[(vCount+1)*(uNum+1)*3+(uCount+1)*3+2]);
			Float3 cornerNormal4 = Float3(-this->evaluatedNormalsDisplay[vCount*(uNum+1)*3+(uCount+1)*3+0],		-this->evaluatedNormalsDisplay[vCount*(uNum+1)*3+(uCount+1)*3+1],		-this->evaluatedNormalsDisplay[vCount*(uNum+1)*3+(uCount+1)*3+2]);
			
			Float2 cornerTex1	 = Float2(uTex0,vTex0);
			Float2 cornerTex2	 = Float2(uTex0,vTex1);
			Float2 cornerTex3	 = Float2(uTex1,vTex1);
			Float2 cornerTex4	 = Float2(uTex1,vTex0);

			float offset = -glParam->offsetDistance*this->normalSign;
			if (offset > 0)
			{
				cornerVertex1 += (cornerNormal1*offset);
				cornerVertex2 += (cornerNormal2*offset);
				cornerVertex3 += (cornerNormal3*offset);
				cornerVertex4 += (cornerNormal4*offset);
			}

			glBegin(GL_QUADS);
				glTexCoord2f(cornerTex1[0],cornerTex1[1]);
				glNormal3f(cornerNormal1[0],cornerNormal1[1],cornerNormal1[2]);
				glVertex4f(cornerVertex1[0],cornerVertex1[1],cornerVertex1[2],cornerVertex1[3]);

				glTexCoord2f(cornerTex2[0],cornerTex2[1]);
				glNormal3f(cornerNormal2[0],cornerNormal2[1],cornerNormal2[2]);
				glVertex4f(cornerVertex2[0],cornerVertex2[1],cornerVertex2[2],cornerVertex2[3]);

				glTexCoord2f(cornerTex3[0],cornerTex3[1]);
				glNormal3f(cornerNormal3[0],cornerNormal3[1],cornerNormal3[2]);
				glVertex4f(cornerVertex3[0],cornerVertex3[1],cornerVertex3[2],cornerVertex3[3]);

				glTexCoord2f(cornerTex4[0],cornerTex4[1]);
				glNormal3f(cornerNormal4[0],cornerNormal4[1],cornerNormal4[2]);
				glVertex4f(cornerVertex4[0],cornerVertex4[1],cornerVertex4[2],cornerVertex4[3]);
			glEnd();

/*			VectorNormalize(cornerNormal1);
			glBegin(GL_LINES);
			glVertex4f(cornerVertex1[0], cornerVertex1[1], cornerVertex1[2], cornerVertex1[3]);
			glVertex3f(cornerVertex1[0]/cornerVertex1[3]+cornerNormal1[0]*20, cornerVertex1[1]/cornerVertex1[3]+cornerNormal1[1]*20, cornerVertex1[2]/cornerVertex1[3]+cornerNormal1[2]*20);
			glEnd();

			glPointSize(8.0);
			glEnable(GL_COLOR_MATERIAL);
			glColor3f(1,0,0);
			glBegin(GL_POINTS);
				glVertex4f(cornerVertex1[0],cornerVertex1[1],cornerVertex1[2],cornerVertex1[3]);
				glVertex4f(cornerVertex2[0],cornerVertex2[1],cornerVertex2[2],cornerVertex2[3]);
				glVertex4f(cornerVertex3[0],cornerVertex3[1],cornerVertex3[2],cornerVertex3[3]);
				glVertex4f(cornerVertex4[0],cornerVertex4[1],cornerVertex4[2],cornerVertex4[3]);
			glEnd();
*/
		}
	}
	if (this->trimmed && this->textured)
	{
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		cgGLDisableTextureParameter(glParam->trimOverlayTrimTextureParam);
		cgGLDisableTextureParameter(glParam->trimOverlayTextureParam);
	}
	else if (this->trimmed)
	{
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		cgGLDisableTextureParameter(glParam->trimTextureParam);
	}
	else if (this->textured)
	{
		glDisable(GL_TEXTURE_RECTANGLE_ARB);
		cgGLDisableTextureParameter(glParam->passiveTextureParam);
	}
	cgGLDisableProfile(glParam->cgFragmentProfile);

	if (this->pickedPoints.size()>0)
		this->DrawPickedPoint(glParam);

	//if (glParam->controlMesh && (glParam->pickedSurfNum == this->surfaceID || glParam->pickedObjectNum*10000 + glParam->pickedFaceNum == this->surfaceID))
	if (glParam->controlMesh)
		this->DrawControlMesh(glParam);

	glPopMatrix();

	if (glParam->drawBBox)
		this->DrawBoundingBox(glParam);

//	glFlush();
}
#else
void NURBS::DrawNURBS(GLParameters* glParam, float transparency)
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
#ifndef BREP
	glMultMatrixf(this->transformationMatrix);
#endif

	GLfloat mat_ambient[]  = {float(this->ka*this->kdColor[0]),float(this->ka*this->kdColor[1]),float(this->ka*this->kdColor[2]), transparency };
	GLfloat mat_diffuse[]  = {float(this->kdColor[0]),float(this->kdColor[1]),float(this->kdColor[2]), transparency };
	GLfloat mat_specular[] = {float(this->ksColor[0]),float(this->ksColor[1]),float(this->ksColor[2]), transparency };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE , mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT , mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, &this->shininess);

	int uNum = this->uNumDisplay;
	int vNum = this->vNumDisplay;

	cgGLEnableProfile(glParam->cgFragmentProfile);
	if (this->trimmed && this->textured)
	{
		glEnable(TRIM_TEXTURE_FORMAT);
		cgGLBindProgram(glParam->trimOverlayProgram);
		cgGLSetTextureParameter(glParam->trimOverlayTrimTextureParam, this->trimTexture);
		cgGLEnableTextureParameter(glParam->trimOverlayTrimTextureParam);
		cgGLSetTextureParameter(glParam->trimOverlayTextureParam, this->texture);
		cgGLEnableTextureParameter(glParam->trimOverlayTextureParam);

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(TRIM_TEXTURE_FORMAT,this->texture);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(TRIM_TEXTURE_FORMAT,this->trimTexture);
	}
	else if (this->trimmed)
	{
		glActiveTexture(GL_TEXTURE0);
		glEnable(TRIM_TEXTURE_FORMAT);
		glBindTexture(TRIM_TEXTURE_FORMAT, this->trimTexture);
		cgGLBindProgram(glParam->trimProgram);
		cgGLSetTextureParameter(glParam->trimTextureParam, this->trimTexture);
		cgGLEnableTextureParameter(glParam->trimTextureParam);
	}
	else if (this->textured)
	{
		cgGLBindProgram(glParam->passiveTextureProgram);
		cgGLSetTextureParameter(glParam->passiveTextureParam, this->texture);
		cgGLEnableTextureParameter(glParam->passiveTextureParam);
		glBindTexture(TRIM_TEXTURE_FORMAT,this->texture);
		glEnable(TRIM_TEXTURE_FORMAT);
	}
	else
		cgGLBindProgram(glParam->passiveProgram);
	
	glEnable(GL_VERTEX_ARRAY);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, this->ptVBO);  // BIND VBO
	glVertexPointer(4, GL_FLOAT, 0, 0);
	glEnable(GL_NORMAL_ARRAY);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, this->normVBO); 
	glNormalPointer(GL_FLOAT,0,0);
	glEnable(GL_TEXTURE_COORD_ARRAY);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, this->texVBO);
	glTexCoordPointer(2,GL_FLOAT,0,0);
	glDrawElements(GL_QUADS,uNum*vNum*4,GL_UNSIGNED_INT, this->indexArray);
	
	/*
	glColor3f(0.0,1.0,0.0);
	glEnable(GL_COLOR_MATERIAL);
	glPointSize(10.0);
	glDrawElements(GL_POINTS,uNum*vNum*4,GL_UNSIGNED_INT, this->indexArray);
	glDisable(GL_COLOR_MATERIAL);
	*/
//	glFlush();

	if (this->trimmed && this->textured)
	{
		glDisable(TRIM_TEXTURE_FORMAT);
		cgGLDisableTextureParameter(glParam->trimOverlayTrimTextureParam);
		cgGLDisableTextureParameter(glParam->trimOverlayTextureParam);
	}
	else if (this->trimmed)
	{
		glDisable(TRIM_TEXTURE_FORMAT);
		cgGLDisableTextureParameter(glParam->trimTextureParam);
	}
	else if (this->textured)
	{
		glDisable(TRIM_TEXTURE_FORMAT);
		cgGLDisableTextureParameter(glParam->passiveTextureParam);
	}

	cgGLDisableProfile(glParam->cgFragmentProfile);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
	glBindBufferARB(GL_ARRAY_BUFFER, 0);
	glDisableClientState (GL_VERTEX_ARRAY);

	if (this->pickedPoints.size()>0)
		this->DrawPickedPoint(glParam);

//	if (glParam->controlMesh && (glParam->pickedSurfNum == this->surfaceID || glParam->pickedObjectNum*10000 + glParam->pickedFaceNum == this->surfaceID))
	if (glParam->controlMesh)
		this->DrawControlMesh(glParam);

	//Rendering Bezier Patches if they exist
	//if (!this->m_bezPatches.empty())
		//this->DrawBezierPatchControlPoints(glParam);

	glPopMatrix();

	if (glParam->drawBBox)
//		this->DrawOBB();
		this->DrawBoundingBox(glParam);

//	glFlush();
}
#endif

void NURBS::DrawPickedPoint(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_ONE_MINUS_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_COLOR_MATERIAL);
	glPointSize(4.0);
	glColor4d(1, 0, 0, 0.1);
	for(int i = 0; i < this->pickedPoints.size(); i++)
	{
		glBegin(GL_POINTS);
		glVertex4f(pickedPoints[i][0], pickedPoints[i][1], pickedPoints[i][2], pickedPoints[i][3]);
		glEnd();
	}
	glPopAttrib();
}

int NURBS::PickControlPoint(int x, int y, float* projectionMatrix, float* modelViewMatrix)
{
	int pickedPointIndex = -1;
	GLuint selectBuf[512];
	GLint viewport[4];
	GLint hits;
	
	glGetIntegerv(GL_VIEWPORT, viewport);
	
	glSelectBuffer(512, selectBuf);
	glRenderMode(GL_SELECT);
	
	glInitNames();
	glPushName(0);
	
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	//  create 5x5 pixel picking region near cursor location 
	gluPickMatrix((GLdouble) x, (GLdouble) (viewport[3] - y), 15.0, 15.0, viewport);
	glMultMatrixf(projectionMatrix);
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf(modelViewMatrix);
	glMultMatrixf(this->transformationMatrix);

	for (int v = 0; v < this->vPoints ; v++)
	{
		for (int u = 0; u < this->uPoints ; u++)
		{
			float x = this->cntlPoints[v*this->uPoints*4+u*4+0];
			float y = this->cntlPoints[v*this->uPoints*4+u*4+1];
			float z = this->cntlPoints[v*this->uPoints*4+u*4+2];
			float w = this->cntlPoints[v*this->uPoints*4+u*4+3];

			glLoadName(v*this->uPoints+u);
			glBegin(GL_POINTS);
				glVertex4f(x,y,z,w);
			glEnd();
		}
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glFlush();
	hits = glRenderMode(GL_RENDER);
	if(hits)
	{ 
		if(hits == 1)
			pickedPointIndex = selectBuf[3];
		else
		{	
			double nearest = -1.0e6;
			for(int i = 0; i < hits; i++)
			{
				double zValue = (double)selectBuf[2+4*i]/0x7fffffff;
				if(zValue > nearest)
				{
					nearest = zValue;
					pickedPointIndex = selectBuf[3+4*i];
				}
			}
		}
	}
	return pickedPointIndex;
}


void NURBS::DrawBoundingBox(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
//	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA);
	glEnable(GL_COLOR_MATERIAL);
//	glEnable(GL_CULL_FACE);
	glLineWidth(2);

	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	Float4 color;
	if (this->surfaceID < 1)
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


	float mat[16], inv[16];
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	InvertMatrix(glParam->modelViewMatrix,inv);
	glMultMatrixf(inv);
#ifndef BREP
	glMultMatrixf(this->transformationMatrix);
#endif

	int uNum = this->uNumDisplay;
	int vNum = this->vNumDisplay;
	for (int vCount=0; vCount < vNum; vCount++)
	{
		for (int uCount=0; uCount < uNum; uCount++)
		{
			Float3 v1 = Float3(minPointsDisplay[vCount*(uNum)*4+4*uCount+0], minPointsDisplay[vCount*(uNum)*4+4*uCount+1], minPointsDisplay[vCount*(uNum)*4+4*uCount+2]);
			Float3 v2 = Float3(maxPointsDisplay[vCount*(uNum)*4+4*uCount+0], maxPointsDisplay[vCount*(uNum)*4+4*uCount+1], maxPointsDisplay[vCount*(uNum)*4+4*uCount+2]);
			bool drawBox = true;
			if (this->evaluatedIntersection)
			{
				drawBox = false;
				for (int k = 0; k < this->numIntersectingBoxes; k++)
				{
					float uVal = (this->intersectingBoxes[4*k+0]+this->intersectingBoxes[4*k+1])/2.0;
					float vVal = (this->intersectingBoxes[4*k+2]+this->intersectingBoxes[4*k+3])/2.0;
					float uValMin = (uCount)/(uNum*1.0);
					float vValMin = (vCount)/(vNum*1.0);
					float uValMax = (uCount+1)/(uNum*1.0);
					float vValMax = (vCount+1)/(vNum*1.0);
					if (uVal > uValMin && uVal <= uValMax && vVal > vValMin && vVal <= vValMax)
						drawBox = true;
				}
			}
			drawBox = true;

			if (drawBox)
			{
				Float3 xAxis = Float3(1,0,0);
				Float3 yAxis = Float3(0,1,0);
				Float3 zAxis = Float3(0,0,1);
			
				xAxis *= VectorDotProduct(xAxis,(v2-v1));
				yAxis *= VectorDotProduct(yAxis,(v2-v1));
				zAxis *= VectorDotProduct(zAxis,(v2-v1));

				Float3 c1 = v1;
				Float3 c2 = v1+yAxis;
				Float3 c3 = v1+xAxis+yAxis;
				Float3 c4 = v1+xAxis;

				Float3 c5 = v1+zAxis;
				Float3 c6 = v1+xAxis+zAxis;
				Float3 c7 = v1+xAxis+yAxis+zAxis;
				Float3 c8 = v1+yAxis+zAxis;
				
				glColor4f(color[0],color[1],color[2],color[3]);
				
				glBegin(GL_QUADS);
				glNormal3f(0,0,-1);
				glVertex3f(c1[0],c1[1],c1[2]);
				glVertex3f(c2[0],c2[1],c2[2]);
				glVertex3f(c3[0],c3[1],c3[2]);
				glVertex3f(c4[0],c4[1],c4[2]);

				glNormal3f(0,0,1);
				glVertex3f(c5[0],c5[1],c5[2]);
				glVertex3f(c6[0],c6[1],c6[2]);
				glVertex3f(c7[0],c7[1],c7[2]);
				glVertex3f(c8[0],c8[1],c8[2]);

				glNormal3f(0,-1,0);
				glVertex3f(c1[0],c1[1],c1[2]);
				glVertex3f(c4[0],c4[1],c4[2]);
				glVertex3f(c6[0],c6[1],c6[2]);
				glVertex3f(c5[0],c5[1],c5[2]);

				glNormal3f(0,1,0);
				glVertex3f(c2[0],c2[1],c2[2]);
				glVertex3f(c8[0],c8[1],c8[2]);
				glVertex3f(c7[0],c7[1],c7[2]);
				glVertex3f(c3[0],c3[1],c3[2]);

				glNormal3f(-1,0,0);
				glVertex3f(c1[0],c1[1],c1[2]);
				glVertex3f(c5[0],c5[1],c5[2]);
				glVertex3f(c8[0],c8[1],c8[2]);
				glVertex3f(c2[0],c2[1],c2[2]);

				glNormal3f(1,0,0);
				glVertex3f(c4[0],c4[1],c4[2]);
				glVertex3f(c3[0],c3[1],c3[2]);
				glVertex3f(c7[0],c7[1],c7[2]);
				glVertex3f(c6[0],c6[1],c6[2]);
				glEnd();
			}
		}
	}	
	glPopMatrix();
	glPopAttrib();
}

void NURBS::DrawOverlayTrimTexture(GLParameters* glParam, Float4 pos)
{
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
	glColor3f(this->kdColor[0],this->kdColor[1],this->kdColor[2]);

	int w = this->trimWidth;
	int h = this->trimHeight;

	cgGLEnableProfile(glParam->cgFragmentProfile);
	if (this->trimmed && this->textured)
	{
		glEnable(TRIM_TEXTURE_FORMAT);
		cgGLBindProgram(glParam->trimOverlayProgram);
		cgGLSetTextureParameter(glParam->trimOverlayTrimTextureParam, this->trimTexture);
		cgGLEnableTextureParameter(glParam->trimOverlayTrimTextureParam);
		cgGLSetTextureParameter(glParam->trimOverlayTextureParam, this->texture);
		cgGLEnableTextureParameter(glParam->trimOverlayTextureParam);

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,this->texture);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,this->trimTexture);
	}
	else if (this->trimmed)
	{
		glEnable(TRIM_TEXTURE_FORMAT);
		glBindTexture(TRIM_TEXTURE_FORMAT, this->trimTexture);
		cgGLBindProgram(glParam->trimProgram);
		cgGLSetTextureParameter(glParam->trimTextureParam, this->trimTexture);
		cgGLEnableTextureParameter(glParam->trimTextureParam);
	}
	else if (this->textured)
	{
		glEnable(TRIM_TEXTURE_FORMAT);
		cgGLBindProgram(glParam->passiveTextureProgram);
		cgGLSetTextureParameter(glParam->passiveTextureParam, this->texture);
		cgGLEnableTextureParameter(glParam->passiveTextureParam);
		glBindTexture(TRIM_TEXTURE_FORMAT, this->texture);
	}
	else
		cgGLBindProgram(glParam->passiveProgram);
		
	if (TRIM_TEXTURE_FORMAT == GL_TEXTURE_2D)
		w=h=1;
	glBegin(GL_QUADS);
	glNormal3f(0,0,-1);
	//If using GL_TEXTURE_RECTANGLE_ARB coords go from [0..N]x[0..N]

	glTexCoord2f(0, 0); 
	glVertex3f(-1, -1, 1);

	glTexCoord2f(w, 0);
	glVertex3f( 1, -1, 1);

	glTexCoord2f(w, h);
	glVertex3f( 1,  1, 1);

	glTexCoord2f(0, h);
	glVertex3f( -1,  1, 1);

	glEnd();
//	glFlush();

	if (this->trimmed && this->textured)
	{
		cgGLDisableTextureParameter(glParam->trimOverlayTrimTextureParam);
		cgGLDisableTextureParameter(glParam->trimOverlayTextureParam);
	}
	else if (this->trimmed)
		cgGLDisableTextureParameter(glParam->trimTextureParam);
	else if (this->textured)
		cgGLDisableTextureParameter(glParam->passiveTextureParam);

	cgGLDisableProfile(glParam->cgFragmentProfile);
	glDisable(TRIM_TEXTURE_FORMAT);

	// Restore the previous views
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

void NURBS::DrawControlMesh(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	//glDisable(GL_DEPTH_TEST);
	glColor3d(0.3, 0.3, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glPointSize(7.0);
	glLineWidth(2.0);
	
	for (int v = 0; v < this->vPoints-1; v++)
	{
		for (int u = 0; u < this->uPoints-1; u++)
		{
			float x1 = this->cntlPoints[v*this->uPoints*4+u*4+0];
			float y1 = this->cntlPoints[v*this->uPoints*4+u*4+1];
			float z1 = this->cntlPoints[v*this->uPoints*4+u*4+2];
			float w1 = this->cntlPoints[v*this->uPoints*4+u*4+3];

			float x2 = this->cntlPoints[v*this->uPoints*4+(u+1)*4+0];
			float y2 = this->cntlPoints[v*this->uPoints*4+(u+1)*4+1];
			float z2 = this->cntlPoints[v*this->uPoints*4+(u+1)*4+2];
			float w2 = this->cntlPoints[v*this->uPoints*4+(u+1)*4+3];

			float x3 = this->cntlPoints[(v+1)*this->uPoints*4+(u+1)*4+0];
			float y3 = this->cntlPoints[(v+1)*this->uPoints*4+(u+1)*4+1];
			float z3 = this->cntlPoints[(v+1)*this->uPoints*4+(u+1)*4+2];
			float w3 = this->cntlPoints[(v+1)*this->uPoints*4+(u+1)*4+3];
					  
			float x4 = this->cntlPoints[(v+1)*this->uPoints*4+u*4+0];
			float y4 = this->cntlPoints[(v+1)*this->uPoints*4+u*4+1];
			float z4 = this->cntlPoints[(v+1)*this->uPoints*4+u*4+2];
			float w4 = this->cntlPoints[(v+1)*this->uPoints*4+u*4+3];

			glBegin(GL_QUADS);
				glVertex4f(x1,y1,z1,w1);
				glVertex4f(x2,y2,z2,w2);
				glVertex4f(x3,y3,z3,w3);
				glVertex4f(x4,y4,z4,w4);
			glEnd();
		}
	}

	for (int v = 0; v < this->vPoints ; v++)
	{
		for (int u = 0; u < this->uPoints ; u++)
		{
			float x = this->cntlPoints[v*this->uPoints*4+u*4+0];
			float y = this->cntlPoints[v*this->uPoints*4+u*4+1];
			float z = this->cntlPoints[v*this->uPoints*4+u*4+2];
			float w = this->cntlPoints[v*this->uPoints*4+u*4+3];

			if ((glParam->pickedSurfNum == this->surfaceID || glParam->pickedObjectNum*10000 + glParam->pickedFaceNum == this->surfaceID) && v*this->uPoints+u == glParam->pickedControlPointNum)
				glColor3f(0, 0.2, 0.9);
			else
				glColor3f(1.0, 0.3, 0);
			glBegin(GL_POINTS);
				glVertex4f(x,y,z,w);
			glEnd();
		}
	}
	glPopAttrib();
}

int NURBS::TrimInOut(int u, int v, int uNum, int vNum)
{
	if (this->trimmed)
	{
		if (this->evalParams->trimData[v*(uNum+1)+u] == 0)
			return 0;
		else
			return 1;
	}
	else
		return 1;
}

void NURBS::GenerateTrimData(int uNum, int vNum, GLParameters* glParam)
{
	// Set up GPU computations
	StartGPUComputation(glParam);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);

	// Initialize the trim texture
	glEnable(TRIM_TEXTURE_FORMAT);

	glBindTexture(TRIM_TEXTURE_FORMAT, this->evalParams->trimTexture);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(TRIM_TEXTURE_FORMAT, 0, LUMINANCE_FLOAT_FORMAT, uNum+1, vNum+1, 0, GL_LUMINANCE, GL_FLOAT, 0);

//	glOrtho(0,1,0,1,0,1);
	gluOrtho2D(0,1,0,1);

	glViewport(0, 0, uNum+1, vNum+1);

	// Bind the fragment program
	cgGLBindProgram(glParam->passiveProgram);

	// Tell OGL to draw to COLOR_ATTACHMENT_0
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, TRIM_TEXTURE_FORMAT, this->evalParams->trimTexture, 0);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glBlendFunc(GL_ONE_MINUS_DST_COLOR,GL_ZERO);	// Set The Blending Function For Translucency
	glEnable(GL_BLEND);
	glColor4f(1,1,1,1);
	for (int curveID = 0; curveID<this->trimCurves.size(); curveID++)
	{
		int uCurveNum;
		Float2 tempPoint = Float2(0,0);
		if (this->trimCurves[curveID]->order == 2 && this->trimCurves[curveID]->nPoints == 2)
		{
			Float2 evalPt1 = this->trimCurves[curveID]->Evalutate(0);
			Float2 evalPt2 = this->trimCurves[curveID]->Evalutate(1);
			glBegin(GL_TRIANGLES);
				glVertex2f(tempPoint[0],tempPoint[1]);
				glVertex2f(evalPt1[0],evalPt1[1]);
				glVertex2f(evalPt2[0],evalPt2[1]);
			glEnd();
		}
		else
		{
			uCurveNum = (int)(this->trimCurveLOD*25);
			float* curvePts = new float[(uCurveNum+1)*2];
			for (int i=0; i<=uCurveNum; i++)
			{
				float u = i*1.0/uCurveNum;
//				if (u==1)
//					u-=DELTA;
				Float2 evalPt = this->trimCurves[curveID]->Evalutate(u);
				curvePts[i*2+0] = evalPt[0];
				curvePts[i*2+1] = evalPt[1];
			}
			for (int i=0; i<uCurveNum; i++)
			{
				glBegin(GL_TRIANGLES);
					glVertex2f(tempPoint[0],tempPoint[1]);
					glVertex2f(curvePts[2*i+0],curvePts[2*i+1]);
					glVertex2f(curvePts[2*(i+1)+0],curvePts[2*(i+1)+1]);
				glEnd();
			}
			delete [] curvePts;
		}
	}

//	glFlush();

	// Read back
	if (this->evalParams->trimData != NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,this->evalParams->trimData);
//		WriteToFileInt(this->evalParams->trimData,uNum+1,vNum+1);
	}

	glDisable(GL_BLEND);
	glDisable(TRIM_TEXTURE_FORMAT);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

void NURBS::GenerateTrimTexture(GLParameters* glParam)
{
	// Set up GPU computations
	StartGPUComputation(glParam);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);

	// Initialize the trim texture
	glEnable(TRIM_TEXTURE_FORMAT);
	glBindTexture(TRIM_TEXTURE_FORMAT, this->trimTexture);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(TRIM_TEXTURE_FORMAT, 0, GL_RGBA, this->trimWidth, this->trimHeight, 0, GL_RGBA, GL_FLOAT, 0);

//	glOrtho(0,1,0,1,0,1);
	gluOrtho2D(0,1,0,1);

	glViewport(0, 0, this->trimWidth, this->trimHeight);

	// Bind the fragment program
	cgGLBindProgram(glParam->passiveProgram);

	// Tell OGL to draw to COLOR_ATTACHMENT_0
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, TRIM_TEXTURE_FORMAT, this->trimTexture, 0);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glBlendFunc(GL_ONE_MINUS_DST_COLOR,GL_ZERO);	// Set The Blending Function For Translucency
	glEnable(GL_BLEND);
	glColor4f(1,1,1,1);
	for (int curveID = 0; curveID<this->trimCurves.size(); curveID++)
	{
		int uNum;
		Float2 tempPoint = Float2(0,0);
		if (this->trimCurves[curveID]->order == 2 && this->trimCurves[curveID]->nPoints == 2)
		{
			Float2 evalPt1 = this->trimCurves[curveID]->Evalutate(0);
			Float2 evalPt2 = this->trimCurves[curveID]->Evalutate(1);
			glBegin(GL_TRIANGLES);
				glVertex2f(tempPoint[0],tempPoint[1]);
				glVertex2f(evalPt1[0],evalPt1[1]);
				glVertex2f(evalPt2[0],evalPt2[1]);
			glEnd();
		}
		else
		{
			uNum = (int)(this->trimCurveLOD*25);
			float* curvePts = new float[(uNum+1)*2];
			for (int i=0; i<=uNum; i++)
			{
				float u = i*1.0/uNum;
//				if (u==1)
//					u-=DELTA;
				Float2 evalPt = this->trimCurves[curveID]->Evalutate(u);
				curvePts[i*2+0] = evalPt[0];
				curvePts[i*2+1] = evalPt[1];
			}
			for (int i=0; i<uNum; i++)
			{
				glBegin(GL_TRIANGLES);
					glVertex2f(tempPoint[0],tempPoint[1]);
					glVertex2f(curvePts[2*i+0],curvePts[2*i+1]);
					glVertex2f(curvePts[2*(i+1)+0],curvePts[2*(i+1)+1]);
				glEnd();
			}
			delete [] curvePts;
		}
	}

#ifdef ANIMATE
//	glRotated(angle,1,1,0);
#endif
//	glutSolidTeapot(0.25);

//	glFlush();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[this->trimWidth*this->trimHeight];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,this->trimWidth,this->trimHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,this->trimWidth,this->trimHeight);
#endif

	glDisable(GL_BLEND);
	glDisable(TRIM_TEXTURE_FORMAT);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

void NURBS::GenerateIntersectionOverlayTexture(float* intersectionBoxes, int numIntersectionBoxes, int boxNum, GLParameters* glParam)
{
	// Set up GPU computations
	StartGPUComputation(glParam);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);
	glEnable(TRIM_TEXTURE_FORMAT);

	//Set up the texture
	this->textured	= true;		

	//Initialize the texture
	glBindTexture(TRIM_TEXTURE_FORMAT, this->texture);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(TRIM_TEXTURE_FORMAT, GL_TEXTURE_WRAP_T, GL_CLAMP);

	gluOrtho2D(0,1,0,1);
	glPointSize(2.0);
	glColor4f(1,0,0,1);
	glViewport(0, 0, this->trimWidth, this->trimHeight);

	cgGLBindProgram(glParam->passiveProgram);

//	int maxMipMapLevels = GetExponent2(max(this->texHeight, this->texWidth));
	int maxMipMapLevels = 1;
	for (int mipMapLevel = 0; mipMapLevel < maxMipMapLevels; mipMapLevel++)
	{
		// Generate the buffer
		glTexImage2D(TRIM_TEXTURE_FORMAT, mipMapLevel, GL_RGBA, this->trimWidth, this->trimHeight, 0, GL_RGBA, GL_FLOAT, 0);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, TRIM_TEXTURE_FORMAT, this->texture, mipMapLevel);

		glClearColor(0,0,0,1);
		glClear(GL_COLOR_BUFFER_BIT);
		for (int k = 0; k < numIntersectionBoxes; k++)
		{
//			float uPos = (intersectionBoxes[4*k+0]+intersectionBoxes[4*k+1])/2.0;
//			float vPos = (intersectionBoxes[4*k+2]+intersectionBoxes[4*k+3])/2.0;
//			glBegin(GL_POINTS);
//			glVertex2f(uPos, vPos);
//			glEnd();

			glBegin(GL_QUADS);
				glVertex2f(intersectionBoxes[4*k+0], intersectionBoxes[4*k+2]);
				glVertex2f(intersectionBoxes[4*k+1], intersectionBoxes[4*k+2]);
				glVertex2f(intersectionBoxes[4*k+1], intersectionBoxes[4*k+3]);
				glVertex2f(intersectionBoxes[4*k+0], intersectionBoxes[4*k+3]);
			glEnd();


		}
//		glFlush();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[this->trimWidth*this->trimHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,this->trimWidth,this->trimHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFileInt(retVal,this->trimWidth,this->trimHeight);
#endif
	}
	glDisable(TRIM_TEXTURE_FORMAT);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

void NURBS::GenerateCurveOverlayTexture(GLParameters* glParam)
{
	//Set up the texture
	this->textured	= true;

	// Set up GPU computations
	StartGPUComputation(glParam);
	glDisable(GL_LIGHTING);
	glDisable(GL_NORMALIZE);

	//Initialize the texture
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->texture);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, this->trimWidth, this->trimHeight, 0, GL_RGBA, GL_FLOAT, 0);

	gluOrtho2D(0,1,0,1);
	glViewport(0, 0, this->trimWidth, this->trimHeight);

	cgGLEnableProfile(glParam->cgFragmentProfile);
	cgGLBindProgram(glParam->passiveProgram);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, this->texture, 0);
	glEnable(GL_COLOR_ATTACHMENT0_EXT);

	glClearColor(0,0,0,1);
	glClear(GL_COLOR_BUFFER_BIT);

	glPointSize(2.0);
	glLineWidth(2.0);
	glColor4f(1,0,.5,1);
	for (int k = 0; k < this->pickedParamPoints.size()-1; k++)
	{
//		glBegin(GL_POINTS);
//		glVertex2f(this->pickedParamPoints[k][0], this->pickedParamPoints[k][1]);
//		glEnd();

		glBegin(GL_LINES);
		glVertex2f(this->pickedParamPoints[k][0], this->pickedParamPoints[k][1]);
		glVertex2f(this->pickedParamPoints[k+1][0], this->pickedParamPoints[k+1][1]);
		glEnd();

	}
//	glFlush();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[this->texWidth*this->texHeight];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,this->texWidth,this->texHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFileInt(retVal,this->texWidth,this->texHeight);
#endif

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

float NURBS::CalculateBoundingBoxSize(void)
{	
	float maxDist = 0;
	int maxPoint1 = 0;
	int maxPoint2 = 0; 
	for (int count1 = 0; count1 < this->uPoints*this->vPoints; count1++)
	{
		for (int count2 = count1; count2 < this->uPoints*this->vPoints ; count2++)
		{
			Float3 point1 = Float3(cntlPoints[count1*4+0]/cntlPoints[count1*4+3],	cntlPoints[count1*4+1]/cntlPoints[count1*4+3], cntlPoints[count1*4+2]/cntlPoints[count1*4+3]);
			Float3 point2 = Float3(cntlPoints[count2*4+0]/cntlPoints[count2*4+3],	cntlPoints[count2*4+1]/cntlPoints[count2*4+3], cntlPoints[count2*4+2]/cntlPoints[count2*4+3]);
			float dist = VectorMagnitude(point1-point2);
			if (dist > maxDist)
			{
				maxDist = dist;
				maxPoint1 = count1;
				maxPoint2 = count2;
			}
		}
	}
//	Float3 point1 = Float3(cntlPoints[maxPoint1*4+0]/cntlPoints[maxPoint1*4+3],	cntlPoints[maxPoint1*4+1]/cntlPoints[maxPoint1*4+3], cntlPoints[maxPoint1*4+2]/cntlPoints[maxPoint1*4+3]);
//	Float3 point2 = Float3(cntlPoints[maxPoint2*4+0]/cntlPoints[maxPoint2*4+3],	cntlPoints[maxPoint2*4+1]/cntlPoints[maxPoint2*4+3], cntlPoints[maxPoint2*4+2]/cntlPoints[maxPoint2*4+3]);
	return maxDist;
}

void NURBS::SaveMesh(const char* filename, GLParameters* glParam)
{
	bool storedReadBack = glParam->readBack;
	if (this->initializedEvalParams)
		delete this->evalParams;

	this->evalParams = new EvaluationParams();
	this->initializedEvalParams = true;

	int uNum = 20;
	int vNum = 20;
	// uMin, uMax, vMin, vMax
	this->evalParams->uNum	= uNum;
	this->evalParams->vNum	= vNum;
	this->evalParams->uMin	= 0.0;
	this->evalParams->uMax	= 1.0;
	this->evalParams->vMin	= 0.0;
	this->evalParams->vMax	= 1.0;

	this->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
	glParam->readBack = true;
	this->SurfaceEvaluation(glParam, false, false);

	ofstream ofs(filename);
	if (!ofs.good())
	{
		cerr<<"Unable to open file \""<<filename<<"\""<<endl;
		abort();
	}

	//Write all the vertices
	for (int vCount = 0; vCount <= vNum; vCount++)
	{
		for (int uCount = 0; uCount <= uNum; uCount++)
		{
			Float4 vertex = Float4(this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+0],this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+1],this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+2],this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+3]);
			vertex[3] *= 4;
			ofs << "v " << vertex[0]/vertex[3] << " " << vertex[1]/vertex[3] << " " << vertex[2]/vertex[3] << endl;
		}
	}

	//Write texture coordinates
	for (int vCount = 0; vCount <= vNum; vCount++)
	{
		for (int uCount = 0; uCount <= uNum; uCount++)
		{
			float uTex = float(uCount/(uNum*1.0));
			float vTex = float(vCount/(vNum*1.0));
			ofs << "t " << uTex << " " << vTex << endl;
		}
	}

	//Write connectivity triangles
	for (int vCount = 0; vCount < vNum; vCount++)
	{
		for (int uCount = 0; uCount < uNum; uCount++)
		{
			ofs << "f " <<  vCount*(uNum+1)+ uCount + 1 << " " << vCount*(uNum+1)+(uCount+1) + 1 << " " << (vCount+1)*(uNum+1)+(uCount+1) + 1 << endl;
			ofs << "f " <<  vCount*(uNum+1)+ uCount + 1 << " " << (vCount+1)*(uNum+1)+(uCount+1) + 1 << " " << (vCount+1)*(uNum+1)+ uCount + 1 << endl;
		}
	}
	delete this->evalParams;

	bool surfacePoints = false;
	if (surfacePoints)
	{
		this->evalParams = new EvaluationParams();
		this->initializedEvalParams = true;

		int uNum = 128;
		int vNum = 128;
		// uMin, uMax, vMin, vMax
		this->evalParams->uNum	= uNum;
		this->evalParams->vNum	= vNum;
		this->evalParams->uMin	= 0.0;
		this->evalParams->uMax	= 1.0;
		this->evalParams->vMin	= 0.0;
		this->evalParams->vMax	= 1.0;

		this->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
		glParam->readBack = true;
		this->SurfaceEvaluation(glParam, false, false);

		// write uNum vNum
		ofs << "u " << uNum << endl;
		ofs << "v " << vNum << endl;
		// write surface vertices
		for (int vCount = 0; vCount <= vNum; vCount++)
		{
			for (int uCount = 0; uCount <= uNum; uCount++)
			{
				Float4 vertex = Float4(this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+0],this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+1],this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+2],this->evalParams->evaluatedPoints[vCount*(uNum+1)*4+uCount*4+3]);
				vertex[3] *= 5;
				ofs << "p " << vertex[0]/vertex[3] << " " << vertex[1]/vertex[3] << " " << vertex[2]/vertex[3] << endl;
			}
		}
		delete this->evalParams;
	}
	
	ofs.close();
	glParam->readBack = storedReadBack;
	this->initializedEvalParams = false;
}

void NURBS::ReadNURBS(char* filename)
{
	ifstream in(filename, ios::in);
	string firstword;
	string comment;
	Float4 cPoint;
	Float3 color;
	int tempInt;

	this->trimmed	= false;
	this->uBaseNum	= 10;
	this->vBaseNum	= 10;

	this->kdColor = Float3(0.760784, 0.843137, 0.196078);
	this->ksColor = Float3(0.9, 0.9, 0.9);
	this->ka = 0.1;
	this->shininess = 50;

	if (!in.good())
	{
		cerr<<"Unable to open file \""<<filename<<"\""<<endl;
		abort();
	}
	
	// Read Header data
	// Color
	// Order of surface
	// Number of control points
	while (in.good())
	{
		in >> firstword;
		if (!in.good()) 
			break;
		if (firstword[0] == 'c' && firstword[1] == 'd') 
		{
			in >> color[0] >> color[1] >> color[2];
			this->kdColor = Float3(color[0],color[1],color[2]);
		}
		else if (firstword[0] == 'c' && firstword[1] == 's') 
		{
			in >> color[0] >> color[1] >> color[2];
			this->ksColor = Float3(color[0],color[1],color[2]);
		}
		else if (firstword[0] == 'c' && firstword[1] == 'a') 
		{
			in >> color[0];
			this->ka = color[0];
		}
		else if (firstword[0] == 'c' && firstword[1] == 'h') 
		{
			in >> color[0];
			this->shininess = color[0];
		}
		else if (firstword[0] == 'o' && firstword[1] == 'u') 
		{
			in >> tempInt;
			this->uOrder = tempInt;
		}
		else if (firstword[0] == 'o' && firstword[1] == 'v') 
		{
			in >> tempInt;
			this->vOrder = tempInt;
		}
		else if (firstword[0] == 'p' && firstword[1] == 'u') 
		{
			in >> tempInt;
			this->uPoints = tempInt;
		}
		else if (firstword[0] == 'p' && firstword[1] == 'v') 
		{
			in >> tempInt;
			this->vPoints = tempInt;
		}
		else if (firstword[0] == 't' && firstword[1] == 'r') 
		{
			in >> tempInt;
			if (tempInt == 1)
				this->trimmed = true;
		}
		else if (firstword[0] == 'h')
			break;
	}
	
	// Allocate Memory for Data
	int vPoints=this->vPoints;
	int uPoints=this->uPoints;

	float* cntlpoints = new float[this->uPoints*this->vPoints*4];
	float* tempuKnots = new float[this->uPoints+this->uOrder];
	float* tempvKnots = new float[this->vPoints+this->vOrder];

	// Read Knot data
	// Read control points
	int index = 0;
	while (in.good())
	{
		in >> firstword;
		if (!in.good()) break;
		if (firstword[0] == 'k' && firstword[1] == 'u') 
		{
			for (int u = 0; (u < this->uPoints + this->uOrder && in.good()); u++)
				in >> tempuKnots[u];
		}
		else if (firstword[0] == 'k' && firstword[1] == 'v') 
		{
			for (int v = 0; (v < this->vPoints + this->vOrder && in.good()); v++)
				in >> tempvKnots[v];
		}
		else if (firstword[0] == 'c' && firstword[1] == 'p') 
		{
			in >> cPoint[0] >> cPoint[1] >> cPoint[2] >> cPoint[3];
			cntlpoints[index*4+0] = cPoint[0];
			cntlpoints[index*4+1] = cPoint[1];
			cntlpoints[index*4+2] = cPoint[2];
			cntlpoints[index*4+3] = cPoint[3];
			index++;
		}
	}

	float* uKnots = new float[this->uPoints+this->uOrder];
	float* vKnots = new float[this->vPoints+this->vOrder];
	for (int u = 0; u < this->uPoints + this->uOrder; u++)
		uKnots[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[this->uPoints + this->uOrder-1]-tempuKnots[0]);
	for (int v = 0; v < this->vPoints + this->vOrder; v++)
		vKnots[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[this->vPoints + this->vOrder-1]-tempvKnots[0]);

	delete[] tempuKnots;
	delete[] tempvKnots;

	this->cntlPoints	= cntlpoints;
	this->uKnotVector	= uKnots;
	this->vKnotVector	= vKnots;
}

void NURBS::SaveNURBS(char* filename)
{
	ofstream out(filename, ios::out);
	if (!out.good())
	{
		cerr << "Unable to open file " << filename << endl;
		abort();
	}

	// Write Header data
	// Color
	// Order of surface
	// Number of control points
	out << "cd " << this->kdColor[0] << " " << this->kdColor[1] << " " << this->kdColor[2] << endl;
	out << "cs " << this->ksColor[0] << " " << this->ksColor[1] << " " << this->ksColor[2] << endl;
	out << "ca " << this->ka << endl;
	out << "ch " << this->shininess << endl;
	out << "ou " << this->uOrder << endl;
	out << "ov " << this->vOrder << endl;
	out << "pu " << this->uPoints << endl;
	out << "pv " << this->vPoints << endl;
	out << "tr " << int(this->trimmed) << endl;
	out << "eb " << this->uBaseNum << " " << this->vBaseNum << endl;
	out << "h" << endl;
	out << "ku ";
	for (int u = 0; (u < this->uPoints + this->uOrder); u++)
		out << this->uKnotVector[u] << " ";
	out << endl;
	out << "kv ";
	for (int v = 0; (v < this->vPoints + this->vOrder); v++)
		out << this->vKnotVector[v] << " ";
	out << endl;
	for (int k = 0; k < this->uPoints*this->vPoints; k++)
	{
		out << "cp " << this->cntlPoints[k * 4 + 0] << " ";
		out << this->cntlPoints[k * 4 + 1] << " ";
		out << this->cntlPoints[k * 4 + 2] << " ";
		out << this->cntlPoints[k * 4 + 3] << endl;
	}
}

void NURBS::GPUSecondDerivativesNonUniformEvaluation(int texWidth, int texHeight, int numQuadraturePoints, float interval[2], GLuint sourceTex, GLuint xDerivs, GLuint yDerivs, GLParameters *glParam)
{
	int numPatchesWidth = texWidth/numQuadraturePoints;
	int numPatchesHeight = texHeight/numQuadraturePoints;

	float patchInterval[2];
	patchInterval[0] = interval[0]*(texWidth  - 1)/(numPatchesWidth);
	patchInterval[1] = interval[1]*(texHeight - 1)/(numPatchesHeight);

	if (numQuadraturePoints == 2)
	{
		patchInterval[0] *= sqrt(3.0)/3.0;
		patchInterval[1] *= sqrt(3.0)/3.0;
	}
	else if (numQuadraturePoints == 3)
	{
		patchInterval[0] *= sqrt(3.0/10.0);
		patchInterval[1] *= sqrt(3.0/10.0);
	}

	glPushAttrib(GL_ALL_ATTRIB_BITS);

	InitTexture(&xDerivs, RGBA_FLOAT_FORMAT, numPatchesWidth, numPatchesHeight, GL_RGBA);
	InitTexture(&yDerivs, RGBA_FLOAT_FORMAT, numPatchesWidth, numPatchesHeight, GL_RGBA);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

	if (numQuadraturePoints == 2)
	{
		cgGLBindProgram(glParam->secondDerv2PointProgram);

		cgGLSetTextureParameter(glParam->secondDerv2PointTextureParam, sourceTex);
		cgGLEnableTextureParameter(glParam->secondDerv2PointTextureParam);

		cgGLSetParameter2fv(glParam->secondDerv2PointUVIntervalParam,interval);
	}
	else if (numQuadraturePoints == 3)
	{
		cgGLBindProgram(glParam->secondDerv3PointProgram);

		cgGLSetTextureParameter(glParam->secondDerv3PointTextureParam, sourceTex);
		cgGLEnableTextureParameter(glParam->secondDerv3PointTextureParam);

		cgGLSetParameter2fv(glParam->secondDerv3PointUVIntervalParam,interval);
	}

	glViewport(0, 0, numPatchesWidth, numPatchesHeight);

	const GLenum buffers[] = {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};
	glDrawBuffers(2, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, yDerivs, 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, xDerivs, 0);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, sourceTex);

	DrawQuad(numPatchesWidth, numPatchesHeight);

//	glFlush();
	if (numQuadraturePoints == 2)
		cgGLDisableTextureParameter(glParam->secondDerv2PointTextureParam);
	else if (numQuadraturePoints == 3)
		cgGLDisableTextureParameter(glParam->secondDerv3PointTextureParam);

#ifdef DUMP
	GLfloat* retVal=new GLfloat[4*(numPatchesWidth)*(numPatchesHeight)];
	glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
	glReadPixels(0, 0, numPatchesWidth, numPatchesHeight, GL_RGBA, GL_FLOAT, retVal);
	WriteToFile(retVal, numPatchesWidth, numPatchesHeight, 0);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, numPatchesWidth, numPatchesHeight, GL_RGBA, GL_FLOAT, retVal);
	WriteToFile(retVal, numPatchesWidth, numPatchesHeight, 0);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
#endif

	glDrawBuffers(1, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
	glPopAttrib();
}

void NURBS::GPUSecondDerivativesEvaluation(int texWidth, int texHeight, float interval[2], GLuint sourceTex, GLuint xDerivs, GLuint yDerivs, GLParameters *glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	InitTexture(&xDerivs, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA);
	InitTexture(&yDerivs, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
		
	cgGLBindProgram(glParam->secondDervProgram);
	cgGLSetTextureParameter(glParam->secondDervTextureParam, sourceTex);
	cgGLEnableTextureParameter(glParam->secondDervTextureParam);
	cgGLSetParameter2fv(glParam->secondDervUVIntervalParam,interval);

	glViewport(0, 0, texWidth, texHeight);

	float size[2];
	size[0] = texWidth-1;
	size[1] = texHeight-1;
	cgGLSetParameter2fv(glParam->secondDervDimensionParam, size);

	const GLenum buffers[] = {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};
	glDrawBuffers(2, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, yDerivs, 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, xDerivs, 0);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, sourceTex);

	DrawQuad(texWidth, texHeight);

//	glFlush();
	cgGLDisableTextureParameter(glParam->secondDervTextureParam);

#ifdef DUMP
	GLfloat* retVal=new GLfloat[4*(texWidth)*(texHeight)];
	glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RGBA,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight,0);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RGBA,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight,0);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
#endif

	glDrawBuffers(1, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
	glPopAttrib();
}

void NURBS::GPUExactNormalEvaluation(GLParameters* glParam)
{
	// Set up GPU computations
	StartGPUComputation(glParam);

	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;
	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;
	int uLen			= int ((uNum+1)/4.0) + 1;
	int vLen			= int ((vNum+1)/4.0) + 1;
	int uBasisFuncVal	= (ku+1)%2;
	int vBasisFuncVal	= (kv+1)%2+3;

	GLuint tempTex[4];
	glGenTextures(4, tempTex);
	
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(tempTex+3, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);

	GLuint uEvaluationTextures[5];
	uEvaluationTextures[0] = this->evalParams->ctrlPointsTexture;
	uEvaluationTextures[1] = this->evalParams->uCtrlPointIndexTexture;
	uEvaluationTextures[2] = this->evalParams->vCtrlPointIndexTexture;
	uEvaluationTextures[3] = this->evalParams->uBasisFuncDervTexture;
	uEvaluationTextures[4] = this->evalParams->vBasisFuncTexture;
	
	GLuint uDervTex = GPUBasisCtrlPointMultiplication(uEvaluationTextures, tempTex[0], tempTex[1], glParam);
	GLuint uSwapTex;
	if (uDervTex == tempTex[0])
		uSwapTex = tempTex[1];
	else
		uSwapTex = tempTex[0];

	cgGLBindProgram(glParam->rationalProgram);
	cgGLSetTextureParameter( glParam->rationalDervTextureParam, uDervTex );
	cgGLEnableTextureParameter(	glParam->rationalDervTextureParam );
	cgGLSetTextureParameter( glParam->rationalSurfaceTextureParam, this->evalParams->evaluatedSurfTexture );
	cgGLEnableTextureParameter(	glParam->rationalSurfaceTextureParam );
		
	glViewport(0, 0, uNum+1 , vNum+1);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, uDervTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfTexture);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, uSwapTex, 0);
	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*1];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_GREEN,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1));
#endif
	cgGLDisableTextureParameter( glParam->rationalSurfaceTextureParam);
	cgGLDisableTextureParameter( glParam->rationalDervTextureParam);

	uDervTex = uSwapTex;

	float interval[2] = {0,0};
	interval[0] = (this->evalParams->uMax - this->evalParams->uMin)/this->evalParams->uNum;
	interval[1] = (this->evalParams->vMax - this->evalParams->vMin)/this->evalParams->vNum;

	if (((glParam->intersection || glParam->drawBBox || glParam->collision || glParam->closestPoint || glParam->hausdorff) && glParam->expandBoundingBox) || glParam->curvatureEvaluation)
		this->GPUSecondDerivativesEvaluation(this->evalParams->uNum+1, this->evalParams->vNum+1, interval, uDervTex, this->evalParams->secondDervTextures[0], this->evalParams->secondDervTextures[1], glParam);
	else if (glParam->computingMoment && this->evalParams->computeMomentError)
		this->GPUSecondDerivativesNonUniformEvaluation(this->evalParams->uNum+1, this->evalParams->vNum+1, this->evalParams->numQuadraturePoints, interval, uDervTex, this->evalParams->secondDervTextures[0], this->evalParams->secondDervTextures[1], glParam);

	GLuint vEvaluationTextures[5];
	vEvaluationTextures[0] = this->evalParams->ctrlPointsTexture;
	vEvaluationTextures[1] = this->evalParams->uCtrlPointIndexTexture;
	vEvaluationTextures[2] = this->evalParams->vCtrlPointIndexTexture;
	vEvaluationTextures[3] = this->evalParams->uBasisFuncTexture;
	vEvaluationTextures[4] = this->evalParams->vBasisFuncDervTexture;
	
	GLuint vDervTex = GPUBasisCtrlPointMultiplication(vEvaluationTextures, tempTex[2], tempTex[3], glParam);
	GLuint vSwapTex;
	if (vDervTex == tempTex[2])
		vSwapTex = tempTex[3];
	else
		vSwapTex = tempTex[2];

	cgGLBindProgram(glParam->rationalProgram);
	cgGLSetTextureParameter( glParam->rationalDervTextureParam, vDervTex );
	cgGLEnableTextureParameter(	glParam->rationalDervTextureParam );
	cgGLSetTextureParameter( glParam->rationalSurfaceTextureParam, this->evalParams->evaluatedSurfTexture );
	cgGLEnableTextureParameter(	glParam->rationalSurfaceTextureParam );
		
	glViewport(0, 0, uNum+1 , vNum+1);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, vDervTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfTexture);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, vSwapTex, 0);
	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

#ifdef DUMP
	retVal=new GLfloat[(uNum+1)*(vNum+1)*1];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1));
#endif

	cgGLDisableTextureParameter( glParam->rationalSurfaceTextureParam);
	cgGLDisableTextureParameter( glParam->rationalDervTextureParam);

	vDervTex = vSwapTex;

	if (((glParam->intersection || glParam->drawBBox || glParam->collision || glParam->closestPoint) && glParam->expandBoundingBox || glParam->hausdorff) || glParam->curvatureEvaluation)
		this->GPUSecondDerivativesEvaluation(this->evalParams->uNum+1, this->evalParams->vNum+1, interval, vDervTex, this->evalParams->secondDervTextures[2], this->evalParams->secondDervTextures[3], glParam);
	else if (glParam->computingMoment && this->evalParams->computeMomentError)
		this->GPUSecondDerivativesNonUniformEvaluation(this->evalParams->uNum+1, this->evalParams->vNum+1, this->evalParams->numQuadraturePoints, interval, vDervTex, this->evalParams->secondDervTextures[2], this->evalParams->secondDervTextures[3], glParam);

	//Bind the normal program
	cgGLBindProgram(glParam->exactNormalProgram);

	cgGLSetTextureParameter(glParam->exactNormaluDervTextureParam, uDervTex);
	cgGLSetTextureParameter(glParam->exactNormalvDervTextureParam, vDervTex);
	cgGLEnableTextureParameter(glParam->exactNormaluDervTextureParam);
	cgGLEnableTextureParameter(glParam->exactNormalvDervTextureParam);

	glViewport(0, 0, uNum+1, vNum+1);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfNormTexture, 0);

	// Bind the src texture
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, vDervTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, uDervTex);

	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

#ifdef DUMP
	float* retVal=new float[(uNum+1)*(vNum+1)];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1));
#endif

	if (glParam->readBack && this->evalParams->evaluatedNormals != NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,uNum+1,vNum+1,GL_RGBA,GL_FLOAT,this->evalParams->evaluatedNormals);
//		WriteToFile(this->evalParams->evaluatedNormals,uNum+1,vNum+1,0);
	}

	cgGLDisableTextureParameter(glParam->exactNormaluDervTextureParam);
	cgGLDisableTextureParameter(glParam->exactNormalvDervTextureParam);

	glDeleteTextures(4, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

void  NURBS::GPUNormalEvaluation(GLParameters* glParam)
{
	// Set up GPU computations
	StartGPUComputation(glParam);

	int uNum	= this->evalParams->uNum;
	int vNum	= this->evalParams->vNum;

	float temp[2];
	temp[0] = uNum;
	temp[1] = vNum;

	//Bind the normal program
	cgGLBindProgram(glParam->normalProgram);

	cgGLSetTextureParameter(glParam->normalTextureParam, this->evalParams->evaluatedSurfTexture);
	cgGLSetParameter2fv(glParam->normalDimParam,temp);
	cgGLEnableTextureParameter(glParam->normalTextureParam);

	glViewport(0, 0, uNum+1, vNum+1);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfNormTexture, 0);

	// Bind the src texture
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfTexture);

	DrawQuad(uNum+1, vNum+1);
//	glFlush();

	cgGLDisableTextureParameter(glParam->normalTextureParam);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

#ifdef PACKED
GLuint NURBS::GPUBasisFunctionEvaluation(int k, int n, int evalNum, float paramMin, float paramMax, float* knotVector, GLParameters* glParam, GLuint* tempDervTex, float* ctrlData, float* paramValues, bool nonUniformEvaluation)
{
	GLuint tempTex[3];
	glGenTextures(3, tempTex);

	int evalLen			= int ((evalNum+1)/4.0) + 1;
	float* knotData		= new float[4*evalLen*(2*k+1)];
	float* basisData	= new float[4*evalLen*(k+1)];

	int prevStartIndex = 0;
	for (int index=0; index<evalLen*4; index++)
	{
		int channelIndex = int (index/(evalLen*1.0));
		int runningIndex = index - channelIndex*evalLen;
		if(index <= evalNum)
		{
			float pVal = 0;
			if (nonUniformEvaluation)
				pVal = paramValues[index];
			else
				pVal = float(index*(paramMax - paramMin)/(1.0*evalNum) + paramMin);

			int startIndex = prevStartIndex;
			while ( pVal >= knotVector[startIndex] && startIndex < n+k )
				startIndex++;
			prevStartIndex = startIndex;

			//	if (index != uNum)
			if (pVal < 1 - DELTA)
				startIndex-=k;
			else
				startIndex-=2*k;

			knotData[4*(2*k+1)*runningIndex+channelIndex]=pVal;
			for (int i = 1; i < (2*k+1); i++)
				knotData[4*(2*k+1)*runningIndex+4*i+channelIndex] = knotVector[startIndex+i-1];

			ctrlData[index]=startIndex;

			for (int i = 0; i < k+1; i++)
				basisData[4*runningIndex*(k+1) + 4*i + channelIndex] = 0;

			basisData[4*runningIndex*(k+1) +  4*(k-1)  + channelIndex] = 1;
		}
		else
		{
			for (int i = 0; i < (2*k+1); i++)
				knotData[4*(2*k+1)*runningIndex+4*i+channelIndex]=0;
			for (int i = 0; i < k+1; i++)
				basisData[4*(k+1)*runningIndex + 4*i + channelIndex]=0;
		}
	}

	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, k+1, evalLen, GL_RGBA, basisData);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, k+1, evalLen, GL_RGBA);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, 2*k+1, evalLen, GL_RGBA, knotData);

	cgGLBindProgram(glParam->evaluateBasisProgram);
	for(int pass=2; pass<=k ;pass++)
	{
		//We ping-pong between src and dst
		GLshort iSrcTexIndex = pass%2,
			iDstTexIndex = (pass+1)%2;

		cgGLSetTextureParameter(glParam->knotDataTextureParam	, tempTex[2]);
		cgGLSetTextureParameter(glParam->basisTextureParam		, tempTex[iSrcTexIndex]);
		cgGLEnableTextureParameter(glParam->knotDataTextureParam);
		cgGLEnableTextureParameter(glParam->basisTextureParam);
		cgGLSetParameter1f(glParam->passParam,pass);

		glViewport(0, 0, k+1, evalLen);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[iDstTexIndex], 0);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[iSrcTexIndex]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		DrawQuad((k-pass)/(0.5*(k+1)), 0, pass/(0.5*(k+1)), 2, k-pass, 0, pass, evalLen);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[(k+1)*(evalLen*1)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,k,evalLen,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,k,evalLen);
#endif
	}
	cgGLDisableTextureParameter(glParam->knotDataTextureParam);
	cgGLDisableTextureParameter(glParam->basisTextureParam); 

	if (glParam->exactNormals || glParam->expandBoundingBox)
	{
		InitTexture(tempDervTex, RGBA_FLOAT_FORMAT, k+1, evalLen, GL_RGBA);

		cgGLBindProgram(glParam->evaluateBasisDervProgram);
		//We ping-pong between src and dst
		GLshort iSrcTexIndex = (k)%2;

		cgGLSetTextureParameter(glParam->knotDataDervTextureParam	, tempTex[2]);
		cgGLEnableTextureParameter(glParam->knotDataDervTextureParam);
		cgGLSetTextureParameter(glParam->basisDervTextureParam		, tempTex[iSrcTexIndex]);
		cgGLEnableTextureParameter(glParam->basisDervTextureParam);
		cgGLSetParameter1f(glParam->passDervParam,k);

		glViewport(0, 0, k+1, evalLen);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, *tempDervTex, 0);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[iSrcTexIndex]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		DrawQuad(0, 0, k/(0.5*(k+1)), 2, 0, 0, k, evalLen);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[(ku+1)*(uLen*1)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,ku,uLen,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,ku,uLen);
#endif DUMP

		cgGLDisableTextureParameter(glParam->knotDataDervTextureParam);
		cgGLDisableTextureParameter(glParam->basisDervTextureParam); 
	}
	delete[] knotData;
	delete[] basisData;

	glDeleteTextures(1, &(tempTex[2]));
	glDeleteTextures(1, &(tempTex[k%2]));
	return tempTex[(k+1)%2];
}
#else
GLuint NURBS::GPUBasisFunctionEvaluation(int k, int n, int evalNum, float paramMin, float paramMax, float* knotVector, GLParameters* glParam, GLuint* tempDervTex, float* ctrlData, float* paramValues, bool nonUniformEvaluation)
{
	GLuint tempTex[3];
	glGenTextures(3, tempTex);

	int evalLen			= int (evalNum+1);
	float* knotData		= new float[evalLen*(2*k+1)];
	float* basisData	= new float[evalLen*(k+1)];

	int prevStartIndex = 0;
	for (int index = 0; index < evalLen; index++)
	{
		int runningIndex = index;
		float pVal = 0;
		if (nonUniformEvaluation)
			pVal = paramValues[index];
		else
			pVal = float(index*(paramMax - paramMin)/(1.0*evalNum) + paramMin);

		int startIndex = prevStartIndex;
		while ( pVal >= knotVector[startIndex] && startIndex < n+k )
			startIndex++;
		prevStartIndex = startIndex;

		//	if (index != uNum)
		if (pVal < 1 - DELTA)
			startIndex-=k;
		else
			startIndex-=2*k;

		knotData[(2*k+1)*runningIndex]=pVal;
		for (int i = 1; i < (2*k+1); i++)
			knotData[(2*k+1)*runningIndex+i] = knotVector[startIndex+i-1];

		ctrlData[index]=startIndex;

		for (int i = 0; i < k+1; i++)
			basisData[runningIndex*(k+1) + i] = 0;

		basisData[runningIndex*(k+1) + k - 1] = 1;
	}

	InitTexture(tempTex+0, LUMINANCE_FLOAT_FORMAT, k+1, evalLen, GL_LUMINANCE, basisData);
	InitTexture(tempTex+1, LUMINANCE_FLOAT_FORMAT, k+1, evalLen, GL_LUMINANCE);
	InitTexture(tempTex+2, LUMINANCE_FLOAT_FORMAT, 2*k+1, evalLen, GL_LUMINANCE, knotData);

	cgGLBindProgram(glParam->evaluateBasisProgram);
	for(int pass=2; pass<=k ;pass++)
	{
		//We ping-pong between src and dst
		GLshort iSrcTexIndex = pass%2,
			iDstTexIndex = (pass+1)%2;

		cgGLSetTextureParameter(glParam->knotDataTextureParam	, tempTex[2]);
		cgGLSetTextureParameter(glParam->basisTextureParam		, tempTex[iSrcTexIndex]);
		cgGLEnableTextureParameter(glParam->knotDataTextureParam);
		cgGLEnableTextureParameter(glParam->basisTextureParam);
		cgGLSetParameter1f(glParam->passParam,pass);

		glViewport(0, 0, k+1, evalLen);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[iDstTexIndex], 0);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[iSrcTexIndex]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		DrawQuad((k-pass)/(0.5*(k+1)), 0, pass/(0.5*(k+1)), 2, k-pass, 0, pass, evalLen);
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[(k+1)*(evalLen*1)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,k,evalLen,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,k,evalLen);
#endif
	}
	cgGLDisableTextureParameter(glParam->knotDataTextureParam);
	cgGLDisableTextureParameter(glParam->basisTextureParam); 

	if (glParam->exactNormals)
	{
		InitTexture(tempDervTex, LUMINANCE_FLOAT_FORMAT, k+1, evalLen, GL_LUMINANCE);

		cgGLBindProgram(glParam->evaluateBasisDervProgram);
		//We ping-pong between src and dst
		GLshort iSrcTexIndex = (k)%2;

		cgGLSetTextureParameter(glParam->knotDataDervTextureParam	, tempTex[2]);
		cgGLEnableTextureParameter(glParam->knotDataDervTextureParam);
		cgGLSetTextureParameter(glParam->basisDervTextureParam		, tempTex[iSrcTexIndex]);
		cgGLEnableTextureParameter(glParam->basisDervTextureParam);
		cgGLSetParameter1f(glParam->passDervParam,k);

		glViewport(0, 0, k+1, evalLen);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, *tempDervTex, 0);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[iSrcTexIndex]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		DrawQuad(0, 0, k/(0.5*(k+1)), 2, 0, 0, k, evalLen);
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[k*(evalNum+1)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,k,evalNum+1,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,k,evalNum+1);
#endif DUMP

		cgGLDisableTextureParameter(glParam->knotDataDervTextureParam);
		cgGLDisableTextureParameter(glParam->basisDervTextureParam); 
	}
	delete[] knotData;
	delete[] basisData;

	glDeleteTextures(1, &(tempTex[2]));
	glDeleteTextures(1, &(tempTex[k%2]));
	return tempTex[(k+1)%2];
}
#endif 

#ifdef NOPINGPONG
GLuint NURBS::GPUBasisCtrlPointMultiplication(GLuint* inputTextures, GLuint outTexture, GLuint swapTexture, GLParameters* glParam)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

	InitTexture(&(outTexture)	, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(&(swapTexture)	, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	cgGLSetTextureParameter(glParam->surfaceMultCtrlPtsTextureParam, inputTextures[0]);
	cgGLSetTextureParameter(glParam->surfaceMultuCtrlIndexTextureParam, inputTextures[1]);
	cgGLSetTextureParameter(glParam->surfaceMultvCtrlIndexTextureParam, inputTextures[2]);
	cgGLSetTextureParameter(glParam->surfaceMultuBasisFuncTextureParam, inputTextures[3]);
	cgGLSetTextureParameter(glParam->surfaceMultvBasisFuncTextureParam, inputTextures[4]);

	cgGLEnableTextureParameter(glParam->surfaceMultCtrlPtsTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultuCtrlIndexTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultvCtrlIndexTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultuBasisFuncTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultvBasisFuncTextureParam);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE4_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[4]);
	glActiveTextureARB(GL_TEXTURE3_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[3]);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[0]);

	cgGLBindProgram(glParam->surfaceMultProgram);

	int	dstTexture = outTexture;

	cgGLSetParameter1f(glParam->surfaceMultkuParam,ku);
	cgGLSetParameter1f(glParam->surfaceMultkvParam,kv);


	glViewport(0, 0, uNum+1 , vNum+1);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTexture, 0);
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
	glFlush();

	DrawQuad(uNum+1, vNum+1);
	glFinish();
#ifdef DUMP
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*1];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_BLUE,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1));
#endif

	cgGLDisableTextureParameter(glParam->surfaceMultCtrlPtsTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultuCtrlIndexTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultvCtrlIndexTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultuBasisFuncTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultvBasisFuncTextureParam);

	// To differentiate between outTexture and swapTexture
	return dstTexture;
}
#else
GLuint NURBS::GPUBasisCtrlPointMultiplication(GLuint* inputTextures, GLuint outTexture, GLuint swapTexture, GLParameters* glParam)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;
	int uLen			= int ((uNum+1)/4.0) + 1;
	int vLen			= int ((vNum+1)/4.0) + 1;

	InitTexture(&(outTexture)	, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(&(swapTexture)	, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);

	cgGLSetTextureParameter(glParam->surfaceMultCtrlPtsTextureParam, inputTextures[0]);
	cgGLSetTextureParameter(glParam->surfaceMultuCtrlIndexTextureParam, inputTextures[1]);
	cgGLSetTextureParameter(glParam->surfaceMultvCtrlIndexTextureParam, inputTextures[2]);
	cgGLSetTextureParameter(glParam->surfaceMultuBasisFuncTextureParam, inputTextures[3]);
	cgGLSetTextureParameter(glParam->surfaceMultvBasisFuncTextureParam, inputTextures[4]);

	cgGLEnableTextureParameter(glParam->surfaceMultCtrlPtsTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultuCtrlIndexTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultvCtrlIndexTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultuBasisFuncTextureParam);
	cgGLEnableTextureParameter(glParam->surfaceMultvBasisFuncTextureParam);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE4_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[4]);
	glActiveTextureARB(GL_TEXTURE3_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[3]);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTextures[0]);

	cgGLBindProgram(glParam->surfaceMultProgram);

	cgGLSetParameter1f(glParam->surfaceMultuLenParam,uLen);
	cgGLSetParameter1f(glParam->surfaceMultvLenParam,vLen);

	int passCount=0;
	int srcTexture, dstTexture;
	for(int vPass=0; vPass<kv ;vPass++)
	{
		for(int uPass=0; uPass<ku ;uPass++)
		{
			//We ping-pong between src and dst
			if ((passCount)%2==0)
			{
				srcTexture = outTexture;
				dstTexture = swapTexture;
			}
			else
			{
				srcTexture = swapTexture;
				dstTexture = outTexture;
			}

			cgGLSetTextureParameter(glParam->surfaceMultSurfPtSrcTextureParam, srcTexture);
			cgGLEnableTextureParameter(glParam->surfaceMultSurfPtSrcTextureParam);

			cgGLSetParameter1f(glParam->surfaceMultuPassParam,uPass);
			cgGLSetParameter1f(glParam->surfaceMultvPassParam,vPass);

			glViewport(0, 0, uNum+1 , vNum+1);

			// Bind the src textures
			glActiveTextureARB(GL_TEXTURE5_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTexture);

			// Specify the dst texture
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTexture, 0);

			DrawQuad(uNum+1, vNum+1);
//			glFlush();
//			glFinish();
			passCount++;
#ifdef DUMP
			GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*1];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,uNum+1,vNum+1,GL_BLUE,GL_FLOAT,retVal);
			WriteToFile(retVal,(uNum+1),(vNum+1));
#endif
		}
	}

	cgGLDisableTextureParameter(glParam->surfaceMultCtrlPtsTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultuCtrlIndexTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultvCtrlIndexTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultuBasisFuncTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultvBasisFuncTextureParam);
	cgGLDisableTextureParameter(glParam->surfaceMultSurfPtSrcTextureParam);

	// To differentiate between outTexture and swapTexture
	return dstTexture;
}
#endif
#ifndef CPUEVAL
int NURBS::SurfaceEvaluation(GLParameters* glParam, bool rendering, bool nonUniformEvaluation)
{
	// Set up GPU computations
	StartGPUComputation(glParam);

	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

	float* uCtrlData	= new float[(uNum+1)];
	float* vCtrlData	= new float[(vNum+1)];

	this->evalParams->uBasisFuncTexture = GPUBasisFunctionEvaluation(ku, nu, uNum, this->evalParams->uMin, this->evalParams->uMax, this->uKnotVector, glParam, &(this->evalParams->uBasisFuncDervTexture), uCtrlData, this->evalParams->uEvalParamPoints, nonUniformEvaluation);

	this->evalParams->vBasisFuncTexture = GPUBasisFunctionEvaluation(kv, nv, vNum, this->evalParams->vMin, this->evalParams->vMax, this->vKnotVector, glParam, &(this->evalParams->vBasisFuncDervTexture), vCtrlData, this->evalParams->vEvalParamPoints, nonUniformEvaluation);

	InitTexture(&(this->evalParams->ctrlPointsTexture), RGBA_FLOAT_FORMAT, nu, nv, GL_RGBA, this->cntlPoints);
	InitTexture(&(this->evalParams->uCtrlPointIndexTexture), LUMINANCE_FLOAT_FORMAT, uNum+1, 1, GL_LUMINANCE, uCtrlData);
	InitTexture(&(this->evalParams->vCtrlPointIndexTexture), LUMINANCE_FLOAT_FORMAT, vNum+1, 1, GL_LUMINANCE, vCtrlData);

	GLuint evaluationTextures[5];
	evaluationTextures[0] = this->evalParams->ctrlPointsTexture;
	evaluationTextures[1] = this->evalParams->uCtrlPointIndexTexture;
	evaluationTextures[2] = this->evalParams->vCtrlPointIndexTexture;
	evaluationTextures[3] = this->evalParams->uBasisFuncTexture;
	evaluationTextures[4] = this->evalParams->vBasisFuncTexture;
	
	this->evalParams->evaluatedSurfTexture = GPUBasisCtrlPointMultiplication(evaluationTextures, this->evalParams->evaluatedSurfTexture, this->evalParams->evaluatedSurfNormTexture, glParam);

	delete[] uCtrlData;
	delete[] vCtrlData;

	if (glParam->readBack && this->evalParams->evaluatedPoints!=NULL && this->identityTransformation)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, uNum+1, vNum+1, GL_RGBA, GL_FLOAT, this->evalParams->evaluatedPoints);
	}
	else if (rendering)
	{
#ifndef VBO
		if (this->evaluatedPointsDisplay != NULL)
		{
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,uNum+1,vNum+1,GL_RGBA,GL_FLOAT,this->evaluatedPointsDisplay);
		}
#else
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, this->ptVBO);
		glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, (uNum+1)*(vNum+1)*4*sizeof(float), NULL, GL_DYNAMIC_DRAW);
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, uNum+1, vNum+1, GL_RGBA, GL_FLOAT, 0);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
#endif
	}

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (rendering)
	{
		if (glParam->exactNormals)
			this->GPUExactNormalEvaluation(glParam);
		else
			this->GPUNormalEvaluation(glParam);

		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
#ifndef VBO
		if(this->evaluatedNormalsDisplay != NULL)
		{
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,uNum+1,vNum+1,GL_RGB,GL_FLOAT,this->evaluatedNormalsDisplay);
		}
#else
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, this->normVBO);
		glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, (uNum+1)*(vNum+1)*3*sizeof(float), NULL, GL_DYNAMIC_DRAW);
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, uNum+1, vNum+1, GL_RGB, GL_FLOAT, 0);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);
#endif
		glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
		if (glParam->drawBBox) 
		{
			if (this->trimmed)
				this->GenerateTrimData(uNum, vNum, glParam);

			// Set up GPU computations
			StartGPUComputation(glParam);

			GLuint tempTex[2];
			glGenTextures(2, tempTex);
			InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, uNum, vNum, GL_RGBA);
			InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, uNum, vNum, GL_RGBA);
			if (this->trimmed)
				GPUComputeBoundingBoxTrimmed(uNum, vNum, this->evalParams->evaluatedSurfTexture, tempTex[0], tempTex[1], this->evalParams->secondDervTextures, this->evalParams->maxExpansionFactor, this->evalParams->trimTexture, glParam, this->minPointsDisplay, this->maxPointsDisplay);
			else
				GPUComputeBoundingBox(uNum, vNum, this->evalParams->evaluatedSurfTexture, tempTex[0], tempTex[1], this->evalParams->secondDervTextures, this->evalParams->maxExpansionFactor, glParam, this->minPointsDisplay, this->maxPointsDisplay);
			glDeleteTextures(2, tempTex);

			// Clean up GPU computations
			EndGPUCompuation(glParam);
		}
	}

	return 0;
}
#else
int NURBS::SurfaceEvaluation(GLParameters* glParam, bool rendering, bool nonUniformEvaluation)
{
	int uNum = this->evalParams->uNum;
	int vNum = this->evalParams->vNum;
	float* vertexData = this->evaluatedPointsDisplay;
	float* normalData = this->evaluatedNormalsDisplay;
	for (int vCount=0; vCount<=vNum; vCount++)
	{
		for (int uCount=0; uCount<=uNum; uCount++)
		{
			float u = float(uCount/(uNum*1.0));
			float v = float(vCount/(vNum*1.0));
			Float4 point = this->Evaluate4(u,v);
			vertexData[vCount*(uNum+1)*4+4*uCount+0]=point[0];
			vertexData[vCount*(uNum+1)*4+4*uCount+1]=point[1];
			vertexData[vCount*(uNum+1)*4+4*uCount+2]=point[2];
			vertexData[vCount*(uNum+1)*4+4*uCount+3]=point[3];

			//Evaluate Normals
			if (rendering)
			{
				Float3 normal = this->EvaluateNormal(u,v);
				VectorNormalize(normal);
				normalData[vCount*(uNum+1)*3+3*uCount+0]=normal[0];
				normalData[vCount*(uNum+1)*3+3*uCount+1]=normal[1];
				normalData[vCount*(uNum+1)*3+3*uCount+2]=normal[2];
			}
		}
	}

#ifndef CPURENDER
	//Init floating-point textures
	glGenTextures(1, glParam->curveTexture);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, glParam->curveTexture[0]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, 0, GL_RGBA, GL_FLOAT, vertexData);				

	DrawQuad(0, 0, 2, 2, 0, 0, uNum+1,vNum+1);
//	glFlush();
//	glFinish();
	glDeleteTextures(1, glParam->curveTexture);
#endif
	return 0;
}
#endif

void NURBS::GPUComputeVariableExpansionFactor(int texWidth, int texHeight, GLuint expansionFactorTex, GLuint secondDerivs[4], GLParameters* glParam, float* expansionFactorData)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	float magFactor = 0;
	GLuint maxTextures[3];
	glGenTextures(3, maxTextures);

	InitTexture(maxTextures+0, LUMINANCE_FLOAT_FORMAT, texWidth, texHeight, GL_LUMINANCE);
	InitTexture(maxTextures+1, LUMINANCE_FLOAT_FORMAT, texWidth, texHeight, GL_LUMINANCE);
	InitTexture(maxTextures+2, LUMINANCE_FLOAT_FORMAT, texWidth, texHeight, GL_LUMINANCE);
	InitTexture(&(expansionFactorTex), LUMINANCE_FLOAT_FORMAT, texWidth, texHeight, GL_LUMINANCE);

	int destTex = 0;
	{
		cgGLBindProgram(glParam->findMaxProgram);
		cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[0]);
		cgGLEnableTextureParameter(glParam->findMaxTextureParam);

		float factor = float(1.0/(8.0*texWidth*texWidth));
		cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[0]);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
		glViewport(0, 0, texWidth, texHeight);

		DrawQuad(texWidth,texHeight);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[(iSize)*(iSize)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,texWidth,texHeight);
#endif
		cgGLDisableTextureParameter(glParam->findMaxTextureParam);
	}

	{
		destTex++;
		cgGLBindProgram(glParam->findMaxProgram);
		cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[1]);
		cgGLEnableTextureParameter(glParam->findMaxTextureParam);

		float factor = float(1.0/(4.0*texWidth*texHeight));
		cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[1]);

		glViewport(0, 0, texWidth, texHeight);
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);

		DrawQuad(texWidth,texHeight);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,texWidth,texHeight);
#endif
		cgGLDisableTextureParameter(glParam->findMaxTextureParam);
	}

	{
		destTex++;
		cgGLBindProgram(glParam->findMaxProgram);
		cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[3]);
		cgGLEnableTextureParameter(glParam->findMaxTextureParam);

		float factor = float(1.0/(8.0*texHeight*texHeight));
		cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[3]);
		glViewport(0, 0, texWidth, texHeight);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
		DrawQuad(texWidth,texHeight);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,texWidth,texHeight);
#endif
		cgGLDisableTextureParameter(glParam->findMaxTextureParam);
	}

	cgGLBindProgram(glParam->mergeExpansionFactorsProgram);
	cgGLSetTextureParameter(glParam->mergeFactorTex1TextureParam, maxTextures[0]);
	cgGLEnableTextureParameter(glParam->mergeFactorTex1TextureParam);
	cgGLSetTextureParameter(glParam->mergeFactorTex2TextureParam, maxTextures[1]);
	cgGLEnableTextureParameter(glParam->mergeFactorTex2TextureParam);
	cgGLSetTextureParameter(glParam->mergeFactorTex3TextureParam, maxTextures[2]);
	cgGLEnableTextureParameter(glParam->mergeFactorTex3TextureParam);

	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[0]);
	glViewport(0, 0, texWidth, texHeight);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, expansionFactorTex , 0);
	DrawQuad(texWidth,texHeight);
//	glFlush();
//	glFinish();
	
#ifdef DUMP
	GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight);
#endif
	glDeleteTextures(3, maxTextures);

	if (glParam->readBack && expansionFactorData!=NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, texWidth, texHeight, GL_RED, GL_FLOAT, expansionFactorData);
//		WriteToFile(expansionFactorData,"expFactor.txt",texWidth,texHeight);
	}
	glPopAttrib();
}

void NURBS::GPUComputeBoundingBox(int texWidth, int texHeight, GLuint sourceTex, GLuint minTex, GLuint maxTex, GLuint secondDerivs[4], float &magFactor, GLParameters* glParam, float* minData, float* maxData)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	GLuint expansionFactorTex;
	magFactor = 0;
	if(glParam->expandBoundingBox)
	{
		GLuint maxTextures[4];
		if (glParam->variableExpansionFactor)
			glGenTextures(4, maxTextures);
		else
			glGenTextures(2, maxTextures);

		int iSize;
		if (texWidth > texHeight)
			iSize = GetNextPower2(texWidth);
		else
			iSize = GetNextPower2(texHeight);

		InitTexture(maxTextures+0, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);
		InitTexture(maxTextures+1, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);

		if (glParam->variableExpansionFactor)
		{
			InitTexture(maxTextures+2, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);
			InitTexture(maxTextures+3, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);
		}

		int destTex = 0;
		{
			cgGLBindProgram(glParam->findMaxProgram);
			cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[0]);
			cgGLEnableTextureParameter(glParam->findMaxTextureParam);

			float factor = float(1.0/(8.0*texWidth*texWidth));
			cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[0]);

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
			glViewport(0, 0, iSize, iSize);
			glClearColor(0,0,0,1);
			glClear(GL_COLOR_BUFFER_BIT);
			glViewport(0, 0, texWidth, texHeight);

			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[(iSize)*(iSize)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			cgGLDisableTextureParameter(glParam->findMaxTextureParam);
			if (!glParam->variableExpansionFactor)
				magFactor += FindMaxValue(iSize, maxTextures, glParam);
		}

		{
			cgGLBindProgram(glParam->findMaxProgram);
			cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[1]);
			cgGLEnableTextureParameter(glParam->findMaxTextureParam);

			float factor = float(1.0/(4.0*texWidth*texHeight));
			cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[1]);
			glViewport(0, 0, texWidth, texHeight);
			if (glParam->variableExpansionFactor)
				destTex++;

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			cgGLDisableTextureParameter(glParam->findMaxTextureParam);
			if (!glParam->variableExpansionFactor)
				magFactor += FindMaxValue(iSize,maxTextures,glParam);
		}

		{
			cgGLBindProgram(glParam->findMaxProgram);
			cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[3]);
			cgGLEnableTextureParameter(glParam->findMaxTextureParam);
	
			float factor = float(1.0/(8.0*texHeight*texHeight));
			cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[3]);
			glViewport(0, 0, texWidth, texHeight);
			if (glParam->variableExpansionFactor)
				destTex++;

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			cgGLDisableTextureParameter(glParam->findMaxTextureParam);
			if (!glParam->variableExpansionFactor)
				magFactor += FindMaxValue(iSize,maxTextures,glParam);
		}

		if (!glParam->variableExpansionFactor)
			glDeleteTextures(2,maxTextures);
		else
		{
			cgGLBindProgram(glParam->mergeExpansionFactorsProgram);
			cgGLSetTextureParameter(glParam->mergeFactorTex1TextureParam, maxTextures[0]);
			cgGLEnableTextureParameter(glParam->mergeFactorTex1TextureParam);
			cgGLSetTextureParameter(glParam->mergeFactorTex2TextureParam, maxTextures[1]);
			cgGLEnableTextureParameter(glParam->mergeFactorTex2TextureParam);
			cgGLSetTextureParameter(glParam->mergeFactorTex3TextureParam, maxTextures[2]);
			cgGLEnableTextureParameter(glParam->mergeFactorTex3TextureParam);
	
			glActiveTextureARB(GL_TEXTURE2_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[2]);
			glActiveTextureARB(GL_TEXTURE1_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[1]);
			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[0]);
			glViewport(0, 0, texWidth, texHeight);

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[3], 0);
			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();
			
#ifdef DUMP
			GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			glDeleteTextures(3, maxTextures);
			expansionFactorTex = maxTextures[3];
		}
	}

	//Transformations
	if (!this->identityTransformation)
	{
		GLuint tempTex;
		glGenTextures(1, &tempTex);
		InitTexture(&tempTex, RGBA_FLOAT_FORMAT, texWidth+1, texHeight+1, GL_RGBA);
		this->GPUTransformEvaluatedPoints(tempTex, glParam);
		sourceTex = tempTex;
	}

	//Bind the program
 	cgGLBindProgram(glParam->boundingBoxProgram);

	cgGLSetTextureParameter(glParam->boundingBoxSurfaceTextureParam, sourceTex);
	cgGLEnableTextureParameter(glParam->boundingBoxSurfaceTextureParam);
	if (glParam->variableExpansionFactor)
	{
		cgGLSetTextureParameter(glParam->boundingBoxFactorTextureParam, expansionFactorTex);
		cgGLEnableTextureParameter(glParam->boundingBoxFactorTextureParam);
	}
	else
		cgGLSetParameter1f(glParam->boundingBoxFactorParam,magFactor);

	if (glParam->intersection || glParam->closestPoint || glParam->hausdorff)
	{
		float identityMatrix[16];
		MakeIdentityMatrix(identityMatrix);
		cgGLSetMatrixParameterfc(glParam->boundingBoxModelViewParam,identityMatrix);
	}
	else
		cgGLSetMatrixParameterfc(glParam->boundingBoxModelViewParam, glParam->modelViewMatrix);

	glViewport(0, 0, texWidth, texHeight);

	// Specify the dst texture
	const GLenum buffers[] = {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};
	glDrawBuffers(2, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTex, 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, minTex, 0);

	// Bind the src texture
	if (glParam->variableExpansionFactor)
	{
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, expansionFactorTex);
	}
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, sourceTex);

	DrawQuad(texWidth, texHeight);
//	glFlush();
//	glFinish();

	cgGLDisableTextureParameter(glParam->boundingBoxSurfaceTextureParam);
	if (glParam->variableExpansionFactor)
	{
		cgGLDisableTextureParameter(glParam->boundingBoxFactorTextureParam);
		glDeleteTextures(1, &expansionFactorTex);
	}

#ifdef DUMP
	GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
	glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight);
#endif
	if (minData!=NULL && maxData!=NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
		glReadPixels(0, 0, texWidth, texHeight, GL_RGBA, GL_FLOAT, maxData);
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, texWidth, texHeight, GL_RGBA, GL_FLOAT, minData);
//		WriteToFile(maxData,"MaxData.txt",4,texWidth*texHeight);
//		WriteToFile(minData,"MinData.txt",4,texWidth*texHeight);
	}

	if (!this->identityTransformation)
		glDeleteTextures(1, &sourceTex);

	glDrawBuffers(1, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
	glPopAttrib();

}

void NURBS::GPUComputeBoundingBoxTrimmed(int texWidth, int texHeight, GLuint sourceTex, GLuint minTex, GLuint maxTex, GLuint secondDerivs[4], float &magFactor, GLuint trimTex, GLParameters* glParam, float* minData, float* maxData)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	GLuint expansionFactorTex;
	magFactor = 0;
	if(glParam->expandBoundingBox)
	{
		GLuint maxTextures[4];
		if (glParam->variableExpansionFactor)
			glGenTextures(4, maxTextures);
		else
			glGenTextures(2, maxTextures);

		int iSize;
		if (texWidth > texHeight)
			iSize = GetNextPower2(texWidth);
		else
			iSize = GetNextPower2(texHeight);

		InitTexture(maxTextures+0, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);
		InitTexture(maxTextures+1, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);

		if (glParam->variableExpansionFactor)
		{
			InitTexture(maxTextures+2, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);
			InitTexture(maxTextures+3, RGBA_FLOAT_FORMAT, iSize, iSize, GL_RGBA);
		}

		int destTex = 0;
		{
			cgGLBindProgram(glParam->findMaxProgram);
			cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[0]);
			cgGLEnableTextureParameter(glParam->findMaxTextureParam);

			float factor = float(1.0/(8.0*texWidth*texWidth));
			cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[0]);

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
			glViewport(0, 0, iSize, iSize);
			glClearColor(0,0,0,1);
			glClear(GL_COLOR_BUFFER_BIT);
			glViewport(0, 0, texWidth, texHeight);

			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[(iSize)*(iSize)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			cgGLDisableTextureParameter(glParam->findMaxTextureParam);
			if (!glParam->variableExpansionFactor)
				magFactor += FindMaxValue(iSize, maxTextures, glParam);
		}

		{
			cgGLBindProgram(glParam->findMaxProgram);
			cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[1]);
			cgGLEnableTextureParameter(glParam->findMaxTextureParam);

			float factor = float(1.0/(4.0*texWidth*texHeight));
			cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[1]);
			glViewport(0, 0, texWidth, texHeight);
			if (glParam->variableExpansionFactor)
				destTex++;

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			cgGLDisableTextureParameter(glParam->findMaxTextureParam);
			if (!glParam->variableExpansionFactor)
				magFactor += FindMaxValue(iSize,maxTextures,glParam);
		}

		{
			cgGLBindProgram(glParam->findMaxProgram);
			cgGLSetTextureParameter(glParam->findMaxTextureParam, secondDerivs[3]);
			cgGLEnableTextureParameter(glParam->findMaxTextureParam);
	
			float factor = float(1.0/(8.0*texHeight*texHeight));
			cgGLSetParameter1f(glParam->findMaxFactorParam, factor);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,secondDerivs[3]);
			glViewport(0, 0, texWidth, texHeight);
			if (glParam->variableExpansionFactor)
				destTex++;

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[destTex], 0);
			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			cgGLDisableTextureParameter(glParam->findMaxTextureParam);
			if (!glParam->variableExpansionFactor)
				magFactor += FindMaxValue(iSize,maxTextures,glParam);
		}

		if (!glParam->variableExpansionFactor)
			glDeleteTextures(2,maxTextures);
		else
		{
			cgGLBindProgram(glParam->mergeExpansionFactorsProgram);
			cgGLSetTextureParameter(glParam->mergeFactorTex1TextureParam, maxTextures[0]);
			cgGLEnableTextureParameter(glParam->mergeFactorTex1TextureParam);
			cgGLSetTextureParameter(glParam->mergeFactorTex2TextureParam, maxTextures[1]);
			cgGLEnableTextureParameter(glParam->mergeFactorTex2TextureParam);
			cgGLSetTextureParameter(glParam->mergeFactorTex3TextureParam, maxTextures[2]);
			cgGLEnableTextureParameter(glParam->mergeFactorTex3TextureParam);
	
			glActiveTextureARB(GL_TEXTURE2_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[2]);
			glActiveTextureARB(GL_TEXTURE1_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[1]);
			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTextures[0]);
			glViewport(0, 0, texWidth, texHeight);

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTextures[3], 0);
			DrawQuad(texWidth,texHeight);
//			glFlush();
//			glFinish();
			
#ifdef DUMP
			GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
			WriteToFile(retVal,texWidth,texHeight);
#endif
			glDeleteTextures(3, maxTextures);
			expansionFactorTex = maxTextures[3];
		}
	}

	//Transformations
	if (!this->identityTransformation)
	{
		GLuint tempTex;
		glGenTextures(1, &tempTex);
		InitTexture(&tempTex, RGBA_FLOAT_FORMAT, texWidth+1, texHeight+1, GL_RGBA);
		this->GPUTransformEvaluatedPoints(tempTex, glParam);
		sourceTex = tempTex;
	}

	//Bind the program
	cgGLBindProgram(glParam->boundingBoxTrimmedProgram);

	cgGLSetTextureParameter(glParam->boundingBoxTrimmedSurfaceTextureParam, sourceTex);
	cgGLEnableTextureParameter(glParam->boundingBoxTrimmedSurfaceTextureParam);

	cgGLSetTextureParameter(glParam->boundingBoxTrimmedTrimTextureParam, trimTex);
	cgGLEnableTextureParameter(glParam->boundingBoxTrimmedTrimTextureParam);
	if (glParam->variableExpansionFactor)
	{
		cgGLSetTextureParameter(glParam->boundingBoxTrimmedFactorTextureParam, expansionFactorTex);
		cgGLEnableTextureParameter(glParam->boundingBoxTrimmedFactorTextureParam);
	}
	else
		cgGLSetParameter1f(glParam->boundingBoxTrimmedFactorParam,magFactor);

	if (glParam->intersection || glParam->closestPoint || glParam->hausdorff)
	{
		float identityMatrix[16];
		MakeIdentityMatrix(identityMatrix);
		cgGLSetMatrixParameterfc(glParam->boundingBoxTrimmedModelViewParam, identityMatrix);
	}
	else
		cgGLSetMatrixParameterfc(glParam->boundingBoxTrimmedModelViewParam, glParam->modelViewMatrix);

	glViewport(0, 0, texWidth, texHeight);

	// Specify the dst texture
	const GLenum buffers[] = {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};
	glDrawBuffers(2, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, maxTex, 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, minTex, 0);

	// Bind the src texture
	if (glParam->variableExpansionFactor)
	{
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, expansionFactorTex);
	}
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, sourceTex);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, trimTex);

	DrawQuad(texWidth, texHeight);
//	glFlush();
//	glFinish();

	cgGLDisableTextureParameter(glParam->boundingBoxTrimmedSurfaceTextureParam);
	cgGLDisableTextureParameter(glParam->boundingBoxTrimmedTrimTextureParam);
	if (glParam->variableExpansionFactor)
	{
		cgGLDisableTextureParameter(glParam->boundingBoxTrimmedFactorTextureParam);
		glDeleteTextures(1, &expansionFactorTex);
	}

#ifdef DUMP
	GLfloat* retVal=new GLfloat[(texWidth)*(texHeight)];
	glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,texWidth,texHeight);
#endif
	if (minData!=NULL && maxData!=NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
		glReadPixels(0, 0, texWidth, texHeight, GL_RGBA, GL_FLOAT, maxData);
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, texWidth, texHeight, GL_RGBA, GL_FLOAT, minData);
//		glReadPixels(0, 0, texWidth*2, texHeight, GL_RED, GL_FLOAT, minData);
//		WriteToFile(minData, texWidth*2, texHeight);
//		WriteToFile(maxData,"MaxData.txt",4,texWidth*texHeight);
//		WriteToFile(minData,"MinData.txt",4,texWidth*texHeight);
	}

	if (!this->identityTransformation)
		glDeleteTextures(1, &sourceTex);

	glDrawBuffers(1, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);
	glPopAttrib();

}

void NURBS::GPUBuildBoundingBoxHierarchy(GLuint bBoxTex, int texSize, GLParameters* glParam, float isBBoxMax, float* maxVal = NULL)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	int viewportx = 0,viewporty = 0;
	float origin[4] = {0.5,0.5,0,0};
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	cgGLSetTextureParameter(glParam->bBoxHierarchyTextureParam, bBoxTex);
	cgGLSetParameter1fv(glParam->bBoxHierarchyIsMaxParam,&isBBoxMax);
	cgGLBindProgram(glParam->bBoxHierarchyProgram);

	for(int currentSize = texSize; currentSize > 1; currentSize = currentSize/2) 
	{
		cgGLSetParameter4fv(glParam->bBoxHierarchyOriginParam, origin);
		cgGLEnableTextureParameter(glParam->bBoxHierarchyTextureParam);

		// Find the location of block to render
		viewportx += currentSize;
		viewporty = 0;
		glViewport(viewportx, viewporty ,currentSize/2, currentSize/2);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, bBoxTex, 0);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, bBoxTex);

		DrawQuad(currentSize/2, currentSize/2);
		glFlush();
		glFinish();
#ifdef DUMP
		GLfloat* retVal = new GLfloat[6*texSize*texSize];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, 2*texSize, texSize, GL_RED, GL_FLOAT, retVal);
		WriteToFileInt(retVal, 2*texSize, texSize);
#endif
		cgGLDisableTextureParameter(glParam->bBoxHierarchyTextureParam);
		origin[0] += currentSize;
	}
	if (maxVal != NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(viewportx, 0, 1, 1, GL_RGB, GL_FLOAT, maxVal);
	}
	glPopAttrib();
}

void NURBS::GPUBuildBoundingBoxHierarchyTrimmedMax(GLuint bBoxTex, int texSize, GLParameters* glParam, float* maxVal = NULL)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	// Clear the rest of the texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, bBoxTex, 0);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	glViewport(texSize, 0 ,texSize*2, texSize);
	glColor4f(0,0,0,0);
	DrawQuad(texSize,texSize);
#ifdef DUMP
	GLfloat* retVal = new GLfloat[2*texSize*texSize];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, 2*texSize, texSize, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, 2*texSize, texSize);
#endif

	int viewportx = 0,viewporty = 0;
	float origin[4] = {0.5,0.5,0,0};
	cgGLSetTextureParameter(glParam->bBoxHierarchyTrimmedMaxTextureParam, bBoxTex);
	cgGLBindProgram(glParam->bBoxHierarchyTrimmedMaxProgram);
	for(int currentSize = texSize; currentSize > 1; currentSize = currentSize/2) 
	{
		cgGLSetParameter4fv(glParam->bBoxHierarchyTrimmedMaxOriginParam, origin);
		cgGLEnableTextureParameter(glParam->bBoxHierarchyTrimmedMaxTextureParam);

		// Find the location of block to render
		viewportx += currentSize;
		viewporty = 0;
		glViewport(viewportx, viewporty ,currentSize/2, currentSize/2);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, bBoxTex, 0);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, bBoxTex);

		DrawQuad(currentSize/2, currentSize/2);
		glFlush();
		glFinish();
#ifdef DUMP
		GLfloat* retVal = new GLfloat[2*texSize*texSize];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, 2*texSize, texSize, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal, 2*texSize, texSize);
#endif
		cgGLDisableTextureParameter(glParam->bBoxHierarchyTrimmedMaxTextureParam);
		origin[0] += currentSize;
	}
	if (maxVal != NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(viewportx, 0, 1, 1, GL_RGB, GL_FLOAT, maxVal);
	}
	glPopAttrib();
}

void NURBS::GPUBuildBoundingBoxHierarchyTrimmedMin(GLuint bBoxTex, int texSize, GLParameters* glParam, float* maxVal = NULL)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	// Clear the rest of the texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, bBoxTex, 0);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	glViewport(texSize, 0 ,texSize*2, texSize);
	glColor4f(0,0,0,0);
	DrawQuad(texSize,texSize);
#ifdef DUMP
	GLfloat* retVal = new GLfloat[2*texSize*texSize];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, 2*texSize, texSize, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, 2*texSize, texSize);
#endif

	int viewportx = 0,viewporty = 0;
	float origin[4] = {0.5,0.5,0,0};
	cgGLSetTextureParameter(glParam->bBoxHierarchyTrimmedMinTextureParam, bBoxTex);
	cgGLBindProgram(glParam->bBoxHierarchyTrimmedMinProgram);
	for(int currentSize = texSize; currentSize > 1; currentSize = currentSize/2) 
	{
		cgGLSetParameter4fv(glParam->bBoxHierarchyTrimmedMinOriginParam, origin);
		cgGLEnableTextureParameter(glParam->bBoxHierarchyTrimmedMaxTextureParam);

		// Find the location of block to render
		viewportx += currentSize;
		viewporty = 0;
		glViewport(viewportx, viewporty ,currentSize/2, currentSize/2);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, bBoxTex, 0);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, bBoxTex);

		DrawQuad(currentSize/2, currentSize/2);
		glFlush();
		glFinish();
#ifdef DUMP
		GLfloat* retVal = new GLfloat[2*texSize*texSize];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, 2*texSize, texSize, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal, 2*texSize, texSize);
#endif
		cgGLDisableTextureParameter(glParam->bBoxHierarchyTrimmedMaxTextureParam);
		origin[0] += currentSize;
	}
	if (maxVal != NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(viewportx, 0, 1, 1, GL_RGB, GL_FLOAT, maxVal);
	}
	glPopAttrib();
}

int NURBS::SurfaceBoundingBoxEvaluation(GLParameters* glParam)
{
	bool timing = false;
	Timer localTimer;
	float normalTime, bBoxTime, hierarchyTime;
	if (timing)
		localTimer.Start();

//	To create non axis aligned bounding-boxes
//	MakeIdentityMatrix(glParam->modelViewMatrix);

	// Create the texture data
	int uNum	= this->evalParams->uNum;
	int vNum	= this->evalParams->vNum;

	int	texWidth	= uNum;
	int	texHeight	= vNum;

	if (glParam->intersection || glParam->collision || glParam->selfIntersection || glParam->closestPoint || glParam->hausdorff)
		texWidth *= 2;

	InitTexture(this->evalParams->bBoxTextures+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA);
	InitTexture(this->evalParams->bBoxTextures+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA);

	if(this->evalParams->maxExpansionFactor==0 && glParam->expandBoundingBox)
		GPUExactNormalEvaluation(glParam);

	if (timing)
	{
		localTimer.Stop();
		normalTime = localTimer.GetElapsedTimeInMicroSec();
		localTimer.Start();
	}

	// Set up GPU computations
	StartGPUComputation(glParam);

	if (glParam->readBack && (glParam->intersection || glParam->inverseEvaluation || glParam->closestPoint))
		GPUComputeBoundingBox(uNum, vNum, this->evalParams->evaluatedSurfTexture, this->evalParams->bBoxTextures[0], this->evalParams->bBoxTextures[1], this->evalParams->secondDervTextures, this->evalParams->maxExpansionFactor, glParam, this->evalParams->minPoints, this->evalParams->maxPoints);
	else if (glParam->intersection || glParam->collision || glParam->inverseEvaluation || glParam->closestPoint || glParam->hausdorff)
		GPUComputeBoundingBox(uNum, vNum, this->evalParams->evaluatedSurfTexture, this->evalParams->bBoxTextures[0], this->evalParams->bBoxTextures[1], this->evalParams->secondDervTextures, this->evalParams->maxExpansionFactor, glParam);

	if (timing)
	{
		localTimer.Stop();
		bBoxTime = localTimer.GetElapsedTimeInMicroSec();
		localTimer.Start();
	}

	if (glParam->drawBBox || glParam->intersection || glParam->collision  || glParam->closestPoint || glParam->hausdorff)
	{
		float bBoxMinVal[3], bBoxMaxVal[3];
		GPUBuildBoundingBoxHierarchy(this->evalParams->bBoxTextures[0], vNum, glParam, 1, bBoxMinVal);
		GPUBuildBoundingBoxHierarchy(this->evalParams->bBoxTextures[1], vNum, glParam, 0, bBoxMaxVal);
		this->bBoxMax = Float3(bBoxMaxVal[0], bBoxMaxVal[1], bBoxMaxVal[2]);
		this->bBoxMin = Float3(bBoxMinVal[0], bBoxMinVal[1], bBoxMinVal[2]);
	}

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (timing)
	{
		localTimer.Stop();
		hierarchyTime = localTimer.GetElapsedTimeInMicroSec();
		cout<<"Total Time          : "<<0.001*int(normalTime + bBoxTime + hierarchyTime)<<endl;
		cout<<"Normal Time         : "<<0.001*int(normalTime)<<endl;
		cout<<"Bounding Box Time   : "<<0.001*int(bBoxTime)<<endl;
		cout<<"Hierarchy Time      : "<<0.001*int(hierarchyTime)<<endl<<endl;
	}

	return 0;
}

int NURBS::SurfaceExpansionFactorEvaluation(GLParameters* glParam)
{
	// Create the texture data
	int uNumPatches	= (this->evalParams->uNum+1)/this->evalParams->numQuadraturePoints;
	int vNumPatches	= (this->evalParams->vNum+1)/this->evalParams->numQuadraturePoints;

	int	texWidth	= uNumPatches;
	int	texHeight	= vNumPatches;

	// Set up GPU computations
	StartGPUComputation(glParam);

	GPUComputeVariableExpansionFactor(uNumPatches, vNumPatches, this->evalParams->expansionFactorTexture, this->evalParams->secondDervTextures, glParam, this->evalParams->expansionFactors);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	return 0;
}

int NURBS::SurfaceBoundingBoxTrimmedEvaluation(GLParameters* glParam)
{
	bool timing = false;
	clock_t initTime, normalTime, boundingBoxTime, hierarchyTime, initialTime, totalTime;
	if (timing)
		initialTime = clock();

//	To create non axis aligned bounding-boxes
//	MakeIdentityMatrix(glParam->modelViewMatrix);

	// Create the texture data
	int uNum	= this->evalParams->uNum;
	int vNum	= this->evalParams->vNum;

	int	texWidth	= uNum;
	int	texHeight	= vNum;

	if (glParam->intersection || glParam->collision || glParam->selfIntersection || glParam->closestPoint || glParam->hausdorff)
		texWidth *= 2;

	InitTexture(this->evalParams->bBoxTextures+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA);
	InitTexture(this->evalParams->bBoxTextures+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA);

	if (timing)
		initTime = (clock() - initialTime);

	if(this->evalParams->maxExpansionFactor==0 && glParam->expandBoundingBox)
		GPUExactNormalEvaluation(glParam);

	if (timing)
		normalTime = (clock() - initialTime);

	// Set up GPU computations
	StartGPUComputation(glParam);

	if (glParam->readBack && (glParam->intersection || glParam->inverseEvaluation || glParam->closestPoint || glParam->hausdorff))
		GPUComputeBoundingBoxTrimmed(uNum, vNum, this->evalParams->evaluatedSurfTexture, this->evalParams->bBoxTextures[0], this->evalParams->bBoxTextures[1], this->evalParams->secondDervTextures, this->evalParams->maxExpansionFactor, this->evalParams->trimTexture, glParam, this->evalParams->minPoints, this->evalParams->maxPoints);
	else if (glParam->intersection || glParam->collision || glParam->inverseEvaluation || glParam->closestPoint || glParam->hausdorff)
		GPUComputeBoundingBoxTrimmed(uNum, vNum, this->evalParams->evaluatedSurfTexture, this->evalParams->bBoxTextures[0], this->evalParams->bBoxTextures[1], this->evalParams->secondDervTextures, this->evalParams->maxExpansionFactor, this->evalParams->trimTexture, glParam);

	if (timing)
		boundingBoxTime = (clock() - initialTime);

	if (glParam->drawBBox || glParam->intersection || glParam->collision  || glParam->closestPoint || glParam->hausdorff)
	{
		float bBoxMinVal[3], bBoxMaxVal[3];
		GPUBuildBoundingBoxHierarchyTrimmedMin(this->evalParams->bBoxTextures[0], vNum, glParam, bBoxMinVal);
		GPUBuildBoundingBoxHierarchyTrimmedMax(this->evalParams->bBoxTextures[1], vNum, glParam, bBoxMaxVal);
		this->bBoxMax = Float3(bBoxMaxVal[0], bBoxMaxVal[1], bBoxMaxVal[2]);
		this->bBoxMin = Float3(bBoxMinVal[0], bBoxMinVal[1], bBoxMinVal[2]);
	}
	if (timing)
		hierarchyTime = (clock() - initialTime);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (timing)
		totalTime = (clock() - initialTime);

	return 0;
}

void NURBS::GPUComputeCurvature(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	// Set up GPU computations
	StartGPUComputation(glParam);

	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;
	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;
	int uLen			= int ((uNum+1)/4.0) + 1;
	int vLen			= int ((vNum+1)/4.0) + 1;
	int uBasisFuncVal	= (ku+1)%2;
	int vBasisFuncVal	= (kv+1)%2+3;

	GLuint tempTex[4];
	glGenTextures(4, tempTex);
	
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);
	InitTexture(tempTex+3, RGBA_FLOAT_FORMAT, uNum+1, vNum+1, GL_RGBA);

	GLuint uEvaluationTextures[5];
	uEvaluationTextures[0] = this->evalParams->ctrlPointsTexture;
	uEvaluationTextures[1] = this->evalParams->uCtrlPointIndexTexture;
	uEvaluationTextures[2] = this->evalParams->vCtrlPointIndexTexture;
	uEvaluationTextures[3] = this->evalParams->uBasisFuncDervTexture;
	uEvaluationTextures[4] = this->evalParams->vBasisFuncTexture;
	
	GLuint uDervTex = GPUBasisCtrlPointMultiplication(uEvaluationTextures, tempTex[0], tempTex[1], glParam);
	GLuint uSwapTex;
	if (uDervTex == tempTex[0])
		uSwapTex = tempTex[1];
	else
		uSwapTex = tempTex[0];

	cgGLBindProgram(glParam->rationalProgram);
	cgGLSetTextureParameter( glParam->rationalDervTextureParam, uDervTex );
	cgGLEnableTextureParameter(	glParam->rationalDervTextureParam );
	cgGLSetTextureParameter( glParam->rationalSurfaceTextureParam, this->evalParams->evaluatedSurfTexture );
	cgGLEnableTextureParameter(	glParam->rationalSurfaceTextureParam );
		
	glViewport(0, 0, uNum+1 , vNum+1);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, uDervTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfTexture);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, uSwapTex, 0);
	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[(uNum+1)*(vNum+1)*1];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1));
#endif
	cgGLDisableTextureParameter( glParam->rationalSurfaceTextureParam);
	cgGLDisableTextureParameter( glParam->rationalDervTextureParam);

	uDervTex = uSwapTex;

	float interval[2] = {0,0};
	interval[0] = (this->evalParams->uMax - this->evalParams->uMin)/this->evalParams->uNum;
	interval[1] = (this->evalParams->vMax - this->evalParams->vMin)/this->evalParams->vNum;

	this->GPUSecondDerivativesEvaluation(this->evalParams->uNum+1, this->evalParams->vNum+1, interval, uDervTex, this->evalParams->secondDervTextures[0], this->evalParams->secondDervTextures[1], glParam);

	GLuint vEvaluationTextures[5];
	vEvaluationTextures[0] = this->evalParams->ctrlPointsTexture;
	vEvaluationTextures[1] = this->evalParams->uCtrlPointIndexTexture;
	vEvaluationTextures[2] = this->evalParams->vCtrlPointIndexTexture;
	vEvaluationTextures[3] = this->evalParams->uBasisFuncTexture;
	vEvaluationTextures[4] = this->evalParams->vBasisFuncDervTexture;
	
	GLuint vDervTex = GPUBasisCtrlPointMultiplication(vEvaluationTextures, tempTex[2], tempTex[3], glParam);
	GLuint vSwapTex;
	if (vDervTex == tempTex[2])
		vSwapTex = tempTex[3];
	else
		vSwapTex = tempTex[2];

	cgGLBindProgram(glParam->rationalProgram);
	cgGLSetTextureParameter( glParam->rationalDervTextureParam, vDervTex );
	cgGLEnableTextureParameter(	glParam->rationalDervTextureParam );
	cgGLSetTextureParameter( glParam->rationalSurfaceTextureParam, this->evalParams->evaluatedSurfTexture );
	cgGLEnableTextureParameter(	glParam->rationalSurfaceTextureParam );
		
	glViewport(0, 0, uNum+1 , vNum+1);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, vDervTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfTexture);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, vSwapTex, 0);
	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

#ifdef DUMP
	retVal=new GLfloat[(uNum+1)*(vNum+1)*1];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1));
#endif

	cgGLDisableTextureParameter( glParam->rationalSurfaceTextureParam);
	cgGLDisableTextureParameter( glParam->rationalDervTextureParam);

	GLuint curvatureTex = vDervTex;
	vDervTex = vSwapTex;

	GPUSecondDerivativesEvaluation(this->evalParams->uNum+1, this->evalParams->vNum+1, interval, vDervTex, this->evalParams->secondDervTextures[2], this->evalParams->secondDervTextures[3], glParam);

	//Bind the normal program
	cgGLBindProgram(glParam->curvatureProgram);

	cgGLSetTextureParameter(glParam->curvatureuDervTextureParam, uDervTex);
	cgGLSetTextureParameter(glParam->curvaturevDervTextureParam, vDervTex);
	cgGLSetTextureParameter(glParam->curvatureuuSecondDervTextureParam, this->evalParams->secondDervTextures[0]);
	cgGLSetTextureParameter(glParam->curvatureuvSecondDervTextureParam, this->evalParams->secondDervTextures[2]);
	cgGLSetTextureParameter(glParam->curvaturevvSecondDervTextureParam, this->evalParams->secondDervTextures[3]);
	cgGLEnableTextureParameter(glParam->curvatureuDervTextureParam);
	cgGLEnableTextureParameter(glParam->curvaturevDervTextureParam);
	cgGLEnableTextureParameter(glParam->curvatureuuSecondDervTextureParam);
	cgGLEnableTextureParameter(glParam->curvatureuvSecondDervTextureParam);
	cgGLEnableTextureParameter(glParam->curvaturevvSecondDervTextureParam);

	glViewport(0, 0, uNum+1, vNum+1);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, curvatureTex, 0);

	// Bind the src texture
	glActiveTextureARB(GL_TEXTURE4_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->secondDervTextures[3]);
	glActiveTextureARB(GL_TEXTURE3_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->secondDervTextures[2]);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->secondDervTextures[0]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, vDervTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, uDervTex);

	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

#ifdef DUMP
	float* retVal=new float[(uNum+1)*(vNum+1)*4];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RGBA,GL_FLOAT,retVal);
	WriteToFile(retVal,(uNum+1),(vNum+1),0);
	WriteToFile(retVal,(uNum+1),(vNum+1),1);
#endif

	cgGLDisableTextureParameter(glParam->exactNormaluDervTextureParam);
	cgGLDisableTextureParameter(glParam->exactNormalvDervTextureParam);
	cgGLDisableTextureParameter(glParam->curvatureuuSecondDervTextureParam);
	cgGLDisableTextureParameter(glParam->curvatureuvSecondDervTextureParam);
	cgGLDisableTextureParameter(glParam->curvaturevvSecondDervTextureParam);

//	glDeleteTextures(4, tempTex);

	if (this->textured)
		glDeleteTextures(1,&(this->texture));
	this->texture = curvatureTex;
	this->textured = true;
	if (tempTex[0] != curvatureTex)
		glDeleteTextures(1,tempTex+0);
	if (tempTex[1] != curvatureTex)
		glDeleteTextures(1,tempTex+1);
	if (tempTex[2] != curvatureTex)
		glDeleteTextures(1,tempTex+2);
	if (tempTex[3] != curvatureTex)
		glDeleteTextures(1,tempTex+3);
	// Clean up GPU computations
	EndGPUCompuation(glParam);

	glPopAttrib();
}


int NURBS::SurfaceCurvatureEvaluation(int curRes, GLParameters* glParam)
{
	glParam->curvatureEvaluation = true;
	bool timing = false;
	clock_t initTime, curvatureTime, initialTime, totalTime;
	if (timing)
		initialTime = clock();

	int uNum = this->trimWidth;
	int vNum = this->trimHeight;

	bool evaluate = true;
	if(this->initializedEvalParams)
	{
		if (this->evalParams->uMin == 0.0  &&
			this->evalParams->uMax == 1.0  &&
			this->evalParams->vMin == 0.0  &&
			this->evalParams->vMax == 1.0  &&
			this->evalParams->uNum == uNum &&
			this->evalParams->vNum == vNum)
				evaluate = false;
	}
	if (evaluate)
	{
		if (this->initializedEvalParams)
			delete this->evalParams;
		this->evalParams = new EvaluationParams();
		this->evalParams->uNum = uNum;
		this->evalParams->vNum = vNum;
		this->initializedEvalParams = true;
		this->SurfaceEvaluation(glParam, false, false);
	}

	if (timing)
		initTime = (clock() - initialTime);

	this->GPUComputeCurvature(glParam);

	if (timing)
		totalTime = (clock() - initialTime);

	glParam->curvatureEvaluation = false;
	return 0;
}


void NURBS::GPUInverseEvaluate(float point[4], GLuint minTex, GLuint maxTex, GLParameters* glParam)
{
	int uNum = this->evalParams->uNum;
	int vNum = this->evalParams->vNum;

	// Set up GPU computations
	StartGPUComputation(glParam);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1.0);
	glDepthFunc(GL_LEQUAL);	

	GLuint tempTex;
	glGenTextures(1, &tempTex);

	InitTexture(&tempTex, GL_RGBA, uNum, vNum, GL_RGBA);

	//Bind the program
	cgGLBindProgram(glParam->invertProgram);

	cgGLSetTextureParameter(glParam->invertBBoxMinTextureParam, minTex);
	cgGLEnableTextureParameter(glParam->invertBBoxMinTextureParam);

	cgGLSetTextureParameter(glParam->invertBBoxMaxTextureParam, maxTex);
	cgGLEnableTextureParameter(glParam->invertBBoxMaxTextureParam);
	
	cgGLSetParameter4fv(glParam->invertPtParam,point);

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);

	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, minTex);
	glViewport(0, 0, uNum, vNum);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
//	glFlush();
	
	glBeginQueryARB(GL_SAMPLES_PASSED_ARB, glParam->occlusionQuery);

	DrawQuad(uNum, vNum);
//	glFlush();

	glEndQuery(GL_SAMPLES_PASSED_ARB);

#ifdef DUMP
		GLfloat *retVal1 = new GLfloat[uNum*vNum];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,uNum,vNum,GL_RED,GL_FLOAT,retVal1);
		WriteToFile(retVal1,uNum,vNum);
#endif

	GLuint sampleCount;
	glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &sampleCount);
	int numHits = sampleCount;

	vector<int> positions;

	FindRenderLocation(tempTex, &numHits, uNum, vNum, &positions, glParam, false);
	this->pickedBoxes = positions;

	glDeleteTextures(1, &tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

int NURBS::SurfaceInverseEvaluate(float* inputPoint, GLParameters* glParam, float* parameterValue)
{
	glParam->inverseEvaluation = true;
	int resolution = 32;
	int totalIterations = 2;
	if (this->initializedEvalParams)
		delete this->evalParams;
	this->evalParams = new EvaluationParams();
	this->initializedEvalParams = true;

	vector<float> evaluationInterval;
	evaluationInterval.push_back(0.0);
	evaluationInterval.push_back(1.0);
	evaluationInterval.push_back(0.0);
	evaluationInterval.push_back(1.0);
	for (int iterationNum = 0; iterationNum < totalIterations; iterationNum++)
	{
		vector<float> newEvaluationInterval;		// uMin, uMax, vMin, vMax
		int numIntervals = (int(evaluationInterval.size())/4);
		for( int oldNumIntervalCount = 0; oldNumIntervalCount < numIntervals; oldNumIntervalCount++)
		{
			this->evalParams->uMin = evaluationInterval[oldNumIntervalCount*4 + 0];
			this->evalParams->uMax = evaluationInterval[oldNumIntervalCount*4 + 1];
			this->evalParams->vMin = evaluationInterval[oldNumIntervalCount*4 + 2];
			this->evalParams->vMax = evaluationInterval[oldNumIntervalCount*4 + 3];

			this->evalParams->uNum = resolution;
			this->evalParams->vNum = resolution;

			this->pickedBoxes.clear();

			this->SurfaceEvaluation(glParam, false, false);

			this->SurfaceBoundingBoxEvaluation(glParam);
			this->GPUInverseEvaluate(inputPoint, this->evalParams->bBoxTextures[0], this->evalParams->bBoxTextures[1], glParam);
			glParam->inverseEvaluation = false;


			for( int box=0; box < int(this->pickedBoxes.size()); box++)
			{
				int vIndex = (int) this->pickedBoxes[box]/this->evalParams->uNum;
				int uIndex = (int) this->pickedBoxes[box]%this->evalParams->uNum;
				float uMin = float(uIndex*(this->evalParams->uMax - this->evalParams->uMin)/(1.0*this->evalParams->uNum) + this->evalParams->uMin);
				float vMin = float(vIndex*(this->evalParams->vMax - this->evalParams->vMin)/(1.0*this->evalParams->vNum) + this->evalParams->vMin);
				float uMax = float((uIndex+1)*(this->evalParams->uMax - this->evalParams->uMin)/(1.0*this->evalParams->uNum) + this->evalParams->uMin);
				float vMax = float((vIndex+1)*(this->evalParams->vMax - this->evalParams->vMin)/(1.0*this->evalParams->vNum) + this->evalParams->vMin);

				newEvaluationInterval.push_back(uMin);
				newEvaluationInterval.push_back(uMax);
				newEvaluationInterval.push_back(vMin);
				newEvaluationInterval.push_back(vMax);
			}
		}
		evaluationInterval = newEvaluationInterval;
	}
	if (evaluationInterval.size() < 4)
		return 0;

	parameterValue[0] = 0;
	parameterValue[1] = 0;

	if (glParam->enhancedAccuracy)
	{
		int numPoints = 0;
		for (int k = 0; k < int(evaluationInterval.size()/4.0); k+=4)
		{
			float uVal0 = evaluationInterval[k*4+0];
			float uVal1 = evaluationInterval[k*4+1];
			float vVal0 = evaluationInterval[k*4+2];
			float vVal1 = evaluationInterval[k*4+3];

			float point00[3], point01[3], point10[3], point11[3];
			Float3 outBaryPoint;
			this->Evaluate(uVal0, vVal0, point00);
			this->Evaluate(uVal0, vVal1, point01);
			this->Evaluate(uVal1, vVal0, point10);
			this->Evaluate(uVal1, vVal1, point11);

			bool insideTri012 = false;
			bool insideTri123 = false;
			insideTri012 = ClosestPointOnPlane(point00, point10, point01, inputPoint, &outBaryPoint);
			if (insideTri012)
			{
				parameterValue[0] += outBaryPoint[1]*(uVal1 - uVal0) + uVal0;
				parameterValue[1] += outBaryPoint[2]*(vVal1 - vVal0) + vVal0;
				numPoints++;
			}
			else
			{
				insideTri123 = ClosestPointOnPlane(point11, point01, point10, inputPoint, &outBaryPoint);
				parameterValue[0] += outBaryPoint[1]*(uVal0 - uVal1) + uVal1;
				parameterValue[1] += outBaryPoint[2]*(vVal0 - vVal1) + vVal1;
				numPoints++;
			}
		}
		parameterValue[0]/=(numPoints*1.0);
		parameterValue[1]/=(numPoints*1.0);
	}
	else
	{
		int numPoints = 0;
		for (int k = 0; k < int(evaluationInterval.size()/4.0); k+=4)
		{
			parameterValue[0] += (evaluationInterval[k*4+0] + evaluationInterval[k*4+1])/2.0;
			parameterValue[1] += (evaluationInterval[k*4+2] + evaluationInterval[k*4+3])/2.0;
			numPoints++;
		}
		parameterValue[0]/=(numPoints*1.0);
		parameterValue[1]/=(numPoints*1.0);
	}

	delete this->evalParams;
	return 1;
}

void NURBS::GPURayInverseEvaluate(float point[4], GLuint minTex, GLuint maxTex, GLParameters* glParam)
{
	int uNum = this->evalParams->uNum;
	int vNum = this->evalParams->vNum;

	// Set up GPU computations
	StartGPUComputation(glParam);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1.0);
	glDepthFunc(GL_LEQUAL);	

	GLuint tempTex;
	glGenTextures(1, &tempTex);

//	InitTexture(&tempTex, GL_RGBA, uNum, vNum, GL_RGBA);
	InitTexture(&tempTex, RGBA_FLOAT_FORMAT, uNum, vNum, GL_RGBA);

	//Bind the program
	cgGLBindProgram(glParam->invertRayProgram);

	cgGLSetTextureParameter(glParam->invertRayBBoxMinTextureParam, minTex);
	cgGLEnableTextureParameter(glParam->invertRayBBoxMinTextureParam);

	cgGLSetTextureParameter(glParam->invertRayBBoxMaxTextureParam, maxTex);
	cgGLEnableTextureParameter(glParam->invertRayBBoxMaxTextureParam);
	
	cgGLSetParameter4fv(glParam->invertRayPtParam,point);

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);

	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, minTex);
	glViewport(0, 0, uNum, vNum);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
//	glFlush();
	
	glBeginQueryARB(GL_SAMPLES_PASSED_ARB, glParam->occlusionQuery);

	DrawQuad(uNum, vNum);
//	glFlush();

	glEndQuery(GL_SAMPLES_PASSED_ARB);

#ifdef DUMP
		GLfloat *retVal1 = new GLfloat[uNum*vNum];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,uNum,vNum,GL_RED,GL_FLOAT,retVal1);
		WriteToFile(retVal1,uNum,vNum);
#endif

	GLuint sampleCount;
	glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &sampleCount);
	int numHits = sampleCount;

	vector<int> positions;
	FindRenderLocation(tempTex, &numHits, uNum, vNum, &positions, glParam);
	this->pickedBoxes = positions;

	glDeleteTextures(1, &tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

void NURBS::GPUClosestPoint(float point[4], GLuint minTex, GLuint maxTex, GLParameters* glParam, int position[2])
{
	int uNum = this->evalParams->uNum;
	int vNum = this->evalParams->vNum;

	// Set up GPU computations
	StartGPUComputation(glParam);

	GLuint tempTex[2];
	glGenTextures(2, tempTex);

	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, uNum, vNum, GL_RGBA);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, uNum, vNum, GL_RGBA);

	//Bind the program
	cgGLBindProgram(glParam->closestPointProgram);

	cgGLSetTextureParameter(glParam->closestPointBBoxMinTextureParam, minTex);
	cgGLEnableTextureParameter(glParam->closestPointBBoxMinTextureParam);

	cgGLSetTextureParameter(glParam->closestPointBBoxMaxTextureParam, maxTex);
	cgGLEnableTextureParameter(glParam->closestPointBBoxMaxTextureParam);
	
	cgGLSetParameter4fv(glParam->closestPointPtParam,point);

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, maxTex);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, minTex);
	glViewport(0, 0, uNum, vNum);

	glClearColor(0,0,0,0); 
	glClear(GL_COLOR_BUFFER_BIT);
	
	DrawQuad(uNum, vNum);
//	glFlush();
//	glFinish();

#ifdef DUMP
		GLfloat *retVal1 = new GLfloat[uNum*vNum];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,uNum,vNum,GL_RED,GL_FLOAT,retVal1);
		WriteToFile(retVal1,uNum,vNum);
#endif

	float location[2];
	float minVal = FindMinValue(uNum, tempTex, glParam, location);

	position[0] = int(location[0]);
	position[1] = int(location[1]);

	glDeleteTextures(2, tempTex);
	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

int NURBS::SurfaceClosestPoint(float* inputPoint, GLParameters* glParam, float* closestParamPoint)
{
	int resolution = 32;
	int totalIterations = 2;
	if (this->initializedEvalParams)
		delete this->evalParams;
	this->evalParams = new EvaluationParams();
	this->initializedEvalParams = true;
	vector<float> evaluationInterval;
	evaluationInterval.push_back(0.0);
	evaluationInterval.push_back(1.0);
	evaluationInterval.push_back(0.0);
	evaluationInterval.push_back(1.0);
	for (int iterationNum = 0; iterationNum < totalIterations; iterationNum++)
	{
		vector<float> newEvaluationInterval;		// uMin, uMax, vMin, vMax
		int numIntervals = (int(evaluationInterval.size())/4);
		for( int oldNumIntervalCount = 0; oldNumIntervalCount < numIntervals; oldNumIntervalCount++)
		{
			this->evalParams->uMin = evaluationInterval[oldNumIntervalCount*4 + 0];
			this->evalParams->uMax = evaluationInterval[oldNumIntervalCount*4 + 1];
			this->evalParams->vMin = evaluationInterval[oldNumIntervalCount*4 + 2];
			this->evalParams->vMax = evaluationInterval[oldNumIntervalCount*4 + 3];

			this->evalParams->uNum = resolution;
			this->evalParams->vNum = resolution;

			this->pickedBoxes.clear();

			this->SurfaceEvaluation(glParam, false, false);

			glParam->inverseEvaluation = true;
			this->SurfaceBoundingBoxEvaluation(glParam);
			int position[2];
			this->GPUClosestPoint(inputPoint, this->evalParams->bBoxTextures[0], this->evalParams->bBoxTextures[1], glParam, position);
			glParam->inverseEvaluation = false;

			int uIndex = int(position[0]/float(this->evalParams->uNum));
			int vIndex = int(position[1]/float(this->evalParams->vNum));
			float uMin = float(uIndex*(this->evalParams->uMax - this->evalParams->uMin)/(1.0*this->evalParams->uNum) + this->evalParams->uMin);
			float vMin = float(vIndex*(this->evalParams->vMax - this->evalParams->vMin)/(1.0*this->evalParams->vNum) + this->evalParams->vMin);
			float uMax = float((uIndex+1)*(this->evalParams->uMax - this->evalParams->uMin)/(1.0*this->evalParams->uNum) + this->evalParams->uMin);
			float vMax = float((vIndex+1)*(this->evalParams->vMax - this->evalParams->vMin)/(1.0*this->evalParams->vNum) + this->evalParams->vMin);

			newEvaluationInterval.push_back(uMin);
			newEvaluationInterval.push_back(uMax);
			newEvaluationInterval.push_back(vMin);
			newEvaluationInterval.push_back(vMax);
		}
		evaluationInterval = newEvaluationInterval;
	}
	if (evaluationInterval.size() < 4)
		return 0;

	closestParamPoint[0] = (evaluationInterval[0] + evaluationInterval[1])/2.0;
	closestParamPoint[1] = (evaluationInterval[2] + evaluationInterval[3])/2.0;

	delete this->evalParams;
	return 1;
}

void NURBS::GPUTransformEvaluatedPoints(GLuint transformedTex, GLParameters* glParam)
{
	int uNum = this->evalParams->uNum;
	int vNum = this->evalParams->vNum;

	//Bind the program
	cgGLBindProgram(glParam->transformSurfaceProgram);

	cgGLSetTextureParameter(glParam->transformSurfaceTextureParam, this->evalParams->evaluatedSurfTexture);
	cgGLEnableTextureParameter(glParam->transformSurfaceTextureParam);

	cgGLSetMatrixParameterfc(glParam->transformSurfaceMatrixParam, this->transformationMatrix);

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, transformedTex, 0);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfTexture);
	glViewport(0, 0, uNum+1, vNum+1);

	DrawQuad(uNum+1, vNum+1);
//	glFlush();
//	glFinish();

	if (glParam->readBack && this->evalParams->evaluatedPoints!=NULL)
	{
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, uNum+1, vNum+1, GL_RGBA, GL_FLOAT, this->evalParams->evaluatedPoints);
	}

#ifdef DUMP
	GLfloat *retVal1 = new GLfloat[(uNum+1)*(vNum+1)];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,retVal1);
	WriteToFile(retVal1,uNum+1,vNum+1);
#endif
}

vector<list<int>> NURBS::FitSihouette(float* points, int nPoints, int intRes)
{
	float tolerance = 1.0/(intRes);
	float mergeTolerance = 100;
	vector<list<int>> pLineList;
	int* points2DMap = new int[intRes*intRes];
	int* pointsMergeMap = new int[nPoints];
	for (int j = 0; j < intRes; j++)
		for (int i = 0; i < intRes; i++)
			points2DMap[j*intRes + i] = -1;
	for (int k = 0; k < nPoints; k++)
	{
		int xPos = int(points[2*k + 0]);
		int yPos = int(points[2*k + 1]);
		if (points2DMap[yPos*intRes + xPos] == -1)
			points2DMap[yPos*intRes + xPos] = k;
		else
		{
			int oldK = points2DMap[yPos*intRes + xPos];
			float floatXPos	= points[2*k + 0];
			float floatYPos	= points[2*k + 1];
			float oldXPos	= points[2*oldK + 0];
			float oldYPos	= points[2*oldK + 1];
			float oldDistFromCenter = (oldXPos - xPos)*(oldXPos - xPos) + (oldYPos - yPos)*(oldYPos - yPos);
			float distFromCenter	= (floatXPos - xPos)*(floatXPos - xPos) + (floatYPos - yPos)*(floatYPos - yPos);
			if (distFromCenter < oldDistFromCenter)
				points2DMap[yPos*intRes + xPos] = k;
		}
		pointsMergeMap[k] = -1;
	}

//	WriteToFileInt(points2DMap, intRes, intRes);
	int unMergedPoints = nPoints;
	int currentPLineIndex = 0;
	while (unMergedPoints > 0)
	{
		// Find first point in the list not merged.
		int currentIndex = 0;
		while (pointsMergeMap[currentIndex] != -1)
			currentIndex++;

		// Create a new polyline with this point as seed
		list<int> pLine;

		// Traverse along one direction till edge or no more points to be added
		// Find closest neighbour other than the previous point.
		// Previous point is not defined at start
		do
		{
			pLine.push_back(currentIndex);
			unMergedPoints--;
			pointsMergeMap[currentIndex] = currentPLineIndex;
			int xPos = int(points[2*currentIndex + 0]);
			int yPos = int(points[2*currentIndex + 1]);
			points2DMap[yPos*intRes + xPos] = -1;

			int nextIndex = FindClosestNeighbour(currentIndex, points2DMap, points, intRes, nPoints);
			currentIndex = nextIndex;
		} while (currentIndex != -1);
		
		// Traverse along the other direction till edge or no more points to be added
		currentIndex = pLine.front();

		int nextIndex = FindClosestNeighbour(currentIndex, points2DMap, points, intRes, nPoints);
		currentIndex = nextIndex;
		while (currentIndex != -1)
		{
			pLine.push_front(currentIndex);
			unMergedPoints--;
			pointsMergeMap[currentIndex] = currentPLineIndex;
			int xPos = int(points[2*currentIndex + 0]);
			int yPos = int(points[2*currentIndex + 1]);
			points2DMap[yPos*intRes + xPos] = -1;

			int nextIndex = FindClosestNeighbour(currentIndex, points2DMap, points, intRes, nPoints);
			currentIndex = nextIndex;
		} 

		// Add the polyline to the polyline list
		if (pLine.size() > 1)
		{
			pLineList.push_back(pLine);
			currentPLineIndex++;
		}
	}
	return pLineList;
}

void NURBS::GPUSilhouetteEvaluation(Float3 viewVector, GLParameters* glParam)
{
	// Set up GPU computations
	StartGPUComputation(glParam);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1.0);
	glDepthFunc(GL_LEQUAL);	

	int uNum = this->evalParams->uNum;
	int vNum = this->evalParams->vNum;

	GLuint tempTex; 
	glGenTextures(1, &tempTex);

	InitTexture(&tempTex, GL_RGBA, uNum, vNum, GL_RGBA);

	//Bind the program
	cgGLBindProgram(glParam->silhouetteProgram);

	cgGLSetTextureParameter(glParam->silhouetteNormalTextureParam, this->evalParams->evaluatedSurfNormTexture);
	cgGLEnableTextureParameter(glParam->silhouetteNormalTextureParam);

//	Float3 viewVector = Float3(glParam->modelViewMatrix[2], glParam->modelViewMatrix[6], glParam->modelViewMatrix[10]);
//	Float3 viewVector = Float3(glParam->modelViewMatrix[1], glParam->modelViewMatrix[5], glParam->modelViewMatrix[9]);

	VectorNormalize(viewVector);
	float data[3];
	data[0] = viewVector[0];
	data[1] = viewVector[1];
	data[2] = viewVector[2];
	float delta = pow(5.0/(uNum*1.0),this->silhouetteLevel+1);

	cgGLSetParameter3fv(glParam->silhouetteVParam, data );
	cgGLSetParameter1f(glParam->silhouetteDeltaParam, delta );

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, this->evalParams->evaluatedSurfNormTexture);
	glViewport(0, 0, uNum, vNum);

	DrawQuad(uNum, vNum);
//	glFlush();
//	glFinish();

#ifdef DUMP
	GLfloat *retVal1 = new GLfloat[(uNum+1)*(vNum+1)];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,uNum+1,vNum+1,GL_RED,GL_FLOAT,retVal1);
	WriteToFile(retVal1,uNum+1,vNum+1);
#endif

	vector<int> positions;
	int numHits;
	FindRenderLocation(tempTex, &numHits, uNum, vNum, &positions, glParam);
	this->silhouettePointIndex = positions;
	this->evaluatedSilhouette = true;

	glDeleteTextures(1, &tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);
}

int NURBS::SurfaceSilhouetteEvaluation(Float3 viewVector, GLParameters* glParam, int silRes, bool fitSilhouette)
{
	glParam->silhouette = true;
	// Store the ModelViewMatrix
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX, glParam->modelViewMatrix);

	int uNum = silRes;
	int vNum = silRes;

	bool evaluate = true;
	if(this->initializedEvalParams)
	{
		if (this->evalParams->uMin == 0.0  &&
			this->evalParams->uMax == 1.0  &&
			this->evalParams->vMin == 0.0  &&
			this->evalParams->vMax == 1.0  &&
			this->evalParams->uNum == uNum &&
			this->evalParams->vNum == vNum)
				evaluate = false;
	}
	if (evaluate)
	{
		if (this->initializedEvalParams)
			delete this->evalParams;
		this->evalParams = new EvaluationParams();
		this->evalParams->uNum = uNum;
		this->evalParams->vNum = vNum;
		this->initializedEvalParams = true;
	}

	Float4 evalInterval;
	evalInterval[0] = 0.0;
	evalInterval[1] = 1.0;
	evalInterval[2] = 0.0;
	evalInterval[3] = 1.0;
	
	vector<Float4> evaluationInterval;
	evaluationInterval.push_back(evalInterval);
	int maxIterations = 1;
	for (this->silhouetteLevel = 0; this->silhouetteLevel < maxIterations; this->silhouetteLevel++)
	{
		vector<Float4> newEvaluationInterval;		// uMin, uMax, vMin, vMax
		int numIntervals = evaluationInterval.size();
		for( int oldNumIntervalCount = 0; oldNumIntervalCount < numIntervals; oldNumIntervalCount++)
		{
			this->evalParams->uMin = evaluationInterval[oldNumIntervalCount][0];
			this->evalParams->uMax = evaluationInterval[oldNumIntervalCount][1];
			this->evalParams->vMin = evaluationInterval[oldNumIntervalCount][2];
			this->evalParams->vMax = evaluationInterval[oldNumIntervalCount][3];

			this->silhouettePointIndex.clear();
			
			if (evaluate)
			{
				this->SurfaceEvaluation(glParam, false, false);
				if (glParam->exactNormals)
					this->GPUExactNormalEvaluation(glParam);
				else
					this->GPUNormalEvaluation(glParam);
			}
			this->GPUSilhouetteEvaluation(viewVector, glParam);

			for( int p=0; p < this->silhouettePointIndex.size(); p++)
			{
				int vIndex = (int) this->silhouettePointIndex[p]/uNum;
				int uIndex = (int) this->silhouettePointIndex[p]%uNum;
				float uMin = float(uIndex*(this->evalParams->uMax - this->evalParams->uMin)/(1.0*uNum) + this->evalParams->uMin);
				float vMin = float(vIndex*(this->evalParams->vMax - this->evalParams->vMin)/(1.0*vNum) + this->evalParams->vMin);
				float uMax = float((uIndex+1)*(this->evalParams->uMax - this->evalParams->uMin)/(1.0*uNum) + this->evalParams->uMin);
				float vMax = float((vIndex+1)*(this->evalParams->vMax - this->evalParams->vMin)/(1.0*vNum) + this->evalParams->vMin);

				Float4 evalInterval;
				evalInterval[0] = uMin;
				evalInterval[1] = uMax;
				evalInterval[2] = vMin;
				evalInterval[3] = vMax;
				newEvaluationInterval.push_back(evalInterval);
			}
		}
		evaluationInterval = newEvaluationInterval;
	}
	float* silhouettePointInterval = new float[evaluationInterval.size()*4];

	float* silData;
	if (fitSilhouette)
		silData = new float[evaluationInterval.size()*2];
	for(int j = 0; j < evaluationInterval.size();j++)
	{
		silhouettePointInterval[j*4+0] = evaluationInterval[j][0];
		silhouettePointInterval[j*4+1] = evaluationInterval[j][1];
		silhouettePointInterval[j*4+2] = evaluationInterval[j][2];
		silhouettePointInterval[j*4+3] = evaluationInterval[j][3];
		if (fitSilhouette)
		{
			silData[j*2+0] = (evaluationInterval[j][0] + evaluationInterval[j][1])*.5*silRes;
			silData[j*2+1] = (evaluationInterval[j][2] + evaluationInterval[j][3])*.5*silRes;
		}
	}

	if (fitSilhouette)
	{
		vector<list<int>> polylines = this->FitSihouette(silData, evaluationInterval.size(), silRes);
//		DumpPolylineData(polylines,intData,numIntervals);
		cout<<"Number of Segments: " << polylines.size() << endl;
		delete[] silData;
	}

	this->GenerateIntersectionOverlayTexture(silhouettePointInterval, evaluationInterval.size(), (int)pow(uNum*1.0,maxIterations), glParam);
	
	delete [] silhouettePointInterval;
	glParam->silhouette = false;
	return 0;
}

void NURBS::UpdateTrimCurves(GLParameters* glParam)
{
	int nPoints			= this->pickedParamPoints.size();
	if (nPoints == 0)
		return;
	Float2 firstPoint		= this->pickedParamPoints[0];
	Float2 lastPoint		= this->pickedParamPoints[nPoints-1];
	float dist				= sqrt((firstPoint[0]-lastPoint[0])*(firstPoint[0]-lastPoint[0]) + (firstPoint[1]-lastPoint[1])*(firstPoint[1]-lastPoint[1]));
	bool newTrimCurveAdded	= false;
	if (dist < 0.05 && nPoints > 2)
	{
		nPoints++;
		int k = 3;
		BSpline* bSplineCurve		= new BSpline();
		bSplineCurve->nPoints		= nPoints;
		bSplineCurve->order			= k;
		int numKnots				= nPoints + k;
		bSplineCurve->cntlPoints	= new float[nPoints*3];
		bSplineCurve->knotVector	= new float[numKnots];
		for (int u = 0; u < nPoints-1; u++)
		{
			bSplineCurve->cntlPoints[u*3+0] = this->pickedParamPoints[u][0];
			bSplineCurve->cntlPoints[u*3+1] = this->pickedParamPoints[u][1];
			bSplineCurve->cntlPoints[u*3+2] = 1.0;
		}
		bSplineCurve->cntlPoints[(nPoints-1)*3+0] = this->pickedParamPoints[0][0];
		bSplineCurve->cntlPoints[(nPoints-1)*3+1] = this->pickedParamPoints[0][1];
		bSplineCurve->cntlPoints[(nPoints-1)*3+2] = 1.0;
		for (int u = 0; u < k-1; u++)
			bSplineCurve->knotVector[u] = 0;
		for (int u = k-1; u < numKnots - k + 1; u++)
			bSplineCurve->knotVector[u] = (u-k+1)/((numKnots-2*k+1)*1.0);
		for (int u = numKnots - k + 1; u < numKnots; u++)
			bSplineCurve->knotVector[u] = 1;

		this->trimCurves.push_back(bSplineCurve);
		newTrimCurveAdded = true;
	}
	else if (nPoints > 2)
	{
		int k = 3;
		BSpline* bSplineCurve		= new BSpline();
		bSplineCurve->nPoints		= 2*nPoints+1;
		bSplineCurve->order			= k;
		int numKnots				= bSplineCurve->nPoints + k;
		bSplineCurve->cntlPoints	= new float[bSplineCurve->nPoints*3];
		bSplineCurve->knotVector	= new float[numKnots];
		for (int u = 0; u < nPoints; u++)
		{
			bSplineCurve->cntlPoints[u*3+0] = this->pickedParamPoints[u][0];
			bSplineCurve->cntlPoints[u*3+1] = this->pickedParamPoints[u][1];
			bSplineCurve->cntlPoints[u*3+2] = 1.0;
		}
		Float2 dirPerpP1P2,p1,p2;
		float thickness = 0.02;
		for (int u = nPoints; u < 2*nPoints-1; u++)
		{
			p1 = this->pickedParamPoints[2*nPoints-u-1];
			p2 = this->pickedParamPoints[2*nPoints-u-2];
			Float2 dirP1P2;
			float lenDirP1P2 = float(sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])));
			dirP1P2[0] =  (p1[0]-p2[0])/lenDirP1P2;
			dirP1P2[1] =  (p1[1]-p2[1])/lenDirP1P2;
			dirPerpP1P2 = Float2(-dirP1P2[1],dirP1P2[0]);
			float newX = p1[0] + dirPerpP1P2[0]*thickness;
			float newY = p1[1] + dirPerpP1P2[1]*thickness;
			bSplineCurve->cntlPoints[u*3+0] = newX;
			bSplineCurve->cntlPoints[u*3+1] = newY;
			bSplineCurve->cntlPoints[u*3+2] = 1.0;
		}
		float newX = p2[0] + dirPerpP1P2[0]*thickness;
		float newY = p2[1] + dirPerpP1P2[1]*thickness;
		bSplineCurve->cntlPoints[(2*nPoints-1)*3+0] = newX;
		bSplineCurve->cntlPoints[(2*nPoints-1)*3+1] = newY;
		bSplineCurve->cntlPoints[(2*nPoints-1)*3+2] = 1.0;

		bSplineCurve->cntlPoints[(2*nPoints)*3+0] = this->pickedParamPoints[0][0];
		bSplineCurve->cntlPoints[(2*nPoints)*3+1] = this->pickedParamPoints[0][1];
		bSplineCurve->cntlPoints[(2*nPoints)*3+2] = 1.0;

		for (int u = 0; u < k-1; u++)
			bSplineCurve->knotVector[u] = 0;
		for (int u = k-1; u < numKnots - k + 1; u++)
			bSplineCurve->knotVector[u] = (u-k+1)/((numKnots-2*k+1)*1.0);
		for (int u = numKnots - k + 1; u < numKnots; u++)
			bSplineCurve->knotVector[u] = 1;

		this->trimCurves.push_back(bSplineCurve);
		newTrimCurveAdded = true;
	}
	if (newTrimCurveAdded)
	{
		if (!this->trimmed)
		{
			this->trimmed = true;
			//Trimming for the outline
			float tempCntlPoints[15] = {0,0,1,1,0,1,1,1,1,0,1,1,0,0,1};
			for(int k=0; k<4; k++)
			{
				BSpline* bSplineCurve		= new BSpline();
				bSplineCurve->nPoints		= 2;
				bSplineCurve->order			= 2;
				int numKnots				= 4;
				bSplineCurve->cntlPoints	= new float[6];
				bSplineCurve->knotVector	= new float[numKnots];
				float tempKnots[4]			= {0,0,1,1};
				bSplineCurve->cntlPoints[0] = tempCntlPoints[k*3+0];
				bSplineCurve->cntlPoints[1] = tempCntlPoints[k*3+1];
				bSplineCurve->cntlPoints[2] = tempCntlPoints[k*3+2];
				bSplineCurve->cntlPoints[3] = tempCntlPoints[k*3+3];
				bSplineCurve->cntlPoints[4] = tempCntlPoints[k*3+4];
				bSplineCurve->cntlPoints[5] = tempCntlPoints[k*3+5];
				for (int u = 0; u < 4; u++)
					bSplineCurve->knotVector[u] = tempKnots[u];
				this->trimCurves.push_back(bSplineCurve);
			}
		}
		this->GenerateTrimTexture(glParam);
	}
}

void NURBS::DrawOBB()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
//	glMultMatrixf(this->transformationMatrix);
	// set vertex color to green	
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.0,1.0,0.0);            
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

#ifdef CUDA
int NURBS::CUDASurfaceBoundingBoxEvaluation(bool constructHierarchy, bool expandBoundingBox, bool readBack)
{
	bool timing = false;
	Timer localTimer;
	float normalTime, bBoxTime, hierarchyTime;
	if (timing)
		localTimer.Start();

//	To create non axis aligned bounding-boxes
//	MakeIdentityMatrix(glParam->modelViewMatrix);

	// Create the texture data
	int uNum	= this->evalParams->uNum;
	int vNum	= this->evalParams->vNum;

	int	texWidth	= uNum;
	int	texHeight	= vNum;

	if (constructHierarchy)
		texWidth *= 2;

	// Allocate Data
	cudaMalloc((void**)&(this->evalParams->bBoxMaxCUDAData), texWidth*texHeight*sizeof(float3));
	cudaMalloc((void**)&(this->evalParams->bBoxMinCUDAData), texWidth*texHeight*sizeof(float3));

	if(expandBoundingBox)
		this->CUDAExactNormalEvaluation(false, true);

	if (timing)
	{
		localTimer.Stop();
		normalTime = localTimer.GetElapsedTimeInMicroSec();
		localTimer.Start();
	}

	if (readBack)
		CUDAComputeBoundingBox(uNum, vNum, this->evalParams->evaluatedCUDAPoints, this->evalParams->bBoxMaxCUDAData, this->evalParams->bBoxMinCUDAData, this->evalParams->secondDervCUDAData, expandBoundingBox, this->evalParams->maxExpansionFactor, this->evalParams->minPoints, this->evalParams->maxPoints);
	else
		CUDAComputeBoundingBox(uNum, vNum, this->evalParams->evaluatedCUDAPoints, this->evalParams->bBoxMaxCUDAData, this->evalParams->bBoxMinCUDAData, this->evalParams->secondDervCUDAData, expandBoundingBox, this->evalParams->maxExpansionFactor);

	
	if (timing)
	{
		localTimer.Stop();
		bBoxTime = localTimer.GetElapsedTimeInMicroSec();
		localTimer.Start();
	}

	if (constructHierarchy)
	{
		float bBoxMinVal[3], bBoxMaxVal[3];
		CUDABuildBoundingBoxHierarchy(uNum, vNum, bBoxMinVal, bBoxMaxVal);
		this->bBoxMax = Float3(bBoxMaxVal[0], bBoxMaxVal[1], bBoxMaxVal[2]);
		this->bBoxMin = Float3(bBoxMinVal[0], bBoxMinVal[1], bBoxMinVal[2]);
	}

	if (timing)
	{
		localTimer.Stop();
		hierarchyTime = localTimer.GetElapsedTimeInMicroSec();
		cout<<"Total Time          : "<<0.001*int(normalTime + bBoxTime + hierarchyTime)<<endl;
		cout<<"Normal Time         : "<<0.001*int(normalTime)<<endl;
		cout<<"Bounding Box Time   : "<<0.001*int(bBoxTime)<<endl;
		cout<<"Hierarchy Time      : "<<0.001*int(hierarchyTime)<<endl<<endl;
	}

	return 0;
}

int NURBS::CUDAComputeBoundingBox(int uNum, int vNum, float4* surfacePointsCUDAData, float3* bBoxMinCUDAData, float3* bBoxMaxCUDAData, float3* secondDervCUDAData[4], bool expandBoundingBox, float &magFactor, float* minData, float* maxData)
{
	magFactor = 0;
	if (expandBoundingBox)
	{
		float* k1CUDAData;
		float* k2CUDAData;
		float* k3CUDAData;
		float* k4CUDAData;
		cudaMalloc((void**)&k1CUDAData, (uNum+1)*(vNum+1)*sizeof(float));
		cudaMalloc((void**)&k2CUDAData, (uNum+1)*(vNum+1)*sizeof(float));
		cudaMalloc((void**)&k3CUDAData, (uNum+1)*(vNum+1)*sizeof(float));
		cudaMalloc((void**)&k4CUDAData, (uNum+1)*(vNum+1)*sizeof(float));
		CUDAExpansionFactorEvaluate(secondDervCUDAData, uNum, vNum, k1CUDAData, k2CUDAData, k3CUDAData, k4CUDAData);
		CUDACheckErrors(" k Evaluation ");

#ifdef THRUST
		Float4 maxValue = THRUSTFindMax(uNum+1, vNum+1, k1CUDAData, k2CUDAData, k3CUDAData, k4CUDAData);
#else
		Float4 maxValue = CPUFindMax(uNum+1, vNum+1, k1CUDAData, k2CUDAData, k3CUDAData, k4CUDAData);
		//Float4 maxValue = CPUFindAvg(uNum+1, vNum+1, k1CUDAData, k2CUDAData, k3CUDAData, k4CUDAData);
#endif
		magFactor = maxValue[0] + maxValue[1] + maxValue[2] + maxValue[3];

		cudaFree(k1CUDAData);
		cudaFree(k2CUDAData);
		cudaFree(k3CUDAData);
		cudaFree(k4CUDAData);
		CUDACheckErrors(" k Max ");
	}

	float4* surfaceData = this->evalParams->evaluatedCUDAPoints;
	//Transformations
	if (this->identityTransformation)
		surfaceData = this->evalParams->evaluatedCUDAPoints;
	else
		surfaceData = this->evalParams->transformedCUDAPoints;

	// Construct BBox
	CUDABoundingBoxEvaluate(surfaceData, uNum, vNum, magFactor, this->evalParams->bBoxMinCUDAData, this->evalParams->bBoxMaxCUDAData);

#ifdef DUMP
	float* tempData = new float[(uNum)*(vNum)*3];

	cudaMemcpy(tempData, bBoxMin, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData ,(uNum+1),(vNum+1), 0, 4);
#endif

	if (minData!=NULL && maxData!=NULL)
	{
		cudaMemcpy(minData, this->evalParams->bBoxMinCUDAData, uNum*vNum*3*sizeof(float), cudaMemcpyDeviceToHost);
		cudaMemcpy(maxData, this->evalParams->bBoxMaxCUDAData, uNum*vNum*3*sizeof(float), cudaMemcpyDeviceToHost);
//		WriteToFile(maxData,"MaxData.txt",4,texWidth*texHeight);
//		WriteToFile(minData,"MinData.txt",4,texWidth*texHeight);
	}

	return 1;
}

int NURBS::CUDABuildBoundingBoxHierarchy(int uNum, int vNum, float* minVal = NULL, float* maxVal = NULL)
{
	int yReadPos = 0;
	int readOffset = 0;
	for(int currentSize = uNum/2; currentSize >= 1; currentSize = currentSize/2) 
	{
		CUDABoundingBoxHierarchy(this->evalParams->bBoxMinCUDAData+readOffset, this->evalParams->bBoxMaxCUDAData+readOffset, uNum, vNum, currentSize, yReadPos);

#ifdef DUMP
		float* tempData = new float[(uNum)*(2*vNum)*3];

		cudaMemcpy(tempData, this->evalParams->bBoxMinCUDAData, uNum*vNum*2*sizeof(float3), cudaMemcpyDeviceToHost);
		WriteToFile(tempData, uNum, vNum*2, 0, 3);

		cudaMemcpy(tempData, this->evalParams->bBoxMaxCUDAData, uNum*vNum*2*sizeof(float3), cudaMemcpyDeviceToHost);
		WriteToFile(tempData, uNum, vNum*2, 0, 3);

		delete[] tempData;
#endif
		yReadPos += currentSize;
		readOffset += uNum*currentSize*2;
	}
	if (minVal != NULL && maxVal!=NULL)
	{
		cudaMemcpy(minVal, this->evalParams->bBoxMinCUDAData+readOffset, sizeof(float3), cudaMemcpyDeviceToHost);
		cudaMemcpy(maxVal, this->evalParams->bBoxMaxCUDAData+readOffset, sizeof(float3), cudaMemcpyDeviceToHost);
	}
	return 1;
}

int NURBS::CUDABasisFunctionEvaluation(int k, int n, int evalNum, float paramMin, float paramMax, float* knotVector, int* ctrlData, float* basisGPUData, float* basisDervGPUData)
{
	int evalLen			= int (evalNum+1);
	float* knotData		= new float[evalLen*(2*k+1)];
	float* basisData	= new float[evalLen*k];

	int prevStartIndex = 0;
	for (int index = 0; index < evalLen; index++)
	{
		int runningIndex = index;
		float pVal=float(index*(paramMax - paramMin)/(1.0*evalNum) + paramMin);
		int startIndex = prevStartIndex;
		while ( pVal >= knotVector[startIndex] && startIndex < n+k )
			startIndex++;
		prevStartIndex = startIndex;

		//	if (index != uNum)
		if (pVal < 1 - DELTA)
			startIndex-=k;
		else
			startIndex-=2*k;

		knotData[(2*k+1)*runningIndex]=pVal;
		for (int i = 1; i < (2*k+1); i++)
			knotData[(2*k+1)*runningIndex+i] = knotVector[startIndex+i-1];

		ctrlData[index]=startIndex;

		for (int i = 0; i < k-1; i++)
			basisData[runningIndex*k + i] = 0;

		basisData[runningIndex*k + k - 1] = 1;
	}

//	WriteToFile(knotData, 2*k+1, evalNum+1);
	float* knotGPUData;
	cudaMalloc((void**)&knotGPUData, (evalNum+1)*(2*k+1)*sizeof(float));
	CUDACheckErrors(" Memory Allocation ");

	cudaMemcpy(basisGPUData, basisData, (evalNum+1)*k*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(knotGPUData, knotData, (evalNum+1)*(2*k+1)*sizeof(float), cudaMemcpyHostToDevice);
	CUDACheckErrors(" Copy Data ");

#ifdef DUMP
	float* tempData = new float[(evalNum+1)*(2*k+1)];
	cudaMemcpy(tempData, knotGPUData, (evalNum+1)*(2*k+1)*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData,(2*k+1),(evalNum+1));
	delete [] tempData;
#endif

	for(int pass=2; pass<=k ;pass++)
	{
		int result = CUDABasisEvaluate(knotGPUData, pass, k, evalNum+1, basisGPUData);
		assert(result == 1);
		CUDACheckErrors(" After Basis Evaluation ");

		// Copy k-1th pass data to basisDervGPUData
		if (basisDervGPUData != NULL && (pass == k-1 || (pass == 2 && k == 2)))
			cudaMemcpy(basisDervGPUData, basisGPUData, (evalNum+1)*k*sizeof(float), cudaMemcpyDeviceToDevice);
	
		CUDACheckErrors(" After Basis Derv Copy ");
#ifdef DUMP
		float* tempData = new float[(evalNum+1)*k];
		cudaMemcpy(tempData, basisGPUData, (evalNum+1)*k*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData,k,(evalNum+1));
		delete [] tempData;
#endif
	}

	if (basisDervGPUData != NULL)
	{
		int result = CUDABasisDervEvaluate(knotGPUData, k, evalNum+1, basisDervGPUData);
		assert(result == 1);
		CUDACheckErrors(" After Basis Derv Evaluation ");
#ifdef DUMP
		float* tempData = new float[(evalNum+1)*k];
		cudaMemcpy(tempData, basisDervGPUData, (evalNum+1)*k*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData,k,(evalNum+1));
		delete [] tempData;
#endif

	}
	cudaFree(knotGPUData);
	delete[] basisData;
	delete[] knotData;

	return 1;

}

int NURBS::CUDABasisCtrlPointMultiplication(float* ctrlPointsGPUData, int* uCtrlPointIndexGPUData, int* vCtrlPointIndexGPUData, float* uBasisGPUData, float* vBasisGPUData, float4* output)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;
	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

#ifdef PINGPONG
	// execute the kernel
	for(int vPass=0; vPass<kv; vPass++)
		for(int uPass=0; uPass<ku; uPass++)
			CUDABasisMult(ctrlPointsGPUData, uCtrlPointIndexGPUData, vCtrlPointIndexGPUData, uBasisGPUData, vBasisGPUData, output, uNum+1, vNum+1, nu, nv, ku, kv, uPass, vPass);
#else
	CUDABasisMult(ctrlPointsGPUData, uCtrlPointIndexGPUData, vCtrlPointIndexGPUData, uBasisGPUData, vBasisGPUData, output, uNum+1, vNum+1, nu, nv, ku, kv, -1, -1);
#endif

#ifdef DUMP
	float* tempData = new float[(uNum+1)*(vNum+1)*4];
	cudaMemcpy(tempData, output, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData ,(uNum+1),(vNum+1),2);
	delete [] tempData;
#endif
	return 1;
}

int NURBS::CUDABasisCtrlPointMultiplicationTextures(cudaArray* ctrlPointsGPUData, cudaArray* uCtrlPointIndexGPUData, cudaArray* vCtrlPointIndexGPUData, cudaArray* uBasisGPUData, cudaArray* vBasisGPUData, float4* output)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;
	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

#ifdef PINGPONG
	// execute the kernel
	for(int vPass=0; vPass<kv; vPass++)
		for(int uPass=0; uPass<ku; uPass++)
			CUDABasisMultTextures(ctrlPointsGPUData, uCtrlPointIndexGPUData, vCtrlPointIndexGPUData, uBasisGPUData, vBasisGPUData, output, uNum+1, vNum+1, nu, nv, ku, kv, uPass, vPass);
#else
	CUDABasisMultTextures(ctrlPointsGPUData, uCtrlPointIndexGPUData, vCtrlPointIndexGPUData, uBasisGPUData, vBasisGPUData, output, uNum+1, vNum+1, nu, nv, ku, kv, -1, -1);
#endif

#ifdef DUMP
	float* tempData = new float[(uNum+1)*(vNum+1)*4];
	cudaMemcpy(tempData, surfPt, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData ,(uNum+1),(vNum+1),2);
	delete [] tempData;
#endif

	return 1;
}

int NURBS::CUDAExactNormalEvaluation(bool rendering, bool evaluateSecondDerivatives)//, cudaGraphicsResource* pointVBOResource, cudaGraphicsResource* normalVBOResource)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;
	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

	float* uBasisGPUData		= this->evalParams->uBasisFuncCUDAData;
	float* vBasisGPUData		= this->evalParams->vBasisFuncCUDAData;
	float* uBasisDervGPUData	= this->evalParams->uBasisDervCUDAData;
	float* vBasisDervGPUData	= this->evalParams->vBasisDervCUDAData;
	float* ctrlPointsGPUData	= this->evalParams->ctrlPointsCUDAData;
	int* uCtrlPointIndexGPUData = this->evalParams->uCtrlPointIndexCUDAData;
	int* vCtrlPointIndexGPUData = this->evalParams->vCtrlPointIndexCUDAData;

	// Map OpenGL buffer object for writing from CUDA
	if (rendering)
		cudaGLMapBufferObject((void**)&(this->evalParams->evaluatedCUDANormals), this->normVBO);
	else
		cudaMalloc((void**)&(this->evalParams->evaluatedCUDANormals), (uNum+1)*(vNum+1)*sizeof(float3));

	float4* uDervGPUData;
	// After rational divide
	float3* uRatDervGPUData;
	cudaMalloc((void**)&uDervGPUData, (uNum+1)*(vNum+1)*sizeof(float4));
	cudaMalloc((void**)&uRatDervGPUData, (uNum+1)*(vNum+1)*sizeof(float3));
	CUDACheckErrors(" Memory Allocation ");

	CUDABasisCtrlPointMultiplication(ctrlPointsGPUData, uCtrlPointIndexGPUData, vCtrlPointIndexGPUData, uBasisDervGPUData, vBasisGPUData, uDervGPUData);
	CUDACheckErrors(" u Derv Evaluation ");

#ifdef DEBUG
		float* tempData = new float[(uNum+1)*(vNum+1)*4];
		cudaMemcpy(tempData, uDervGPUData, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData ,(uNum+1),(vNum+1), 0);
		delete [] tempData;
#endif

	CUDARationalDivide(this->evalParams->evaluatedCUDAPoints, uDervGPUData, uNum+1, vNum+1, uRatDervGPUData);
	CUDACheckErrors(" u Rational Derv Evaluation ");

	float4* vDervGPUData;
	// After rational divide
	float3* vRatDervGPUData;
	cudaMalloc((void**)&vDervGPUData, (uNum+1)*(vNum+1)*sizeof(float4));
	cudaMalloc((void**)&vRatDervGPUData, (uNum+1)*(vNum+1)*sizeof(float3));
	CUDACheckErrors(" Memory Allocation ");

	CUDABasisCtrlPointMultiplication(ctrlPointsGPUData, uCtrlPointIndexGPUData, vCtrlPointIndexGPUData, uBasisGPUData, vBasisDervGPUData, vDervGPUData);
	CUDACheckErrors(" v Derv Evaluation ");

	CUDARationalDivide(this->evalParams->evaluatedCUDAPoints, vDervGPUData, uNum+1, vNum+1, vRatDervGPUData);
	CUDACheckErrors(" v Rational Derv Evaluation ");

#ifdef DEBUG
		float* tempData = new float[(uNum+1)*(vNum+1)*4];

		cudaMemcpy(tempData, uDervGPUData, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData ,(uNum+1),(vNum+1), 0, 4);

		cudaMemcpy(tempData, vDervGPUData, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData ,(uNum+1),(vNum+1), 0, 4);

		cudaMemcpy(tempData, uRatDervGPUData, (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData ,(uNum+1),(vNum+1), 0, 3);

		cudaMemcpy(tempData, vRatDervGPUData, (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData ,(uNum+1),(vNum+1), 0, 3);
#endif

	// Evaluate second derivatives
	if (evaluateSecondDerivatives)
	{
		float uInterval = this->evalParams->uNum/(this->evalParams->uMax - this->evalParams->uMin);
		float vInterval = this->evalParams->vNum/(this->evalParams->vMax - this->evalParams->vMin);
		cudaMalloc((void**)&(this->evalParams->secondDervCUDAData[0]), (uNum+1)*(vNum+1)*sizeof(float3));
		cudaMalloc((void**)&(this->evalParams->secondDervCUDAData[1]), (uNum+1)*(vNum+1)*sizeof(float3));
		cudaMalloc((void**)&(this->evalParams->secondDervCUDAData[2]), (uNum+1)*(vNum+1)*sizeof(float3));
		cudaMalloc((void**)&(this->evalParams->secondDervCUDAData[3]), (uNum+1)*(vNum+1)*sizeof(float3));
		CUDASecondDerivativeEvaluate(uRatDervGPUData, vRatDervGPUData, uNum+1, vNum+1, uInterval, vInterval, this->evalParams->secondDervCUDAData);
		CUDACheckErrors(" Second Derv Evaluation ");

#ifdef DEBUG
		float* tempData = new float[(uNum+1)*(vNum+1)*3];
		cudaMemcpy(tempData, this->evalParams->secondDervCUDAData[0], (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData, (uNum+1), (vNum+1), 0, 3);

		cudaMemcpy(tempData, this->evalParams->secondDervCUDAData[1], (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData, (uNum+1), (vNum+1), 0, 3);

		cudaMemcpy(tempData, this->evalParams->secondDervCUDAData[2], (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData, (uNum+1), (vNum+1), 0, 3);

		cudaMemcpy(tempData, this->evalParams->secondDervCUDAData[3], (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
		WriteToFile(tempData, (uNum+1), (vNum+1), 0, 3);

		delete [] tempData;
#endif
	}

	// Evaluate surface normals
	CUDAExactNormalEvaluate(uRatDervGPUData, vRatDervGPUData, uNum+1, vNum+1, this->evalParams->evaluatedCUDANormals);
	CUDACheckErrors(" Exact Normal Evaluation ");

#ifdef DUMP
	float* tempData = new float[(uNum+1)*(vNum+1)*3];
	cudaMemcpy(tempData, surfNormal, (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData ,3,(uNum+1)*(vNum+1));
	delete [] tempData;
#endif

	// Clean up temp data
	cudaFree(uDervGPUData);
	cudaFree(vDervGPUData);
	cudaFree(uRatDervGPUData);
	cudaFree(vRatDervGPUData);

	if (rendering)
		cudaGLUnmapBufferObject(this->normVBO);

	return 1;
}

int NURBS::CUDANormalEvaluation(bool rendering)//, cudaGraphicsResource* pointVBOResource, cudaGraphicsResource* normalVBOResource)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	// Map OpenGL buffer object for writing from CUDA
	if (rendering)
		cudaGLMapBufferObject((void**)&(this->evalParams->evaluatedCUDANormals), this->normVBO);
	else
		cudaMalloc((void**)&(this->evalParams->evaluatedCUDANormals), (uNum+1)*(vNum+1)*sizeof(float3));

	CUDANormalEvaluate(this->evalParams->evaluatedCUDAPoints, uNum+1, vNum+1, this->evalParams->evaluatedCUDANormals);

#ifdef DUMP
	float* tempData = new float[(uNum+1)*(vNum+1)*3];
	cudaMemcpy(tempData, surfNormal, (uNum+1)*(vNum+1)*3*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData ,3,(uNum+1)*(vNum+1));
	delete [] tempData;
#endif

	if (rendering)
		cudaGLUnmapBufferObject(this->normVBO);

	return 1;
}

int NURBS::CUDASurfaceEvaluation(bool exactNormals, bool evaluateSecondDerivatives, bool readBack, bool rendering)//, cudaGraphicsResource* pointVBOResource, cudaGraphicsResource* normalVBOResource)
{
	//Set up for evaluation

	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

	int* uCtrlData		= new int[(uNum+1)];
	int* vCtrlData		= new int[(vNum+1)];

	cudaMalloc((void**)&(this->evalParams->uBasisFuncCUDAData), (uNum+1)*ku*sizeof(float));
	cudaMalloc((void**)&(this->evalParams->vBasisFuncCUDAData), (vNum+1)*kv*sizeof(float));
	if(exactNormals)
	{
		cudaMalloc((void**)&(this->evalParams->uBasisDervCUDAData), (uNum+1)*ku*sizeof(float));
		cudaMalloc((void**)&(this->evalParams->vBasisDervCUDAData), (vNum+1)*kv*sizeof(float));
	}
	CUDACheckErrors(" Memory Allocation ");

	CUDABasisFunctionEvaluation(ku, nu, uNum, this->evalParams->uMin, this->evalParams->uMax, this->uKnotVector, uCtrlData, this->evalParams->uBasisFuncCUDAData, this->evalParams->uBasisDervCUDAData);
	CUDABasisFunctionEvaluation(kv, nv, vNum, this->evalParams->vMin, this->evalParams->vMax, this->vKnotVector, vCtrlData, this->evalParams->vBasisFuncCUDAData, this->evalParams->vBasisDervCUDAData);

#ifdef DUMP
	float* tempData = new float[(uNum+1)*ku];
	cudaMemcpy(tempData, this->evalParams->uBasisDervCUDAData, (uNum+1)*ku*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData,ku,(uNum+1));
	cudaMemcpy(tempData, this->evalParams->vBasisDervCUDAData, (uNum+1)*ku*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData,ku,(uNum+1));
	delete [] tempData;
#endif
	
	cudaMalloc((void**)&(this->evalParams->ctrlPointsCUDAData), nu*nv*4*sizeof(float));
	cudaMalloc((void**)&(this->evalParams->uCtrlPointIndexCUDAData), (uNum+1)*sizeof(int));
	cudaMalloc((void**)&(this->evalParams->vCtrlPointIndexCUDAData), (vNum+1)*sizeof(int));
	CUDACheckErrors(" Memory Allocation ");

	// Copy data to device
	cudaMemcpy(this->evalParams->ctrlPointsCUDAData, this->cntlPoints, nu*nv*4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(this->evalParams->uCtrlPointIndexCUDAData, uCtrlData, (uNum+1)*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(this->evalParams->vCtrlPointIndexCUDAData, vCtrlData, (vNum+1)*sizeof(int), cudaMemcpyHostToDevice);
	CUDACheckErrors(" Copy Data ");

	delete[] uCtrlData;
	delete[] vCtrlData;

	// map OpenGL buffer object for writing from CUDA
	if (rendering)
	{
		cudaGLMapBufferObject((void**)&(this->evalParams->evaluatedCUDAPoints), this->ptVBO);
//		cudaGraphicsMapResources(1, &pointVBOResource, 0);
//		size_t numBytes;
//		cudaGraphicsResourceGetMappedPointer((void**)&(this->evalParams->evaluatedCUDAPoints), &numBytes, pointVBOResource);
	}
	else
		cudaMalloc((void**)&(this->evalParams->evaluatedCUDAPoints), (uNum+1)*(vNum+1)*sizeof(float4));

	CUDABasisCtrlPointMultiplication(this->evalParams->ctrlPointsCUDAData, this->evalParams->uCtrlPointIndexCUDAData, this->evalParams->vCtrlPointIndexCUDAData, this->evalParams->uBasisFuncCUDAData, this->evalParams->vBasisFuncCUDAData, this->evalParams->evaluatedCUDAPoints);

	if (rendering)
	{
		cudaGLUnmapBufferObject(this->ptVBO);
		//cudaGraphicsUnmapResources(1, &pointVBOResource, 0);
	}
	CUDACheckErrors(" End Evaluation ");

	if (readBack && this->evalParams->evaluatedPoints!=NULL && this->identityTransformation)
		cudaMemcpy(this->evalParams->evaluatedPoints, this->evalParams->evaluatedCUDAPoints, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	if (rendering)
	{
		if (exactNormals)
			CUDAExactNormalEvaluation(true, false);//, pointVBOResource, normalVBOResource);
		else
			CUDANormalEvaluation(true);//, pointVBOResource, normalVBOResource);
	}
	return 0;
}

int NURBS::CUDASurfaceTransformation(bool readBack)
{
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	// Handle transformations
	cudaMalloc((void**)&(this->evalParams->transformedCUDAPoints), (uNum+1)*(vNum+1)*sizeof(float4));

	CUDATransformEvaluatedPoints(this->evalParams->evaluatedCUDAPoints, uNum+1, vNum+1, this->transformationMatrix, this->evalParams->transformedCUDAPoints);
	CUDACheckErrors(" Transformation ");

#ifdef DUMP
	float* tempData = new float[(uNum)*(vNum)*4];

	cudaMemcpy(tempData, transformedData, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData ,(uNum+1),(vNum+1), 0, 4);
#endif
	if (readBack && this->evalParams->evaluatedPoints!=NULL)
		cudaMemcpy(this->evalParams->evaluatedPoints, this->evalParams->transformedCUDAPoints, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);
	CUDACheckErrors(" Data Transfer ");

	return 0;
}

//iddo: gets Bezier surfaces (output of SplitSurfaceIntoFlatBezPatches) - each of size uOrder*vOrder*4 and stores in CUDA memory
void NURBS::CUDASetBezPatches()
{
	int uNum = this->uOrder;
	int vNum = this->vOrder;

	//freeing CUDA memory of bezPatches, in case they were allocated before
	if (!m_bezCUDAPatches.empty())
	{
		for (int i=0; i<m_bezCUDAPatches.size(); ++i)
			cudaFree(m_bezCUDAPatches[i]);
		m_bezCUDAPatches = std::vector<float4*>(); //reset it to empty
		CUDACheckErrors(" Bezier Empty ");
	}

	for (int i = 0; i < m_bezPatches.size(); ++i)
	{
		float4* bezPatch;
		cudaMalloc((void**)&(bezPatch), uNum*vNum*sizeof(float4));
		cudaMemcpy(bezPatch, m_bezPatches[i], uNum*vNum*sizeof(float4), cudaMemcpyHostToDevice);
		CUDACheckErrors(" Bezier Memory Allocation ");
		m_bezCUDAPatches.push_back(bezPatch);
	}
}

void NURBS::DrawBezierPatchControlPoints(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
//	glDisable(GL_DEPTH_TEST);
	glColor3d(0.3, 0.3, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);

	glPointSize(7.0);
	glLineWidth(2.0);

	for (int bezPatch = 0; bezPatch < this->m_bezPatches.size(); bezPatch++)
	{
		float* curBezPatchData = m_bezPatches[bezPatch];
		glColor3d(0.3, 0.3, 1.0);
		/*
		{
			int u = 0, v = 0, uOff = this->uOrder-1, vOff = this->vOrder-1;
			float x1 = curBezPatchData[v*this->uOrder*4+u*4+0];
			float y1 = curBezPatchData[v*this->uOrder*4+u*4+1];
			float z1 = curBezPatchData[v*this->uOrder*4+u*4+2];
			float w1 = curBezPatchData[v*this->uOrder*4+u*4+3];

			float x2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+0];
			float y2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+1];
			float z2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+2];
			float w2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+3];

			float x3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+0];
			float y3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+1];
			float z3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+2];
			float w3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+3];
					  
			float x4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+0];
			float y4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+1];
			float z4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+2];
			float w4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+3];

			glBegin(GL_QUADS);
				glVertex4f(x1,y1,z1,w1);
				glVertex4f(x2,y2,z2,w2);
				glVertex4f(x3,y3,z3,w3);
				glVertex4f(x4,y4,z4,w4);
			glEnd();
		}
		*/
		for (int v = 0; v < this->vOrder-1; v++)
		{
			for (int u = 0; u < this->uOrder-1; u++)
			{
				glColor3d(0.3, 1.0, 0.3);
				int uOff = 1, vOff = 1;
				float x1 = curBezPatchData[v*this->uOrder*4+u*4+0];
				float y1 = curBezPatchData[v*this->uOrder*4+u*4+1];
				float z1 = curBezPatchData[v*this->uOrder*4+u*4+2];
				float w1 = curBezPatchData[v*this->uOrder*4+u*4+3];

				float x2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+0];
				float y2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+1];
				float z2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+2];
				float w2 = curBezPatchData[v*this->uOrder*4+(u+uOff)*4+3];

				float x3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+0];
				float y3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+1];
				float z3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+2];
				float w3 = curBezPatchData[(v+vOff)*this->uOrder*4+(u+uOff)*4+3];
						  
				float x4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+0];
				float y4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+1];
				float z4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+2];
				float w4 = curBezPatchData[(v+vOff)*this->uOrder*4+u*4+3];
				
				//glBegin(GL_QUADS);
				glBegin(GL_POINTS);
					glVertex4f(x1,y1,z1,w1);
					glVertex4f(x2,y2,z2,w2);
					glVertex4f(x3,y3,z3,w3);
					glVertex4f(x4,y4,z4,w4);
				glEnd();
				/*
				Color3d(0.3, 0.3, 1.0);

				if (v == 0)
				{
					glBegin(GL_LINES);
						glVertex4f(x1,y1,z1,w1);
						glVertex4f(x2,y2,z2,w2);
					glEnd();
				}
				
				if (u == 0)
				{
					glBegin(GL_LINES);
						glVertex4f(x4,y4,z4,w4);
						glVertex4f(x1,y1,z1,w1);
					glEnd();
				}

				if (v == vOrder-2)
				{
					glBegin(GL_LINES);
					glVertex4f(x3,y3,z3,w3);
					glVertex4f(x4,y4,z4,w4);
					glEnd();
				}
				
				if (u == uOrder-2)
				{
					glBegin(GL_LINES);
					glVertex4f(x2,y2,z2,w2);
					glVertex4f(x3,y3,z3,w3);
					glEnd();
				}
				*/


			}
		}
/*
		for (int v = 0; v < this->vOrder ; v++)
		{
			for (int u = 0; u < this->uOrder ; u++)
			{
				float x = curBezPatchData[v*this->uOrder*4+u*4+0];
				float y = curBezPatchData[v*this->uOrder*4+u*4+1];
				float z = curBezPatchData[v*this->uOrder*4+u*4+2];
				float w = curBezPatchData[v*this->uOrder*4+u*4+3];

				glBegin(GL_POINTS);
					glVertex4f(x,y,z,w);
				glEnd();
			}
		}
		*/
	}
	glPopAttrib();
}

#ifdef CUDATEXTURES
int NURBS::CUDASurfaceEvaluationTextures(bool exactNormals, bool evaluateSecondDerivatives, bool readBack, bool rendering)
{
	//Set up for evaluation
	// Evaluation grid size
	int uNum			= this->evalParams->uNum;
	int vNum			= this->evalParams->vNum;

	int ku				= this->uOrder;
	int kv				= this->vOrder;
	int nu				= this->uPoints;
	int nv				= this->vPoints;

	int* uCtrlData			= new int[(uNum+1)];
	int* vCtrlData			= new int[(vNum+1)];

	cudaMalloc((void**)&(this->evalParams->uBasisFuncCUDAData), (uNum+1)*ku*sizeof(float));
	cudaMalloc((void**)&(this->evalParams->vBasisFuncCUDAData), (vNum+1)*kv*sizeof(float));
	if(exactNormals)
	{
		cudaMalloc((void**)&(this->evalParams->uBasisDervCUDAData), (uNum+1)*ku*sizeof(float));
		cudaMalloc((void**)&(this->evalParams->vBasisDervCUDAData), (vNum+1)*kv*sizeof(float));
	}
	CUDACheckErrors(" Memory Allocation ");

	CUDABasisFunctionEvaluation(ku, nu, uNum, this->evalParams->uMin, this->evalParams->uMax, this->uKnotVector, uCtrlData, this->evalParams->uBasisFuncCUDAData, this->evalParams->uBasisDervCUDAData);

#ifdef DUMP
	float* tempData = new float[(uNum+1)*ku];
	cudaMemcpy(tempData, uBasisGPUData, (uNum+1)*ku*sizeof(float), cudaMemcpyDeviceToHost);
	WriteToFile(tempData,ku,(uNum+1));
	delete [] tempData;
#endif
	
	CUDABasisFunctionEvaluation(kv, nv, vNum, this->evalParams->vMin, this->evalParams->vMax, this->vKnotVector, vCtrlData, this->evalParams->vBasisFuncCUDAData, this->evalParams->vBasisDervCUDAData);


	// Allocate array and copy Texture data
    cudaChannelFormatDesc float4ChannelDesc = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
    cudaChannelFormatDesc float1ChannelDesc	= cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindFloat);
    cudaChannelFormatDesc int1ChannelDesc	= cudaCreateChannelDesc(32, 0, 0, 0, cudaChannelFormatKindSigned);

    cudaArray* uBasisGPUArray;
    cudaArray* vBasisGPUArray;
    cudaArray* ctrlPointsCUDAArray;
    cudaArray* uCtrlPointIndexCUDAArray;
    cudaArray* vCtrlPointIndexCUDAArray;

	cudaMallocArray(&ctrlPointsCUDAArray,		&float4ChannelDesc, nu, nv); 
    cudaMallocArray(&uBasisGPUArray,			&float1ChannelDesc, ku, uNum+1); 
    cudaMallocArray(&vBasisGPUArray,			&float1ChannelDesc, kv, vNum+1); 
	cudaMallocArray(&uCtrlPointIndexCUDAArray,	&int1ChannelDesc,	uNum+1, 1); 
    cudaMallocArray(&vCtrlPointIndexCUDAArray,	&int1ChannelDesc,	vNum+1, 1); 
 	CUDACheckErrors(" Memory Allocation ");

	// Copy data to device
	cudaMemcpyToArray(ctrlPointsCUDAArray, 0, 0, this->cntlPoints, nu*nv*4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpyToArray(uCtrlPointIndexCUDAArray, 0, 0, uCtrlData, (uNum+1)*sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpyToArray(vCtrlPointIndexCUDAArray, 0, 0, vCtrlData, (vNum+1)*sizeof(int), cudaMemcpyHostToDevice);

	// Copy basis data internally to array
	cudaMemcpyToArray(uBasisGPUArray, 0, 0, this->evalParams->uBasisFuncCUDAData, (uNum+1)*ku*sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemcpyToArray(vBasisGPUArray, 0, 0, this->evalParams->vBasisFuncCUDAData, (vNum+1)*kv*sizeof(float), cudaMemcpyDeviceToDevice);

	// Copy basis index data from CUDA array to GPU memory location for exact normal evaluation
	if (exactNormals)
	{
		cudaMalloc((void**)&(this->evalParams->ctrlPointsCUDAData), nu*nv*4*sizeof(float));
		cudaMalloc((void**)&(this->evalParams->uCtrlPointIndexCUDAData), (uNum+1)*sizeof(int));
		cudaMalloc((void**)&(this->evalParams->vCtrlPointIndexCUDAData), (vNum+1)*sizeof(int));

		cudaMemcpyFromArray(this->evalParams->ctrlPointsCUDAData, ctrlPointsCUDAArray, 0, 0, nu*nv*4*sizeof(float), cudaMemcpyDeviceToDevice);
		cudaMemcpyFromArray(this->evalParams->uCtrlPointIndexCUDAData, uCtrlPointIndexCUDAArray, 0, 0, (uNum+1)*sizeof(int), cudaMemcpyDeviceToDevice);
		cudaMemcpyFromArray(this->evalParams->vCtrlPointIndexCUDAData, vCtrlPointIndexCUDAArray, 0, 0, (vNum+1)*sizeof(int), cudaMemcpyDeviceToDevice);
	}
	CUDACheckErrors(" Copy Data ");

	delete[] uCtrlData;
	delete[] vCtrlData;

	// map OpenGL buffer object for writing from CUDA
	if (rendering)
		cudaGLMapBufferObject((void**)&(this->evalParams->evaluatedCUDAPoints), this->ptVBO);
	else
		cudaMalloc((void**)&(this->evalParams->evaluatedCUDAPoints), (uNum+1)*(vNum+1)*sizeof(float4));

	CUDABasisCtrlPointMultiplicationTextures(ctrlPointsCUDAArray, uCtrlPointIndexCUDAArray, vCtrlPointIndexCUDAArray, uBasisGPUArray, vBasisGPUArray,  this->evalParams->evaluatedCUDAPoints);

	if (rendering)
		cudaGLUnmapBufferObject(this->ptVBO);

	CUDACheckErrors(" End Evaluation ");

	if (readBack && this->evalParams->evaluatedPoints!=NULL && this->identityTransformation)
		cudaMemcpy(this->evalParams->evaluatedPoints, this->evalParams->evaluatedCUDAPoints, (uNum+1)*(vNum+1)*4*sizeof(float), cudaMemcpyDeviceToHost);

	if (rendering)
	{
		if (exactNormals)
			CUDAExactNormalEvaluation(true, false);//, pointVBOResource, normalVBOResource);
		else
			CUDANormalEvaluation(true);//, pointVBOResource, normalVBOResource);
	}
	cudaFreeArray(uBasisGPUArray);
	cudaFreeArray(vBasisGPUArray);
	cudaFreeArray(ctrlPointsCUDAArray);
	cudaFreeArray(uCtrlPointIndexCUDAArray);
	cudaFreeArray(vCtrlPointIndexCUDAArray);
	return 0;
}
#endif
#endif
