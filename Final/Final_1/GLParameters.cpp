#include "GLParameters.h"
#include "Utilities.h"

GLParameters::GLParameters()
{
	this->initialized			= false;
	this->smooth				= true;
	this->wireframe				= false;
	this->controlMesh			= false;
	this->picked				= false;
	this->drawingON				= false;
	this->intersection			= false;
	this->collision				= false;
	this->silhouette			= false;
	this->selfIntersection		= false;
	this->inverseEvaluation		= false;
	this->closestPoint			= false;
	this->hausdorff				= false;
	this->curvatureEvaluation	= false;
	this->computingMoment		= false;
	this->displayNURBS			= false;
	
	this->exactNormals				= true;
	this->drawBBox					= false;
	this->expandBoundingBox			= true;
	this->variableExpansionFactor	= false;
	this->readBack					= false;
	this->enhancedAccuracy			= false;
	this->drawVoxels				= true;
	this->closestPointComputed		= false;

	this->surfVisibilityCutOff	= 0;
	this->objVisibilityCutOff	= 0;
	this->pickedSurfNum			= -1;
	this->pickedControlPointNum = -1;
	this->pickedObjectNum		= -1;
	this->pickedFaceNum			= -1;
	this->offsetDistance		= 0;
	this->voxelCount			= 10;
	this->displayLevel			= 0;
}

GLParameters::~GLParameters()
{
	//Delete Textures and FBO
	if (!this->initialized)
		return;

	glDeleteFramebuffersEXT(1, &this->fbo);
	glDeleteRenderbuffersEXT(1, &this->depthBuffer);
	glDeleteRenderbuffersEXT(1, &this->stencilBuffer);
	glDeleteQueries(1, &this->occlusionQuery);

	cgDestroyProgram(this->trimProgram);
	cgDestroyProgram(this->surfaceMultProgram);
	cgDestroyProgram(this->evaluateBasisProgram);
	cgDestroyProgram(this->normalProgram);
	cgDestroyProgram(this->passiveProgram);
	cgDestroyProgram(this->boundingBoxProgram);
	cgDestroyProgram(this->invertProgram);
	cgDestroyProgram(this->bBoxOverlapProgram);
	cgDestroyProgram(this->bBoxHierarchyProgram);
	cgDestroyProgram(this->evaluateBasisDervProgram);
	cgDestroyProgram(this->exactNormalProgram);
	cgDestroyProgram(this->silhouetteProgram);
	cgDestroyProgram(this->closestPointProgram);
	cgDestroyProgram(this->curvatureProgram);

	cgDestroyProgram(this->passiveSurfaceIDProgram);
	cgDestroyProgram(this->trimSurfaceIDProgram);

#ifdef OCCLUSIONFIND
	cgDestroyProgram(this->occlusionProgram);
	cgDestroyProgram(this->findPrepassProgram);
#endif

#ifdef GPUFIND
	cgDestroyProgram(this->findPrepassProgram);
#ifdef GPUFIND1
	cgDestroyProgram(this->findProgram);
	cgDestroyProgram(this->findReduceProgram);
#endif
#ifdef GPUFIND2
	cgDestroyProgram(this->findProgram);
	cgDestroyProgram(this->findReduceProgram);
#endif
#ifdef GPUFIND3
	cgDestroyProgram(this->findDownSweepProgram);
	cgDestroyProgram(this->findUpSweepProgram);
	cgDestroyProgram(this->findPosChangeProgram);
	cgDestroyProgram(this->findVertexProgram);
	cgDestroyProgram(this->findScaleProgram);
#endif
	cgDestroyProgram(this->referenceProgram);
	cgDestroyProgram(this->intervalProgram);
#endif

#ifdef EVALUATECURVE
	cgDestroyProgram(this->basisMultProgram);
	cgDestroyProgram(this->reductionProgram);
#endif
}

void GLParameters::InitializeCG()
{
	this->cgContext = cgCreateContext();

	// get the best profile for this hardware
//	this->cgFragmentProfile = CG_PROFILE_ARBFP1;
	this->cgFragmentProfile = cgGLGetLatestProfile(CG_GL_FRAGMENT);
	cgGLSetOptimalOptions(this->cgFragmentProfile);

	this->cgVertexProfile = cgGLGetLatestProfile(CG_GL_VERTEX);
	cgGLSetOptimalOptions(this->cgVertexProfile); 

	this->InitializeTrimProgram();
	this->InitializePassiveProgram();
	this->InitializeSurfaceEvaluationProgram();
	this->InitializeNormalProgram();
	this->InitializeBoundingBoxProgram();
	this->InitializeInvertProgram();
	this->InitializeIntersectionPrograms();
	this->InitializeReferencePrograms();
	this->InitializeFindPrograms();
	this->InitializeSilhouetteProgram();
	this->InitializeClosestPointProgram();
	this->InitializeSurfaceIDProgram();
	this->InitializeCurvatureProgram();
	this->InitializeMomentProgram();

#ifdef GPUCOLLISION
	this->InitializeOrientedBoundingBoxProgram();
#endif

#ifdef EVALUATECURVE
	this->InitializeCurveEvaluationProgram();
#endif

	//iddo:
	this->InitializeHausdorffIddoProgram();

	//Generate the Frame Buffer Object
	glGenFramebuffersEXT(1, &this->fbo);
	glGenRenderbuffersEXT(1, &this->depthBuffer);
	glGenRenderbuffersEXT(1, &this->stencilBuffer);
	glGenQueriesARB(1,&this->occlusionQuery);

	this->initialized = true;
}

void GLParameters::InitializeNormalProgram()
{
	this->normalProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/normal.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->normalProgram != NULL)
	{
		cgGLLoadProgram(this->normalProgram);
		this->normalTextureParam = cgGetNamedParameter(this->normalProgram,"texture");
		this->normalDimParam = cgGetNamedParameter(this->normalProgram,"dim");
	}
}

void GLParameters::InitializeReferencePrograms()
{
#ifdef GPUFIND
	this->referenceProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/referenceList.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->referenceProgram != NULL)
	{
		cgGLLoadProgram(this->referenceProgram);
		this->referenceOverlapTexParam	= cgGetNamedParameter(this->referenceProgram,"overlap");
		this->referenceOldListTexParam	= cgGetNamedParameter(this->referenceProgram,"oldList");
		this->referenceSizeDataParam	= cgGetNamedParameter(this->referenceProgram,"sizeData");
	}
#ifndef NVIDIA
	const char* args[] = {"-po", "ATI_draw_buffers", 0};
#else
	const char* args[] = {0};
#endif
	this->intervalProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ComputeInterval.cg",this->cgFragmentProfile, NULL, args);
	if(this->intervalProgram != NULL)
	{
		cgGLLoadProgram(this->intervalProgram);
		this->intervalRefListTexParam	= cgGetNamedParameter(this->intervalProgram,"rlist");
		this->intervalMinMax1			= cgGetNamedParameter(this->intervalProgram,"minmax1");
		this->intervalMinMax2			= cgGetNamedParameter(this->intervalProgram,"minmax2");
		this->intervalUVNum				= cgGetNamedParameter(this->intervalProgram,"uvnum");
	}
#endif
}

void GLParameters::InitializeTrimProgram()
{
	//Setup the trim fragment program
	this->trimProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Stencil.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->trimProgram != NULL)
	{
		cgGLLoadProgram(this->trimProgram);
		this->trimTextureParam	= cgGetNamedParameter(this->trimProgram, "trimTexture");
	}

	this->trimOverlayProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/TrimOverlay.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->trimOverlayProgram != NULL)
	{
		cgGLLoadProgram(this->trimOverlayProgram);
		this->trimOverlayTrimTextureParam	= cgGetNamedParameter(this->trimOverlayProgram, "trimTexture");
		this->trimOverlayTextureParam		= cgGetNamedParameter(this->trimOverlayProgram, "texture");
	}
}

void GLParameters::InitializePassiveProgram()
{
	this->passiveProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Passive.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->passiveProgram != NULL)
		cgGLLoadProgram(this->passiveProgram);

	this->passiveTextureProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/PassiveTexture.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->passiveTextureProgram != NULL)
	{
		cgGLLoadProgram(this->passiveTextureProgram);
		this->passiveTextureParam	= cgGetNamedParameter(this->passiveTextureProgram, "texture");
	}
}

void GLParameters::InitializeSurfaceIDProgram()
{
	//Setup the trim fragment program
	this->trimSurfaceIDProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/TrimSurfaceID.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->trimSurfaceIDProgram != NULL)
	{
		cgGLLoadProgram(this->trimSurfaceIDProgram);
		this->trimSurfaceIDTextureParam = cgGetNamedParameter(this->trimSurfaceIDProgram, "trimTexture");
		this->trimSurfaceIDParam = cgGetNamedParameter(this->trimSurfaceIDProgram, "surfaceID");
	}

	this->passiveSurfaceIDProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/PassiveSurfaceID.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->passiveSurfaceIDProgram != NULL)
	{
		cgGLLoadProgram(this->passiveSurfaceIDProgram);
		this->passiveSurfaceIDParam = cgGetNamedParameter(this->passiveSurfaceIDProgram, "surfaceID");
	}
}

void GLParameters::InitializeSurfaceEvaluationProgram()
{
	//Initialize texture
	this->curveTexture = new GLuint[7];
	this->basisDervTexture = new GLuint[2];
	//Setup the basis function program
#ifdef PACKED
	this->evaluateBasisProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BEvaluatePacked.cg",this->cgFragmentProfile, NULL, NULL);
#else
	this->evaluateBasisProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BEvaluate.cg",this->cgFragmentProfile, NULL, NULL);
#endif
	if(this->evaluateBasisProgram != NULL)
	{
		cgGLLoadProgram(this->evaluateBasisProgram);
		this->knotDataTextureParam	= cgGetNamedParameter(this->evaluateBasisProgram, "knot");
		this->basisTextureParam		= cgGetNamedParameter(this->evaluateBasisProgram, "basis");
		this->passParam				= cgGetNamedParameter(this->evaluateBasisProgram, "k");
	}

	//Setup the basis function Derivative program
#ifdef PACKED
	this->evaluateBasisDervProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BDervEvaluatePacked.cg",this->cgFragmentProfile, NULL, NULL);
#else
	this->evaluateBasisDervProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BDervEvaluate.cg",this->cgFragmentProfile, NULL, NULL);
#endif
	if(this->evaluateBasisDervProgram != NULL)
	{
		cgGLLoadProgram(this->evaluateBasisDervProgram);
		this->knotDataDervTextureParam	= cgGetNamedParameter(this->evaluateBasisDervProgram, "knot");
		this->basisDervTextureParam		= cgGetNamedParameter(this->evaluateBasisDervProgram, "basis");
		this->passDervParam				= cgGetNamedParameter(this->evaluateBasisDervProgram, "k");
	}

#ifdef NOPINGPONG
	//Setup the basis function multiplication program
	this->surfaceMultProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/SurfaceMultNoPingPong.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->surfaceMultProgram != NULL)
	{
		cgGLLoadProgram(this->surfaceMultProgram);
		this->surfaceMultCtrlPtsTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "ctrlPts");
		this->surfaceMultuCtrlIndexTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "uCtrlIndex");
		this->surfaceMultvCtrlIndexTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "vCtrlIndex");
		this->surfaceMultuBasisFuncTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "uBasisFunc");
		this->surfaceMultvBasisFuncTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "vBasisFunc");
		this->surfaceMultkuParam				= cgGetNamedParameter(this->surfaceMultProgram, "ku");
		this->surfaceMultkvParam				= cgGetNamedParameter(this->surfaceMultProgram, "kv");
	}
#else
	this->surfaceMultProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/SurfaceMultPacked.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->surfaceMultProgram != NULL)
	{
		cgGLLoadProgram(this->surfaceMultProgram);
		this->surfaceMultCtrlPtsTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "ctrlPts");
		this->surfaceMultuCtrlIndexTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "uCtrlIndex");
		this->surfaceMultvCtrlIndexTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "vCtrlIndex");
		this->surfaceMultuBasisFuncTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "uBasisFunc");
		this->surfaceMultvBasisFuncTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "vBasisFunc");
		this->surfaceMultSurfPtSrcTextureParam	= cgGetNamedParameter(this->surfaceMultProgram, "surfPtSrc");
		this->surfaceMultuLenParam				= cgGetNamedParameter(this->surfaceMultProgram, "uLen");
		this->surfaceMultvLenParam				= cgGetNamedParameter(this->surfaceMultProgram, "vLen");
		this->surfaceMultuPassParam				= cgGetNamedParameter(this->surfaceMultProgram, "uPass");
		this->surfaceMultvPassParam				= cgGetNamedParameter(this->surfaceMultProgram, "vPass");
	}
#endif
	//Setup the normal evaluation program
	this->exactNormalProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ExactNormal.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->exactNormalProgram != NULL)
	{
		cgGLLoadProgram(this->exactNormalProgram);
		this->exactNormaluDervTextureParam	= cgGetNamedParameter(this->exactNormalProgram, "uDervTex");
		this->exactNormalvDervTextureParam	= cgGetNamedParameter(this->exactNormalProgram, "vDervTex");
	}

	this->rationalProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/RationalDerivatives.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->rationalProgram != NULL)
	{
		cgGLLoadProgram(this->rationalProgram);
		this->rationalDervTextureParam		= cgGetNamedParameter(this->rationalProgram, "dervTexture");
		this->rationalSurfaceTextureParam	= cgGetNamedParameter(this->rationalProgram, "surfaceTex");
	}
}

void GLParameters::InitializeCurvatureProgram()
{
	//Setup curvature evaluation
	this->curvatureProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Curvature.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->curvatureProgram != NULL)
	{
		cgGLLoadProgram(this->curvatureProgram);
		this->curvatureuDervTextureParam		= cgGetNamedParameter(this->curvatureProgram, "uDervTex");
		this->curvaturevDervTextureParam		= cgGetNamedParameter(this->curvatureProgram, "vDervTex");
		this->curvatureuuSecondDervTextureParam	= cgGetNamedParameter(this->curvatureProgram, "uuSecondDervTex");
		this->curvatureuvSecondDervTextureParam	= cgGetNamedParameter(this->curvatureProgram, "uvSecondDervTex");
		this->curvaturevvSecondDervTextureParam	= cgGetNamedParameter(this->curvatureProgram, "vvSecondDervTex");
	}
}

void GLParameters::InitializeMomentProgram()
{
	#ifndef NVIDIA
		const char* args[] = {"-po", "ATI_draw_buffers", 0};
	#else
		const char* args[] = {0};
	#endif

	//Setup moment evaluation
	this->moment2PointProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Moment2Point.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->moment2PointProgram != NULL)
	{
		cgGLLoadProgram(this->moment2PointProgram);
		this->moment2PointSurfaceTextureParam	= cgGetNamedParameter(this->moment2PointProgram, "surfaceTex");
		this->moment2PointNormalTextureParam	= cgGetNamedParameter(this->moment2PointProgram, "normalTex");
		this->moment2PointMultConstantParam		= cgGetNamedParameter(this->moment2PointProgram, "dS");
	}

	this->moment3PointProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Moment3Point.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->moment3PointProgram != NULL)
	{
		cgGLLoadProgram(this->moment3PointProgram);
		this->moment3PointSurfaceTextureParam	= cgGetNamedParameter(this->moment3PointProgram, "surfaceTex");
		this->moment3PointNormalTextureParam	= cgGetNamedParameter(this->moment3PointProgram, "normalTex");
		this->moment3PointMultConstantParam		= cgGetNamedParameter(this->moment3PointProgram, "dS");
	}

	this->reduceSum4Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceSum4.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceSum4Program != NULL)
	{
		cgGLLoadProgram(this->reduceSum4Program);
		this->reduceSum4InputTextureParam = cgGetNamedParameter(this->reduceSum4Program, "inputTex");
	}

	this->reduceSum9Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceSum9.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceSum9Program != NULL)
	{
		cgGLLoadProgram(this->reduceSum9Program);
		this->reduceSum9InputTextureParam	= cgGetNamedParameter(this->reduceSum9Program, "inputTex");
	}

	this->reduceSum2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceSum2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceSum2Program != NULL)
	{
		cgGLLoadProgram(this->reduceSum2Program);
		this->reduceSum2InputTextureParam	= cgGetNamedParameter(this->reduceSum2Program, "inputTex");
		this->reduceSum2OffsetParam			= cgGetNamedParameter(this->reduceSum2Program, "offset");
	}

	this->reduceSum2Pow2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceSum2Pow2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceSum2Pow2Program != NULL)
	{
		cgGLLoadProgram(this->reduceSum2Pow2Program);
		this->reduceSum2Pow2InputTextureParam	= cgGetNamedParameter(this->reduceSum2Pow2Program, "inputTex");
		this->reduceSum2Pow2OffsetParam			= cgGetNamedParameter(this->reduceSum2Pow2Program, "offset");
	}

	this->momentTrimMultProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/MomentTrimMult.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->momentTrimMultProgram != NULL)
	{
		cgGLLoadProgram(this->momentTrimMultProgram);
		this->momentTrimMultMomentTextureParam		= cgGetNamedParameter(this->momentTrimMultProgram, "momentTex");
		this->momentTrimMultTrimTextureParam		= cgGetNamedParameter(this->momentTrimMultProgram, "trimTex");
	}

	this->secondDerv2PointProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/SecondDerivatives2Point.cg",this->cgFragmentProfile, NULL, args);
	if(this->secondDerv2PointProgram != NULL)
	{
		cgGLLoadProgram(this->secondDerv2PointProgram);
		this->secondDerv2PointTextureParam		= cgGetNamedParameter(this->secondDerv2PointProgram,"texture");
		this->secondDerv2PointUVIntervalParam	= cgGetNamedParameter(this->secondDerv2PointProgram,"uvInterval");
	}

	this->secondDerv3PointProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/SecondDerivatives3Point.cg",this->cgFragmentProfile, NULL, args);
	if(this->secondDerv3PointProgram != NULL)
	{
		cgGLLoadProgram(this->secondDerv3PointProgram);
		this->secondDerv3PointTextureParam		= cgGetNamedParameter(this->secondDerv3PointProgram,"texture");
		this->secondDerv3PointUVIntervalParam	= cgGetNamedParameter(this->secondDerv3PointProgram,"uvInterval");
	}

	this->momentErrorProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/MomentError.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->momentErrorProgram != NULL)
	{
		cgGLLoadProgram(this->momentErrorProgram);
		this->momentErrorSurfaceTextureParam	= cgGetNamedParameter(this->momentErrorProgram, "surfaceTex");
		this->momentErrorExpansionTextureParam	= cgGetNamedParameter(this->momentErrorProgram, "expansionTex");
		this->momentErrorMomentTextureParam		= cgGetNamedParameter(this->momentErrorProgram, "momentTex");
		this->momentErrorTrimTextureParam		= cgGetNamedParameter(this->momentErrorProgram, "trimTex");
		this->momentErrorTrimPointsParam		= cgGetNamedParameter(this->momentErrorProgram, "trimPoints");
	}

	this->momentTriangleProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/MomentTriangle.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->momentTriangleProgram != NULL)
	{
		cgGLLoadProgram(this->momentTriangleProgram);
		this->momentTriangleVertex1TextureParam	= cgGetNamedParameter(this->momentTriangleProgram, "vertex1Tex");
		this->momentTriangleVertex2TextureParam	= cgGetNamedParameter(this->momentTriangleProgram, "vertex2Tex");
		this->momentTriangleVertex3TextureParam	= cgGetNamedParameter(this->momentTriangleProgram, "vertex3Tex");
	}
}

#ifdef GPUCOLLISION
void GLParameters::InitializeOrientedBoundingBoxProgram()
{
	this->oBBoxOverlapProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/OBBOverlap.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->oBBoxOverlapProgram != NULL)
	{
		cgGLLoadProgram(this->oBBoxOverlapProgram);
		this->oBBoxMin1TextureParam = cgGetNamedParameter(this->oBBoxOverlapProgram, "bBoxMinA");
		this->oBBoxMax1TextureParam = cgGetNamedParameter(this->oBBoxOverlapProgram, "bBoxMaxA");
		this->oBBoxMin2TextureParam = cgGetNamedParameter(this->oBBoxOverlapProgram, "bBoxMinB");
		this->oBBoxMax2TextureParam = cgGetNamedParameter(this->oBBoxOverlapProgram, "bBoxMaxB");
		this->oBBoxTransformParam	= cgGetNamedParameter(this->oBBoxOverlapProgram, "R");
		this->oBBoxTranslateParam	= cgGetNamedParameter(this->oBBoxOverlapProgram, "T");
	}
}
#endif

void GLParameters::InitializeBoundingBoxProgram()
{
	//Setup the bounding box generation program
#ifndef NVIDIA
	const char* args[] = {"-po", "ATI_draw_buffers", 0};
#else
	const char* args[] = {0};
#endif
	if (this->variableExpansionFactor)
		this->boundingBoxProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BoundingBoxVariable.cg",this->cgFragmentProfile, NULL, args);
	else
		this->boundingBoxProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BoundingBoxConstant.cg",this->cgFragmentProfile, NULL, args);
	if(this->boundingBoxProgram != NULL)
	{
		cgGLLoadProgram(this->boundingBoxProgram);
		this->boundingBoxSurfaceTextureParam	= cgGetNamedParameter(this->boundingBoxProgram,"surface");
		this->boundingBoxModelViewParam			= cgGetNamedParameter(this->boundingBoxProgram,"viewmatrix");
		if (this->variableExpansionFactor)
			this->boundingBoxFactorTextureParam = cgGetNamedParameter(this->boundingBoxProgram,"magFactor");
		else
			this->boundingBoxFactorParam		= cgGetNamedParameter(this->boundingBoxProgram,"magFactor");
	}

	if (this->variableExpansionFactor)
		this->boundingBoxTrimmedProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BoundingBoxTrimmedVariable.cg",this->cgFragmentProfile, NULL, args);
	else
		this->boundingBoxTrimmedProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BoundingBoxTrimmedConstant.cg",this->cgFragmentProfile, NULL, args);
	if(this->boundingBoxTrimmedProgram != NULL)
	{
		cgGLLoadProgram(this->boundingBoxTrimmedProgram);
		this->boundingBoxTrimmedSurfaceTextureParam	= cgGetNamedParameter(this->boundingBoxTrimmedProgram,"surface");
		this->boundingBoxTrimmedTrimTextureParam	= cgGetNamedParameter(this->boundingBoxTrimmedProgram,"trimTex");
		this->boundingBoxTrimmedModelViewParam		= cgGetNamedParameter(this->boundingBoxTrimmedProgram,"viewmatrix");
		if (this->variableExpansionFactor)
			this->boundingBoxTrimmedFactorTextureParam = cgGetNamedParameter(this->boundingBoxTrimmedProgram,"magFactor");
		else
			this->boundingBoxTrimmedFactorParam		= cgGetNamedParameter(this->boundingBoxTrimmedProgram,"magFactor");
	}

	this->bBoxHierarchyProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxHierarchy.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxHierarchyProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxHierarchyProgram);
		this->bBoxHierarchyTextureParam	= cgGetNamedParameter(this->bBoxHierarchyProgram,"texture");
		this->bBoxHierarchyOriginParam	= cgGetNamedParameter(this->bBoxHierarchyProgram,"origin");
		this->bBoxHierarchyIsMaxParam	= cgGetNamedParameter(this->bBoxHierarchyProgram,"isBBoxMax");
	}

	this->bBoxHierarchyTrimmedMaxProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxHierarchyTrimmedMax.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxHierarchyTrimmedMaxProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxHierarchyTrimmedMaxProgram);
		this->bBoxHierarchyTrimmedMaxTextureParam	= cgGetNamedParameter(this->bBoxHierarchyTrimmedMaxProgram,"texture");
		this->bBoxHierarchyTrimmedMaxOriginParam	= cgGetNamedParameter(this->bBoxHierarchyTrimmedMaxProgram,"origin");
	}

	this->bBoxHierarchyTrimmedMinProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxHierarchyTrimmedMin.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxHierarchyTrimmedMinProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxHierarchyTrimmedMinProgram);
		this->bBoxHierarchyTrimmedMinTextureParam	= cgGetNamedParameter(this->bBoxHierarchyTrimmedMinProgram,"texture");
		this->bBoxHierarchyTrimmedMinOriginParam	= cgGetNamedParameter(this->bBoxHierarchyTrimmedMinProgram,"origin");
	}

	this->secondDervProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/SecondDerivatives.cg",this->cgFragmentProfile, NULL, args);
	if(this->secondDervProgram != NULL)
	{
		cgGLLoadProgram(this->secondDervProgram);
		this->secondDervTextureParam	= cgGetNamedParameter(this->secondDervProgram,"texture");
		this->secondDervUVIntervalParam	= cgGetNamedParameter(this->secondDervProgram,"uvinterval");
		this->secondDervDimensionParam	= cgGetNamedParameter(this->secondDervProgram,"dim");
	}

	this->findMaxProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/FindMax.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->findMaxProgram != NULL)
	{
		cgGLLoadProgram(this->findMaxProgram);
		this->findMaxTextureParam	= cgGetNamedParameter(this->findMaxProgram, "inputTex");
		this->findMaxFactorParam	= cgGetNamedParameter(this->findMaxProgram, "multFactor");
	}

	this->reduceMaxProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMax.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMaxProgram != NULL)
	{
		cgGLLoadProgram(this->reduceMaxProgram);
		this->reduceMaxTextureParam= cgGetNamedParameter(this->reduceMaxProgram, "texture");
	}

	this->mergeExpansionFactorsProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/MergeFactors.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->mergeExpansionFactorsProgram != NULL)
	{
		cgGLLoadProgram(this->mergeExpansionFactorsProgram);
		this->mergeFactorTex1TextureParam	= cgGetNamedParameter(this->mergeExpansionFactorsProgram, "tex1");
		this->mergeFactorTex2TextureParam	= cgGetNamedParameter(this->mergeExpansionFactorsProgram, "tex2");
		this->mergeFactorTex3TextureParam	= cgGetNamedParameter(this->mergeExpansionFactorsProgram, "tex3");
	}

	this->transformSurfaceProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/TransformSurface.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->transformSurfaceProgram != NULL)
	{
		cgGLLoadProgram(this->transformSurfaceProgram);
		this->transformSurfaceTextureParam	= cgGetNamedParameter(this->transformSurfaceProgram, "surface");
		this->transformSurfaceMatrixParam	= cgGetNamedParameter(this->transformSurfaceProgram, "matrix");
	}

}

void GLParameters::InitializeInvertProgram()
{
	this->invertProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Invert.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->invertProgram != NULL)
	{
		cgGLLoadProgram(this->invertProgram);
		this->invertBBoxMinTextureParam	= cgGetNamedParameter(this->invertProgram, "minTexture");
		this->invertBBoxMaxTextureParam = cgGetNamedParameter(this->invertProgram, "maxTexture");
		this->invertPtParam				= cgGetNamedParameter(this->invertProgram, "point");
	}

	this->invertRayProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/InvertRay.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->invertRayProgram != NULL)
	{
		cgGLLoadProgram(this->invertRayProgram);
		this->invertRayBBoxMinTextureParam	= cgGetNamedParameter(this->invertRayProgram, "minTexture");
		this->invertRayBBoxMaxTextureParam  = cgGetNamedParameter(this->invertRayProgram, "maxTexture");
		this->invertRayPtParam				= cgGetNamedParameter(this->invertRayProgram, "point");
	}
}

void GLParameters::InitializeClosestPointProgram()
{
	this->closestPointProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ClosestPoint.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->closestPointProgram != NULL)
	{
		cgGLLoadProgram(this->closestPointProgram);
		this->closestPointBBoxMinTextureParam	= cgGetNamedParameter(this->closestPointProgram,"minTexture");
		this->closestPointBBoxMaxTextureParam	= cgGetNamedParameter(this->closestPointProgram,"maxTexture");
		this->closestPointPtParam				= cgGetNamedParameter(this->closestPointProgram,"point");
	}

	this->reduceMinProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMin.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMinProgram != NULL)
	{
		cgGLLoadProgram(this->reduceMinProgram);
		this->reduceMinTextureParam= cgGetNamedParameter(this->reduceMinProgram, "texture");
	}

	this->bBoxDistProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxDistance.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxDistProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxDistProgram);
		this->bBoxDistMaxATextureParam			= cgGetNamedParameter(this->bBoxDistProgram, "bBoxMaxA");
		this->bBoxDistMinATextureParam			= cgGetNamedParameter(this->bBoxDistProgram, "bBoxMinA");
		this->bBoxDistMaxBTextureParam			= cgGetNamedParameter(this->bBoxDistProgram, "bBoxMaxB");
		this->bBoxDistMinBTextureParam			= cgGetNamedParameter(this->bBoxDistProgram, "bBoxMinB");
		this->bBoxDistReferenceListTextureParam	= cgGetNamedParameter(this->bBoxDistProgram, "referenceList");
		this->bBoxDistLevelOriginParam			= cgGetNamedParameter(this->bBoxDistProgram, "levelOrigin");
	}

	this->bBoxHausdorffDistProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxHausdorffDistance.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxHausdorffDistProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxHausdorffDistProgram);
		this->bBoxHausdorffDistMaxATextureParam				= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "bBoxMaxA");
		this->bBoxHausdorffDistMinATextureParam				= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "bBoxMinA");
		this->bBoxHausdorffDistMaxBTextureParam				= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "bBoxMaxB");
		this->bBoxHausdorffDistMinBTextureParam				= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "bBoxMinB");
		this->bBoxHausdorffDistReferenceListTextureParam	= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "referenceList");
		this->bBoxHausdorffDistRowNumberTextureParam		= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "rowNumbers");
		this->bBoxHausdorffDistLevelOriginParam				= cgGetNamedParameter(this->bBoxHausdorffDistProgram, "levelOrigin");
	}

	this->reduceMinBBox2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMinBBox2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMinBBox2Program != NULL)
	{
		cgGLLoadProgram(this->reduceMinBBox2Program);
		this->reduceMinBBox2TextureParam	= cgGetNamedParameter(this->reduceMinBBox2Program, "inputTex");
		this->reduceMinBBox2OffsetParam		= cgGetNamedParameter(this->reduceMinBBox2Program, "offset");
	}

	this->reduceMinMinBBox2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMinMinBBox2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMinMinBBox2Program != NULL)
	{
		cgGLLoadProgram(this->reduceMinMinBBox2Program);
		this->reduceMinMinBBox2TextureParam	= cgGetNamedParameter(this->reduceMinMinBBox2Program, "inputTex");
		this->reduceMinMinBBox2OffsetParam		= cgGetNamedParameter(this->reduceMinMinBBox2Program, "offset");
	}

	this->reduceMinMaxBBox2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMinMaxBBox2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMinMaxBBox2Program != NULL)
	{
		cgGLLoadProgram(this->reduceMinMaxBBox2Program);
		this->reduceMinMaxBBox2TextureParam	= cgGetNamedParameter(this->reduceMinMaxBBox2Program, "inputTex");
		this->reduceMinMaxBBox2OffsetParam		= cgGetNamedParameter(this->reduceMinMaxBBox2Program, "offset");
	}
/*
	this->reduceMaxBBox2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMaxBBox2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMaxBBox2Program != NULL)
	{
		cgGLLoadProgram(this->reduceMaxBBox2Program);
		this->reduceMaxBBox2TextureParam	= cgGetNamedParameter(this->reduceMaxBBox2Program, "inputTex");
		this->reduceMaxBBox2OffsetParam		= cgGetNamedParameter(this->reduceMaxBBox2Program, "offset");
	}
*/
	this->reduceMinBBox4Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ReduceMinBBox4.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->reduceMinBBox4Program != NULL)
	{
		cgGLLoadProgram(this->reduceMinBBox4Program);
		this->reduceMinBBox4TextureParam	= cgGetNamedParameter(this->reduceMinBBox4Program, "inputTex");
	}

	this->bBoxDistFilterProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxDistFilter.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxDistFilterProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxDistFilterProgram);
		this->bBoxDistFilterTextureParam	= cgGetNamedParameter(this->bBoxDistFilterProgram, "texture");
		this->bBoxDistFilterMaxValParam		= cgGetNamedParameter(this->bBoxDistFilterProgram, "maxVal");
	}

	this->bBoxHausdorffFilter1Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxHausdorffFilter1.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxHausdorffFilter1Program != NULL)
	{
		cgGLLoadProgram(this->bBoxHausdorffFilter1Program);
		this->bBoxHausdorffFilter1MinMaxTextureParam	= cgGetNamedParameter(this->bBoxHausdorffFilter1Program, "minMaxTexture");
		this->bBoxHausdorffFilter1MinValParam			= cgGetNamedParameter(this->bBoxHausdorffFilter1Program, "minVal");
	}

	this->bBoxHausdorffFilter2Program = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxHausdorffFilter2.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxHausdorffFilter2Program != NULL)
	{
		cgGLLoadProgram(this->bBoxHausdorffFilter2Program);
		this->bBoxHausdorffFilter2MinMaxTextureParam	= cgGetNamedParameter(this->bBoxHausdorffFilter2Program, "minMaxTexture");
		this->bBoxHausdorffFilter2MinRowTextureParam	= cgGetNamedParameter(this->bBoxHausdorffFilter2Program, "minMaxRowTexture");
		this->bBoxHausdorffFilter2FilterRowTextureParam	= cgGetNamedParameter(this->bBoxHausdorffFilter2Program, "filterRowTexture");
		this->bBoxHausdorffFilter2TexSizeParam			= cgGetNamedParameter(this->bBoxHausdorffFilter2Program, "texSize");
	}

	this->objectBBoxDistanceProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/ObjectBBoxDistance.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxDistProgram != NULL)
	{
		cgGLLoadProgram(this->objectBBoxDistanceProgram);
		this->objectBBoxDistanceMinATextureParam	= cgGetNamedParameter(this->objectBBoxDistanceProgram, "bBoxMinA");
		this->objectBBoxDistanceMaxATextureParam	= cgGetNamedParameter(this->objectBBoxDistanceProgram, "bBoxMaxA");
		this->objectBBoxDistanceMinBTextureParam	= cgGetNamedParameter(this->objectBBoxDistanceProgram, "bBoxMinB");
		this->objectBBoxDistanceMaxBTextureParam	= cgGetNamedParameter(this->objectBBoxDistanceProgram, "bBoxMaxB");
	}
}

void GLParameters::InitializeSilhouetteProgram()
{
	this->silhouetteProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/Silhouette.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->silhouetteProgram != NULL)
	{
		cgGLLoadProgram(this->silhouetteProgram);
		this->silhouetteNormalTextureParam	= cgGetNamedParameter(this->silhouetteProgram, "normalTexture");
		this->silhouetteVParam				= cgGetNamedParameter(this->silhouetteProgram, "v");
		this->silhouetteDeltaParam			= cgGetNamedParameter(this->silhouetteProgram, "delta");
	}
}

void GLParameters::InitializeFindPrograms()
{
#ifdef OCCLUSIONFIND
	this->findPrepassProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/Prepass.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findPrepassProgram);
	this->findPrepassTextureParam = cgGetNamedParameter(this->findPrepassProgram, "texture");

	this->occlusionProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/Occlusion.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->occlusionProgram);
	this->occlusionTextureParam = cgGetNamedParameter(this->occlusionProgram, "texture");
#endif

#ifdef GPUFIND
	this->findPrepassProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/Prepass.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findPrepassProgram);
	this->findPrepassTextureParam = cgGetNamedParameter(this->findPrepassProgram, "texture");

#ifdef GPUFIND1
	this->findReduceProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/ReduceFind.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findReduceProgram);
	this->findReduceTextureParam = cgGetNamedParameter(this->findReduceProgram, "texture");
	this->findReduceOriginParam = cgGetNamedParameter(this->findReduceProgram, "origin");

	this->findProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/Find.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findProgram);
	this->findTextureParam = cgGetNamedParameter(this->findProgram, "texture");
	this->findSizeParam = cgGetNamedParameter(this->findProgram, "iSize");
	this->findInputSizeParam = cgGetNamedParameter(this->findProgram,"InputSize");
#endif
#ifdef GPUFIND2
	this->findReduceProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/ReduceFind.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findReduceProgram);
	this->findReduceTextureParam = cgGetNamedParameter(this->findReduceProgram, "texture");
	this->findReduceOriginParam = cgGetNamedParameter(this->findReduceProgram, "origin");

	this->findProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/Find2.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findProgram);
	this->findTextureParam = cgGetNamedParameter(this->findProgram, "texture");
	this->findSourceTextureParam = cgGetNamedParameter(this->findProgram,"sourceTex");
	this->findViewportParam = cgGetNamedParameter(this->findProgram,"viewport");
	this->findOriginParam = cgGetNamedParameter(this->findProgram,"origin");
#endif
#ifdef GPUFIND3
	this->findDownSweepProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/FindDownSweep.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findDownSweepProgram);
	this->findDownSweepTextureParam = cgGetNamedParameter(this->findDownSweepProgram, "texture");
	this->findDownSweepOriginParam	= cgGetNamedParameter(this->findDownSweepProgram,"origin");
	this->findDownSweepxyDistParam	= cgGetNamedParameter(this->findDownSweepProgram,"xyDist");

	this->findUpSweepProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/FindUpSweep.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findUpSweepProgram);
	this->findUpSweepTextureParam		= cgGetNamedParameter(this->findUpSweepProgram,"texture");
	this->findUpSweepOriginParam		= cgGetNamedParameter(this->findUpSweepProgram,"origin");
	this->findUpSweepParentOriginParam	= cgGetNamedParameter(this->findUpSweepProgram,"parentOrigin");
	this->findUpSweepyDirectionParam	= cgGetNamedParameter(this->findUpSweepProgram,"yDirection");

	this->findPosChangeProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/FindPosChange.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findPosChangeProgram);
	this->findPosChangeTextureParam			= cgGetNamedParameter(this->findPosChangeProgram,"texture");
	this->findPosChangeInputTextureParam	= cgGetNamedParameter(this->findPosChangeProgram,"textureInput");

	this->findVertexProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/FindVertex.cg", this->cgVertexProfile, NULL, NULL);
	cgGLLoadProgram(this->findVertexProgram);
	this->findVertexOutputSizeParam		= cgGetNamedParameter(this->findVertexProgram, "outputTexSize");

	this->findScaleProgram = cgCreateProgramFromFile(this->cgContext, CG_SOURCE, "CGFiles/FindScale.cg", this->cgFragmentProfile, NULL, NULL);
	cgGLLoadProgram(this->findScaleProgram);
#endif
#endif
}

void GLParameters::InitializeIntersectionPrograms()
{
	this->bBoxOverlapProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/BBoxOverlap.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->bBoxOverlapProgram != NULL)
	{
		cgGLLoadProgram(this->bBoxOverlapProgram);
		this->bBoxMaxATextureParam			= cgGetNamedParameter(this->bBoxOverlapProgram, "bBoxMaxA");
		this->bBoxMinATextureParam			= cgGetNamedParameter(this->bBoxOverlapProgram, "bBoxMinA");
		this->bBoxMaxBTextureParam			= cgGetNamedParameter(this->bBoxOverlapProgram, "bBoxMaxB");
		this->bBoxMinBTextureParam			= cgGetNamedParameter(this->bBoxOverlapProgram, "bBoxMinB");
		this->bBoxReferenceListTextureParam	= cgGetNamedParameter(this->bBoxOverlapProgram, "referenceList");
		this->bBoxLevelOriginParam			= cgGetNamedParameter(this->bBoxOverlapProgram, "levelOrigin");
	}
}



//iddo:
void GLParameters::InitializeHausdorffIddoProgram()
{
	this->hausdorffIddoProgram = cgCreateProgramFromFile(this->cgContext,CG_SOURCE,"CGFiles/HausdorffIddo.cg",this->cgFragmentProfile, NULL, NULL);
	if(this->hausdorffIddoProgram != NULL) {
		cgGLLoadProgram(this->hausdorffIddoProgram);
		this->hausdorffIddoSurface1Param	= cgGetNamedParameter(this->hausdorffIddoProgram, "surface1Pts");
		this->hausdorffIddoSurface2Param	= cgGetNamedParameter(this->hausdorffIddoProgram, "surface2Cps");
		this->hausdorffIddoPrevDistsParam	= cgGetNamedParameter(this->hausdorffIddoProgram, "prevDists");

		this->hausdorffIddoSurface2OrdUParam	= cgGetNamedParameter(this->hausdorffIddoProgram, "ordU");
		this->hausdorffIddoSurface2OrdVParam	= cgGetNamedParameter(this->hausdorffIddoProgram, "ordV");
	}
}