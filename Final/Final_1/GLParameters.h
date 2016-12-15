#include "Includes.h"

#ifndef GLPARAMETERS
#define GLPARAMETERS

class GLParameters
{
public:
	GLParameters();
	~GLParameters();

	//Setup functions
	void InitializeCG();

	void InitializeNormalProgram();
	void InitializeTrimProgram();
	void InitializePassiveProgram();
	void InitializeCurveEvaluationProgram();
	void InitializeSurfaceEvaluationProgram();
	void InitializeTSplineEvaluationProgram();
	void InitializeInvertProgram();
	void InitializeSilhouetteProgram();
	void InitializeFindPrograms();
	void InitializeIntersectionPrograms();
	void InitializeReferencePrograms();
	void InitializeBoundingBoxProgram();
	void InitializeClosestPointProgram();
	void InitializeSurfaceIDProgram();
	void InitializeCurvatureProgram();
	void InitializeMomentProgram();

#ifdef GPUCOLLISION
	void InitializeOrientedBoundingBoxProgram();
#endif

	bool initialized;
	// CG Variables
	GLuint fbo;
	GLuint depthBuffer;
	GLuint stencilBuffer;
	GLuint* curveTexture;
	GLuint* basisDervTexture;
	CGcontext	cgContext;
	CGprofile	cgFragmentProfile;
	CGprofile	cgVertexProfile;
	GLuint occlusionQuery;

	CGprogram	passiveProgram;

	CGprogram	passiveSurfaceIDProgram;
	CGparameter	passiveSurfaceIDParam;

	CGprogram	passiveTextureProgram;
	CGparameter	passiveTextureParam;

	CGprogram	trimOverlayProgram;
	CGparameter	trimOverlayTrimTextureParam;
	CGparameter	trimOverlayTextureParam;

	CGprogram	trimProgram;
	CGparameter	trimTextureParam;

	CGprogram	trimSurfaceIDProgram;
	CGparameter	trimSurfaceIDTextureParam;
	CGparameter	trimSurfaceIDParam;

	CGprogram	evaluateBasisProgram;
	CGparameter	knotDataTextureParam;
	CGparameter	basisTextureParam;
	CGparameter	passParam;

	CGprogram	surfaceMultProgram;
	CGparameter	surfaceMultCtrlPtsTextureParam;
	CGparameter	surfaceMultuCtrlIndexTextureParam;
	CGparameter	surfaceMultvCtrlIndexTextureParam;
	CGparameter	surfaceMultuBasisFuncTextureParam;
	CGparameter	surfaceMultvBasisFuncTextureParam;
	CGparameter	surfaceMultSurfPtSrcTextureParam;
#ifdef NOPINGPONG
	CGparameter	surfaceMultkuParam;
	CGparameter	surfaceMultkvParam;
#else
	CGparameter	surfaceMultuLenParam;
	CGparameter	surfaceMultvLenParam;
	CGparameter	surfaceMultuPassParam;
	CGparameter	surfaceMultvPassParam;
#endif

	CGprogram	normalProgram;
	CGparameter	normalTextureParam; 
	CGparameter	normalDimParam;

	CGprogram	boundingBoxProgram;
	CGparameter	boundingBoxSurfaceTextureParam;
	CGparameter boundingBoxModelViewParam;
	CGparameter boundingBoxFactorParam;
	CGparameter boundingBoxFactorTextureParam;

	CGprogram	boundingBoxTrimmedProgram;
	CGparameter	boundingBoxTrimmedSurfaceTextureParam;
	CGparameter	boundingBoxTrimmedTrimTextureParam;
	CGparameter boundingBoxTrimmedModelViewParam;
	CGparameter boundingBoxTrimmedFactorParam;
	CGparameter boundingBoxTrimmedFactorTextureParam;

	CGprogram   invertProgram;
	CGparameter invertBBoxMinTextureParam;
	CGparameter invertBBoxMaxTextureParam;
	CGparameter invertPtParam;

	CGprogram   invertRayProgram;
	CGparameter invertRayBBoxMinTextureParam;
	CGparameter invertRayBBoxMaxTextureParam;
	CGparameter invertRayPtParam;

	CGprogram   silhouetteProgram;
	CGparameter silhouetteNormalTextureParam;
	CGparameter silhouetteVParam;
	CGparameter silhouetteDeltaParam;

	CGprogram   bBoxOverlapProgram;
	CGparameter bBoxMinATextureParam;
	CGparameter bBoxMinBTextureParam;
	CGparameter bBoxMaxATextureParam;
	CGparameter bBoxMaxBTextureParam;
	CGparameter bBoxReferenceListTextureParam;
	CGparameter bBoxLevelOriginParam;

	CGprogram   transformSurfaceProgram;
	CGparameter transformSurfaceTextureParam;
	CGparameter transformSurfaceMatrixParam;

	CGprogram	bBoxHierarchyProgram;
    CGparameter bBoxHierarchyTextureParam;
    CGparameter bBoxHierarchyOriginParam;
	CGparameter bBoxHierarchyIsMaxParam;

	CGprogram	bBoxHierarchyTrimmedMaxProgram;
    CGparameter bBoxHierarchyTrimmedMaxTextureParam;
    CGparameter bBoxHierarchyTrimmedMaxOriginParam;

	CGprogram	bBoxHierarchyTrimmedMinProgram;
    CGparameter bBoxHierarchyTrimmedMinTextureParam;
    CGparameter bBoxHierarchyTrimmedMinOriginParam;

	CGprogram	evaluateBasisDervProgram;
	CGparameter	knotDataDervTextureParam;
	CGparameter	basisDervTextureParam;
	CGparameter	passDervParam;

	CGprogram	exactNormalProgram;
	CGparameter	exactNormaluDervTextureParam;
	CGparameter	exactNormalvDervTextureParam;

	CGprogram	rationalProgram;
	CGparameter	rationalSurfaceTextureParam;
	CGparameter	rationalDervTextureParam;

	CGprogram	secondDervProgram;
	CGparameter	secondDervTextureParam;
	CGparameter secondDervUVIntervalParam;
	CGparameter secondDervDimensionParam;
	
	CGprogram	findMaxProgram;
	CGparameter findMaxTextureParam;
	CGparameter findMaxFactorParam;

	CGprogram	reduceMaxProgram;
	CGparameter reduceMaxTextureParam;
	
	CGprogram	reduceMinProgram;
	CGparameter reduceMinTextureParam;

	CGprogram	mergeExpansionFactorsProgram;
	CGparameter mergeFactorTex1TextureParam;
	CGparameter mergeFactorTex2TextureParam;
	CGparameter mergeFactorTex3TextureParam;

	CGprogram   closestPointProgram;
	CGparameter closestPointBBoxMinTextureParam;
	CGparameter closestPointBBoxMaxTextureParam;
	CGparameter closestPointPtParam;

	CGprogram   bBoxDistProgram;
	CGparameter bBoxDistMinATextureParam;
	CGparameter bBoxDistMinBTextureParam;
	CGparameter bBoxDistMaxATextureParam;
	CGparameter bBoxDistMaxBTextureParam;
	CGparameter bBoxDistReferenceListTextureParam;
	CGparameter bBoxDistLevelOriginParam;

	CGprogram   bBoxHausdorffDistProgram;
	CGparameter bBoxHausdorffDistMinATextureParam;
	CGparameter bBoxHausdorffDistMinBTextureParam;
	CGparameter bBoxHausdorffDistMaxATextureParam;
	CGparameter bBoxHausdorffDistMaxBTextureParam;
	CGparameter bBoxHausdorffDistReferenceListTextureParam;
	CGparameter bBoxHausdorffDistRowNumberTextureParam;
	CGparameter bBoxHausdorffDistLevelOriginParam;

	CGprogram	reduceMinBBox2Program;
	CGparameter reduceMinBBox2TextureParam;
	CGparameter reduceMinBBox2OffsetParam;

	CGprogram	reduceMinMinBBox2Program;
	CGparameter reduceMinMinBBox2TextureParam;
	CGparameter reduceMinMinBBox2OffsetParam;

	CGprogram	reduceMinMaxBBox2Program;
	CGparameter reduceMinMaxBBox2TextureParam;
	CGparameter reduceMinMaxBBox2OffsetParam;

//	CGprogram	reduceMaxBBox2Program;
//	CGparameter reduceMaxBBox2TextureParam;
//	CGparameter reduceMaxBBox2OffsetParam;

	CGprogram	reduceMinBBox4Program;
	CGparameter reduceMinBBox4TextureParam;

	CGprogram   bBoxDistFilterProgram;
	CGparameter bBoxDistFilterTextureParam;
	CGparameter bBoxDistFilterMaxValParam;

	CGprogram   bBoxHausdorffFilter1Program;
	CGparameter bBoxHausdorffFilter1MinMaxTextureParam;
	CGparameter bBoxHausdorffFilter1MinValParam;

	CGprogram   bBoxHausdorffFilter2Program;
	CGparameter bBoxHausdorffFilter2MinMaxTextureParam;
	CGparameter bBoxHausdorffFilter2MinRowTextureParam;
	CGparameter bBoxHausdorffFilter2FilterRowTextureParam;
	CGparameter bBoxHausdorffFilter2TexSizeParam;

	CGprogram   objectBBoxDistanceProgram;
	CGparameter objectBBoxDistanceMinATextureParam;
	CGparameter objectBBoxDistanceMaxATextureParam;
	CGparameter objectBBoxDistanceMinBTextureParam;
	CGparameter objectBBoxDistanceMaxBTextureParam;

	CGprogram   curvatureProgram;
	CGparameter curvatureuDervTextureParam;
	CGparameter curvaturevDervTextureParam;
	CGparameter curvatureuuSecondDervTextureParam;
	CGparameter curvatureuvSecondDervTextureParam;
	CGparameter curvaturevvSecondDervTextureParam;

	CGprogram   moment2PointProgram;
	CGparameter moment2PointSurfaceTextureParam;
	CGparameter moment2PointNormalTextureParam;
	CGparameter moment2PointMultConstantParam;

	CGprogram   moment3PointProgram;
	CGparameter moment3PointSurfaceTextureParam;
	CGparameter moment3PointNormalTextureParam;
	CGparameter moment3PointMultConstantParam;

	CGprogram	reduceSum2Pow2Program;
	CGparameter reduceSum2Pow2InputTextureParam;
	CGparameter reduceSum2Pow2OffsetParam;

	CGprogram	reduceSum2Program;
	CGparameter reduceSum2InputTextureParam;
	CGparameter reduceSum2OffsetParam;

	CGprogram   reduceSum4Program;
	CGparameter reduceSum4InputTextureParam;

	CGprogram   reduceSum9Program;
	CGparameter reduceSum9InputTextureParam;

	CGprogram	momentTrimMultProgram;
	CGparameter momentTrimMultMomentTextureParam;
	CGparameter momentTrimMultTrimTextureParam;

	CGprogram	secondDerv2PointProgram;
	CGparameter	secondDerv2PointTextureParam;
	CGparameter secondDerv2PointUVIntervalParam;

	CGprogram	secondDerv3PointProgram;
	CGparameter	secondDerv3PointTextureParam;
	CGparameter secondDerv3PointUVIntervalParam;

	CGprogram   momentErrorProgram;
	CGparameter momentErrorSurfaceTextureParam;
	CGparameter momentErrorExpansionTextureParam;
	CGparameter momentErrorMomentTextureParam;
	CGparameter momentErrorTrimTextureParam;
	CGparameter momentErrorTrimPointsParam;

	CGprogram   momentTriangleProgram;
	CGparameter momentTriangleVertex1TextureParam;
	CGparameter momentTriangleVertex2TextureParam;
	CGparameter momentTriangleVertex3TextureParam;

#ifdef EVALUATECURVE
	CGprogram	basisMultProgram;
	CGprogram	reductionProgram;
	CGparameter	basisFuncTextureParam;
	CGparameter	xDistParam;
	CGparameter	texHeightParam;
	CGparameter	knotIndexTextureParam;
	CGparameter	cntlPtsTextureParam;
	CGparameter	curvePtsTextureParam;
#endif

#ifdef GPUCOLLISION
	CGprogram oBBoxOverlapProgram;
	CGparameter oBBoxMin1TextureParam;
	CGparameter oBBoxMax1TextureParam;
	CGparameter oBBoxMin2TextureParam;
	CGparameter oBBoxMax2TextureParam;
	CGparameter oBBoxTransformParam;
	CGparameter oBBoxTranslateParam;
#endif

#ifdef OCCLUSIONFIND
	CGprogram	findPrepassProgram;
	CGparameter findPrepassTextureParam;

	CGprogram occlusionProgram;
	CGparameter occlusionTextureParam;
#endif

#ifdef GPUFIND
	CGprogram	findPrepassProgram;
	CGparameter findPrepassTextureParam;

#ifdef GPUFIND1
	CGprogram	findReduceProgram;
	CGparameter findReduceTextureParam;
	CGparameter findReduceOriginParam;

	CGprogram	findProgram;
	CGparameter findTextureParam;
	CGparameter findSizeParam;
	CGparameter findInputSizeParam;
#endif
#ifdef GPUFIND2
	CGprogram	findReduceProgram;
	CGparameter findReduceTextureParam;
	CGparameter findReduceOriginParam;

	CGprogram	findProgram;
	CGparameter findTextureParam;
	CGparameter findSourceTextureParam;
	CGparameter findOriginParam;
	CGparameter findViewportParam;
#endif
#ifdef GPUFIND3
	CGprogram	findDownSweepProgram;
	CGparameter findDownSweepTextureParam;
	CGparameter findDownSweepOriginParam;
	CGparameter findDownSweepxyDistParam;

	CGprogram	findUpSweepProgram;
	CGparameter findUpSweepTextureParam;
	CGparameter findUpSweepOriginParam;
	CGparameter findUpSweepParentOriginParam;
	CGparameter findUpSweepyDirectionParam;

	CGprogram	findPosChangeProgram;
	CGparameter findPosChangeTextureParam;
	CGparameter findPosChangeInputTextureParam;

	CGprogram	findVertexProgram;
	CGparameter findVertexOutputSizeParam;

	CGprogram	findScaleProgram;
#endif

	CGprogram referenceProgram;
	CGparameter referenceOverlapTexParam;
	CGparameter referenceOldListTexParam;
	CGparameter referenceSizeDataParam;

	CGprogram intervalProgram;
	CGparameter intervalRefListTexParam;
	CGparameter intervalMinMax1;
	CGparameter intervalMinMax2;
	CGparameter intervalUVNum;
#endif

	//iddo:
	void InitializeHausdorffIddoProgram();
	CGprogram	hausdorffIddoProgram;
	CGparameter	hausdorffIddoSurface1Param;
	CGparameter	hausdorffIddoSurface2Param;
	CGparameter hausdorffIddoPrevDistsParam;
	CGparameter hausdorffIddoSurface2OrdUParam;
	CGparameter hausdorffIddoSurface2OrdVParam;


	bool picked;
	bool drawingON;
	int pickedSurfNum;
	int pickedControlPointNum;

	int pickedObjectNum;
	int pickedFaceNum;

	bool smooth;
	bool wireframe;
	bool controlMesh;

	bool intersection;
	bool exactNormals;
	bool expandBoundingBox;
	bool drawBBox;
	bool readBack;
	bool silhouette;
	bool selfIntersection;
	bool collision;
	bool inverseEvaluation;
	bool variableExpansionFactor;
	bool enhancedAccuracy;
	bool closestPoint;
	bool computingMoment;
	bool curvatureEvaluation;
	bool hausdorff;
	bool drawVoxels;
	bool displayNURBS;

	Float3 bBoxMin;		// Global bounding box Min point
	Float3 bBoxMax;		// Global bounding box Max point

	float surfVisibilityCutOff;		// Visibility cut off for surfaces
	float objVisibilityCutOff;		// Visibility cut off for objects
	float modelViewMatrix[16];		// Model View matrix for the current View
	float* pickedPoint;				// Picked point in window coordinates
	float offsetDistance;			// Surface offset distance
	int voxelCount;					// voxel count
	int displayLevel;				// Level of hierarchy to display

	Float3 point1;
	Float3 point2;
	bool closestPointComputed;

	// CUDA GL Interop
//	struct cudaGraphicsResource* pointVBOResource;
//	struct cudaGraphicsResource* normalVBOResource;


	// NURBS Definitions for testing
	GLUnurbsObj *theNurb;

};

#endif