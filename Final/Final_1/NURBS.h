#pragma once
#include "Utilities.h"
#include "GLParameters.h"

class BSpline
{
public:
	BSpline();
	~BSpline(){/*delete [] cntlPoints; delete [] knotVector;*/};

	Float2 Evalutate(float);
	float Basis(int,int,float);

	float* cntlPoints;				// Control point array
	float* knotVector;				// Knot Vector
	int order;
	int nPoints;					// Number of Control Points
};

class EvaluationParams 
{
public:
	EvaluationParams();
	~EvaluationParams();

	bool rendering;							// True if used for rendering

	float uMin;								// Range for evaluation u1
	float uMax;								// Range for evaluation u2
	float vMin;								// Range for evaluation v1
	float vMax;								// Range for evaluation v2

	float* uEvalParamPoints;				// u Parametric evaluation points for nou-uniform evaluation
	float* vEvalParamPoints;				// v Parametric evaluation points for nou-uniform evaluation

	int uNum;								// Evaluation grid u size
	int vNum;								// Evaluation grid v size
	int uNumTrim;							// Trim grid u size
	int vNumTrim;							// Trim grid v size

	int numQuadraturePoints;				// Number of quadrature points for integration

	GLuint ctrlPointsTexture;				// Surface data
	GLuint uCtrlPointIndexTexture;			// Index of u control points
	GLuint vCtrlPointIndexTexture;			// Index of v control points

	GLuint uBasisFuncTexture;				// Texture ID for u basis function
	GLuint vBasisFuncTexture;				// Texture ID for v basis function
	GLuint uBasisFuncDervTexture;			// Texture ID for u basis function derivative
	GLuint vBasisFuncDervTexture;			// Texture ID for v basis function derivative

	GLuint evaluatedSurfTexture;			// Texture to store the evaluated surface
	GLuint evaluatedSurfNormTexture;		// Texture to store the evaluated surface normal

#ifdef CUDA
	float*	uBasisFuncCUDAData;				// CUDA pointer to u basis function
	float*	vBasisFuncCUDAData;				// CUDA pointer to v basis function
	float*	uBasisDervCUDAData;				// CUDA pointer to u basis function derivative
	float*	vBasisDervCUDAData;				// CUDA pointer to v basis function derivative
	float*	ctrlPointsCUDAData;				// CUDA pointer to control points
	int*	uCtrlPointIndexCUDAData;		// CUDA pointer to index of u control points
	int*	vCtrlPointIndexCUDAData;		// CUDA pointer to index of v control points
	float3* secondDervCUDAData[4];			// Pointer to CUDA pointers with second derivative

	float3* bBoxMinCUDAData;				// CUDA pointer to min bbox data
	float3* bBoxMaxCUDAData;				// CUDA pointer to max bbox data

	float4* evaluatedCUDAPoints;			// CUDA pointer to the evaluted surface
	float4* transformedCUDAPoints;			// CUDA pointer to the transformed surface
	float3* evaluatedCUDANormals;			// CUDA pointer to the evaluted surface normal
#endif

	GLuint* bBoxTextures;					// Textures to store the evaluated bounding boxes
	GLuint* rotatedBBoxTextures;			// Textures to store the totated bounding boxes
	GLuint* secondDervTextures;				// Textures to store the second derivatives
	
	GLuint trimTexture;						// Trim texture;
	GLuint expansionFactorTexture;			// Expansion factor texture for variable expansion case

	int srcTexIndex, dstTexIndex;			// Indices for ping-pong
	
	float* minPoints;						// Min point array of bounding box
	float* maxPoints;						// Max point array of bounding box
	float* evaluatedPoints;					// Evaluated surface points array
	float* evaluatedNormals;				// Evaluated normals array
	float* trimData;						// Trim information showing 0 or 1
	float* expansionFactors;				// Bounding box expansion factor for variable expansion case
	float  maxExpansionFactor;				// Bounding box expansion factor

	bool computeMomentError;				// Boolean to compute error
};

class NURBS
{
public:
	NURBS();
	~NURBS();

	//CPU Evaluations
	Float4 Evaluate4(float, float);						// Evaluates the 4 components of the NURBS surface 
	Float3 Evaluate(float, float);						// Evaluates the 3 components of the NURBS surface after dividing by w
	void Evaluate(float, float, float[3]);				// Evaluates the 3 components of the NURBS and outputs to third argument float[3]
	Float4 Evaluate4uDerv(float, float);				// Evaluates the 4 components of the NURBS surface u derivative
	Float4 Evaluate4vDerv(float, float);				// Evaluates the 4 components of the NURBS surface u derivative
	Float3 EvaluateNormal(float, float);				// Evaluates the 3 components of the NURBS surface normal after dividing by w
	float Basis(int, int, float, int);					// BSpline basis of particular order

	void InitNURBS(int);								// Initializes Duck or Flag test model
	void ReadNURBS(char*);								// Reads in NURBS Surface from file
	void SaveNURBS(char*);								// Save NURBS Surface to file

	void DrawNURBS(GLParameters*, float);				// Draws the NURBS surface
	void DrawPickedPoint(GLParameters*);				// Draws the picked point on the NURBS surface
	void GenerateCurveOverlayTexture(GLParameters*);	// Generates the texture drawn on the surface
	void DrawOverlayTrimTexture(GLParameters*,Float4);  // Draws the NURBS trim texture and overlay texture
	void SaveMesh(const char*, GLParameters*);			// Evaluate and save the mesh in obj format

	float CalculateBoundingBoxSize();					// Calculates the size of the diagonal of the boundingbox based on control points
	void DrawBoundingBox(GLParameters*);				// Draws the NURBS bounding boxes
	void DrawControlMesh(GLParameters*);				// Draws the NURBS Control Mesh
	int PickControlPoint(int, int, float*, float*);		// Pick the control point of the NURBS mesh
	void UpdateTrimCurves(GLParameters*);				// Update trim curves after drawing
	void DrawOBB();										// Draw the OBB of the surface

	void GenerateTrimData(int, int, GLParameters*);		// Generates the trim texture and stores it in a array
	void GenerateTrimTexture(GLParameters*);			// Generates the trim texture and stores it in the texture
	void GenerateIntersectionOverlayTexture(float*, int, int, GLParameters*);				// Generates the intesection texture for display
	int TrimInOut(int, int, int, int);

	bool operator< (const NURBS &a){return (a.visibilityFactor <= this->visibilityFactor);};

	GLuint GPUBasisFunctionEvaluation(int k, int n, int evalNum, float paramMin, float paramMax, float* knotVector, GLParameters* glParam, GLuint* tempDervTex, float* ctrlData, float* paramValues, bool nonUniformEvaluation);

	GLuint GPUBasisCtrlPointMultiplication(GLuint*, GLuint, GLuint, GLParameters*);
	void GPUSecondDerivativesEvaluation(int, int, float[2], GLuint, GLuint, GLuint, GLParameters*);
	void GPUSecondDerivativesNonUniformEvaluation(int, int, int, float[2], GLuint, GLuint, GLuint, GLParameters*);
	void GPUNormalEvaluation(GLParameters*);
	void GPUExactNormalEvaluation(GLParameters*);
	void GPUSilhouetteEvaluation(Float3, GLParameters*);
	void GPUComputeVariableExpansionFactor(int, int, GLuint, GLuint[4], GLParameters*, float* expansionFactorData = NULL);
	void GPUComputeBoundingBox(int, int, GLuint, GLuint, GLuint, GLuint[4], float&, GLParameters*, float* minData = NULL, float* maxData = NULL );
	void GPUComputeBoundingBoxTrimmed(int, int, GLuint, GLuint, GLuint, GLuint[4], float&, GLuint, GLParameters*, float* minData = NULL, float* maxData = NULL );
	void GPUBuildBoundingBoxHierarchy(GLuint, int, GLParameters*, float, float*);
	void GPUBuildBoundingBoxHierarchyTrimmedMax(GLuint, int, GLParameters*, float*);
	void GPUBuildBoundingBoxHierarchyTrimmedMin(GLuint, int, GLParameters*, float*);
	void GPUInverseEvaluate(float[4], GLuint, GLuint, GLParameters*);
	void GPURayInverseEvaluate(float[4], GLuint, GLuint, GLParameters*);
	void GPUClosestPoint(float[4], GLuint, GLuint, GLParameters*, int[2]);
	void GPUComputeCurvature(GLParameters*);

	void GPUTransformEvaluatedPoints(GLuint, GLParameters*);

	vector<list<int>> FitSihouette(float*, int, int );							// int nPoints, resolution
	int SurfaceSilhouetteEvaluation(Float3, GLParameters* glParam, int, bool);	// int resolution bool fitSilhouette
	int SurfaceEvaluation(GLParameters* glParam, bool rendering, bool nonUniformEvaluation);
//	int SurfaceEvaluation(GLParameters*, bool);									// Boolean true for rendering
//	int SurfaceNonUniformEvaluation(GLParameters*);								// Non uniform evaluation of surfaces
	int SurfaceBoundingBoxEvaluation(GLParameters*);
	int SurfaceExpansionFactorEvaluation(GLParameters*);
	int SurfaceBoundingBoxTrimmedEvaluation(GLParameters*);
	int SurfaceInverseEvaluate(float*, GLParameters*, float*);					//Float4 input, Float2 output
	int SurfaceClosestPoint(float*, GLParameters*, float*);
	int SurfaceCurvatureEvaluation(int, GLParameters*);

#ifdef CUDA
	int CUDABasisFunctionEvaluation(int, int, int, float, float, float*, int*, float*, float*);
	int CUDABasisCtrlPointMultiplication(float*, int*, int*, float*, float*, float4*);
	int CUDABasisCtrlPointMultiplicationTextures(cudaArray*, cudaArray*, cudaArray*, cudaArray*, cudaArray*, float4*);
	int	CUDAComputeBoundingBox(int, int, float4*, float3*, float3*, float3* secondDervCUDAData[4], bool, float&, float* minData = NULL, float* maxData = NULL);
	int CUDAComputeBoundingBox();
	int CUDABuildBoundingBoxHierarchy(int, int, float*, float*);

	int CUDANormalEvaluation(bool);							// Boolean rendering
	int CUDAExactNormalEvaluation(bool, bool);				// Boolean rendering, Boolean second derivatives
	int CUDASurfaceEvaluation(bool, bool, bool, bool);		// Boolean exactNormals, Boolean secondDerivatives, Boolean readBack, Boolean rendering
	int CUDASurfaceBoundingBoxEvaluation(bool, bool, bool);	// Boolean contruct hieararchy, Boolean expand bounding box, Boolean read back
	int CUDASurfaceTransformation(bool);					// Boolean read back

//	int CUDANormalEvaluation(bool, cudaGraphicsResource*, cudaGraphicsResource*);						// Boolean rendering
//	int CUDAExactNormalEvaluation(bool, cudaGraphicsResource*, cudaGraphicsResource*);					// Boolean rendering
//	int CUDASurfaceEvaluation(bool, bool, bool, cudaGraphicsResource*, cudaGraphicsResource*);			// Boolean exactNormals, Boolean readBack, Boolean rendering

	//iddo: Storing Bezier patches
	vector<float*> m_bezPatches;
	vector<float4*> m_bezCUDAPatchesTransformed;
	vector<float4*> m_bezCUDAPatches;
	void CUDASetBezPatches(); //gets Bezier surfaces - each of size uOrder*vOrder*4 and stores in CUDA memory

	//adarsh: Drawing Bezier Control Points
	void DrawBezierPatchControlPoints(GLParameters*);

#ifdef CUDATEXTURES
	int CUDASurfaceEvaluationTextures(bool, bool, bool, bool);		// Boolean exactNormals, Boolean secondDerivatives, Boolean readBack, Boolean rendering
#endif
#endif

	float transformationMatrix[16];			// Matrix used for transformation
	bool identityTransformation;			// True if the transformation is identity
	Float3 kdColor;							// Diffuse Color
	Float3 ksColor;							// Specular Color
	float ka;								// Ambient factor
//	float transparency;						// Transparency
	float shininess;						// Shininess
	
	float* cntlPoints;						// Control Points array
	float* uKnotVector;						// Knot Vector in the u direction
	float* vKnotVector;						// Knot Vector in the v direction
	int uOrder;								// Order in the u direction
	int vOrder;								// Order in the v direction
	int uPoints;							// Number of Control points in the u direction
	int vPoints;							// Number of Control points in the v direction

	bool trimmed;							// Trimmed surface flag if surface is trimmed
	vector<BSpline*> trimCurves;			// Trim curves of ths surface

	GLuint trimTexture;
	int trimWidth;
	int trimHeight;
	float trimCurveLOD;						// LOD of Trim curve evaluation

	float sizeFactor;						// Ratio for the size of the surface to size of the model in world coordinates 

	bool textured;
	GLuint texture;

	float uParamRange;						// Range of the Knot vector in u direction
	float vParamRange;						// Range of the Knot vector in v direction
	float uParamOffset;						// Offset of the Knot vector from 0 in u direction
	float vParamOffset;						// Offset of the Knot vector from 0 in v direction

	int surfaceID;							// Global surfaceID
	float visibilityFactor;					// Ratio of part of the surface visible to the total screen size

	Float3 bBoxMin;							// Surface bounding box Min point
	Float3 bBoxMax;							// Surface bounding box Max point

	vector<int>		pickedBoxes;			// Index of the bounding box picked during a certain level of evaluation
	vector<Float2>	pickedParamPoints;		// Parametric points picked on the surface
	vector<Float2>	prevPickedParamPoints;	// Parametric points picked previously used for drawing on the surface
	vector<Float4>	pickedPoints;			// Points picked on the surface
	vector<Float4>	prevPickedPoints;		// Points picked previously used for drawing on the surface

	int uBaseNum;							// Evaluation grid base u size for LOD = 1
	int vBaseNum;							// Evaluation grid base v size for LOD = 1
	int uNumDisplay;						// Evaluation grid u size
	int vNumDisplay;						// Evaluation grid v size

	float* evaluatedPointsDisplay;			// Evaluated surface points array
	float* evaluatedNormalsDisplay;			// Evaluated surface normals array
	float* minPointsDisplay;				// Min point of bounding box
	float* maxPointsDisplay;				// Max point of bounding box

	bool evaluatedVBO;						// True if VBOs have been evaluated
	GLuint ptVBO;							// VBO for the evaluated point coordinates
	GLuint normVBO;							// VBO for the normals
	GLuint texVBO;							// VBO for the texture coordinates
	GLuint* indexArray;						// Indices for the VBO

	bool evaluatedSilhouette;				// True if Silhoette is evaluated
	vector<int> silhouettePointIndex;		// Index of the silhouette points
	int silhouetteLevel;					// Level of iteration;

	bool evaluatedIntersection;				// True if intersection is evaluated
	float* intersectingBoxes;				// Parametric values of intersecting bounding boxes;
	int numIntersectingBoxes;				// Number of intersecting boxes

	bool initializedEvalParams;				// True if evalParams exist
	EvaluationParams* evalParams;			// Temporary evaluation parameters

	float moment0;							// Volume contribution due to surface
	Float3 moment1;							// x,y,z-Moment contribution due to surface
	float normalSign;						// -1 or 1 based on the outward direction of the NURBS normals;
	float moment0Error;						// Error in volume contribution
	Float3 moment1Error;					// Error in moment contribution
};

inline bool NURBSgreaterTest(const NURBS* p1, const NURBS* p2)
{
	if(!p1)
		return true;
	if(!p2)
		return false;
	return (p1->visibilityFactor > p2->visibilityFactor);
}

inline void TranslateNURBS(NURBS* surface,float dist)
{

	for (int v = 0; v < surface->vPoints ; v++)
	{
		for (int u = 0; u < surface->uPoints; u++)
		{
			float angle = 30;
			float tempX = surface->cntlPoints[v*surface->uPoints*4+u*4+1];
			float tempZ = surface->cntlPoints[v*surface->uPoints*4+u*4+2];
			surface->cntlPoints[v*surface->uPoints*4+u*4+1] = tempX*cos(angle*PI/180.0) + tempZ*sin(angle*PI/180);
			surface->cntlPoints[v*surface->uPoints*4+u*4+2] = -tempX*sin(angle*PI/180.0) + tempZ*cos(angle*PI/180);
		}
	}
	for (int v = 0; v < surface->vPoints ; v++)
		for (int u = 0; u < surface->uPoints; u++)
			surface->cntlPoints[v*surface->uPoints*4+u*4+2] += dist;

}

inline void TransformNURBSCntlPts(NURBS* surface, float angle)
{

	for (int v = 0; v < surface->vPoints ; v++)
	{
		for (int u = 0; u < surface->uPoints; u++)
		{
			float tempX = surface->cntlPoints[v*surface->uPoints*4+u*4+0];
			float tempY = surface->cntlPoints[v*surface->uPoints*4+u*4+1];
			surface->cntlPoints[v*surface->uPoints*4+u*4+0] = tempX*cos(angle*PI/180.0) + tempY*sin(angle*PI/180);
			surface->cntlPoints[v*surface->uPoints*4+u*4+1] = -tempX*sin(angle*PI/180.0) + tempY*cos(angle*PI/180);
		}
	}
}
