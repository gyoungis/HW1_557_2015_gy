#pragma once
#include "Includes.h"
#include "GLParameters.h"

void MakeIdentityMatrix(float[16]);

class Vertex
{
public:
	Float3 point;
	Float3 normal;
	Float2 texCoords;
	Float4 color;
	vector<int> commonFaces;
};

class Triangle
{
public:
	Index3 vertexIndex;			// Indices of the vertices in the triangle
	int adjacentFaceIndex[3];	// Indices of the adjacent triangles
	Vertex vertices[3];			// Coordinates of each vertex
	Float3 faceNormal;			// normal of the triangle for flat shading
	Vertex &operator[] (int i){return vertices[i];};
	int triangleID;			// ID of triangle in face
};

class BBoxData
{
public:
	BBoxData(){};
	~BBoxData(){surfaces.clear();};

	Float3 minPoint;
	Float3 maxPoint;
	Float3 avgNorm;
	vector<int> surfaces;
	vector<int> triangles;
	vector<int> faceIDs;
	int objID;
	int solid;
	int intersecting;
	int childIndex1;
	int childIndex2;
	int index;
};

class VoxelData
{
public:
	VoxelData(){ level1InOut = NULL; numLevels = 0; storeBoxData = false;}
	~VoxelData(){ if (level1InOut!=NULL) delete[] level1InOut; if (bBox!=NULL) delete[] bBox; if (numLevels!=0) delete[] bBoxHierarchy; numLevels = 0;}

	// In out classification
	int numDivX;
	int numDivY;
	int numDivZ;
	float gridSizeX;
	float gridSizeY;
	float gridSizeZ;
	inOutDType* level1InOut;
	

	int numDivX2;
	int numDivY2;
	int numDivZ2;
	float gridSizeX2;
	float gridSizeY2;
	float gridSizeZ2;
	vector<int> level2Index; // Index for boundary voxels, done in Classify Tessellation
	inOutDType* level2InOut; // Global for Level 2
	
	vector<int> boundaryVoxels;

	BBoxData* bBox;
	BBoxData* bBoxHierarchy;
	int numLevels;
	bool storeBoxData;
};


class Light
{
public:
	Light() {};
	Light(const Light &that);
	Light &operator=(const Light &that);

	GLfloat ambient[4]; // ambient color, RGBA
	GLfloat diffuse[4]; // diffuse color, RGBA
	GLfloat specular[4]; // specular color, RGBA
	GLfloat pos[4]; // position XYZW
	GLenum id; // light identifier

	void Apply(); // add the light to the scene
};

class Camera
{
public:
	Camera &operator=(const Camera &that);

	float fov;				// field of view
	Float3 eye;			// eye point, cop
	Float3 center;		// point the eye is looking at
	Float3 up;			// up direction

	float nearcp, farcp;	// near and far clipping planes
	float leftcp, rightcp;	// left and right clipping planes
	float topcp, bottomcp;	// left and right clipping planes
};

class Viewport
{
public:
	int w, h; // width and height
};

class ClosestPolylineData
{
public:
	int index;				// Index of the polyline itself
	int closestIndex;		// Index of the closest polyline in the list
	int closestOrder;		// 0 = 1e2s, 1 = 1s2e, 2  = 1e2e, 3 = 1s2s
	float closestDist;		// Distance to the closest polyline
};

void WriteToFile(float*, int, int);
void WriteToFileInt(float*, int, int);
void WriteToFileInt(float*, int, int, int);
void WriteToFileInt(float*, char*, int, int);
void WriteToFileInt(int*, int, int);
void WriteToFile(float*, char*, int, int);
void WriteToFile(double*, int, int);
void WriteToFile(float*, int, int, int);
void WriteToFile(float*, int, int, int, int);
void WriteToFile3(float*, int, int, int);
void WriteToFileAll(float*, int, int, int);
void WriteToFileCPP(float*, char*, int, int);
void WriteToFileCPPInt(float*, char*, int w, int h);
char* TextFileRead(char*);
int TextFileWrite(char*, char*);
void PrintShaderInfoLog(GLuint);
void PrintProgramInfoLog(GLuint);

void DumpScreenToFile(char*, int, int, int);
void WriteToFile(char*, float*, int, int, float, float);

int GetNextPower2(int);
int GetExponent2(int);
void DrawQuad(float, float, float, float, float , float, float, float);
void DrawQuad(float, float);
void DrawBox(Float3, Float3, Float4);
void DrawFloor(float, float, float, float, float, bool);
void DrawPlane(float, float, float, float);
void DrawPlane(double*,Float4);
double GetLOD();
void BSplineBasis(int, float, int, float*, float*);
void BSplineBasisDerv(int, float, int, float*, float*);
float CubicBSplineBasis(float, float*);
void MarkClosestPoints(Float3, Float3, Float3, bool markPoints = false);

inline double ViewMatrixScaleMagnitude(float*);
inline float DistanceR7(float*, float*);
float PolylineDistance(int, int, int, int, float*, int*);
float SqrPolylineDistanceP1(int, int, int, int, float*, int*);
void DrawBackground(Float4, Float4);
bool ClosestPointOnPlane(float[3], float[3], float[3], float[3], Float3*);
bool AABBIntersection(Float3, Float3, Float3, Float3);

void Convert2Matrix(Float3 v[3], float m[9]);
void Convert2Vector(float m[9], Float3 v[3]);
void MakeIdentityMatrix(float m[16]);
void InvertMatrix(float src[16], float inv[16]);
void InvertMatrix3(float src[9], float inv[9]);
void TransposeMatrix(float src[16], float trans[16]);
void TransposeMatrix3(float src[16], float trans[16]);
void MultMatrix(float A[9], float B[9], float m[9]);
Float3 GetPerpendicular(Float3);

int tri_tri_intersect_with_isectline(float V0[3], float V1[3], float V2[3], float U0[3], float U1[3], float U2[3], int *coplanar, float isectpt1[3], float isectpt2[3]);
int NoDivTriTriIsect(float V0[3], float V1[3], float V2[3], float U0[3], float U1[3], float U2[3]);
int TriBoxOverlap(float boxcenter[3], float boxhalfsize[3], float triverts[3][3]);

Float3 GetBarycentricCoordinates(Float3, float[3],float[3], float[3]);
int PolylineDistCompare(const void * a, const void * b);
void DumpPolylineData(vector<list<int>>, float*, int);
int FindClosestNeighbour(int, int*, float*, int, int);

bool OBBOverlap(BBoxData*, BBoxData*, float[16], float[16], float, float);
bool OBBOverlap(BBoxData*, BBoxData*, float[16]);

int UnlockACIS();

inline float TetVolume(Float3 v0, Float3 v1, Float3 v2, Float3 v3)
{
	Float3 v10 = v0 - v1;
	Float3 v20 = v0 - v2;
	Float3 v30 = v0 - v3;
	float tetVolume = v10[0]*v20[1]*v30[2] - v10[0]*v20[2]*v30[1] + v10[1]*v20[2]*v30[0] - v10[1]*v20[0]*v30[2] + v10[2]*v20[0]*v30[1] - v10[2]*v20[1]*v30[0];
	return tetVolume/6.0;
}

inline double ViewMatrixScaleMagnitude(float* M)
{
	double scale =  M[15]*(M[0]*M[5]*M[10] - M[0]*M[6]*M[9] - M[1]*M[4]*M[10] + M[1]*M[6]*M[8] + M[2]*M[4]*M[9] - M[2]*M[5]*M[8]);
	return pow(scale,(1/3.0));
}

inline bool AlmostNearPoints( Float2 a, Float2 b)
{
	return (max(fabs(a[0]-b[0]) , fabs(a[1]-b[1])) <= 2+DELTA);
//	return ((fabs(a[0]-b[0]) + fabs(a[1]-b[1])) <= 1+DELTA);
}

inline bool NearPoints( Float2 a, Float2 b)
{
	return (max(fabs(a[0]-b[0]) , fabs(a[1]-b[1])) <= 1+DELTA);
//	return ((fabs(a[0]-b[0]) + fabs(a[1]-b[1])) <= 1+DELTA);
}

inline float Distance( Float2 a, Float2 b)
{
	return (fabs(a[0]-b[0]) + fabs(a[1]-b[1]));
}

inline float Distance( Float3 a, Float3 b)
{
	return sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

inline float SqrDistance( Float3 a, Float3 b)
{
	return ((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

inline float Distance(float* p1, float* p2)
{
	return sqrt( (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) );
}

inline float DistanceR7(float* p1, float* p2)
{
	return sqrt(	(p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]) + (p1[2]-p2[2])*(p1[2]-p2[2]) + (p1[3]-p2[3])*(p1[3]-p2[3]) +
					(p1[4]-p2[4])*(p1[4]-p2[4]) + (p1[5]-p2[5])*(p1[5]-p2[5]) + (p1[6]-p2[6])*(p1[6]-p2[6]) );
}

inline float SqrDistanceR2(Float2 p1, Float2 p2)
{
	return (p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]);
}

inline void CopyTransformationMatrix(float src[16], float dst[16])
{
	for (int i = 0; i < 16; i++)
		dst[i] = src[i];
}

inline int GetNextDiv4(int a)
{
	return (a%4==0) ? a: a + (4-a%4);
}


inline int GetNextDiv2(int a)
{
	return (a%2==0) ? a : a+1;
}