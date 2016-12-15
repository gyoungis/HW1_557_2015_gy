#pragma once
#include "Utilities.h"
#include "NURBS.h"
#include "GLParameters.h"

class Face
{
public:
	Face() {visibilityFactor = 0; this->isVertexColored = false; this->isNURBS = false; this->isMarked = false; moment0 = 0; moment1[0] = 0; moment1[1] = 0; moment1[2] = 0;};
	~Face() {triangles.clear();if(this->isNURBS) delete surface;};

	vector<Triangle> triangles;
	vector<vector<int>> vertexFaces;

	bool trimmed;
	GLuint trimTexture;
	int trimWidth,trimHeight;
	Float3 kdColor;						// Diffuse Color
	Float3 ksColor;						// Specular Color
	float ka;							// Ambient factor
//	float transparency;					// Transparency
	float shininess;					// Shininess
	float visibilityFactor;

	bool isVertexColored;
	bool isNURBS;
	NURBS* surface;
	Float3 bBoxMin;					// Face bounding box Min point
	Float3 bBoxMax;					// Face bounding box Max point

	GLuint dlid; // display list id
	int parentObjID;
	int surfID;

	void DrawFace(GLParameters*, bool, float);		// draw the object
	void DrawOBB();
	float GetDistanceFromSphere(float* viewMatrix);
	int GetCommonFace(int, int, int);
	void CalculateStructureTensor();

	bool isMarked;

	float moment0;				// Volume contribution due to surface
	Float3 moment1;				// x,y,z-Moment contribution due to surface
};

inline bool FACEgreaterTest(const Face p1, const Face p2)
{
	return (p1.visibilityFactor > p2.visibilityFactor);
}

class Object
{
public:
	Object() {voxelData = NULL; dlid = rand(); visibilityFactor = 0; totalNumTriangles = 0; MakeIdentityMatrix(this->transformationMatrix); this->identityTransformation = true; volume = 0; ACISVolume = 0; massCenterComputed = false;};
	Object(const Object &that);
	~Object() {if (voxelData!=NULL) delete voxelData; faces.clear(); /*transforms.clear();*/};

	vector<Face*> faces;
	GLuint dlid;
	float visibilityFactor;
	int objID;
	int totalNumTriangles;

	float transformationMatrix[16];	// Matrix used for transformation
	bool identityTransformation;	// True if Identity
	Float3 bBoxMin;					// Object bounding box Min point
	Float3 bBoxMax;					// Object bounding box Max point
	float maxModelSize;				// Size of the model
	VoxelData* voxelData;			// Voxellised data of the object
	GLuint voxelDLid;				// Voxel data display list ID

	Object &operator=(const Object &that);
	void ReadObject(char *fname);
	void ReadObject2(char *fname);
	void CreateDisplayLists(GLParameters*);
	void DrawSceneObject(GLParameters*, bool, float);		// draw the object with scene transformations
	void DrawObject(GLParameters*, bool, float);			// draw the object without transformations
	void CreateDisplayListObject(GLParameters*);
	void ClassifyInOut(GLParameters*);
	void ClassifyInOut2x(GLParameters*);
	void ClassifyInOut2xLevel2(GLParameters*, int, int);
	void ClassifyTessellation(GLParameters*);
	void DrawInOutPoints();
	void PerformVoxelization(GLParameters*);
	void PerformVoxelizationLevel2(GLParameters*);
	void SaveVoxelization(GLParameters*);
	void BuildHierarchy(GLParameters*);
	void AddPointToVoxel(int, int, int, int, int);
	void AddNormal(int, int);								//Add surface Normals
	bool TestTriBox(int, int, int, Triangle*);
	void DrawVoxels(GLParameters*);
	void GenVoxelsDisplayLists(GLParameters*);
	void DrawVoxelHierarchy(GLParameters*);
	void DrawOBB();
	void DrawFaceBoundingBoxes(GLParameters*);
	void DrawCOM();

	float CalculateVolume(bool);	//Calculate the volume of the object bool for timing

	bool massCenterComputed;		// Flag to indicate computing center of mass
	float volume;					// Volume of the object
	Float3 massCenter;				// Center of mass of the object

	float ACISVolume;				// ACIS Volume of the object
	Float3 ACISMassCenter;			// ACIS center of mass of the object
};

float GetFaceFaceClosestPoint(Face*, Face*, float*, float*, float*, float*);