#include "Includes.h"
#include "NURBS.h"
#include "Object.h"

class Node2D
{
public:
	Node2D(){nodeNumber = -1;};
	~Node2D(){};

	int nodeNumber;
	string label;
	Float3 vertexVal;
	Float3 vertexDu;
	Float3 vertexDv;
	Float3 vertexDuDv;
};

class Node3D
{
public:
	Node3D(){nodeNumber = -1;};
	~Node3D(){};

	int nodeNumber;
	string label;
	Float3 vertexVal;
	Float3 vertexDu;
	Float3 vertexDv;
	Float3 vertexDw;
	Float3 vertexDuDv;
	Float3 vertexDvDw;
	Float3 vertexDwDu;
	Float3 vertexDuDvDw;
};


class Hermite2DPatch
{
public:
	Hermite2DPatch();
	~Hermite2DPatch();

	//CPU Evaluations
//	Float3 Evaluate(float, float);						// Evaluates the 3 components of the Hermite surface
//	Float4 EvaluateuDerv(float, float);					// Evaluates the 3 components of the Hermite surface u derivative
//	Float4 EvaluatevDerv(float, float);					// Evaluates the 3 components of the Hermite surface u derivative
//	Float3 EvaluateNormal(float, float);				// Evaluates the 3 components of the Hermite surface normal

	void ConvertToNURBS(NURBS*);						// Converts Hermite patch to NURBS and stores data in the NURBS*

	float transformationMatrix[16];			// Matrix used for transformation
	bool identityTransformation;			// True if the transformation is identity
	Float3 kdColor;							// Diffuse Color
	Float3 ksColor;							// Specular Color
	float ka;								// Ambient factor
//	float transparency;						// Transparency
	float shininess;						// Shininess
	
	// Control Points
	Float3 vertexVal[4];
	Float3 vertexDu[4];
	Float3 vertexDv[4];
	Float3 vertexDuDv[4];

	bool initialized;

};

class Element2D
{
public:
	Element2D(){elementNumber = -1;};
	~Element2D(){};

	int nodes[4];
	string label;
	int elementNumber;
};

class SFM2D
{
public:
	SFM2D(){};
	~SFM2D(){};

	Float3 val;
	Float3 du;
	Float3 dv;
	Float3 dudv;
	int elementNumber;
	int elemNodeNumber;
};

class Hermite2DPatchMesh
{
public:
	Hermite2DPatchMesh();
	~Hermite2DPatchMesh();

	bool initialized;
	vector<Element2D*> elements;
	vector<Node2D*> nodes;
	vector<SFM2D*> sfm;
	vector<Hermite2DPatch*> patches;
};

class Element3D
{
public:
	Element3D(){elementNumber = -1;};
	~Element3D(){};

	int nodes[8];
	string label;
	int elementNumber;
};

class LocalDOF
{
public:
	LocalDOF(){};
	~LocalDOF(){};

	Float3 val;
	Float3 du;
	Float3 dv;
	Float3 dw;
	Float3 dudv;
	Float3 dvdw;
	Float3 dwdu;
	Float3 dudvdw;
	int elementNumber;
	int elemNodeNumber;
};

class Local2GlobalMap
{
public:
	Local2GlobalMap(){};
	~Local2GlobalMap(){};
	void GenerateMap(Float3 localVec[3], Float3 globalVec[3], float derMap[9]);

	float der1Map[9];
	float der2Map[9];
	float der3Map;
	int elementNumber;
	int elemNodeNumber;
};


class Hermite3DMesh
{
public:
	Hermite3DMesh();
	~Hermite3DMesh();

	// Initialize LocalDOF using global to local map
	void ConvertGlobal2Local();
	void SaveMeshDisplayFaces(char* fileName);

	bool initialized;
	vector<Element3D*> elements;
	vector<Node3D*> nodes;
	vector<LocalDOF*> localDOF;
	vector<Local2GlobalMap*> meshGlobal2LocalMap;
	vector<int> displayElements;
	vector<int> displayFaces;

	vector<Hermite2DPatch*> displayPatches;
	vector<NURBS*> nurbsPatches;
};