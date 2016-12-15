#include "Includes.h"
#include "NURBS.h"
#include "Object.h"

int FindClosestNeighbour(int, int*, float*, int, int);
vector<list<int>> FitPolylines(float*, int, int);
bool DetectCollision(NURBS*, NURBS*, int, GLParameters*, bool);
void CalculateSurfaceIntersection(NURBS*, NURBS*, int, GLParameters*, bool);
void CalculateSurfaceIntersection(vector<NURBS*>*, int, GLParameters*);
void ObjectCollision(Object*, Object*, GLParameters*);

bool ExtractIntersectionData(NURBS*, NURBS*,int, int, int, int, float*);
int GPUSurfaceIntersection(NURBS*, NURBS*, GLParameters*, float* &x, float* &y, float* &z, int,  Float4* timeData = NULL);

int ComputeCollision(NURBS*, NURBS*, GLParameters*, float* &x, float* &y, float* &z, int, bool, Float4* timeData = NULL);
void GetClickParametricPoint(vector<NURBS*>*, float, GLParameters*); // float   camera.nearcp closestZVal
void GetClickParametricPoint(vector<Object*>*, float, GLParameters*);

float ClosestPointData(NURBS*, NURBS*, int, int, int, int, Float3*, Float3*);
float GPUSurfaceClosestPoint(NURBS* surface1, NURBS* surface2, float*, float*, GLParameters* glParam);
float GetSurfaceClosestPoint(NURBS*, NURBS*, float*, float*, int, GLParameters*, bool);

float GetObjectClosestPoint(Object*, Object*, float*, float*, float*, int, GLParameters*, bool);
vector<Float2> GPUObjectClosestPoint(Object*, Object*, GLParameters*);

void InteractiveLOD(vector<NURBS*>, vector<Object*>, Camera, GLParameters*);
void DrawSurfaceIDNURBS(NURBS*, GLParameters*);
void GenerateSurfaceID(vector<NURBS*>, vector<Object*>, Camera, Viewport, GLuint, GLParameters*);
