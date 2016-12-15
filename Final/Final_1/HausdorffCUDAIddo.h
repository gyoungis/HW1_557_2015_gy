#include "Includes.h"
#include "NURBS.h"
#include "Object.h"

//iddo:
//float IddoGetCUDASurfaceHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, int intRes, GLParameters* glParam, bool forcedEvaluate);
#ifdef CUDA
float IddoGetCUDASurfaceHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, float* convergenceStatus, int intRes, float kb, GLParameters* glParam, bool forcedEvaluate);
float GetMaxDistanceInTexture(float4* cudaRes, int uNum, int vNum, int& iMax, int& jMax, float point2[3]);
void ConstructBezDataTex(NURBS* surface2, double flatnessTol);
void InitializeFBOTextures(float4* fboTexs[2], float* convTexs[2], int uNum, int vNum);
void InitializeFBOTextures(float4* fboTexs[2], int uNum, int vNum);
#endif