#include "Includes.h"
#include "NURBS.h"
#include "Object.h"

float CUDAHausdorffData(NURBS*, NURBS*, int, int, int, int, Float3*, Float3*);
float GetCUDASurfaceHausdorffDistance(NURBS*, NURBS*, float*, float*, int, GLParameters*, bool);
float CUDAHausdorffDistance(NURBS* surface1, NURBS* surface2, float* point1, float* point2, bool readBack, bool renderTextureOutput);

