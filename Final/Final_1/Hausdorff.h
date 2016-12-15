#include "Includes.h"
#include "NURBS.h"
#include "Object.h"

float HausdorffData(NURBS*, NURBS*, int, int, int, int, Float3*, Float3*);
float GPUHausdorffDistance(NURBS* surface1, NURBS* surface2, float*, float*, GLParameters* glParam);
float CPUHausdorffDistance(NURBS* surface1, NURBS* surface2, float*, float*, GLParameters* glParam);
float GetSurfaceHausdorffDistance(NURBS*, NURBS*, float*, float*, int, GLParameters*, bool);
