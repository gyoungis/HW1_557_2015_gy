#include "Includes.h"
#include "NURBS.h"
#include "Object.h"

Float4 GetGPUObjectMoment(Object*, int, int, int, GLParameters*, Float4*, bool, bool, bool, bool);
Float4 GetSurfaceMoment(NURBS*, int, int, int, int, GLParameters*, Float4*, bool, bool);		// Boolean error analysis and forced evaluate
Float4 GetGPUTriangulatedMoment(float*, float*, float*, int, int, GLParameters*);
Float4 GetTriangulatedMoment(Face*);
Float4 UpdateObjectMoment(Object*, int, Float4*, GLParameters*);
