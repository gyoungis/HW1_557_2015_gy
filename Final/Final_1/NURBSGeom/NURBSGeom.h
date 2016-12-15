#ifndef NURBSGEOM_H
#define NURBSGEOM_H

/**
@file NURBSGeom.h

Main header file for the NURBSGeom library.
*/

#include <assert.h>

extern const double NG_EPS2;
extern const double NG_INF;

//Error values.
extern const int NG_NO_ERR;
extern const int NG_ERR;


#include "NGPoint.h"
#include "NGKnotVector.h"
#include "NGBasis.h"
#include "NGCurve.h"
#include "NGSurface.h"

typedef NGPoint<2> NGPoint2;
typedef NGPoint<3> NGPoint3;

#endif //NURBSGEOM_H

