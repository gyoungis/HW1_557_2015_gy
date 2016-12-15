#include "NURBS.h"

/*
//local utilities (working on NURBSGeom geometry)
#include <NURBSGeom.h>


typedef std::pair<double, double> uvPair;

int EvaluateBoundingCone(const std::vector<NGPoint3 >& vecs,
									  NGPoint3& rAxis, double& rCosAngle);
bool ShouldRefineSurface(const NGSurface& srf, double flatnessTol, std::map<uvPair, NGPoint3>& uvNormalsMap);
void SplitNGSurfaceIntoFlatBezPatches(NGSurface& ngSrf, std::vector<NGSurface>& bezPatches, double flatnessTol);
void BuildBezTextureData(const NGSurface& srf, float* bezData);
*/

// Function that gets a NURBS as input and outputs textures corresponding to flat Bezier patches.
// Uses the NURBSGeom library.
// flatnessTol is the maximal angle span (in angles!) of the normals of a patch, to be considered
// flat. If flatnessTol is negative, the split is only performed to Bezier patches.
void SplitSurfaceIntoFlatBezPatches(NURBS* nurbs, std::vector<float*>& bezTexData, double flatnessTol);


//Given a surface S and du,dv, Ks is a bound on:
//||S(u,v)-S(u+du,v+dv)||
//If du,dv are sampling steps, this gives a bound on the maximal distance between
//any point on the surface and its nearest sample point.
double ComputeKsBound(NURBS* nurbs, double du, double dv);
