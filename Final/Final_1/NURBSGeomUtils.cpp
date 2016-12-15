#include "NURBSGeomUtils.h"
#include "NURBSGeom\NURBSGeom.h"

#include <map>
#include <deque>

typedef std::pair<double, double> uvPair;

// Evaluate bounding cone of a set of vectors, 
// returns NG_ERR if opening angle is greater than 90.
int EvaluateBoundingCone(const std::vector<NGPoint3 >& vecs,
						 NGPoint3& rAxis, double& rCosAngle)
{
	int res = NG_NO_ERR;
	assert(!vecs.empty());

	std::vector<NGPoint3 > vecsNorm(vecs.size()); // Will hold the normalized vectors.
	NGPoint3 Axis; // Initializes to (0,..0)
	size_t i=0;
	for (i=0; i<vecsNorm.size(); ++i)
	{
		vecsNorm[i] = vecs[i].Normalize();
		Axis += vecsNorm[i];
	}
	rAxis = Axis.Normalize();
	// Now rAxis holds the normalized sum of normalized vectors (the "average" vector).

	rCosAngle = 1.0;
	for (i=0; i<vecsNorm.size(); ++i)
	{
		double dotProd = rAxis.InnerProd(vecsNorm[i]);
		if (dotProd < 0.0)
			return NG_ERR;
		if (dotProd < rCosAngle)
			rCosAngle = dotProd;
	}

	return res;
}


bool ShouldRefineSurface(const NGSurface& srf, double flatnessTol, std::map<uvPair, NGPoint3>& uvNormalsMap)
{
	bool shouldRefineSrf = false;

	//note: if flatnessTol is negative, don't refine beyond basic Bezier patches
	if (flatnessTol > 0.0)
	{
		//Evaluate normals at points.
		int err = NG_NO_ERR;
		std::vector<NGPoint3 > normals;
		NGPoint3 vec3d;
		std::map<uvPair, NGPoint3 >::const_iterator findIter;

		double minU, maxU, minV, maxV;
		int res = srf.GetDomain(minU, maxU, minV, maxV);
		assert(res == NG_NO_ERR);

		findIter = uvNormalsMap.find(std::make_pair(minU,minV));
		if (findIter != uvNormalsMap.end())
		{
			vec3d = findIter->second;
		}
		else
		{
			err = srf.EvaluateNormalAt(minU, minV, vec3d.GetCArray());
			assert(err == NG_NO_ERR);

			uvNormalsMap[std::make_pair(minU,minV)] = vec3d;
		}
		normals.push_back(vec3d);


		findIter = uvNormalsMap.find(std::make_pair(maxU,minV));
		if (findIter != uvNormalsMap.end())
		{
			vec3d = findIter->second;
		}
		else
		{
			err = srf.EvaluateNormalAt(maxU, minV, vec3d.GetCArray());
			assert(err == NG_NO_ERR);

			uvNormalsMap[std::make_pair(maxU,minV)] = vec3d;
		}
		normals.push_back(vec3d);


		findIter = uvNormalsMap.find(std::make_pair(minU,maxV));
		if (findIter != uvNormalsMap.end())
		{
			vec3d = findIter->second;
		}
		else
		{
			err = srf.EvaluateNormalAt(minU, maxV, vec3d.GetCArray());
			assert(err == NG_NO_ERR);

			uvNormalsMap[std::make_pair(minU,maxV)] = vec3d;
		}
		normals.push_back(vec3d);


		findIter = uvNormalsMap.find(std::make_pair(maxU,maxV));
		if (findIter != uvNormalsMap.end())
		{
			vec3d = findIter->second;
		}
		else
		{
			err = srf.EvaluateNormalAt(maxU, maxV, vec3d.GetCArray());
			assert(err == NG_NO_ERR);

			uvNormalsMap[std::make_pair(maxU,maxV)] = vec3d;
		}
		normals.push_back(vec3d);


		double uMid = 0.5*(minU+maxU);
		double vMid = 0.5*(minV+maxV);
		err = srf.EvaluateNormalAt(uMid, vMid, vec3d.GetCArray());
		assert(err == NG_NO_ERR);
		normals.push_back(vec3d);

		NGPoint3 axis;
		double cosAngle;
		int coneRes = EvaluateBoundingCone(normals, axis, cosAngle);

		// Check for recursion termination criteria.
		double angleSpan = (45.0/(atan(1.0)))*acos(cosAngle);
		if (!(coneRes == NG_NO_ERR) || angleSpan > flatnessTol)
			shouldRefineSrf = true;

		/*
		//Additional check for small srfs. is this needed?
		//ToDo: possibly make it a 3d check.
		//ToDo: possibly make a check of the edges (thin triangles) instead of radius.
		double uMin, uMax, vMin, vMax;
		m_Srf.GetDomain(uMin, uMax, vMin, vMax);
		//For thin domains need to compare bounding radius with smaller side.
		//ToDo: Make this check more sophisticated (e.g., based on bbox of loops).
		double minSide = std::min((uMax-uMin),(vMax-vMin));
		if ((u4-u1)*(u4-u1) + (v4-v1)*(v4-v1) < 0.01*(minSide*minSide))
		shouldRefineTriangle = false; //ToDo: make constant depend on user.
		*/
	}

	return shouldRefineSrf;
}


void SplitNGSurfaceIntoFlatBezPatches(NGSurface& ngSrf, std::vector<NGSurface>& bezPatches, double flatnessTol)
{
	//split at knots for Bezier patches:
	std::vector<NGSurface> tmp;
	ngSrf.SplitAtKnots(tmp);

	//start splitting until flatness tolerance is acheived
	std::map<uvPair, NGPoint3> uvNormalsMap;

	std::deque<NGSurface> srfQueue(tmp.begin(), tmp.end()); //TODO - use iterators in SplitAtKnots, or some other nicer solution that doesn't copy..

	while(!srfQueue.empty()) {
		const NGSurface& srf = srfQueue.front();
		bool shouldRefine = ShouldRefineSurface(srf, flatnessTol, uvNormalsMap);

		//iddo debug - just split to Bezier patches (no flatness criterion) - can now be done by assigning negative flatnessTol
		//shouldRefine = false;

		if (shouldRefine) {
			double minU, maxU, minV, maxV;
			int res = srf.GetDomain(minU, maxU, minV, maxV);
			assert(res == NG_NO_ERR);

			NGSurface srf1, srf2;
			if (maxU-minU > maxV-minV) {
				res = srf.SubdivideAtU(0.5*(minU+maxU), srf1, srf2);
			}
			else {
				res = srf.SubdivideAtV(0.5*(minV+maxV), srf1, srf2);
			}
			assert(res == NG_NO_ERR);

			srfQueue.push_back(srf1);
			srfQueue.push_back(srf2);
		}
		else {
			bezPatches.push_back(srf);
		}

		srfQueue.pop_front(); //remove srf from queue (it was either refined or copied to output.
	}
}


//copies the control net into bezData
//bezData should be allocated outside to (srf->DegreeU()+1)*(srf->DegreeU()+1)*4
void BuildBezTextureData(const NGSurface& srf, float* bezData)
{
	unsigned orderU = srf.DegreeU() + 1; 
	unsigned orderV = srf.DegreeV() + 1;
	unsigned i,j;

	for (i=0; i<orderU; ++i)
		for (j=0; j<orderV; ++j) {
			//iddo: do we fill the texture like this (this is how it's done in GPView so far)
			//or fill [(i*orderV+j)*4+0] - as long as we're consistent it shouldn't matter.. 
			bezData[(i+orderU*j)*4 + 0] =  srf.GetControlPointPCoord(i,j,0);
			bezData[(i+orderU*j)*4 + 1] =  srf.GetControlPointPCoord(i,j,1);
			bezData[(i+orderU*j)*4 + 2] =  srf.GetControlPointPCoord(i,j,2);
			bezData[(i+orderU*j)*4 + 3] =  srf.GetControlPointW(i,j); //we set w=1 for non-rational
		}
}

//note: before bezTexData is destructed, the floats should be deleted[].
//iddo: maybe implement using vector<vector<float> >..
void SplitSurfaceIntoFlatBezPatches(NURBS* nurbs, std::vector<float*>& bezTexData, double flatnessTol)
{
	//convert nurbs to ngSrf
	float* cntlPoints = nurbs->cntlPoints;						// Control Points array
	float* uKnotVector = nurbs->uKnotVector;						// Knot Vector in the u direction
	float* vKnotVector = nurbs->vKnotVector;						// Knot Vector in the v direction
	int uOrder = nurbs->uOrder;								// Order in the u direction
	int vOrder = nurbs->vOrder;								// Order in the v direction
	int uPoints = nurbs->uPoints;							// Number of Control points in the u direction
	int vPoints = nurbs->vPoints;							// Number of Control points in the v direction

	NGSurface ngSrf(3,uOrder-1, vOrder-1, uPoints, vPoints, true); //projective surface (with w) in R3 
	//setting cps
	int i,j,coord;
	for (i=0; i<uPoints; ++i) {
		for (j=0; j<vPoints; ++j) {
			for (coord=0; coord<3; ++coord) {
				double val = cntlPoints[j*uPoints*4+i*4+coord];
				ngSrf.SetControlPointPCoord(i,j,coord,val);
			}
			double w = cntlPoints[j*uPoints*4+i*4+3];
			ngSrf.SetControlPointW(i,j,w);
		}
	}

	for (i = 0; i < uPoints + uOrder; ++i)
		ngSrf.SetKnotValue(true,i,uKnotVector[i]);
	for (j = 0; j < vPoints + vOrder; ++j)
		ngSrf.SetKnotValue(false,j,vKnotVector[j]);

	std::vector<NGSurface> bezPatches;
	SplitNGSurfaceIntoFlatBezPatches(ngSrf, bezPatches, flatnessTol);

	//copy bezPatches to bezTexData (allocate memory for bezTexData - free it after textures are
	//constructed and before vector is destructed)
	for (i=0; i<bezPatches.size(); ++i) {
		float* bezData = new float[uOrder*vOrder*4];
		BuildBezTextureData(bezPatches[i], bezData);
		bezTexData.push_back(bezData);
	}
}


/////////////////////////
//Iddo: The following functions are for computing a bound Ks on a surface S.
//Given a surface S and du,dv, Ks is a bound on:
//||S(u,v)-S(u+du,v+dv)||

//ComputeKsBoundNGAux computes the bound from the paper on the whole NURBS surface
//ComputeKsBoundNG splits the NURBS into (possibly flat) Bezier patches and takes the maximal bound.
//These functions work on NGSurface as input (note: they are not implemented for rational surfaces yet).
//ComputeKsBound_OLD is an I/F that wraps these functions.
//The methods above give rather large bounds, therefore I implemented a new
//ComputeKsBound, which is based on the expanded bounding boxes, computed by CUDA,
//and taking the largest box radius as the bound. This gives the best results.


//Given a surface S and du,dv, Ks is a bound on:
//||S(u,v)-S(u+du,v+dv)||
//If du,dv are sampling steps, this gives a bound on the maximal distance between
//any point on the surface and its nearest sample point.
//We compute Ks with the following formula (see paper):
//Ks = du*(max(Su_x)+max(Su_y)+max(Su_z)) + dv*(max(Sv_x)+max(Sv_y)+max(Sv_z))
double ComputeKsBoundNGAux(const NGSurface& S, double du, double dv)
{
	NGSurface Su, Sv;
	double minBox[3] = {0,0,0}, maxBox[3] = {0,0,0};
	int res;
	res = S.PartialDerivSurface(true, Su);
	assert(res == NG_NO_ERR);
	res = S.PartialDerivSurface(false, Sv);
	assert(res == NG_NO_ERR);

	//iddo debug
	//  S.SaveITD("S1.itd");
	//  Su.SaveITD("Su1.itd");
	//  Sv.SaveITD("Sv1.itd");

	double Ks = -1.0;
	Su.GetBBox(minBox, maxBox);
	Ks = du*(maxBox[0]+maxBox[1]+maxBox[2]);

	Sv.GetBBox(minBox, maxBox);
	Ks += dv*(maxBox[0]+maxBox[1]+maxBox[2]);

	return Ks;
}


double ComputeKsBoundNG(const NGSurface& S, double du, double dv)
{
	unsigned i;

	//testing
	//double K = ComputeKsBoundNGAux(S, du, dv); return K;

	double flatnessTol = -20.0;
	std::vector<NGSurface> bezPatches;
	NGSurface Stmp (S); //so as not to modify S
	SplitNGSurfaceIntoFlatBezPatches(Stmp, bezPatches, flatnessTol);

	double Ks = ComputeKsBoundNGAux(bezPatches[0], du, dv);
	for (i=1; i<bezPatches.size(); ++i) {
		double Ki = ComputeKsBoundNGAux(bezPatches[i], du, dv);

		//for Bezier patches in NR - need to scale du,dv by knots
		//double minU, maxU, minV, maxV;
		//bezPatches[i].GetDomain(minU, maxU, minV, maxV);
		//double Ki = ComputeKsBoundNGAux(bezPatches[i], 0.01*(maxU-minU), 0.01*(maxV-minV)); 

		if (Ki > Ks) Ks = Ki;
	}

	return Ks;
}

double ComputeKsBound_OLD(NURBS* nurbs, double du, double dv)
{
	int i,j,coord;

	//convert nurbs to ngSrf
	float* cntlPoints = nurbs->cntlPoints;						// Control Points array
	float* uKnotVector = nurbs->uKnotVector;						// Knot Vector in the u direction
	float* vKnotVector = nurbs->vKnotVector;						// Knot Vector in the v direction
	int uOrder = nurbs->uOrder;								// Order in the u direction
	int vOrder = nurbs->vOrder;								// Order in the v direction
	int uPoints = nurbs->uPoints;							// Number of Control points in the u direction
	int vPoints = nurbs->vPoints;							// Number of Control points in the v direction

	bool isRational = false;
	double w00 = cntlPoints[3];
	for (i=0; i<uPoints; ++i) {
		for (j=0; j<vPoints; ++j) {
			double w = cntlPoints[j*uPoints*4+i*4+3];
			if (fabs(w - w00) > NG_EPS2) {
				isRational = true;
				//break;
				assert(false);
				return -1; //not implemented yet
			}
		}
		if (isRational) break;
	}

	if (!isRational) {
		assert(fabs(1.0 - w00) < NG_EPS2); //iddo: do we need to scale if w's are not 1?
	}

	//NGSurface ngSrf(3,uOrder-1, vOrder-1, uPoints, vPoints, true); //projective surface (with w) in R3 
	NGSurface ngSrf(3,uOrder-1, vOrder-1, uPoints, vPoints, isRational); //surface in R3 

	//setting cps
	for (i=0; i<uPoints; ++i) {
		for (j=0; j<vPoints; ++j) {
			for (coord=0; coord<3; ++coord) {
				double val = cntlPoints[j*uPoints*4+i*4+coord];
				ngSrf.SetControlPointPCoord(i,j,coord,val);
			}
			if (isRational) {
				double w = cntlPoints[j*uPoints*4+i*4+3];
				ngSrf.SetControlPointW(i,j,w);
			}
		}
	}

	for (i = 0; i < uPoints + uOrder; ++i)
		ngSrf.SetKnotValue(true,i,uKnotVector[i]);
	for (j = 0; j < vPoints + vOrder; ++j)
		ngSrf.SetKnotValue(false,j,vKnotVector[j]);

	// Now we have an NGSurface we can compute Ks using the auxilary function
	double Ks = ComputeKsBoundNG(ngSrf, du, dv);
	return Ks;
}

//New implementation based on CUDASurfaceBoundingBoxEvaluation
//TODO: move it to a different location, if it doesn't use NGlib doesn't need to be here.
double ComputeKsBound(NURBS* nurbs, double du, double dv)
{
	int i,j;
	//	int uNum	= nurbs->evalParams->uNum;
	//	int vNum	= nurbs->evalParams->vNum;
	int uNum	= int(1.0/du);
	int vNum	= int(1.0/dv);

	//iddo: no assumptions on previous point evaluation - maybe should work on a copy surface so not to modify surface.
	if (nurbs->evalParams->minPoints != NULL)
		delete [] nurbs->evalParams->minPoints;
	nurbs->evalParams->minPoints		= new float[(uNum)*(vNum)*4];
	if (nurbs->evalParams->maxPoints != NULL)
		delete [] nurbs->evalParams->maxPoints;
	nurbs->evalParams->maxPoints		= new float[(uNum)*(vNum)*4];

#ifdef CUDA
	nurbs->CUDASurfaceEvaluation(true, true, true, false);//exact normals, second derivs, readback, no render
	nurbs->CUDASurfaceBoundingBoxEvaluation(false, true, true); 
#endif 

	float *maxPts = nurbs->evalParams->maxPoints;
	float *minPts = nurbs->evalParams->minPoints;
	double KsSq = (maxPts[0]-minPts[0])*(maxPts[0]-minPts[0]) + 
		(maxPts[1]-minPts[1])*(maxPts[1]-minPts[1]) +
		(maxPts[2]-minPts[2])*(maxPts[2]-minPts[2]);

	for (i=0; i<uNum+1; ++i)
	{
		for (j=0; j<vNum+1; ++j)
		{
			int idx = i*(vNum+1) + j;
			double KijSq = (maxPts[idx]-minPts[idx])*(maxPts[idx]-minPts[idx]) + 
				(maxPts[idx+1]-minPts[idx+1])*(maxPts[idx+1]-minPts[idx+1]) +
				(maxPts[idx+2]-minPts[idx+2])*(maxPts[idx+2]-minPts[idx+2]);
			if (KijSq > KsSq)
				KsSq = KijSq;
		}
	}

	return 0.5*sqrt(KsSq);
}

