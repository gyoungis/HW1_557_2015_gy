#include "NURBSGeom.h"
#include "NGSurface.h"
#include "NGBasis.h"

#include <algorithm>

#pragma warning(disable:4996) //warning on fopen

const double NG_SINGULAR_NORMAL_OFFSET(1e-6);

NGSurface::NGSurface(	unsigned _dim, unsigned _uDegree, unsigned _vDegree,
					unsigned _uNumControlPoints, unsigned _vNumControlPoints,
					bool _projective):
m_dim(_dim), m_projective(_projective),
m_uDegree(_uDegree), m_vDegree(_vDegree), 
m_uKV(_uDegree+_uNumControlPoints+1), 
m_vKV(_vDegree+_vNumControlPoints+1)
{
	m_controlPoints.assign(_dim + (_projective ? 1 : 0), 
		std::vector<double>(_uNumControlPoints*_vNumControlPoints)); 
}

NGSurface::NGSurface(const NGSurface &srf):
m_dim(srf.m_dim), m_projective(srf.m_projective),
m_uDegree(srf.m_uDegree), m_vDegree(srf.m_vDegree), 
m_uKV(srf.m_uKV), m_vKV(srf.m_vKV), 
m_controlPoints(srf.m_controlPoints)
{}

int NGSurface::EvaluateAt(double u, double v, double* pcoords, double* w) const
{
	//if(	u<m_uKV[this->m_uDegree] || u>m_uKV[m_uKV.size()-m_uDegree-1] ||
	//	v<m_vKV[m_vDegree] || v>m_vKV[m_vKV.size()-m_vDegree-1] )
	//	return NG_ERR;//parameters exceed domain

	//if(t<m_kv[m_degree] || t>m_kv[m_kv.size()-m_degree-1])
	//	return NG_ERR;//parameter exceeds domain
	
	//A more tolerant function, we assume the user wanted to be inside the domain.
	//Possibly add a tolerance to the above check.
	if(u<m_uKV[m_uDegree])
		u = m_uKV[m_uDegree];
	if (u>m_uKV[m_uKV.size()-m_uDegree-1])
		u = m_uKV[m_uKV.size()-m_uDegree-1];
	if(v<m_vKV[m_vDegree])
		v = m_vKV[m_vDegree];
	if (v>m_vKV[m_vKV.size()-m_vDegree-1])
		v = m_vKV[m_vKV.size()-m_vDegree-1];

	unsigned i,j,k;

	// Using binary search to find relevant indices.
	NGKnotVector::const_iterator vIter = 
		std::lower_bound(m_vKV.begin()+m_vDegree, m_vKV.end()-m_vDegree-1, v);
	if ((*vIter>v) || (vIter==m_vKV.end()-m_vDegree-1)) //end case of last parameter (no cp corresponds to it)
		--vIter;
	//Since lower_bound returns the one-after if v is not in the array.We need
	//to take the one before.
	//The end parameter evaluation is a special case.

	unsigned endIndexV = (unsigned)(vIter - m_vKV.begin());
	unsigned indexV = endIndexV - m_vDegree;

	// For efficiency precompute B_j(v) (instead of computing it inside the loop).
	std::vector<double> Bv(m_vDegree+1);
	for(j=0;j<=m_vDegree;++j)
	{
		Bv[j] = NGBasis::Evaluate(v,j+indexV,m_vDegree,m_vKV);
	}

	// Initialize sum to be (0,...0).
	if (Projective())
		*w = 0;
	else
		*w = 1.0;
	for (j = 0; j<Dim(); ++j)
		pcoords[j] = 0;

	NGKnotVector::const_iterator uIter = 
		std::lower_bound(m_uKV.begin()+m_uDegree, m_uKV.end()-m_uDegree-1, u);
	if ( (*uIter > u) || (uIter==m_uKV.end()-m_uDegree-1))
		--uIter; 
	//Since lower_bound returns the one-after if u is not in the array.We need
	//to take the one before.
	//The end parameter evaluation is a special case.

	unsigned endIndexU = (unsigned)(uIter - m_uKV.begin());
	unsigned  indexU = endIndexU - m_uDegree;

	for(i=indexU; i<=endIndexU;++i)
	{
		double B_iu = NGBasis::Evaluate(u,i,m_uDegree,m_uKV);
		if (B_iu != 0.0)
		{
			for(j=indexV;j<=endIndexV;++j)
			{
				double B_jv = Bv[j-indexV];
				if (B_jv != 0.0)
				{
					for (k=0; k<Dim(); ++k)
						pcoords[k] += GetControlPointPCoord(i,j,k) * B_iu * B_jv;
					if (Projective())
						*w += GetControlPointW(i,j) * B_iu * B_jv;
				}
			}
		}
	}

	return NG_NO_ERR;
}

int NGSurface::EvaluateAt(double u, double v, double* cartesian_coords) const
{
	unsigned i = 0;
	double w = 1;
	int res = EvaluateAt(u, v, cartesian_coords, &w);
	if (res != NG_NO_ERR)
		return res;
	if (Projective())
	{
		for (i=0; i<Dim(); ++i)
			cartesian_coords[i] /= w;
	}

	return res;
}

int NGSurface::IsoCurve(bool uDirConst, double param, NGCurve& rOutCrv) const
{
	unsigned i,j,k;
	int res = NG_NO_ERR;

	unsigned numCPOut = uDirConst ? NumControlPointsV() : NumControlPointsU();
	unsigned numCPOther = uDirConst ? NumControlPointsU() : NumControlPointsV();
	const NGKnotVector& kvOut = uDirConst ? m_vKV : m_uKV;
	const NGKnotVector& kvOther = uDirConst ? m_uKV : m_vKV;

	// Constructing the out curve with kvOut knot vector.
	rOutCrv = NGCurve(Dim(),(unsigned)kvOut.size()-numCPOut-1, numCPOut, Projective());
	assert(kvOut.size()==rOutCrv.NumKnots());
	for (i=0; i<rOutCrv.NumKnots(); ++i)
		rOutCrv.SetKnotValue(i, kvOut[i]);


	// Constructing the out curve with kvOther knot vector.
	NGCurve otherCurvei(Dim(),(unsigned)kvOther.size()-numCPOther-1, numCPOther, Projective()); 
	assert(kvOther.size()==otherCurvei.NumKnots());
	for (i=0; i<otherCurvei.NumKnots(); ++i)
		otherCurvei.SetKnotValue(i, kvOther[i]);
	
	
	double* outCP = new double[Dim()]; //auxilary point for evaluation
	for (i=0; i<numCPOut; ++i)
	{
		//copy a series of control points as otherCurvei control points
		for(j=0; j<numCPOther; ++j)
		{
			if (!uDirConst)
				for (k=0; k<Dim()+(Projective() ? 1 : 0); ++k)
					otherCurvei.SetControlPointPCoord(j, k, GetControlPointPCoord(i,j,k));
			else
				for (k=0; k<Dim()+(Projective() ? 1 : 0); ++k)
					otherCurvei.SetControlPointPCoord(j, k, GetControlPointPCoord(j,i,k));
		}

		double w;
		res = otherCurvei.EvaluateAt(param, outCP, &w);
		if (res != NG_NO_ERR) return res;

		for (k=0; k<Dim(); ++k)
			rOutCrv.SetControlPointPCoord(i, k, outCP[k]);
		if (Projective())
			rOutCrv.SetControlPointW(i, w);
	}
	delete[] outCP;

	return res;
}

//Returns the partial derivative surface of the surface.
//Not applicable to rational surfaces.
int NGSurface::PartialDerivSurface(bool uDir, NGSurface& rDerivSurface) const
{
    unsigned i, j, k;

	//rational surfaces not supported yet.
	//In order to support rational surfaces a symbolic surface multiplication
	//is required to perform: (x'w-xw')/w^2
    if (Projective()) {
		assert(false);
    }

	const NGKnotVector& mod_kv = uDir ? m_uKV : m_vKV;
	const NGKnotVector& other_kv = uDir ? m_vKV : m_uKV;
	unsigned Degree = uDir ? DegreeU() : DegreeV();
	unsigned cpNumU = uDir ? NumControlPointsU()-1 : NumControlPointsU();
	unsigned cpNumV = uDir ? NumControlPointsV() : NumControlPointsV()-1;

	assert(Degree >= 1);

	if (uDir)
		rDerivSurface = NGSurface(Dim(), DegreeU()-1, DegreeV(), 
			NumControlPointsU()-1, NumControlPointsV(), false);
	else
		rDerivSurface = NGSurface(Dim(), DegreeU(), DegreeV()-1, 
			NumControlPointsU(), NumControlPointsV()-1, false);

    if (Degree >= 1) {
		double Denom = 0;
		for (i = 0; i < cpNumU; ++i)
		{
			for (j = 0; j < cpNumV; ++j) 
			{
				if (uDir)
					Denom = mod_kv[i + Degree + 1] - mod_kv[i + 1];
				else
					Denom = mod_kv[j + Degree + 1] - mod_kv[j + 1];


				if (fabs(Denom) < NG_EPS2)
					Denom = NG_EPS2;

				double ScaleFactor = (Degree/Denom);

				if (uDir)
				{
					for (k=0; k < Dim(); ++k)
					{
						rDerivSurface.SetControlPointPCoord(i, j, k, (GetControlPointCoord(i+1,j,k) - 
							GetControlPointPCoord(i,j,k)) * ScaleFactor);
					}
				}
				else
				{
					for (k=0; k < Dim(); ++k)
					{
						rDerivSurface.SetControlPointPCoord(i, j, k, (GetControlPointCoord(i,j+1,k) - 
							GetControlPointPCoord(i,j,k)) * ScaleFactor);
					}
				}
			}
		}
	}
    else {
		assert(false); //we don't really want to have such curves in
		//this application possibly in the future implement a zero surface.
    }

	// Assigning the knot vectors.
	for (i=0; i<mod_kv.size() - 2; ++i)
		rDerivSurface.SetKnotValue(uDir, i, mod_kv[i+1]);
	for (i=0; i<other_kv.size(); ++i)
		rDerivSurface.SetKnotValue(!uDir, i, other_kv[i]);

	return NG_NO_ERR;
}

int NGSurface::EvaluatePartialDerivAt(bool uDir, double u, double v, 
											double* pDerivVec, double* pDerivW) const
{
	unsigned i;
	int res = NG_NO_ERR;

	// Returns a vector (x'(u,v),y'(u,v),...,w'(u,v)), unlike the next function.
	const std::vector<NGSurface>& m_derivs = uDir ? m_uDerivs : m_vDerivs;

	// Constructing the partial derivative auxilary structure.
	if (m_derivs.empty())
	{
		//Since this is a const method, 
		//we require a const cast for this lazy evaluation.
		std::vector<NGSurface>& ncm_derivs = 
			const_cast<std::vector<NGSurface>&>(m_derivs);

		// Construct the deriv auxilary structure.
		std::vector<NGSurface> ScalarSrfs;
		SplitScalar(ScalarSrfs);

		for (i=0; i < ScalarSrfs.size(); ++i)
		{
			NGSurface ScalarDerivSrf;
			res = ScalarSrfs[i].PartialDerivSurface(uDir, ScalarDerivSrf);
			assert(res == NG_NO_ERR);
			if (res != NG_NO_ERR)
			{
				ncm_derivs.clear();
				return res;
			}

			ncm_derivs.push_back(ScalarDerivSrf);
		}
	}

	for (i=0; i < Dim(); ++i)
	{
		res = m_derivs[i].EvaluateAt(u, v, &pDerivVec[i]);
		assert(res == NG_NO_ERR);
		if (res != NG_NO_ERR)
			return res;
	}
	if (Projective())
	{
		res = m_derivs[Dim()].EvaluateAt(u, v, pDerivW);		
		assert(res == NG_NO_ERR);
		if (res != NG_NO_ERR)
			return res;
	}

	return res;
}

int NGSurface::EvaluatePartialDerivsAt(double u, double v, 
									double* Su,  
									double* Sv) const
{
	unsigned i;
	int res = NG_NO_ERR;

	//Su = (dSx/du, dSy/du, dSz/du)
	double SuW;

	res = EvaluatePartialDerivAt(true, u, v, Su, &SuW);
	if (res != NG_NO_ERR) return res;

	//Sv = (dSx/dv, dSy/dv, dSz/dv)
	double SvW;

	res = EvaluatePartialDerivAt(false, u, v, Sv, &SvW);
	if (res != NG_NO_ERR) return res;

	if (Projective())
	{
		//handle rational derivatives:
		//Su[i] = (dSi/du * Sw - dSw/du * Si)/Sw^2
		//Sv[i] = (dSi/dv * Sw - dSw/dv * Si)/Sw^2

		double* S = new double[Dim()];
		double SW;
		res = EvaluateAt(u, v, S, &SW);
		if (res != NG_NO_ERR) return res;
		double denom = SW * SW;

		for (i=0; i<Dim(); ++i)
		{
			Su[i] = (Su[i]*SW - SuW*S[i])/denom;
		}

		for (i=0; i<Dim(); ++i)
		{
			Sv[i] = (Sv[i]*SW - SvW*S[i])/denom;
		}

		delete[] S;
	}

	return res;
}

int NGSurface::EvaluateNormalAt(double u, double v, double pVec[3],
									  double singularOffset) const
{
	assert(Dim() == 3); // Since we compute a cross product in R3.

	int res = NG_NO_ERR;
	double uDeriv[3], vDeriv[3];
	res = EvaluatePartialDerivsAt(u, v, uDeriv, vDeriv);
	if (res != NG_NO_ERR) return res;

	bool bReCalc = false;
	double uDerivSqLen = uDeriv[0]*uDeriv[0] + uDeriv[1]*uDeriv[1] + uDeriv[2]*uDeriv[2]; 
	if (uDerivSqLen < singularOffset)		
	{
		// if uDeriv is near zero - we're at a u-singular point move down v.
		bReCalc = true;
		v -= singularOffset;
		//if we went below first knot, we should have increased and not decreased
		if (v < m_vKV[0]) 			
			v += 2.0*singularOffset;
	}
	double vDerivSqLen = vDeriv[0]*vDeriv[0] + vDeriv[1]*vDeriv[1] + vDeriv[2]*vDeriv[2]; 
	if (vDerivSqLen < singularOffset)
	{
		bReCalc = true;
		u -= singularOffset;
		//if we went below first knot, we should have increased and not decreased
		if (u < m_uKV[0]) 			
			u += 2.0*singularOffset;
	}
	if (bReCalc)
	{
		res = EvaluatePartialDerivsAt(u, v, uDeriv, vDeriv);
		if (res != NG_NO_ERR) return res;
	}

	//Cross product.
	pVec[0] = uDeriv[1]*vDeriv[2] - uDeriv[2]*vDeriv[1];
	pVec[1] = uDeriv[2]*vDeriv[0] - uDeriv[0]*vDeriv[2];
	pVec[2] = uDeriv[0]*vDeriv[1] - uDeriv[1]*vDeriv[0];

	return res;
}

//Always returns a Cartesian (not rational) vector.
int NGSurface::EvaluateCurveDerivAt(double t, const NGCurve& uvCrv, 
									double* pVec) const
{
	int res = NG_NO_ERR;
	unsigned i;

	//uvPt = (u0,v0)
	//Note: if uvCrv is rational there will also be a third coord w0.
	double uvPt[2]; //cartesian uv point, EvaluateAt handled the w if exists.
	double uvPtW;
	res = uvCrv.EvaluateAt(t, uvPt, &uvPtW);
	if (res != NG_NO_ERR) return res;

	//uvDeriv = (du/dt, dv/dt). 
	//Note: if uvCrv is rational there will also be a third coord w. 
	//      the derivative is then (x'(t)w(t)-x(t)w'(t))/w^2(t)
	double uvDeriv[2];
	double uvDerivW;
	res = uvCrv.EvaluateDerivAt(t, uvDeriv, &uvDerivW);
	if (res != NG_NO_ERR) return res;

	if (uvCrv.Projective())
	{
		//handle rational derivatives.
		double denom = uvPtW * uvPtW;
		uvDeriv[0] = (uvDeriv[0]*uvPtW - uvPt[0]*uvDerivW) / denom;
		uvDeriv[1] = (uvDeriv[1]*uvPtW - uvPt[1]*uvDerivW) / denom;
	}

	//If we had assumed Dim()==3 we could have avoided dynamic alloc, 
	//but this is more general.
	double* Su = new double[Dim()];
	double* Sv = new double[Dim()];
	res = EvaluatePartialDerivsAt(uvPt[0], uvPt[1], Su, Sv);  
	if (res != NG_NO_ERR) return res;

	//pVec= Su*du/dt + Sv*dv/dt
	for (i=0; i<Dim(); ++i)
	{
		pVec[i] = Su[i]*uvDeriv[0] + Sv[i]*uvDeriv[1];
	}
	delete [] Su;
	delete [] Sv;

	return res;
}

int NGSurface::KnotInsertAt(bool uDir, double param, NGSurface& rOutSrf) const
{
	unsigned i,j,k;
	int res = NG_NO_ERR;

	unsigned numCPDir = uDir ? NumControlPointsU() : NumControlPointsV();
	unsigned numCPDirOther = uDir ? NumControlPointsV() : NumControlPointsU();
	unsigned DegDir = uDir ? DegreeU() : DegreeV();
	unsigned DegDirOther = uDir ? DegreeV() : DegreeU();
	const NGKnotVector& kvDir = uDir ? m_uKV : m_vKV;
	const NGKnotVector& kvDirOther = uDir ? m_vKV : m_uKV;

	// Perform knot insertion on each of the isoCurves (there is a small overhead
	// in this, can be handled by more specialized functions)
	// and then use the control points as the surface mesh.
	std::vector<NGCurve> isoCurves(numCPDirOther, NGCurve(Dim(), DegDir, numCPDir, Projective()));
	for (i=0; i<isoCurves.size(); ++i)
	{
		// Copy knot vector to isocurves.
		assert(kvDir.size()==isoCurves[i].NumKnots());
		for (k=0; k<isoCurves[i].NumKnots(); ++k)
			isoCurves[i].SetKnotValue(k, kvDir[k]);
	}

	for (i=0; i<numCPDirOther; ++i)
	{
		//copy a series of control points as isoCurves[i] control points
		for(j=0; j<numCPDir; ++j)
		{
			if (uDir)
			{
				for (k=0; k<Dim()+(Projective()?1:0); ++k)
					isoCurves[i].SetControlPointPCoord(j,k, GetControlPointPCoord(j,i,k));
			}
			else
			{
				for (k=0; k<Dim()+(Projective()?1:0); ++k)
					isoCurves[i].SetControlPointPCoord(j,k, GetControlPointPCoord(i,j,k));
			}
		}
		res = isoCurves[i].KnotInsertAt(param);
		if (res != NG_NO_ERR) return res;
	}

	// Construct rOutSrf
	rOutSrf = NGSurface(Dim(), DegreeU(), DegreeV(), 
		NumControlPointsU() + (uDir ? 1 : 0), 
		NumControlPointsV() + (uDir ? 0 : 1), 
		Projective());

	if (uDir)
		rOutSrf.m_vKV = m_vKV;
	else
		rOutSrf.m_uKV = m_uKV;
	for (i=0; i<isoCurves[0].NumKnots(); ++i)
		rOutSrf.SetKnotValue(uDir,i,isoCurves[0].GetKnotValue(i));

	// Constructing the new control mesh from isoCurves control points
	for (i=0; i<isoCurves.size(); ++i)
	{
		for (j=0; j<isoCurves[i].NumControlPoints(); ++j)
		{
			if (uDir)
			{
				for (k=0; k<Dim()+(Projective()?1:0); ++k)
				{
					rOutSrf.SetControlPointPCoord(j, i, k, 
						isoCurves[i].GetControlPointPCoord(j,k));
				}
			}
			else
			{
				for (k=0; k<Dim()+(Projective()?1:0); ++k)
				{
					rOutSrf.SetControlPointPCoord(i, j, k, 
						isoCurves[i].GetControlPointPCoord(j,k));
				}
			}
		}
	}

	return res;
}

int NGSurface::SubdivideAt(bool uDir, double param, 
						   NGSurface& rOutSrf1, NGSurface& rOutSrf2) const
{
	int res = NG_NO_ERR;
	double uMin, uMax, vMin, vMax;
	res = GetDomain(uMin, uMax, vMin, vMax);
	if (res != NG_NO_ERR) return res;
	double paramMin = uDir ? uMin : vMin;
	double paramMax = uDir ? uMax : vMax;
	if (param < paramMin || param > paramMax) return NG_ERR;

	NGSurface auxSrf(*this);
	const NGKnotVector& paramKV = uDir ? auxSrf.m_uKV : auxSrf.m_vKV;  
	unsigned paramDegree = uDir ? DegreeU() : DegreeV();

	NGKnotVector::const_iterator lbIter = 
		std::lower_bound(paramKV.begin(), paramKV.end(), param);

	unsigned prevIndex = (unsigned)(lbIter - paramKV.begin()); 
	//prevIndex won't change whereas lbIter will be invalidated

	// We want the index of the largest knot still smaller than u. 
	if (param - *lbIter < NG_EPS2)
		--prevIndex;
	while (paramKV[prevIndex+paramDegree] - param >= NG_EPS2)
	{
		NGSurface outSrf;
		res = auxSrf.KnotInsertAt(uDir, param, outSrf);;
		auxSrf = outSrf;
	}


	// When we're here there are DegreeU multiple knots of u in auxSrf,
	// all that is left is to split auxSrf.
	unsigned numCP1 = prevIndex + 1; //m_cp[*][prevIndex] will be duplicated

	unsigned paramNumCP = 
		uDir ? auxSrf.NumControlPointsU() : auxSrf.NumControlPointsV();
	unsigned numCP2 = paramNumCP - numCP1 + 1;
	if (uDir)
	{
		rOutSrf1 = NGSurface(Dim(), DegreeU(), DegreeV(), 
			numCP1, NumControlPointsV(), Projective());
		rOutSrf2 = NGSurface(Dim(), DegreeU(), DegreeV(), 
			numCP2, NumControlPointsV(), Projective());
	}
	else
	{
		rOutSrf1 = NGSurface(Dim(), DegreeU(), DegreeV(), 
			NumControlPointsU(), numCP1, Projective());
		rOutSrf2 = NGSurface(Dim(), DegreeU(), DegreeV(), 
			NumControlPointsU(), numCP2, Projective());
	}
	// Setting knots and control points of rOutSrf1
	unsigned i,j,k;
	if (uDir)
		rOutSrf1.m_vKV = auxSrf.m_vKV;
	else
		rOutSrf1.m_uKV = auxSrf.m_uKV;

	for (i=0; i < prevIndex+paramDegree+1; ++i)
		rOutSrf1.SetKnotValue(uDir, i, auxSrf.GetKnotValue(uDir,i));
	rOutSrf1.SetKnotValue(uDir, i, param); //uKV[prevIndex+Degree()+1]=u


	unsigned numCP1U = uDir ? numCP1 : NumControlPointsU();
	unsigned numCP1V = uDir ? NumControlPointsV() : numCP1;
	unsigned numCP2U = uDir ? numCP2 : NumControlPointsU();
	unsigned numCP2V = uDir ? NumControlPointsV() : numCP2;
	for (i=0; i<numCP1U; ++i)
		for (j=0; j<numCP1V; ++j)
			for (k=0; k<Dim()+(Projective()?1:0); ++k)
			{
				rOutSrf1.SetControlPointPCoord(i, j, k, auxSrf.GetControlPointPCoord(i, j, k));
			}
	// Setting knots and control points of rOutCrv2
	if (uDir)
	{
		rOutSrf2.m_vKV = auxSrf.m_vKV;
	}
	else
	{
		rOutSrf2.m_uKV = auxSrf.m_uKV;
	}
	rOutSrf2.SetKnotValue(uDir, 0,param);

	size_t paramKnotsSize = uDir ? auxSrf.m_uKV.size() : auxSrf.m_vKV.size();
	for (i=prevIndex+1; i < paramKnotsSize; ++i)
		rOutSrf2.SetKnotValue(uDir, i - prevIndex, auxSrf.GetKnotValue(uDir, i));
	for (i=0; i<numCP2U; ++i)
		for (j=0; j<numCP2V; ++j)
		{
			if (uDir)
			{
				for (k=0; k<Dim()+(Projective()?1:0); ++k)
				{
					rOutSrf2.SetControlPointPCoord(i, j, k, 
						auxSrf.GetControlPointPCoord(prevIndex+i, j, k));
				}
			}
			else
			{
				for (k=0; k<Dim()+(Projective()?1:0); ++k)
				{
					rOutSrf2.SetControlPointPCoord(i, j, k, 
						auxSrf.GetControlPointPCoord(i, prevIndex+j, k));
				}
			}
		}

	return res;
}


bool NGSurface::IsValid() const
{
	bool ret = true;

	if (m_uDegree<1 || m_vDegree<1) 
	{
		//error message
		ret = false;
	}

	// #kv = deg + #cp + 1
	if (m_uKV.size() != m_uDegree + NumControlPointsU() + 1 ||
		m_vKV.size() != m_vDegree + NumControlPointsV() + 1) 
	{
		//error message
		ret = false;
	}

	//knot vector validity
	if (!m_uKV.IsValid() || !m_vKV.IsValid()) {
		//error message
		ret = false;
	}

	//control points consistency
	if (m_controlPoints.empty())
	{
		//error message
		ret = false;
	}

	return ret;
}

int NGSurface::GetDomain(double& minU, double& maxU, 
						 double& minV, double& maxV) const
{
	if(!this->IsValid())
		return NG_ERR;

	minU = m_uKV[this->m_uDegree];
	maxU = m_uKV[this->m_uKV.size() - this->m_uDegree - 1];
	minV = m_vKV[this->m_vDegree];
	maxV = m_vKV[this->m_vKV.size() - this->m_vDegree - 1];

	return NG_NO_ERR;
}

void NGSurface::GetBBox(double* minPt, double* maxPt) const
{
	assert(Dim() > 0);
	unsigned i,j,k;
	for (k = 0; k<Dim(); ++k)
	{
		minPt[k] = GetControlPointCoord(0,0,k);
		maxPt[k] = GetControlPointCoord(0,0,k);
	}
	
	for(i=0;i<NumControlPointsU();++i)
	{
		for(j=0;j<NumControlPointsV();++j)
		{
			for (k=0; k<Dim(); ++k)
			{
				double coordK = GetControlPointCoord(i,j,k);
				if (coordK > maxPt[k])
					maxPt[k] = coordK;
				else
				if (coordK < minPt[k])
					minPt[k] = coordK;
			}
		}
	}
}

void NGSurface::GetKnotsAndMultiplicity(bool uDir, std::vector<unsigned>& rMult, 
										std::vector<double>& rKnots) const
{
	rMult.clear();
	rKnots.clear();

	NGKnotVector kv = uDir ? m_uKV : m_vKV;

	assert(!kv.empty());//sanity check
	rKnots.push_back(kv[0]);
	rMult.push_back(1);
	for (size_t i=1; i<kv.size(); ++i) 
	{
		if (kv[i] == kv[i-1])
			++(rMult.back());
		else
		{
			assert(kv[i] > kv[i-1]); //sanity check
			rKnots.push_back(kv[i]);
			rMult.push_back(1);		
		}
	}
}

void NGSurface::GetC1Discontinuities(bool uDir, std::vector<double>& rDiscontKnots) const
{
	std::vector<unsigned> Mult;
	std::vector<double> Knots;
	GetKnotsAndMultiplicity(uDir, Mult, Knots);

	unsigned deg = uDir ? DegreeU() : DegreeV();
	for (size_t i=0; i<Mult.size(); ++i)
	{
		if (Mult[i] >= deg)
			rDiscontKnots.push_back(Knots[i]);
	}
}

void NGSurface::Random(	double minCoord, double maxCoord,
						unsigned maxUKnotMultiplicity, 
						unsigned maxVKnotMultiplicity, 
						double minUVal, double maxUVal,
						double minVVal, double maxVVal){
	if(maxUKnotMultiplicity==0)
		maxUKnotMultiplicity = m_uDegree+1;
	if(maxVKnotMultiplicity==0)
		maxVKnotMultiplicity = m_vDegree+1;

	m_uKV.Random(maxUKnotMultiplicity,minUVal,maxUVal);
	m_vKV.Random(maxVKnotMultiplicity,minVVal,maxVVal);

	double interval = maxCoord-minCoord;
	for(unsigned i=0;i<NumControlPointsU();i++)
		for(unsigned j=0;j<NumControlPointsV();j++)
		{
			// Random control points:
      unsigned k;
			for(k=0;k<Dim();++k)
				SetControlPointPCoord(i,j,k, 
					minCoord + ((abs(rand())%10001)/10000.0 * interval));
			if(Projective())
				SetControlPointPCoord(i,j,k, 0.5 +((abs(rand())%10001)/10000.0));
		}
}

void NGSurface::SaveITD(const char* filename, const char* surfaceName)const
{
	FILE* outFile = fopen(filename, "w");
	fprintf(outFile, "[OBJECT %s\n", (surfaceName==NULL ? "NONE" : surfaceName));
	fprintf(outFile, "\t[SURFACE BSPLINE %d %d %d %d %s%d\n", 
		NumControlPointsU(), NumControlPointsV(), 
		(m_uDegree+1), (m_vDegree+1),
		(Projective() ? "P" : "E"), Dim());

	fprintf(outFile, "\t");
	PrintKVToFile(outFile, m_uKV);
	fprintf(outFile, "\n");

	fprintf(outFile, "\t");
	PrintKVToFile(outFile, m_vKV);
	fprintf(outFile, "\n");

	for(unsigned j=0;j<NumControlPointsV();j++){
		for(unsigned i=0;i<NumControlPointsU();i++){
			fprintf(outFile, "\t");
			{
				// Printing a point to the file.
				fprintf(outFile,"[");
				if(Projective())
					fprintf(outFile, "%lf ", GetControlPointW(i,j));
				fprintf(outFile, "%lf", GetControlPointPCoord(i,j,0));
				for(unsigned k=1;k<Dim();k++)
					fprintf(outFile, " %lf", GetControlPointPCoord(i,j,k));
				fprintf(outFile,"]");
			}

			fprintf(outFile, "\n");
		}
		fprintf(outFile, "\n");
	}
	fprintf(outFile, "\t]\n]\n");
	fflush(outFile);
}



// Splits the surface into its scalar components
void NGSurface::SplitScalar(std::vector<NGSurface>& rScalarSrfs) const
{
	unsigned i, j, k;
	unsigned ScalarSrfsLength = Dim() + (Projective() ? 1 : 0);

	//Every surface in rScalarSrfs is a non-rational surface of dimension 1,
	//which holds NumControlPointsU()*NumControlPointsV() points and a copy
	//of their knot vectors.
	rScalarSrfs.assign(ScalarSrfsLength, 
		NGSurface(1, DegreeU(), DegreeV(), 
			NumControlPointsU(), NumControlPointsV(), false));

	for (i=0; i<NumKnotsU(); ++i)
		for (j=0; j<ScalarSrfsLength; ++j)
			rScalarSrfs[j].SetKnotValue(true, i, GetKnotValue(true, i));
	for (i=0; i<NumKnotsV(); ++i)
		for (j=0; j<ScalarSrfsLength; ++j)
			rScalarSrfs[j].SetKnotValue(false, i, GetKnotValue(false, i));
		
	for (i=0; i<NumControlPointsU(); ++i)
	{
		for (j=0; j<NumControlPointsV(); ++j)
			//The k'th rScalarSrf holds the k'th coordinate of the control points.
			for (k=0; k<ScalarSrfsLength; ++k)
				rScalarSrfs[k].SetControlPointPCoord(i,j,0, 
					GetControlPointPCoord(i, j, k));
	}
}



int NGSurface::SplitAtKnots(std::vector<NGSurface>& rOutSrfs) const
{
  int err = NG_NO_ERR;
  unsigned i,j;
  std::vector<NGSurface> uSplit;

  std::vector<unsigned> uMult;
  std::vector<double> uKnots;
  GetKnotsAndMultiplicity(true, uMult, uKnots);
  assert(uKnots.size() > 1);

  //Splitting at u values
  if (uKnots.size() == 2) {
    //no inner knots along u //iddo: this code is actually redundant - taken care of inside, but for clarity..
    uSplit.push_back(*this);
  }
  else {
    NGSurface srf(*this);
    for (i=1; i<uKnots.size()-1; ++i) {
      NGSurface srf1, srf2;
      err = srf.SubdivideAt(true, uKnots[i], srf1, srf2);
      assert(err == NG_NO_ERR);

      uSplit.push_back(srf1);
      srf = srf2;
    }
    uSplit.push_back(srf);
  }

  std::vector<unsigned> vMult;
  std::vector<double> vKnots;
  GetKnotsAndMultiplicity(false, vMult, vKnots);
  assert(vKnots.size() > 1);

  if (vKnots.size() == 2) {
    //no inner knots along v - no splitting along v
    rOutSrfs = uSplit;
  }
  else {
    for (j=0; j<uSplit.size(); ++j) {
      NGSurface uSrf = uSplit[j];
      for (i=1; i<vKnots.size()-1; ++i) {
        NGSurface srf1, srf2;
        err = uSrf.SubdivideAt(false, vKnots[i], srf1, srf2);
        assert(err == NG_NO_ERR);

        rOutSrfs.push_back(srf1);
        uSrf = srf2;
      }

      rOutSrfs.push_back(uSrf);

    }
  }

  return NG_NO_ERR;
}
