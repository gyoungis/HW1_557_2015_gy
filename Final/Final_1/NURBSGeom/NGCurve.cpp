#include "NURBSGeom.h"
#include "NGCurve.h"
#include "NGBasis.h"

#include <algorithm>

#pragma warning(disable:4996) //warning on fopen

NGCurve::NGCurve(unsigned _dim, unsigned _degree, 
				 unsigned _numControlPoints, bool _projective):
m_dim(_dim), m_projective(_projective),
m_degree(_degree), m_kv(_numControlPoints+_degree+1), 
m_controlPoints(_dim + (_projective ? 1 : 0), std::vector<double>(_numControlPoints))
{}

NGCurve::NGCurve(const NGCurve& crv):
m_dim(crv.m_dim), m_projective(crv.m_projective),
m_degree(crv.m_degree), 
m_kv(crv.m_kv), 
m_controlPoints(crv.m_controlPoints)
{}

void NGCurve::Reverse()
{
	size_t i;
	for (i=0; i<m_controlPoints.size(); ++i)
	{
		std::reverse(m_controlPoints[i].begin(), m_controlPoints[i].end());
	}

	std::reverse(m_controlPoints.begin(), m_controlPoints.end());
	double maxKnot = m_kv.back();
	size_t last = m_kv.size() - 1;
	NGKnotVector aux_kv(m_kv); //if we don't use an auxilary kv, we will fold
	//back after passing the half.
	for (i=0; i<m_kv.size(); ++i)
	{
		m_kv[i] = maxKnot - aux_kv[last-i];
	}
}

int NGCurve::EvaluateAt(double t, double* pcoords, double* w)const
{
	//if(t<m_kv[m_degree] || t>m_kv[m_kv.size()-m_degree-1])
	//	return NG_ERR;//parameter exceeds domain
	
	//A more tolerant function, we assume the user wanted to be inside the domain.
	//Possibly add a tolerance to the above check.
	if(t<m_kv[m_degree])
		t = m_kv[m_degree];
	if (t>m_kv[m_kv.size()-m_degree-1])
		t = m_kv[m_kv.size()-m_degree-1];

	unsigned i,j;

	// Initialize sum to be (0,...0).
	if (Projective())
		*w = 0;
	else
		*w = 1;
	for (j=0; j<Dim(); ++j)
		pcoords[j] = 0;

	// Using binary search to find relevant indices.
	NGKnotVector::const_iterator tIter = 
		std::lower_bound(m_kv.begin()+m_degree, m_kv.end()-m_degree-1, t);
	if ((*tIter > t) || (tIter==m_kv.end()-m_degree-1))
		--tIter; 
	//Since lower_bound returns the one-after if t is not in the array.We need
	//to take the one before.
	//The end parameter evaluation is a special case.

	unsigned endIndex = (unsigned)(tIter - m_kv.begin());
	unsigned  index = endIndex - m_degree;

	for(i=index;i<=endIndex;++i)
	{
		double B_it = NGBasis::Evaluate(t,i,m_degree,m_kv);
		if (B_it != 0.0)
		{
			for (j=0; j<Dim(); ++j)
				pcoords[j] += (m_controlPoints[j][i]) * B_it;
			if (Projective())
				*w += (m_controlPoints[j][i]) * B_it;
		}
	}

	return NG_NO_ERR;
}

int NGCurve::EvaluateAt(double t, double* cartesian_coords)const
{
	unsigned i = 0;
	double w = 1;
	int res = EvaluateAt(t, cartesian_coords, &w);
	if (res != NG_NO_ERR)
		return res;
	if (Projective())
	{
		for (i=0; i<Dim(); ++i)
			cartesian_coords[i] /= w;
	}

	return res;
}


bool NGCurve::IsValid() const
{
	bool ret = true;

	if (m_degree < 1) 
	{
		//error message
		ret = false;
	}
	// #kv = deg + #cp + 1
	if (m_kv.size() != m_degree + NumControlPoints() + 1) 
	{
		//error message
		ret = false;
	}
	//knot vector validity
	if (!m_kv.IsValid()) {
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

int NGCurve::GetDomain(double& minT, double& maxT)const{
	if(!IsValid())
		return NG_ERR;
	minT = m_kv[m_degree];
	maxT = m_kv[m_kv.size() - m_degree - 1];
	return NG_NO_ERR;
}

void NGCurve::GetBBox(double* minPt, double* maxPt) const
{
	unsigned i;
	assert(Dim() > 0);
	for (i = 0; i<Dim(); ++i)
	{
		minPt[i] = GetControlPointCoord(0,i);
		maxPt[i] = GetControlPointCoord(0,i);
	}

	for(unsigned i=1;i<NumControlPoints();++i)
	{
		for (unsigned j=0; j<Dim(); ++j)
		{
			double coordJ = GetControlPointCoord(i,j);
			if (coordJ > maxPt[j])
				maxPt[j] = coordJ;
			else
			if (coordJ < minPt[j])
				minPt[j] = coordJ;
		}
	}
}

void NGCurve::GetKnotsAndMultiplicity(std::vector<unsigned>& rMult, 
									  std::vector<double>& rKnots) const
{
	rMult.clear();
	rKnots.clear();

	assert(!m_kv.empty());//sanity check
	rKnots.push_back(m_kv[0]);
	rMult.push_back(1);
	for (size_t i=1; i<m_kv.size(); ++i) 
	{
		if (m_kv[i] == m_kv[i-1])
			++(rMult.back());
		else
		{
			assert(m_kv[i] > m_kv[i-1]); //sanity check
			rKnots.push_back(m_kv[i]);
			rMult.push_back(1);		
		}
	}
}

void NGCurve::GetC1Discontinuities(std::vector<double>& rDiscontKnots) const
{
	std::vector<unsigned> Mult;
	std::vector<double> Knots;
	GetKnotsAndMultiplicity(Mult, Knots);

	for (size_t i=0; i<Mult.size(); ++i)
	{
		if (Mult[i] >= Degree())
			rDiscontKnots.push_back(Knots[i]);
	}
}


void NGCurve::Random(double minCoord, double maxCoord, 
					 unsigned maxKnotMultiplicity, 
					 double minVal, double maxVal)
{
	if(maxKnotMultiplicity==0)
		maxKnotMultiplicity = m_degree+1;
	m_kv.Random(maxKnotMultiplicity,minVal,maxVal);

  unsigned i,j;
	double interval = maxVal-minVal;
	for(i=0;i<NumControlPoints();i++)
	{
		for(j=0;j<Dim();j++)
			m_controlPoints[j][i] = minVal + ((abs(rand())%10001)/10000.0 * interval);
		if(Projective())
			m_controlPoints[j][i] = 0.5 +((abs(rand())%10001)/10000.0);
	}
}

// Splits the curve into its scalar components (including w and therefore all crvs are not rational)
void NGCurve::SplitScalar(std::vector<NGCurve>& rScalarCrvs) const
{
	unsigned ScalarCrvsLength = Dim() + (Projective() ? 1 : 0);

	//every curve in rScalarCrvs holds NumOfControlPoints points of dimension 1
	//and a copy of this knot vector
	rScalarCrvs.assign(ScalarCrvsLength, 
		NGCurve(1, Degree(), NumControlPoints(), false)); //all scalars are not projective

	for (unsigned i = 0; i < ScalarCrvsLength; ++i)
	{
		rScalarCrvs[i].m_controlPoints[0] = m_controlPoints[i];
		rScalarCrvs[i].m_kv = m_kv;
	}
}

int NGCurve::DerivativeCurve(NGCurve& rCurve) const
{
    unsigned i, j;

	//rational curves not supported yet.
	//In order to support rational curves a symbolic curve multiplication
	//is required to perform: (x'w-xw')/w^2
    if (Projective()) {
		assert(false);
    }

	assert(Degree() >=1);

	rCurve = NGCurve(Dim(), Degree()-1, NumControlPoints()-1, false);
    if (Degree() >= 1) {
		for (i = 0; i < NumControlPoints() - 1; i++) {
			double Denom = m_kv[i + Degree() + 1] - m_kv[i + 1];

			if (fabs(Denom) < NG_EPS2)
				Denom = NG_EPS2;

			double ScaleFactor = (Degree()/Denom);

			for (j=0; j<Dim(); ++j)
			{
				double coordJ = ScaleFactor *
					(GetControlPointCoord(i+1,j) - GetControlPointCoord(i,j));
				rCurve.SetControlPointPCoord(i,j,coordJ);
			}
		}
	}
    else {
		assert(false); //we don't really want to have such curves in this application
		for (i = 0; i < (unsigned)std::max((int)NumControlPoints() - 1, 1); i++)
			for (j = 0; j <= Dim(); j++)
				rCurve.SetControlPointPCoord(i,j,0.0);
    }

	for (i=1; i<m_kv.size() - 2; ++i)
		rCurve.SetKnotValue(i, GetKnotValue(i+1));

	return NG_NO_ERR;
}

int NGCurve::EvaluateDerivAt(double t, double* pDerivVec, double* pDerivW) const
{
	int res = NG_NO_ERR;
	unsigned i;

	//Returns a vector (x'(t),y'(t),...,w'(t)).

	if (m_derivs.empty())
	{
		//Since this is a const method, 
		//we require a const cast for this lazy evaluation.
		std::vector<NGCurve>& ncm_derivs = 
			const_cast<std::vector<NGCurve>&>(m_derivs);

		std::vector<NGCurve> ScalarCrvs;
		SplitScalar(ScalarCrvs);
		for (i=0; i < ScalarCrvs.size(); ++i)
		{
			ncm_derivs.push_back(NGCurve(1, Degree()-1, NumControlPoints(), false));
			res = ScalarCrvs[i].DerivativeCurve(ncm_derivs[i]);
			assert(res == NG_NO_ERR);
			if (res != NG_NO_ERR)
			{
				ncm_derivs.clear();
				return res;
			}
		}
	}


	for (i=0; i < Dim(); ++i)
	{
		res = m_derivs[i].EvaluateAt(t, &pDerivVec[i]);		
		assert(res == NG_NO_ERR);
		if (res != NG_NO_ERR)
			return res;
	}
	if (Projective())
	{
		res = m_derivs[Dim()].EvaluateAt(t, pDerivW);		
		assert(res == NG_NO_ERR);
		if (res != NG_NO_ERR)
			return res;
	}

	return res;
}

int NGCurve::KnotInsertAt(double t)
{
	/*
	Let Index be the last knot in old knot vector less than t and
	let j be j = Index - Degree. Also let k be the curve order. Then,
	Case 1: Q(i) = P(i), i <= j						     
	
		            t     -    t(i)        t(i+k-1)  -   t		     
	case 2: Q(i) = --------------- P(i) + --------------- P(i-1), j<i<=Index  
		            t(i+k-1) - t(i)        t(i+k-1) - t(i)		     
									     
	case 3: Q(i) = P(i-1), Index < i	

	For more see also:
	"Recursive proof of Boehm's knot insertion technique", by Phillip J Barry
	Ronald N Goldman, CAD, Volume 20 number 4 1988, pp 181-182.
	Which also references the original 1980 paper by Boehm.

	*/

	unsigned i;
	int res = NG_NO_ERR;
	double tMin, tMax;
	res = GetDomain(tMin, tMax);
	if (res != NG_NO_ERR) return res;
	if (t < tMin || t > tMax) return NG_ERR;

	NGKnotVector::iterator lbIter = 
		std::lower_bound(m_kv.begin(), m_kv.end(), t);
	//if (*lbIter == t) //is eps comparison really needed?
	if (t - *lbIter < NG_EPS2)
	{
		//if (*(lbIter + Degree() - 1) == t)
		if (*(lbIter + Degree() - 1) - t < NG_EPS2)
			return NG_ERR; //knot multiplicity >= Degree
		else
			--lbIter; //if *lbIter==t, get the one before 
					  //(i.e., largest knot that is smaller than t).
	}
	//now lbIter holds the last knot that is smaller than t.
	unsigned Index = (unsigned)(lbIter - m_kv.begin());
	unsigned j = (unsigned)(Index - Degree());


    // Case 1: Copy all points upto (Index - Degree) as is.
	//No need to do anything.

	// Case 2:
	// Auxilary vector of Degree case 2 points.
	std::vector<std::vector<double> > aux(Dim() + (Projective() ? 1 : 0), std::vector<double>(Degree()));
    for (j = Index - Degree() + 1; j <= Index; j++)
	{
		for (unsigned i = 0; i < Dim() + Projective() ? 1 : 0; i++)
		{
			aux[i][j - (Index - Degree() + 1)] =
	       ((t - m_kv[j]) * m_controlPoints[i][j] +
			(m_kv[j + Degree()] - t) * m_controlPoints[i][j - 1]) /
				      (m_kv[j + Degree()] - m_kv[j]);
		}
	}

	// Case 3: Copy all points from Index+1 to end (add a point at the end)
	for (i=0; i<m_controlPoints.size(); ++i)
		m_controlPoints[i].push_back(m_controlPoints[i].back());
	for (i=0; i<m_controlPoints.size(); ++i)
	{
		for (j = (unsigned)m_controlPoints[i].size() - 2; j >= Index; --j)
		{
			m_controlPoints[i][j+1] = m_controlPoints[i][j];
		}

		// Copy the auxilary vector control points.
		std::copy(aux[i].begin(), aux[i].end(), m_controlPoints[i].begin()+Index-Degree()+1);
	}	

	// Update the knot vector.
	m_kv.insert(lbIter+1, t); //lbIter is now invalidated

	return res;
}


int NGCurve::Subdivide(double t, NGCurve& rOutCrv1, NGCurve& rOutCrv2) const
{
	int res = NG_NO_ERR;
	double tMin, tMax;
	res = GetDomain(tMin, tMax);
	if (res != NG_NO_ERR) return res;
	if (t < tMin + NG_EPS2 || t > tMax - NG_EPS2) return NG_ERR;

	NGCurve auxCrv(*this);
	NGKnotVector::iterator lbIter = 
		std::lower_bound(auxCrv.m_kv.begin(), auxCrv.m_kv.end(), t);

	unsigned prevIndex = (unsigned)(lbIter - auxCrv.m_kv.begin()); 
	//prevIndex won't change whereas lbIter will be invalidated

	// We want the index of the largest knot still smaller than t. 
	if (t - *lbIter < NG_EPS2)
		--prevIndex;
	while (auxCrv.m_kv[prevIndex+Degree()] - t >= NG_EPS2)
	{
		res = auxCrv.KnotInsertAt(t);
	}


	// When we're here there are Degree multiple knots of t in auxCrv,
	// all that is left is to split auxCrv.
	unsigned numCP1 = prevIndex + 1; //the last CP is m_cp[prevIndex]
	unsigned numCP2 = auxCrv.NumControlPoints() - numCP1 + 1; //m_cp[prevIndex] is copied twice
	rOutCrv1 = NGCurve(Dim(), Degree(), numCP1, Projective());
	rOutCrv2 = NGCurve(Dim(), Degree(), numCP2, Projective());

	// Setting knots and control points of rOutCrv1
	unsigned i,j;
	for (i=0; i < prevIndex+Degree()+1; ++i)
		rOutCrv1.SetKnotValue(i, auxCrv.GetKnotValue(i));
	rOutCrv1.SetKnotValue(i, t); //kv[prevIndex+Degree()+1]=t
	for (i=0; i<numCP1; ++i)
		for (j=0; j<Dim()+(Projective() ? 1 : 0); ++j)
		{
			rOutCrv1.SetControlPointPCoord(i, j, auxCrv.GetControlPointPCoord(i,j));
		}

	// Setting knots and control points of rOutCrv2
	rOutCrv2.SetKnotValue(0,t);
	for (i=prevIndex+1; i < auxCrv.m_kv.size(); ++i)
		rOutCrv2.SetKnotValue(i - prevIndex, auxCrv.GetKnotValue(i));
	for (i=0; i<numCP2; ++i)
		for (j=0; j<Dim()+(Projective() ? 1 : 0); ++j)
		{
			rOutCrv2.SetControlPointPCoord(i, j, auxCrv.GetControlPointPCoord(prevIndex+i,j));
		}

	return res;
}


int NGCurve::ExtractSubCurve(double t1, double t2, NGCurve& rOutCrv) const
{
	int res = NG_NO_ERR;
	double tMin, tMax;
	res = GetDomain(tMin, tMax);
	if (res != NG_NO_ERR) return res;
	
	if (t1 > t2)
		std::swap(t1, t2);

	NGCurve auxCrv1, auxCrv2;
	if (t1 < tMin + NG_EPS2)
	{
		t1 = tMin;
		auxCrv2 = *this;
	}
	else
	{
		res = Subdivide(t1, auxCrv1, auxCrv2);
		if (res != NG_NO_ERR)
			return res;
	}

	if (t2 > tMax - NG_EPS2)
	{
		t2 = tMax;
		rOutCrv = auxCrv2;
	}
	else
	{
		res = auxCrv2.Subdivide(t2, rOutCrv, auxCrv1);
		if (res != NG_NO_ERR)
			return res;

	}

	return res;
}


void NGCurve::SaveITD(const char* filename, const char* curveName)const
{
	FILE* outFile = fopen(filename, "w");
	fprintf(outFile, "[OBJECT %s\n", (curveName==NULL ? "NONE" : curveName));
	fprintf(outFile, "\t[CURVE BSPLINE %d %d %s%d\n", 
		NumControlPoints(), (m_degree+1),
		(Projective() ? "P" : "E"), Dim());

	fprintf(outFile, "\t");
	PrintKVToFile(outFile, m_kv);
	fprintf(outFile, "\n");

	for(unsigned i=0; i<NumControlPoints(); ++i){
		fprintf(outFile, "\t");
		// Printing the control point:
		{
			fprintf(outFile,"[");
			if(Projective())
				fprintf(outFile, "%lf ", GetControlPointW(i));
			fprintf(outFile, "%lf", m_controlPoints[0][i]);
			for(unsigned j=1;j<Dim();j++)
				fprintf(outFile, " %lf", m_controlPoints[j][i]);
			fprintf(outFile,"]");
		}

		fprintf(outFile, "\n");
	}

	fprintf(outFile, "\t]\n]\n");
	fflush(outFile);
}
