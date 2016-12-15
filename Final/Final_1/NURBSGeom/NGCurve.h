#ifndef NGCURVE_H
#define NGCURVE_H

/**
@file NGCurve.h

Class for representing NURBS d-dimensional curve.
*/

#include "NURBSGeom.h"

class NGCurve
{
public:
	/** Default constructor - it is constructed to an invalid state.
		Used mainly when an output reference is needed by a function.*/
	NGCurve() : m_dim(0), m_projective(false), m_degree(0), m_controlPoints(), m_kv(), m_derivs() {}
	/// Constructor that allocates memory and fills with zeros.
	NGCurve(unsigned _dim, unsigned _degree, 
			unsigned _numControlPoints, bool _projective=false);

	/// Copy constructor.
	NGCurve(const NGCurve& crv);

	/// Access functions.

	/// Dimension of curve.
	inline unsigned Dim() const {return m_dim;}
	/// Degree of curve.
	inline unsigned Degree() const {return m_degree;}
	/// Number of control points of curve.
	inline unsigned NumControlPoints() const 
		{return (unsigned)m_controlPoints.back().size();}
	/// Size of curve knot vector.
	inline unsigned NumKnots() const {return (unsigned)m_kv.size();}
	/// Is the curve projective.
	inline bool Projective() const {return m_projective;}

	/** Access the coordIndex homogeneous coordinate of the i'th control point.
		For non-rational (non-projective) curves this function is equivalent
		to GetControlPointCoord.
		@note Assumes coordIndex is a valid index.
	*/
	inline double GetControlPointPCoord(unsigned i, unsigned coordIndex) const 
	{return m_controlPoints[coordIndex][i];}

	/** Access the coordIndex Cartesian coordinate of the i'th control point.
		For non-rational curves this function is equivalent to GetControlPointPCoord.
		However, for rational (projective) curves, the coordinate is divided by its weight.
	*/
	inline double GetControlPointCoord(unsigned i, unsigned coordIndex) const
	{return GetControlPointPCoord(i,coordIndex)/GetControlPointW(i);}

	/** Set the coordIndex homogeneous coordinate of the i'th control point.
		@note For non-rational (non-projective) curves there is no difference
		between the homogeneous and non-homogeneous coordinates.
		@note Assumes coordIndex is a valid index.
	*/
	inline void SetControlPointPCoord(unsigned i, unsigned coord, double coordVal) 
		{m_controlPoints[coord][i] = coordVal;}

	/// Access the weight of the i'th control point (1.0 if non-projective).
	inline double GetControlPointW(unsigned i) const
	{return Projective() ? m_controlPoints[Dim()][i] : 1.0;}
	/** Set the weight of the i'th control point.
		@note Assumes surface is projective.*/
	inline void SetControlPointW(unsigned i, double w)
	{assert(Projective()); m_controlPoints[Dim()][i] = w;}

	/// Access to i'th knot.
	inline double GetKnotValue(unsigned i) const {return m_kv[i];}
	/// Setting i'th knot.
	inline void SetKnotValue(unsigned i, double val) {m_kv[i] = val;} 
	
	/// Validity checks.
	bool IsValid() const;

	/** Evaluation of curve at t.
		@param t Parameter to evaluate at.
		@param pcoords Array holding the evaluated homogeneous coordinates.
		@param w Will hold the output weight of the evaluated point (1.0 if
		surface is non-projective).
		@return NG_NO_ERR on success.
		@note pcoords will be filled with Dim() values. It is the caller's
		responsibility to allocate sufficient memory for them.*/
	int EvaluateAt(double t, double* pcoords, double* w) const;

	/** Evaluation of curve at t. Unlike the previous function, the result
		here will always be a Cartesian point.
		@param t Parameter to evaluate at.
		@param cartesian_coords Array (of size Dim()) holding the evaluated
		Cartesian coordinates.
		@return NG_NO_ERR on success.
		@note cartesian_coords will be filled with Dim() values. It is the caller's
		responsibility to allocate sufficient memory for them.*/
	int EvaluateAt(double t, double* cartesian_coords) const;

	/** Domain of the curve.
		@param minT Domain minimal value.
		@param maxT Domain maximal value.
		@return NG_NO_ERR on success.
	*/
	int GetDomain(double& minT, double& maxT) const;

	/** Bounding box of the curve.
		@param minPt Array holding the coordinates of minimal point of bounding box.
		@param maxPt Array holding the coordinates of maximal point of bounding box.
		@note Returned arrays are Cartesian points of same dimension as surface,
		it is the caller's responsibility to allocate sufficient memory for them.
		@note Bounding box is not tight, it is computed on the control mesh.*/
	void GetBBox(double* minPt, double* maxPt) const;

	/// Returns a vector of curve knot values with their corresponding multiplicities.
	void GetKnotsAndMultiplicity(std::vector<unsigned>& rMult, 
		std::vector<double>& rKnots) const;

	/** Returns a vector of knot values where there are C1-discontinuities 
		(including clamped end knots).
	*/
	void GetC1Discontinuities(std::vector<double>& rDiscontKnots) const;

	/// Reversing the curve in place.
	void Reverse();
	/// Returns the reversed curve.
	inline NGCurve Reverse() const
	{
		NGCurve ret(*this);
		ret.Reverse();
		return ret;
	}

	/**
	Returns a new curve, equal to the given curve, differentiated once.
	Let the old control polygon be P(i), i = 0 to k-1, and Q(i) be the new one then:
	Q(i) = Degree * (P(i+1) - P(i)) / (Kv(i + Degree + 1) - Kv(i + 1)), i = 0 to k-2.

	For more see any basic CAGD book, for example:
	Chapter 6 of "Geometric Modeling with Splines: An Introduction."
	by Cohen, Riesenfeld and Elber.

	@param rCurve Reference to output curve.
	@return NG_NO_ERR on success.
	@note Not implemented for rational curves.*/
	int DerivativeCurve(NGCurve& rCurve) const ;
	

	/** Returns the vector of derivatives (x'(t),y'(t),...,w'(t)).

	@param pDerivVec Reference to output vector, which will be of same dimension as control points.
	@param pDerivW Evaluated weight coordinate of partial derivative vector
	(1.0 if not projective).
	@return NG_NO_ERR on success.
	@note For rational curves, in order to get the derivative, the return vector
	should be processed, for example: [x'(t)w(t)-x(t)w'(t)]/w^2.
	@note pVec will be filled with Dim() values. It is the caller's
	responsibility to allocate sufficient memory for them.*/
	int EvaluateDerivAt(double t, double* pDerivVec, double* pDerivW) const;

	/**
	Refines the curve, by inserting a new knot at t, using Boehm's algorithm.
	See:
	"Recursive proof of Boehm's knot insertion technique", by Phillip J Barry
	Ronald N Goldman, CAD, Volume 20 number 4 1988, pp 181-182.
	Which also references the original 1980 paper by Boehm.

	@param t The parameter for the new knot.
	@return NG_NO_ERR on success.
	*/
	int KnotInsertAt(double t);

	/**
	Returns a refined curve, with a new knot at t.

	@param t The parameter for the new knot.
	@param rOutCrv The output curve.
	@return NG_NO_ERR on success.
	*/
	inline int KnotInsertAt(double t, NGCurve& rOutCrv) const
	{
		rOutCrv = *this;
		return rOutCrv.KnotInsertAt(t);
	}

	/**
	Subdivides the curve at parameter t.

	@param t The parameter for the subdivision.
	@param rOutCrv1 The first output curve.
	@param rOutCrv2 The second output curve.
	@return NG_NO_ERR on success.
	*/
	int Subdivide(double t, NGCurve& rOutCrv1, NGCurve& rOutCrv2) const;

	/**
	Extracts the subcurve between parameters t1 and t2.

	@param rOutCrv The extracted output curve.
	@return NG_NO_ERR on success.
	*/
	int ExtractSubCurve(double t1, double t2, NGCurve& rOutCrv) const;

	/// Splits the curve into its scalar components (x(t),y(t),..etc.).
	void SplitScalar(std::vector<NGCurve>& rScalarCrvs) const;

	/** Constructing a random curve.
	@param minCoord Minimal value for coordinate.
	@param maxCoord Maximal value for coordinate.
	@param maxMultiplicity Maximal multiplicity of knots.
	@param minVal Minimal value of first knot.
	@param maxVal Maximal value of last knot.
	*/
	void Random(double minCoord=0, double maxCoord=1, 
		unsigned maxKnotMultiplicity=0, 
		double minVal=0, double maxVal=1);

	/// Auxilary printing function.
	void SaveITD(const char* filename, const char* curveName=NULL)const;
private:
	unsigned m_dim;
	bool m_projective;

	unsigned m_degree;

	//Control points hold a vector for each coordinate:
	//x = m_controlPoints[0],..,w = m_controlPoins.back();
	//m_controlPoints[j][0] = CP(0)[j], [j][1] = CP(1)[j] etc.
	std::vector<std::vector<double> > m_controlPoints;
	NGKnotVector m_kv;

	/** Auxilary structures for fast derivitive evaluation. Holds 1-dimensional
		derivative curves x'(t),y'(t),...w'(t).*/
	std::vector<NGCurve> m_derivs;
};

#endif //NGCURVE_H
