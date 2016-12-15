#ifndef NGSURFACE_H
#define NGSURFACE_H

/**
@file NGSurface.h

Class for representing NURBS d-dimensional surface.
*/

#include "NURBSGeom.h"

/** Default value for distance from singular point in which to compute normal,
	see EvaluateNormalAt method documentation.
*/
extern const double NG_SINGULAR_NORMAL_OFFSET;

class NGSurface
{
public:
	/** Default constructor - it is constructed to an invalid state.
		To be used mainly when an output reference is needed by a function.*/
	NGSurface() : m_dim(0), m_projective(false), m_uDegree(), m_vDegree(), 
		m_controlPoints(), m_uKV(), m_vKV() {}

	/// Constructor that allocates memory and fills with zeros.
	NGSurface(	unsigned _dim, unsigned _uDegree, unsigned _vDegree,
				unsigned _uNumControlPoints, unsigned _vNumControlPoints,
				bool _projective=false);
	/// Copy constructor.
	NGSurface(const NGSurface& srf);

	/// Access functions.

	/// Dimension of surface.
	inline unsigned Dim() const {return m_dim;}
	/// U-degree of surface.
	inline unsigned DegreeU() const {return m_uDegree;}
	/// V-degree of surface.
	inline unsigned DegreeV() const {return m_vDegree;}
	/// Size of surface u knot vector.
	inline unsigned NumKnotsU() const {return (unsigned)m_uKV.size();}
	/// Size of surface v knot vector.
	inline unsigned NumKnotsV() const {return (unsigned)m_vKV.size();}

	/// Number of control points in u direction.
	inline unsigned NumControlPointsU() const {return (NumKnotsU()-DegreeU()-1);}
	/// Number of control points in v direction.
	inline unsigned NumControlPointsV() const {return (NumKnotsV()-DegreeV()-1);}
	///< Total number of control points is: NumControlPointsU()*NumControlPointsV().

	/// Is the surface projective.
	inline bool Projective() const {return m_projective;}

	/** Access the coordIndex homogeneous coordinate of the (i,j)'th control point.
		For non-rational (non-projective) surfaces this function is equivalent
		to GetControlPointCoord.
		@note Assumes coordIndex is a valid index.
	*/
	inline double GetControlPointPCoord(unsigned i, unsigned j, unsigned coordIndex) const {
		assert(m_controlPoints.size() > coordIndex);
		return m_controlPoints[coordIndex][i*NumControlPointsV() + j];
	}
	/** Set the coordIndex homogeneous coordinate of the (i,j)'th control point.
		@note For non-rational (non-projective) surfaces there is no difference
		between homogeneous and non-homogeneous coordinates.
		@note Assumes coordIndex is a valid index.
	*/
	inline void SetControlPointPCoord(unsigned i, unsigned j, 
		unsigned coordIndex, double coordVal) {
		assert(m_controlPoints.size() > coordIndex);
		m_controlPoints[coordIndex][i*NumControlPointsV() + j] = coordVal;
	}
	/** Access the coordIndex Cartesian coordinate of the (i,j)'th control point.
		For non-rational surfaces this function is equivalent to GetControlPointPCoord.
		However, for rational (projective) surfaces, the cordinate is divided by its weight.
	*/
	inline double GetControlPointCoord(unsigned i, unsigned j, unsigned coordIndex) const 
	{
		return GetControlPointPCoord(i, j, coordIndex)/GetControlPointW(i,j);
	}
	/// Access the weight of the (i,j)th control point (1.0 if non-projective).
	inline double GetControlPointW(unsigned i, unsigned j) const
	{return Projective() ? m_controlPoints[Dim()][i*NumControlPointsV() + j] : 1.0;}
	/** Set the weight of the (i,j)th control point.
		@note Assumes surface is projective.
	*/
	inline void SetControlPointW(unsigned i, unsigned j, double w)
	{assert(Projective()); m_controlPoints[Dim()][i*NumControlPointsV() + j] = w;}

	/** Access to j'th knot in u or v direction.
		@param uDir u-direction if true v-direction if false.
	*/
	inline double GetKnotValue(bool uDir, unsigned j) const 
		{return (uDir ? m_uKV[j] : m_vKV[j]);}
	/** Setting the j'th knot in u or v direction.
		@param uDir u-direction if true v-direction if false.
	*/
	inline void SetKnotValue(bool uDir, unsigned j, double val) 
		{(uDir ? m_uKV[j] : m_vKV[j]) = val;}

	/** Returns a vector of surface knot values in u or v directions,
		with their corresponding multiplicities.
	*/
	void GetKnotsAndMultiplicity(bool uDir, std::vector<unsigned>& rMult, 
		std::vector<double>& rKnots) const;

	/** Returns a vector of knot values where there are C1-discontinuities 
		(including clamped end knots).
		@param uDir Discontinuities in u direction if true, v direction if false.
		@param rDiscontKnots Knot parameters where the discontinuities occur.
	*/
	void GetC1Discontinuities(bool uDir, std::vector<double>& rDiscontKnots) const;

	/** Domain of the curve.
		@param minU Domain minimal u value.
		@param maxU Domain maximal u value.
		@param minV Domain minimal v value.
		@param maxV Domain maximal v value.
		@return NG_NO_ERR on success.
	*/
	int GetDomain(double& minU, double& maxU, double& minV, double& maxV)const;

	/** Bounding box of the surface.
		@param minPt Array holding the coordinates of minimal point of bounding box.
		@param maxPt Array holding the coordinates of maximal point of bounding box.
		@note Returned arrays are Euclidean points of same dimension as surface,
		it is the caller's responsibility to allocate sufficient memory for them.
		@note Bounding box is not tight, it is computed on the control mesh.
	*/
	void GetBBox(double* minPt, double* maxPt) const;

	/** Evaluation of surface at (u,v).
		@param u Parameter to evaluate at in u direction.
		@param v Parameter to evaluate at in v direction.
		@param pcoords Array holding the evaluated homogeneous coordinates.
		@param w Will hold the output weight of the evaluated point (1.0 if
		surface is non-projective).
		@return NG_NO_ERR on success.
		@note pcoords will be filled with Dim() values. It is the caller's
		responsibility to allocate sufficient memory for them.
	*/
	int EvaluateAt(double u, double v, double* pcoords, double* w) const;

	/** Evaluation of surface at (u,v). Unlike the previous function, the result
		here will always be a Cartesian point.
		@param u Parameter to evaluate at in u direction.
		@param v Parameter to evaluate at in v direction.
		@param cartesian_coords Array (of size Dim()) holding the evaluated
		Cartesian coordinates.
		@return NG_NO_ERR on success.
		@note cartesian_coords will be filled with Dim() values. It is the caller's
		responsibility to allocate sufficient memory for them.
	*/
	int EvaluateAt(double u, double v, double* cartesian_coords) const;

	/** Returns an iso-parametric curve of the surface.
	@param uDirConst Is u or v the constant iso-parameter.
	@param parameter The iso-parameter.
	@param rOutCrv The output iso-curve.
	@return NG_NO_ERR on success.
	*/
	int IsoCurve(bool uDirConst, double param, NGCurve& rOutCrv) const;

	/** Returns the partial derivative @b surface of the surface. That is, the
		returned surface represents the partial derivative field.
		@param uDir u-partial derivative if true v-partial derivative if false.
		@param rDerivSurface Partial derivative surface.
		@return NG_NO_ERR on success.
		@note Not applicable to rational surfaces.*/
	int PartialDerivSurface(bool uDir, NGSurface& rDerivSurface) const;

	/** Evaluation of partial derivative at (u,v).
		@param uDir u-partial derivative if true v-partial derivative if false.
		@param u Parameter to evaluate at in u direction.
		@param v Parameter to evaluate at in v direction.
		@param pVec Evaluated homogeneous coordinates of partial derivative vector. 
		@param pDerivW Evaluated weight coordinate of partial derivative vector
		(1.0 if not projective).
		@return NG_NO_ERR on success.
		@note For rational (projective) surfaces, pVec is not the partial derivative. 
		The actual partial derivative can be computed using the formula: 
		(dSi/du * Sw - dSw/du * Si)/Sw^2.
		See also EvaluatePartialDerivsAt, which already computes the formula.
		@note pVec will be filled with Dim() values. It is the caller's
		responsibility to allocate sufficient memory for them.
	*/
	int EvaluatePartialDerivAt(bool uDir, double u, double v, double* pVec, double* pDerivW) const;

	/** Evaluation of both partial derivatives at (u,v).
		@param u Parameter to evaluate at in u direction.
		@param v Parameter to evaluate at in v direction.
		@param pUDeriv Evaluated partial derivative vector in the u direction. 
		@param pVDeriv Evaluated partial derivative vector in the v direction. 
		@return NG_NO_ERR on success.
		@note Unlike EvaluatePartialDerivAt, for rational surfaces the partial
		derivative vectors are evaluated completely using the rational derivative
		fromula.
		@note pUDeriv and pVDeriv will be filled with Dim() values. It is the
		caller's responsibility to allocate sufficient memory for them.
	*/
	int EvaluatePartialDerivsAt(double u, double v, 
									double* pUDeriv,  
									double* pVDeriv) const;

	/** Returns the normal vector of the surface at (u,v).
		@param pVec Evaluated normal vector (not normalized).
		@param singularOffset Value to perturb (u,v) away from singular point (see note).
		@return NG_NO_ERR on success.
		@note Evaluating the normal at a singular point results in a zero-vector. This
		can result, for example, in a dark region in the neighborhood of the singular
		point when visualizing it. In such cases it is sometimes useful to evaluate
		the normal some distance from the singular point. 
		If singularOffset is set to zero, the normal near singular points will
		be near zero, otherwise it will be evaluated at a perturbed distance from
		the singular point.
		@note Assumes Dim() = 3. 
	*/
	int EvaluateNormalAt(double u, double v, double pVec[3], 
					double singularOffset = NG_SINGULAR_NORMAL_OFFSET) const;

	/** Evaluates the 3D derivative d/dt of the uv curve on the surface at 
		parameter t. Uses the chain rule: 
		dS(u(t),v(t))/dt = dS/du*du/dt + dS/dv*dv/dt.
		@param uDir u-partial derivative if true v-partial derivative if false.
		@param t Parameter to evaluate at on uvCrv.
		@param uvCrv Curve in uv-space of surface.
		@param pVec Evaluated derivative vector in R3. 
		(of the same type as surface control points).
		@return NG_NO_ERR on success.
		@note pVec will be filled with Dim() values. It is the
		caller's responsibility to allocate sufficient memory for them.
	*/
	int EvaluateCurveDerivAt(double t, const NGCurve& uvCrv, 
									double* pVec) const;


	/** Knot insertion.
	@param uDir Is the knot inserted in u or v knot vector.
	@param param Knot parameter to insert.
	@param rOutSrf Output surface with inserted knot.
	@return NG_NO_ERR upon success.
	@note Returns an error if knot param is alreay inserted with Degree multiplicity. */
	int KnotInsertAt(bool uDir, double param, NGSurface& rOutSrf) const;
	/// Knot insertion in u direction.
	inline int KnotInsertAtU(double param, NGSurface& rOutSrf) const 
		{return KnotInsertAt(true, param, rOutSrf);}
	/// Knot insertion in v direction.
	inline int KnotInsertAtV(double param, NGSurface& rOutSrf) const 
		{return KnotInsertAt(false, param, rOutSrf);}

	/** Surface subdivision.
	@param uDir Is the surface subdivided in u or v knot vector.
	@param param Knot parameter to subdivide at.
	@param rOutSrf1 First output surface.
	@param rOutSrf2 Second output surface.
	@return Error value.*/
	int SubdivideAt(bool uDir, double param, 
					NGSurface& rOutSrf1, NGSurface& rOutSrf2) const;
	/// Subdivision in u-direction.
	int SubdivideAtU(double u, NGSurface& rOutSrf1, NGSurface& rOutSrf2) const 
		{return SubdivideAt(true, u, rOutSrf1, rOutSrf2);}
	/// Subdivision in v-direction.
	int SubdivideAtV(double v, NGSurface& rOutSrf1, NGSurface& rOutSrf2) const 
		{return SubdivideAt(false, v, rOutSrf1, rOutSrf2);}

	/// Validity checks.
	bool IsValid()const;

	/// Auxilary printing function.
	void SaveITD(const char* filename, const char* surfaceName=NULL)const;

	/** Constructing a random surface.
	@param minCoord Minimal value for coordinate.
	@param maxCoord Maximal value for coordinate.
	@param maxUKnotMultiplicity Maximal multiplicity of u knots.
	@param maxVKnotMultiplicity Maximal multiplicity of v knots.
	@param minUVal Minimal value of first u knot.
	@param maxUVal Maximal value of last u knot.
	@param minVVal Minimal value of first v knot.
	@param maxVVal Maximal value of last v knot.
	*/
	void Random(double minCoord=0, double maxCoord=1,
				unsigned maxUKnotMultiplicity=0, 
				unsigned maxVKnotMultiplicity=0, 
				double minUVal=0, double maxUVal=1,
				double minVVal=0, double maxVVal=1);

	/// Splits the surface into its scalar components (x(u,v),y(u,v),..etc.).
	void SplitScalar(std::vector<NGSurface>& rScalarSrfs) const;


  /// Splits the surface at inner knots to get a set of surfaces that are equivalent to Bezier patches. 
  int SplitAtKnots(std::vector<NGSurface>& rOutSrfs) const;

private:
	unsigned m_dim;
	bool m_projective;

	unsigned m_uDegree, m_vDegree;

	//Control points hold a vector for each coordinate:
	//x = m_controlPoints[0],..,w = m_controlPoins.back();
	//m_controlPoints[j][0] = CP(0,0)[j], [j][1] = CP(0,1)[j] etc.
	std::vector<std::vector<double> > m_controlPoints;

	NGKnotVector m_uKV;
	NGKnotVector m_vKV;

	/** Auxilary structures for faster evaluation of partial derivatives. 
		Holds 1-dimensional derivative curves xDu(t),yDu(t),...wDu(t). */
	std::vector<NGSurface> m_uDerivs;
	std::vector<NGSurface> m_vDerivs;
};

#endif //NGSURFACE_H
