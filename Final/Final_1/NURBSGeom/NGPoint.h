#ifndef NGPOINT_H
#define NGPOINT_H

/**
@file NGPoint.h

Template class for representing NURBS d-dimensional Cartesian points.
Used for input/output points and vectors.
*/

#include <vector>
#include <math.h>
#include "NURBSGeom.h"

template <unsigned Dim_>
class NGPoint
{
public:
	/// Default constructor: fill coords with zeros.
	NGPoint() {memset((char *)m_coords, 0, Dim_*sizeof(double));}

	/// Copy constructor.
	NGPoint(const NGPoint<Dim_>& pt) {
		for (unsigned i=0; i<Dim_; ++i)
			m_coords[i] = pt.m_coords[i];
	}

	/// Dimension of point.
	inline unsigned Dim() const {return Dim_;}

	/// Assignment operator.
	inline NGPoint& operator=(const NGPoint& pt) {
		for (unsigned i=0; i<Dim_; ++i) m_coords[i] = pt.m_coords[i];
		return *this;
	}

	/// Summing pt to this point.
	inline NGPoint<Dim_> operator+(const NGPoint<Dim_>& pt) const
	{
		NGPoint<Dim_> res(*this);
		res += pt;
		return res;
	}

	/// Subtracting pt from this point.
	inline NGPoint<Dim_> operator-(const NGPoint<Dim_>& pt)const
	{
		NGPoint<Dim_> res(*this);
		res -= pt;
		return res;
	}

	/// Scaling this by val in place.
	inline void ScaleS(double val) {
		for(unsigned i=0; i < Dim(); ++i) m_coords[i] *= val;
	}

	/// Scaling this by val returning a new point.
	inline NGPoint<Dim_> Scale(double val) const {
		NGPoint<Dim_> res(*this);
		res.ScaleS(val);
		return res;
	}

	/// Summing pt to this point in place.
	inline NGPoint<Dim_>& operator+=(const NGPoint<Dim_>& pt) {
		for(unsigned i=0;i<Dim();++i) m_coords[i] += pt.m_coords[i];
		return *this;
	}

	/// Subtracting pt from this point in place.
	inline NGPoint<Dim_>& operator-=(const NGPoint<Dim_>& pt) {
		return operator+=(-pt);
	}

	/// Unary operator-.
	inline NGPoint<Dim_> operator-() const
	{
		return this->Scale(-1.0);
	}


	/// Direct access functions to coordinates.
	inline double operator[](unsigned i)const {
		assert(i<Dim());
		return this->m_coords[i];
	}
	inline double& operator[](unsigned i) {
		assert(i<Dim());
		return this->m_coords[i];
	}

	/// Returns true if pt is equal to this up to eps tolerance.
	bool Equals(const NGPoint<Dim_>& pt, double eps)const {
		for(unsigned i=0;i<Dim();i++)
			if(fabs(GetCoord(i) - pt.GetCoord(i)) > eps)
				return false;
		return true;	
	}

	/// Get Cartesian coordinate i.
	inline double GetCoord(unsigned i) const {
		assert(i<Dim());
		return m_coords[i];
	}

	/// Vector operations:
	/// Squared length of the vector described by this point.
	double SquaredLength() const {
		double sum = 0.0;
		for (unsigned i=0; i<Dim(); ++i)
			sum += GetCoord(i)*GetCoord(i);
		return sum;
	}

	/// Length of the vector described by this point (square root of SquaredLength).
	inline double Length() const {return sqrt(SquaredLength());}
	
	/** Normalizes the vector that the point represents in place.
		For projective vectors the result is a (x,y,...,w=1) point
		which represents a unit-length vector in Cartesian space.*/
	void NormalizeS() {
		double denom = Length();
		for (unsigned i=0; i<Dim(); ++i) m_coords[i] /= denom;
	}

	/** Normalizes the vector that the point represents, and returns
		a new point.*/
	inline NGPoint Normalize() const {
		NGPoint res(*this);
		res.NormalizeS();
		return res;
	}

	/** D-dimensional inner product.
		Allows InnerProd of projective with non-projective as long 
		as dim is the same.*/
	double InnerProd(const NGPoint<Dim_>& pt) const {
		double res = 0.0;
		for (unsigned i=0; i<Dim(); ++i) {res += GetCoord(i) * pt.GetCoord(i);}
		return res;		
	}

	/// Cross product of first 2 Cartesian-coords.
	inline double CrossProd2D(const NGPoint<Dim_>& pt) const {
		return GetCoord(0)*pt.GetCoord(1) - GetCoord(1)*pt.GetCoord(0);
	}
	/// Cross product of first 3 Cartesian-coords.
	NGPoint<3> CrossProd3D(const NGPoint<Dim_>& pt) const {
		NGPoint<3> res; //3D point
		res[0] = GetCoord(1)*pt.GetCoord(2) - GetCoord(2)*pt.GetCoord(1);
		res[1] = GetCoord(2)*pt.GetCoord(0) - GetCoord(0)*pt.GetCoord(2);
		res[2] = GetCoord(0)*pt.GetCoord(1) - GetCoord(1)*pt.GetCoord(0);
		return res;
	}

	/** Access to the inner C-array representation. Used e.g., when calling
	an evaluation function, to minimize copies and allocations.
	Usage example:
	@code
	NGPoint<3> outPt;
	srf.EvaluateAt(0.5, 0.5, outPt.GetCArray());
	@endcode	*/
	inline double* GetCArray() {return m_coords;} 
	inline const double* GetCArray() const {return m_coords;} 

private:
	/// Inner C-array representation.
	double m_coords[Dim_];

	/// The following are private to force comparison with tolerance.
	bool operator==(const NGPoint& pt);
	bool operator!=(const NGPoint& pt);
};

#endif //NGPOINT_H
