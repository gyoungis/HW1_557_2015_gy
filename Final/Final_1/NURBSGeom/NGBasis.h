/**
@file NGBasis.h

Evaluation of B-Spline basis functions.
*/

#ifndef NGBASIS_H
#define NGBASIS_H

#include "NURBSGeom.h"
#include "NGKnotVector.h"

class NGBasis
{
public:
	/**
	Evaluating a B-Spline basis function using the recursive
	definition of the spline basis functions.
	See any basic CAGD book, for example:
	Chapter 6 of "Geometric Modeling with Splines: An Introduction."
	by Cohen, Riesenfeld and Elber.

	@param t The parameter for the evaluation.
	@param index Index of B-Spline basis function.
	@param degree Degree of B-Spline basis functions.
	@param kv The knot vector that defines the B-Spline basis functions.
	@return The evaluated value.
	*/
	static double Evaluate(double t, unsigned index, 
		unsigned degree, const NGKnotVector& kv);

private:
	NGBasis(){};
	/// Inner recursive evaluation function.
	static double EvaluateRecursive(double t, unsigned index, 
		unsigned degree, const NGKnotVector& kv);
};

#endif //NGBASIS_H
