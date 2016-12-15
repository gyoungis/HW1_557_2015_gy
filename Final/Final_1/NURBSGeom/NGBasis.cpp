#include "NURBSGeom.h"
#include "NGBasis.h"


double NGBasis::Evaluate(double t, unsigned index, 
						 unsigned degree, const NGKnotVector& kv)
{
	// In case of C0 discontinuity inside a curve (not very common)
	// or end-parameter in clamped end condition (very common).
	if (t == kv[index+1] && t==kv[index+degree+1])
		return 1.0;

	if( index < 0 || index+degree+1 >= kv.size() ||
		t < kv[index] || t >= kv[index+degree+1] )
		return 0;
	return EvaluateRecursive(t,index,degree,kv);
}

double NGBasis::EvaluateRecursive(double t, unsigned index, 
								  unsigned degree, const NGKnotVector& kv)
{
	if(degree == 0){
		if(t < kv[index] || t>= kv[index+1])
			return 0;
		else
			return 1;
	}

	double val1 = EvaluateRecursive(t,index,degree-1,kv);
	double val2 = EvaluateRecursive(t,index+1,degree-1,kv);
	return ( (val1<= NG_EPS2 ? 0 : 
				val1*((t-kv[index])/(kv[index+degree]-kv[index])) ) 
					+ 
			 (val2<= NG_EPS2 ? 0 : 
				val2*((kv[index+degree+1]-t)/(kv[index+degree+1]-kv[index+1]))) 
			);
}
