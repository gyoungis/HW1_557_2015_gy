#ifndef NGKNOTVECTOR_H
#define NGKNOTVECTOR_H

/**
@file NGKnotVector.h

Class for representing NURBS knot vectors.
*/

#include <vector>

/** NGKnotVector is a wrapper through inheritance to std::vector<double>
	No virtual destructor is implemented as it is not supposed to be derived
	from, and no members were added (just member functions).
*/
class NGKnotVector : public std::vector<double> 
{
public:
	/// Default constructor.
	NGKnotVector() : std::vector<double>() {}
	/// Constructor of knot vector of size n initialized with zeros.
	NGKnotVector(unsigned n) : std::vector<double>(n, 0.0) {}
	/// Constructor from vector.
	NGKnotVector(const std::vector<double>& v) : std::vector<double>(v) {}

	/// Checking validity of knot vector.
	bool IsValid()const;

	/** Constructing a random knot vector.
	@param maxMultiplicity Maximal multiplicity of knots.
	@param minVal Minimal value of first knot.
	@param maxVal Maximal value of last knot.
	*/
	void Random(unsigned maxMultiplicity=1, double minVal=0, double maxVal=1);
};

/// Auxilary printing function.
void PrintKVToFile(FILE* f, const NGKnotVector& kv);

#endif //NGKNOTVECTOR_H
