#include "NURBSGeom.h"
#include "NGKnotVector.h"

bool NGKnotVector::IsValid() const
{
	for(unsigned i=1;i<this->size();i++)
		if(((*this)[i-1]) > (*this)[i])
			return false;
	return true;
}

void NGKnotVector::Random(unsigned maxMultiplicity, double minVal, double maxVal)
{
	double currentVal(0);
	unsigned i=0;

	while(i<this->size()){
		currentVal = currentVal + ((10 + (abs(rand())%(10001-10)))/10000.0);
		unsigned multiplicity = (abs(rand())%maxMultiplicity)+1;
		while(i<this->size() && multiplicity-- > 0)
			(*this)[i++] = currentVal;
	}

	// Scale to fit min/max vals
	double interval = maxVal-minVal;
	double minKV = (*this)[0];
	double maxKV = this->back();
	double intervalKV = maxKV-minKV;
	for(unsigned i=0;i<this->size();i++){
		(*this)[i] = minVal + ((((*this)[i]-minKV)/(intervalKV)) * interval);
	}
}

void PrintKVToFile(FILE* f, const NGKnotVector& kv)
{
	fprintf(f, "[KV");
	for(unsigned i=0; i<kv.size(); ++i)
		fprintf(f, " %lf", kv[i]);
	fprintf(f, "]");
}
