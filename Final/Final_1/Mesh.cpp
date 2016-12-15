#include "Mesh.h"


Hermite2DPatch::Hermite2DPatch()
{
	this->initialized	= false;	

	this->identityTransformation	= true;
	MakeIdentityMatrix(this->transformationMatrix);
};

Hermite2DPatch::~Hermite2DPatch()
{

};

Hermite2DPatchMesh::Hermite2DPatchMesh()
{
	this->initialized = false;
}

Hermite2DPatchMesh::~Hermite2DPatchMesh()
{
	this->elements.clear();
	this->nodes.clear();
	this->patches.clear();
}

Hermite3DMesh::Hermite3DMesh()
{
	initialized = false;
}

Hermite3DMesh::~Hermite3DMesh()
{
	this->elements.clear();
	this->nodes.clear();
	this->localDOF.clear();
	this->meshGlobal2LocalMap.clear();
	this->displayPatches.clear();
}

void Hermite2DPatch::ConvertToNURBS(NURBS* surface)
{
	surface->trimmed	= false;
	surface->uBaseNum	= 5;
	surface->vBaseNum	= 5;
	surface->ka			= this->ka;
	surface->kdColor	= this->kdColor;
	surface->ksColor	= this->ksColor;
	surface->identityTransformation = this->identityTransformation;
	for (int k = 0; k < 16; k++)
		surface->transformationMatrix[k] = this->transformationMatrix[k];

	surface->uOrder		= 4;
	surface->vOrder		= 4;
	surface->uPoints	= 4;
	surface->vPoints	= 4;

	float tempuKnots[8] = {0, 0, 0, 0, 1, 1, 1, 1};
	float tempvKnots[8] = {0, 0, 0, 0, 1, 1, 1, 1};

	surface->uKnotVector = new float[surface->uPoints+surface->uOrder];
	surface->vKnotVector = new float[surface->vPoints+surface->vOrder];

	for (int u = 0; u < surface->uPoints + surface->uOrder; u++)
		surface->uKnotVector[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[surface->uPoints + surface->uOrder-1]-tempuKnots[0]);
	for (int v = 0; v < surface->vPoints + surface->vOrder; v++)
		surface->vKnotVector[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[surface->vPoints + surface->vOrder-1]-tempvKnots[0]);

	float* bezierCntlPts = new float[surface->uPoints*surface->vPoints*4];

	int vertexIndex[4]		= {0, 3, 12, 15};
	int vertexUIndex[4]		= {1, 2, 13, 14};
	int vertexVIndex[4]		= {4, 7, 8, 11};
	int vertexUVIndex[4]	= {5, 6, 9, 10};
	for (int i = 0; i < 4; i++)
	{
		int index = vertexIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k];
		bezierCntlPts[(index*4) + 3] = 1;
		
		int signU = 1;
		int signV = 1;
		if (i == 1 || i == 3)
			signU = -1;
		if (i == 2 || i == 3)
			signV = -1;

		Float3 currentDu = this->vertexDu[i]*signU;
		Float3 currentDv = this->vertexDv[i]*signV;
		Float3 currentDuDv = this->vertexDuDv[i]*signU*signV;

		index = vertexUIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentDu[k];
		bezierCntlPts[(index*4) + 3] = 1;

		index = vertexVIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentDv[k];
		bezierCntlPts[(index*4) + 3] = 1;

		index = vertexUVIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentDu[k] + (1.0/3.0)*currentDv[k] + (1.0/9.0)*currentDuDv[k];
		bezierCntlPts[(index*4) + 3] = 1;
	}
	surface->cntlPoints = bezierCntlPts;
}

void Local2GlobalMap::GenerateMap(Float3 localVec[3], Float3 globalVec[3], float derMap[9])
{
	float globalMat[9];
	float localMat[9];
	Convert2Matrix(globalVec, globalMat);
	Convert2Matrix(localVec, localMat);
	float invGlobalMat[9];
	InvertMatrix3(globalMat, invGlobalMat);
	MultMatrix(localMat, invGlobalMat, derMap);
}

void Hermite3DMesh::ConvertGlobal2Local()
{
	// Initialize Local to Global Map
	for (int k = 0; k < this->elements.size(); k++)
	{
		for (int j = 0; j < 8; j++)
		{
			Float3 localDer1Vec[3];
			Float3 globalDer1Vec[3];
			float globalDer1Mat[9];
			float localDer1Mat[9];
			float* SE1 = this->meshGlobal2LocalMap[k * 8 + j]->der1Map;

			globalDer1Vec[0] = this->nodes[this->elements[k]->nodes[j]]->vertexDu;
			globalDer1Vec[1] = this->nodes[this->elements[k]->nodes[j]]->vertexDv;
			globalDer1Vec[2] = this->nodes[this->elements[k]->nodes[j]]->vertexDw;

			Convert2Matrix(globalDer1Vec, globalDer1Mat);
			MultMatrix(globalDer1Mat, SE1, localDer1Mat);
			Convert2Vector(localDer1Mat, localDer1Vec);
			
			this->localDOF[k * 8 + j]->du = localDer1Vec[0];
			this->localDOF[k * 8 + j]->dv = localDer1Vec[1];
			this->localDOF[k * 8 + j]->dw = localDer1Vec[2];
			
			Float3 localDer2Vec[3];
			Float3 globalDer2Vec[3];
			float globalDer2Mat[9];
			float localDer2Mat[9];
			float* SE2 = this->meshGlobal2LocalMap[k * 8 + j]->der2Map;

			globalDer2Vec[0] = this->nodes[this->elements[k]->nodes[j]]->vertexDuDv;
			globalDer2Vec[1] = this->nodes[this->elements[k]->nodes[j]]->vertexDwDu;
			globalDer2Vec[2] = this->nodes[this->elements[k]->nodes[j]]->vertexDvDw;

			Convert2Matrix(globalDer2Vec, globalDer2Mat);
			MultMatrix(globalDer2Mat, SE2, localDer2Mat);
			Convert2Vector(localDer2Mat, localDer2Vec);

			this->localDOF[k * 8 + j]->dudv = localDer2Vec[0];
			this->localDOF[k * 8 + j]->dwdu = localDer2Vec[1];
			this->localDOF[k * 8 + j]->dvdw = localDer2Vec[2];
			
			this->localDOF[k * 8 + j]->dudvdw = this->meshGlobal2LocalMap[k * 8 + j]->der3Map*this->nodes[this->elements[k]->nodes[j]]->vertexDuDvDw;

		}
	}
}


void Hermite3DMesh::SaveMeshDisplayFaces(char* fileName)
{
	ofstream outFile(fileName, ios::out);

	for (int i = 0; i < this->displayElements.size(); i++)
	{
		int k = this->displayElements[i] - 1;
		int face = this->displayFaces[i] - 1;
		NURBS* surface = this->nurbsPatches[i];

		for (int nodes = 0; nodes < 16; nodes++)
		{
			outFile << surface->cntlPoints[nodes * 4 + 0] << '\t';
			outFile << surface->cntlPoints[nodes * 4 + 1] << '\t';
			outFile << surface->cntlPoints[nodes * 4 + 2] << '\t';
			outFile << surface->cntlPoints[nodes * 4 + 3] << '\t';
		}
		outFile << '\n';
	}
	outFile.close();
}

/*
void Hermite2DPatch::ConvertToNURBS(NURBS* surface)
{
	surface->trimmed	= false;
	surface->uBaseNum	= 5;
	surface->vBaseNum	= 5;
	surface->ka			= this->ka;
	surface->kdColor	= this->kdColor;
	surface->ksColor	= this->ksColor;
	surface->identityTransformation = this->identityTransformation;
	for (int k = 0; k < 16; k++)
		surface->transformationMatrix[k] = this->transformationMatrix[k];

	surface->uOrder		= 4;
	surface->vOrder		= 4;
	surface->uPoints	= 4;
	surface->vPoints	= 4;

	float tempuKnots[8] = {0, 0, 0, 0, 1, 1, 1, 1};
	float tempvKnots[8] = {0, 0, 0, 0, 1, 1, 1, 1};

	surface->uKnotVector = new float[surface->uPoints+surface->uOrder];
	surface->vKnotVector = new float[surface->vPoints+surface->vOrder];

	for (int u = 0; u < surface->uPoints + surface->uOrder; u++)
		surface->uKnotVector[u] =	(tempuKnots[u]-tempuKnots[0])/(tempuKnots[surface->uPoints + surface->uOrder-1]-tempuKnots[0]);
	for (int v = 0; v < surface->vPoints + surface->vOrder; v++)
		surface->vKnotVector[v] =	(tempvKnots[v]-tempvKnots[0])/(tempvKnots[surface->vPoints + surface->vOrder-1]-tempvKnots[0]);

	float* bezierCntlPts = new float[surface->uPoints*surface->vPoints*4];

//	float uScale = 25;
//	float vScale = 25;

	int vertexIndex[4]		= {0, 3, 12, 15};
	int vertexUIndex[4]		= {1, 2, 13, 14};
	int vertexVIndex[4]		= {4, 7, 8, 11};
	int vertexUVIndex[4]	= {5, 6, 9, 10};
	for (int i = 0; i < 4; i++)
	{
		int index;
		float uScale, vScale;
		index = vertexIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k];
		bezierCntlPts[(index*4) + 3] = 1;
		

		
		int signU = 1;
		int signV = 1;
		if (i == 1 || i == 3)
			signU = -1;
		if (i == 2 || i == 3)
			signV = -1;

//		if (i == 0 || i == 1)
//			uScale = Distance(this->vertexVal[0], this->vertexVal[1]);
//		else
//			uScale = Distance(this->vertexVal[2], this->vertexVal[3]);
//		if (i == 0 || i == 2)
//			vScale = Distance(this->vertexVal[0], this->vertexVal[2]);
//		else
//			vScale = Distance(this->vertexVal[1], this->vertexVal[3]);
//		int signUV = signU*signV;

//		Float3 currentElementDu = float(signU*uScale)*this->vertexDu[i];
//		Float3 currentElementDv = float(signV*vScale)*this->vertexDv[i];
//		Float3 currentElementDuDv = float(signUV*uScale*vScale)*this->vertexDuDv[i];
	

		Float3 vertexNormal = VectorCrossProduct(this->vertexDu[i], this->vertexDv[i]);
		VectorNormalize(vertexNormal);
		Float3 currentDu = this->vertexDu[i]*signU;
		Float3 currentDv = this->vertexDv[i]*signV;
		Float3 currentDuDv = this->vertexDuDv[i]*signU*signV;
//		VectorNormalize(currentDu);
//		VectorNormalize(currentDv);
		float lenDu = VectorMagnitude(this->vertexDu[i]);
		float lenDv = VectorMagnitude(this->vertexDv[i]);
		Float3 elementDuDir, elementDvDir;
		if (i == 0)
		{
			elementDuDir = this->vertexVal[1]-this->vertexVal[0];
			elementDvDir = this->vertexVal[2]-this->vertexVal[0];
		}
		else if (i == 1)
		{
			elementDuDir = this->vertexVal[0]-this->vertexVal[1];
			elementDvDir = this->vertexVal[3]-this->vertexVal[1];
		}
		else if (i == 2)
		{
			elementDuDir = this->vertexVal[3]-this->vertexVal[2];
			elementDvDir = this->vertexVal[0]-this->vertexVal[2];
		}
		else if (i == 3)
		{
			elementDuDir = this->vertexVal[2]-this->vertexVal[3];
			elementDvDir = this->vertexVal[1]-this->vertexVal[3];
		}

		Float3 elementDuProjection = elementDuDir - (VectorDotProduct(vertexNormal, elementDuDir))*vertexNormal;
		Float3 elementDvProjection = elementDvDir - (VectorDotProduct(vertexNormal, elementDvDir))*vertexNormal;
		float lenElementDuProjection = VectorMagnitude(elementDuProjection);
		float lenElementDvProjection = VectorMagnitude(elementDvProjection);

		float test1 = VectorDotProduct(elementDuProjection, vertexNormal)/VectorMagnitude(elementDuProjection);
		float test2 = VectorDotProduct(elementDvProjection, vertexNormal)/VectorMagnitude(elementDvProjection);

//		VectorNormalize(elementDuProjection);
//		VectorNormalize(elementDvProjection);

		float cosAngleDuDuProjection = VectorDotProduct(elementDuProjection, currentDu)/lenElementDuProjection;
		float cosAngleDvDuProjection = VectorDotProduct(elementDuProjection, currentDv)/lenElementDuProjection;
		float cosAngleDuDvProjection = VectorDotProduct(elementDvProjection, currentDu)/lenElementDvProjection;
		float cosAngleDvDvProjection = VectorDotProduct(elementDvProjection, currentDv)/lenElementDvProjection;

		float duScale = sqrt(lenDu*lenDu*cosAngleDuDuProjection*cosAngleDuDuProjection + lenDv*lenDv*(1 - cosAngleDvDuProjection*cosAngleDvDuProjection));
		float dvScale = sqrt(lenDv*lenDv*cosAngleDvDvProjection*cosAngleDvDvProjection + lenDu*lenDu*(1 - cosAngleDuDvProjection*cosAngleDuDvProjection));

		Float3 currentElementDu = duScale*elementDuProjection;
		Float3 currentElementDv = dvScale*elementDvProjection;

		Float3 currentElementDuDv = float(duScale*dvScale)*this->vertexDuDv[i];

		Float3 tempDuDv = this->vertexDuDv[i];
		VectorNormalize(tempDuDv);
		float tempDot = VectorDotProduct(tempDuDv, vertexNormal);

		index = vertexUIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentDu[k];
//			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentElementDu[k];
		bezierCntlPts[(index*4) + 3] = 1;

		index = vertexVIndex[i];
		for (int k = 0; k < 3; k++)
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentDv[k];
//			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentElementDv[k];
		bezierCntlPts[(index*4) + 3] = 1;

		index = vertexUVIndex[i];
		for (int k = 0; k < 3; k++)
//			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentElementDu[k] + (1.0/3.0)*currentElementDv[k] + (1.0/9.0)*currentElementDuDv[k];
			bezierCntlPts[(index*4) + k] = this->vertexVal[i][k] + (1.0/3.0)*currentDu[k] + (1.0/3.0)*currentDv[k] + (1.0/9.0)*currentDuDv[k];
		bezierCntlPts[(index*4) + 3] = 1;
	}
	surface->cntlPoints = bezierCntlPts;

}
*/
