#include "GeometricOperations.h"
#include "GPUUtilities.h"

// Surface Intersection
int FindClosestNeighbour(int currentIndex, int* points2DMap, float* points, int maxRes, int nPoints, int surfX)
{
	int resolution = 1;
	vector<int> neighbourPoints;
	int xPosIndex = int(points[7*currentIndex + surfX  ]);
	int yPosIndex = int(points[7*currentIndex + surfX+1]);

	int startXIndex = xPosIndex - resolution;
	int startYIndex = yPosIndex - resolution;
	if (startXIndex < 0)
		startXIndex = 0;
	if (startYIndex < 0)
		startYIndex = 0;

	// Add neighbourpoints to the list;
	for (int jPos = startYIndex; jPos <= yPosIndex+resolution && jPos < maxRes; jPos++)
		for (int iPos = startXIndex; iPos <= xPosIndex+resolution && iPos < maxRes; iPos++)
			if (points2DMap[jPos*maxRes + iPos] != -1)
				neighbourPoints.push_back(points2DMap[jPos*maxRes + iPos]);

	if (neighbourPoints.size() == 0)
		return -1;
	else if(neighbourPoints.size() == 1)
		return neighbourPoints[0];
	else
	{
		float closestDist = DistanceR7(points+neighbourPoints[0]*7, points+currentIndex*7); 
		int closestPointIndex = neighbourPoints[0];
		for (int k = 1; k < neighbourPoints.size(); k++)
		{
			float dist = DistanceR7(points+neighbourPoints[k]*7, points+currentIndex*7);
			if (dist < closestDist)
				closestPointIndex = neighbourPoints[k];
		}
		return closestPointIndex;
	}

	return -1;
}


#ifdef POLYFIT1
vector<list<int>> FitPolylines(float* points, int nPoints, int intRes)
{
	float tolerance = 1.0/(intRes);
	vector<list<int>> polylines;
	for (int k = 0; k < nPoints; k++)
	{
		list<int> pLine;
		pLine.push_back(k);
		polylines.push_back(pLine);
	}

	bool merged = true;
	while (merged)
	{
		// Find closest polyline pair
		float closestDistance = intRes;
		int iPos = -1;
		int jPos = -1;
		int closestOrder = -1;
		for (int j = 0; j < polylines.size(); j++)
		{
			for (int i = 0; i < polylines.size(); i++)
			{
				if (i!=j)
				{
					int pLine1start	= polylines[i].front();
					int pLine1end	= polylines[i].back();
					int pLine2start	= polylines[j].front();
					int pLine2end	= polylines[j].back();
					int order;
					float separationDistance = PolylineDistance(pLine1start,pLine1end,pLine2start,pLine2end,points,&order);
					if (separationDistance <= closestDistance)
					{
						closestDistance = separationDistance;
						iPos = i;
						jPos = j;
						closestOrder = order;
					}
				}

			}
		}

		// If distance < tolerance, Merge
		// else Break
		if (closestDistance < 800)
		{
			if (closestOrder == 0)
			{
				polylines[iPos].splice(polylines[iPos].end(),polylines[jPos]);
				vector<list<int>>::iterator removeIndex = polylines.begin()+jPos;
				polylines.erase(removeIndex);
			}
			if (closestOrder == 1)
			{
				polylines[jPos].splice(polylines[jPos].end(),polylines[iPos]);
				vector<list<int>>::iterator removeIndex = polylines.begin()+iPos;
				polylines.erase(removeIndex);
			}
			if (closestOrder == 2)
			{
				polylines[jPos].reverse();
				polylines[iPos].splice(polylines[iPos].end(),polylines[jPos]);
				vector<list<int>>::iterator removeIndex = polylines.begin()+jPos;
				polylines.erase(removeIndex);
			}
			if (closestOrder == 3)
			{
				polylines[iPos].reverse();
				polylines[jPos].splice(polylines[jPos].end(),polylines[iPos]);
				vector<list<int>>::iterator removeIndex = polylines.begin()+iPos;
				polylines.erase(removeIndex);
			}
			merged = true;
			if (closestOrder == -1)
				merged = false;
		}
		else
			merged = false;

	}
	return polylines;
}
#endif

#ifdef POLYFIT2
vector<list<int>> FitPolylines(float* points, int nPoints, int intRes)
{
	float tolerance = 1.0/(intRes);
	float mergeTolerance = 100;
	vector<list<int>> pLineList;
	for (int k = 0; k < nPoints; k++)
	{
		list<int> pLine;
		pLine.push_back(k);
		pLineList.push_back(pLine);
	}

	bool merged = true;
	while (merged)
	{
		int	nPLines = pLineList.size();
		// Compute minimal distance between all points/pLineList 
		ClosestPolylineData* closePolyLineDataList = new ClosestPolylineData[nPLines];
		for (int i = 0; i < nPLines; i++)
			(closePolyLineDataList+i)->index = -1;
		for (int i = 0; i < nPLines; i++)
		{
			float closestDistance = intRes*2;
			int closestOrder = -1;
			int jPos = -1;
			int pLine1start	= pLineList[i].front();
			int pLine1end	= pLineList[i].back();
			for (int j = 0; j < nPLines; j++)
			{
				int pLine2start	= pLineList[j].front();
				int pLine2end	= pLineList[j].back();
				int order;
//				float separationDistance = SqrPolylineDistanceP1(pLine1start,pLine1end,pLine2start,pLine2end,points,&order);
				float separationDistance = PolylineDistance(pLine1start,pLine1end,pLine2start,pLine2end,points,&order);
				if (separationDistance <= closestDistance && i!=j)
				{
					closestDistance = separationDistance;
					jPos = j;
					closestOrder = order;
				}
			}
			(closePolyLineDataList+i)->index = i;
			(closePolyLineDataList+i)->closestIndex = jPos;
			(closePolyLineDataList+i)->closestOrder = closestOrder;
			(closePolyLineDataList+i)->closestDist = closestDistance;
		}

		// Sort the pLineList in terms of closest distance
		qsort(closePolyLineDataList, nPLines, sizeof(ClosestPolylineData), PolylineDistCompare);

		// If distance < tolerance, Merge all pLineList
		// else Break
		merged = false;
		for (int i = 0; i < nPLines && (closePolyLineDataList+i)->closestDist < mergeTolerance ; i++)
		{
			int iPos = (closePolyLineDataList+i)->index;
			int closestOrder = (closePolyLineDataList+i)->closestOrder;
			int jPos = (closePolyLineDataList+i)->closestIndex;
			if (pLineList[iPos].size()!=0 && pLineList[jPos].size()!=0 && closestOrder != -1)
			{
				if (closestOrder == 0)
					pLineList[iPos].splice(pLineList[iPos].end(),pLineList[jPos]);
				else if (closestOrder == 1)
					pLineList[iPos].splice(pLineList[iPos].begin(),pLineList[jPos]);
				else if (closestOrder == 2)
				{
					pLineList[jPos].reverse();
					pLineList[iPos].splice(pLineList[iPos].end(),pLineList[jPos]);
				}
				else if (closestOrder == 3)
				{
					pLineList[jPos].reverse();
					pLineList[iPos].splice(pLineList[iPos].begin(),pLineList[jPos]);
				}
				merged = true;
			}
		}
		vector<list<int>> polylinesTransfer;
		for (int i = 0; i < nPLines; i++)
		{
			if (pLineList[i].size() != 0)
			{
				polylinesTransfer.push_back(pLineList[i]);
				pLineList[i].empty();
			}
		}
		pLineList.empty();
		pLineList = polylinesTransfer;
		delete[] closePolyLineDataList;
	}
	return pLineList;
}
#endif

#ifdef POLYFIT3
vector<list<int>> FitPolylines(float* points, int nPoints, int intRes)
{
	float tolerance = 1.0/(intRes);
	float mergeTolerance = 100;
	int surfX = 5;
	int surfY = surfX + 1;
	vector<list<int>> pLineList;
	int* points2DMap = new int[intRes*intRes];
	int* pointsMergeMap = new int[nPoints];
	for (int j = 0; j < intRes; j++)
		for (int i = 0; i < intRes; i++)
			points2DMap[j*intRes + i] = -1;
	for (int k = 0; k < nPoints; k++)
	{
		int xPos = int(points[7*k + surfX]);
		int yPos = int(points[7*k + surfY]);
		if (points2DMap[yPos*intRes + xPos] == -1)
			points2DMap[yPos*intRes + xPos] = k;
		else
		{
			int oldK = points2DMap[yPos*intRes + xPos];
			float floatXPos	= points[7*k + surfX];
			float floatYPos	= points[7*k + surfY];
			float oldXPos	= points[7*oldK + surfX];
			float oldYPos	= points[7*oldK + surfY];
			float oldDistFromCenter = (oldXPos - xPos)*(oldXPos - xPos) + (oldYPos - yPos)*(oldYPos - yPos);
			float distFromCenter	= (floatXPos - xPos)*(floatXPos - xPos) + (floatYPos - yPos)*(floatYPos - yPos);
			if (distFromCenter < oldDistFromCenter)
				points2DMap[yPos*intRes + xPos] = k;
		}
		pointsMergeMap[k] = -1;
	}

//	WriteToFileInt(points2DMap, intRes, intRes);
	int unMergedPoints = nPoints;
	int currentPLineIndex = 0;
	while (unMergedPoints > 0)
	{
		// Find first point in the list not merged.
		int currentIndex = 0;
		while (pointsMergeMap[currentIndex] != -1)
			currentIndex++;

		// Create a new polyline with this point as seed
		list<int> pLine;

		// Traverse along one direction till edge or no more points to be added
		// Find closest neighbour other than the previous point.
		// Previous point is not defined at start
		do
		{
			pLine.push_back(currentIndex);
			unMergedPoints--;
			pointsMergeMap[currentIndex] = currentPLineIndex;
			int xPos = int(points[7*currentIndex + surfX]);
			int yPos = int(points[7*currentIndex + surfY]);
			points2DMap[yPos*intRes + xPos] = -1;

			int nextIndex = FindClosestNeighbour(currentIndex, points2DMap, points, intRes, nPoints, surfX);
			currentIndex = nextIndex;
		} while (currentIndex != -1);
		
		// Traverse along the other direction till edge or no more points to be added
		currentIndex = pLine.front();

		int nextIndex = FindClosestNeighbour(currentIndex, points2DMap, points, intRes, nPoints, surfX);
		currentIndex = nextIndex;
		while (currentIndex != -1)
		{
			pLine.push_front(currentIndex);
			unMergedPoints--;
			pointsMergeMap[currentIndex] = currentPLineIndex;
			int xPos = int(points[7*currentIndex + surfX]);
			int yPos = int(points[7*currentIndex + surfY]);
			points2DMap[yPos*intRes + xPos] = -1;

			int nextIndex = FindClosestNeighbour(currentIndex, points2DMap, points, intRes, nPoints, surfX);
			currentIndex = nextIndex;
		} 

		// Add the polyline to the polyline list
		if (pLine.size() > 1)
		{
			pLineList.push_back(pLine);
			currentPLineIndex++;
		}
	}
	return pLineList;
}
#endif

Float4 QuadIntersection(float A0[3],float A1[3], float A2[3], float A3[3], float B0[3],float B1[3], float B2[3], float B3[3])
{
	// 0 No Intersection
	// 1 A012 Intersects with B012
	// 2 A012 Intersects with B023
	// 4 A023 Intersects with B012
	// 8 A023 Intersects with B023
	// 3 Case 1 and 2
	// 5 Case 1 and 3
	// 9 Case 1 and 4
	// 6 Case 2 and 3
	// 10 Case 2 and 4
	// 12 Case 3 amd 4
	// 7 Case 1 and 2 and 3
	// 14 Case 2 and 3 and 4
	// 13 Case 1 and 3 and 4
	// 11 Case 1 and 2 and 4
	// 15 Case 1 and 2 and 3 and 4

	int iTest1 = NoDivTriTriIsect(A0,A1,A2,B0,B1,B2);
	int iTest2 = NoDivTriTriIsect(A0,A1,A2,B0,B2,B3);
	int iTest3 = NoDivTriTriIsect(A0,A2,A3,B0,B1,B2);
	int iTest4 = NoDivTriTriIsect(A0,A2,A3,B0,B2,B3);
	Float4 iStatus = Float4(iTest1,iTest2,iTest3,iTest4);
	return iStatus;
}

float ClosestPointData(NURBS* surface1, NURBS* surface2, int uIndex1, int vIndex1, int uIndex2, int vIndex2, Float3 *point1, Float3 *point2)
{
	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;

	Float3 A[4];
	Float3 B[4];

	A[0][0] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[0][1] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[0][2] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];

	A[1][0] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[1][1] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[1][2] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];

	A[2][0] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[2][1] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];
	A[2][2] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];

	A[3][0] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[3][1] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];
	A[3][2] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];

	B[0][0] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[0][1] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[0][2] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	
	B[1][0] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[1][1] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[1][2] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];

	B[2][0] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[2][1] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];
	B[2][2] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];

	B[3][0] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[3][1] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];
	B[3][2] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];

	float minDist = -1;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			float dist = Distance(A[i],B[j]);

			int uIndex1add, vIndex1add, uIndex2add, vIndex2add;
			vIndex1add = (i/2.0);
			vIndex2add = (j/2.0);

			if (i==0)
				uIndex1add = 0;
			else if (i==1)
				uIndex1add = 1;
			else if (i==2)
				uIndex1add = 1;
			else if (i==3)
				uIndex1add = 0;

			if (j==0)
				uIndex2add = 0;
			else if (j==1)
				uIndex2add = 1;
			else if (j==2)
				uIndex2add = 1;
			else if (j==3)
				uIndex2add = 0;

			Float2 tempUV1 = Float2(uIndex1 + uIndex1add, vIndex1 + vIndex1add);
			Float2 tempUV2 = Float2(uIndex2 + uIndex2add, vIndex2 + vIndex2add);

			if ((dist < minDist || minDist == -1) && surface1->TrimInOut(tempUV1[0], tempUV1[1], uNum, vNum) && surface2->TrimInOut(tempUV2[0], tempUV2[1], uNum, vNum) )
			{
				*point1 = A[i];
				*point2 = B[j];
				minDist = dist;
			}
		}
	}

//	*point1 = (A[0] + A[1] + A[2] + A[3])/4.0;
//	*point2 = (B[0] + B[1] + B[2] + B[3])/4.0;

	return minDist;
}

bool ExtractIntersectionData(NURBS* surface1, NURBS* surface2, int uIndex1, int vIndex1, int uIndex2, int vIndex2, float* intersectionData)
{
	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;

	float A0[3], A1[3], A2[3], A3[3];
	float B0[3], B1[3], B2[3], B3[3];

	A0[0] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];
	A0[1] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];
	A0[2] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+0)*4+3];

	A1[0] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];
	A1[1] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];
	A1[2] = surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+0)*(uNum+1)*4+(uIndex1+1)*4+3];

	A2[0] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];
	A2[1] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];
	A2[2] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+1)*4+3];

	A3[0] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+0]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];
	A3[1] = surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+1]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];
	A3[2] =	surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+2]/surface1->evalParams->evaluatedPoints[(vIndex1+1)*(uNum+1)*4+(uIndex1+0)*4+3];

	B0[0] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	B0[1] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	B0[2] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+0)*4+3];
	
	B1[0] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];
	B1[1] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];
	B1[2] = surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+0)*(uNum+1)*4+(uIndex2+1)*4+3];

	B2[0] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];
	B2[1] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];
	B2[2] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+1)*4+3];

	B3[0] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+0]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];
	B3[1] = surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+1]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];
	B3[2] =	surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+2]/surface2->evalParams->evaluatedPoints[(vIndex2+1)*(uNum+1)*4+(uIndex2+0)*4+3];

	Float4 iResult = QuadIntersection(A0,A1,A2,A3,B0,B1,B2,B3);

	if (iResult[0] == 0 && iResult[1]==0 && iResult[2]==0 && iResult[3]==0)
		return false;
	
#ifdef DUMP
	if (iResult[0] == 1 && iResult[1]==1 && iResult[2]==1)
	{
		float* data = new float[24];
		data[0] = A0[0];
		data[1] = A0[1];
		data[2] = A0[2];

		data[3] = A1[0];
		data[4] = A1[1];
		data[5] = A1[2];

		data[6] = A2[0];
		data[7] = A2[1];
		data[8] = A2[2];

		data[9] = A3[0];
		data[10] = A3[1];
		data[11] = A3[2];

		data[12] = B0[0];
		data[13] = B0[1];
		data[14] = B0[2];

		data[15] = B1[0];
		data[16] = B1[1];
		data[17] = B1[2];

		data[18] = B2[0];
		data[19] = B2[1];
		data[20] = B2[2];

		data[21] = B3[0];
		data[22] = B3[1];
		data[23] = B3[2];

		WriteToFile(data,3,8);
	}
#endif

	float bBoxMin1[3], bBoxMax1[3], bBoxMin2[3], bBoxMax2[3], bBoxMin[3], bBoxMax[3];
	
	bBoxMin1[0] = surface1->evalParams->minPoints[vIndex1*uNum*4+uIndex1*4+0];
	bBoxMin1[1] = surface1->evalParams->minPoints[vIndex1*uNum*4+uIndex1*4+1];
	bBoxMin1[2] = surface1->evalParams->minPoints[vIndex1*uNum*4+uIndex1*4+2];
	bBoxMax1[0] = surface1->evalParams->maxPoints[vIndex1*uNum*4+uIndex1*4+0];
	bBoxMax1[1] = surface1->evalParams->maxPoints[vIndex1*uNum*4+uIndex1*4+1];
	bBoxMax1[2] = surface1->evalParams->maxPoints[vIndex1*uNum*4+uIndex1*4+2];

	bBoxMin2[0] = surface2->evalParams->minPoints[vIndex2*uNum*4+uIndex2*4+0];
	bBoxMin2[1] = surface2->evalParams->minPoints[vIndex2*uNum*4+uIndex2*4+1];
	bBoxMin2[2] = surface2->evalParams->minPoints[vIndex2*uNum*4+uIndex2*4+2];
	bBoxMax2[0] = surface2->evalParams->maxPoints[vIndex2*uNum*4+uIndex2*4+0];
	bBoxMax2[1] = surface2->evalParams->maxPoints[vIndex2*uNum*4+uIndex2*4+1];
	bBoxMax2[2] = surface2->evalParams->maxPoints[vIndex2*uNum*4+uIndex2*4+2];

	bBoxMin[0] = __max(bBoxMin1[0],bBoxMin2[0]);
	bBoxMin[1] = __max(bBoxMin1[1],bBoxMin2[1]);
	bBoxMin[2] = __max(bBoxMin1[2],bBoxMin2[2]);

	bBoxMax[0] = __min(bBoxMax1[0],bBoxMax2[0]);
	bBoxMax[1] = __min(bBoxMax1[1],bBoxMax2[1]);
	bBoxMax[2] = __min(bBoxMax1[2],bBoxMax2[2]);
	
	Float3 midPoint,midPoint1,midPoint2,midPoint3,midPoint4 ;
	Float2 surf1UV1, surf1UV2, surf1UV3, surf1UV4;
	Float2 surf2UV1, surf2UV2, surf2UV3, surf2UV4;

//  Key for surface points parametric coordinates
//	A0 (uIndex1+0	, vIndex1+0	)
//	A1 (uIndex1+1	, vIndex1+0	)
//	A2 (uIndex1+1	, vIndex1+1	)
//	A3 (uIndex1+0	, vIndex1+1	)

	float intersectionPoint1[3], intersectionPoint2[3];
	int coplanar,intersection,iCount=0;
	if (iResult[0]==1)
	{
		intersection =  tri_tri_intersect_with_isectline(A0,A1,A2,B0,B1,B2,&coplanar,intersectionPoint1,intersectionPoint2);
		if (intersection == 1 && coplanar == 0)
		{
			midPoint1[0] = (intersectionPoint1[0] + intersectionPoint2[0])/2.0;
			midPoint1[1] = (intersectionPoint1[1] + intersectionPoint2[1])/2.0;
			midPoint1[2] = (intersectionPoint1[2] + intersectionPoint2[2])/2.0;
			Float3 surf1Bary	= GetBarycentricCoordinates(midPoint1, A0, A1, A2);
			Float3 surf2Bary	= GetBarycentricCoordinates(midPoint1, B0, B1, B2);
			surf1UV1[0]			= surf1Bary[0]*(uIndex1+0) + surf1Bary[1]*(uIndex1+1) + surf1Bary[2]*(uIndex1+1);
			surf1UV1[1]			= surf1Bary[0]*(vIndex1+0) + surf1Bary[1]*(vIndex1+0) + surf1Bary[2]*(vIndex1+1);
			surf2UV1[0]			= surf2Bary[0]*(uIndex2+0) + surf2Bary[1]*(uIndex2+1) + surf2Bary[2]*(uIndex2+1);
			surf2UV1[1]			= surf2Bary[0]*(vIndex2+0) + surf2Bary[1]*(vIndex2+0) + surf2Bary[2]*(vIndex2+1);
			iCount++;
		}
	}
	if (iResult[1]==1)
	{
		intersection =  tri_tri_intersect_with_isectline(A0,A1,A2,B0,B2,B3,&coplanar,intersectionPoint1,intersectionPoint2);
		if (intersection == 1 && coplanar == 0)
		{
			midPoint2[0] = (intersectionPoint1[0] + intersectionPoint2[0])/2.0;
			midPoint2[1] = (intersectionPoint1[1] + intersectionPoint2[1])/2.0;
			midPoint2[2] = (intersectionPoint1[2] + intersectionPoint2[2])/2.0;
			Float3 surf1Bary	= GetBarycentricCoordinates(midPoint2, A0, A1, A2);
			Float3 surf2Bary	= GetBarycentricCoordinates(midPoint2, B0, B2, B3);
			surf1UV2[0]			= surf1Bary[0]*(uIndex1+0) + surf1Bary[1]*(uIndex1+1) + surf1Bary[2]*(uIndex1+1);
			surf1UV2[1]			= surf1Bary[0]*(vIndex1+0) + surf1Bary[1]*(vIndex1+0) + surf1Bary[2]*(vIndex1+1);
			surf2UV2[0]			= surf2Bary[0]*(uIndex2+0) + surf2Bary[1]*(uIndex2+1) + surf2Bary[2]*(uIndex2+0);
			surf2UV2[1]			= surf2Bary[0]*(vIndex2+0) + surf2Bary[1]*(vIndex2+1) + surf2Bary[2]*(vIndex2+1);
			iCount++;
		}
	}
	if (iResult[2]==1)
	{
		intersection =  tri_tri_intersect_with_isectline(A0,A2,A3,B0,B1,B2,&coplanar,intersectionPoint1,intersectionPoint2);
		if (intersection == 1 && coplanar == 0)
		{
			midPoint3[0] = (intersectionPoint1[0] + intersectionPoint2[0])/2.0;
			midPoint3[1] = (intersectionPoint1[1] + intersectionPoint2[1])/2.0;
			midPoint3[2] = (intersectionPoint1[2] + intersectionPoint2[2])/2.0;
			Float3 surf1Bary	= GetBarycentricCoordinates(midPoint3, A0, A2, A3);
			Float3 surf2Bary	= GetBarycentricCoordinates(midPoint3, B0, B1, B2);
			surf1UV3[0]			= surf1Bary[0]*(uIndex1+0) + surf1Bary[1]*(uIndex1+1) + surf1Bary[2]*(uIndex1+0);
			surf1UV3[1]			= surf1Bary[0]*(vIndex1+0) + surf1Bary[1]*(vIndex1+1) + surf1Bary[2]*(vIndex1+1);
			surf2UV3[0]			= surf2Bary[0]*(uIndex2+0) + surf2Bary[1]*(uIndex2+1) + surf2Bary[2]*(uIndex2+1);
			surf2UV3[1]			= surf2Bary[0]*(vIndex2+0) + surf2Bary[1]*(vIndex2+0) + surf2Bary[2]*(vIndex2+1);
			iCount++;
		}
	}
	if (iResult[3]==1)
	{
		intersection =  tri_tri_intersect_with_isectline(A0,A2,A3,B0,B2,B3,&coplanar,intersectionPoint1,intersectionPoint2);
		if (intersection == 1 && coplanar == 0)
		{
			midPoint4[0] = (intersectionPoint1[0] + intersectionPoint2[0])/2.0;
			midPoint4[1] = (intersectionPoint1[1] + intersectionPoint2[1])/2.0;
			midPoint4[2] = (intersectionPoint1[2] + intersectionPoint2[2])/2.0;
			Float3 surf1Bary	= GetBarycentricCoordinates(midPoint4, A0, A2, A3);
			Float3 surf2Bary	= GetBarycentricCoordinates(midPoint4, B0, B2, B3);
			surf1UV4[0]			= surf1Bary[0]*(uIndex1+0) + surf1Bary[1]*(uIndex1+1) + surf1Bary[2]*(uIndex1+0);
			surf1UV4[1]			= surf1Bary[0]*(vIndex1+0) + surf1Bary[1]*(vIndex1+1) + surf1Bary[2]*(vIndex1+1);
			surf2UV4[0]			= surf2Bary[0]*(uIndex2+0) + surf2Bary[1]*(uIndex2+1) + surf2Bary[2]*(uIndex2+0);
			surf2UV4[1]			= surf2Bary[0]*(vIndex2+0) + surf2Bary[1]*(vIndex2+1) + surf2Bary[2]*(vIndex2+1);
			iCount++;
		}
	}

	if (iCount == 0)
		return false;

	midPoint = (midPoint1 + midPoint2 + midPoint3 + midPoint4)/iCount;
	float surf1u = (surf1UV1[0] + surf1UV2[0] + surf1UV3[0] + surf1UV4[0])/iCount;
	float surf1v = (surf1UV1[1] + surf1UV2[1] + surf1UV3[1] + surf1UV4[1])/iCount;
	float surf2u = (surf2UV1[0] + surf2UV2[0] + surf2UV3[0] + surf2UV4[0])/iCount;
	float surf2v = (surf2UV1[1] + surf2UV2[1] + surf2UV3[1] + surf2UV4[1])/iCount;
	intersectionData[0] = midPoint[0];
	intersectionData[1] = midPoint[1];
	intersectionData[2] = midPoint[2];
	intersectionData[3] = surf1u;
	intersectionData[4] = surf1v;
	intersectionData[5] = surf2u;
	intersectionData[6] = surf2v;

//	if ((midPoint[0] - bBoxMin[0]) < DELTA || (bBoxMax[0] - midPoint[0]) < DELTA ||
//		(midPoint[1] - bBoxMin[1]) < DELTA || (bBoxMax[1] - midPoint[1]) < DELTA ||
//		(midPoint[2] - bBoxMin[2]) < DELTA || (bBoxMax[2] - midPoint[2]) < DELTA   )
//		return false;

	return true;
}



#ifdef GPUFIND
int GPUSurfaceIntersection(NURBS* surface1, NURBS* surface2, GLParameters* glParam, float* &ibox1, float* &ibox2, float* &iData, int iterationNum, Float4* timeData)
{
	//Set up for evaluation
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_DEPTH_TEST);	glDepthFunc(GL_LEQUAL);	
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ZERO);

	vector<Float4> referenceList;
	referenceList.clear();
	Float4 initialReference = Float4(0,0,0,0);
	referenceList.push_back(initialReference);

	GLuint referenceListTex[3];
	glGenTextures(3, referenceListTex);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[0]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, 1, 1, 0, GL_RGBA, GL_FLOAT, initialReference.value);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[1]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[2]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);

	int drawTexture=0;

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;
	int maxLevel = GetExponent2(uNum);
	
	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2,tempTex);

	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	int sizeX,sizeY;
	int numOverlaps = 1;

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

	cgGLSetTextureParameter( glParam->bBoxMinATextureParam, surface1->evalParams->bBoxTextures[0]);
	cgGLSetTextureParameter( glParam->bBoxMaxATextureParam, surface1->evalParams->bBoxTextures[1]);
	cgGLSetTextureParameter( glParam->bBoxMinBTextureParam, surface2->evalParams->bBoxTextures[0]);
	cgGLSetTextureParameter( glParam->bBoxMaxBTextureParam, surface2->evalParams->bBoxTextures[1]);

	for (int level = 1; level <= maxLevel; level++)
	{
		int levelOriginX = int((pow(2.0,maxLevel-level)-1)*pow(2.0,level+1));
		drawTexture = (drawTexture+1)%2;
		int referenceListSize = ceil(sqrt(numOverlaps*1.0));

		int intersectionArraySizeX = referenceListSize*4;
		int intersectionArraySizeY = referenceListSize*4;

		// Create a texture that is a power of two texture
		cgGLBindProgram(glParam->bBoxOverlapProgram);

		cgGLEnableTextureParameter(	glParam->bBoxMinATextureParam);
    	cgGLEnableTextureParameter(	glParam->bBoxMaxATextureParam);
		cgGLEnableTextureParameter(	glParam->bBoxMinBTextureParam);
		cgGLEnableTextureParameter(	glParam->bBoxMaxBTextureParam);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, intersectionArraySizeX, intersectionArraySizeY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

		cgGLSetTextureParameter( glParam->bBoxReferenceListTextureParam, referenceListTex[(drawTexture+1)%2]);

		cgGLEnableTextureParameter(	glParam->bBoxReferenceListTextureParam);
		cgGLSetParameter1f(glParam->bBoxLevelOriginParam, levelOriginX);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE4_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[1]);
		glActiveTextureARB(GL_TEXTURE3_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[0]);
		glActiveTextureARB(GL_TEXTURE2_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[1]);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[0]);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[(drawTexture+1)%2]);

		glViewport(0, 0, intersectionArraySizeX, intersectionArraySizeY);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		glClear(GL_COLOR_BUFFER_BIT);
		glFlush();
	
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB,glParam->occlusionQuery);
		DrawQuad(intersectionArraySizeX, intersectionArraySizeY);
		glFlush();
		glFinish();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);

		GLuint fragmentCount;
		glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &fragmentCount);
		numOverlaps = fragmentCount;

#ifdef DUMP
		GLfloat* retVal=new GLfloat[intersectionArraySizeX*intersectionArraySizeY];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,intersectionArraySizeX, intersectionArraySizeY, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal,intersectionArraySizeX, intersectionArraySizeY);
#endif

		cgGLDisableTextureParameter(glParam->bBoxMinATextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMaxATextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMinBTextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMaxBTextureParam);
		cgGLDisableTextureParameter(glParam->bBoxReferenceListTextureParam);

		vector<int> positions;

		GLuint tex;
		clock_t begintime;
		if (timeData != NULL)
			begintime = clock();

		tex =  FindRenderLocation(tempTex[0], &numOverlaps, intersectionArraySizeX, intersectionArraySizeY, &positions, glParam, true, timeData);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		if (timeData != NULL)
		{
			clock_t endtime = clock();
			timeData->value[3] += float(endtime-begintime);
		}

		sizeX = ceil(sqrt(numOverlaps*1.0));
		sizeY = sizeX;

//		sizeX = numOverlaps%1024;
//		sizeY = int(numOverlaps/1024.0)+1;

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[drawTexture]);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, sizeX, sizeY, 0, GL_RGBA, GL_FLOAT, NULL);

		float sizeData[4];
		sizeData[0] = sizeX;
		sizeData[1] = sizeY;
		sizeData[2] = numOverlaps;
		sizeData[3] = 0;

		cgGLBindProgram(glParam->referenceProgram);

		cgGLSetTextureParameter(glParam->referenceOldListTexParam, referenceListTex[(drawTexture+1)%2]);
		cgGLEnableTextureParameter(glParam->referenceOldListTexParam);
		cgGLSetTextureParameter(glParam->referenceOverlapTexParam, tex);
		cgGLEnableTextureParameter(glParam->referenceOverlapTexParam);
		
		cgGLSetParameter4fv(glParam->referenceSizeDataParam, sizeData);

		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[(drawTexture+1)%2]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tex);
			
		glViewport(0, 0, sizeX, sizeY);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, referenceListTex[drawTexture], 0);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);
		glFlush();

		DrawQuad(sizeX, sizeY);
		glFlush();
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[sizeX*sizeY*4];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, sizeX, sizeY ,GL_RGBA, GL_FLOAT, retVal);
		WriteToFile(retVal, 4, sizeX*sizeY);
#endif
		cgGLDisableTextureParameter(glParam->referenceOldListTexParam);
		cgGLDisableTextureParameter(glParam->referenceOverlapTexParam);
		glDeleteTextures(1,&tex);
	}                                    /// For each level loop ends

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[2]);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, sizeX, sizeY, 0, GL_RGBA, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[(drawTexture+1)%2]);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, sizeX, sizeY, 0,GL_RGBA, GL_FLOAT, NULL);

	const GLenum buffers[] = {GL_COLOR_ATTACHMENT0_EXT,GL_COLOR_ATTACHMENT1_EXT};
	glDrawBuffers(2, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, referenceListTex[2], 0);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, referenceListTex[(drawTexture+1)%2], 0);

	cgGLBindProgram(glParam->intervalProgram);
	cgGLSetTextureParameter(glParam->intervalRefListTexParam, referenceListTex[drawTexture]);
	cgGLEnableTextureParameter(glParam->intervalRefListTexParam);

	float minMax1[4];
	minMax1[0] = surface1->evalParams->uMin;
	minMax1[1] = surface1->evalParams->uMax;
	minMax1[2] = surface1->evalParams->vMin;
	minMax1[3] = surface1->evalParams->vMax;

	float minMax2[4];
	minMax2[0] = surface2->evalParams->uMin;
	minMax2[1] = surface2->evalParams->uMax;
	minMax2[2] = surface2->evalParams->vMin;
	minMax2[3] = surface2->evalParams->vMax;

	float uvNum[2];
	uvNum[0] = uNum;
	uvNum[1] = vNum;

	cgGLSetParameter4fv(glParam->intervalMinMax1, minMax1);
	cgGLSetParameter4fv(glParam->intervalMinMax2, minMax2);
	cgGLSetParameter2fv(glParam->intervalUVNum, uvNum);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, referenceListTex[drawTexture]);
	
	glViewport(0, 0, sizeX, sizeY);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(sizeX, sizeY);
	glFlush();

	ibox1=new float[sizeX*sizeY*4];
	ibox2=new float[sizeX*sizeY*4];

	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, sizeX, sizeY, GL_RGBA, GL_FLOAT, ibox1);

	glReadBuffer(GL_COLOR_ATTACHMENT1_EXT);
	glReadPixels(0, 0, sizeX, sizeY, GL_RGBA, GL_FLOAT, ibox2);

	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glFinish();
	
	cgGLDisableTextureParameter(glParam->intervalRefListTexParam);

	glDrawBuffers(1, buffers);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT1_EXT, GL_TEXTURE_RECTANGLE_ARB, 0, 0);

	//Unbind the frame buffer and disable the program
	cgGLDisableProfile(glParam->cgFragmentProfile);

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	glDeleteTextures(2, tempTex);
	glDeleteTextures(3, referenceListTex);

	// Restore the previous views
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
	return numOverlaps;
}
#else
int GPUSurfaceIntersection(NURBS* surface1, NURBS* surface2, GLParameters* glParam, float* &ibox1, float* &ibox2, float* &iData, int iterationNum, Float4* timeData)
{
	// Set up GPU computations
	StartGPUComputation(glParam);

	//Set up for evaluation
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ZERO);

	vector<Float4> referenceList;
	referenceList.clear();
	Float4 initialReference = Float4(0,0,0,0);
	referenceList.push_back(initialReference);

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;
	int maxLevel = GetExponent2(uNum);
	
	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2,tempTex);

	//Enable surface generation program
	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	int viewportx,viewporty;
	int numOverlaps=1;

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

	cgGLSetTextureParameter( glParam->bBoxMinATextureParam, surface1->evalParams->bBoxTextures[0]);
	cgGLSetTextureParameter( glParam->bBoxMaxATextureParam, surface1->evalParams->bBoxTextures[1]);
	cgGLSetTextureParameter( glParam->bBoxMinBTextureParam, surface2->evalParams->bBoxTextures[0]);
	cgGLSetTextureParameter( glParam->bBoxMaxBTextureParam, surface2->evalParams->bBoxTextures[1]);

	for (int level = 1; level <= maxLevel; level++)
	{
		int levelOriginX = int((pow(2.0,maxLevel-level)-1)*pow(2.0,level+1));
		int referenceListSize = ceil(sqrt(referenceList.size()*1.0));

		int intersectionArraySizeX = referenceListSize*4;
		int intersectionArraySizeY = referenceListSize*4;

		// Create a texture that is a power of two texture
		float* referenceListData = new float[referenceListSize*referenceListSize*4];
		for (int j = 0; j < referenceListSize; j++)
		{
			for (int i = 0; i < referenceListSize; i++)
			{
				if (j*referenceListSize + i < referenceList.size())
				{
					referenceListData[(j*referenceListSize + i)*4 + 0] = referenceList[j*referenceListSize + i][0];
					referenceListData[(j*referenceListSize + i)*4 + 1] = referenceList[j*referenceListSize + i][1];
					referenceListData[(j*referenceListSize + i)*4 + 2] = referenceList[j*referenceListSize + i][2];
					referenceListData[(j*referenceListSize + i)*4 + 3] = referenceList[j*referenceListSize + i][3];
				}
				else
				{
					referenceListData[(j*referenceListSize + i)*4 + 0] = -1;
					referenceListData[(j*referenceListSize + i)*4 + 1] = -1;
					referenceListData[(j*referenceListSize + i)*4 + 2] = -1;
					referenceListData[(j*referenceListSize + i)*4 + 3] = -1;
				}
			}
		}

		cgGLBindProgram(glParam->bBoxOverlapProgram);

		cgGLEnableTextureParameter(	glParam->bBoxMinATextureParam);
    	cgGLEnableTextureParameter(	glParam->bBoxMaxATextureParam);
		cgGLEnableTextureParameter(	glParam->bBoxMinBTextureParam);
		cgGLEnableTextureParameter(	glParam->bBoxMaxBTextureParam);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, intersectionArraySizeX, intersectionArraySizeY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

		InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, referenceListSize, referenceListSize, GL_RGBA, referenceListData);

		cgGLSetTextureParameter( glParam->bBoxReferenceListTextureParam, tempTex[1]);

		cgGLEnableTextureParameter(	glParam->bBoxReferenceListTextureParam);
		cgGLSetParameter1f(glParam->bBoxLevelOriginParam, levelOriginX);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE4_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[1]);
		glActiveTextureARB(GL_TEXTURE3_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[0]);
		glActiveTextureARB(GL_TEXTURE2_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[1]);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[0]);
		glActiveTextureARB(GL_TEXTURE0_ARB);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);

		glViewport(0, 0, intersectionArraySizeX, intersectionArraySizeY);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		glClear(GL_COLOR_BUFFER_BIT);
		glFlush();
	
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB,glParam->occlusionQuery);
		DrawQuad(intersectionArraySizeX, intersectionArraySizeY);
		glFlush();
		glFinish();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);

		GLuint fragmentCount;
		glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &fragmentCount);
		numOverlaps = fragmentCount;

#ifdef DUMP
		GLfloat* retVal=new GLfloat[intersectionArraySizeX*intersectionArraySizeY];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,intersectionArraySizeX, intersectionArraySizeY, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal,intersectionArraySizeX, intersectionArraySizeY);
#endif

		cgGLDisableTextureParameter(glParam->bBoxMinATextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMaxATextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMinBTextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMaxBTextureParam);
		cgGLDisableTextureParameter(glParam->bBoxReferenceListTextureParam);

		vector<int> positions;

		clock_t begintime;
		if (timeData != NULL)
			begintime = clock();

		FindRenderLocation(tempTex[0], &numOverlaps, intersectionArraySizeX, intersectionArraySizeY, &positions, glParam, false, timeData);

		if (timeData != NULL)
		{
			clock_t endtime = clock();
			timeData->value[3] += (endtime-begintime);
		}

		numOverlaps = positions.size();
		vector<Float4> newReferenceList;
		for (int overlap = 0; overlap < numOverlaps; overlap++)
		{
			int pos = positions[overlap];
			int xPos = pos%intersectionArraySizeX;
			int yPos = int(pos/intersectionArraySizeX);
			int referenceListLocationX = int(xPos/4.0);
			int referenceListLocationY = int(yPos/4.0);
			int boxLocationAx = ((xPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][0]))*2;
			int boxLocationAy = (int((xPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][1]))*2;
			int boxLocationBx = ((yPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][2]))*2;
			int boxLocationBy = (int((yPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][3]))*2;
			Float4 refCoords = Float4(boxLocationAx, boxLocationAy, boxLocationBx, boxLocationBy);
			newReferenceList.push_back(refCoords);
		}
		referenceList.clear();
		referenceList = newReferenceList;
		delete[] referenceListData;
	}                                    /// For each level loop ends

	clock_t begintime;
	if (timeData != NULL)
		begintime = clock();

	vector<Float4> tempIntParamRange1;
	vector<Float4> tempIntParamRange2;
	vector<Float4> tempIntDataVector1;
	vector<Float3> tempIntDataVector2;
	int numIntersectionBoxes = 0;

	for (int k = 0; k < referenceList.size(); k++)
	{
		int uIndex1 = (int) referenceList[k][0]/2.0;
		int vIndex1 = (int) referenceList[k][1]/2.0;
		int uIndex2 = (int) referenceList[k][2]/2.0;
		int vIndex2 = (int) referenceList[k][3]/2.0;

		float uMin1 = float(uIndex1*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
		float vMin1 = float(vIndex1*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);
		float uMax1 = float((uIndex1+1)*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
		float vMax1 = float((vIndex1+1)*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);

		Float4 intRange1 = Float4(uMin1,uMax1,vMin1,vMax1);
	
		float uMin2 = float(uIndex2*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
		float vMin2 = float(vIndex2*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);
		float uMax2 = float((uIndex2+1)*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
		float vMax2 = float((vIndex2+1)*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);

		Float4 intRange2 = Float4(uMin2,uMax2,vMin2,vMax2);

		// Self intersection Test
		bool isIntersecting = true;
		if (fabs(uMin2 - uMin1) <= (surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum)  && fabs(vMin2 - vMin1)<= (surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) && glParam->selfIntersection)
			isIntersecting = false;

		if (glParam->readBack && isIntersecting)
		{
			float intersectionData[7];
			isIntersecting = ExtractIntersectionData(surface1,surface2,uIndex1,vIndex1,uIndex2,vIndex2,intersectionData);
			if (isIntersecting)
			{
				Float4 tempIntData1 = Float4(intersectionData[0],intersectionData[1],intersectionData[2],intersectionData[3]);
				Float3 tempIntData2 = Float3(intersectionData[4],intersectionData[5],intersectionData[6]);
				tempIntDataVector1.push_back(tempIntData1);
				tempIntDataVector2.push_back(tempIntData2);
			}
		}
		if (isIntersecting)
		{
			tempIntParamRange1.push_back(intRange1);
			tempIntParamRange2.push_back(intRange2);
			numIntersectionBoxes++;
		}
	}

	ibox1=new float[numIntersectionBoxes*4];
	ibox2=new float[numIntersectionBoxes*4];
	if (glParam->readBack)
		iData = new float[numIntersectionBoxes*7];

	for (int k = 0; k < numIntersectionBoxes; k++)
	{
		ibox1[4*k+0] = tempIntParamRange1[k][0];
		ibox1[4*k+1] = tempIntParamRange1[k][1];
		ibox1[4*k+2] = tempIntParamRange1[k][2];
		ibox1[4*k+3] = tempIntParamRange1[k][3];

		ibox2[4*k+0] = tempIntParamRange2[k][0];
		ibox2[4*k+1] = tempIntParamRange2[k][1];
		ibox2[4*k+2] = tempIntParamRange2[k][2];
		ibox2[4*k+3] = tempIntParamRange2[k][3];
		if (glParam->readBack)
		{
			iData[7*k+0] = tempIntDataVector1[k][0];
			iData[7*k+1] = tempIntDataVector1[k][1];
			iData[7*k+2] = tempIntDataVector1[k][2];
			iData[7*k+3] = tempIntDataVector1[k][3];
			iData[7*k+4] = tempIntDataVector2[k][0];
			iData[7*k+5] = tempIntDataVector2[k][1];
			iData[7*k+6] = tempIntDataVector2[k][2];
		}
	}

	tempIntParamRange1.clear();
	tempIntParamRange2.clear();
	tempIntDataVector1.clear();
	tempIntDataVector2.clear();

	glDeleteTextures(2, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (timeData != NULL)
	{
		clock_t endtime = clock();
		timeData->value[2] += (endtime-begintime);
	}

	return numIntersectionBoxes;
}
#endif
void CalculateSurfaceIntersection(vector<NURBS*>* nurbsSurfaces, int intRes, GLParameters* glParam)
{
	glParam->intersection = true;
	int uNum = intRes;
	int vNum = intRes;
	bool timing = false;

	Float4 readTime = Float4(0,0,0,0);
	long unsigned int evaluationTime = 0;
	long unsigned int intersectionTime = 0;
	long unsigned int fittingTime = 0;
	clock_t beginIntTime, beginFittingTime, beginTime;
	if (timing)
		beginTime = clock();

	for (int surfNum = 0; surfNum < nurbsSurfaces->size(); surfNum++)
	{
		NURBS* surface = nurbsSurfaces->at(surfNum);
		bool evaluate = true;
		if(surface->initializedEvalParams)
		{
			if (surface->evalParams->uMin == 0.0  &&
				surface->evalParams->uMax == 1.0  &&
				surface->evalParams->vMin == 0.0  &&
				surface->evalParams->vMax == 1.0  &&
				surface->evalParams->uNum == uNum &&
				surface->evalParams->vNum == vNum)
					evaluate = false;
		}
		if (evaluate)
		{
			if (surface->initializedEvalParams)
				delete surface->evalParams;
			surface->evalParams = new EvaluationParams();
			surface->initializedEvalParams = true;

			// uMin, uMax, vMin, vMax
			surface->evalParams->uNum	= uNum;
			surface->evalParams->vNum	= vNum;
			surface->evalParams->uMin	= 0.0;
			surface->evalParams->uMax	= 1.0;
			surface->evalParams->vMin	= 0.0;
			surface->evalParams->vMax	= 1.0;
			surface->numIntersectingBoxes = 0;
			if (surface->numIntersectingBoxes != 0)
				delete [] surface->intersectingBoxes;
			surface->evaluatedIntersection = false;

			if (glParam->readBack)
			{
				surface->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
				surface->evalParams->minPoints			= new float[(uNum)*(vNum)*4];
				surface->evalParams->maxPoints			= new float[(uNum)*(vNum)*4];
			}
			surface->SurfaceEvaluation(glParam, false, false);
			surface->SurfaceBoundingBoxEvaluation(glParam);
		}
	}

	if (timing)
		evaluationTime += clock() - beginTime;

	for (int iSurfNum = 0; iSurfNum < nurbsSurfaces->size(); iSurfNum++)
	{
		for (int jSurfNum = iSurfNum+1; jSurfNum < nurbsSurfaces->size(); jSurfNum++)
		{
			NURBS* surface1 = nurbsSurfaces->at(iSurfNum);
			NURBS* surface2 = nurbsSurfaces->at(jSurfNum);
			float* interval1	= NULL;
			float* interval2	= NULL;
			float* intData		= NULL;

			if (timing)
				beginIntTime = clock();

			int numIntervals = 0;
			if (AABBIntersection(surface1->bBoxMin, surface1->bBoxMax, surface2->bBoxMin, surface2->bBoxMax))
				numIntervals = GPUSurfaceIntersection(surface1, surface2, glParam, interval1, interval2, intData, 0);

			if (numIntervals > 0)
				cout<<"Number of Points  : "<< numIntervals<< endl;

			if (timing)
			{
				intersectionTime += clock() - beginIntTime;
				beginFittingTime = clock();
			}

			if (glParam->readBack)
			{
//				vector<list<int>> polylines = FitPolylines(intData, numIntervals, intRes);
//				cout<<"Number of Segments: " << polylines.size() << endl;
			}
			if (timing)	
				fittingTime += clock() - beginFittingTime;

			if (numIntervals > 0)
			{
				if (surface1->numIntersectingBoxes > 0)
				{
					surface1->numIntersectingBoxes += numIntervals;
					int newIntBoxSize1 = surface1->numIntersectingBoxes;
					surface1->intersectingBoxes = (float*)realloc(surface1->intersectingBoxes, 4*newIntBoxSize1*sizeof(float));
					memcpy(surface1->intersectingBoxes + 4*(newIntBoxSize1 - numIntervals), interval1, numIntervals*4*sizeof(float));
				}
				else
				{
					surface1->numIntersectingBoxes = numIntervals;
					surface1->intersectingBoxes = interval1;
				}

				if (surface2->numIntersectingBoxes > 0)
				{
					surface2->numIntersectingBoxes += numIntervals;
					int newIntBoxSize2 = surface2->numIntersectingBoxes;
					surface2->intersectingBoxes = (float*)realloc(surface2->intersectingBoxes, 4*newIntBoxSize2*sizeof(float));
					memcpy(surface2->intersectingBoxes + 4*(newIntBoxSize2 - numIntervals), interval2, numIntervals*4*sizeof(float));
				}
				else
				{
					surface2->numIntersectingBoxes = numIntervals;
					surface2->intersectingBoxes = interval2;
				}
				surface1->evaluatedIntersection = true;
				surface2->evaluatedIntersection = true;
			}
			delete[] intData;
		}
	}
	for (int surfNum = 0; surfNum < nurbsSurfaces->size(); surfNum++)
	{
		if (nurbsSurfaces->at(surfNum)->evaluatedIntersection)
			nurbsSurfaces->at(surfNum)->GenerateIntersectionOverlayTexture(nurbsSurfaces->at(surfNum)->intersectingBoxes, nurbsSurfaces->at(surfNum)->numIntersectingBoxes, intRes, glParam);
	}

	glParam->intersection = false;

	if (timing)
	{
		clock_t endTime = clock();
		cout<<"Total Time        : "<<(0.001*(endTime - beginTime))<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationTime)<<endl;
		cout<<"Intersection Time : "<<(0.001*intersectionTime)<<endl;
		cout<<"Curve Fitting Time: "<<(0.001*(fittingTime))<<endl<<endl;
	}
}

void CalculateSurfaceIntersection(NURBS* surface1, NURBS* surface2, int intRes, GLParameters* glParam, bool forcedEvaluate)
{
	glParam->intersection = true;
	int uNum = intRes;
	int vNum = intRes;
	bool timing = true;

	Float4 readTime = Float4(0,0,0,0);
	long unsigned int evaluationtime = 0;
	long unsigned int intersectiontime = 0;
	clock_t beginTime, beginIntTime, beginEvalTime, beginFittingTime;

	if (timing)
		beginTime = clock();

	bool evaluate = true;
	if(surface1->initializedEvalParams && surface2->initializedEvalParams && !forcedEvaluate)
	{
		if (surface1->evalParams->uMin == 0.0  &&
			surface1->evalParams->uMax == 1.0  &&
			surface1->evalParams->vMin == 0.0  &&
			surface1->evalParams->vMax == 1.0  &&
			surface1->evalParams->uNum == uNum &&
			surface1->evalParams->vNum == vNum &&
			surface2->evalParams->uMin == 0.0  &&
			surface2->evalParams->uMax == 1.0  &&
			surface2->evalParams->vMin == 0.0  &&
			surface2->evalParams->vMax == 1.0  &&
			surface2->evalParams->uNum == uNum &&
			surface2->evalParams->vNum == vNum)
				evaluate = false;
	}

	if (!evaluate && forcedEvaluate)
		evaluate = true;

	if (evaluate)
	{
		if (surface1->initializedEvalParams)
			delete surface1->evalParams;
		surface1->evalParams = new EvaluationParams();
		surface1->initializedEvalParams = true;

		if (surface2->initializedEvalParams)
			delete surface2->evalParams;
		surface2->evalParams = new EvaluationParams();
		surface2->initializedEvalParams = true;
	}
	if (surface1->numIntersectingBoxes > 0)
		delete [] surface1->intersectingBoxes;
	surface1->numIntersectingBoxes = 0;
	surface1->evaluatedIntersection = false;

	if (surface2->numIntersectingBoxes > 0)
	delete [] surface2->intersectingBoxes;
	surface2->numIntersectingBoxes = 0;
	surface2->evaluatedIntersection = false;

	int maxIterations = 1;
	
	// uMin, uMax, vMin, vMax
	int numIntervals = 1;
	float *interval1 = new float[4];
	interval1[0] = 0.0;
	interval1[1] = 1.0;
	interval1[2] = 0.0;
	interval1[3] = 1.0;

	float *interval2 = new float[4];
	interval2[0] = 0.0;
	interval2[1] = 1.0;
	interval2[2] = 0.0;
	interval2[3] = 1.0;

	float* intData = new float;

//	int iterationFactor = 32;
//	int uBaseNum = max(surface1->uBaseNum,surface2->uBaseNum);
//	int vBaseNum = max(surface1->uBaseNum,surface2->uBaseNum);
//	int baseNum  = max(uBaseNum,vBaseNum);
//	int uNum = GetNextPower2(baseNum*iterationFactor);
//	int vNum = GetNextPower2(baseNum*iterationFactor);

	bool readBack = glParam->readBack;
	glParam->readBack = false;
	for (int iterationNum = 0; iterationNum < maxIterations; iterationNum++)
	{
		if (readBack && iterationNum == maxIterations-1)
			glParam->readBack = true;
		surface1->evalParams->uNum	= uNum;
		surface1->evalParams->vNum	= vNum;
		surface2->evalParams->uNum	= uNum;
		surface2->evalParams->vNum	= vNum;

		float* newInt1 = NULL;
		float* newInt2 = NULL;
		float* newIntData = NULL;
		int newIntBoxSize = 0;
		for( int oldIntersectionCount = 0; oldIntersectionCount < numIntervals; oldIntersectionCount++)
		{
			surface1->evalParams->uMin = interval1[oldIntersectionCount*4+0];
			surface1->evalParams->uMax = interval1[oldIntersectionCount*4+1];
			surface1->evalParams->vMin = interval1[oldIntersectionCount*4+2];
			surface1->evalParams->vMax = interval1[oldIntersectionCount*4+3];

			surface2->evalParams->uMin = interval2[oldIntersectionCount*4+0];
			surface2->evalParams->uMax = interval2[oldIntersectionCount*4+1];
			surface2->evalParams->vMin = interval2[oldIntersectionCount*4+2];
			surface2->evalParams->vMax = interval2[oldIntersectionCount*4+3];
	
			if (timing)
				beginEvalTime = clock();

			if (evaluate)
			{
				if (glParam->readBack)
				{
					surface1->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
					surface1->evalParams->minPoints			= new float[(uNum)*(vNum)*4];
					surface1->evalParams->maxPoints			= new float[(uNum)*(vNum)*4];
					surface2->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
					surface2->evalParams->minPoints			= new float[(uNum)*(vNum)*4];
					surface2->evalParams->maxPoints			= new float[(uNum)*(vNum)*4];
				}
				surface1->SurfaceEvaluation(glParam, false, false);
				surface2->SurfaceEvaluation(glParam, false, false);
			}
			if (evaluate || !surface1->identityTransformation)
				surface1->SurfaceBoundingBoxEvaluation(glParam);
			if (evaluate || !surface2->identityTransformation)
				surface2->SurfaceBoundingBoxEvaluation(glParam);

			float *ibox1 = NULL;
			float *ibox2 = NULL;
			float *iData = NULL;
			int numOverlaps;

			if (timing)
			{
				beginIntTime = clock();
				evaluationtime += beginIntTime - beginEvalTime;
				numOverlaps = GPUSurfaceIntersection(surface1, surface2, glParam, ibox1, ibox2, iData, iterationNum, &readTime);
				intersectiontime += clock() - beginIntTime;
			}
			else
				numOverlaps = GPUSurfaceIntersection(surface1, surface2, glParam, ibox1, ibox2, iData, iterationNum);

			newIntBoxSize += numOverlaps;
			newInt1 = (float*)realloc(newInt1,4*newIntBoxSize*sizeof(float));
			newInt2 = (float*)realloc(newInt2,4*newIntBoxSize*sizeof(float));
			memcpy(newInt1+4*(newIntBoxSize-numOverlaps),ibox1,numOverlaps*4*sizeof(float));
			memcpy(newInt2+4*(newIntBoxSize-numOverlaps),ibox2,numOverlaps*4*sizeof(float));
			if (glParam->readBack)
			{
				newIntData = (float*)realloc(newIntData,7*newIntBoxSize*sizeof(float));
				memcpy(newIntData+7*(newIntBoxSize-numOverlaps),iData,numOverlaps*7*sizeof(float));
				delete[] iData;
			}
			delete[] ibox1;
			delete[] ibox2;
		}
		delete[] interval1;
		delete[] interval2;
		interval1 = newInt1;
		interval2 = newInt2;
		intData  = newIntData;
		numIntervals = newIntBoxSize;
	}
	
	if (timing)
		beginFittingTime = clock();

	if (glParam->readBack)
	{
		vector<list<int>> polylines = FitPolylines(intData, numIntervals, int(pow(float(intRes),maxIterations)));
//		DumpPolylineData(polylines,intData,numIntervals);
		if (timing)
		{
			int polylinePoints = 0;
			for(int j=0; j < polylines.size(); j++)
				polylinePoints += polylines[j].size();
			cout<<"Number of Segments: " << polylines.size() << endl;
			cout<<"Points in Polyline: " << polylinePoints << endl;
		}
	}

	if (timing)
	{
		clock_t endTime = clock();
		cout<<"Number of Points  : "<< numIntervals<< endl;
		cout<<"Total Time        : "<<(0.001*(endTime - beginTime))<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationtime)<<endl;
		cout<<"Intersection Time : "<<(0.001*intersectiontime)<<endl;
		cout<<"Find Time         : "<<(0.001*readTime[3])<<endl;
		cout<<"CPU Tri-Tri Time  : "<<(0.001*readTime[2])<<endl;
		cout<<"Curve Fitting Time: "<<(0.001*(endTime - beginFittingTime))<<endl<<endl;
	//	cout<<"Ratio Find Time 1 : "<<(1.*readTime[0])/readTime[3]<<endl;
	//	cout<<"Ratio Find Time 2 : "<<(1.*readTime[1])/readTime[3]<<endl;
	//	cout<<"Ratio Find Time 3 : "<<(1.*readTime[2])/readTime[3]<<endl;
	}

#ifndef AUTOTIMING
	surface1->numIntersectingBoxes = numIntervals;
	surface2->numIntersectingBoxes = numIntervals;
	surface1->intersectingBoxes = interval1;
	surface2->intersectingBoxes = interval2;
	surface1->evaluatedIntersection = true;
	surface2->evaluatedIntersection = true;

	surface1->GenerateIntersectionOverlayTexture(surface1->intersectingBoxes, surface1->numIntersectingBoxes, (int)pow(intRes*1.0,maxIterations), glParam);
	surface2->GenerateIntersectionOverlayTexture(surface2->intersectingBoxes, surface2->numIntersectingBoxes, (int)pow(intRes*1.0,maxIterations), glParam);
#endif

	delete[] intData;
	glParam->intersection = false;
}

#ifdef GPUFIND
int ComputeCollision(NURBS* surface1, NURBS* surface2, GLParameters* glParam, float* &ibox1, float* &ibox2, float* &iData, int iterationNum, bool iterating, Float4* timeData)
{
	return 0;
}
#else
int ComputeCollision(NURBS* surface1, NURBS* surface2, GLParameters* glParam, float* &ibox1, float* &ibox2, float* &iData, int iterationNum, bool iterating, Float4* timeData)
{
	//Set up for evaluation
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	vector<Float4> referenceList;
	referenceList.clear();
	Float4 initialReference = Float4(0,0,0,0);
	referenceList.push_back(initialReference);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ZERO);

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;
	int maxLevel = GetExponent2(uNum);
	
	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2,tempTex);

	//Enable surface generation program
	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	int viewportx,viewporty;
	int numOverlaps=1;

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

	cgGLSetTextureParameter( glParam->bBoxMinATextureParam, surface1->evalParams->bBoxTextures[0]);
	cgGLSetTextureParameter( glParam->bBoxMaxATextureParam, surface1->evalParams->bBoxTextures[1]);
	cgGLSetTextureParameter( glParam->bBoxMinBTextureParam, surface2->evalParams->bBoxTextures[0]);
	cgGLSetTextureParameter( glParam->bBoxMaxBTextureParam, surface2->evalParams->bBoxTextures[1]);

	for (int level = 1; level <= maxLevel; level++)
	{
		int levelOriginX = int((pow(2.0,maxLevel-level)-1)*pow(2.0,level+1));
		int referenceListSize = ceil(sqrt(referenceList.size()*1.0));

		int intersectionArraySizeX = referenceListSize*4;
		int intersectionArraySizeY = referenceListSize*4;

		// Create a texture that is a power of two texture
		float* referenceListData = new float[referenceListSize*referenceListSize*4];
		for (int j = 0; j < referenceListSize; j++)
		{
			for (int i = 0; i < referenceListSize; i++)
			{
				if (j*referenceListSize + i < referenceList.size())
				{
					referenceListData[(j*referenceListSize + i)*4 + 0] = referenceList[j*referenceListSize + i][0];
					referenceListData[(j*referenceListSize + i)*4 + 1] = referenceList[j*referenceListSize + i][1];
					referenceListData[(j*referenceListSize + i)*4 + 2] = referenceList[j*referenceListSize + i][2];
					referenceListData[(j*referenceListSize + i)*4 + 3] = referenceList[j*referenceListSize + i][3];
				}
				else
				{
					referenceListData[(j*referenceListSize + i)*4 + 0] = -1;
					referenceListData[(j*referenceListSize + i)*4 + 1] = -1;
					referenceListData[(j*referenceListSize + i)*4 + 2] = -1;
					referenceListData[(j*referenceListSize + i)*4 + 3] = -1;
				}
			}
		}

		cgGLBindProgram(glParam->bBoxOverlapProgram);

		cgGLEnableTextureParameter(	glParam->bBoxMinATextureParam);
    	cgGLEnableTextureParameter(	glParam->bBoxMaxATextureParam);
		cgGLEnableTextureParameter(	glParam->bBoxMinBTextureParam);
		cgGLEnableTextureParameter(	glParam->bBoxMaxBTextureParam);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, intersectionArraySizeX, intersectionArraySizeY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, referenceListSize, referenceListSize, 0, GL_RGBA, GL_FLOAT, referenceListData);

		cgGLSetTextureParameter( glParam->bBoxReferenceListTextureParam, tempTex[1]);

		cgGLEnableTextureParameter(	glParam->bBoxReferenceListTextureParam);
		cgGLSetParameter1f(glParam->bBoxLevelOriginParam, levelOriginX);

		// Bind the src textures
		glActiveTextureARB(GL_TEXTURE4_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[1]);
		glActiveTextureARB(GL_TEXTURE3_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[0]);
		glActiveTextureARB(GL_TEXTURE2_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[1]);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[0]);
		glActiveTextureARB(GL_TEXTURE0_ARB);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);

		glViewport(0, 0, intersectionArraySizeX, intersectionArraySizeY);

		// Specify the dst texture
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glFlush();
	
		glBeginQueryARB(GL_SAMPLES_PASSED_ARB,glParam->occlusionQuery);
		DrawQuad(intersectionArraySizeX, intersectionArraySizeY);
		glFlush();
		glFinish();
		glEndQueryARB(GL_SAMPLES_PASSED_ARB);

		GLuint fragmentCount;
		glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &fragmentCount);
		numOverlaps = fragmentCount;

#ifdef DUMP
		GLfloat* retVal=new GLfloat[intersectionArraySizeX*intersectionArraySizeY];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,intersectionArraySizeX, intersectionArraySizeY, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal,intersectionArraySizeX, intersectionArraySizeY);
#endif

		cgGLDisableTextureParameter(glParam->bBoxMinATextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMaxATextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMinBTextureParam);
		cgGLDisableTextureParameter(glParam->bBoxMaxBTextureParam);
		cgGLDisableTextureParameter(glParam->bBoxReferenceListTextureParam);

		vector<int> positions;

		clock_t begintime;
		if (timeData != NULL)
			begintime = clock();

		FindRenderLocation(tempTex[0], &numOverlaps, intersectionArraySizeX, intersectionArraySizeY, &positions, glParam, false, timeData);

		if (timeData != NULL)
		{
			clock_t endtime = clock();
			timeData->value[3] += (endtime-begintime);
		}

		numOverlaps = positions.size();
		vector<Float4> newReferenceList;
		for (int overlap = 0; overlap < numOverlaps; overlap++)
		{
			int pos = positions[overlap];
			int xPos = pos%intersectionArraySizeX;
			int yPos = int(pos/intersectionArraySizeX);
			int referenceListLocationX = int(xPos/4.0);
			int referenceListLocationY = int(yPos/4.0);
			int boxLocationAx = ((xPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][0]))*2;
			int boxLocationAy = (int((xPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][1]))*2;
			int boxLocationBx = ((yPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][2]))*2;
			int boxLocationBy = (int((yPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][3]))*2;
			Float4 refCoords = Float4(boxLocationAx, boxLocationAy, boxLocationBx, boxLocationBy);
			newReferenceList.push_back(refCoords);
		}
		referenceList.clear();
		referenceList = newReferenceList;
		delete[] referenceListData;
	}                                    /// For each level loop ends
	
	int numIntersectionBoxes;
	if (iterating)
	{
		clock_t begintime;
		if (timeData != NULL)
			begintime = clock();

		vector<Float4> tempIntParamRange1;
		vector<Float4> tempIntParamRange2;
		vector<Float4> tempIntDataVector1;
		vector<Float3> tempIntDataVector2;
		numIntersectionBoxes = 0;

		for (int k = 0; k < referenceList.size(); k++)
		{
			int uIndex1 = (int) referenceList[k][0]/2.0;
			int vIndex1 = (int) referenceList[k][1]/2.0;
			int uIndex2 = (int) referenceList[k][2]/2.0;
			int vIndex2 = (int) referenceList[k][3]/2.0;

			float uMin1 = float(uIndex1*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
			float vMin1 = float(vIndex1*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);
			float uMax1 = float((uIndex1+1)*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
			float vMax1 = float((vIndex1+1)*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);

			Float4 intRange1 = Float4(uMin1,uMax1,vMin1,vMax1);
		
			float uMin2 = float(uIndex2*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
			float vMin2 = float(vIndex2*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);
			float uMax2 = float((uIndex2+1)*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
			float vMax2 = float((vIndex2+1)*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);

			Float4 intRange2 = Float4(uMin2,uMax2,vMin2,vMax2);
			bool isIntersecting = true;

			if (glParam->readBack && isIntersecting)
			{
				float intersectionData[7];
				isIntersecting = ExtractIntersectionData(surface1,surface2,uIndex1,vIndex1,uIndex2,vIndex2,intersectionData);
				if (isIntersecting)
				{
					Float4 tempIntData1 = Float4(intersectionData[0],intersectionData[1],intersectionData[2],intersectionData[3]);
					Float3 tempIntData2 = Float3(intersectionData[4],intersectionData[5],intersectionData[6]);
					tempIntDataVector1.push_back(tempIntData1);
					tempIntDataVector2.push_back(tempIntData2);
				}
			}
			if (isIntersecting)
			{
				tempIntParamRange1.push_back(intRange1);
				tempIntParamRange2.push_back(intRange2);
				numIntersectionBoxes++;
			}
		}

		ibox1=new float[numIntersectionBoxes*4];
		ibox2=new float[numIntersectionBoxes*4];
		if (glParam->readBack)
			iData = new float[numIntersectionBoxes*7];

		for (int k = 0; k < numIntersectionBoxes; k++)
		{
			ibox1[4*k+0] = tempIntParamRange1[k][0];
			ibox1[4*k+1] = tempIntParamRange1[k][1];
			ibox1[4*k+2] = tempIntParamRange1[k][2];
			ibox1[4*k+3] = tempIntParamRange1[k][3];

			ibox2[4*k+0] = tempIntParamRange2[k][0];
			ibox2[4*k+1] = tempIntParamRange2[k][1];
			ibox2[4*k+2] = tempIntParamRange2[k][2];
			ibox2[4*k+3] = tempIntParamRange2[k][3];
			if (glParam->readBack)
			{
				iData[7*k+0] = tempIntDataVector1[k][0];
				iData[7*k+1] = tempIntDataVector1[k][1];
				iData[7*k+2] = tempIntDataVector1[k][2];
				iData[7*k+3] = tempIntDataVector1[k][3];
				iData[7*k+4] = tempIntDataVector2[k][0];
				iData[7*k+5] = tempIntDataVector2[k][1];
				iData[7*k+6] = tempIntDataVector2[k][2];
			}
		}

		tempIntParamRange1.clear();
		tempIntParamRange2.clear();
		tempIntDataVector1.clear();
		tempIntDataVector2.clear();

		if (timeData != NULL)
		{
			clock_t endtime = clock();
			timeData->value[2] += (endtime-begintime);
		}
	}
	else
		numIntersectionBoxes = referenceList.size();

	//Unbind the frame buffer and disable the program
	cgGLDisableProfile(glParam->cgFragmentProfile);

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	glDeleteTextures(2, tempTex);

	// Restore the previous views
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
	return numIntersectionBoxes;
}
#endif

// Collision Detection
bool DetectCollision(NURBS* surface1, NURBS* surface2, int intRes, GLParameters* glParam, bool evaluate)
{
	glParam->collision = true;
	bool timing = false;

	if (evaluate)
	{
		surface1->evalParams = new EvaluationParams();
		surface2->evalParams = new EvaluationParams();
		int uNum = intRes;
		int vNum = intRes;
		surface1->evalParams->uNum	= uNum;
		surface1->evalParams->vNum	= vNum;
		surface2->evalParams->uNum	= uNum;
		surface2->evalParams->vNum	= vNum;
		surface1->initializedEvalParams = true;
		surface2->initializedEvalParams = true;
	}

	// uMin, uMax, vMin, vMax
	int NumberOfIntervals = 1;
	float *interval1 = new float[4];
	interval1[0] = 0.0;
	interval1[1] = 1.0;
	interval1[2] = 0.0;
	interval1[3] = 1.0;

	float *interval2 = new float[4];
	interval2[0] = 0.0;
	interval2[1] = 1.0;
	interval2[2] = 0.0;
	interval2[3] = 1.0;

	float* collisionData = new float;

	Float4 readTime = Float4(0,0,0,0);
	long unsigned int evaluationTime = 0;
	long unsigned int collisionTime = 0;
	clock_t beginTime, beginEvalTime;
		
	if (timing)
		beginTime = clock();

	int maxIterations = 1;
	for (int iterationNum = 0; iterationNum < maxIterations; iterationNum++)
	{
		float* newCollision1 = NULL;
		float* newCollision2 = NULL;
		float* newCollisionData = NULL;
		int newCollisionBoxSize = 0;
		for( int oldCollisionCount = 0; oldCollisionCount < NumberOfIntervals; oldCollisionCount++)
		{
			surface1->evalParams->uMin = interval1[oldCollisionCount*4+0];
			surface1->evalParams->uMax = interval1[oldCollisionCount*4+1];
			surface1->evalParams->vMin = interval1[oldCollisionCount*4+2];
			surface1->evalParams->vMax = interval1[oldCollisionCount*4+3];

			surface2->evalParams->uMin = interval2[oldCollisionCount*4+0];
			surface2->evalParams->uMax = interval2[oldCollisionCount*4+1];
			surface2->evalParams->vMin = interval2[oldCollisionCount*4+2];
			surface2->evalParams->vMax = interval2[oldCollisionCount*4+3];
	
			if (timing)
				beginEvalTime = clock();
			
			if (evaluate)
			{
				surface1->SurfaceEvaluation(glParam, false, false);
				surface1->SurfaceBoundingBoxEvaluation(glParam);
				surface2->SurfaceEvaluation(glParam, false, false);
				surface2->SurfaceBoundingBoxEvaluation(glParam);
			}

			float *collisionBox1 = NULL;
			float *collisionBox2 = NULL;
			float *collisionData = NULL;

			int numCollisions;
			if (timing)
			{
				evaluationTime += clock()-beginEvalTime;
				clock_t beginCollisionTime = clock();
				if (maxIterations == 1)
					numCollisions = ComputeCollision(surface1, surface2, glParam, collisionBox1, collisionBox2, collisionData, iterationNum, false, &readTime);
				else
					numCollisions = ComputeCollision(surface1, surface2, glParam, collisionBox1, collisionBox2, collisionData, iterationNum, true, &readTime);
				collisionTime += clock()-beginCollisionTime;
			}
			else
			{
				if (maxIterations == 1)
					numCollisions = ComputeCollision(surface1, surface2, glParam, collisionBox1, collisionBox2, collisionData, iterationNum, false);
				else
					numCollisions = ComputeCollision(surface1, surface2, glParam, collisionBox1, collisionBox2, collisionData, iterationNum, true);
			}

			newCollisionBoxSize += numCollisions;
			if (maxIterations > 1)
			{
				newCollision1 = (float*)realloc(newCollision1,4*newCollisionBoxSize*sizeof(float));
				newCollision2 = (float*)realloc(newCollision2,4*newCollisionBoxSize*sizeof(float));
				memcpy(newCollision1+4*(newCollisionBoxSize-numCollisions),collisionBox1,numCollisions*4*sizeof(float));
				memcpy(newCollision2+4*(newCollisionBoxSize-numCollisions),collisionBox2,numCollisions*4*sizeof(float));
			}
			delete[] collisionBox1;
			delete[] collisionBox2;
		}
		if (maxIterations > 1)
		{
			delete[] interval1;
			delete[] interval2;
			interval1 = newCollision1;
			interval2 = newCollision2;
			collisionData  = newCollisionData;
		}
		NumberOfIntervals = newCollisionBoxSize;
	}

	if (timing)
	{
		clock_t totalTime = clock()-beginTime;
		cout<<"Total Time        : "<<(0.001*totalTime)<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationTime)<<endl;
		cout<<"Collision Time    : "<<(0.001*collisionTime)<<endl;
		cout<<"Find Time         : "<<(0.001*readTime[3])<<endl;
	}

#ifdef DUMP

#endif
	bool colliding = false;
	if (NumberOfIntervals > 0)
		colliding = true;

	glParam->collision = false;
	if (evaluate)
	{
		delete	surface1->evalParams;
		delete	surface2->evalParams;
	}
	return colliding;
}


#ifndef GPUCOLLISION
void ObjectCollision(Object* object1, Object* object2, GLParameters* glParam)
{
	if (object1->voxelData == NULL || object2->voxelData == NULL)
		return;

	int numBoxes1 = object1->voxelData->numDivX * object1->voxelData->numDivY * object1->voxelData->numDivZ;
	int numBoxes2 = object2->voxelData->numDivX * object2->voxelData->numDivY * object2->voxelData->numDivZ;
	vector<Float2> intersectionList;

	for (int i = 0; i < numBoxes1; i++)
		object1->voxelData->bBox[i].intersecting = 0;
	for (int j = 0; j < numBoxes2; j++)
		object2->voxelData->bBox[j].intersecting = 0;

    // Matrix to transform OBB B into A reference to allow A to be treated as an AABB
	// Matrix transformBA = transformB * Matrix.Invert(transformA);
	float invTransformA[16];
	float transformBA[16];
	InvertMatrix(object1->transformationMatrix, invTransformA);
	MultiplyTransforms(object2->transformationMatrix, invTransformA, transformBA);

	for (int i = 0; i < numBoxes1; i++)
	{
		for (int j = 0; j < numBoxes2; j++)
		{
			if (object1->voxelData->bBox[i].solid == 1 && object2->voxelData->bBox[j].solid == 1)
			{
				bool overlap = OBBOverlap(&(object1->voxelData->bBox[i]), &(object2->voxelData->bBox[j]), transformBA);
				if (overlap)
				{
					object1->voxelData->bBox[i].intersecting = 1;
					object2->voxelData->bBox[j].intersecting = 1;
					for (int p = 0; p < object1->voxelData->bBox[i].surfaces.size(); p++)
					{
						for (int q = 0; q < object2->voxelData->bBox[j].surfaces.size(); q++)
						{
							Float2 intSurfaces = Float2(object1->voxelData->bBox[i].surfaces[p],object2->voxelData->bBox[j].surfaces[q]);
							intersectionList.push_back(intSurfaces);
						}
					}
				}
			}
		}
	}
}
#else
void ObjectCollision(Object* object1, Object* object2, GLParameters* glParam)
{
	if (object1->voxelData == NULL || object2->voxelData == NULL)
		return;

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

	int numBoxes1 = object1->voxelData->numDivX * object1->voxelData->numDivY * object1->voxelData->numDivZ;
	int numBoxes2 = object2->voxelData->numDivX * object2->voxelData->numDivY * object2->voxelData->numDivZ;
	vector<Float2> intersectionList;

	vector<int> invIndexX;
	for (int i = 0; i < numBoxes1; i++)
	{
		object1->voxelData->bBox[i].intersecting = 0;
		if (object1->voxelData->bBox[i].solid == 1)
			invIndexX.push_back(i);
	}
	int boxCountX = (int) invIndexX.size();

	vector<int> invIndexY;
	for (int i = 0; i < numBoxes2; i++)
	{
		object2->voxelData->bBox[i].intersecting = 0;
		if (object2->voxelData->bBox[i].solid == 1)
			invIndexY.push_back(i);
	} 
	int boxCountY = (int) invIndexY.size();

	if (boxCountX > 4096 || boxCountY > 4096)
	{
		cout << "Cannot perform GPU collision detection " << endl;
		return;
	}

	float* boxDataMin1 = new float[boxCountX*4];
	float* boxDataMax1 = new float[boxCountX*4];
	float* boxDataMin2 = new float[boxCountY*4];
	float* boxDataMax2 = new float[boxCountY*4];
	
	for (int i = 0; i < boxCountX; i++)
	{
		int boxIndex = invIndexX[i];
		boxDataMin1[i*4+0] = object1->voxelData->bBox[boxIndex].minPoint[0];
		boxDataMin1[i*4+1] = object1->voxelData->bBox[boxIndex].minPoint[1];
		boxDataMin1[i*4+2] = object1->voxelData->bBox[boxIndex].minPoint[2];
		boxDataMin1[i*4+3] = 1;

		boxDataMax1[i*4+0] = object1->voxelData->bBox[boxIndex].maxPoint[0];
		boxDataMax1[i*4+1] = object1->voxelData->bBox[boxIndex].maxPoint[1];
		boxDataMax1[i*4+2] = object1->voxelData->bBox[boxIndex].maxPoint[2];
		boxDataMax1[i*4+3] = 1;
	}

	for (int i = 0; i < boxCountY; i++)
	{
		int boxIndex = invIndexY[i];
		boxDataMin2[i*4+0] = object2->voxelData->bBox[boxIndex].minPoint[0];
		boxDataMin2[i*4+1] = object2->voxelData->bBox[boxIndex].minPoint[1];
		boxDataMin2[i*4+2] = object2->voxelData->bBox[boxIndex].minPoint[2];
		boxDataMin2[i*4+3] = 1;

		boxDataMax2[i*4+0] = object2->voxelData->bBox[boxIndex].maxPoint[0];
		boxDataMax2[i*4+1] = object2->voxelData->bBox[boxIndex].maxPoint[1];
		boxDataMax2[i*4+2] = object2->voxelData->bBox[boxIndex].maxPoint[2];
		boxDataMax2[i*4+3] = 1;
	}

	GLuint tempTex[5];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(5, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, boxCountX, 1, GL_RGBA, boxDataMin1);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, boxCountX, 1, GL_RGBA, boxDataMax1);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, boxCountY, 1, GL_RGBA, boxDataMin2);
	InitTexture(tempTex+3, RGBA_FLOAT_FORMAT, boxCountY, 1, GL_RGBA, boxDataMax2);
	InitTexture(tempTex+4, RGBA_FLOAT_FORMAT, boxCountX, boxCountY, GL_RGBA, NULL);

	//Enable surface generation program
	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	// Matrix to transform OBB B into A reference to allow A to be treated as an AABB
	// Matrix transformBA = transformB * Matrix.Invert(transformA);
	float invTransformA[16];
	float transformBA[16];
	float transformBAtest[16];
	InvertMatrix(object1->transformationMatrix, invTransformA);
	MultiplyTransforms(object2->transformationMatrix, invTransformA, transformBA);
	float transform[9];
	transform[0] = transformBA[0];
	transform[1] = transformBA[1];
	transform[2] = transformBA[2];
	transform[3] = transformBA[4];
	transform[4] = transformBA[5];
	transform[5] = transformBA[6];
	transform[6] = transformBA[8];
	transform[7] = transformBA[9];
	transform[8] = transformBA[10];

	float translate[3];
	translate[0] = transformBA[12];
	translate[1] = transformBA[13];
	translate[2] = transformBA[14];

	//Bind the program
	cgGLBindProgram(glParam->oBBoxOverlapProgram);

	cgGLSetTextureParameter(glParam->oBBoxMin1TextureParam, tempTex[0]);
	cgGLEnableTextureParameter(glParam->oBBoxMin1TextureParam);
	cgGLSetTextureParameter(glParam->oBBoxMax1TextureParam, tempTex[1]);
	cgGLEnableTextureParameter(glParam->oBBoxMax1TextureParam);
	cgGLSetTextureParameter(glParam->oBBoxMin2TextureParam, tempTex[2]);
	cgGLEnableTextureParameter(glParam->oBBoxMin2TextureParam);
	cgGLSetTextureParameter(glParam->oBBoxMax2TextureParam, tempTex[3]);
	cgGLEnableTextureParameter(glParam->oBBoxMax2TextureParam);
	
	cgGLSetMatrixParameterfc(glParam->oBBoxTransformParam, transform);
	cgGLSetParameter3fv(glParam->oBBoxTranslateParam, translate);

	glActiveTextureARB(GL_TEXTURE3_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[3]);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

	glViewport(0, 0, boxCountX, boxCountY);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[4], 0);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(boxCountX, boxCountY);
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[boxCountX*boxCountY];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, boxCountX, boxCountY, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, boxCountX, boxCountY);
#endif

	vector<int> positions;
	int numOverlaps;
	FindRenderLocation(tempTex[4], &numOverlaps, boxCountX, boxCountY, &positions, glParam, false);

	for (int k = 0; k < positions.size(); k++)
	{
		int pos = positions[k];
		int posX = pos%boxCountX;
		int posY = int(pos/boxCountX);
		int i = invIndexX[posX];
		int j = invIndexY[posY];
		object1->voxelData->bBox[i].intersecting = 1;
		object2->voxelData->bBox[j].intersecting = 1;
		for (int p = 0; p < object1->voxelData->bBox[i].surfaces.size(); p++)
		{
			for (int q = 0; q < object2->voxelData->bBox[j].surfaces.size(); q++)
			{
				Float2 intSurfaces = Float2(object1->voxelData->bBox[i].surfaces[p],object2->voxelData->bBox[j].surfaces[q]);
				intersectionList.push_back(intSurfaces);
			}
		}
	}

	//Disable cg texture params

	//Unbind the frame buffer and disable the program
	cgGLDisableProfile(glParam->cgFragmentProfile);

	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	glDeleteTextures(5, tempTex);

	// Restore the previous views
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glPopAttrib();
}
#endif
void GetClickParametricPoint(vector<Object*>* objects, float closestZVal, GLParameters* glParam)
{
	// Store the ModelViewMatrix
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX, glParam->modelViewMatrix);

	for (int objID = 0; objID < (*objects).size(); objID++)
	{
		for (int i = 0; i < (*objects)[objID]->faces.size(); i++)
		{
			if ((*objects)[objID]->faces[i]->isNURBS)
			{
				NURBS* surface = (*objects)[objID]->faces[i]->surface;
				// Save previous picked points
				if(glParam->drawingON)
				{
					surface->prevPickedPoints = surface->pickedPoints;
					surface->prevPickedParamPoints = surface->pickedParamPoints;
				}
				else
				{
					surface->prevPickedPoints.clear();
					surface->prevPickedParamPoints.clear();
				}

				Float4 evalInterval;
				evalInterval[0] = 0.0;
				evalInterval[1] = 1.0;
				evalInterval[2] = 0.0;
				evalInterval[3] = 1.0;
				vector<Float4> evaluationInterval;
				evaluationInterval.push_back(evalInterval);

				// Total number of iterations to reach the final uv coordinates
				int totalIterations = 2;

				for (int iterationNum = 0; iterationNum < totalIterations; iterationNum++)
				{
					vector<Float4> newEvaluationInterval;		// uMin, uMax, vMin, vMax
					int numIntervals = evaluationInterval.size();
					for( int oldNumIntervalCount = 0; oldNumIntervalCount < numIntervals; oldNumIntervalCount++)
					{
						if (surface->initializedEvalParams)
							delete surface->evalParams;
						surface->evalParams = new EvaluationParams();
						surface->initializedEvalParams = true;

						surface->evalParams->uMin = evaluationInterval[oldNumIntervalCount][0];
						surface->evalParams->uMax = evaluationInterval[oldNumIntervalCount][1];
						surface->evalParams->vMin = evaluationInterval[oldNumIntervalCount][2];
						surface->evalParams->vMax = evaluationInterval[oldNumIntervalCount][3];

						surface->pickedBoxes.clear();

						float iterationFactor = 2;
						int uNum = int(surface->uBaseNum*iterationFactor);
						int vNum = int(surface->vBaseNum*iterationFactor);
						surface->evalParams->uNum	= uNum;
						surface->evalParams->vNum	= vNum;

						glParam->inverseEvaluation =  true;
						surface->SurfaceEvaluation(glParam, false, false);
						surface->SurfaceBoundingBoxEvaluation(glParam);
						surface->GPURayInverseEvaluate(glParam->pickedPoint, surface->evalParams->bBoxTextures[0], surface->evalParams->bBoxTextures[1], glParam);
						glParam->inverseEvaluation =  false;

						for( int box=0; box < surface->pickedBoxes.size(); box++)
						{
							int vIndex = (int) surface->pickedBoxes[box]/uNum;
							int uIndex = (int) surface->pickedBoxes[box]%uNum;
							float uMin = float(uIndex*(surface->evalParams->uMax - surface->evalParams->uMin)/(1.0*uNum) + surface->evalParams->uMin);
							float vMin = float(vIndex*(surface->evalParams->vMax - surface->evalParams->vMin)/(1.0*vNum) + surface->evalParams->vMin);
							float uMax = float((uIndex+1)*(surface->evalParams->uMax - surface->evalParams->uMin)/(1.0*uNum) + surface->evalParams->uMin);
							float vMax = float((vIndex+1)*(surface->evalParams->vMax - surface->evalParams->vMin)/(1.0*vNum) + surface->evalParams->vMin);

							Float4 evalInterval;
							evalInterval[0] = uMin;
							evalInterval[1] = uMax;
							evalInterval[2] = vMin;
							evalInterval[3] = vMax;
							newEvaluationInterval.push_back(evalInterval);
						}
					}
					evaluationInterval = newEvaluationInterval;
				}

				surface->pickedPoints.clear();
				surface->pickedParamPoints.clear();
				for(int j = 0; j < evaluationInterval.size();j++)
				{
					float uVal = (evaluationInterval[j][0]+evaluationInterval[j][1])/2.0;
					float vVal = (evaluationInterval[j][2]+evaluationInterval[j][3])/2.0;
					Float4 surfacePoint = surface->Evaluate4(uVal,vVal);
					Float2 parametricPoint = Float2(uVal,vVal);
					surface->pickedPoints.push_back(surfacePoint);
					surface->pickedParamPoints.push_back(parametricPoint);
				}
			}
		}
	}
	int pickedFaceNum	= -1;
	int pickedObjectNum		= -1;
	Float4 pickedSurfacePoint;
	Float2 pickedParamPoint;
	for (int objID = 0; objID < (*objects).size(); objID++)
	{
		for (int i = 0; i < (*objects)[objID]->faces.size(); i++)
		{
			if ((*objects)[objID]->faces[i]->isNURBS)
			{
				NURBS* surface = (*objects)[objID]->faces[i]->surface;
				for(int j = 0; j < surface->pickedPoints.size();j++)
				{
					Float4 surfacePoint = surface->pickedPoints[j];
					Float2 parametricPoint = surface->pickedParamPoints[j];
					Float4 transformedPoint = TransformPoint(surfacePoint, glParam->modelViewMatrix);
					if (transformedPoint[2] > closestZVal)
					{
						pickedFaceNum = i;
						pickedObjectNum	 = objID;
						pickedSurfacePoint = surfacePoint;
						pickedParamPoint = parametricPoint;
						closestZVal = transformedPoint[2];
					}
				}
			}
		}
	}

	for (int objID = 0; objID < (*objects).size(); objID++)
	{
		for (int i = 0; i < (*objects)[objID]->faces.size(); i++)
		{
			if ((*objects)[objID]->faces[i]->isNURBS)
			{
				NURBS* surface = (*objects)[objID]->faces[i]->surface;
				surface->pickedParamPoints.clear();
				surface->pickedPoints.clear();
				surface->pickedPoints = surface->prevPickedPoints;
				surface->pickedParamPoints = surface->prevPickedParamPoints;
			}
		}
	}

	if (pickedFaceNum != -1 && pickedObjectNum != -1)
	{
		NURBS* surface = (*objects)[pickedObjectNum]->faces[pickedFaceNum]->surface;
		float dist = 1;
		if (surface->pickedParamPoints.size() > 1)
		{
			Float2 lastPoint	= surface->pickedParamPoints.back();
			dist				= sqrt((pickedParamPoint[0]-lastPoint[0])*(pickedParamPoint[0]-lastPoint[0]) + (pickedParamPoint[1]-lastPoint[1])*(pickedParamPoint[1]-lastPoint[1]));
		}
		if (dist > 0.01)
		{
			surface->pickedParamPoints.push_back(pickedParamPoint);
			surface->pickedPoints.push_back(pickedSurfacePoint);
			surface->GenerateCurveOverlayTexture(glParam);
		}
		glParam->pickedFaceNum = pickedFaceNum;
		glParam->pickedObjectNum = pickedObjectNum;
	}

}


void GetClickParametricPoint(vector<NURBS*>* nurbsSurfaces, float closestZVal, GLParameters* glParam)
{
	// Store the ModelViewMatrix
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX, glParam->modelViewMatrix);

	int startSurfNum = 0;
	int endSurfNum = nurbsSurfaces->size();
	if(glParam->drawingON && glParam->pickedSurfNum != -1)
	{
		startSurfNum = glParam->pickedSurfNum;
		endSurfNum = startSurfNum+1;
	}
	for (int i=startSurfNum; i<endSurfNum; i++)
	{
		NURBS* surface = nurbsSurfaces->at(i);
		// Save previous picked points
		if(glParam->drawingON)
		{
			surface->prevPickedPoints = surface->pickedPoints;
			surface->prevPickedParamPoints = surface->pickedParamPoints;
		}
		else
		{
			surface->prevPickedPoints.clear();
			surface->prevPickedParamPoints.clear();
		}

		Float4 evalInterval;
		evalInterval[0] = 0.0;
		evalInterval[1] = 1.0;
		evalInterval[2] = 0.0;
		evalInterval[3] = 1.0;
		vector<Float4> evaluationInterval;
		evaluationInterval.push_back(evalInterval);

		// Total number of iterations to reach the final uv coordinates
		int totalIterations = 2;

		for (int iterationNum = 0; iterationNum < totalIterations; iterationNum++)
		{
			vector<Float4> newEvaluationInterval;		// uMin, uMax, vMin, vMax
			int numIntervals = evaluationInterval.size();
			for( int oldNumIntervalCount = 0; oldNumIntervalCount < numIntervals; oldNumIntervalCount++)
			{
				if (surface->initializedEvalParams)
					delete surface->evalParams;
				surface->evalParams = new EvaluationParams();
				surface->initializedEvalParams = true;

				surface->evalParams->uMin = evaluationInterval[oldNumIntervalCount][0];
				surface->evalParams->uMax = evaluationInterval[oldNumIntervalCount][1];
				surface->evalParams->vMin = evaluationInterval[oldNumIntervalCount][2];
				surface->evalParams->vMax = evaluationInterval[oldNumIntervalCount][3];

				surface->pickedBoxes.clear();

				float iterationFactor = 2;
				int uNum = int(surface->uBaseNum*iterationFactor);
				int vNum = int(surface->vBaseNum*iterationFactor);
				surface->evalParams->uNum	= uNum;
				surface->evalParams->vNum	= vNum;

				glParam->inverseEvaluation =  true;
				surface->SurfaceEvaluation(glParam, false, false);
				surface->SurfaceBoundingBoxEvaluation(glParam);
				surface->GPURayInverseEvaluate(glParam->pickedPoint, surface->evalParams->bBoxTextures[0], surface->evalParams->bBoxTextures[1], glParam);
				glParam->inverseEvaluation =  false;

				for( int box=0; box < surface->pickedBoxes.size(); box++)
				{
					int vIndex = (int) surface->pickedBoxes[box]/uNum;
					int uIndex = (int) surface->pickedBoxes[box]%uNum;
					float uMin = float(uIndex*(surface->evalParams->uMax - surface->evalParams->uMin)/(1.0*uNum) + surface->evalParams->uMin);
					float vMin = float(vIndex*(surface->evalParams->vMax - surface->evalParams->vMin)/(1.0*vNum) + surface->evalParams->vMin);
					float uMax = float((uIndex+1)*(surface->evalParams->uMax - surface->evalParams->uMin)/(1.0*uNum) + surface->evalParams->uMin);
					float vMax = float((vIndex+1)*(surface->evalParams->vMax - surface->evalParams->vMin)/(1.0*vNum) + surface->evalParams->vMin);

					Float4 evalInterval;
					evalInterval[0] = uMin;
					evalInterval[1] = uMax;
					evalInterval[2] = vMin;
					evalInterval[3] = vMax;
					newEvaluationInterval.push_back(evalInterval);
				}
			}
			evaluationInterval = newEvaluationInterval;
		}

		surface->pickedPoints.clear();
		surface->pickedParamPoints.clear();
		for(int j = 0; j < evaluationInterval.size();j++)
		{
			float uVal = (evaluationInterval[j][0]+evaluationInterval[j][1])/2.0;
			float vVal = (evaluationInterval[j][2]+evaluationInterval[j][3])/2.0;
			Float4 surfacePoint = surface->Evaluate4(uVal,vVal);
			Float2 parametricPoint = Float2(uVal,vVal);
			surface->pickedPoints.push_back(surfacePoint);
			surface->pickedParamPoints.push_back(parametricPoint);
		}
	}

	int pickedSurfaceNum = -1;
	Float4 pickedSurfacePoint;
	Float2 pickedParamPoint;
	for (int i=startSurfNum; i<endSurfNum; i++)
	{
		NURBS* surface = nurbsSurfaces->at(i);
		for(int j = 0; j < surface->pickedPoints.size();j++)
		{
			Float4 surfacePoint = surface->pickedPoints[j];
			Float2 parametricPoint = surface->pickedParamPoints[j];
			Float4 transformedPoint = TransformPoint(surfacePoint, glParam->modelViewMatrix);
			if (transformedPoint[2] > closestZVal)
			{
				pickedSurfaceNum = i;
				pickedSurfacePoint = surfacePoint;
				pickedParamPoint = parametricPoint;
				closestZVal = transformedPoint[2];
			}
		}
	}

	for (int i=startSurfNum; i<endSurfNum; i++)
	{
		NURBS* surface = nurbsSurfaces->at(i);
		surface->pickedParamPoints.clear();
		surface->pickedPoints.clear();
		surface->pickedPoints = surface->prevPickedPoints;
		surface->pickedParamPoints = surface->prevPickedParamPoints;
	}

	if (pickedSurfaceNum!=-1)
	{
		NURBS* surface = nurbsSurfaces->at(pickedSurfaceNum);
		float dist = 1;
		if (surface->pickedParamPoints.size() > 1)
		{
			Float2 lastPoint	= surface->pickedParamPoints.back();
			dist				= sqrt((pickedParamPoint[0]-lastPoint[0])*(pickedParamPoint[0]-lastPoint[0]) + (pickedParamPoint[1]-lastPoint[1])*(pickedParamPoint[1]-lastPoint[1]));
		}
		if (dist > 0.01)
		{
			surface->pickedParamPoints.push_back(pickedParamPoint);
			surface->pickedPoints.push_back(pickedSurfacePoint);
			surface->GenerateCurveOverlayTexture(glParam);
		}
		glParam->pickedSurfNum = pickedSurfaceNum;
	}

}

// Closest Point Computations
float GPUSurfaceClosestPoint(NURBS* surface1, NURBS* surface2, float* point1, float* point2, GLParameters* glParam)
{
	bool operationFailed = false;
	bool timing = true;
	clock_t beginTime, beginFindTime, beginDistTime, distTime, findTime1, findTime2, beginFinalCalcTime, finalCalcTime, totalTime;
	if (timing)
	{
		beginTime = clock();
		distTime  = 0;
		findTime1 = 0;
		findTime2 = 0;
	}

	// Set up GPU computations
	StartGPUComputation(glParam);

	//Set up for evaluation
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glBlendFunc(GL_ONE,GL_ZERO);

	vector<Float4> referenceList;
	referenceList.clear();
	Float4 initialReference = Float4(0,0,0,0);
	referenceList.push_back(initialReference);

	int uNum = surface1->evalParams->uNum;
	int vNum = surface1->evalParams->vNum;
	int maxLevel = GetExponent2(uNum);
	
	GLuint tempTex[3];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(3,tempTex);

	//Enable surface generation program
	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	int numOverlaps = 1;

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
    glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);


	for (int level = 1; level <= maxLevel && !operationFailed; level++)
	{
		if (timing)
			beginDistTime = clock();

		int levelOriginX = int((pow(2.0,maxLevel-level)-1)*pow(2.0,level+1));
		int referenceListSize = ceil(sqrt(referenceList.size()*1.0));

		int distanceArraySizeX = referenceListSize*4;
		int distanceArraySizeY = referenceListSize*4;

		if (distanceArraySizeX < 2048)
		{
			// Create a texture that is a power of two texture
			float* referenceListData = new float[referenceListSize*referenceListSize*4];
			for (int j = 0; j < referenceListSize; j++)
			{
				for (int i = 0; i < referenceListSize; i++)
				{
					if (j*referenceListSize + i < referenceList.size())
					{
						referenceListData[(j*referenceListSize + i)*4 + 0] = referenceList[j*referenceListSize + i][0];
						referenceListData[(j*referenceListSize + i)*4 + 1] = referenceList[j*referenceListSize + i][1];
						referenceListData[(j*referenceListSize + i)*4 + 2] = referenceList[j*referenceListSize + i][2];
						referenceListData[(j*referenceListSize + i)*4 + 3] = referenceList[j*referenceListSize + i][3];
					}
					else
					{
						referenceListData[(j*referenceListSize + i)*4 + 0] = -1;
						referenceListData[(j*referenceListSize + i)*4 + 1] = -1;
						referenceListData[(j*referenceListSize + i)*4 + 2] = -1;
						referenceListData[(j*referenceListSize + i)*4 + 3] = -1;
					}
				}
			}

			InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, distanceArraySizeX, distanceArraySizeY, GL_RGBA);
			InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, referenceListSize, referenceListSize, GL_RGBA, referenceListData);

			cgGLBindProgram(glParam->bBoxDistProgram);

			cgGLEnableTextureParameter(glParam->bBoxDistMinATextureParam);
    		cgGLEnableTextureParameter(glParam->bBoxDistMaxATextureParam);
			cgGLEnableTextureParameter(glParam->bBoxDistMinBTextureParam);
			cgGLEnableTextureParameter(glParam->bBoxDistMaxBTextureParam);

			cgGLSetTextureParameter(glParam->bBoxDistMinATextureParam, surface1->evalParams->bBoxTextures[0]);
			cgGLSetTextureParameter(glParam->bBoxDistMaxATextureParam, surface1->evalParams->bBoxTextures[1]);
			cgGLSetTextureParameter(glParam->bBoxDistMinBTextureParam, surface2->evalParams->bBoxTextures[0]);
			cgGLSetTextureParameter(glParam->bBoxDistMaxBTextureParam, surface2->evalParams->bBoxTextures[1]);

			cgGLSetTextureParameter(glParam->bBoxDistReferenceListTextureParam, tempTex[1]);

			cgGLEnableTextureParameter(glParam->bBoxDistReferenceListTextureParam);
			cgGLSetParameter1f(glParam->bBoxDistLevelOriginParam, levelOriginX);

			// Bind the src textures
			glActiveTextureARB(GL_TEXTURE4_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[1]);
			glActiveTextureARB(GL_TEXTURE3_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface2->evalParams->bBoxTextures[0]);
			glActiveTextureARB(GL_TEXTURE2_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[1]);
			glActiveTextureARB(GL_TEXTURE1_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surface1->evalParams->bBoxTextures[0]);
			glActiveTextureARB(GL_TEXTURE0_ARB);

			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);

			glViewport(0, 0, distanceArraySizeX, distanceArraySizeY);

			// Specify the dst texture
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

			glClearColor(1e5,1e5,1e5,1e5);
			glClear(GL_COLOR_BUFFER_BIT);
			glFlush();
		
			DrawQuad(distanceArraySizeX, distanceArraySizeY);
			glFlush();
			glFinish();

#ifdef DUMP
			GLfloat* retVal=new GLfloat[distanceArraySizeX*distanceArraySizeY*4];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0, 0, distanceArraySizeX, distanceArraySizeY, GL_RGBA, GL_FLOAT, retVal);
			WriteToFile(retVal, distanceArraySizeX, distanceArraySizeY, 0);
	//		WriteToFile(retVal, distanceArraySizeX, distanceArraySizeY, 1);
#endif

			cgGLDisableTextureParameter(glParam->bBoxDistMinATextureParam);
			cgGLDisableTextureParameter(glParam->bBoxDistMaxATextureParam);
			cgGLDisableTextureParameter(glParam->bBoxDistMinBTextureParam);
			cgGLDisableTextureParameter(glParam->bBoxDistMaxBTextureParam);
			cgGLDisableTextureParameter(glParam->bBoxDistReferenceListTextureParam);

			// Find the maximum extent of distance of bounding box with minimum distance
			if (timing)
			{
				beginFindTime = clock();
				distTime += (beginFindTime - beginDistTime);
			}

			float minBBoxMaxExtent = FindMinBBoxMaxExtent(tempTex[0], distanceArraySizeX, distanceArraySizeY, glParam);

			if (timing)
				findTime1 += (clock() - beginFindTime);
			if (timing)
				beginFindTime = clock();

			{
				cgGLBindProgram(glParam->bBoxDistFilterProgram);

	//			InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, distanceArraySizeX, distanceArraySizeY, GL_RGBA);
				glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
				glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, distanceArraySizeX, distanceArraySizeY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

				glActiveTextureARB(GL_TEXTURE0_ARB);
				glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

				cgGLSetTextureParameter(glParam->bBoxDistFilterTextureParam, tempTex[0]);
				cgGLEnableTextureParameter(glParam->bBoxDistFilterTextureParam);

				cgGLSetParameter1f(glParam->bBoxDistFilterMaxValParam, minBBoxMaxExtent);

				glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[2], 0);
				glViewport(0, 0, distanceArraySizeX, distanceArraySizeY);

				glClearColor(0,0,0,0);
				glClear(GL_COLOR_BUFFER_BIT);

				DrawQuad(distanceArraySizeX, distanceArraySizeY);
				glFlush();
				glFinish();

#ifdef DUMP
				GLfloat* retVal=new GLfloat[distanceArraySizeX*distanceArraySizeY];
				glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
				glReadPixels(0, 0, distanceArraySizeX, distanceArraySizeY, GL_RED, GL_FLOAT, retVal);
				WriteToFile(retVal, distanceArraySizeX, distanceArraySizeY);
#endif
				cgGLDisableTextureParameter(glParam->bBoxDistFilterTextureParam);
			}

			vector<int> positions;
			FindRenderLocation(tempTex[2], &numOverlaps, distanceArraySizeX, distanceArraySizeY, &positions, glParam);

			if (timing)
				findTime2 += (clock() - beginFindTime);

			numOverlaps = positions.size();
			vector<Float4> newReferenceList;
			for (int overlap = 0; overlap < numOverlaps; overlap++)
			{
				int pos = positions[overlap];
				int xPos = pos%distanceArraySizeX;
				int yPos = int(pos/distanceArraySizeX);
				int referenceListLocationX = int(xPos/4.0);
				int referenceListLocationY = int(yPos/4.0);
				int boxLocationAx = ((xPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][0]))*2;
				int boxLocationAy = (int((xPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][1]))*2;
				int boxLocationBx = ((yPos%4)%2		+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][2]))*2;
				int boxLocationBy = (int((yPos%4)/2)+ int(referenceList[referenceListLocationY*referenceListSize + referenceListLocationX][3]))*2;
				Float4 refCoords = Float4(boxLocationAx, boxLocationAy, boxLocationBx, boxLocationBy);
				newReferenceList.push_back(refCoords);
			}
			referenceList.clear();
			referenceList = newReferenceList;
			delete[] referenceListData;
		}// For if >4096
		else
			operationFailed = true;
	}
	// For each level loop ends

	float closestDistance = -1;
	if (!operationFailed)
	{
		if (timing)
			beginFinalCalcTime = clock();

		Float3 closestSurf1Point,closestSurf2Point;
		for (int k = 0; k < referenceList.size(); k++)
		{
			Float3 surf1Point;
			Float3 surf2Point;
			int uIndex1 = (int) referenceList[k][0]/2.0;
			int vIndex1 = (int) referenceList[k][1]/2.0;
			int uIndex2 = (int) referenceList[k][2]/2.0;
			int vIndex2 = (int) referenceList[k][3]/2.0;

			float tempMinDist = -1;
			if (glParam->readBack)
				tempMinDist = ClosestPointData(surface1, surface2, uIndex1, vIndex1, uIndex2, vIndex2, &surf1Point, &surf2Point);
			else
			{
				float uMin1 = float(uIndex1*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
				float vMin1 = float(vIndex1*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);
				float uMax1 = float((uIndex1+1)*(surface1->evalParams->uMax - surface1->evalParams->uMin)/(1.0*uNum) + surface1->evalParams->uMin);
				float vMax1 = float((vIndex1+1)*(surface1->evalParams->vMax - surface1->evalParams->vMin)/(1.0*vNum) + surface1->evalParams->vMin);
			
				float uMin2 = float(uIndex2*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
				float vMin2 = float(vIndex2*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);
				float uMax2 = float((uIndex2+1)*(surface2->evalParams->uMax - surface2->evalParams->uMin)/(1.0*uNum) + surface2->evalParams->uMin);
				float vMax2 = float((vIndex2+1)*(surface2->evalParams->vMax - surface2->evalParams->vMin)/(1.0*vNum) + surface2->evalParams->vMin);

				Float3 tempSurf1Point = surface1->Evaluate((uMin1+uMax1)*.5,(vMin1+vMax1)*.5);
				Float3 tempSurf2Point = surface2->Evaluate((uMin2+uMax2)*.5,(vMin2+vMax2)*.5);

				if (! surface1->identityTransformation)
					surf1Point = TransformPoint(tempSurf1Point, surface1->transformationMatrix);
				else
					surf1Point = tempSurf1Point;

				tempMinDist = Distance(surf1Point, surf2Point);

				if (! surface2->identityTransformation)
					surf2Point = TransformPoint(tempSurf2Point, surface2->transformationMatrix);
				else
					surf2Point = tempSurf2Point;
				tempMinDist = Distance(surf1Point, surf2Point);
			}
			if ((tempMinDist >= 0 && tempMinDist <= closestDistance) || closestDistance < 0)
			{
				closestSurf1Point = surf1Point;
				closestSurf2Point = surf2Point;
				closestDistance = tempMinDist;
			}
		}

		if (closestDistance >= 0)
		{
			point1[0] = closestSurf1Point[0];
			point1[1] = closestSurf1Point[1];
			point1[2] = closestSurf1Point[2];
			point2[0] = closestSurf2Point[0];
			point2[1] = closestSurf2Point[1];
			point2[2] = closestSurf2Point[2];
		}
	}

	glDeleteTextures(3, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	if (timing)
	{
		finalCalcTime  = clock() - beginFinalCalcTime;
		totalTime = clock() - beginTime;
		cout << "Traversal Total Time : " << totalTime*0.001 << endl;
		cout << "Distance Time        : " << distTime*0.001 << endl;
		cout << "Find Min BBox Time   : " << findTime1*0.001 << endl;
		cout << "BBox Range Time      : " << findTime2*0.001 << endl;
		cout << "Trianguled Time      : " << finalCalcTime*0.001 << endl << endl;
	}
	if (!operationFailed)
		return closestDistance;
	else
		return -1;
}

float GetSurfaceClosestPoint(NURBS* surface1, NURBS* surface2, float* point1, float* point2, int intRes, GLParameters* glParam, bool forcedEvaluate)
{
	glParam->closestPoint = true;
	int uNum = intRes;
	int vNum = intRes;
//	int uNum = 32;
//	int vNum = 32;
	bool timing = true;
	long unsigned int evaluationTime = 0;
	long unsigned int bBoxTime = 0;
	long unsigned int closestPointTime = 0;
	clock_t beginTime = 0;
	clock_t beginEvalTime = 0;
	clock_t beginBBoxTime = 0;
	clock_t beginClosestPointTime = 0;

	if (timing)
		beginTime = clock();

	bool evaluate = true;
	if(surface1->initializedEvalParams && surface2->initializedEvalParams && !forcedEvaluate)
	{
		if (surface1->evalParams->uMin == 0.0  &&
			surface1->evalParams->uMax == 1.0  &&
			surface1->evalParams->vMin == 0.0  &&
			surface1->evalParams->vMax == 1.0  &&
			surface1->evalParams->uNum == uNum &&
			surface1->evalParams->vNum == vNum &&
			surface2->evalParams->uMin == 0.0  &&
			surface2->evalParams->uMax == 1.0  &&
			surface2->evalParams->vMin == 0.0  &&
			surface2->evalParams->vMax == 1.0  &&
			surface2->evalParams->uNum == uNum &&
			surface2->evalParams->vNum == vNum)
				evaluate = false;
	}

	if (!evaluate && forcedEvaluate)
		evaluate = true;

	if (evaluate)
	{
		if (surface1->initializedEvalParams)
			delete surface1->evalParams;
		surface1->evalParams = new EvaluationParams();
		surface1->initializedEvalParams = true;

		if (surface2->initializedEvalParams)
			delete surface2->evalParams;
		surface2->evalParams = new EvaluationParams();
		surface2->initializedEvalParams = true;

		surface1->evalParams->uNum	= uNum;
		surface1->evalParams->vNum	= vNum;
		surface2->evalParams->uNum	= uNum;
		surface2->evalParams->vNum	= vNum;

		surface1->evalParams->uMin = 0.0;
		surface1->evalParams->uMax = 1.0;
		surface1->evalParams->vMin = 0.0;
		surface1->evalParams->vMax = 1.0;

		surface2->evalParams->uMin = 0.0;
		surface2->evalParams->uMax = 1.0;
		surface2->evalParams->vMin = 0.0;
		surface2->evalParams->vMax = 1.0;
	}

	if (timing)
		beginEvalTime = clock();
	if (evaluate)
	{
		if (glParam->readBack)
		{
			surface1->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
			surface1->evalParams->minPoints			= new float[(uNum)*(vNum)*4];
			surface1->evalParams->maxPoints			= new float[(uNum)*(vNum)*4];
			surface2->evalParams->evaluatedPoints	= new float[(uNum+1)*(vNum+1)*4];
			surface2->evalParams->minPoints			= new float[(uNum)*(vNum)*4];
			surface2->evalParams->maxPoints			= new float[(uNum)*(vNum)*4];
		}
		surface1->SurfaceEvaluation(glParam, false, false);
		surface2->SurfaceEvaluation(glParam, false, false);
		if (surface1->trimmed)
		{
			surface1->evalParams->trimData = new float[(uNum+1)*(vNum+1)];
			surface1->GenerateTrimData(uNum, vNum, glParam);
		}
		if (surface2->trimmed)
		{
			surface2->evalParams->trimData = new float[(uNum+1)*(vNum+1)];
			surface2->GenerateTrimData(uNum, vNum, glParam);
		}
	}
	if (timing)
	{
		beginBBoxTime = clock();
		evaluationTime += beginBBoxTime - beginEvalTime;
	}
	if (evaluate || !surface1->identityTransformation)
	{
		if (surface1->trimmed)
			surface1->SurfaceBoundingBoxTrimmedEvaluation(glParam);
		else
			surface1->SurfaceBoundingBoxEvaluation(glParam);
	}
	if (evaluate || !surface2->identityTransformation)
	{
		if (surface2->trimmed)
			surface2->SurfaceBoundingBoxTrimmedEvaluation(glParam);
		else
			surface2->SurfaceBoundingBoxEvaluation(glParam);
	}
	if (timing)
	{
		beginClosestPointTime = clock();
		bBoxTime += beginClosestPointTime - beginBBoxTime;
	}

	float minDist = GPUSurfaceClosestPoint(surface1, surface2, point1, point2, glParam);

	if (timing)
		closestPointTime += clock() - beginClosestPointTime;

	glParam->closestPoint = false;
	if (timing)
	{
		clock_t endTime = clock();
		cout<<"Total Time        : "<<(0.001*(endTime - beginTime))<<endl;
		cout<<"Evaluation Time   : "<<(0.001*evaluationTime)<<endl;
		cout<<"BBox Time         : "<<(0.001*bBoxTime)<<endl;
		cout<<"Traversal Time    : "<<(0.001*closestPointTime)<<endl<<endl;
	}
	return minDist;
}

bool CheckDuplicate(Float2 surf, vector<Float2>* iList)
{
	bool duplicate = false;
	int surf1 = int(surf[0]);
	int surf2 = int(surf[1]);
	for (int i = 0; i < iList->size(); i++)
	{
		Float2 val = iList->at(i);
		if (surf1 ==  int(val[0]) && surf2 ==  int(val[1]))
			duplicate = true;
	}
	return duplicate;
}

vector<Float2> GPUObjectClosestPoint(Object* object1, Object* object2, GLParameters* glParam)
{
	vector<Float2> intersectionList;
	if (object1->voxelData == NULL || object2->voxelData == NULL)
		return intersectionList;

	int numBoxes1 = object1->voxelData->numDivX * object1->voxelData->numDivY * object1->voxelData->numDivZ;
	int numBoxes2 = object2->voxelData->numDivX * object2->voxelData->numDivY * object2->voxelData->numDivZ;

	vector<int> invIndexX;
	for (int i = 0; i < numBoxes1; i++)
	{
		object1->voxelData->bBox[i].intersecting = 0;
		if (object1->voxelData->bBox[i].solid == 1)
			invIndexX.push_back(i);
	}
	int boxCountX = (int) invIndexX.size();

	vector<int> invIndexY;
	for (int i = 0; i < numBoxes2; i++)
	{
		object2->voxelData->bBox[i].intersecting = 0;
		if (object2->voxelData->bBox[i].solid == 1)
			invIndexY.push_back(i);
	} 
	int boxCountY = (int) invIndexY.size();

	if (boxCountX > 4096 || boxCountY > 4096)
	{
		cout << "Cannot perform GPU collision detection " << endl;
		return intersectionList;
	}

	// Set up GPU computations
	StartGPUComputation(glParam);

	float* boxDataMin1 = new float[boxCountX*4];
	float* boxDataMax1 = new float[boxCountX*4];
	float* boxDataMin2 = new float[boxCountY*4];
	float* boxDataMax2 = new float[boxCountY*4];

	// Transform bounding boxes
	Float3 translate1 = Float3(object1->transformationMatrix[12], object1->transformationMatrix[13], object1->transformationMatrix[14]);
		
	for (int i = 0; i < boxCountX; i++)
	{
		int boxIndex = invIndexX[i];
		boxDataMin1[i*4+0] = object1->voxelData->bBox[boxIndex].minPoint[0] + translate1[0];
		boxDataMin1[i*4+1] = object1->voxelData->bBox[boxIndex].minPoint[1] + translate1[1];
		boxDataMin1[i*4+2] = object1->voxelData->bBox[boxIndex].minPoint[2] + translate1[2];
		boxDataMin1[i*4+3] = 1;

		boxDataMax1[i*4+0] = object1->voxelData->bBox[boxIndex].maxPoint[0] + translate1[0];
		boxDataMax1[i*4+1] = object1->voxelData->bBox[boxIndex].maxPoint[1] + translate1[1];
		boxDataMax1[i*4+2] = object1->voxelData->bBox[boxIndex].maxPoint[2] + translate1[2];
		boxDataMax1[i*4+3] = 1;
	}

	Float3 translate2 = Float3(object2->transformationMatrix[12], object2->transformationMatrix[13], object2->transformationMatrix[14]);

	for (int i = 0; i < boxCountY; i++)
	{
		int boxIndex = invIndexY[i];
		boxDataMin2[i*4+0] = object2->voxelData->bBox[boxIndex].minPoint[0] + translate2[0];
		boxDataMin2[i*4+1] = object2->voxelData->bBox[boxIndex].minPoint[1] + translate2[1];
		boxDataMin2[i*4+2] = object2->voxelData->bBox[boxIndex].minPoint[2] + translate2[2];
		boxDataMin2[i*4+3] = 1;

		boxDataMax2[i*4+0] = object2->voxelData->bBox[boxIndex].maxPoint[0] + translate2[0];
		boxDataMax2[i*4+1] = object2->voxelData->bBox[boxIndex].maxPoint[1] + translate2[1];
		boxDataMax2[i*4+2] = object2->voxelData->bBox[boxIndex].maxPoint[2] + translate2[2];
		boxDataMax2[i*4+3] = 1;
	}

	GLuint tempTex[5];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(6, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, boxCountX, 1, GL_RGBA, boxDataMin1);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, boxCountX, 1, GL_RGBA, boxDataMax1);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, boxCountY, 1, GL_RGBA, boxDataMin2);
	InitTexture(tempTex+3, RGBA_FLOAT_FORMAT, boxCountY, 1, GL_RGBA, boxDataMax2);
	InitTexture(tempTex+4, RGBA_FLOAT_FORMAT, boxCountX, boxCountY, GL_RGBA, NULL);

	//Bind the program
	cgGLBindProgram(glParam->objectBBoxDistanceProgram);

	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMinATextureParam);
	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMaxATextureParam);
	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMinBTextureParam);
	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMaxBTextureParam);

	cgGLSetTextureParameter(glParam->objectBBoxDistanceMinATextureParam, tempTex[0]);
	cgGLSetTextureParameter(glParam->objectBBoxDistanceMaxATextureParam, tempTex[1]);
	cgGLSetTextureParameter(glParam->objectBBoxDistanceMinBTextureParam, tempTex[2]);
	cgGLSetTextureParameter(glParam->objectBBoxDistanceMaxBTextureParam, tempTex[3]);
	
	glActiveTextureARB(GL_TEXTURE3_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[3]);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

	glViewport(0, 0, boxCountX, boxCountY);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[4], 0);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(boxCountX, boxCountY);
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[boxCountX*boxCountY];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, boxCountX, boxCountY, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, boxCountX, boxCountY);
#endif

	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMinATextureParam);
	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMaxATextureParam);
	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMinBTextureParam);
	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMaxBTextureParam);


	// Find the maximum extent of distance of bounding box with minimum distance
	float minBBoxMaxExtent = FindMinBBoxMaxExtent(tempTex[4], boxCountX, boxCountY, glParam);
	
	// Find boxes in range
	{
		cgGLBindProgram(glParam->bBoxDistFilterProgram);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[5]);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, boxCountX, boxCountY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[4]);

		cgGLSetTextureParameter(glParam->bBoxDistFilterTextureParam, tempTex[4]);
		cgGLEnableTextureParameter(glParam->bBoxDistFilterTextureParam);

		cgGLSetParameter1f(glParam->bBoxDistFilterMaxValParam, minBBoxMaxExtent);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[5], 0);
		glViewport(0, 0, boxCountX, boxCountY);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);

		DrawQuad(boxCountX, boxCountY);
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[distanceArraySizeX*distanceArraySizeY];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, distanceArraySizeX, distanceArraySizeY, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal, distanceArraySizeX, distanceArraySizeY);
#endif
		cgGLDisableTextureParameter(glParam->bBoxDistFilterTextureParam);
	}

	vector<int> positions;
	int numOverlaps;
	FindRenderLocation(tempTex[5], &numOverlaps, boxCountX, boxCountY, &positions, glParam, false);

	int numSurf1 = object1->faces.size();
	int numSurf2 = object2->faces.size();
	int hashListSize = numSurf1 * numSurf2;
	int* surfaceHashList = new int[hashListSize];
	for (int i = 0; i < hashListSize; i++)
		surfaceHashList[i] = 0;

	for (int k = 0; k < positions.size(); k++)
	{
		int pos = positions[k];
		int posX = pos%boxCountX;
		int posY = int(pos/boxCountX);
		int i = invIndexX[posX];
		int j = invIndexY[posY];
		object1->voxelData->bBox[i].intersecting = true;
		object2->voxelData->bBox[j].intersecting = true;
		for (int p = 0; p < object1->voxelData->bBox[i].surfaces.size(); p++)
		{
			for (int q = 0; q < object2->voxelData->bBox[j].surfaces.size(); q++)
			{
				int surfIndex1 = object1->voxelData->bBox[i].surfaces[p];
				int surfIndex2 = object2->voxelData->bBox[j].surfaces[q];
				int hash = surfIndex2*numSurf1 + surfIndex1;
				if (surfaceHashList[hash] == 0)
				{
					Float2 intSurfaces = Float2(surfIndex1,surfIndex2);
					intersectionList.push_back(intSurfaces);
					surfaceHashList[hash] = 1;
				}
			}
		}
	}

	delete[] surfaceHashList;

	// Delete Textures
	glDeleteTextures(6, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	return intersectionList;
}


vector<Float2> GPUObjectClosestPointFaceBased(Object* object1, Object* object2, GLParameters* glParam)
{
	vector<Float2> closeList;

	// Set up GPU computations
	StartGPUComputation(glParam);

	int boxCountX = object1->faces.size();
	int boxCountY = object2->faces.size();

	if (boxCountX > 4096 || boxCountY > 4096)
	{
		cout << "Cannot perform GPU collision detection " << endl;
		return closeList;
	}

	float* boxDataMin1 = new float[boxCountX*4];
	float* boxDataMax1 = new float[boxCountX*4];
	float* boxDataMin2 = new float[boxCountY*4];
	float* boxDataMax2 = new float[boxCountY*4];

	// Transform bounding boxes
	Float3 translate1 = Float3(object1->transformationMatrix[12], object1->transformationMatrix[13], object1->transformationMatrix[14]);
	
	for (int i = 0; i < boxCountX; i++)
	{
		object1->faces[i]->isMarked = false;
		boxDataMin1[i*4+0] = object1->faces[i]->bBoxMin[0] + translate1[0];
		boxDataMin1[i*4+1] = object1->faces[i]->bBoxMin[1] + translate1[1];
		boxDataMin1[i*4+2] = object1->faces[i]->bBoxMin[2] + translate1[2];
		boxDataMin1[i*4+3] = 1;							  
														  
		boxDataMax1[i*4+0] = object1->faces[i]->bBoxMax[0] + translate1[0];
		boxDataMax1[i*4+1] = object1->faces[i]->bBoxMax[1] + translate1[1];
		boxDataMax1[i*4+2] = object1->faces[i]->bBoxMax[2] + translate1[2];
		boxDataMax1[i*4+3] = 1;
	}

	// Transform bounding boxes
	Float3 translate2 = Float3(object2->transformationMatrix[12], object2->transformationMatrix[13], object2->transformationMatrix[14]);

	for (int i = 0; i < boxCountY; i++)
	{
		object2->faces[i]->isMarked = false;
		boxDataMin2[i*4+0] = object2->faces[i]->bBoxMin[0] + translate2[0];
		boxDataMin2[i*4+1] = object2->faces[i]->bBoxMin[1] + translate2[1];
		boxDataMin2[i*4+2] = object2->faces[i]->bBoxMin[2] + translate2[2];
		boxDataMin2[i*4+3] = 1;							  
														  
		boxDataMax2[i*4+0] = object2->faces[i]->bBoxMax[0] + translate2[0];
		boxDataMax2[i*4+1] = object2->faces[i]->bBoxMax[1] + translate2[1];
		boxDataMax2[i*4+2] = object2->faces[i]->bBoxMax[2] + translate2[2];
		boxDataMax2[i*4+3] = 1;
	}

	GLuint tempTex[5];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(6, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, boxCountX, 1, GL_RGBA, boxDataMin1);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, boxCountX, 1, GL_RGBA, boxDataMax1);
	InitTexture(tempTex+2, RGBA_FLOAT_FORMAT, boxCountY, 1, GL_RGBA, boxDataMin2);
	InitTexture(tempTex+3, RGBA_FLOAT_FORMAT, boxCountY, 1, GL_RGBA, boxDataMax2);
	InitTexture(tempTex+4, RGBA_FLOAT_FORMAT, boxCountX, boxCountY, GL_RGBA, NULL);

	//Bind the program
	cgGLBindProgram(glParam->objectBBoxDistanceProgram);

	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMinATextureParam);
	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMaxATextureParam);
	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMinBTextureParam);
	cgGLEnableTextureParameter(glParam->objectBBoxDistanceMaxBTextureParam);

	cgGLSetTextureParameter(glParam->objectBBoxDistanceMinATextureParam, tempTex[0]);
	cgGLSetTextureParameter(glParam->objectBBoxDistanceMaxATextureParam, tempTex[1]);
	cgGLSetTextureParameter(glParam->objectBBoxDistanceMinBTextureParam, tempTex[2]);
	cgGLSetTextureParameter(glParam->objectBBoxDistanceMaxBTextureParam, tempTex[3]);
	
	glActiveTextureARB(GL_TEXTURE3_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[3]);
	glActiveTextureARB(GL_TEXTURE2_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[2]);
	glActiveTextureARB(GL_TEXTURE1_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);

	glViewport(0, 0, boxCountX, boxCountY);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[4], 0);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(boxCountX, boxCountY);
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[boxCountX*boxCountY*4];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, boxCountX, boxCountY, GL_RGBA, GL_FLOAT, retVal);
	WriteToFile(retVal, boxCountX, boxCountY, 0);
#endif

	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMinATextureParam);
	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMaxATextureParam);
	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMinBTextureParam);
	cgGLDisableTextureParameter(glParam->objectBBoxDistanceMaxBTextureParam);


	// Find the maximum extent of distance of bounding box with minimum distance
	float minBBoxMaxExtent = FindMinBBoxMaxExtent(tempTex[4], boxCountX, boxCountY, glParam);
	
	// Find boxes in range
	{
		cgGLBindProgram(glParam->bBoxDistFilterProgram);

		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[5]);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, boxCountX, boxCountY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[4]);

		cgGLSetTextureParameter(glParam->bBoxDistFilterTextureParam, tempTex[4]);
		cgGLEnableTextureParameter(glParam->bBoxDistFilterTextureParam);

		cgGLSetParameter1f(glParam->bBoxDistFilterMaxValParam, minBBoxMaxExtent);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[5], 0);
		glViewport(0, 0, boxCountX, boxCountY);

		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);

		DrawQuad(boxCountX, boxCountY);
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[distanceArraySizeX*distanceArraySizeY];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, distanceArraySizeX, distanceArraySizeY, GL_RED, GL_FLOAT, retVal);
		WriteToFile(retVal, distanceArraySizeX, distanceArraySizeY);
#endif
		cgGLDisableTextureParameter(glParam->bBoxDistFilterTextureParam);
	}

	vector<int> positions;
	int numOverlaps;
	FindRenderLocation(tempTex[5], &numOverlaps, boxCountX, boxCountY, &positions, glParam, false);

	for (int k = 0; k < positions.size(); k++)
	{
		int pos = positions[k];
		int posX = pos%boxCountX;
		int posY = int(pos/boxCountX);
		Float2 closeData = Float2(posX,posY);
		closeList.push_back(closeData);
		object1->faces[posX]->isMarked = true;
		object2->faces[posY]->isMarked = true;
	}

	// Delete Textures
	glDeleteTextures(6, tempTex);

	// Clean up GPU computations
	EndGPUCompuation(glParam);

	return closeList;
}


float GetObjectClosestPoint(Object* object1, Object* object2, float* point1, float* point2, float* K, int intRes, GLParameters* glParam, bool forcedEvaluate)
{
	bool timing = false;
	long unsigned int closestPointTime = 0;
	long unsigned int surfaceClosestPointTime = 0;
	long unsigned int faceClosestPointTime = 0;
	clock_t beginClosestPointTime;
	clock_t beginSurfaceClosestPointTime;
	clock_t beginFaceClosestPointTime;
	bool firstStage = true;
	bool NURBSDist	= true;

	if (timing)
		beginClosestPointTime = clock();
	
	vector<Float2> iList;
	if (firstStage)
	{
		vector<Float2> iList1 =	GPUObjectClosestPoint(object1, object2, glParam);
		vector<Float2> iList2 =	GPUObjectClosestPointFaceBased(object1, object2, glParam);

		if (iList1.size() < iList2.size() && iList1.size() != 0)
			iList = iList1;
		else
			iList = iList2;
	}
	else
	{
		for (int p = 0; p < object1->faces.size(); p++)
		{
			for (int q = 0; q < object2->faces.size(); q++)
			{
				Float2 tempPair = Float2(p,q);
				iList.push_back(tempPair);
			}
		}
	}
	if (timing)
		closestPointTime += clock() - beginClosestPointTime;

	cout << "Surface Pairs  : " << iList.size() << endl;
	glParam->closestPoint = true;

	//perform surface closest point
	float minDist = -1;
	Float3 closestPoint1;
	Float3 closestPoint2;
	bool abort = false;
	int closestIndex = -1;
	if (iList.size() > 10 && !forcedEvaluate)
		abort = true;
	for (int i = 0; i < iList.size() && !abort; i++)
	{
		int faceIndex1 = iList[i][0];
		int faceIndex2 = iList[i][1];
		Face* face1 = object1->faces[faceIndex1];
		Face* face2 = object2->faces[faceIndex2];
		bool distComputed = false;
		if (face1->isNURBS && face2->isNURBS && NURBSDist)
		{
			distComputed = true;
			float point1[3];
			float point2[3];
			if (timing)
				beginSurfaceClosestPointTime = clock();
			float dist = GetSurfaceClosestPoint(face1->surface, face2->surface, point1, point2, intRes, glParam, false);
			if (dist < 0)
				distComputed = false;

			if (timing)
				surfaceClosestPointTime += clock() - beginSurfaceClosestPointTime;

			if (distComputed && (dist <= minDist || minDist < 0))
			{
				closestPoint1 = Float3(point1[0], point1[1], point1[2]);
				closestPoint2 = Float3(point2[0], point2[1], point2[2]);
				minDist = dist;
				*K = max(face1->surface->evalParams->maxExpansionFactor, face2->surface->evalParams->maxExpansionFactor);
				closestIndex = i;
			}
		}

		if (!distComputed)
		{
			float point1[3];
			float point2[3];
			if (timing)
				beginFaceClosestPointTime = clock();
			float dist = GetFaceFaceClosestPoint(face1, face2, object1->transformationMatrix, object2->transformationMatrix, point1, point2);
			if (timing)
				faceClosestPointTime += clock() - beginFaceClosestPointTime;
			if (dist <= minDist  || minDist < 0)
			{
				closestPoint1 = Float3(point1[0], point1[1], point1[2]);
				closestPoint2 = Float3(point2[0], point2[1], point2[2]);
				minDist = dist;
				*K = 0;
				closestIndex = i;
				cout << "Computing triangulated distance for pair " << iList[i][0] << " " << iList[i][0] << endl;
			}
		}
		if (i%100 == 0)
			cout << i << endl;
	}

	glParam->closestPoint = false;

	point1[0] = closestPoint1[0];
	point1[1] = closestPoint1[1];
	point1[2] = closestPoint1[2];
	point2[0] = closestPoint2[0];
	point2[1] = closestPoint2[1];
	point2[2] = closestPoint2[2];

	if (timing)
	{
		clock_t endTime = clock();
		cout<<"Minimum Distance  : "<< minDist << endl;
		cout<<"Tolerance         : "<< *K << endl;
		cout<<"Total Time        : "<<(0.001*(endTime - beginClosestPointTime))<<endl;
		cout<<"Closest Point Time: "<<(0.001*closestPointTime)<<endl;
		cout<<"Surface           : "<<(0.001*surfaceClosestPointTime)<<endl;
		cout<<"Face              : "<<(0.001*faceClosestPointTime)<<endl<<endl;
	}

	return minDist;
}


// Interactive Rendering
void InteractiveLOD(vector<NURBS*> nurbsSurfaces, vector<Object*> objects, Camera camera, GLParameters* glParam)
{
	// Initialize visibility factors to zero

	for (int i=0; i<nurbsSurfaces.size();i++)
		nurbsSurfaces[i]->visibilityFactor = 0;

	for (int i=0; i<objects.size(); i++)
	{
		objects[i]->visibilityFactor = 0;
		for (int j=0; j<objects[i]->faces.size(); j++)
			objects[i]->faces[j]->visibilityFactor = 0;
	}

	clock_t computationBeginTime = clock();
  	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho (camera.leftcp,camera.rightcp,camera.bottomcp,camera.topcp,camera.nearcp,camera.farcp);

	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	float viewMatrix[16];
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);
//	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

	int numViews = 36;
	int tileResX = 256;
	int tileResY = 256;
	int numViewsX = (int)ceil(sqrt(numViews*1.0));
	int numViewsY = (int)ceil(sqrt(numViews*1.0));


	GLuint tempTex;
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(1, &tempTex);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, tileResX*numViewsX, tileResY*numViewsY, 0, GL_RGBA, GL_FLOAT, 0);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, glParam->depthBuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, tileResX*numViewsX, tileResY*numViewsY);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, glParam->depthBuffer);

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex, 0);
	glEnable(GL_COLOR_ATTACHMENT0_EXT);

//	CheckFrameBuffer2();
	glViewport (0, 0, tileResX*numViewsX, tileResY*numViewsY);
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	for (int view = 0; view < numViews; view++)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		if (view < 12)
			glRotated(view*30.0,1,0,0);
		else if (view < 24)
			glRotated(view*30.0,0,1,0);
		else if (view < 36)
		{
			glRotated(90,0,1,0);
			glRotated(view*30.0,0,0,1);
		}
		glMultMatrixf(viewMatrix);

		int xViewPos = view%numViewsX;
		int yViewPos = (int)(view*1.0)/(numViewsX*1.0);

		glViewport (xViewPos*tileResX, yViewPos*tileResY, tileResX, tileResY);

		// To Init Depth Buffer draw objects once
		if (glParam->displayNURBS)
			for (int i=0; i<nurbsSurfaces.size();i++)
				nurbsSurfaces[i]->DrawNURBS(glParam, 1.0);

		// Draw all the objects
		for (int i=0; i<objects.size(); i++)
			objects[i]->DrawSceneObject(glParam, false, 1.0);

		// Draw objects again to count rendered Pixels
		if (glParam->displayNURBS)
		{
			for (int i=0; i<nurbsSurfaces.size();i++)
			{
				glBeginQueryARB(GL_SAMPLES_PASSED_ARB,glParam->occlusionQuery);
				glColor4f(rand()/(RAND_MAX*1.0),.0,.7,1);
				nurbsSurfaces[i]->DrawNURBS(glParam, 1.0);
				glEndQueryARB(GL_SAMPLES_PASSED_ARB);
				GLuint fragmentCount;
				glGetQueryObjectuivARB(glParam->occlusionQuery, GL_QUERY_RESULT_ARB, &fragmentCount);
				float coverageRatio = (fragmentCount*1.0)/(tileResX*tileResY*1.0);
				nurbsSurfaces[i]->visibilityFactor += coverageRatio;
			}
		}

		// Draw all the objects
		for (int i=0; i<objects.size(); i++)
		{
			glColor4f(rand()/(RAND_MAX*1.0),.0,.7,1);
			objects[i]->DrawSceneObject(glParam, true, 1.0);
		}

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
#ifdef DUMP
		GLfloat *retVal = new GLfloat[tileResX*numViewsX*tileResY*numViewsY];
		glReadBuffer(GL_DEPTH_ATTACHMENT_EXT);
		glReadPixels(0,0,tileResX*numViewsX, tileResY*numViewsY,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,tileResX*numViewsX, tileResY*numViewsY);
#endif

	for (int i=0; i<objects.size(); i++)
	{
		for (int j=0; j<objects[i]->faces.size(); j++)
		{
			float distance = objects[i]->faces[j]->GetDistanceFromSphere(viewMatrix)/100.0;
			objects[i]->faces[j]->visibilityFactor*=distance/(tileResX*tileResY*numViews*1.0);
			objects[i]->visibilityFactor += objects[i]->faces[j]->visibilityFactor;
		}
	}

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDeleteTextures(1, &tempTex);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	unsigned int computationTime = (clock()-computationBeginTime);
	cout << "Visibility computed in " << computationTime*0.001 << " seconds" << endl;
}

void DrawSurfaceIDNURBS(NURBS* surface, GLParameters* glParam)
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf(surface->transformationMatrix);
	
	int uNum = surface->uNumDisplay;
	int vNum = surface->vNumDisplay;

	cgGLEnableProfile(glParam->cgFragmentProfile);
	if (surface->trimmed)
	{
		glActiveTexture(GL_TEXTURE0);
		glEnable(TRIM_TEXTURE_FORMAT);
		glBindTexture(TRIM_TEXTURE_FORMAT, surface->trimTexture);
		cgGLBindProgram(glParam->trimSurfaceIDProgram);
		cgGLSetTextureParameter(glParam->trimSurfaceIDTextureParam, surface->trimTexture);
		cgGLEnableTextureParameter(glParam->trimSurfaceIDTextureParam);
		cgGLSetParameter1f(glParam->trimSurfaceIDParam, surface->surfaceID);
	}
	else
	{
		cgGLBindProgram(glParam->passiveSurfaceIDProgram);
		cgGLSetParameter1f(glParam->passiveSurfaceIDParam, surface->surfaceID);
	}
#ifdef VBO	
	glEnable(GL_VERTEX_ARRAY);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, surface->ptVBO);  // BIND VBO
	glVertexPointer(4, GL_FLOAT, 0, 0);
	glEnable(GL_NORMAL_ARRAY);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, surface->normVBO); 
	glNormalPointer(GL_FLOAT,0,0);
	glEnable(GL_TEXTURE_COORD_ARRAY);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, surface->texVBO);
	glTexCoordPointer(2,GL_FLOAT,0,0);
	glDrawElements(GL_QUADS,uNum*vNum*4,GL_UNSIGNED_INT, surface->indexArray);
#endif
	glFlush();
	glFinish();
	if (surface->trimmed)
	{
		glDisable(TRIM_TEXTURE_FORMAT);
		cgGLDisableTextureParameter(glParam->trimSurfaceIDTextureParam);
	}

	cgGLDisableProfile(glParam->cgFragmentProfile);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
	glBindBufferARB(GL_ARRAY_BUFFER, 0);
	glDisableClientState (GL_VERTEX_ARRAY);

	glPopMatrix();
	glFlush();
	glFinish();
}

void GenerateSurfaceID(vector<NURBS*> nurbsSurfaces, vector<Object*> objects, Camera camera, Viewport viewport, GLuint surfaceIDTex, GLParameters* glParam)
{
	clock_t computationBeginTime = clock();
  	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(camera.leftcp,camera.rightcp,camera.bottomcp,camera.topcp,camera.nearcp,camera.farcp);

	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);	
	float viewMatrix[16];
	glMatrixMode(GL_MODELVIEW);
	glGetFloatv(GL_MODELVIEW_MATRIX,viewMatrix);
//	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(1, &surfaceIDTex);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, surfaceIDTex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, LUMINANCE_FLOAT_FORMAT, viewport.w, viewport.h, 0, GL_LUMINANCE, GL_FLOAT, 0);

	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);

	GLuint depthBuffer;
	glGenRenderbuffersEXT(1, &depthBuffer);
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthBuffer);
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT, viewport.w, viewport.h);
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, depthBuffer);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, surfaceIDTex, 0);
	glEnable(GL_COLOR_ATTACHMENT0_EXT);

//	CheckFrameBuffer2();
	glViewport (0, 0, viewport.w, viewport.h);
	glClearColor(-1,-1,-1,-1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixf(viewMatrix); 

	// Draw the surfaces with surfaceID
	for (int i=0; i<nurbsSurfaces.size();i++)
		DrawSurfaceIDNURBS(nurbsSurfaces[i], glParam);

	// Draw all the objects
	for (int i=0; i<objects.size(); i++)
		objects[i]->DrawSceneObject(glParam, false, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

#ifdef DUMP
	GLfloat *retVal = new GLfloat[viewport.w*viewport.h];
	glReadBuffer(GL_DEPTH_ATTACHMENT_EXT);
	glReadPixels(0,0,viewport.w,viewport.h,GL_RED,GL_FLOAT,retVal);
	WriteToFileInt(retVal,viewport.w,viewport.h);
#endif

	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
	glDeleteRenderbuffersEXT(1, &depthBuffer);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	unsigned int computationTime = (clock()-computationBeginTime);
	cout << "Surface ID computed in " << computationTime*0.001 << " seconds" << endl;
}
