#include "Utilities.h"

bool PointBoxOverlap(BBoxData* box, Float3 p)
{
	return ( p[0] >= box->minPoint[0] && p[0] <= box->maxPoint[0] &&
			 p[1] >= box->minPoint[1] && p[1] <= box->maxPoint[1] &&
			 p[2] >= box->minPoint[2] && p[2] <= box->maxPoint[2]);
}
/*
bool OBBOverlap(BBoxData* box1, BBoxData* box2, float transform1[16], float transform2[16], float gridSize1, float gridSize2)
{
	bool overlap = false;
	// Using conventions from collision detection book
	// c OBB center point
	// u[3] Local x,y, & z axes
	// e positive halfwidth extents

	float T[16];
	MultiplyTransforms(transform2, transform1, T);

	Float3 origBox2v000 = Float3(box2->minPoint[0], box2->minPoint[1], box2->minPoint[2]);
	Float3 origBox2v111 = Float3(box2->maxPoint[0], box2->maxPoint[1], box2->maxPoint[2]);

	Float3 box2v000 = TransformPoint(origBox2v000, T);
	Float3 box2v111 = TransformPoint(origBox2v111, T);

	Float3 xAxis = Float3(T[0], T[4], T[8]);
	Float3 yAxis = Float3(T[1], T[5], T[9]);
	Float3 zAxis = Float3(T[2], T[6], T[10]);
	VectorNormalize(xAxis);
	VectorNormalize(yAxis);
	VectorNormalize(zAxis);

	xAxis *= VectorDotProduct(xAxis, (box2v111 - box2v000));
	yAxis *= VectorDotProduct(yAxis, (box2v111 - box2v000));
	zAxis *= VectorDotProduct(zAxis, (box2v111 - box2v000));

	Float3 box2v100 = box2v000 + xAxis;
	Float3 box2v010 = box2v000 + yAxis;
	Float3 box2v001 = box2v000 + zAxis;
	Float3 box2v110 = box2v000 + xAxis + yAxis;
	Float3 box2v011 = box2v000 + yAxis + zAxis;
	Float3 box2v101 = box2v000 + xAxis + zAxis;


	if		(PointBoxOverlap(box1, box2v000)) overlap = true;
	else if (PointBoxOverlap(box1, box2v111)) overlap = true;
	else if (PointBoxOverlap(box1, box2v100)) overlap = true;
	else if (PointBoxOverlap(box1, box2v010)) overlap = true;
	else if (PointBoxOverlap(box1, box2v001)) overlap = true;
	else if (PointBoxOverlap(box1, box2v110)) overlap = true;
	else if (PointBoxOverlap(box1, box2v011)) overlap = true;
	else if (PointBoxOverlap(box1, box2v101)) overlap = true;
	
	return overlap;
}
*/

bool OBBOverlap(BBoxData* boxA, BBoxData* boxB, float transformA[16], float transformB[16], float gridSizeA, float gridSizeB)
{
    // Matrix to transform OBB B into A reference to allow A to be treated as an AABB
	// Matrix transformBA = transformB * Matrix.Invert(transformA);
	float invTransformA[16];
	float transformBA[16];
	InvertMatrix(transformA, invTransformA);
	MultiplyTransforms(transformB, invTransformA, transformBA);

	Float3 centerA		= (boxA->maxPoint + boxA->minPoint) * 0.5;
	Float3 extentA		= (boxA->maxPoint - boxA->minPoint) * 0.5;

	Float3 origCenterB	= (boxB->maxPoint + boxB->minPoint) * 0.5;
	Float3 extentB		= (boxB->maxPoint - boxB->minPoint) * 0.5;
    Float3 centerB		= TransformPoint(origCenterB, transformBA);

	Float3 separation	= centerB - centerA;

	Float3 rotRow0 = Float3(transformBA[0], transformBA[1], transformBA[2]);
	Float3 rotRow1 = Float3(transformBA[4], transformBA[5], transformBA[6]);
	Float3 rotRow2 = Float3(transformBA[8], transformBA[9], transformBA[10]);

	Float3 absRotRow0 = Float3(fabs(transformBA[0]), fabs(transformBA[1]), fabs(transformBA[2 ]));
	Float3 absRotRow1 = Float3(fabs(transformBA[4]), fabs(transformBA[5]), fabs(transformBA[6 ]));
	Float3 absRotRow2 = Float3(fabs(transformBA[8]), fabs(transformBA[9]), fabs(transformBA[10]));

	Float3 absRotCol0 = Float3(fabs(transformBA[0]), fabs(transformBA[4]), fabs(transformBA[8 ]));
	Float3 absRotCol1 = Float3(fabs(transformBA[1]), fabs(transformBA[5]), fabs(transformBA[9 ]));
	Float3 absRotCol2 = Float3(fabs(transformBA[2]), fabs(transformBA[6]), fabs(transformBA[10]));

	float r, r0, r1, r01;

    //--- Test case 1 - X axis
    r = separation[0];
	r1 = VectorDotProduct(extentB, absRotCol0);
    r01 = extentA[0] + r1;

    if (r > r01) return false;

    //--- Test case 1 - Y axis
    r = separation[1];
	r1 = VectorDotProduct(extentB, absRotCol1);
    r01 = extentA[1] + r1;

    if (r > r01) return false;

    //--- Test case 1 - Z axis
    r = separation[2];
	r1 = VectorDotProduct(extentB, absRotCol2);
    r01 = extentA[2] + r1;

	if (r > r01) return false;

    //--- Test case 2 - X axis
    r = fabs(VectorDotProduct(rotRow0, separation));
    r0 = VectorDotProduct(extentA, absRotRow0);
    r01 = r0 + extentB[0];

    if (r > r01) return false;

    //--- Test case 2 - Y axis
    r = fabs(VectorDotProduct(rotRow1, separation));
    r0 = VectorDotProduct(extentA, absRotRow1);
    r01 = r0 + extentB[1];

    if (r > r01) return false;

    //--- Test case 2 - Z axis
    r = fabs(VectorDotProduct(rotRow2, separation));
    r0 = VectorDotProduct(extentA, absRotRow2);
    r01 = r0 + extentB[2];

    if (r > r01) return false;

    //--- Test case 3 # 1
	r = fabs(separation[2] * transformBA[1] - separation[1] * transformBA[2]);
    r0 = extentA[1] * fabs(transformBA[2])  + extentA[2] * fabs(transformBA[1]);
    r1 = extentB[1] * fabs(transformBA[8])  + extentB[2] * fabs(transformBA[4]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 2
	r = fabs(separation[2] * transformBA[5] - separation[1] * transformBA[6]);
    r0 = extentA[1] * fabs(transformBA[6])  + extentA[2] * fabs(transformBA[5]);
    r1 = extentB[0] * fabs(transformBA[8])  + extentB[2] * fabs(transformBA[0]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 3
	r = fabs(separation[2] * transformBA[9] - separation[1] * transformBA[10]);
    r0 = extentA[1] * fabs(transformBA[10]) + extentA[2] * fabs(transformBA[9]);
    r1 = extentB[0] * fabs(transformBA[4])  + extentB[1] * fabs(transformBA[0]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 4
	r = fabs(separation[0] * transformBA[2] - separation[2] * transformBA[0]);
    r0 = extentA[0] * fabs(transformBA[2]) + extentA[2] * fabs(transformBA[0]);
    r1 = extentB[1] * fabs(transformBA[9]) + extentB[2] * fabs(transformBA[5]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 5
	r = fabs(separation[0] * transformBA[6] - separation[2] * transformBA[4]);
    r0 = extentA[0] * fabs(transformBA[6]) + extentA[2] * fabs(transformBA[4]);
    r1 = extentB[0] * fabs(transformBA[9]) + extentB[2] * fabs(transformBA[1]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 6
	r = fabs(separation[0] * transformBA[10] - separation[2] * transformBA[8]);
    r0 = extentA[0] * fabs(transformBA[10]) + extentA[2] * fabs(transformBA[8]);
    r1 = extentB[0] * fabs(transformBA[5]) + extentB[1] * fabs(transformBA[1]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 7
	r = fabs(separation[1] * transformBA[0] - separation[0] * transformBA[1]);
    r0 = extentA[0] * fabs(transformBA[1])  + extentA[1] * fabs(transformBA[0]);
    r1 = extentB[1] * fabs(transformBA[10]) + extentB[2] * fabs(transformBA[6]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 8
	r = fabs(separation[1] * transformBA[4] - separation[0] * transformBA[5]);
    r0 = extentA[0] * fabs(transformBA[5])  + extentA[1] * fabs(transformBA[4]);
    r1 = extentB[0] * fabs(transformBA[10]) + extentB[2] * fabs(transformBA[2]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 9
	r = fabs(separation[1] * transformBA[8] - separation[0] * transformBA[9]);
    r0 = extentA[0] * fabs(transformBA[9]) + extentA[1] * fabs(transformBA[8]);
    r1 = extentB[0] * fabs(transformBA[6]) + extentB[1] * fabs(transformBA[2]);
    r01 = r0 + r1;

    if (r > r01) return false;

	// No separating axis, then we have intersection
    return true;  
}

bool OBBOverlap(BBoxData* boxA, BBoxData* boxB, float transformBA[16])
{
	Float3 centerA		= (boxA->maxPoint + boxA->minPoint) * 0.5;
	Float3 extentA		= (boxA->maxPoint - boxA->minPoint) * 0.5;

	Float3 origCenterB	= (boxB->maxPoint + boxB->minPoint) * 0.5;
	Float3 extentB		= (boxB->maxPoint - boxB->minPoint) * 0.5;
    Float3 centerB		= TransformPoint(origCenterB, transformBA);

	Float3 separation	= centerB - centerA;

	Float3 rotRow0 = Float3(transformBA[0], transformBA[1], transformBA[2]);
	Float3 rotRow1 = Float3(transformBA[4], transformBA[5], transformBA[6]);
	Float3 rotRow2 = Float3(transformBA[8], transformBA[9], transformBA[10]);

	Float3 absRotRow0 = Float3(fabs(transformBA[0]), fabs(transformBA[1]), fabs(transformBA[2 ]));
	Float3 absRotRow1 = Float3(fabs(transformBA[4]), fabs(transformBA[5]), fabs(transformBA[6 ]));
	Float3 absRotRow2 = Float3(fabs(transformBA[8]), fabs(transformBA[9]), fabs(transformBA[10]));

	Float3 absRotCol0 = Float3(fabs(transformBA[0]), fabs(transformBA[4]), fabs(transformBA[8 ]));
	Float3 absRotCol1 = Float3(fabs(transformBA[1]), fabs(transformBA[5]), fabs(transformBA[9 ]));
	Float3 absRotCol2 = Float3(fabs(transformBA[2]), fabs(transformBA[6]), fabs(transformBA[10]));

	float r, r0, r1, r01;

    //--- Test case 1 - X axis
    r = separation[0];
	r1 = VectorDotProduct(extentB, absRotCol0);
    r01 = extentA[0] + r1;

    if (r > r01) return false;

    //--- Test case 1 - Y axis
    r = separation[1];
	r1 = VectorDotProduct(extentB, absRotCol1);
    r01 = extentA[1] + r1;

    if (r > r01) return false;

    //--- Test case 1 - Z axis
    r = separation[2];
	r1 = VectorDotProduct(extentB, absRotCol2);
    r01 = extentA[2] + r1;

	if (r > r01) return false;

    //--- Test case 2 - X axis
    r = fabs(VectorDotProduct(rotRow0, separation));
    r0 = VectorDotProduct(extentA, absRotRow0);
    r01 = r0 + extentB[0];

    if (r > r01) return false;

    //--- Test case 2 - Y axis
    r = fabs(VectorDotProduct(rotRow1, separation));
    r0 = VectorDotProduct(extentA, absRotRow1);
    r01 = r0 + extentB[1];

    if (r > r01) return false;

    //--- Test case 2 - Z axis
    r = fabs(VectorDotProduct(rotRow2, separation));
    r0 = VectorDotProduct(extentA, absRotRow2);
    r01 = r0 + extentB[2];

    if (r > r01) return false;

    //--- Test case 3 # 1
	r = fabs(separation[2] * transformBA[1] - separation[1] * transformBA[2]);
    r0 = extentA[1] * fabs(transformBA[2])  + extentA[2] * fabs(transformBA[1]);
    r1 = extentB[1] * fabs(transformBA[8])  + extentB[2] * fabs(transformBA[4]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 2
	r = fabs(separation[2] * transformBA[5] - separation[1] * transformBA[6]);
    r0 = extentA[1] * fabs(transformBA[6])  + extentA[2] * fabs(transformBA[5]);
    r1 = extentB[0] * fabs(transformBA[8])  + extentB[2] * fabs(transformBA[0]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 3
	r = fabs(separation[2] * transformBA[9] - separation[1] * transformBA[10]);
    r0 = extentA[1] * fabs(transformBA[10]) + extentA[2] * fabs(transformBA[9]);
    r1 = extentB[0] * fabs(transformBA[4])  + extentB[1] * fabs(transformBA[0]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 4
	r = fabs(separation[0] * transformBA[2] - separation[2] * transformBA[0]);
    r0 = extentA[0] * fabs(transformBA[2]) + extentA[2] * fabs(transformBA[0]);
    r1 = extentB[1] * fabs(transformBA[9]) + extentB[2] * fabs(transformBA[5]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 5
	r = fabs(separation[0] * transformBA[6] - separation[2] * transformBA[4]);
    r0 = extentA[0] * fabs(transformBA[6]) + extentA[2] * fabs(transformBA[4]);
    r1 = extentB[0] * fabs(transformBA[9]) + extentB[2] * fabs(transformBA[1]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 6
	r = fabs(separation[0] * transformBA[10] - separation[2] * transformBA[8]);
    r0 = extentA[0] * fabs(transformBA[10]) + extentA[2] * fabs(transformBA[8]);
    r1 = extentB[0] * fabs(transformBA[5]) + extentB[1] * fabs(transformBA[1]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 7
	r = fabs(separation[1] * transformBA[0] - separation[0] * transformBA[1]);
    r0 = extentA[0] * fabs(transformBA[1])  + extentA[1] * fabs(transformBA[0]);
    r1 = extentB[1] * fabs(transformBA[10]) + extentB[2] * fabs(transformBA[6]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 8
	r = fabs(separation[1] * transformBA[4] - separation[0] * transformBA[5]);
    r0 = extentA[0] * fabs(transformBA[5])  + extentA[1] * fabs(transformBA[4]);
    r1 = extentB[0] * fabs(transformBA[10]) + extentB[2] * fabs(transformBA[2]);
    r01 = r0 + r1;

    if (r > r01) return false;

    //--- Test case 3 # 9
	r = fabs(separation[1] * transformBA[8] - separation[0] * transformBA[9]);
    r0 = extentA[0] * fabs(transformBA[9]) + extentA[1] * fabs(transformBA[8]);
    r1 = extentB[0] * fabs(transformBA[6]) + extentB[1] * fabs(transformBA[2]);
    r01 = r0 + r1;

    if (r > r01) return false;

	// No separating axis, then we have intersection
    return true;  
}
