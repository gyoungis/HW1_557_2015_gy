#pragma once
#include "cutil_math.h"
#include "CUDAHelper.h"
  
//use closest control point initialization of iteration or use midpoint
#define USE_CLOSEST_CP_INIT 1

//maximal number of iterations - TODO: pass it as a parameter and conduct experiments..
#define MAX_ITER 5

#define MAX_ORDER 10 
#define UV_STOP_TOLERANCE 0.01

//iddo testing - stopping when distance differences are less than Kb - TODO pass it as param
//#define KB_STOP_TOLERANCE 0.01

// Evaluating Bez crv (up to order MAX_ORDER) - storing only three points needed for
// evaluating derivs (not all subdivided crvs)
__device__ void EvalBezCrv(float4 crv[MAX_ORDER], int ordT, float t, float4* ct, float4* ctLeft, float4* ctRight)
{
	float4 crvAux[MAX_ORDER];
	float4 crvAuxTmp[MAX_ORDER];
	int i,j;
	for (i=0; i<ordT; ++i) 
		crvAux[i] = crv[i]; //copy crv to crvAux (maybe can work directly on crv if we copy from outside)
	for (i=1; i<ordT-1; ++i)
	{
		for (j=0; j<ordT-i; ++j) 
			crvAuxTmp[j] = lerp(crvAux[j], crvAux[j+1], t);

		for (j=0; j<ordT-i; ++j)
			crvAux[j] = crvAuxTmp[j]; //copy auxTmp to aux
	}
	//when reaching here, crvAux[0],crvAux[1] should hold cuLeft and cuRight
	*ctLeft = crvAux[0];
	*ctRight = crvAux[1];
	*ct = lerp(*ctLeft, *ctRight, t);
}

// Returns surface evaluation and partial derivitives at (u,v).
// Note that if surface is rational the actual partial derivative computation
// should use the formula (Su*w-S*wu)/w^2 
__device__ void EvalBezSrf(float4* surface2Cps, int ordU, int ordV, float u, float v, float4* S, float4* Su, float4* Sv)
{
	int i,j;
	float4 c[MAX_ORDER];
	float4 cL[MAX_ORDER];
	float4 cR[MAX_ORDER];
	for (i=0; i<ordU; ++i)
	{
		float4 crv[MAX_ORDER];
		for (j=0; j<ordV;++j)
		{
			float4 cpij = surface2Cps[j*ordU+i];
			crv[j] = cpij;
		}
		float4 cv, cvL, cvR;
		EvalBezCrv(crv,ordV,v, &cv, &cvL, &cvR);
		c[i] = cv;
		cL[i] = cvL;
		cR[i] = cvR;
	}

	float4 cuL, cuR;
	EvalBezCrv(c, ordU, u, S, &cuL, &cuR);
	*Su = ordU*(cuR-cuL);

	float4 c1, c_1, c1L, c1R;
	EvalBezCrv(cR, ordU, u, &c1, &c1L, &c1R);
	EvalBezCrv(cL, ordU, u, &c_1, &c1L, &c1R);
	*Sv = ordV*(c1-c_1);
}


// Projecting a point pt onto the tangent plane at (u,v) resulting in du,dv
// du = <p-S(u,v),Su>/<Su,Su>
// dv = <p-S(u,v),Sv>/<Sv,Sv>

// projdist needed for kb dist stop criteria
__device__ void PointProjectOnTangentPlane(float3 pt, float4* surface2Cps, int ordU, int ordV, float u, float v, float* du, float* dv, float* projdist)
{
	// Assume rational surface:
	float4 rS, rSu, rSv;
	EvalBezSrf(surface2Cps, ordU, ordV, u, v, &rS, &rSu, &rSv);

	float3 S, Su, Sv;
	//iddo - in the future possibly optimize for non-rational surfaces
	S.x = rS.x / rS.w;
	S.y = rS.y / rS.w;
	S.z = rS.z / rS.w;

	Su.x = (rS.w*rSu.x - rSu.w*rS.x)/(rS.w*rS.w);
	Su.y = (rS.w*rSu.y - rSu.w*rS.y)/(rS.w*rS.w);
	Su.z = (rS.w*rSu.z - rSu.w*rS.z)/(rS.w*rS.w);

	Sv.x = (rS.w*rSv.x - rSv.w*rS.x)/(rS.w*rS.w);
	Sv.y = (rS.w*rSv.y - rSv.w*rS.y)/(rS.w*rS.w);
	Sv.z = (rS.w*rSv.z - rSv.w*rS.z)/(rS.w*rS.w);

	//Project onto plane and use barycentric coordinates (computed with area ratios)
	//1. Projecting pt onto plane
	float3 SuxSv = cross(Su,Sv);

	//singular point workaround: if on a singular point perturb a little
	if (length(SuxSv) == 0.0)
	{
		*du = 2.0f*UV_STOP_TOLERANCE;
		if (u + *du > 1.0f) *du = -2.0f*UV_STOP_TOLERANCE;
		*dv = 2.0f*UV_STOP_TOLERANCE;
		if (v + *dv > 1.0f) *dv = -2.0f*UV_STOP_TOLERANCE;
		return;
	}


	float3 pr = pt - (dot(pt-S,SuxSv)/dot(SuxSv,SuxSv))*SuxSv;
	//du = SignedAreaOfTriangle(pr, S+Sv, S)/SignedAreaOfTriangle(S,S+Su,S+Sv);
	//dv = SignedAreaOfTriangle(pr, S, S+Su)/SignedAreaOfTriangle(S,S+Su,S+Sv);
	float3 prSxSv = cross(pr-S,Sv);
	*du = length(prSxSv)/length(SuxSv);
	if (dot(prSxSv,SuxSv) < 0)
		*du = -(*du);
	float3 SuxprS = cross(Su,pr-S);
	*dv = length(SuxprS)/length(SuxSv);
	if (dot(SuxprS,SuxSv) < 0)
		*dv = -(*dv);

	//iddo testing - needed for kb dist stop criteria
	*projdist = length(pt-pr);
}


__global__ void CUDAHausdorffIddoKernel(float4* surf1Pts, float4* prevDists, float* prevConvStatus, int uNum1, int vNum1, float4* surface2Cps, int ordU, int ordV, 
										float kb_stop_tolerance, //iddo testing - if negative, stop criterion with UVTOL, if positive stop by kb distance comparison
										float4* oPoint, float* currentConvStatus)
{
	unsigned int i = blockIdx.x*blockDim.x + threadIdx.x;
	unsigned int j = blockIdx.y*blockDim.y + threadIdx.y;

	int index = j*uNum1+i;

	if (i < uNum1 && j < vNum1)
	{
		float4 pt = surf1Pts[index];
		float3 p = make_float3(pt.x,pt.y,pt.z);
		if (pt.w != 1.0) p = p/pt.w;


		float4 cp4 = surface2Cps[0];
		float3 cp = make_float3(cp4.x,cp4.y,cp4.z);
		if (cp4.w != 1.0) cp = cp/cp4.w;

		float dist = length(p-cp);
		float i0=0, j0=0;

#ifdef USE_CLOSEST_CP_INIT 
		// Finding an initial guess by going over the control mesh
		//(iddo: need to test if this slows down the shader and maybe using midpoint is better)
		for (int i=0; i<ordU; ++i)
		{
			for (int j=0; j<ordV;++j)
			{
				float4 cpij4 = surface2Cps[j*ordU+i];
				float3 cpij = make_float3(cpij4.x,cpij4.y,cpij4.z);
				if (cpij4.w != 1.0)
					cpij = cpij/cpij4.w;

				float tmp = length(p-cpij);
				if (tmp < dist)
				{
					dist = tmp;
					cp = cpij;
					i0 = i; j0=j;
				}		
			}
		}

		// Once we are here, we have an initial i0,j0:
		float u = float(i0)/float(ordU-1);
		float v = float(j0)/float(ordV-1);
#else
		//use midpoint initialization:
		float u = 0.5;
		float v = 0.5;
#endif

		/*
		//iddo debug
		float4 prevPtD = prevDists[index];
		if (dist < prevPtD.w)
		{
			oPoint[index] = make_float4(cp.x, cp.y, cp.z, dist);
		}
		else
		{
			oPoint[index] = prevPtD;
		}
		return;
		//end debug
		*/

		//NR iteration:
		float iterDist, prevIterDist, prevPrevIterDist;
		float alpha = 0;
		int breakIter = MAX_ITER;
		float currentDiff = -1;
		for (int iter=0; iter < MAX_ITER; ++iter) //a higher value slows down the computation somewhat but is more accurate..
		{
			float du, dv;

			/*		
			PointProjectOnTangentPlane(p, surface2Cps, 
			ordU, ordV,
			u, v,
			&du, &dv);
			*/
			//iddo testing - iterDist comparison as stop criterion.
			PointProjectOnTangentPlane(p, surface2Cps, ordU, ordV, u, v, &du, &dv, &iterDist);

			//iddo testing - compare with uv tol if kb_stop_tolerance is negative

			//stopping criterion
			// Getting rid of UV Stop criterion completely for now
			/*
			if ((kb_stop_tolerance < 0) && abs(du) < UV_STOP_TOLERANCE && abs(dv) < UV_STOP_TOLERANCE)
			{
				converged = 1;
				break;
			}
			else 
			{
			*/
			u += du;
			if (u > 1.0) u = 1.0;
			if (u < 0.0) u = 0.0;
			v += dv;
			if (v > 1.0) v = 1.0;
			if (v < 0.0) v = 0.0;

			/*
			//iddo - in this implementation I use an extra evaluation here - we can probably use this evaluation inside the
			//next iteration of PointProjectOnTangentPlane, or do something else more efficient.. 
			//note: this can have a real effect on runtime - need to optimize the code..
			float4 S, Su, Sv;
			EvalBezSrf(surface2Cps, ordU, ordV, u, v, &S, &Su, &Sv);
			float3 S3 = make_float3(S.x,S.y,S.z);
			if (S.w != 1.0) S3 = S3/S.w;
			float iterDist = length(p-S3);  
			*/

			//iddo - in this implementation I compute the projection distance (iterDist) inside PointProjectOnTangentPlane and use that..
			if (iter> 0) //no comparison in first iter
			{
				currentDiff		= fabs(prevIterDist - iterDist);
				float prevDiff	= fabs(prevPrevIterDist - prevIterDist);
				if (iter > 1)
					alpha = currentDiff/prevDiff;
				else 
					prevPrevIterDist = prevIterDist;
				//iterative method stopping criterion based on distance diff comparison (activated if kb_stop_tolerance >=0)..
				if (currentDiff < kb_stop_tolerance)
				{
					breakIter = iter;
					break;
				}
			}

			prevPrevIterDist = prevIterDist;
			prevIterDist = iterDist;

			// }// UV stop condition
		}

		float prevAlpha = prevConvStatus[index];

		// Evaluate the distance from the projected point
		float4 S, Su, Sv;
		EvalBezSrf(surface2Cps, ordU, ordV, u, v, &S, &Su, &Sv);

		float3 S3 = make_float3(S.x,S.y,S.z);
		if (S.w != 1.0) S3 = S3/S.w;
		dist = length(p-S3);  

		float4 prevPt = prevDists[index];
		if (dist < prevPt.w)
		{
			//currentConvStatus[index] = alpha;
			currentConvStatus[index] = breakIter;
			//currentConvStatus[index] = currentDiff;
			oPoint[index] = make_float4(S3.x, S3.y, S3.z, dist);
		}
		else
		{
			oPoint[index] = prevPt;
			currentConvStatus[index] = prevAlpha;
		}
	}
} 

// Wrapper for the __global__ call that sets up the kernel call
extern "C" int CUDAHausdorffIddo(float4* surf1Pts, float4* prevDists, float* prevConvStatus, int uNum1, int vNum1, float4* surface2Cps, int ordU, int ordV, float kBStopTolerance, float4* oPoint, float* currentConvStatus)
{
	// execute the kernel
	//dim3 block(GetBlockSize(uNum1,4), GetBlockSize(vNum1,4), 1); //works!!
	dim3 block(GetBlockSize(uNum1,8), GetBlockSize(vNum1,8), 1);
	//dim3 block(GetBlockSize(uNum1,16), GetBlockSize(vNum1,16), 1);
	dim3 grid(GetGridSize(uNum1,block.x), GetGridSize(vNum1,block.y), 1);
	CUDAHausdorffIddoKernel<<<grid, block>>>(surf1Pts, prevDists, prevConvStatus, uNum1, vNum1, surface2Cps, ordU, ordV, kBStopTolerance, oPoint, currentConvStatus);
	return 1;
}

