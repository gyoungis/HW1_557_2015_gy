/*
 * Copyright 1993-2013 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

// Simple 3D volume renderer

#ifndef _VOLUMERENDER_KERNEL_CU_
#define _VOLUMERENDER_KERNEL_CU_

//#define TEXTURE  // Use 3D texture instead of arrays
//#define VOLRENDER

#include <helper_cuda.h>
#include <helper_math.h>

typedef unsigned int  uint;
typedef unsigned char uchar;

cudaArray *d_volumeArray = 0;
// ---------------------- 3 more cudaArrays --------------------
cudaArray *d_volumeArray2 = 0;
cudaArray *d_volumeArray3 = 0;
cudaArray *d_volumeArray4 = 0;
// ----------------------------------------------------------------
cudaArray *d_transferFuncArray;

typedef unsigned char VolumeType;
//typedef unsigned short VolumeType;

texture<VolumeType, 3, cudaReadModeNormalizedFloat> tex;         // 3D texture

// ----------------------- Float Pointers for arrays ------------------------
unsigned char *gpuArray;
unsigned char *xArray;
unsigned char *yArray;
unsigned char *zArray;
// --------------------------------------------------------------


// ------------------------------ 3 more 3D textures ------------------------
texture<VolumeType, 3, cudaReadModeNormalizedFloat> tex2;
texture<VolumeType, 3, cudaReadModeNormalizedFloat> tex3;
texture<VolumeType, 3, cudaReadModeNormalizedFloat> tex4;
// --------------------------------------------------------------------------

texture<float4, 1, cudaReadModeElementType>         transferTex; // 1D transfer function texture

typedef struct
{
    float4 m[3];
} float3x4;

__constant__ float3x4 c_invViewMatrix;  // inverse view matrix

struct Ray
{
    float3 o;   // origin
    float3 d;   // direction
};

// intersect ray with a box
// http://www.siggraph.org/education/materials/HyperGraph/raytrace/rtinter3.htm

__device__
int intersectBox(Ray r, float3 boxmin, float3 boxmax, float *tnear, float *tfar)
{
    // compute intersection of ray with all six bbox planes
    float3 invR = make_float3(1.0f) / r.d;
    float3 tbot = invR * (boxmin - r.o);
    float3 ttop = invR * (boxmax - r.o);

    // re-order intersections to find smallest and largest on each axis
    float3 tmin = fminf(ttop, tbot);
    float3 tmax = fmaxf(ttop, tbot);

    // find the largest tmin and the smallest tmax
    float largest_tmin = fmaxf(fmaxf(tmin.x, tmin.y), fmaxf(tmin.x, tmin.z));
    float smallest_tmax = fminf(fminf(tmax.x, tmax.y), fminf(tmax.x, tmax.z));

    *tnear = largest_tmin;
    *tfar = smallest_tmax;

    return smallest_tmax > largest_tmin;
}

// transform vector by matrix (no translation)
__device__
float3 mul(const float3x4 &M, const float3 &v)
{
    float3 r;
    r.x = dot(v, make_float3(M.m[0]));
    r.y = dot(v, make_float3(M.m[1]));
    r.z = dot(v, make_float3(M.m[2]));
    return r;
}

// transform vector by matrix with translation
__device__
float4 mul(const float3x4 &M, const float4 &v)
{
    float4 r;
    r.x = dot(v, M.m[0]);
    r.y = dot(v, M.m[1]);
    r.z = dot(v, M.m[2]);
    r.w = 1.0f;
    return r;
}

__device__ uint rgbaFloatToInt(float4 rgba)
{
    rgba.x = __saturatef(rgba.x);   // clamp to [0.0, 1.0]
    rgba.y = __saturatef(rgba.y);
    rgba.z = __saturatef(rgba.z);
    rgba.w = __saturatef(rgba.w);
    return (uint(rgba.w*255)<<24) | (uint(rgba.z*255)<<16) | (uint(rgba.y*255)<<8) | uint(rgba.x*255);
}

__global__ void
d_render(uint *d_output, uint imageW, uint imageH,
         float density, float brightness,
         float transferOffset, float transferScale, float zCutoff, bool grayscale, float3 boxMin, float3 boxMax, cudaExtent volumeSize, unsigned char* gpuInOut, unsigned char* xNorm, unsigned char* yNorm, unsigned char* zNorm)
{
    const int maxSteps = 1000;
    const float tstep = 0.005f;
    const float opacityThreshold = 0.95f;
    //const float3 boxMin = make_float3(-1.0f, -1.0f, -1.0f);
    //const float3 boxMax = make_float3(1.0f, 1.0f, 1.0f);
	
	
    uint x = blockIdx.x*blockDim.x + threadIdx.x;
    uint y = blockIdx.y*blockDim.y + threadIdx.y;

    if ((x >= imageW) || (y >= imageH)) return;
	float aspectRatio = imageH*1.0 / imageW;

    float u = (x / (float) imageW)*2.0f-1.0f;
	float v = ((y / (float)imageH)*2.0f - 1.0f) * aspectRatio;
	
    // calculate eye ray in world space
    Ray eyeRay;

	// Parallel Projection
	//eyeRay.o = make_float3(mul(c_invViewMatrix, make_float4(u, v, 4.0f, 1.0f)));
	// eyeRay.d = normalize(make_float3(0, 0, -1.0f));


	// Perspective Projection
	// z-component of direction angle of perspective
	//eyeRay.o = make_float3(mul(c_invViewMatrix, make_float4(0, 0, 8.0f, 1.0f)));
	eyeRay.o = make_float3(mul(c_invViewMatrix, make_float4(0, 0, 2.0f, 1.0f)));
	eyeRay.d = normalize(make_float3(u, v, -2.0f));

	eyeRay.d = mul(c_invViewMatrix, eyeRay.d);

    // find intersection with box
    float tnear, tfar;
    int hit = intersectBox(eyeRay, boxMin, boxMax, &tnear, &tfar);

    if (!hit)
	{	
		//float4 sum = make_float4(1.0f,1.0f,1.0f,1.0f);

	    // write output color
		//d_output[y*imageW + x] = rgbaFloatToInt(sum);

		return;
	}

    if (tnear < 0.0f) tnear = 0.0f;     // clamp to near plane

    // march along ray from front to back, accumulating color
    float4 sum = make_float4(0.0f);

    float t = tnear;
    float3 pos = eyeRay.o + eyeRay.d*tnear;
    float3 step = eyeRay.d*tstep;
	
	// ------------------ x, y, z ratios ----------------
	float3 Maxes = boxMax;
	float xDenom = 2 * Maxes.x;
	float yDenom = 2 * Maxes.y;
	float zDenom = 2 * Maxes.z;

	float data = 0;
	float data2 = 0;
	float data3 = 0;
	float data4 = 0;

	int numDivX = volumeSize.width;
	int numDivY = volumeSize.height;
	int numDivZ = volumeSize.depth;



    for (int i=0; i<maxSteps; i++)
    {
        // read from 3D texture
        // remap position to [0, 1] coordinates
		if (pos.x*1.0 / xDenom + 0.5f >= 0 && pos.x*1.0 / xDenom + 0.5f <= 1)
		//if (pos.x*0.5 + 0.5f >= 0 && pos.x*0.5 + 0.5f <= 1)
		{
#ifdef TEXTURE
			// Set Textures to go through the correct ranges
			data = tex3D(tex, pos.x*1.0 / xDenom + 0.5f, pos.y*1.0 / yDenom + 0.5f, pos.z*1.0 / zDenom + 0.5f);
			data2 = tex3D(tex2, pos.x*1.0 / xDenom + 0.5f, pos.y*1.0 / yDenom + 0.5f, pos.z*1.0 / zDenom + 0.5f);
			data3 = tex3D(tex3, pos.x*1.0 / xDenom + 0.5f, pos.y*1.0 / yDenom + 0.5f, pos.z*1.0 / zDenom + 0.5f);
			data4 = tex3D(tex4, pos.x*1.0 / xDenom + 0.5f, pos.y*1.0 / yDenom + 0.5f, pos.z*1.0 / zDenom + 0.5f);

			/*data2 = tex3D(tex2, pos.x*0.5 + 0.5f, pos.y*0.5 + 0.5f, pos.z*0.5 + 0.5f);
			data3 = tex3D(tex3, pos.x*0.5 + 0.5f, pos.y*0.5 + 0.5f, pos.z*0.5 + 0.5f);
			data4 = tex3D(tex4, pos.x*0.5 + 0.5f, pos.y*0.5 + 0.5f, pos.z*0.5 + 0.5f);*/
#else
			// ------------------- Use this for Indexing -----------------------------
			int x = (pos.x + 1.0)/2.0 * numDivX;
			int y = (pos.y + 1.0)/2.0 * numDivY;
			int z = (pos.z + 1.0)/2.0 * numDivZ;
			
			int index = z*(numDivY)*(numDivX)+y*(numDivX)+x;
			data = gpuInOut[index];
			data2 = xNorm[index];
			data3 = yNorm[index];
			data4 = zNorm[index];
			// ------------------------------------------------------------------------
#endif
		}
		

		// --------------------- Convert x, y, z to range -1 to 1 ------------------
#ifdef TEXTURE
		float sample2 = (data2 - 0.5) * 2;
		float sample3 = (data3 - 0.5) * 2;
		float sample4 = (data4 - 0.5) * 2;
#else
		float dataX = data2;
		float dataY = data3;
		float dataZ = data4;
		// ---------------------------- 3 more Samples for X, Y, Z ---------------------------
		float sample2 = (dataX / 127.5) - 1.0;
		float sample3 = (dataY / 127.5) - 1.0;
		float sample4 = (dataZ / 127.5) - 1.0;
		// ------------------------------------------------------------------------
#endif
		// ------------------------------------------------------------------------
#ifdef VOLRENDER
		float sample = data;
#else
		float sample = data;
#endif
		
		if (pos.z < zCutoff)
		{
			sample = 0;
		}
        //sample *= 64.0f;    // scale for 10-bit data

        // lookup in transfer function texture
        float4 col = tex1D(transferTex, (sample-transferOffset-0.24)*transferScale);
		//float4 col = tex1D(transferTex, (sample - transferOffset)*transferScale);
        col.w *= density;
		if (grayscale)
			col.x = col.y = col.z = col.w;

        // "under" operator for back-to-front blending
        //sum = lerp(sum, col, col.w);

        // pre-multiply alpha
        col.x *= col.w;
        col.y *= col.w;
        col.z *= col.w;
        // "over" operator for front-to-back blending
#ifdef VOLRENDER
        sum = sum + col*(1.0f - sum.w);
		if (sample > 0.5)
			sum = make_float4(1, 0, 0, 1);
#endif
		// --------------------- Local Light ----------------------------------
		float3 lightPosition = make_float3(mul(c_invViewMatrix, make_float4(0, 5.0f, 1.0f, 1.0f)));
		float4 globalAmbient = { 1.0, 0.2, 0.2, 1.0 };
		float4 ka = { 0.2, 0.5, 0.0, 1.0 };
		float4 kd = { 0.2, 0.5, 0.0, 1.0 };
		float4 ks = { 0.39, 0.27, 0.17, 1.0 };
		float4 lightColor = { 1.0, 0.0, 0.0, 1.0 };
		float shininess = 1;

		// ------- Light Constants --------
		float4 emissive = { 0.0, 0.0, 0.0, 1.0 };
		float4 ambient = ka * globalAmbient;

		// ----------------- Compute Lighting -----------------------
		float3 normal = make_float3(sample2, sample3, sample4);	
		float3 L = normalize(lightPosition - pos);
		float diffuseLight = fmaxf(dot(normal, L), 0.0);
		float4 diffuse = kd * lightColor * diffuseLight;

		float3 V = normalize(eyeRay.o - pos);
		float3 H = normalize(L + V);
		float specularLight = pow(fmaxf(dot(normal, H), 0.0), shininess);
		if (diffuseLight <= 0)
			specularLight = 0;
		float4 specular = ks * lightColor * specularLight;
		float4 color = emissive + ambient + diffuse + specular;
		
		// ------------------------ End Lighting ----------------------------------------

#ifndef VOLRENDER
#ifdef TEXTURE
		if (sample > 0.5)
			sum = color;
#else
		if (sample >= 127.5)
			sum = color;
#endif
		
#endif
		//if (sample > 1.0 / 127.0 || sample < -1.0 / 127.0)
		//	sum = make_float4(1, 0, 0, 1);
	
        // exit early if opaque
        if (sum.w > opacityThreshold)
            break;
		

        t += tstep;

        if (t > tfar) break;

        pos += step;
    }

    sum *= brightness;

	//if (sum.w < 0.3)
	//if (sum.x < 0.1 && sum.y < 0.1 && sum.z < 0.1 && sum.w < 0.1)
	//	sum = make_float4(1.0f,1.0f,1.0f,1.0f);

	//sum = make_float4(1.0f,1.0f,1.0f,1.0f) - sum;

    // write output color
    d_output[y*imageW + x] = rgbaFloatToInt(sum);
	//d_output[y*imageW + x] = rgbaFloatToInt(make_float4(1.0, 1.0, 1.0, 1.0));
	
}

extern "C"
void setTextureFilterMode(bool bLinearFilter)
{
    tex.filterMode = bLinearFilter ? cudaFilterModeLinear : cudaFilterModePoint;
}

extern "C"
void initCuda(void *h_volume, void *h_volume2, void *h_volume3, void *h_volume4, cudaExtent volumeSize)
{
    // create 3D array
    cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<VolumeType>();			// Creates CUDA channel format descriptor
#ifdef TEXTURE
	// ------------------------------ 4 cudaMallocs for 3D textures --------------
	checkCudaErrors(cudaMalloc3DArray(&d_volumeArray, &channelDesc, volumeSize));		// Create 3D array for inout data
	checkCudaErrors(cudaMalloc3DArray(&d_volumeArray2, &channelDesc, volumeSize));
	checkCudaErrors(cudaMalloc3DArray(&d_volumeArray3, &channelDesc, volumeSize));
	checkCudaErrors(cudaMalloc3DArray(&d_volumeArray4, &channelDesc, volumeSize));
	// ---------------------------------------------------------------------------

	// copy data to 3D array
	cudaMemcpy3DParms copyParams = {0};
	copyParams.srcPtr   = make_cudaPitchedPtr(h_volume, volumeSize.width*sizeof(VolumeType), volumeSize.width, volumeSize.height);
	copyParams.dstArray = d_volumeArray;
	copyParams.extent   = volumeSize;
	copyParams.kind     = cudaMemcpyHostToDevice;
	checkCudaErrors(cudaMemcpy3D(&copyParams));

	// ------------------------------ 3 copyParams for Normals-------------------------------------
	cudaMemcpy3DParms copyParams2 = { 0 };
	copyParams2.srcPtr = make_cudaPitchedPtr((void*)h_volume2, volumeSize.width*sizeof(VolumeType), volumeSize.width, volumeSize.height);
	copyParams2.dstArray = d_volumeArray2;
	copyParams2.extent = volumeSize;
	copyParams2.kind = cudaMemcpyHostToDevice;
	checkCudaErrors(cudaMemcpy3D(&copyParams2));

	cudaMemcpy3DParms copyParams3 = { 0 };
	copyParams3.srcPtr = make_cudaPitchedPtr((void*)h_volume3, volumeSize.width*sizeof(VolumeType), volumeSize.width, volumeSize.height);
	copyParams3.dstArray = d_volumeArray3;
	copyParams3.extent = volumeSize;
	copyParams3.kind = cudaMemcpyHostToDevice;
	checkCudaErrors(cudaMemcpy3D(&copyParams3));

	cudaMemcpy3DParms copyParams4 = { 0 };
	copyParams4.srcPtr = make_cudaPitchedPtr((void*)h_volume4, volumeSize.width*sizeof(VolumeType), volumeSize.width, volumeSize.height);
	copyParams4.dstArray = d_volumeArray4;
	copyParams4.extent = volumeSize;
	copyParams4.kind = cudaMemcpyHostToDevice;
	checkCudaErrors(cudaMemcpy3D(&copyParams4));
	// ---------------------------------------------------------------------------------------

	// set texture parameters
	tex.normalized = true;                      // access with normalized texture coordinates
	tex.filterMode = cudaFilterModeLinear;      // linear interpolation
	tex.addressMode[0] = cudaAddressModeClamp;  // clamp texture coordinates
	tex.addressMode[1] = cudaAddressModeClamp;

	// ---------------------- 3 more set texture parameters -----------------------------
	tex2.normalized = true;
	tex2.filterMode = cudaFilterModeLinear;
	tex2.addressMode[0] = cudaAddressModeClamp;
	tex2.addressMode[1] = cudaAddressModeClamp;

	tex3.normalized = true;
	tex3.filterMode = cudaFilterModeLinear;
	tex3.addressMode[0] = cudaAddressModeClamp;
	tex3.addressMode[1] = cudaAddressModeClamp;

	tex4.normalized = true;
	tex4.filterMode = cudaFilterModeLinear;
	tex4.addressMode[0] = cudaAddressModeClamp;
	tex4.addressMode[1] = cudaAddressModeClamp;
	// -------------------------------------------------------------------------------------

	// bind array to 3D texture
	checkCudaErrors(cudaBindTextureToArray(tex, d_volumeArray, channelDesc));
	// -------------------- Bind 3 more arrays --------------------------------
	checkCudaErrors(cudaBindTextureToArray(tex2, d_volumeArray2, channelDesc));
	checkCudaErrors(cudaBindTextureToArray(tex3, d_volumeArray3, channelDesc));
	checkCudaErrors(cudaBindTextureToArray(tex4, d_volumeArray4, channelDesc));
#else	
	// --------------------------- Array cudaMallocs -----------------------------
	checkCudaErrors(cudaMalloc((void**)&gpuArray, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char)));				
	checkCudaErrors(cudaMalloc((void**)&xArray, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char)));
	checkCudaErrors(cudaMalloc((void**)&yArray, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char)));
	checkCudaErrors(cudaMalloc((void**)&zArray, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char)));
	// --------------------------------------------------------------------------

	// ------------------- Array cudaMemcpys ----------------------
	checkCudaErrors(cudaMemcpy(gpuArray, h_volume, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(xArray, h_volume2, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(yArray, h_volume3, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char), cudaMemcpyHostToDevice));
	checkCudaErrors(cudaMemcpy(zArray, h_volume4, volumeSize.depth * volumeSize.width * volumeSize.height * sizeof(unsigned char), cudaMemcpyHostToDevice));
	// -------------------------------------------------------------
#endif
   
	// ------------------------------------------------------------------------

    // create transfer function texture
    float4 transferFunc[] =
    {
        {  0.0, 0.0, 0.0, 0.0, },
        {  1.0, 0.0, 0.0, 1.0, },
        {  1.0, 0.5, 0.0, 1.0, },
        {  1.0, 1.0, 0.0, 1.0, },
        {  0.0, 1.0, 0.0, 1.0, },
        {  0.0, 1.0, 1.0, 1.0, },
        {  0.0, 0.0, 1.0, 1.0, },
        {  1.0, 0.0, 1.0, 1.0, },
        {  0.0, 0.0, 0.0, 0.0, },
    };

    cudaChannelFormatDesc channelDesc2 = cudaCreateChannelDesc<float4>();
    cudaArray *d_transferFuncArray;
    checkCudaErrors(cudaMallocArray(&d_transferFuncArray, &channelDesc2, sizeof(transferFunc)/sizeof(float4), 1));
    checkCudaErrors(cudaMemcpyToArray(d_transferFuncArray, 0, 0, transferFunc, sizeof(transferFunc), cudaMemcpyHostToDevice));

    transferTex.filterMode = cudaFilterModeLinear;
    transferTex.normalized = true;    // access with normalized texture coordinates
    transferTex.addressMode[0] = cudaAddressModeClamp;   // wrap texture coordinates

    // Bind the array to the texture
    checkCudaErrors(cudaBindTextureToArray(transferTex, d_transferFuncArray, channelDesc2));
}

extern "C"
void freeCudaBuffers()
{
#ifdef TEXTURE
    checkCudaErrors(cudaFreeArray(d_volumeArray));
	// ---------------------------------------
	checkCudaErrors(cudaFreeArray(d_volumeArray2));
	checkCudaErrors(cudaFreeArray(d_volumeArray3));
	checkCudaErrors(cudaFreeArray(d_volumeArray4));
	// --------------------------------------
#endif
    checkCudaErrors(cudaFreeArray(d_transferFuncArray));
}


extern "C"
void render_kernel(dim3 gridSize, dim3 blockSize, uint *d_output, uint imageW, uint imageH,
                   float density, float brightness, float transferOffset, float transferScale, float zCutoff, bool grayscale, float3 boxMin, float3 boxMax, cudaExtent volumeSize)
{
    d_render<<<gridSize, blockSize>>>(d_output, imageW, imageH, density,
                                      brightness, transferOffset, transferScale, zCutoff, grayscale, boxMin, boxMax, volumeSize, gpuArray, xArray, yArray, zArray);
}

extern "C"
void copyInvViewMatrix(float *invViewMatrix, size_t sizeofMatrix)
{
    checkCudaErrors(cudaMemcpyToSymbol(c_invViewMatrix, invViewMatrix, sizeofMatrix));
}


#endif // #ifndef _VOLUMERENDER_KERNEL_CU_
