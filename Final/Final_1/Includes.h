//C++ Includes
#include <vector>
#include <list>
#include <fstream>
#include <cmath>
#include <time.h>
#include <sys/timeb.h>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <assert.h>

//Project Includes
#include "FloatVector.h"
#include "Timer.h"

//#define CPURENDER
#define USEFREEGLUT

#define CUDA
//#define CHECKCUDAERRORS
//#define CUDATEXTURES
#define THRUST

// disable 'possible loss of data' warnings on MSVC
#pragma warning(disable : 4244)
// disable 'truncation from double to float' warnings on MSVC
#pragma warning(disable : 4305)

#define ACISOBJ
//#define CONVERTTONURBS
#ifdef ACISOBJ
	#define BREP
//	#define TESSELLATE
#endif
//#define OBJFILE
//#define NURBSFILE

//#define COMPUTEMOMENTS
//#define COMPUTEDISTANCEMETRICS
//#define COMPUTEHAUSDORFFDISTANCE
//#define TWOSIDEDHAUSDORFF
//#define DYNAMICHD

//#define SURFACEHD
//#define DUCKYHD
//#define TEAPOTHD

//#define CPUEVAL
#define PACKED
//#define PINGPONG

#ifndef CPUEVAL
	#define VBO
#endif
#define DISPLAYLISTS
//#define DRAWFACEBBOX
//#define DRAWEDGES
//#define VISUALDEBUG
//#define MSAA

//#define OCCLUSIONFIND
//#define GPUFIND

// if GPUFIND is defined, define anyone of the following methods
// if GPUFIND is not defined the values are read back and read on the CPU
//#define GPUFIND1	// GPU with for loop
//#define GPUFIND2	// GPU without for loop
//#define GPUFIND3	// Stream reduction

// Polyline fitting algorithm : choose one
//#define POLYFIT1	// Merging polylines
//#define POLYFIT2	// Merging polylines
#define POLYFIT3	// Depth first search

#define STENCILBUFFERMETHOD
#define GPUCOLLISION
//#define INTERACTIVECOLLISION

#ifdef COMPUTEDISTANCEMETRICS
#ifdef BREP
	#define DYNAMICOBJECTMOVE
#else
	#define SURFACEMETRICS
#endif
#endif

//#define AUTOTIMING
//#define INTERACTIVETIMING

// Courtesy Google :-)
#define PI 3.14159265 
#define DELTA 1e-5

//Switches based on the graphics card 
#define NVIDIA
#ifdef NVIDIA
	#define RGBA_FLOAT_FORMAT GL_FLOAT_RGBA32_NV
	#define LUMINANCE_FLOAT_FORMAT GL_FLOAT_R32_NV
	#define FRAMEBUFFER_FLOAT_FORMAT GL_FLOAT_R32_NV
	#define FRAMEBUFFER_FORMAT GL_LUMINANCE
#else
	#define RGBA_FLOAT_FORMAT GL_RGBA_FLOAT32_ATI
	#define LUMINANCE_FLOAT_FORMAT GL_LUMINANCE_FLOAT32_ATI
	#define FRAMEBUFFER_FLOAT_FORMAT GL_RGBA_FLOAT32_ATI
	#define FRAMEBUFFER_FORMAT GL_RGBA
#endif

//Texture format
#define TRIM_TEXTURE_FORMAT GL_TEXTURE_RECTANGLE_ARB
//#define TRIM_TEXTURE_FORMAT GL_TEXTURE_2D

//GL Includes
#include <GL/glew.h>
#ifdef USEFREEGLUT
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glext.h>

//CG Includes
#include <cg/cgGL.h>

//CUDA Includes
#ifdef CUDA
#ifndef CPURENDER
//	#define VBO
#endif
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <vector_types.h>
#include <vector_functions.h>
#endif


#ifdef ACISOBJ
//ACIS Includes
#include "acis.hxx"					// Declares system wide parameters
#include "license.hxx"				// Licensing for R26
#include "spa_unlock_result.hxx"	// Licensing for R26
#include "cstrapi.hxx"				// Declares constructor API's
#include "kernapi.hxx"				// Declares kernel API's
#include "af_api.hxx"				// Declares faceter API's
#include "getbox.hxx"				// Declares get_*_box function
#include "box.hxx"					// Declares box class
#include "alltop.hxx"				// Declares topology classes
#include "position.hxx" 			// Declares position class
#include "vector.hxx"				// Declares vector class
#include "ppmeshmg.hxx" 			// Declares poly_point_mesh manager
#include "refine.hxx"				// Declares refinement class
#include "facet_options.hxx"		// Declares facet options class
#include "af_enum.hxx"				// Declares enum types
#include "boolapi.hxx"				// Declares boolean APIs,
#include "lists.hxx"    			// Declares ENTITY_LIST class
#include "fileinfo.hxx" 			// Declares fileinfo class
#include "splsurf.hxx"				// Declares spline surface class
#include "spline.hxx"				// Declares spline surface class
#include "spldef.hxx"				// Declares spline surface class
#include "bs3surf.hxx"				// Declares bs3 spline surface class
#include "sp3srtn.hxx"				// Declares bs3 spline surface class
#include "sps3srtn.hxx"				// Declares bs3 spline surface class
#include "rnd_api.hxx"				// Declares Color atrributes
#include "rgbcolor.hxx" 			// Declares Color atrributes
#include "pcudef.hxx"				// Declares pcurve class
#include "pcurve.hxx"				// Declares PCURVE class
#include "bs2curve.hxx"				// Declares bs2 spline curve class
#include "sps2crtn.hxx" 			// Declares bs2 spline curve class
#include "point.hxx"				// Declares point class
#include "cnv2spl_opts.hxx"			// Declares Convert to Spline options class
#include "intrapi.hxx"				// Declares bounding box calculation routines
#include "vector_utils.hxx"			// Declares distance utilites
#include "clearapi.hxx"				// Declares clearance functions
#include "mprop.hxx"				// Declares mass properties class
#endif

#define INOUTDFORMAT GL_BYTE
typedef GLbyte inOutDType;
using namespace std;
