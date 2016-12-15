/*
    Volume rendering sample

    This sample loads a 3D volume from disk and displays it using
    ray marching and 3D textures.

    Note - this is intended to be an example of using 3D textures
    in CUDA, not an optimized volume renderer.

    Changes
    sgg 22/3/2010
    - updated to use texture for display instead of glDrawPixels.
    - changed to render from front-to-back rather than back-to-front.
*/

// OpenGL Graphics includes
#include <GL/glew.h>
#if defined (__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif
//#define TEXTURE

//#include <GL/glfw3.h>
// CUDA Runtime, Interop, and includes
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <vector_types.h>
#include <vector_functions.h>
#include <driver_functions.h>

// CUDA utilities
#include <helper_cuda.h>
#include <helper_cuda_gl.h>

// Helper functions
#include <helper_cuda.h>
#include <helper_functions.h>
#include <helper_timer.h>

#include <math.h>
#define pi 3.1415926 //in order to use sin&cos function

typedef unsigned int uint;
typedef unsigned char uchar;

#define MAX_EPSILON_ERROR 5.00f
#define THRESHOLD         0.30f

// Define the files that are to be save and the reference images for validation
const char *sOriginal[] =
{
    "volume.ppm",
    NULL
};

const char *sReference[] =
{
    "ref_volume.ppm",
    NULL
};

const char *sSDKsample = "CUDA 3D Volume Render";

//const char *volumeFilename = "DTzComp.raw";
//cudaExtent volumeSize = make_cudaExtent(100, 100, 19);
//typedef unsigned char VolumeType;

//const char *volumeFilename = "QuadVA.raw";
//cudaExtent volumeSize = make_cudaExtent(92, 92, 136);
//typedef unsigned char VolumeType;

//const char *volumeFilename = "BiV2CT.raw";
//cudaExtent volumeSize = make_cudaExtent(300, 300, 150);
//typedef unsigned char VolumeType;

//const char *volumeFilename = "BiV2CTChest.raw";
//cudaExtent volumeSize = make_cudaExtent(512, 512, 216);
//typedef unsigned char VolumeType;

//const char *volumeFilename = "RatLV.raw";
//cudaExtent volumeSize = make_cudaExtent(576, 426, 426);
//typedef unsigned char VolumeType;


//const char *volumeFilename = "Hammer.raw";

// ------------------- 4 Normal Files --------------------------------
const char *xNormals = "xNormals.raw";
const char *yNormals = "yNormals.raw";
const char *zNormals = "zNormals.raw";
const char *inouts = "inouts.raw";
// -----------------------------------------------------------------
const int sizeX = 156;
const int sizeY = 212;
const int sizeZ = 300;

cudaExtent volumeSize = make_cudaExtent(sizeX, sizeY, sizeZ);
typedef  unsigned char VolumeType;

uint width = 800, height = 800;
dim3 blockSize(16, 16);
dim3 gridSize;
bool autoRotate = false;
bool autoRotatex = false;
float clipPlaneDistZ = -1.0;

float invViewMatrix[12];
float viewMatrix[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

float density = 0.1f;
float brightness = 1.0f;
float transferOffset = 0.0f;
float transferScale = 1.0f;
bool linearFiltering = true;
bool grayscale = false;

GLuint pbo = 0;     // OpenGL pixel buffer object
GLuint tex = 0;     // OpenGL texture object
struct cudaGraphicsResource *cuda_pbo_resource; // CUDA Graphics Resource (to transfer PBO)

StopWatchInterface *timer = 0;

// Auto-Verification Code
const int frameCheckNumber = 2;
int fpsCount = 0;        // FPS count for averaging
int fpsLimit = 1;        // FPS limit for sampling
int g_Index = 0;
unsigned int frameCount = 0;

int *pArgc;
char **pArgv;

#ifndef MAX
#define MAX(a,b) ((a > b) ? a : b)
#endif

extern "C" void setTextureFilterMode(bool bLinearFilter);
extern "C" void initCuda(void *h_volume, void *h_volume2, void *h_volume3, void *h_volume4, cudaExtent volumeSize);
extern "C" void freeCudaBuffers();
extern "C" void render_kernel(dim3 gridSize, dim3 blockSize, uint *d_output, uint imageW, uint imageH,
                              float density, float brightness, float transferOffset, float transferScale, float zCutoff, bool grayscale, float3 boxMin, float3 boxMax, cudaExtent volumeSize);
extern "C" void copyInvViewMatrix(float *invViewMatrix, size_t sizeofMatrix);

void initPixelBuffer();

void computeFPS()
{
    frameCount++;
    fpsCount++;

    if (fpsCount == fpsLimit)
    {
        char fps[256];
        float ifps = 1.f / (sdkGetAverageTimerValue(&timer) / 1000.f);
        sprintf(fps, "GPVol: %3.1f fps", ifps);

        glutSetWindowTitle(fps);
        fpsCount = 0;

        fpsLimit = (int)MAX(1.f, ifps);
        sdkResetTimer(&timer);
    }
}

// render image using CUDA
void render()
{
    copyInvViewMatrix(invViewMatrix, sizeof(float4)*3);
	
    // map PBO to get CUDA device pointer
    uint *d_output;
    // map PBO to get CUDA device pointer
    checkCudaErrors(cudaGraphicsMapResources(1, &cuda_pbo_resource, 0));
    size_t num_bytes;
    checkCudaErrors(cudaGraphicsResourceGetMappedPointer((void **)&d_output, &num_bytes,
                                                         cuda_pbo_resource));
    //printf("CUDA mapped PBO: May access %ld bytes\n", num_bytes);

    // clear image
    checkCudaErrors(cudaMemset(d_output, 0, width*height*4));

	
#ifdef TEXTURE
	float xRatio;
	float yRatio;
	float zRatio;
	// -------------- set boxMin and boxMax values ------------------
	if (sizeX <= sizeY && sizeX <= sizeZ)		// Find which dimension to set as 1 and set others relative to it
	{
		xRatio = float((1.0*sizeX) / (1.0*sizeX));
		yRatio = float((1.0*sizeY) / (1.0*sizeX));
		zRatio = float((1.0*sizeZ) / (1.0*sizeX));
	}
	else if (sizeY <= sizeX && sizeY <= sizeZ)
	{
		xRatio = float((1.0*sizeX) / (1.0*sizeY));
		yRatio = float((1.0*sizeY) / (1.0*sizeY));
		zRatio = float((1.0*sizeZ) / (1.0*sizeY));
	}
	else
	{
		xRatio = float((1.0*sizeX) / (1.0*sizeZ));
		yRatio = float((1.0*sizeY) / (1.0*sizeZ));
		zRatio = float((1.0*sizeZ) / (1.0*sizeZ));
	}

	float3 boxMin = make_float3(-xRatio, -yRatio, -zRatio);
	float3 boxMax = make_float3(xRatio, yRatio, zRatio);
	clipPlaneDistZ = -zRatio;
#else
	float3 boxMin = make_float3(-1.0f, -1.0f, -1.0f);
	float3 boxMax = make_float3(1.0f, 1.0f, 1.0f);
	clipPlaneDistZ = -1.0;
#endif
	
	
    // call CUDA kernel, writing results to PBO
    render_kernel(gridSize, blockSize, d_output, width, height, density, brightness, transferOffset, transferScale, clipPlaneDistZ, grayscale, boxMin, boxMax, volumeSize);

    getLastCudaError("kernel failed");

    checkCudaErrors(cudaGraphicsUnmapResources(1, &cuda_pbo_resource, 0));
}

// display results using OpenGL (called by GLUT)
void display()
{
    sdkStartTimer(&timer);

	// use OpenGL to build view matrix
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	GLfloat modelView[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, modelView);
	glPopMatrix();
	
	invViewMatrix[0] = modelView[0];
	invViewMatrix[1] = modelView[1];
	invViewMatrix[2] = modelView[2];
	invViewMatrix[3] = modelView[12];
	invViewMatrix[4] = modelView[4];
	invViewMatrix[5] = modelView[5];
	invViewMatrix[6] = modelView[6];
	invViewMatrix[7] = modelView[13];
	invViewMatrix[8] = modelView[8];
	invViewMatrix[9] = modelView[9];
	invViewMatrix[10] = modelView[10];
	invViewMatrix[11] = modelView[14];
	
	render();

    // display results
	glClear(GL_COLOR_BUFFER_BIT);

    // draw image from PBO
    glDisable(GL_DEPTH_TEST);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
#if 0
    // draw using glDrawPixels (slower)
    glRasterPos2i(0, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glDrawPixels(width, height, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);
#else
    // draw using texture

    // copy from pbo to texture
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

	glPushMatrix();
	glLoadIdentity();
    // draw textured quad
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f(1, 0);
    glVertex2f(1, 0);
    glTexCoord2f(1, 1);
    glVertex2f(1, 1);
    glTexCoord2f(0, 1);
    glVertex2f(0, 1);
    glEnd();
	glPopMatrix();

    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
#endif

    glutSwapBuffers();
    glutReportErrors();

    sdkStopTimer(&timer);

    computeFPS();
}

void idle()
{
	if (autoRotate)
	{
	    glutPostRedisplay();
	}

	else if (autoRotatex)
	{
		float dx = 1;
		float dy = 0;
		glMatrixMode(GL_MODELVIEW);
		float viewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);	// get the viewMatrix
		float3 localAxis = make_float3(dy, dx, 0);
		glLoadIdentity();
		glRotatef(sqrt(dy*dy + dx*dx), localAxis.x, localAxis.y, localAxis.z);
		glMultMatrixf(viewMatrix);
		glutPostRedisplay();
	}
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:
            exit(EXIT_SUCCESS);
            break;

        case 'f':
        case 'F':
            linearFiltering = !linearFiltering;
            setTextureFilterMode(linearFiltering);
            break;

        case '+':
        case '=':
            density += 0.001f;
            break;

        case '-':
        case '_':
            density -= 0.001f;
            break;

        case ']':
            brightness += 0.05f;
            break;

        case '[':
            brightness -= 0.05f;
            break;

        case ';':
            transferOffset += 0.001f;
            break;

        case '\'':
            transferOffset -= 0.001f;
            break;

        case '.':
            transferScale += 0.01f;
            break;

        case ',':
            transferScale -= 0.01f;
            break;

		case 'a':
		case 'A':
			autoRotate = !autoRotate;
			break;

		case 'b':
		case 'B':
			autoRotatex = !autoRotatex;
			break;

		case 'g':
		case 'G':
			grayscale = !grayscale;
			break;

		case 'q':
		case 'Q':
			clipPlaneDistZ += 0.05f;
			break;

		case 'w':
		case 'W':
			clipPlaneDistZ -= 0.05f;
			break;

        default:
            break;
    }

    //printf("density = %.2f, brightness = %.2f, transferOffset = %.2f, transferScale = %.2f\n", density, brightness, transferOffset, transferScale);
    glutPostRedisplay();
}

int clickX, clickY;
int buttonState = 0;
//	0	Up
//	1	Left Button
//	2	Right Button
//	3	Wheel Up
//	4	Wheel Down
//	5	Ctrl + Left Button
//	6	Shift + Left Button
//	7	Alt + Left Button

void mouseClick(int button, int state, int x, int y)
{
	int modifier = glutGetModifiers();
    if (state == GLUT_DOWN)
    {
		if (button == GLUT_LEFT_BUTTON && modifier == GLUT_ACTIVE_CTRL)
			buttonState = 5;
		else if (button == GLUT_LEFT_BUTTON && modifier == GLUT_ACTIVE_SHIFT)
			buttonState = 6;
		else if (button == GLUT_LEFT_BUTTON && modifier == GLUT_ACTIVE_ALT)
			buttonState = 7; 
		else if (button == GLUT_LEFT_BUTTON)
			buttonState = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			buttonState = 2;
		else if (button == 3)
			buttonState = 3;
		else if (button == 4)
			buttonState = 4;
	}
    else if (state == GLUT_UP)
    {
        buttonState = 0;
    }
	
	if (buttonState == 3 || buttonState == 4)  // Zoom
	{
		float2 midPoint;
		midPoint.x = width / 2.0f;
		midPoint.y = height / 2.0f;

		glMatrixMode(GL_MODELVIEW);
		float viewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);	// get the viewMatrix

		float scale = 1.0;
		if (buttonState == 4)
			scale = 0.926f;
		else
			scale = 1.080f;

		density *= scale;
		float2 move;
		move.x = (x - midPoint.x)*((1.0f - 1.0f / scale) / 100.0f);
		move.y = (y - midPoint.y)*((1.0f - 1.0f / scale) / 100.0f);

		glLoadIdentity();
		glScaled(scale, scale, scale);
		//glTranslated(-move.x, move.y, 0);
		glMultMatrixf(viewMatrix);
	}

	clickX = x;
	clickY = y;
    glutPostRedisplay();
}

void mouseMove(int x, int y)
{
	float dx, dy;
	dx = (float)(x - clickX);
	dy = (float)(y - clickY);
	if (buttonState == 1)	// Rotation
	{
		glMatrixMode(GL_MODELVIEW);
		float viewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);	// get the viewMatrix
		float3 localAxis = make_float3(dy, dx, 0);
		glLoadIdentity();
		glRotatef(sqrt(dy*dy+dx*dx), localAxis.x, localAxis.y, localAxis.z);
		glMultMatrixf(viewMatrix);
	}
	else if (buttonState == 2)	// Pan
	{
		glMatrixMode(GL_MODELVIEW);
		float viewMatrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, viewMatrix);	// get the viewMatrix
		float3 localTranslate = make_float3(-2*dx / width, 2*dy / height, 0);
		glLoadIdentity();
		glTranslatef(localTranslate.x, localTranslate.y, localTranslate.z);
		glMultMatrixf(viewMatrix);
	}
	clickX = x;
	clickY = y;
	if (buttonState != 0)
		glutPostRedisplay();
}

int iDivUp(int a, int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

void reshape(int w, int h)
{
    width = w;
    height = h;
    initPixelBuffer();

	glViewport(0, 0, width, height);

    // calculate new grid size
    gridSize = dim3(iDivUp(width, blockSize.x), iDivUp(height, blockSize.y));
}

void initGL()
{
#ifndef TEXTURE
	float xRatio;
	float yRatio;
	float zRatio;
	// -------------- set boxMin and boxMax values ------------------
	if (sizeX <= sizeY && sizeX <= sizeZ)		// Find which dimension to set as 1 and set others relative to it
	{
		xRatio = float((1.0*sizeX) / (1.0*sizeX));
		yRatio = float((1.0*sizeY) / (1.0*sizeX));
		zRatio = float((1.0*sizeZ) / (1.0*sizeX));
	}
	else if (sizeY <= sizeX && sizeY <= sizeZ)
	{
		xRatio = float((1.0*sizeX) / (1.0*sizeY));
		yRatio = float((1.0*sizeY) / (1.0*sizeY));
		zRatio = float((1.0*sizeZ) / (1.0*sizeY));
	}
	else
	{
		xRatio = float((1.0*sizeX) / (1.0*sizeZ));
		yRatio = float((1.0*sizeY) / (1.0*sizeZ));
		zRatio = float((1.0*sizeZ) / (1.0*sizeZ));
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glScalef(4/xRatio, 4/yRatio, 4/zRatio);
#else
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glScalef(4, 4, 4 );
#endif
	//glTranslatef(0, 0, 4);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 1.0, 0.0, 1.0, -10.0, 10.0);
}


void cleanup()
{
    sdkDeleteTimer(&timer);

    freeCudaBuffers();

    if (pbo)
    {
        cudaGraphicsUnregisterResource(cuda_pbo_resource);
        glDeleteBuffersARB(1, &pbo);
        glDeleteTextures(1, &tex);
    }
}

void initGL(int *argc, char **argv)
{
    // initialize GLUT callback functions
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutCreateWindow("CUDA volume rendering");
	
	
    glewInit();

    if (!glewIsSupported("GL_VERSION_2_0 GL_ARB_pixel_buffer_object"))
    {
        printf("Required OpenGL extensions missing.");
        exit(EXIT_SUCCESS);
    }
}

void initPixelBuffer()
{
    if (pbo)
    {
        // unregister this buffer object from CUDA C
        checkCudaErrors(cudaGraphicsUnregisterResource(cuda_pbo_resource));

        // delete old buffer
        glDeleteBuffersARB(1, &pbo);
        glDeleteTextures(1, &tex);
    }

    // create pixel buffer object for display
    glGenBuffersARB(1, &pbo);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, pbo);
    glBufferDataARB(GL_PIXEL_UNPACK_BUFFER_ARB, width*height*sizeof(GLubyte)*4, 0, GL_STREAM_DRAW_ARB);
    glBindBufferARB(GL_PIXEL_UNPACK_BUFFER_ARB, 0);

    // register this buffer object with CUDA
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_pbo_resource, pbo, cudaGraphicsMapFlagsWriteDiscard));

    // create texture for display
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

// Load raw data from disk
void *loadRawFile(char *filename, size_t size)
{
    FILE *fp = fopen(filename, "rb");

    if (!fp)
    {
        fprintf(stderr, "Error opening file '%s'\n", filename);
        return 0;
    }

    void *data = malloc(size);
    size_t read = fread(data, 1, size, fp);
    fclose(fp);

    printf("Read '%s', %d bytes\n", filename, read);

    return data;
}

// Load raw data from disk
void *loadRawTextFile(char *filename, size_t size)
{
    FILE *fp = fopen(filename, "r");

    if (!fp)
    {
        fprintf(stderr, "Error opening file '%s'\n", filename);
        return 0;
    }

    void *data = malloc(size);
    size_t read = fread(data, 1, size, fp);
    fclose(fp);

    printf("Read '%s', %d bytes\n", filename, read);

    return data;
}
// General initialization call for CUDA Device
int chooseCudaDevice(int argc, const char **argv, bool bUseOpenGL)
{
    int result = 0;

    if (bUseOpenGL)
    {
        result = findCudaGLDevice(argc, argv);
    }
    else
    {
        result = findCudaDevice(argc, argv);
    }

    return result;
}

void runSingleTest(const char *ref_file, const char *exec_path)
{
    bool bTestResult = true;

    uint *d_output;
    checkCudaErrors(cudaMalloc((void **)&d_output, width*height*sizeof(uint)));
    checkCudaErrors(cudaMemset(d_output, 0, width*height*sizeof(uint)));

    float modelView[16] =
    {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 4.0f, 1.0f
    };

    invViewMatrix[0] = modelView[0];
    invViewMatrix[1] = modelView[4];
    invViewMatrix[2] = modelView[8];
    invViewMatrix[3] = modelView[12];
    invViewMatrix[4] = modelView[1];
    invViewMatrix[5] = modelView[5];
    invViewMatrix[6] = modelView[9];
    invViewMatrix[7] = modelView[13];
    invViewMatrix[8] = modelView[2];
    invViewMatrix[9] = modelView[6];
    invViewMatrix[10] = modelView[10];
    invViewMatrix[11] = modelView[14];

    // call CUDA kernel, writing results to PBO
    copyInvViewMatrix(invViewMatrix, sizeof(float4)*3);

    // Start timer 0 and process n loops on the GPU
    int nIter = 10;
	float3 boxMin = make_float3(-1.0f, -1.0f, -1.0f);
	float3 boxMax = make_float3(1.0f, 1.0f, 1.0f);
    for (int i = -1; i < nIter; i++)
    {
        if (i == 0)
        {
            cudaDeviceSynchronize();
            sdkStartTimer(&timer);
        }

        render_kernel(gridSize, blockSize, d_output, width, height, density, brightness, transferOffset, transferScale, clipPlaneDistZ, grayscale, boxMin, boxMax, volumeSize);
    }

    cudaDeviceSynchronize();
    sdkStopTimer(&timer);
    // Get elapsed time and throughput, then log to sample and master logs
    double dAvgTime = sdkGetTimerValue(&timer)/(nIter * 1000.0);
    printf("volumeRender, Throughput = %.4f MTexels/s, Time = %.5f s, Size = %u Texels, NumDevsUsed = %u, Workgroup = %u\n",
           (1.0e-6 * width * height)/dAvgTime, dAvgTime, (width * height), 1, blockSize.x * blockSize.y);


    getLastCudaError("Error: render_kernel() execution FAILED");
    checkCudaErrors(cudaDeviceSynchronize());

    unsigned char *h_output = (unsigned char *)malloc(width*height*4);
    checkCudaErrors(cudaMemcpy(h_output, d_output, width*height*4, cudaMemcpyDeviceToHost));

    sdkSavePPM4ub("volume.ppm", h_output, width, height);
    bTestResult = sdkComparePPM("volume.ppm", sdkFindFilePath(ref_file, exec_path), MAX_EPSILON_ERROR, THRESHOLD, true);

    cudaFree(d_output);
    free(h_output);
    cleanup();

    exit(bTestResult ? EXIT_SUCCESS : EXIT_FAILURE);
}

////////////////////////////////////////////////////////////////////////////////
// Program main
////////////////////////////////////////////////////////////////////////////////
int
main(int argc, char **argv)
{
    pArgc = &argc;
    pArgv = argv;

    char *ref_file = NULL;

    //start logs
    printf("%s Starting...\n\n", sSDKsample);

    if (checkCmdLineFlag(argc, (const char **)argv, "file"))
    {
        getCmdLineArgumentString(argc, (const char **)argv, "file", &ref_file);
        fpsLimit = frameCheckNumber;
    }

    if (ref_file)
    {
        // use command-line specified CUDA device, otherwise use device with highest Gflops/s
        chooseCudaDevice(argc, (const char **)argv, false);
    }
    else
    {
        // First initialize OpenGL context, so we can properly set the GL for CUDA.
        // This is necessary in order to achieve optimal performance with OpenGL/CUDA interop.
        initGL(&argc, argv);

        // use command-line specified CUDA device, otherwise use device with highest Gflops/s
        chooseCudaDevice(argc, (const char **)argv, true);
    }

    // parse arguments
    char *filename;

    if (getCmdLineArgumentString(argc, (const char **) argv, "volume", &filename))
    {
		// Set Volume Filename if required
    }

    int n;

    if (checkCmdLineFlag(argc, (const char **) argv, "size"))
    {
        n = getCmdLineArgumentInt(argc, (const char **) argv, "size");
        volumeSize.width = volumeSize.height = volumeSize.depth = n;
    }

    if (checkCmdLineFlag(argc, (const char **) argv, "xsize"))
    {
        n = getCmdLineArgumentInt(argc, (const char **) argv, "xsize");
        volumeSize.width = n;
    }

    if (checkCmdLineFlag(argc, (const char **) argv, "ysize"))
    {
        n = getCmdLineArgumentInt(argc, (const char **) argv, "ysize");
        volumeSize.height = n;
    }

    if (checkCmdLineFlag(argc, (const char **) argv, "zsize"))
    {
        n= getCmdLineArgumentInt(argc, (const char **) argv, "zsize");
        volumeSize.depth = n;
    }

    // load volume data
	// ----------------------------- 4 Normals Get Paths for each -------------------------------------
	char *xNormPath = sdkFindFilePath(xNormals, argv[0]);
	char *yNormPath = sdkFindFilePath(yNormals, argv[0]);
	char *zNormPath = sdkFindFilePath(zNormals, argv[0]);
	char *inoutPath = sdkFindFilePath(inouts, argv[0]);
	// ---------------------------------------------------------------------------------

	if (xNormPath == NULL || yNormPath == NULL || zNormPath == NULL || inoutPath == NULL)
    {
		printf("Error finding file '%s'\n", inoutPath);
        exit(EXIT_FAILURE);
    }

    size_t size = volumeSize.width*volumeSize.height*volumeSize.depth*sizeof(VolumeType);

	// ---------------------- 4 File Normals Load files ----------------------------------
	void *h_xnormals = loadRawFile(xNormPath, size);
	void *h_ynormals = loadRawFile(yNormPath, size);
	void *h_znormals = loadRawFile(zNormPath, size);
	void *h_inouts = loadRawFile(inoutPath, size);
	// ----------------------------------------------------------------------------------

	// ------------------------- 4 File InitCuda ---------------------------
	initCuda(h_inouts, h_xnormals, h_ynormals, h_znormals, volumeSize);
	free(h_xnormals);
	free(h_ynormals);
	free(h_znormals);
	free(h_inouts);
	// --------------------------------------------------------------------

    sdkCreateTimer(&timer);

    printf("Press '+' and '-' to change density (0.001 increments)\n"
           "      ']' and '[' to change brightness\n"
           "      ';' and ''' to modify transfer function offset\n"
           "      '.' and ',' to modify transfer function scale\n\n");

    // calculate new grid size
    gridSize = dim3(iDivUp(width, blockSize.x), iDivUp(height, blockSize.y));

    if (ref_file)
    {
        runSingleTest(ref_file, argv[0]);
    }
    else
    {
        // This is the normal rendering path for VolumeRender
        glutDisplayFunc(display);
        glutKeyboardFunc(keyboard);
        glutMouseFunc(mouseClick);
        glutMotionFunc(mouseMove);
        glutReshapeFunc(reshape);
        glutIdleFunc(idle);

        initPixelBuffer();
		initGL();

        atexit(cleanup);

        glutMainLoop();
    }

    cudaDeviceReset();
    exit(EXIT_SUCCESS);
}
