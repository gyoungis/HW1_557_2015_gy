#include "NURBS.h"

extern void InitTexture(GLuint*, GLint, int, int, GLenum, float*);
extern void InitTexture(GLuint*, GLint, int, int, GLenum);
extern float FindMaxValue(int, GLuint[2], GLParameters*);
extern float FindMinValue(int, GLuint[2], GLParameters*, float[2]);
extern GLuint FindRenderLocation(GLuint, int*, int, int, vector<int>*, GLParameters*, bool x=false, Float4* timeData = NULL);
extern void StartGPUComputation(GLParameters*);
extern void EndGPUCompuation(GLParameters*);
extern void CheckFrameBuffer();
extern float FindMinBBoxMaxExtent(GLuint, int, int, GLParameters*);
extern Float4 FindRowMinMaxBBoxExtent(GLuint, GLuint, int, int, GLParameters*);	//Return min row texture and min value of max range
extern void RowFilterTexture(GLuint, GLuint, int, int, GLParameters*);	//Return row filter
extern void ReduceMomentSum(GLuint, int, int, GLParameters*, float moment[4]);
extern void Reduce4Sum(GLuint, GLuint, int, int, GLParameters*);
extern void Reduce9Sum(GLuint, GLuint, int, int, GLParameters*);
