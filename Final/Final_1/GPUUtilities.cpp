#include "GPUUtilities.h"
#include <CG/cgGL.h>
#include <CG/cg.h>
#include <GL/gl.h>
#include <vector>

extern void InitTexture(GLuint* texIndex, GLint internalFormat, int texWidth, int texHeight, GLenum sourceFormat, float* data)
{
	//	TexImage2D KEY
	//
	//	Texture type
	//	Mipmap level
	//	GPU internalFormat	0 - LUMINANCE_FLOAT_FORMAT 1 - RGBA_FLOAT_FORMAT 2 - FRAMEBUFFER_FORMAT
	//	Width
	//	Height
	//	Border
	//	Source data format
	//	Source data type
	//	Texture data
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, *texIndex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, internalFormat, texWidth, texHeight, 0, sourceFormat, GL_FLOAT, data);
}

extern void InitTexture(GLuint* texIndex, GLint internalFormat, int texWidth, int texHeight, GLenum sourceFormat)
{
	//	TexImage2D KEY
	//
	//	Texture type
	//	Mipmap level
	//	GPU internalFormat	0 - LUMINANCE_FLOAT_FORMAT 1 - RGBA_FLOAT_FORMAT 2 - FRAMEBUFFER_FORMAT
	//	Width
	//	Height
	//	Border
	//	Source data format
	//	Source data type
	//	Texture data
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, *texIndex);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, internalFormat, texWidth, texHeight, 0, sourceFormat, GL_FLOAT, NULL);
}

extern void StartGPUComputation(GLParameters* glParam)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

	//Enable fragment program
	cgGLEnableProfile(glParam->cgFragmentProfile);

	//Bind our FBO and tell OGL to draw to it
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);

	//Init floating-point textures
	glEnable(GL_TEXTURE_RECTANGLE_ARB);

}

extern void EndGPUCompuation(GLParameters* glParam)
{
	//Unbind the frame buffer and disable the program
	cgGLDisableProfile(glParam->cgFragmentProfile);
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	// Restore the previous views
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
}

extern void CheckFrameBuffer()
{     
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);  

	//Our FBO is perfect, return true
	if(status == GL_FRAMEBUFFER_COMPLETE_EXT)
	{
		cout << "Framebuffer Complete"<<endl;
		return;
	}

	switch(status) 
	{                                          
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT: 
			cout << "FBO has one or several image attachments with different internal formats" << endl;

		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
			cout << "FBO has one or several image attachments with different dimensions" << endl;

		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT: 
			cout << "FBO missing an image attachment" << endl;

		case GL_FRAMEBUFFER_UNSUPPORTED_EXT:                   
			cout << "FBO format unsupported" << endl;

		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:                   
			cout << "FBO Incomplete" << endl;

		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:
			cout << "FBO Incomplete Draw Buffer" << endl;
	}
	cout << "Unsupported Framebuffer Format" << endl;
}
void CheckGLError()
{
	GLenum error = glGetError();
	switch(error)
	{
	case GL_NO_ERROR:
		break;
	case GL_INVALID_ENUM:
		cout << "Invalid Enum"<<endl;
		break;
	case GL_INVALID_VALUE:
		cout << "Invalid Value"<<endl;
		break;
	case GL_INVALID_OPERATION:
		cout << "Invalid Operation"<<endl;
		break;
	case GL_STACK_OVERFLOW:
		cout << "Stack Overflow"<<endl;
		break;
	case GL_STACK_UNDERFLOW:
		cout << "Stack Undeflow"<<endl;
		break;
	case GL_OUT_OF_MEMORY:
		cout << "Out of Memory"<<endl;
		break;
	}
}

extern float FindMaxValue(int inputTexSize, GLuint sourceTex[2], GLParameters* glParam)
{
	cgGLBindProgram(glParam->reduceMaxProgram);
	
	for(int texSize = inputTexSize/2, count = 0; texSize >= 1; texSize=texSize/2, count++)
	{
		cgGLSetTextureParameter(glParam->reduceMaxTextureParam, sourceTex[count%2]);
		cgGLEnableTextureParameter(glParam->reduceMaxTextureParam);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, sourceTex[count%2]);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, sourceTex[(count+1)%2], 0);
		glViewport(0, 0, texSize, texSize);

		DrawQuad(texSize, texSize);
//		glFlush();
//		glFinish();
#ifdef DUMP
		GLfloat* retVal=new GLfloat[texSize*texSize];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texSize,texSize,GL_GREEN,GL_FLOAT,retVal);
		WriteToFile(retVal,texSize,texSize);
#endif
		cgGLDisableTextureParameter(glParam->reduceMaxTextureParam);
	}
	float maxVal;
	glReadPixels(0, 0, 1, 1, GL_RED, GL_FLOAT, &maxVal);
	return maxVal;
}

float FindMinBBoxMaxExtent(GLuint texture, int texWidth, int texHeight, GLParameters* glParam)
{
	int inputTexSize;
	if (texWidth > texHeight)
	{
		inputTexSize = GetNextPower2(texWidth);
		if (inputTexSize != texWidth)
			inputTexSize /= 2;
	}
	else
	{
		inputTexSize = GetNextPower2(texHeight);
		if (inputTexSize != texHeight)
			inputTexSize /= 2;
	}

	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);

	GLuint srcTex	= texture;
	GLuint dstTex	= tempTex[0];

	if (inputTexSize != texWidth)
	{
		cgGLBindProgram(glParam->reduceMinBBox2Program);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		cgGLSetTextureParameter(glParam->reduceMinBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinBBox2TextureParam);

		float offset[2];
		offset[0] = inputTexSize;
		offset[1] = 0;
		cgGLSetParameter2fv(glParam->reduceMinBBox2OffsetParam, offset);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);

		glViewport(0, 0, inputTexSize, texHeight);

		DrawQuad(inputTexSize, texHeight);
//		glFlush();
//		glFinish();

		srcTex = dstTex;
		dstTex = tempTex[1];

		cgGLDisableTextureParameter(glParam->reduceMinBBox2TextureParam);
	}
	if (inputTexSize != texHeight)
	{
		cgGLBindProgram(glParam->reduceMinBBox2Program);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		cgGLSetTextureParameter(glParam->reduceMinBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinBBox2TextureParam);

		float offset[2];
		offset[0] = 0;
		offset[1] = inputTexSize;
		cgGLSetParameter2fv(glParam->reduceMinBBox2OffsetParam, offset);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);

		glViewport(0, 0, inputTexSize, inputTexSize);

		DrawQuad(inputTexSize, inputTexSize);
//		glFlush();
//		glFinish();

		srcTex = dstTex;
		dstTex = tempTex[0];
		if (srcTex == dstTex)
			dstTex = tempTex[1];

		cgGLDisableTextureParameter(glParam->reduceMinBBox2TextureParam);
	}

	cgGLBindProgram(glParam->reduceMinBBox4Program);
	for(int texSize = inputTexSize/2, count = 0; texSize >= 1; texSize=texSize/2, count++)
	{
		if (count > 0)
		{
			if (count%2)
			{
				srcTex = tempTex[0];
				dstTex = tempTex[1];
			}
			else
			{
				srcTex = tempTex[1];
				dstTex = tempTex[0];
			}
		}

		cgGLSetTextureParameter(glParam->reduceMinBBox4TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinBBox4TextureParam);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);
		glViewport(0, 0, texSize, texSize);

		DrawQuad(texSize, texSize);
//		glFlush();
//		glFinish();
		cgGLDisableTextureParameter(glParam->reduceMinBBox4TextureParam);

#ifdef DUMP
		GLfloat* retVal=new GLfloat[texSize*texSize];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texSize,texSize,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,texSize,texSize);
#endif
	}
	float minExtent[4];
	glReadPixels(0, 0, 1, 1, GL_RGBA, GL_FLOAT, &minExtent);
	glDeleteTextures(2, tempTex);
	return minExtent[0]+minExtent[1];
}

void RowFilterTexture(GLuint texture, GLuint outTexture, int texWidth, int outTexSize, GLParameters* glParam)
{
	int texHeight = texWidth;
	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);

	GLuint srcTex	= texture;
	GLuint dstTex	= tempTex[0];
	// Reduce 2 times along width to find Min Max value for each row
	cgGLBindProgram(glParam->reduceMinMaxBBox2Program);
	for(int currentTexWidth = texWidth/2, count = 0; count < 2; currentTexWidth/=2, count++)
	{
		if (count == 1)
		{
			srcTex = tempTex[0];
			dstTex = tempTex[1];
		}

		cgGLSetTextureParameter(glParam->reduceMinMaxBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinMaxBBox2TextureParam);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		float offset[2];
		offset[0] = 1;
		offset[1] = 0;
		cgGLSetParameter2fv(glParam->reduceMinMaxBBox2OffsetParam, offset);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);
		glViewport(0, 0, currentTexWidth, texHeight);

		DrawQuad(currentTexWidth*2, texHeight);
//		glFlush();
//		glFinish();
#ifdef DUMP
		GLfloat* retVal=new GLfloat[currentTexWidth*texHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,currentTexWidth,texHeight,GL_GREEN,GL_FLOAT,retVal);
		WriteToFile(retVal,currentTexWidth,texHeight);
#endif
		cgGLDisableTextureParameter(glParam->reduceMinMaxBBox2TextureParam);
	}

	// Read back to CPU
	float* filter1Values = new float[texWidth*texHeight];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texWidth/4, texHeight, GL_RGBA, GL_FLOAT, filter1Values);

	// Reduce all values and write to out texture
	float* filterRowValues = new float[outTexSize*outTexSize];
	for (int i = 0; i < outTexSize*outTexSize; i++)
		filterRowValues[i] = 1;
	for (int i = 0; i < texWidth*texHeight/4; i++)
	{
		int currentRowValue = int(filter1Values[i*4+2]);
		if (currentRowValue >= 0)
		{
			float currentFilterValue = filter1Values[i*4+0];
			if (currentFilterValue == 0)
				filterRowValues[currentRowValue] = 0;
		}
	}

	// Upload data to texture
	InitTexture(&outTexture, LUMINANCE_FLOAT_FORMAT, outTexSize, outTexSize, GL_LUMINANCE, filterRowValues);	
	// WriteToFile(minMaxValues, outTexSize, outTexSize, 1);

	glDeleteTextures(2, tempTex);
	delete[] filterRowValues;
	delete[] filter1Values;
}



Float4 FindRowMinMaxBBoxExtent(GLuint texture, GLuint outTexture, int texWidth, int outTexSize, GLParameters* glParam)
{
	int texHeight = texWidth;
	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);

	GLuint srcTex	= texture;
	GLuint dstTex	= tempTex[0];
	// Reduce 2 times along width to find Min Max value for each row
	cgGLBindProgram(glParam->reduceMinMaxBBox2Program);
	for(int currentTexWidth = texWidth/2, count = 0; count < 2; currentTexWidth/=2, count++)
	{
		if (count == 1)
		{
			srcTex = tempTex[0];
			dstTex = tempTex[1];
		}

		cgGLSetTextureParameter(glParam->reduceMinMaxBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinMaxBBox2TextureParam);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		float offset[2];
		offset[0] = 1;
		offset[1] = 0;
		cgGLSetParameter2fv(glParam->reduceMinMaxBBox2OffsetParam, offset);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);
		glViewport(0, 0, currentTexWidth, texHeight);

		DrawQuad(currentTexWidth*2, texHeight);
//		glFlush();
//		glFinish();
#ifdef DUMP
		GLfloat* retVal=new GLfloat[currentTexWidth*texHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,currentTexWidth,texHeight,GL_GREEN,GL_FLOAT,retVal);
		WriteToFile(retVal,currentTexWidth,texHeight);
#endif
		cgGLDisableTextureParameter(glParam->reduceMinMaxBBox2TextureParam);
	}

	// Read back to CPU
	float* minMaxRowValues = new float[texWidth*texHeight];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texWidth/4, texHeight, GL_RGBA, GL_FLOAT, minMaxRowValues);

	// Reduce all values and write to out texture
	float* minMaxValues = new float[outTexSize*outTexSize*4];
	for (int i = 0; i < outTexSize*outTexSize*4; i++)
		minMaxValues[i] = -1;
	for (int i = 0; i < texWidth*texHeight/4; i++)
	{
		int currentRowValue = int(minMaxRowValues[i*4+2]);
		if (currentRowValue >= 0)
		{
			float currentMinValue = minMaxRowValues[i*4+0];
			float currentMaxValue = minMaxRowValues[i*4+1];
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			//(x2.y < x1.y && x2.x > 0) || x1.x < 0)
			if ((currentMaxValue < minMaxValues[currentRowValue*4+1] && currentMinValue > 0) || minMaxValues[currentRowValue*4+0] < 0)
			{
				minMaxValues[currentRowValue*4+0] = currentMinValue;
				minMaxValues[currentRowValue*4+1] = currentMaxValue;
				minMaxValues[currentRowValue*4+2] = minMaxRowValues[i*4+2];
				minMaxValues[currentRowValue*4+3] = minMaxRowValues[i*4+3];
			}
		}
	}

	// Upload data to texture
	InitTexture(&outTexture, RGBA_FLOAT_FORMAT, outTexSize, outTexSize, GL_RGBA, minMaxValues);
	
	//WriteToFile(minMaxValues, outTexSize, outTexSize, 1);

	srcTex	= texture;
	dstTex	= tempTex[0];
	// Reduce 2 times along width to find Min Max value for each row
	cgGLBindProgram(glParam->reduceMinMinBBox2Program);
	for(int currentTexWidth = texWidth/2, count = 0; count < 2; currentTexWidth/=2, count++)
	{
		if (count == 1)
		{
			srcTex = tempTex[0];
			dstTex = tempTex[1];
		}

		cgGLSetTextureParameter(glParam->reduceMinMinBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinMinBBox2TextureParam);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		float offset[2];
		offset[0] = 1;
		offset[1] = 0;
		cgGLSetParameter2fv(glParam->reduceMinMinBBox2OffsetParam, offset);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);
		glViewport(0, 0, currentTexWidth, texHeight);

		DrawQuad(currentTexWidth*2, texHeight);
//		glFlush();
//		glFinish();
#ifdef DUMP
		GLfloat* retVal=new GLfloat[currentTexWidth*texHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,currentTexWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,currentTexWidth,texHeight);
#endif
		cgGLDisableTextureParameter(glParam->reduceMinMinBBox2TextureParam);
	}

	// Read back to CPU
	float* minMinRowValues = new float[texWidth*texHeight];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texWidth/4, texHeight, GL_RGBA, GL_FLOAT, minMinRowValues);

	// Reduce all values and write to out texture
	float* minMinValues = new float[outTexSize*outTexSize*4];
	for (int i = 0; i < outTexSize*outTexSize*4; i++)
		minMinValues[i] = -1;
	for (int i = 0; i < texWidth*texHeight/4; i++)
	{
		int currentRowValue = int(minMaxRowValues[i*4+2]);
		if (currentRowValue >= 0)
		{
			float currentMinValue = minMinRowValues[i*4+0];
			float currentMaxValue = minMinRowValues[i*4+1];
			// int xPos = currentRowValue%outTexSize;
			// int yPos = int(currentRowValue/(outTexSize*1.0));
			// ((x2.x < x1.x && x2.x > 0) || (x2.x == x1.x && x2.y > x1.y) || x1.x < 0)
			if ((currentMinValue < minMinValues[currentRowValue*4+0] && currentMinValue > 0) || (currentMinValue == minMinValues[currentRowValue*4+0] && currentMaxValue > minMinValues[currentRowValue*4+1]) || minMinValues[currentRowValue*4+0] < 0)
			{
				minMinValues[currentRowValue*4+0] = currentMinValue;
				minMinValues[currentRowValue*4+1] = currentMaxValue;
				minMinValues[currentRowValue*4+2] = minMinRowValues[i*4+2];
				minMinValues[currentRowValue*4+3] = minMinRowValues[i*4+3];
			}
		}
	}

	//WriteToFile(minMinValues, 1, outTexSize*outTexSize, 0);

	// Reduce to find Max Value of MinMin distances
	Float4 maxMinMinValue = Float4(minMinValues[0], minMinValues[1], minMinValues[2], minMinValues[3]);
	for (int i = 0; i < outTexSize*outTexSize; i++)
	{
		if (minMinValues[i*4 + 0] > maxMinMinValue[0])
		{
			maxMinMinValue[0] = minMinValues[i*4 + 0];
			maxMinMinValue[1] = minMinValues[i*4 + 1];
			maxMinMinValue[2] = minMinValues[i*4 + 2];
			maxMinMinValue[3] = minMinValues[i*4 + 3];
		}
	}

	glDeleteTextures(2, tempTex);
	delete[] minMaxRowValues;
	delete[] minMinRowValues;
	delete[] minMinValues;
	delete[] minMaxValues;
	return maxMinMinValue;
}


/*
Float4 FindRowMinMaxBBoxExtent(GLuint texture, GLuint outTexture, int texWidth, int texHeight, int outTexSize, GLParameters* glParam)
{
	int inputTexHeight, inputTexWidth;
	inputTexHeight = GetNextPower2(texHeight)/2;
	inputTexWidth = GetNextPower2(texWidth)/2;


	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);

	GLuint srcTex	= texture;
	GLuint dstTex	= tempTex[0];

	cgGLBindProgram(glParam->reduceMinBBox2Program);
	for(int texHeight = inputTexHeight, count = 0; texHeight >= 1; texHeight/=2, count++)
	{
		if (count > 0)
		{
			if (count%2)
			{
				srcTex = tempTex[0];
				if (texHeight != 1)
					dstTex = tempTex[1];
				else
					dstTex = outTexture;
			}
			else
			{
				srcTex = tempTex[1];
				if (texHeight != 1)
					dstTex = tempTex[0];
				else
					dstTex = outTexture;
			}
		}

		cgGLSetTextureParameter(glParam->reduceMinBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMinBBox2TextureParam);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		float offset[2];
		offset[0] = 0;
		offset[1] = texHeight;
		cgGLSetParameter2fv(glParam->reduceMinBBox2OffsetParam, offset);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);
		glViewport(0, 0, texWidth, texHeight);

		DrawQuad(texWidth, texHeight);
//		glFlush();
//		glFinish();
		cgGLDisableTextureParameter(glParam->reduceMinBBox2TextureParam);

#ifdef DUMP
		GLfloat* retVal=new GLfloat[texWidth*texHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,texWidth,texHeight);
#endif
	}

	// Find Max value of the range
	srcTex	= outTexture;
	dstTex	= tempTex[0];

	cgGLBindProgram(glParam->reduceMaxBBox2Program);
	for(int texWidth = inputTexWidth, count = 0; texWidth >= 1; texWidth/=2, count++)
	{
		if (count > 0)
		{
			if (count%2)
			{
				srcTex = tempTex[0];
				dstTex = tempTex[1];
			}
			else
			{
				srcTex = tempTex[1];
				dstTex = tempTex[0];
			}
		}

		cgGLSetTextureParameter(glParam->reduceMaxBBox2TextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceMaxBBox2TextureParam);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		float offset[2];
		offset[0] = texWidth;
		offset[1] = 0;
		cgGLSetParameter2fv(glParam->reduceMaxBBox2OffsetParam, offset);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);
		glViewport(0, 0, texWidth, 1);

		DrawQuad(texWidth, 1);
//		glFlush();
//		glFinish();
		cgGLDisableTextureParameter(glParam->reduceMaxBBox2TextureParam);

#ifdef DUMP
		GLfloat* retVal=new GLfloat[texWidth*1];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texWidth,1,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,texWidth,1);
#endif
	}
	float maxExtent[4];
	glReadPixels(0, 0, 1, 1, GL_RGBA, GL_FLOAT, &maxExtent);

	glDeleteTextures(2, tempTex);
	Float4 outMaxExtent = Float4( maxExtent[0], maxExtent[1], maxExtent[2], maxExtent[3]);
	return outMaxExtent;
}
*/

void ReduceMomentSum(GLuint texture, int texWidth, int texHeight, GLParameters* glParam, float moment[4])
{
	int	sqrTexWidth = GetNextPower2(texWidth);
	int sqrTexHeight = GetNextPower2(texHeight);

	GLuint tempTex[2];
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glGenTextures(2, tempTex);
	InitTexture(tempTex+0, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);
	InitTexture(tempTex+1, RGBA_FLOAT_FORMAT, texWidth, texHeight, GL_RGBA, NULL);

	GLuint srcTex	= texture;
	GLuint dstTex	= tempTex[0];
	int currentTexWidth = sqrTexWidth;
	int currentTexHeight = sqrTexHeight;

	// Reduce along width direction
//	if (sqrTexWidth != texWidth)
	{
		currentTexWidth /= 2;
		cgGLBindProgram(glParam->reduceSum2Pow2Program);
		cgGLSetTextureParameter(glParam->reduceSum2Pow2InputTextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceSum2Pow2InputTextureParam);

		float offset[4];
		offset[0] = currentTexWidth;
		offset[1] = 0;
		offset[2] = texWidth-currentTexWidth;
		offset[3] = 0;
		cgGLSetParameter4fv(glParam->reduceSum2Pow2OffsetParam, offset);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);

		glViewport(0, 0, currentTexWidth, texHeight);

		DrawQuad(currentTexWidth, texHeight);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[currentTexWidth*texHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,currentTexWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,currentTexWidth,texHeight);
		delete[] retVal;
#endif

		srcTex = dstTex;
		dstTex = tempTex[1];

		cgGLDisableTextureParameter(glParam->reduceSum2Pow2InputTextureParam);
		currentTexWidth /= 2;
	}
	cgGLBindProgram(glParam->reduceSum2Program);
	while (currentTexWidth != 0)
	{

		cgGLSetTextureParameter(glParam->reduceSum2InputTextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceSum2InputTextureParam);

		float offset[2];
		offset[0] = currentTexWidth;
		offset[1] = 0;
		cgGLSetParameter2fv(glParam->reduceSum2OffsetParam, offset);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);

		glViewport(0, 0, currentTexWidth, texHeight);

		DrawQuad(currentTexWidth, texHeight);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[currentTexWidth*texHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,currentTexWidth,texHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,currentTexWidth,texHeight);
		delete[] retVal;
#endif

		GLuint oldSrcTex = srcTex;
		srcTex = dstTex;
		dstTex = oldSrcTex;

		cgGLDisableTextureParameter(glParam->reduceSum2InputTextureParam);
		currentTexWidth /= 2;
	}
//	if (sqrTexHeight != texHeight)
	{
		currentTexHeight /= 2;
		InitTexture(&dstTex, RGBA_FLOAT_FORMAT, 1, currentTexHeight, GL_RGBA, NULL);
		cgGLBindProgram(glParam->reduceSum2Pow2Program);

		cgGLSetTextureParameter(glParam->reduceSum2Pow2InputTextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceSum2Pow2InputTextureParam);

		float offset[4];
		offset[0] = 0;
		offset[1] = currentTexHeight;
		offset[2] = 0;
		offset[3] = texHeight - currentTexHeight;
		cgGLSetParameter4fv(glParam->reduceSum2Pow2OffsetParam, offset);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);

		glViewport(0, 0, 1, currentTexHeight);

		DrawQuad(1, currentTexHeight);
		glFlush();
		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[1*currentTexHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,1,currentTexHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,1,currentTexHeight);
		delete[] retVal;
#endif

		GLuint oldSrcTex = srcTex;
		srcTex = dstTex;
		dstTex = oldSrcTex;

		cgGLDisableTextureParameter(glParam->reduceSum2Pow2InputTextureParam);
		currentTexHeight /= 2;
	}
	cgGLBindProgram(glParam->reduceSum2Program);
	while (currentTexHeight != 0)
	{
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, srcTex);

		cgGLSetTextureParameter(glParam->reduceSum2InputTextureParam, srcTex);
		cgGLEnableTextureParameter(glParam->reduceSum2InputTextureParam);

		float offset[2];
		offset[0] = 0;
		offset[1] = currentTexHeight;
		cgGLSetParameter2fv(glParam->reduceSum2OffsetParam, offset);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, dstTex, 0);

		glViewport(0, 0, 1, currentTexHeight);

		DrawQuad(1, currentTexHeight);
//		glFlush();
//		glFinish();

#ifdef DUMP
		GLfloat* retVal=new GLfloat[1*currentTexHeight];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,1,currentTexHeight,GL_RED,GL_FLOAT,retVal);
		WriteToFile(retVal,1,currentTexHeight);
		delete[] retVal;
#endif

		GLuint oldSrcTex = srcTex;
		srcTex = dstTex;
		dstTex = oldSrcTex;

		cgGLDisableTextureParameter(glParam->reduceSum2InputTextureParam);
		currentTexHeight /= 2;
	}

	glReadPixels(0, 0, 1, 1, GL_RGBA, GL_FLOAT, moment);
	glDeleteTextures(2, tempTex);
}

void Reduce4Sum(GLuint inputTex, GLuint outputTex, int texWidth, int texHeight, GLParameters* glParam)
{
	cgGLBindProgram(glParam->reduceSum4Program);

	cgGLEnableTextureParameter(glParam->reduceSum4InputTextureParam);
	cgGLSetTextureParameter(glParam->reduceSum4InputTextureParam, inputTex);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTex);

	glViewport(0, 0, texWidth/2, texHeight/2);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, outputTex, 0);

	DrawQuad(texWidth/2, texHeight/2);
	glFlush();
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[texWidth*texHeight/4];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texWidth/2, texHeight/2, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, texWidth/2, texHeight/2);
#endif

	cgGLDisableTextureParameter(glParam->reduceSum4InputTextureParam);
}

void Reduce9Sum(GLuint inputTex, GLuint outputTex, int texWidth, int texHeight,  GLParameters* glParam)
{
	cgGLBindProgram(glParam->reduceSum9Program);

	cgGLEnableTextureParameter(glParam->reduceSum9InputTextureParam);
	cgGLSetTextureParameter(glParam->reduceSum9InputTextureParam, inputTex);

	// Bind the src textures
	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, inputTex);

	glViewport(0, 0, texWidth/3, texHeight/3);

	// Specify the dst texture
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, outputTex, 0);

	DrawQuad(texWidth/3, texHeight/3);
	glFlush();
	glFinish();

#ifdef DUMP
	GLfloat* retVal=new GLfloat[texWidth*texHeight/9];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texWidth/3, texHeight/3, GL_RED, GL_FLOAT, retVal);
	WriteToFile(retVal, texWidth/3, texHeight/3);
#endif

	cgGLDisableTextureParameter(glParam->reduceSum4InputTextureParam);
}


GLuint FindRenderLocation(GLuint texture, int* numHits, int texWidth, int texHeight, vector<int>* locations, GLParameters* glParam, bool returnTexture, Float4* timeData)
{
#ifndef GPUFIND
#ifdef OCCLUSIONFIND
	int iSize;
	if (texWidth > texHeight)
		iSize = GetNextPower2(texWidth);
	else
		iSize = GetNextPower2(texHeight);

	clock_t begintime;
	if (timeData != NULL)
		begintime = clock();

	GLuint tempTex[2];
	glGenTextures(2, tempTex);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[0]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, iSize, iSize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[1]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, iSize, iSize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

	cgGLBindProgram(glParam->findPrepassProgram);
	cgGLSetTextureParameter(glParam->findPrepassTextureParam, texture);
	cgGLEnableTextureParameter(glParam->findPrepassTextureParam);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

	glViewport(0,0,iSize,iSize);
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[0], 0);

	glViewport(0, 0, texWidth, texHeight);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(texWidth, texHeight);
//	glFlush();
//	glFinish();

	cgGLDisableTextureParameter(glParam->findPrepassTextureParam);

	if(timeData!=NULL)
	{
		timeData->value[0] += (clock()-begintime);
		begintime = clock();
	}

#ifdef DUMP
	GLfloat *retVal = new GLfloat[iSize*iSize];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,iSize,iSize,GL_RED,GL_FLOAT,retVal);
	WriteToFileInt(retVal,iSize,iSize);
#endif

	int currentSize = iSize/2;
	int sourceTex = 0;
	int destTex = 1;

	cgGLBindProgram(glParam->occlusionProgram);
	cgGLSetTextureParameter(glParam->occlusionTextureParam, tempTex[sourceTex]);
	cgGLEnableTextureParameter(glParam->occlusionTextureParam);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTex[sourceTex]);

#ifdef NVIDIA
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTex[destTex], 0);
#else
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT,0);
	glDrawBuffer(GL_BACK);
#endif
	// DrawLocations x,y,width,height
	vector<Float4> drawLocations;
	drawLocations.push_back(Float4(0,0,currentSize,currentSize));
	drawLocations.push_back(Float4(iSize/2,0,currentSize,currentSize));
	drawLocations.push_back(Float4(0,iSize/2,currentSize,currentSize));
	drawLocations.push_back(Float4(iSize/2,iSize/2,currentSize,currentSize));
	while (currentSize>=1)
	{
		vector<Float4> newDrawLocations;
		GLuint* queries = new GLuint[drawLocations.size()];
		glGenQueries(drawLocations.size(),queries);
		for (int locationNum = 0; locationNum < drawLocations.size(); locationNum++)
		{
			glViewport(drawLocations[locationNum][0], drawLocations[locationNum][1], drawLocations[locationNum][2], drawLocations[locationNum][3]);

//			glClear(GL_COLOR_BUFFER_BIT);

			glBeginQueryARB(GL_SAMPLES_PASSED_ARB,queries[locationNum]);

			DrawQuad(0,0,2,2,drawLocations[locationNum][0], drawLocations[locationNum][1], drawLocations[locationNum][2], drawLocations[locationNum][3]);

			glEndQueryARB(GL_SAMPLES_PASSED_ARB);
			glFinish();
		}
		for (int locationNum = 0; locationNum < drawLocations.size(); locationNum++)
		{
			GLuint fragmentCount;
			glGetQueryObjectuivARB(queries[locationNum], GL_QUERY_RESULT_ARB, &fragmentCount);
			if (fragmentCount>0)
			{
				if (currentSize!=1)
				{
					Float4 newLocation1 = Float4(drawLocations[locationNum][0],drawLocations[locationNum][1],currentSize/2,currentSize/2);
					Float4 newLocation2 = Float4(drawLocations[locationNum][0]+currentSize/2,drawLocations[locationNum][1],currentSize/2,currentSize/2);
					Float4 newLocation3 = Float4(drawLocations[locationNum][0],drawLocations[locationNum][1]+currentSize/2,currentSize/2,currentSize/2);
					Float4 newLocation4 = Float4(drawLocations[locationNum][0]+currentSize/2,drawLocations[locationNum][1]+currentSize/2,currentSize/2,currentSize/2);
					newDrawLocations.push_back(newLocation1);
					newDrawLocations.push_back(newLocation2);
					newDrawLocations.push_back(newLocation3);
					newDrawLocations.push_back(newLocation4);
				}
				else
				{
					Float4 newLocation = Float4(drawLocations[locationNum][0],drawLocations[locationNum][1],1,1);
					newDrawLocations.push_back(newLocation);
				}
			}

#ifdef DUMP
			GLfloat *retVal = new GLfloat[iSize*iSize];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,iSize,iSize,GL_RED,GL_FLOAT,retVal);
			WriteToFileInt(retVal,iSize,iSize);
#endif
		}
		currentSize/=2;
		glDeleteQueries(drawLocations.size(),queries);
		delete[] queries;
		drawLocations = newDrawLocations;
	}
	cgGLDisableTextureParameter(glParam->findPrepassTextureParam);

	if(timeData!=NULL)
		timeData->value[1] += (clock()-begintime);

	for (int locationNum = 0; locationNum < drawLocations.size(); locationNum++)
		locations->push_back(drawLocations[locationNum][1]*(texWidth)+drawLocations[locationNum][0]);

#ifndef NVIDIA
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, glParam->fbo);
	glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
#endif
	glDeleteTextures(2, tempTex);
	return 0;
#else
	float* positionData = new float[texWidth*texHeight];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texWidth ,texHeight, GL_RED, GL_FLOAT, positionData);
	for (int vCount=0; vCount<texHeight; vCount++)
		for (int uCount=0; uCount<texWidth; uCount++)
			if (positionData[vCount*(texWidth)+uCount] == 1)
					locations->push_back(vCount*(texWidth)+uCount);
	*numHits = locations->size();
	delete [] positionData;
	return 0;
#endif
#else
#ifdef GPUFIND1
	int iSize;
	if (texWidth > texHeight)
		iSize = GetNextPower2(texWidth);
	else
		iSize = GetNextPower2(texHeight);

	clock_t begintime = clock();
	GLuint iTexture[2];
	glGenTextures(2, iTexture);

	glEnable(GL_TEXTURE_RECTANGLE_ARB);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[0]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, iSize*2, iSize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

	cgGLBindProgram(glParam->findPrepassProgram);
	cgGLSetTextureParameter(glParam->findPrepassTextureParam, texture);
	cgGLEnableTextureParameter(glParam->findPrepassTextureParam);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

	glViewport(0,0,2*iSize,iSize);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[0], 0);

	glViewport(0, 0,texWidth, texHeight);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(texWidth, texHeight);
//	glFlush();
//	glFinish();

	cgGLDisableTextureParameter(glParam->findPrepassTextureParam);

	if(timeData!=NULL)
		timeData->value[0] += (clock()-begintime);
	begintime = clock();
#ifdef DUMP
	GLfloat *retVal = new GLfloat[iSize*iSize*2];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,iSize*2,iSize,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,iSize*2,iSize);
#endif

	float vorigin[] = {0,0,0,0};
	float viewporty = 0;
	float viewportx = iSize;
	int count = 0;

	//Build the hierarchy
	cgGLBindProgram(glParam->findReduceProgram);
	cgGLSetTextureParameter(glParam->findReduceTextureParam, iTexture[0]);
	cgGLEnableTextureParameter(glParam->findReduceTextureParam);

	for(int cnt=iSize; cnt>1; cnt=cnt/2,count++)
	{
		// vorigin serves as the origin of the texture to read from
		if(cnt==iSize)
			vorigin[0]=vorigin[1]=0;
		else
		{
			vorigin[0] = iSize;
			vorigin[1] = viewporty - iSize/(pow(2.0,count));  
		}
		cgGLSetParameter4fv(glParam->findReduceOriginParam,vorigin);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[0]);

		glViewport(viewportx, viewporty, cnt/2, cnt/2);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[0], 0);

		DrawQuad(cnt/2, cnt/2);
//		glFlush();
//		glFinish();

		// Location of the next block to render into 
		viewporty += iSize/(pow(2.0,count+1));

#ifdef DUMP
		GLfloat *retVal = new GLfloat[iSize*iSize*2];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,iSize*2,iSize,GL_RED,GL_FLOAT,retVal);
		WriteToFileInt(retVal,iSize*2,iSize);
#endif
	}
	cgGLDisableTextureParameter(glParam->findReduceTextureParam);

	if(timeData!=NULL)
		timeData->value[1] += (clock()-begintime);
	begintime = clock();

	float tempNumHits;
	viewporty -= iSize/(pow(2.0,count));
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(viewportx, viewporty, 1, 1, GL_RED, GL_FLOAT, &tempNumHits);
	*numHits = int(tempNumHits);

	float origin[4];
	viewportx = ceil(sqrt(*numHits*1.0));
	viewporty = viewportx;

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[1]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, viewportx, viewporty, 0, GL_RGBA, GL_FLOAT, NULL);

	cgGLSetTextureParameter(glParam->findTextureParam, iTexture[0]);
	cgGLEnableTextureParameter(glParam->findTextureParam);
	cgGLSetParameter1f(glParam->findSizeParam,iSize);
	cgGLSetParameter1f(glParam->findInputSizeParam,viewportx);

	cgGLBindProgram(glParam->findProgram);

	glViewport(0, 0,viewportx, viewporty);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[1], 0);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[0]);

	DrawQuad(viewportx, viewporty);
//	glFlush();
//	glFinish();

	glDisable(GL_TEXTURE_RECTANGLE_ARB);
	cgGLDisableTextureParameter(glParam->findTextureParam);

	if(timeData!=NULL)
		timeData->value[2] += (clock()-begintime);

	if(!returnTexture)
	{
		float* pickedBoxIndex = new float[viewportx*viewporty*3];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,viewportx,viewporty,GL_RGB,GL_FLOAT,pickedBoxIndex);

		for(int hitCount = 0; hitCount < *numHits ; hitCount++)
			locations->push_back((int)pickedBoxIndex[hitCount*3]+(int)pickedBoxIndex[hitCount*3+1]*texWidth);
		delete [] pickedBoxIndex;
		glDeleteTextures(2, iTexture);
		return 0;
	}
	else
	{
		glDeleteTextures(1, iTexture);
		return iTexture[1];
	}
#endif
#ifdef GPUFIND2
	int iSize;
	if (texWidth > texHeight)
		iSize = GetNextPower2(texWidth);
	else
		iSize = GetNextPower2(texHeight);
	clock_t begintime = clock();

	GLuint iTexture[3];
	glGenTextures(3, iTexture);

	glEnable(GL_TEXTURE_RECTANGLE_ARB);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[0]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, iSize*2, iSize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

	cgGLBindProgram(glParam->findPrepassProgram);
	cgGLSetTextureParameter(glParam->findPrepassTextureParam, texture);
	cgGLEnableTextureParameter(glParam->findPrepassTextureParam);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

	glViewport(0,0,2*iSize,iSize);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[0], 0);

	glViewport(0, 0,texWidth, texHeight);

	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	DrawQuad(texWidth, texHeight);
//	glFinish();

	cgGLDisableTextureParameter(glParam->findPrepassTextureParam);

	if(timeData!=NULL)
		timeData->value[0] += (clock()-begintime);
	begintime = clock();
#ifdef DUMP
	GLfloat *retVal = new GLfloat[iSize*iSize*2];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0,0,iSize*2,iSize,GL_RED,GL_FLOAT,retVal);
	WriteToFile(retVal,iSize*2,iSize);
#endif

	float vorigin[] = {0,0,0,0};
	float viewporty = 0;
	float viewportx = iSize;
	int count = 0;

	//Build the hierarchy
	cgGLBindProgram(glParam->findReduceProgram);
	cgGLSetTextureParameter(glParam->findReduceTextureParam, iTexture[0]);
	cgGLEnableTextureParameter(glParam->findReduceTextureParam);

	for(int cnt=iSize; cnt>1; cnt=cnt/2,count++)
	{
		// vorigin serves as the origin of the texture to read from
		if(cnt==iSize)
			vorigin[0]=vorigin[1]=0;
		else
		{
			vorigin[0] = iSize;
			vorigin[1] = viewporty - iSize/(pow(2.0,count));  
		}
		cgGLSetParameter4fv(glParam->findReduceOriginParam,vorigin);

		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[0]);

		glViewport(viewportx, viewporty, cnt/2, cnt/2);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[0], 0);

		DrawQuad(cnt/2, cnt/2);
//		glFinish();

		// Location of the next block to render into 
		viewporty += iSize/(pow(2.0,count+1));

#ifdef DUMP
		GLfloat *retVal = new GLfloat[iSize*iSize*2];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,iSize*2,iSize,GL_RED,GL_FLOAT,retVal);
		WriteToFileInt(retVal,iSize*2,iSize);
#endif
	}
	cgGLDisableTextureParameter(glParam->findReduceTextureParam);

	if(timeData!=NULL)
		timeData->value[1] += (clock()-begintime);
	begintime = clock();

	float origin[2];
	viewportx = ceil(sqrt(*numHits*1.0));
	viewporty = viewportx;

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[1]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, viewportx, viewporty, 0, GL_RGBA, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[2]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, RGBA_FLOAT_FORMAT, viewportx, viewporty, 0, GL_RGBA, GL_FLOAT, NULL);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[1], 0);
	glViewport(0,0,viewportx,viewporty);
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
//	glFlush();

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[2], 0);
	glViewport(0,0,viewportx,viewporty);
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);
//	glFlush();

	cgGLBindProgram(glParam->findProgram);
	glViewport(0, 0,viewportx, viewporty);
	if(numHits>0)
	{
		float *hitarray = (float*)calloc(3*(*numHits),sizeof(float));
		int noOfLevels = log(iSize*1.)/log(2.);
		for(int level=noOfLevels;level>0;level--)
		{
			if(level==1)
			{
				origin[0] = 0.;
				origin[1] = 0.;
			}
			else 
			{
				origin[0] = iSize;
				origin[1] = 0;
				// Go to the origin of current level
				origin[1] = pow(2.,noOfLevels-level+2)*(pow(2.,level-2)-1);
			}

			cgGLSetTextureParameter(glParam->findTextureParam, iTexture[0]);
			cgGLEnableTextureParameter(glParam->findTextureParam);
			cgGLSetTextureParameter(glParam->findSourceTextureParam, iTexture[level%2+1]);
			cgGLEnableTextureParameter(glParam->findSourceTextureParam);
			cgGLSetParameter2fv(glParam->findOriginParam,origin);

			float viewport_numhits[3];
			viewport_numhits[0]=viewportx;
			viewport_numhits[1]=viewporty;
			viewport_numhits[2]=(*numHits);
			cgGLSetParameter3fv(glParam->findViewportParam,viewport_numhits);
			
			glActiveTexture(GL_TEXTURE1_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,iTexture[level%2+1]);
			glActiveTexture(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB,iTexture[0]);

			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, iTexture[(level+1)%2+1], 0);

			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, iTexture[0]);

			DrawQuad(viewportx, viewporty);
//			glFinish();

			cgGLDisableTextureParameter(glParam->findTextureParam);
			cgGLDisableTextureParameter(glParam->findSourceTextureParam);

#ifdef DUMP
			GLfloat *retVal1 = new GLfloat[viewportx*viewporty*3];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0,0,viewportx,viewporty,GL_RGB,GL_FLOAT,retVal1);
			WriteToFileAll(retVal1,viewportx,viewporty,3);
#endif
		}
	}

	if(timeData!=NULL)
		timeData->value[2] += (clock()-begintime);

	if(!returnTexture)
	{
		float* pickedBoxIndex = new float[viewportx*viewporty*3];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,viewportx,viewporty,GL_RGB,GL_FLOAT,pickedBoxIndex);

		for(int hitCount = 0; hitCount < (*numHits) ; hitCount++)
			locations->push_back((int)pickedBoxIndex[hitCount*3]+(int)pickedBoxIndex[hitCount*3+1]*texWidth);
		delete [] pickedBoxIndex;
		glDeleteTextures(3, iTexture);
		return 0;
	}
	else
	{
		glDeleteTextures(1,&(iTexture[0]));
		glDeleteTextures(1,&(iTexture[2]));
		return iTexture[1];
	}
#endif
#ifdef GPUFIND3
	int texSize;
	if (texWidth > texHeight)
		texSize = GetNextPower2(texWidth);
	else
		texSize = GetNextPower2(texHeight);

	clock_t begintime = clock();
	GLuint tempTexture[2];
	glGenTextures(2, tempTexture);

	glEnable(GL_TEXTURE_RECTANGLE_ARB);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[0]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, texSize*2, texSize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[1]);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, texSize*2, texSize, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);


	// Transfer data to a square texture
	cgGLBindProgram(glParam->findPrepassProgram);
	cgGLSetTextureParameter(glParam->findPrepassTextureParam, texture);
	cgGLEnableTextureParameter(glParam->findPrepassTextureParam);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texture);

	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTexture[0], 0);

	glViewport(0, 0, texSize*2, texSize);
	glClearColor(0,0,0,0);
	glClear(GL_COLOR_BUFFER_BIT);

	glViewport(0, 0, texWidth, texHeight);

	DrawQuad(texWidth, texHeight);
//	glFinish();

	cgGLDisableTextureParameter(glParam->findPrepassTextureParam);

	if(timeData!=NULL)
		timeData->value[0] += (clock()-begintime);
	begintime = clock();

	cgGLBindProgram(glParam->findDownSweepProgram);
	cgGLSetTextureParameter(glParam->findDownSweepTextureParam, tempTexture[0]);
	cgGLEnableTextureParameter(glParam->findDownSweepTextureParam);

	glActiveTextureARB(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[0]);

	for (int currentSize = texSize*texSize/2, xLevel = 0, yLevel = 0; currentSize>1; currentSize/=2)
	{
		float currentHeight	= (currentSize > texSize)?texSize:currentSize;
		float currentWidth	= (int)ceil(currentSize/(1.0*texSize));

		float originX = (int)texSize*(2-1/(pow(2.0,xLevel-1))) ;
		float originY = (int)texSize*(1-1/(pow(2.0,yLevel))) ;
		float xDisp = 0;
		float yDisp = 0;

		if (pow(2.0,xLevel) <= texSize)
			xLevel++;
		else
			yLevel++;
		
		if (pow(2.0,xLevel) <= texSize)
		{
			xDisp = -1;
			originX += 0.5;
		}
		else
		{
			yDisp = -1;
			originY += 0.5;
		}

		cgGLSetParameter2f(glParam->findDownSweepOriginParam, originX, originY);
		cgGLSetParameter2f(glParam->findDownSweepxyDistParam, xDisp, yDisp);
		int viewportX = (int)texSize*(2-1/(pow(2.0,xLevel-1)));
		int viewportY = (int)texSize*(1-1/(pow(2.0,yLevel)));

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTexture[0], 0);
		glViewport(viewportX, viewportY, currentWidth, currentHeight);
		if (pow(2.0,xLevel) <= texSize)
			DrawQuad(2*currentWidth, currentHeight);
		else
			DrawQuad(currentWidth, 2*currentHeight);

//		glFlush();
//		glFinish();
#ifdef DUMP
		GLfloat* retVal=new GLfloat[texSize*texSize*2];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, texSize*2, texSize, GL_RED, GL_FLOAT, retVal);
		WriteToFileInt(retVal,texSize*2,texSize);
#endif
	}
	cgGLDisableTextureParameter(glParam->findDownSweepTextureParam);

#ifdef DUMP
	GLfloat* retVal=new GLfloat[texSize*texSize*2];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(0, 0, texSize*2, texSize, GL_RED, GL_FLOAT, retVal);
	WriteToFileInt(retVal,texSize*2,texSize);
#endif

	if(timeData!=NULL)
		timeData->value[1] += (clock()-begintime);
	begintime = clock();

#ifndef NVIDIA
	// Get number of hits
	float tempVal[2];
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	glReadPixels(2*texSize-1, texSize-4, 1, 2, GL_RED, GL_FLOAT, tempVal);
	*numHits = tempVal[0]+tempVal[1];
#endif

	int numHitsX = ceil(sqrt((*numHits)*1.0));
	int numHitsY = numHitsX;

//	int numHitsX = (*numHits)%1024;
//	int numHitsY = int((*numHits)/1024.0)+1;

	if ((*numHits) > 0)
	{
		// Make 2 copies of the same texture
	//	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[1]);
	//	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
	//	glCopyTexSubImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, 0, 0, 0, 0, 2*texSize, texSize);

		cgGLBindProgram(glParam->findPrepassProgram);

		cgGLSetTextureParameter(glParam->findPrepassTextureParam, tempTexture[0]);
		cgGLEnableTextureParameter(glParam->findPrepassTextureParam);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[0]);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTexture[1], 0);

		glViewport(0, 0, texSize*2, texSize);
		DrawQuad(texSize*2, texSize);
//		glFlush();
//		glFinish();
		cgGLDisableTextureParameter(glParam->findPrepassTextureParam);

		cgGLBindProgram(glParam->findUpSweepProgram);
		for (int currentSize = 2, xLevel = GetExponent2(texSize), yLevel = GetExponent2(texSize); currentSize<=texSize*texSize; currentSize*=2)
		{
			int srcTexture = (yLevel+xLevel)%2;
			int dstTexture = (yLevel+xLevel+1)%2;

			cgGLSetTextureParameter(glParam->findUpSweepTextureParam, tempTexture[srcTexture]);
			cgGLEnableTextureParameter(glParam->findUpSweepTextureParam);
			glActiveTextureARB(GL_TEXTURE0_ARB);
			glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[srcTexture]);
		
			glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTexture[dstTexture], 0);
		
			float currentHeight	= (currentSize > texSize)?texSize:currentSize;
			float currentWidth	= (int)ceil(currentSize/(1.0*texSize));

			float parentOriginX = (int)texSize*(2-1/(pow(2.0,xLevel))) + 0.5;
			float parentOriginY = (int)texSize*(1-1/(pow(2.0,yLevel-1))) + 0.5;

			if (yLevel>1)
				yLevel--;
			else
				xLevel--;
			
			int viewportX = (int)texSize*(2-1/(pow(2.0,xLevel)));
			int viewportY = (int)texSize*(1-1/(pow(2.0,yLevel-1)));

			cgGLSetParameter2f(glParam->findUpSweepParentOriginParam, parentOriginX, parentOriginY);
			if (xLevel+yLevel >= GetExponent2(texSize))
			{
				cgGLSetParameter1f(glParam->findUpSweepyDirectionParam, 1);
				cgGLSetParameter2f(glParam->findUpSweepOriginParam, viewportX+0.5, viewportY-0.5);
			}
			else
			{
				cgGLSetParameter1f(glParam->findUpSweepyDirectionParam, 0);
				cgGLSetParameter2f(glParam->findUpSweepOriginParam, viewportX-0.5, viewportY+0.5);
			}

			glViewport(viewportX, viewportY, currentWidth, currentHeight);
			DrawQuad(currentWidth, currentHeight);

//			glFlush();
//			glFinish();
#ifdef DUMP
			GLfloat* retVal=new GLfloat[texSize*texSize*2];
			glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
			glReadPixels(0, 0, texSize*2, texSize, GL_RED, GL_FLOAT, retVal);
			WriteToFileInt(retVal,texSize*2,texSize);
#endif
		}
		cgGLDisableTextureParameter(glParam->findUpSweepTextureParam);

		cgGLBindProgram(glParam->findPosChangeProgram);

		cgGLSetTextureParameter(glParam->findPosChangeTextureParam, tempTexture[0]);
		cgGLEnableTextureParameter(glParam->findPosChangeTextureParam);
		cgGLSetTextureParameter(glParam->findPosChangeInputTextureParam, tempTexture[1]);
		cgGLEnableTextureParameter(glParam->findPosChangeInputTextureParam);
		glActiveTextureARB(GL_TEXTURE1_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[1]);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[0]);

		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTexture[1], 0);

		glViewport(0, 0, texSize, texSize);
		DrawQuad(texSize, texSize);
//		glFlush();
//		glFinish();
		cgGLDisableTextureParameter(glParam->findPosChangeTextureParam);
		cgGLDisableTextureParameter(glParam->findPosChangeInputTextureParam);

#ifdef DUMP
		float* retVal1=new GLfloat[texSize*texSize*4];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, texSize, texSize, GL_RGBA, GL_FLOAT, retVal1);
		WriteToFile(retVal1,4,texSize*texSize);
#endif

		GLuint indexVBO;
		glGenBuffersARB(1,&indexVBO);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, indexVBO);
		glBufferDataARB(GL_PIXEL_PACK_BUFFER_ARB, 4*(texSize)*(texSize)*sizeof(float), NULL, GL_DYNAMIC_DRAW);
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, texSize, texSize, GL_RGBA, GL_FLOAT, 0);
		glBindBufferARB(GL_PIXEL_PACK_BUFFER_ARB, 0);

		// Setup Texture
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, tempTexture[0]);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, FRAMEBUFFER_FLOAT_FORMAT, numHitsX, numHitsY, 0, FRAMEBUFFER_FORMAT, GL_FLOAT, NULL);

		// Setup Viewport
		glViewport(0, 0, numHitsX, numHitsY);

		// Setup Projection Matrices
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, numHitsX, 0, numHitsY, -10, 10);
		
		cgGLBindProgram(glParam->passiveProgram);
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);

		// Setup Passive Fragment Program
		cgGLBindProgram(glParam->findScaleProgram);

		float numHitsXY[2];
		numHitsXY[0] = numHitsX;
		numHitsXY[1] = numHitsY;

		// Setup Vertex Program
		cgGLEnableProfile(glParam->cgVertexProfile);
		cgGLBindProgram(glParam->findVertexProgram);
		cgGLSetParameter2fv(glParam->findVertexOutputSizeParam, numHitsXY);

		// Setup Framebuffer
		glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, tempTexture[0], 0);

		// Indices for the VBO
		GLuint* indexArray = new GLuint[texSize*texSize];
		for (int k=0; k<texSize*texSize; k++)
			indexArray[k] = k;

		glEnable(GL_VERTEX_ARRAY);
		glBindBufferARB(GL_ARRAY_BUFFER_ARB, indexVBO);  // BIND VBO
		glVertexPointer(4, GL_FLOAT, 0, 0);
		glDrawElements(GL_POINTS,texSize*texSize,GL_UNSIGNED_INT,indexArray);

		cgGLDisableProfile(glParam->cgVertexProfile);

		glDeleteBuffersARB(1,&indexVBO);
		delete [] indexArray;

#ifdef DUMP
		float* retVal = new GLfloat[numHitsX*numHitsY*4];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0, 0, numHitsX, numHitsY, GL_BLUE, GL_FLOAT, retVal);
		WriteToFile(retVal, numHitsX, numHitsY);
#endif

		// Setup Projection Matrices
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	}

	if(!returnTexture)
	{
		// readback data from texture
		float* pickedBoxIndex = new float[numHitsX*numHitsY*4];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,numHitsX,numHitsY,GL_RGBA,GL_FLOAT,pickedBoxIndex);

		for(int hitCount = 0; hitCount < *numHits ; hitCount++)
			locations->push_back((int)pickedBoxIndex[hitCount*4]+(int)pickedBoxIndex[hitCount*4+1]*texWidth);
		delete [] pickedBoxIndex;
		glDeleteTextures(2, tempTexture);
		return 0;
	}
	else
	{
		glDeleteTextures(1, &tempTexture[1]);
		return tempTexture[0];
	}

#endif
#endif
}

extern float FindMinValue(int inputTexSize, GLuint sourceTex[2], GLParameters* glParam, float location[2])
{
	cgGLBindProgram(glParam->reduceMinProgram);
	
	for(int texSize = inputTexSize/2, count = 0; texSize >= 1; texSize=texSize/2, count++)
	{
		cgGLSetTextureParameter(glParam->reduceMinTextureParam, sourceTex[count%2]);
		cgGLEnableTextureParameter(glParam->reduceMinTextureParam);
		glActiveTextureARB(GL_TEXTURE0_ARB);
		glBindTexture(GL_TEXTURE_RECTANGLE_ARB, sourceTex[count%2]);
		
	    glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_RECTANGLE_ARB, sourceTex[(count+1)%2], 0);
		glViewport(0, 0, texSize, texSize);

		DrawQuad(texSize, texSize);
//		glFlush();
//		glFinish();
		cgGLDisableTextureParameter(glParam->reduceMinTextureParam);

#ifdef DUMP
		GLfloat* retVal=new GLfloat[texSize*texSize];
		glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);
		glReadPixels(0,0,texSize,texSize,GL_GREEN,GL_FLOAT,retVal);
		WriteToFile(retVal,texSize,texSize);
#endif
	}
	float minVal[3];
	glReadPixels(0, 0, 1, 1, GL_RGB, GL_FLOAT, &minVal);
	location[0] = minVal[1];
	location[1] = minVal[2];
	return minVal[0];
}


