//
//  main.cpp
//  OpenGL4Test
//
//  Created by Rafael Radkowski on 5/28/15.
//  Copyright (c) 2015 -. All rights reserved.
//

// stl include
#include <iostream>
#include <string>

// GLEW include
#include <GL/glew.h>

// GLM include files
#define GLM_FORCE_INLINE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


// glfw includes
#include <GLFW/glfw3.h>


// include local files
#include "controls.h"
#include "HCI557Common.h"
//#include <algorithm>



using namespace std;




static const string vs_string =
"#version 410 core                                                 \n"
"                                                                   \n"
"uniform mat4 projectionMatrix;                                    \n"
"uniform mat4 viewMatrix;                                           \n"
"uniform mat4 modelMatrix;                                          \n"
"in vec3 in_Position;                                               \n"
"                                                                   \n"
"in vec3 in_Color;                                                  \n"
"out vec3 pass_Color;                                               \n"
"                                                                  \n"
"void main(void)                                                   \n"
"{                                                                 \n"
"    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(in_Position, 1.0);  \n"
"    pass_Color = in_Color;                                         \n"
"}                                                                 \n";

// Fragment shader source code. This determines the colors in the fragment generated in the shader pipeline. In this case, it colors the inside of our triangle specified by our vertex shader.
static const string fs_string  =
"#version 410 core                                                 \n"
"                                                                  \n"
"in vec3 pass_Color;                                                 \n"
"out vec4 color;                                                    \n"
"void main(void)                                                   \n"
"{                                                                 \n"
"    color = vec4(pass_Color, 1.0);                               \n"
"}                                                                 \n";




/// Camera control matrices
glm::mat4 projectionMatrix; // Store the projection matrix
glm::mat4 viewMatrix; // Store the view matrix
glm::mat4 modelMatrix; // Store the model matrix




// The handle to the window object
GLFWwindow*         window;


// Define some of the global variables we're using for this sample
GLuint program;








///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// Fill this functions with your model code.

// USE THESE vertex array objects to define your objects
unsigned int vaoID[2];
unsigned int vboID[3];



/*!
 ADD YOUR CODE TO CREATE THE TRIANGLE STRIP MODEL TO THIS FUNCTION
 */
unsigned int createTriangleStripModel(void)
{
	int size = 126;
    // use the vertex array object vaoID[0] for this model representation
	float* vertices = new float[size];
	float* colors = new float[size];

	vertices[0] = -1.5;	vertices[1] = -1.0;	vertices[2] = 1.5; // 0
	vertices[3] = -1.5;	vertices[4] = 0.0;	vertices[5] = 1.5; // 1
	vertices[6] = -0.5;	vertices[7] = -1.0;	vertices[8] = 1.5; // 2
	vertices[9] = -0.5; vertices[10] = 0.0; vertices[11] = 1.5; // 3
	vertices[12] = -0.5; vertices[13] = -1.0; vertices[14] = -0.5; // 4
	vertices[15] = -0.5; vertices[16] = 0.0; vertices[17] = -0.5; // 5
	vertices[18] = 1.5; vertices[19] = -1.0; vertices[20] = -0.5; // 6
	vertices[21] = 1.5; vertices[22] = 0.0; vertices[23] = -0.5; // 7
	vertices[24] = 1.5; vertices[25] = -1.0; vertices[26] = -1.5; // 8
	vertices[27] = 1.5; vertices[28] = 0.0; vertices[29] = -1.5; // 9
	
	vertices[30] = -0.5; vertices[31] = -1.0; vertices[32] = -1.5; // 10
	vertices[33] = -0.5; vertices[34] = 0.0; vertices[35] = -1.5; // 11 
	vertices[36] = -1.5; vertices[37] = -1.0; vertices[38] = -1.5; // 12
	vertices[39] = -1.5; vertices[40] = 0.0; vertices[41] = -1.5; // 13
	vertices[42] = -1.5; vertices[43] = -1.0; vertices[44] = -0.5; // 14
	vertices[45] = -1.5; vertices[46] = 0.0; vertices[47] = -0.5; // 15
	vertices[48] = -1.5; vertices[49] = 1.0; vertices[50] = -1.5; // 16
	vertices[51] = -1.5; vertices[52] = 1.0; vertices[53] = -0.5; // 17
	vertices[54] = -0.5; vertices[55] = 1.0; vertices[56] = -1.5; // 18
	vertices[57] = -0.5; vertices[58] = 1.0; vertices[59] = -0.5; // 19
	vertices[60] = 1.5; vertices[61] = 0.0; vertices[62] = -1.5; //9
	vertices[63] = 1.5; vertices[64] = 0.0; vertices[65] = -0.5; //7
	vertices[66] = -0.5; vertices[67] = 0.0; vertices[68] = -0.5; //5
	vertices[69] = -0.5; vertices[70] = 1.0; vertices[71] = -0.5; // 19
	vertices[72] = -1.5; vertices[73] = 0.0; vertices[74] = -0.5; // 15
	vertices[75] = -1.5; vertices[76] = 1.0; vertices[77] = -0.5; // 17
	vertices[78] = -1.5; vertices[79] = 1.0; vertices[80] = -1.5; // 16
	vertices[81] = -1.5; vertices[82] = -1.0; vertices[83] = -1.5; // 12
	vertices[84] = -0.5; vertices[85] = 1.0; vertices[86] = -1.5; // 18
	vertices[87] = 1.5; vertices[88] = 0.0; vertices[89] = -1.5; //9
	vertices[90] = 1.5; vertices[91] = -1.0; vertices[92] = -1.5; // 8
	vertices[93] = -1.5; vertices[94] = -1.0; vertices[95] = -1.5; // 12
	vertices[96] = 1.5; vertices[97] = -1.0; vertices[98] = -0.5; // 6
	vertices[99] = -0.5; vertices[100] = -1.0; vertices[101] = -0.5; // 4
	vertices[102] = -1.5; vertices[103] = -1.0; vertices[104] = -0.5; // 14
	vertices[105] = -0.5;	vertices[106] = -1.0;	vertices[107] = 1.5; // 2
	vertices[108] = -1.5;	vertices[109] = -1.0;	vertices[110] = 1.5; // 0
	vertices[111] = -1.5; vertices[112] = -1.0; vertices[113] = -0.5; // 14
	vertices[114] = -1.5;	vertices[115] = 0.0;	vertices[116] = 1.5; // 1
	vertices[117] = -1.5; vertices[118] = 0.0; vertices[119] = -0.5; // 15
	vertices[120] = -0.5; vertices[121] = 0.0; vertices[122] = 1.5; // 3
	vertices[123] = -0.5; vertices[124] = 0.0; vertices[125] = -0.5; // 5
	
	
	colors[0] = 1.0; colors[1] = 0.0; colors[2] = 0.0;
	colors[3] = 1.0; colors[4] = 0.0; colors[5] = 0.0;
	colors[6] = 1.0; colors[7] = 0.0; colors[8] = 0.0;
	colors[9] = 1.0; colors[10] = 0.0; colors[11] = 0.0;
	colors[12] = 1.0; colors[13] = 0.0; colors[14] = 0.0;
	colors[15] = 1.0; colors[16] = 0.0; colors[17] = 0.0;
	colors[18] = 1.0; colors[19] = 0.0; colors[20] = 0.0;
	colors[21] = 1.0; colors[22] = 0.0; colors[23] = 0.0;
	colors[24] = 1.0; colors[25] = 0.0; colors[26] = 0.0;
	colors[27] = 1.0; colors[28] = 0.0; colors[29] = 0.0;

	colors[30] = 1.0; colors[31] = 0.0; colors[32] = 0.0;
	colors[33] = 1.0; colors[34] = 0.0; colors[35] = 0.0;
	colors[36] = 1.0; colors[37] = 0.0; colors[38] = 0.0;
	colors[39] = 1.0; colors[40] = 0.0; colors[41] = 0.0;
	colors[42] = 1.0; colors[43] = 0.0; colors[44] = 0.0;
	colors[45] = 1.0; colors[46] = 0.0; colors[47] = 0.0;
	colors[48] = 1.0; colors[49] = 0.0; colors[50] = 0.0;
	colors[51] = 1.0; colors[52] = 0.0; colors[53] = 0.0;
	colors[54] = 1.0; colors[55] = 0.0; colors[56] = 0.0;
	colors[57] = 1.0; colors[58] = 0.0; colors[59] = 0.0;
	colors[60] = 1.0; colors[61] = 0.0; colors[62] = 0.0;
	colors[63] = 1.0; colors[64] = 0.0; colors[65] = 0.0;
	colors[66] = 1.0; colors[67] = 0.0; colors[68] = 0.0;
	colors[69] = 1.0; colors[70] = 0.0; colors[71] = 0.0;
	colors[72] = 1.0; colors[73] = 0.0; colors[74] = 0.0;
	colors[75] = 1.0; colors[76] = 0.0; colors[77] = 0.0;
	colors[78] = 1.0; colors[79] = 0.0; colors[80] = 0.0;
	colors[81] = 1.0; colors[82] = 0.0; colors[83] = 0.0;
	colors[84] = 1.0; colors[85] = 0.0; colors[86] = 0.0;
	colors[87] = 1.0; colors[88] = 0.0; colors[89] = 0.0;
	colors[90] = 1.0; colors[91] = 0.0; colors[92] = 0.0;
	colors[93] = 1.0; colors[94] = 0.0; colors[95] = 0.0;
	colors[96] = 1.0; colors[97] = 0.0; colors[98] = 0.0;
	colors[99] = 1.0; colors[100] = 0.0; colors[101] = 0.0;
	colors[102] = 1.0; colors[103] = 0.0; colors[104] = 0.0;
	colors[105] = 1.0; colors[106] = 0.0; colors[107] = 0.0;
	colors[108] = 1.0; colors[109] = 0.0; colors[110] = 0.0;
	colors[111] = 1.0; colors[112] = 0.0; colors[113] = 0.0;
	colors[114] = 1.0; colors[115] = 0.0; colors[116] = 0.0;
	colors[117] = 1.0; colors[118] = 0.0; colors[119] = 0.0;
	colors[120] = 1.0; colors[121] = 0.0; colors[122] = 0.0;
	colors[123] = 1.0; colors[124] = 0.0; colors[125] = 0.0;




    //TODO:
	vaoID[0];
	vboID[0];
	glGenVertexArrays(2, &vaoID[0]); // Create Vertex Array Objects
	glBindVertexArray(vaoID[0]);  // Bind first Vertex Array Object

	glGenBuffers(2, vboID); // Create Vertex Buffer Objects

	// Vertices
	glBindBuffer(GL_ARRAY_BUFFER, vboID[0]); // Bind Vertex Buffer Object
	glBufferData(GL_ARRAY_BUFFER, size * sizeof(GLfloat), vertices, GL_STATIC_DRAW); // Set the Size and data of VBO

	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0); // Vertex Attributes Pointer
	glEnableVertexAttribArray(0); // Disable Vertex Array Object

	// Color
	glBindBuffer(GL_ARRAY_BUFFER, vboID[1]);
	glBufferData(GL_ARRAY_BUFFER, size * sizeof(GLfloat), colors, GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);


	//delete[] vertices;
    return 1;
}

/*!
 ADD YOUR CODE TO CREATE A MODEL USING PRIMITIVES OF YOUR CHOISE TO THIS FUNCTION
 */
unsigned int createMyModel(void)
{
    // use the vertex array object vaoID[1] for this model representation
	int size = 306;
	// use the vertex array object vaoID[0] for this model representation
	float* vertices = new float[size];
	float* colors = new float[size];

	vertices[0] = -1.5;	vertices[1] = -1.0;	vertices[2] = 1.5; // 0
	vertices[3] = -1.5;	vertices[4] = 0.0;	vertices[5] = 1.5; // 1
	vertices[6] = -0.5;	vertices[7] = -1.0;	vertices[8] = 1.5; // 2

	vertices[9] = -1.5;	vertices[10] = 0.0;	vertices[11] = 1.5; // 1
	vertices[12] = -0.5; vertices[13] = -1.0; vertices[14] = 1.5; // 2
	vertices[15] = -0.5; vertices[16] = 0.0; vertices[17] = 1.5; // 3

	vertices[18] = -0.5; vertices[19] = -1.0; vertices[20] = 1.5; // 2
	vertices[21] = -0.5; vertices[22] = 0.0; vertices[23] = 1.5; // 3
	vertices[24] = -0.5; vertices[25] = -1.0; vertices[26] = -0.5; // 4

	vertices[27] = -0.5; vertices[28] = 0.0; vertices[29] = 1.5; // 3
	vertices[30] = -0.5; vertices[31] = -1.0; vertices[32] = -0.5; // 4
	vertices[33] = -0.5; vertices[34] = 0.0; vertices[35] = -0.5; // 5

	vertices[36] = -0.5; vertices[37] = -1.0; vertices[38] = -0.5; // 4
	vertices[39] = -0.5; vertices[40] = 0.0; vertices[41] = -0.5; // 5
	vertices[42] = 1.5; vertices[43] = -1.0; vertices[44] = -0.5; // 6

	vertices[45] = -0.5; vertices[46] = 0.0; vertices[47] = -0.5; // 5
	vertices[48] = 1.5; vertices[49] = -1.0; vertices[50] = -0.5; // 6
	vertices[51] = 1.5; vertices[52] = 0.0; vertices[53] = -0.5; // 7

	vertices[54] = 1.5; vertices[55] = -1.0; vertices[56] = -0.5; // 6
	vertices[57] = 1.5; vertices[58] = 0.0; vertices[59] = -0.5; // 7
	vertices[60] = 1.5; vertices[61] = -1.0; vertices[62] = -1.5; // 8

	vertices[63] = 1.5; vertices[64] = 0.0; vertices[65] = -0.5; // 7
	vertices[66] = 1.5; vertices[67] = -1.0; vertices[68] = -1.5; // 8
	vertices[69] = 1.5; vertices[70] = 0.0; vertices[71] = -1.5; // 9

	vertices[72] = 1.5; vertices[73] = -1.0; vertices[74] = -1.5; // 8
	vertices[75] = 1.5; vertices[76] = 0.0; vertices[77] = -1.5; // 9
	vertices[78] = -0.5; vertices[79] = -1.0; vertices[80] = -1.5; // 10

	vertices[81] = 1.5; vertices[82] = 0.0; vertices[83] = -1.5; // 9
	vertices[84] = -0.5; vertices[85] = -1.0; vertices[86] = -1.5; // 10
	vertices[87] = -0.5; vertices[88] = 0.0; vertices[89] = -1.5; // 11 

	vertices[90] = -0.5; vertices[91] = -1.0; vertices[92] = -1.5; // 10
	vertices[93] = -0.5; vertices[94] = 0.0; vertices[95] = -1.5; // 11 
	vertices[96] = -1.5; vertices[97] = -1.0; vertices[98] = -1.5; // 12

	vertices[99] = -0.5; vertices[100] = 0.0; vertices[101] = -1.5; // 11 
	vertices[102] = -1.5; vertices[103] = -1.0; vertices[104] = -1.5; // 12
	vertices[105] = -1.5; vertices[106] = 0.0; vertices[107] = -1.5; // 13

	vertices[108] = -1.5; vertices[109] = -1.0; vertices[110] = -1.5; // 12
	vertices[111] = -1.5; vertices[112] = 0.0; vertices[113] = -1.5; // 13
	vertices[114] = -1.5; vertices[115] = -1.0; vertices[116] = -0.5; // 14

	vertices[117] = -1.5; vertices[118] = 0.0; vertices[119] = -1.5; // 13
	vertices[120] = -1.5; vertices[121] = -1.0; vertices[122] = -0.5; // 14
	vertices[123] = -1.5; vertices[124] = 0.0; vertices[125] = -0.5; // 15

	vertices[126] = -1.5; vertices[127] = -1.0; vertices[128] = -0.5; // 14
	vertices[129] = -1.5; vertices[130] = 0.0; vertices[131] = -0.5; // 15
	vertices[132] = -1.5; vertices[133] = 1.0; vertices[134] = -1.5; // 16

	vertices[135] = -1.5; vertices[136] = 0.0; vertices[137] = -0.5; // 15
	vertices[138] = -1.5; vertices[139] = 1.0; vertices[140] = -1.5; // 16
	vertices[141] = -1.5; vertices[142] = 1.0; vertices[143] = -0.5; // 17

	vertices[144] = -1.5; vertices[145] = 1.0; vertices[146] = -1.5; // 16
	vertices[147] = -1.5; vertices[148] = 1.0; vertices[149] = -0.5; // 17
	vertices[150] = -0.5; vertices[151] = 1.0; vertices[152] = -1.5; // 18

	vertices[153] = -1.5; vertices[154] = 1.0; vertices[155] = -0.5; // 17
	vertices[156] = -0.5; vertices[157] = 1.0; vertices[158] = -1.5; // 18
	vertices[159] = -0.5; vertices[160] = 1.0; vertices[161] = -0.5; // 19

	vertices[162] = 1.5; vertices[163] = 0.0; vertices[164] = -0.5; // 7
	vertices[165] = 1.5; vertices[166] = 0.0; vertices[167] = -1.5; // 9
	vertices[168] = -0.5; vertices[169] = 1.0; vertices[170] = -0.5; // 19

	vertices[171] = 1.5; vertices[172] = 0.0; vertices[173] = -1.5; // 9
	vertices[174] = -0.5; vertices[175] = 1.0; vertices[176] = -0.5; // 19
	vertices[177] = -0.5; vertices[178] = 1.0; vertices[179] = -1.5; // 18

	vertices[180] = 1.5; vertices[181] = 0.0; vertices[182] = -1.5; // 9
	vertices[183] = -0.5; vertices[184] = 1.0; vertices[185] = -1.5; // 18
	vertices[186] = -0.5; vertices[187] = 0.0; vertices[188] = -1.5; // 11 

	vertices[189] = -0.5; vertices[190] = 1.0; vertices[191] = -1.5; // 18
	vertices[192] = -0.5; vertices[193] = 0.0; vertices[194] = -1.5; // 11 
	vertices[195] = -1.5; vertices[196] = 0.0; vertices[197] = -1.5; // 13

	vertices[198] = -0.5; vertices[199] = 1.0; vertices[200] = -1.5; // 18
	vertices[201] = -1.5; vertices[202] = 0.0; vertices[203] = -1.5; // 13
	vertices[204] = -1.5; vertices[205] = 1.0; vertices[206] = -1.5; // 16

	vertices[207] = -1.5; vertices[208] = 0.0; vertices[209] = -1.5; // 13
	vertices[210] = -1.5; vertices[211] = 1.0; vertices[212] = -1.5; // 16
	vertices[213] = -1.5; vertices[214] = 0.0; vertices[215] = -0.5; // 15

	vertices[216] = -1.5;	vertices[217] = 0.0;	vertices[218] = 1.5; // 1
	vertices[219] = -0.5; vertices[220] = 0.0; vertices[221] = 1.5; // 3
	vertices[222] = -0.5; vertices[223] = 0.0; vertices[224] = -0.5; // 5

	vertices[225] = -1.5;	vertices[226] = 0.0;	vertices[227] = 1.5; // 1
	vertices[228] = -0.5; vertices[229] = 0.0; vertices[230] = -0.5; // 5
	vertices[231] = -1.5; vertices[232] = 0.0; vertices[233] = -0.5; // 15

	vertices[234] = -1.5;	vertices[235] = -1.0;	vertices[236] = 1.5; // 0
	vertices[237] = -1.5;	vertices[238] = 0.0;	vertices[239] = 1.5; // 1
	vertices[240] = -1.5; vertices[241] = 0.0; vertices[242] = -0.5; // 15

	vertices[243] = -1.5;	vertices[244] = -1.0;	vertices[245] = 1.5; // 0
	vertices[246] = -1.5; vertices[247] = -1.0; vertices[248] = -0.5; // 14
	vertices[249] = -1.5; vertices[250] = 0.0; vertices[251] = -0.5; // 15

	vertices[252] = -1.5;	vertices[253] = -1.0;	vertices[254] = 1.5; // 0
	vertices[255] = -1.5; vertices[256] = -1.0; vertices[257] = -0.5; // 14
	vertices[258] = -0.5; vertices[259] = -1.0; vertices[260] = 1.5; // 2

	vertices[261] = -1.5; vertices[262] = -1.0; vertices[263] = -0.5; // 14
	vertices[264] = -0.5; vertices[265] = -1.0; vertices[266] = 1.5; // 2
	vertices[267] = -0.5; vertices[268] = -1.0; vertices[269] = -0.5; // 4

	vertices[270] = -1.5; vertices[271] = -1.0; vertices[272] = -0.5; // 14
	vertices[273] = -1.5; vertices[274] = -1.0; vertices[275] = -1.5; // 12
	vertices[276] = 1.5; vertices[277] = -1.0; vertices[278] = -1.5; // 8

	vertices[279] = 1.5; vertices[280] = -1.0; vertices[281] = -1.5; // 8
	vertices[282] = 1.5; vertices[283] = -1.0; vertices[284] = -0.5; // 6
	vertices[285] = -1.5; vertices[286] = -1.0; vertices[287] = -0.5; // 14

	vertices[288] = -1.5; vertices[289] = 0.0; vertices[290] = -0.5; // 15
	vertices[291] = -1.5; vertices[292] = 1.0; vertices[293] = -0.5; // 17
	vertices[294] = -0.5; vertices[295] = 1.0; vertices[296] = -0.5; // 19

	vertices[297] = 1.5; vertices[298] = 0.0; vertices[299] = -0.5; // 7
	vertices[300] = -0.5; vertices[301] = 1.0; vertices[302] = -0.5; // 19
	vertices[303] = -0.5; vertices[304] = 0.0; vertices[305] = -0.5; // 5


	colors[0] = 1.0; colors[1] = 0.0; colors[2] = 0.0;
	colors[3] = 1.0; colors[4] = 0.0; colors[5] = 0.0;
	colors[6] = 1.0; colors[7] = 0.0; colors[8] = 0.0;
	colors[9] = 1.0; colors[10] = 0.0; colors[11] = 0.0;
	colors[12] = 1.0; colors[13] = 0.0; colors[14] = 0.0;
	colors[15] = 1.0; colors[16] = 0.0; colors[17] = 0.0;
	colors[18] = 1.0; colors[19] = 0.0; colors[20] = 0.0;
	colors[21] = 1.0; colors[22] = 0.0; colors[23] = 0.0;
	colors[24] = 1.0; colors[25] = 0.0; colors[26] = 0.0;
	colors[27] = 1.0; colors[28] = 0.0; colors[29] = 0.0;

	colors[30] = 0.5; colors[31] = 0.0; colors[32] = 0.0;
	colors[33] = 0.5; colors[34] = 0.0; colors[35] = 0.0;
	colors[36] = 0.5; colors[37] = 0.0; colors[38] = 0.0;
	colors[39] = 0.5; colors[40] = 0.0; colors[41] = 0.0;
	colors[42] = 0.5; colors[43] = 0.0; colors[44] = 0.0;
	colors[45] = 0.5; colors[46] = 0.0; colors[47] = 0.0;
	colors[48] = 0.5; colors[49] = 0.0; colors[50] = 0.0;
	colors[51] = 0.5; colors[52] = 0.0; colors[53] = 0.0;
	colors[54] = 0.5; colors[55] = 0.0; colors[56] = 0.0;
	colors[57] = 0.5; colors[58] = 0.0; colors[59] = 0.0;

	colors[60] = 1.0; colors[61] = 0.0; colors[62] = 0.0;
	colors[63] = 1.0; colors[64] = 0.0; colors[65] = 0.0;
	colors[66] = 1.0; colors[67] = 0.0; colors[68] = 0.0;
	colors[69] = 1.0; colors[70] = 0.0; colors[71] = 0.0;
	colors[72] = 1.0; colors[73] = 0.0; colors[74] = 0.0;
	colors[75] = 1.0; colors[76] = 0.0; colors[77] = 0.0;
	colors[78] = 1.0; colors[79] = 0.0; colors[80] = 0.0;
	colors[81] = 1.0; colors[82] = 0.0; colors[83] = 0.0;
	colors[84] = 1.0; colors[85] = 0.0; colors[86] = 0.0;
	colors[87] = 1.0; colors[88] = 0.0; colors[89] = 0.0;

	colors[90] = 1.0; colors[91] = 0.0; colors[92] = 0.0;
	colors[93] = 1.0; colors[94] = 0.0; colors[95] = 0.0;
	colors[96] = 1.0; colors[97] = 0.0; colors[98] = 0.0;
	colors[99] = 1.0; colors[100] = 0.0; colors[101] = 0.0;
	colors[102] = 1.0; colors[103] = 0.0; colors[104] = 0.0;
	colors[105] = 1.0; colors[106] = 0.0; colors[107] = 0.0;
	colors[108] = 1.0; colors[109] = 0.0; colors[110] = 0.0;
	colors[111] = 1.0; colors[112] = 0.0; colors[113] = 0.0;
	colors[114] = 1.0; colors[115] = 0.0; colors[116] = 0.0;
	colors[117] = 1.0; colors[118] = 0.0; colors[119] = 0.0;

	colors[120] = 1.0; colors[121] = 0.0; colors[122] = 0.0;
	colors[123] = 1.0; colors[124] = 0.0; colors[125] = 0.0;
	colors[126] = 1.0; colors[127] = 0.0; colors[128] = 0.0;
	colors[129] = 1.0; colors[130] = 0.0; colors[131] = 0.0;
	colors[132] = 1.0; colors[133] = 0.0; colors[134] = 0.0;
	colors[135] = 1.0; colors[136] = 0.0; colors[137] = 0.0;
	colors[138] = 1.0; colors[139] = 0.0; colors[140] = 0.0;
	colors[141] = 1.0; colors[142] = 0.0; colors[143] = 0.0;
	colors[144] = 1.0; colors[145] = 0.0; colors[146] = 0.0;
	colors[147] = 1.0; colors[148] = 0.0; colors[149] = 0.0;

	colors[150] = 1.0; colors[151] = 0.0; colors[152] = 0.0;
	colors[153] = 1.0; colors[154] = 0.0; colors[155] = 0.0;
	colors[156] = 1.0; colors[157] = 0.0; colors[158] = 0.0;
	colors[159] = 1.0; colors[160] = 0.0; colors[161] = 0.0;
	colors[162] = 0.5; colors[163] = 0.0; colors[164] = 0.0;
	colors[165] = 0.5; colors[166] = 0.0; colors[167] = 0.0;
	colors[168] = 0.5; colors[169] = 0.0; colors[170] = 0.0;
	colors[171] = 1.0; colors[172] = 0.0; colors[173] = 0.0;
	colors[174] = 1.0; colors[175] = 0.0; colors[176] = 0.0;
	colors[177] = 1.0; colors[178] = 0.0; colors[179] = 0.0;

	colors[180] = 1.0; colors[181] = 0.0; colors[182] = 0.0;
	colors[183] = 1.0; colors[184] = 0.0; colors[185] = 0.0;
	colors[186] = 1.0; colors[187] = 0.0; colors[188] = 0.0;
	colors[189] = 1.0; colors[190] = 0.0; colors[191] = 0.0;
	colors[192] = 1.0; colors[193] = 0.0; colors[194] = 0.0;
	colors[195] = 1.0; colors[196] = 0.0; colors[197] = 0.0;
	colors[198] = 1.0; colors[199] = 0.0; colors[200] = 0.0;
	colors[201] = 1.0; colors[202] = 0.0; colors[203] = 0.0;
	colors[204] = 1.0; colors[205] = 0.0; colors[206] = 0.0;
	colors[207] = 1.0; colors[208] = 0.0; colors[209] = 0.0;

	colors[210] = 1.0; colors[211] = 0.0; colors[212] = 0.0;
	colors[213] = 1.0; colors[214] = 0.0; colors[215] = 0.0;
	colors[216] = 1.0; colors[217] = 0.0; colors[218] = 0.0;
	colors[219] = 1.0; colors[220] = 0.0; colors[221] = 0.0;
	colors[222] = 1.0; colors[223] = 0.0; colors[224] = 0.0;
	colors[225] = 1.0; colors[226] = 0.0; colors[227] = 0.0;
	colors[228] = 1.0; colors[229] = 0.0; colors[230] = 0.0;
	colors[231] = 1.0; colors[232] = 0.0; colors[233] = 0.0;
	colors[234] = 1.0; colors[235] = 0.0; colors[236] = 0.0;
	colors[237] = 1.0; colors[238] = 0.0; colors[239] = 0.0;

	colors[240] = 1.0; colors[241] = 0.0; colors[242] = 0.0;
	colors[243] = 1.0; colors[244] = 0.0; colors[245] = 0.0;
	colors[246] = 1.0; colors[247] = 0.0; colors[248] = 0.0;
	colors[249] = 1.0; colors[250] = 0.0; colors[251] = 0.0;
	colors[252] = 1.0; colors[253] = 0.0; colors[254] = 0.0;
	colors[255] = 1.0; colors[256] = 0.0; colors[257] = 0.0;
	colors[258] = 1.0; colors[259] = 0.0; colors[260] = 0.0;
	colors[261] = 1.0; colors[262] = 0.0; colors[263] = 0.0;
	colors[264] = 1.0; colors[265] = 0.0; colors[266] = 0.0;
	colors[267] = 1.0; colors[268] = 0.0; colors[269] = 0.0;

	colors[270] = 1.0; colors[271] = 0.0; colors[272] = 0.0;
	colors[273] = 1.0; colors[274] = 0.0; colors[275] = 0.0;
	colors[276] = 1.0; colors[277] = 0.0; colors[278] = 0.0;
	colors[279] = 1.0; colors[280] = 0.0; colors[281] = 0.0;
	colors[282] = 1.0; colors[283] = 0.0; colors[284] = 0.0;
	colors[285] = 1.0; colors[286] = 0.0; colors[287] = 0.0;

	colors[288] = 0.5; colors[289] = 0.0; colors[290] = 0.0;
	colors[291] = 0.5; colors[292] = 0.0; colors[293] = 0.0;
	colors[294] = 0.5; colors[295] = 0.0; colors[296] = 0.0;

	colors[297] = 1.0; colors[298] = 0.0; colors[299] = 0.0;
	colors[300] = 1.0; colors[301] = 0.0; colors[302] = 0.0;
	colors[303] = 1.0; colors[304] = 0.0; colors[305] = 0.0;


    //TODO:
    vaoID[1];
    
	glGenVertexArrays(2, &vaoID[1]); // Create Vertex Array Objects
	glBindVertexArray(vaoID[1]);  // Bind first Vertex Array Object

	glGenBuffers(2, vboID); // Create Vertex Buffer Objects

	// Vertices
	glBindBuffer(GL_ARRAY_BUFFER, vboID[0]); // Bind Vertex Buffer Object
	glBufferData(GL_ARRAY_BUFFER, size * sizeof(GLfloat), vertices, GL_STATIC_DRAW); // Set the Size and data of VBO

	glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0); // Vertex Attributes Pointer
	glEnableVertexAttribArray(0); // Disable Vertex Array Object

	// Color
	glBindBuffer(GL_ARRAY_BUFFER, vboID[1]);
	glBufferData(GL_ARRAY_BUFFER, size * sizeof(GLfloat), colors, GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);

    return 1;
}



/*!
 ADD YOUR CODE TO RENDER THE TRIANGLE STRIP MODEL TO THIS FUNCTION
 */
void renderTriangleStripModel(void)
{
	glUseProgram(program);
	glBindVertexArray(vaoID[0]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 42);
	glBindVertexArray(0);
	glUseProgram(0);
    
}

/*!
 ADD YOUR CODE TO RENDER YOUR MODEL TO THIS FUNCTION
 */
void renderMyModel(void)
{
	glUseProgram(program);
	glBindVertexArray(vaoID[1]);
	glDrawArrays(GL_TRIANGLES, 0, 102);
	glBindVertexArray(0);
	glUseProgram(0);

    
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*!
 This function creates the two models
 */
void setupScene(void) {
    
    createTriangleStripModel();
    createMyModel();
    
}




int main(int argc, const char * argv[])
{
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Init glfw, create a window, and init glew
    
    // Init the GLFW Window
    window = initWindow();
    
    
    // Init the glew api
    initGlew();
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// The Shader Program starts here
    
    // Vertex shader source code. This draws the vertices in our window. We have 3 vertices since we're drawing an triangle.
    // Each vertex is represented by a vector of size 4 (x, y, z, w) coordinates.
    static const string vertex_code = vs_string;
    static const char * vs_source = vertex_code.c_str();
    
    // Fragment shader source code. This determines the colors in the fragment generated in the shader pipeline. In this case, it colors the inside of our triangle specified by our vertex shader.
    static const string fragment_code = fs_string;
    static const char * fs_source = fragment_code.c_str();
    
    // This next section we'll generate the OpenGL program and attach the shaders to it so that we can render our triangle.
    program = glCreateProgram();
    
    // We create a shader with our fragment shader source code and compile it.
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fs_source, NULL);
    glCompileShader(fs);
    
    // We create a shader with our vertex shader source code and compile it.
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vs_source, NULL);
    glCompileShader(vs);
    
    // We'll attach our two compiled shaders to the OpenGL program.
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    
    glLinkProgram(program);
    
    // We'll specify that we want to use this program that we've attached the shaders to.
    glUseProgram(program);
    
    //// The Shader Program ends here
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    /// IGNORE THE NEXT PART OF THIS CODE
    /// IGNORE THE NEXT PART OF THIS CODE
    /// IGNORE THE NEXT PART OF THIS CODE
    // It controls the virtual camera
    
    // Set up our green background color
    static const GLfloat clear_color[] = { 0.6f, 0.7f, 1.0f, 1.0f };
    static const GLfloat clear_depth[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    
    
    projectionMatrix = glm::perspective(1.1f, (float)800 / (float)600, 0.1f, 100.f);
    modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)); // Create our model matrix which will halve the size of our model
    viewMatrix = glm::lookAt(glm::vec3(1.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    
    int projectionMatrixLocation = glGetUniformLocation(program, "projectionMatrix"); // Get the location of our projection matrix in the shader
    int viewMatrixLocation = glGetUniformLocation(program, "viewMatrix"); // Get the location of our view matrix in the shader
    int modelMatrixLocation = glGetUniformLocation(program, "modelMatrix"); // Get the location of our model matrix in the shader
    
    
    glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, &projectionMatrix[0][0]); // Send our projection matrix to the shader
    glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &viewMatrix[0][0]); // Send our view matrix to the shader
    glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &modelMatrix[0][0]); // Send our model matrix to the shader
    
    
    glBindAttribLocation(program, 0, "in_Position");
    glBindAttribLocation(program, 1, "in_Color");
    
    //// The Shader Program ends here
    //// START TO READ AGAIN
    //// START TO READ AGAIN
    //// START TO READ AGAIN
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    // this creates the scene
    setupScene();
    
    

    // Enable depth test
    // ignore this line, it allows us to keep the distance value after we proejct each object to a 2d canvas.
    glEnable(GL_DEPTH_TEST);
    
    // This is our render loop. As long as our window remains open (ESC is not pressed), we'll continue to render things.
    while(!glfwWindowShouldClose(window))
    {
        
        // Clear the entire buffer with our green color (sets the background to be green).
        glClearBufferfv(GL_COLOR , 0, clear_color);
        glClearBufferfv(GL_DEPTH , 0, clear_depth);
        
        

        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //// This generate the object
        // Enable the shader program
        glUseProgram(program);
        
        // this changes the camera location
        glm::mat4 rotated_view = viewMatrix * GetRotationMatrix();
        glUniformMatrix4fv(viewMatrixLocation, 1, GL_FALSE, &rotated_view[0][0]); // send the view matrix to our shader
        
        // This moves the model to the right
        modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, 0.0f, 0.0f));
        glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &modelMatrix[0][0]); // Send our model matrix to the shader
        
        //renderTriangleStripModel();
        
        // This moves the model to the left
        modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(-2.0f, -0.0f, 0.0f));
        glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, &modelMatrix[0][0]); // Send our model matrix to the shader
        
        renderMyModel();
        
        
        glUseProgram(0);
        //// This generate the object
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        
        // Swap the buffers so that what we drew will appear on the screen.
        glfwSwapBuffers(window);
        glfwPollEvents();
        
    }
    
    // Program clean up when the window gets closed.
    glDeleteVertexArrays(2, vaoID);
    glDeleteProgram(program);
}

