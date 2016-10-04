//
//  main_spotlight.cpp
//  HCI 557 Spotlight example
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
#include <glm/gtx/transform.hpp>


// glfw includes
#include <GLFW/glfw3.h>


// include local files
#include "controls.h"
#include "HCI557Common.h"
#include "CoordSystem.h"
#include "Sphere3D.h"
#include "GLAppearance.h"




using namespace std;


// The handle to the window object
GLFWwindow*         window;

// Define some of the global variables we're using for this sample
GLuint program;

/* A trackball to move and rotate the camera view */
extern Trackball trackball;



int main(int argc, const char * argv[])
{
    
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Init glfw, create a window, and init glew
    
    // Init the GLFW Window
    window = initWindow();
    
    
    // Init the glew api
    initGlew();
    
    

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Create some models
    
    // coordinate system
    CoordSystem* cs = new CoordSystem(40.0);

    
    // create an apperance object.
    GLAppearance* apperance = new GLAppearance("../../../data/shaders/multi_vertex_lights.vs", "../../../data/shaders/multi_vertex_lights.fs");
    
	// --------------- 3 more Apperance Objects ----------------------
	GLAppearance* apperance2 = new GLAppearance("../../../data/shaders/multi_vertex_lights.vs", "../../../data/shaders/multi_vertex_lights.fs");
	GLAppearance* apperance3 = new GLAppearance("../../../data/shaders/multi_vertex_lights.vs", "../../../data/shaders/multi_vertex_lights.fs");
	GLAppearance* apperance4 = new GLAppearance("../../../data/shaders/multi_vertex_lights.vs", "../../../data/shaders/multi_vertex_lights.fs");
	// ----------------------------------------------------------------
    
    // ----------------- Light for Green Sphere ----------------
    GLSpotLightSource  light_source1;
    light_source1._lightPos = glm::vec4(6.0,17.0,7.0, 1.0);
    light_source1._ambient_intensity = 0.0;
    light_source1._specular_intensity = 4.0;
    light_source1._diffuse_intensity = 3.0;
    light_source1._attenuation_coeff = 0.01;
    light_source1._cone_angle = 20.0; // in degree
    light_source1._cone_direction = glm::vec3(0.0, -1.0, -0.5); // this must be aligned with the object and light position.
    // --------------------------------------------------------
    
	// ----------------- Light for Blue Sphere --------------------
    GLDirectLightSource  light_source2;
    light_source2._lightPos = glm::vec4(1.0,0.0,3.0, 0.0);
    light_source2._ambient_intensity = 0.1;
    light_source2._specular_intensity = 5.5;
    light_source2._diffuse_intensity = 1.0;
    light_source2._attenuation_coeff = 0.02;
   // ------------------------------------------------------------

	// ---------------- Light for Red Sphere ---------------------
	GLDirectLightSource light_source3;
	light_source3._lightPos = glm::vec4(-24.0, 13.0, 9.0, 1.0);
	light_source3._ambient_intensity = 0.1;
	light_source3._specular_intensity = 7.5;
	light_source3._diffuse_intensity = 1.0;
	light_source3._attenuation_coeff = 0.02;
	// -----------------------------------------------------------

	// ---------------- Lights for Yellow Sphere -------------------
	GLSpotLightSource light_source4;
	light_source4._lightPos = glm::vec4(21.0, 17.0, 2.0, 1.0);
	light_source4._ambient_intensity = 0.1;
	light_source4._specular_intensity = 1.5;
	light_source4._diffuse_intensity = 1.0;
	light_source4._attenuation_coeff = 0.02;
	light_source4._cone_angle = 13.0;
	light_source4._cone_direction = glm::vec3(0.0, -1.0, 0.0);

	GLDirectLightSource light_source5;
	light_source5._lightPos = glm::vec4(19.0, 0.0, 7.0, 1.0);
	light_source5._ambient_intensity = 0.1;
	light_source5._specular_intensity = 1.5;
	light_source5._diffuse_intensity = 0.5;
	light_source5._attenuation_coeff = 0.02;
	// ----------------------------------------------------------

    // add the spot light to this apperance object
    apperance->addLightSource(light_source1); // Green Sphere
   
	// ---------------- Add light sources to 3 more apperances objects --------------------------
	apperance2->addLightSource(light_source4); // Yellow Sphere
	apperance2->addLightSource(light_source5);

	apperance3->addLightSource(light_source2); // Blue Sphere

	apperance4->addLightSource(light_source3); // Red Sphere
	// -------------------------------------------------------------------------------------------

    // Create a material object
    GLMaterial material; // Green Sphere
    material._diffuse_material = glm::vec3(0.0, 1.0, 0.0);
    material._ambient_material = glm::vec3(0.0, 0.0, 0.0);
    material._specular_material = glm::vec3(0.0, 1.0, 0.0);
    material._shininess = 12.0;

	// --------------- 3 more Material Objects ---------------------
	GLMaterial material2; // Yellow Sphere
	material2._diffuse_material = glm::vec3(1.0, 1.0, 0.2);
	material2._ambient_material = glm::vec3(1.0, 1.0, 0.2);
	material2._specular_material = glm::vec3(1.0, 1.0, 0.2);
	material2._shininess = 10.0;

	GLMaterial material3; // Blue Sphere
	material3._diffuse_material = glm::vec3(0.0, 0.0, 1.0);
	material3._ambient_material = glm::vec3(0.0, 0.0, 1.0);
	material3._specular_material = glm::vec3(0.0, 0.0, 0.0);
	material3._shininess = 10.0;
	
	GLMaterial material4; // Red Sphere
	material4._diffuse_material = glm::vec3(1.0, 0.0, 0.0);
	material4._ambient_material = glm::vec3(1.0, 0.0, 0.0);
	material4._specular_material = glm::vec3(1.0, 0.2, 0.2);
	material4._shininess = 13.0;

    // Add the material to the apperance object
    apperance->setMaterial(material); // Green Sphere
    apperance->finalize();
    
	// ---------- Set 3 more appearance materials ------------
	apperance2->setMaterial(material2); // Yellow Sphere
	apperance2->finalize();

	apperance3->setMaterial(material3); // Blue Sphere
	apperance3->finalize();

	apperance4->setMaterial(material4); // Red Sphere
	apperance4->finalize();
	// -------------------------------------------------------

    // create the sphere geometry
    GLSphere3D* sphere = new GLSphere3D(6.0, 0.0, 0.0, 5.0, 90, 50); // third sphere
    sphere->setApperance(*apperance);								 // Green Sphere
    sphere->init();

	// -------------- 3 more Spheres ----------------------
	GLSphere3D* sphere2 = new GLSphere3D(21.0, 0.0, 0.0, 5.0, 90, 50); // fourth sphere
	sphere2->setApperance(*apperance2);									// Yellow Sphere
	sphere2->init();
    
	GLSphere3D* sphere3 = new GLSphere3D(-9.0, 0.0, 0.0, 5.0, 90, 50); // second sphere
	sphere3->setApperance(*apperance3);									// Blue Sphere
	sphere3->init();

	GLSphere3D* sphere4 = new GLSphere3D(-24.0, 0.0, 0.0, 5.0, 90, 50); // first sphere
	sphere4->setApperance(*apperance4);									// Red Sphere
	sphere4->init();
    // ---------------------------------------------------

    
    
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Main render loop
    
    // Set up our green background color
    static const GLfloat clear_color[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    static const GLfloat clear_depth[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    
    // This sets the camera to a new location
    // the first parameter is the eye position, the second the center location, and the third the up vector. 
    SetViewAsLookAt(glm::vec3(12.0f, 12.0f, 35.5f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    
    
    // Enable depth test
    // ignore this line, it allows us to keep the distance value after we proejct each object to a 2d canvas.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
   // sphere->enableNormalVectorRenderer();
    
    // This is our render loop. As long as our window remains open (ESC is not pressed), we'll continue to render things.
    while(!glfwWindowShouldClose(window))
    {
        
        // Clear the entire buffer with our green color (sets the background to be green).
        glClearBufferfv(GL_COLOR , 0, clear_color);
        glClearBufferfv(GL_DEPTH , 0, clear_depth);
        
    
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //// This renders the objects
        
        // Set the trackball locatiom
        SetTrackballLocation(trackball.getRotationMatrix());
        
        // draw the objects
        cs->draw();
        
        sphere->draw();
		sphere2->draw();
		sphere3->draw();
		sphere4->draw();
        
        //// This renders the objects
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        
        // Swap the buffers so that what we drew will appear on the screen.
        glfwSwapBuffers(window);
        glfwPollEvents();
        
    }
    
    
    delete cs;


}

