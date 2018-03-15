#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include <gl/glut.h>
#include <math.h>
#include "Vector3D.h"
#include "Matrix3D.h"
#include "CubeMesh.h"
#include "QuadMesh.h"


//Changed the size of the mesh to 256 to add more vertices and make the holes smoother
const int meshSize = 256;

const int vWidth = 650;     // Viewport width in pixels
const int vHeight = 500;    // Viewport height in pixels

static int currentButton;
static unsigned char currentKey;

// Lighting/shading and material properties for submarine - upcoming lecture - just copy for now

// Light properties
static GLfloat light_position0[] = { -6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_position1[] = { 6.0F, 12.0F, 0.0F, 1.0F };

static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

// Material properties
static GLfloat submarine_mat_ambient[] = { 0.4F, 0.2F, 0.0F, 1.0F };
static GLfloat submarine_mat_specular[] = { 0.1F, 0.1F, 0.0F, 1.0F };
static GLfloat submarine_mat_diffuse[] = { 0.9F, 0.5F, 0.0F, 1.0F };
static GLfloat submarine_mat_shininess[] = { 0.0F };

// A quad mesh representing the ground / sea floor 
static QuadMesh groundMesh;



// Structure defining a bounding box, currently unused
//struct BoundingBox {
//    Vector3D min;
//    Vector3D max;
//} BBox;

// Prototypes for functions in this module
void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void mouse(int button, int state, int x, int y);
void mouseMotionHandler(int xMouse, int yMouse);
void keyboard(unsigned char key, int x, int y);
void functionKeys(int key, int x, int y);
Vector3D ScreenToWorld(int x, int y);
void createHole(float xPos, float zPos);



//the angles for turning parts of the robot
float direction = 0.0;
float shoulderDirection = 0.0;
float elbowDirection = 0.0;

//angles for rotating the camera
float cameraSpin = 0.0;
float cameraUpDown = 0.0;

//keeps track of the old mouse coordinates when performing the mouse dragging
float oldX = 0.0;
float oldY = 0.0;

//controls how far the camera will zoom in or out
float zoom = 0.0;

//check to see if it is an appropriate time to dig a hole
bool canDig = false;

int main(int argc, char **argv)
{
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(vWidth, vHeight);
    glutInitWindowPosition(200, 30);
    glutCreateWindow("Assignment 2");

    // Initialize GL
    initOpenGL(vWidth, vHeight);

    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotionHandler);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(functionKeys);

    // Start event loop, never returns
    glutMainLoop();

    return 0;
}


// Set up OpenGL. For viewport and projection setup see reshape(). */
void initOpenGL(int w, int h)
{
    // Set up and enable lighting
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    //glEnable(GL_LIGHT1);   // This light is currently off

    // Other OpenGL setup
    glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
    glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see 
    glClearColor(0.6F, 0.6F, 0.6F, 0.0F);  // Color and depth for glClear
	glClearDepth(1.0f);
    glEnable(GL_NORMALIZE);    // Renormalize normal vectors 
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective

    // Set up ground/sea floor quad mesh
    Vector3D origin = NewVector3D(-8.0f, 0.0f, 8.0f);
    Vector3D dir1v = NewVector3D(1.0f, 0.0f, 0.0f);
    Vector3D dir2v = NewVector3D(0.0f, 0.0f, -1.0f);
    groundMesh = NewQuadMesh(meshSize);
    InitMeshQM(&groundMesh, meshSize, origin, 16.0, 16.0, dir1v, dir2v);

    Vector3D ambient = NewVector3D(0.0f, 0.05f, 0.0f);
    Vector3D diffuse = NewVector3D(0.4f, 0.8f, 0.4f);
    Vector3D specular = NewVector3D(0.04f, 0.04f, 0.04f);
    SetMaterialQM(&groundMesh, ambient, diffuse, specular, 0.2);

    // Set up the bounding box of the scene
    // Currently unused. You could set up bounding boxes for your objects eventually.
    //Set(&BBox.min, -8.0f, 0.0, -8.0);
    //Set(&BBox.max, 8.0f, 6.0,  8.0);

	//initialize the old mouse coordinates to the middle of the screen
	oldX = vWidth / 2.0;
	oldY = vHeight / 2.0;

}


// Callback, called whenever GLUT determines that the window should be redisplayed
// or glutPostRedisplay() has been called.
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// Set up the camera at position (0, 0.1, 15) looking at the origin, up along positive y axis
	gluLookAt(0.0, 0.1, 15.0 + zoom, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);


	// Set robot material properties
	glMaterialfv(GL_FRONT, GL_AMBIENT, submarine_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, submarine_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, submarine_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, submarine_mat_shininess);


	//directions/angles for rotation for turning the robot
	float xDirection = sin(direction* 3.14159265358979323846 / 180);
	float zDirection = -cos(direction* 3.14159265358979323846 / 180);


	//Vector which the camera will rotate around
	Vector3D cameraPitcher = NewVector3D(1.0, 0.0, 0.0);

	//set boundaries for camera rotation
	if (cameraUpDown >= 90.0)
		cameraUpDown = 90.0;
	if (cameraUpDown <= 0.0)
		cameraUpDown = 0.0;

	//set boundaries for shoulder rotation
	if (shoulderDirection <= -90.0)
		shoulderDirection = -90.0;
	if (shoulderDirection >= 0.0)
		shoulderDirection = 0.0;

	//set boundaries for elbow rotation
	if (elbowDirection >= 0.0)
		elbowDirection = 0.0;
	if (elbowDirection <= -150.0)
		elbowDirection = -150.0;

	//this will control the up-down rotation of the camera
	glRotatef(cameraUpDown, cameraPitcher.x, cameraPitcher.y, cameraPitcher.z);

	//this will rotate the camera around (side to side) around the invisible dome
	cameraPitcher.x = sin(cameraSpin* 3.14159265358979323846 / 180);
	cameraPitcher.z = -cos(cameraSpin* 3.14159265358979323846 / 180);
	glRotatef(cameraSpin, 0.0, 1.0, 0.0);


	glPushMatrix();

	//rotates robot around the base
	glRotatef(direction, 0, 1, 0);

	//for the robot base
	glPushMatrix();
	glutSolidSphere(1.0, 50, 50);
	glPopMatrix();

	//rotates the shoulder
	glRotatef(shoulderDirection, 0.0, 0.0, 1.0);
	//main arm of robot (shoulder)
	glPushMatrix(); 
	glTranslatef(0.0, 1.0, 0.0);
	glScalef(0.5, 3.0, 0.5);
	glTranslatef(0.0, 0.5, 0.0);
	glutSolidCube(1.0);
	glPopMatrix();

	//rotates the elbow 
	glTranslatef(0.0, 4.0, 0.0);
	glRotatef(elbowDirection, 0.0, 0.0, 1.0);
	glTranslatef(0.0, -4.0, 0.0);

	//mini arm of the robot (elbow)
	glPushMatrix();
	glTranslatef(0.0, 3.0, 0.0);
	glTranslatef(0.0, 1.0, 0.0);
	glScalef(0.4, 2.0, 0.4);
	glTranslatef(0.0, 0.5, 0.0);
	glutSolidCube(1.0);
	glPopMatrix();

	//shovel of the robot
	glPushMatrix(); 
	glTranslatef(0.0, 6.0, 0.0);
	glScalef(0.1, 1.0, 2.0);
	glTranslatef(0.0, 0.5, 0.0);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();


	//This is the forward kinematics code using the 4x4 matrices with each tranformation done to the vector
	Matrix3D m = NewIdentity();
	Vector3D v = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&m, NewScale(1.0, 3.0, 1.0));//scale it to tip of the shovel
	MatrixLeftMultiplyV(&m, NewRotateZ(elbowDirection)); //elbow rotation

	MatrixLeftMultiplyV(&m, NewTranslate(0.0, 3.0, 0.0));//bottom of vector is at the tip of the shoulder boom
	MatrixLeftMultiplyV(&m, NewTranslate(0.0, 1.0, 0.0));//translates up
	MatrixLeftMultiplyV(&m, NewRotateZ(shoulderDirection));//pitch for shoulder
	MatrixLeftMultiplyV(&m, NewRotateY(direction));//yaw for shoulder

	VectorLeftMultiply(&v, &m);



	//this is when the tip of the shovel hits the x-z plane
	if (v.y >= -0.2 && v.y <= 0.2 && canDig)
	{
		//create a hole where the shovel is placed
		createHole(v.x, v.z);
	}




    // Draw ground/sea floor
    DrawMeshQM(&groundMesh, meshSize);

    glutSwapBuffers();   // Double buffering, swap buffers
}

//calculates where to create hole and depth of the hole
void createHole(float x, float z)
{
	//set a 'b' value to control height
	//set an 'a' value to control the width
	float a = 2.0;
	float b = -0.5;

	int i = 0;
	for (i = 0; i < groundMesh.numVertices; i++)
	{
		//calculate the distance from the current hole to this vertex
		float r = sqrt(pow((x - groundMesh.vertices[i].position.x), 2) + pow((z - groundMesh.vertices[i].position.z), 2));

		//now plug in the r,a,b, into the gaussian formula to get the y-value of the current vertex
		float updatedY = b * exp(-a * pow(r, 2));
		
		//update depth of hole at this vertex
		if (updatedY < 0.0)
		{
			groundMesh.vertices[i].position.y = groundMesh.vertices[i].position.y + updatedY;
		}
	}

}


// Callback, called at initialization and whenever user resizes the window.
void reshape(int w, int h)
{
    // Set up viewport, projection, then change to modelview matrix mode - 
    // display function will then set up camera and do modeling transforms.
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLdouble)w / h, 0.2, 40.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

// Callback, handles input from the keyboard, non-arrow keys
void keyboard(unsigned char key, int x, int y)
{
    switch (key)//the controls of the robot
    {
	//a and d for the base (rotate base)
	//s and w for the shoulder (rotate the shoulder)
	//o and l for the elbow (rotate the elbow)
	//user can only dig when moving the shovel down using 'w' and 'o' keys
	case 'a':
		direction += 2.0f;
		canDig = false;
		break;
	case 'd':
		direction -= 2.0f;
		canDig = false;
		break;
	case 's': 
		shoulderDirection = (int)(shoulderDirection + 5) % 360;
		canDig = false;
		break;
	case 'w':
		shoulderDirection = (int)(shoulderDirection - 5) % 360;
		canDig = true;
		break;
	case 'o': 
		elbowDirection = (int)(elbowDirection - 5) % 360;
		canDig = true;
		break;
	case 'l':  
		elbowDirection = (int)(elbowDirection + 5) % 360;
		canDig = false;
		break;
	default:
		direction = 0.0f;
		canDig = false;
		break;
    }


    glutPostRedisplay();   // Trigger a window redisplay
}

// Callback, handles input from the keyboard, function and arrow keys
void functionKeys(int key, int x, int y)
{
    // Help key
    if (key == GLUT_KEY_F1)
	{
		printf("Help:\n");
		printf("Robot Controls:\n");
		printf("W key: Moves the robot shoulder forward\n");
		printf("S key: Moves the robot shoulder backwards\n");
		printf("A key: Turns the robot counter-clockwise\n");
		printf("D key: Turns the robot clockwise\n");
		printf("O key: Rotates the elbow downwards\n");
		printf("L key: Rotates the elbow upwards\n\n");

		printf("Camera Controls:\n");
		printf("Left Mouse Click + Mouse Movement: Rotates the camera around the scene\n");
		printf("Mouse Scroll Up: Camera zoom in\n");
		printf("Mouse Scroll Down: Camera zoom out\n");
		printf("\n");
    }
 

    glutPostRedisplay();   // Trigger a window redisplay
}


// Mouse button callback - use only if you want to 
void mouse(int button, int state, int x, int y)
{
	//zoom in on mouse scroll up
	if (button == 3)
	{
		zoom -= 1.0;;
	}
	//zoom out on mouse scroll down
	if (button == 4)
	{
		zoom += 1.0;;
	}


    glutPostRedisplay();   // Trigger a window redisplay
}


// Mouse motion callback - use only if you want to 
void mouseMotionHandler(int xMouse, int yMouse)
{
	//when holding left click on mouse and moving mouse, rotate the camera
    if (currentButton == GLUT_LEFT_BUTTON)
    {
		if (xMouse > oldX)
		{
			cameraSpin += 3.0;
		}
		if (xMouse < oldX)
		{
			cameraSpin -= 3.0;
		}
		if (yMouse > oldY)
		{
			cameraUpDown = (int)(cameraUpDown + 3) % 360;
		}
		if (yMouse < oldY)
		{
			cameraUpDown = (int)(cameraUpDown - 3) % 360;
		}
    }

	oldX = xMouse;
	oldY = yMouse;

    glutPostRedisplay();   // Trigger a window redisplay
}


Vector3D ScreenToWorld(int x, int y)
{
    // you will need to finish this if you use the mouse
    return NewVector3D(0, 0, 0);
}



