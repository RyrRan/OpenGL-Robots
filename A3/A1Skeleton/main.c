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
#include <stdlib.h>


//Changed the size of the mesh to 256 to add more vertices and make the holes smoother
const int meshSize = 256;

const int vWidth = 650;     // Viewport width in pixels
const int vHeight = 500;    // Viewport height in pixels

static int currentButton;
static unsigned char currentKey;

// Lighting/shading and material properties for submarine - upcoming lecture - just copy for now

// Light properties
//static GLfloat light_position0[] = { -6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_position0[] = { -26.0F, 12.0F, -26.0F, 1.0F };

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
void createHole();
float botDip(float x, float y);
void timerSpeed(char prevKey);
void botMovement();




//the angles for turning parts of the human player
float direction = 0.0;
float shoulderDirection = 0.0;
float elbowDirection = 0.0;
//controls speed of the human player
float speed = 0.0;
//updated positions of the player's robot after a transformation
float xPos = 0.0;
float zPos = 0.0;
float yPos = 0.0;


//the angles for turning parts of the AI
float directionAI = 0.0;
float shoulderDirectionAI = 0.0;
float elbowDirectionAI = 0.0;
//controls speed of the AI
float speedAI = 0.0;
//updated positions of the AI
float xPosAI = 0.0;
float zPosAI = 0.0;
float yPosAI = 0.0;



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



//This is for the movement of the robot
Matrix3D mPlayerBase;
Vector3D vPlayerBase;
//This is for the movement of the AI
Matrix3D mPlayerBaseAI;
Vector3D vPlayerBaseAI;
//This is for where the FPV camera needs to look at
Matrix3D mPov;
Vector3D vPov;
Matrix3D mPovFirst;
Vector3D vPovFirst;

//this keeps track of the previous key in order to determine the movement speed function
char previousKey;

//a bool that allows the user to switch between first person camera and the default camera
bool isFirstPerson = false;

//this controls the random direction change of the robot
bool dChange = false;

//This is the health bar for the player's robot and the AI robot
int playerHealth = 100;
int aiHealth = 100;



//timer function to add a delay after an action is performed 
static void Time(int value) {
	glutPostRedisplay();
	glutTimerFunc(100, Time, 0);
}



int main(int argc, char **argv)
{
	// Initialize GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(vWidth, vHeight);
	glutInitWindowPosition(200, 30);
	glutCreateWindow("Assignment 3");

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
														 //Vector3D origin = NewVector3D(-8.0f, 0.0f, 8.0f);
	Vector3D origin = NewVector3D(-16.0f, 0.0f, 16.0f);
	Vector3D dir1v = NewVector3D(1.0f, 0.0f, 0.0f);
	Vector3D dir2v = NewVector3D(0.0f, 0.0f, -1.0f);
	groundMesh = NewQuadMesh(meshSize);
	//InitMeshQM(&groundMesh, meshSize, origin, 16.0, 16.0, dir1v, dir2v);
	InitMeshQM(&groundMesh, meshSize, origin, 32.0, 32.0, dir1v, dir2v);

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

	//create holes to start the program
	createHole();

	//a toggle between having a first person camera and the default camera
	isFirstPerson = false;
}


// Callback, called whenever GLUT determines that the window should be redisplayed
// or glutPostRedisplay() has been called.
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	// Set up the camera at position (0, 0.1, 15) looking at the origin, up along positive y axis as the default camera
	if (isFirstPerson == false)
	{
		gluLookAt(0.0, 0.1, 15.0 + zoom, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	}
	//change the camera to first person
	else if (isFirstPerson == true)
	{
		gluLookAt(vPovFirst.x, vPovFirst.y, vPovFirst.z,
			vPov.x, vPov.y, vPov.z,
				0.0, 1.0, 0.0);

		cameraSpin = 0.0;
		cameraUpDown = 0.0;
		oldX = 0.0;
		oldY = 0.0;
	}


	// Set robot material properties
	glMaterialfv(GL_FRONT, GL_AMBIENT, submarine_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, submarine_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, submarine_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, submarine_mat_shininess);

		

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





	//This is the modelling for the AI's robot
	///////////////////////////////////////////////////////////////////////////////////////////////////
	
	//randomizes the ai's movement
	botMovement();

	//the shoulder and elbow of the ai's weapon is prefixed
	shoulderDirectionAI = 90;
	elbowDirectionAI = 20;

	//directions/angles for rotation for turning the robot
	float xDirectionAI = sin(directionAI * 3.14159265358979323846 / 180);
	float zDirectionAI = -cos(directionAI * 3.14159265358979323846 / 180);

	//calculate the new X, Y, Z positions
	xPosAI -= speedAI * xDirectionAI;
	zPosAI += speedAI * zDirectionAI;


	glPushMatrix();


	glTranslatef(10.0, 0.0, 0.0);
	glTranslatef(xPosAI, yPosAI, zPosAI);
	glRotatef(directionAI - 90, 0, 1, 0);

	//this is to make the wheels
	glPushMatrix();
	GLUquadricObj *wheels[4];
	GLUquadricObj *rims[4];

	for (int i = 0; i < 4; i++)
	{
		wheels[i] = gluNewQuadric();
		rims[i] = gluNewQuadric();
	}

	//transformations to add the wheels to the sides
	glTranslatef(-2.0, 0.5, 1.5);
	gluCylinder(wheels[0], 0.5, 0.5, 1, 30, 40);
	glutSolidSphere(0.5, 50, 50);
	glTranslatef(4.0, 0.0, 0.0);
	gluCylinder(wheels[1], 0.5, 0.5, 1, 30, 40);
	glTranslatef(0.0, 0.0, -4.0);
	gluCylinder(wheels[2], 0.5, 0.5, 1, 30, 40);
	glTranslatef(-4.0, 0.0, 0.0);
	gluCylinder(wheels[3], 0.5, 0.5, 1, 30, 40);

	glPushMatrix();
	glTranslatef(-2.0, 0.5, 1.5);
	gluDisk(rims[0], 0.5, 0.5, 50, 50);
	glPopMatrix();

	glPopMatrix();


	glTranslatef(0.0, 0.5, 0.0);

	//the body of the ai(the square base)
	glPushMatrix();
	glScalef(2.0, 0.5, 2.0);
	glutSolidCube(2.0);
	glPopMatrix();


	//the sphere where the whole arms turns around (translate to make sure the arm is attached to it)
	glTranslatef(0.0, 1.0, 0.0);
	glutSolidSphere(1.0, 50, 50);

	//rotates the shoulder
	glRotatef(shoulderDirectionAI, 0.0, 0.0, 1.0);
	//main arm of robot (shoulder)
	glPushMatrix();
	glTranslatef(0.0, 1.0, 0.0);
	glScalef(0.5, 3.0, 0.5);
	glTranslatef(0.0, 0.5, 0.0);
	glutSolidCube(1.0);
	glPopMatrix();

	//rotates the elbow 
	glTranslatef(0.0, 4.0, 0.0);
	glRotatef(elbowDirectionAI, 0.0, 0.0, 1.0);
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



	//This is the forward kinematics code using the 4x4 matrices for the AI SHOVEL
	Matrix3D mAI = NewIdentity();
	Vector3D vAI = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&mAI, NewScale(1.0, 3.0, 1.0));//scale it to tip of the shovel
	MatrixLeftMultiplyV(&mAI, NewRotateZ(elbowDirectionAI)); //elbow rotation

	MatrixLeftMultiplyV(&mAI, NewTranslate(0.0, 3.0, 0.0));//bottom of vector is at the tip of the shoulder boom
	MatrixLeftMultiplyV(&mAI, NewTranslate(0.0, 1.0, 0.0));//translates up
	MatrixLeftMultiplyV(&mAI, NewRotateZ(shoulderDirectionAI));//pitch for shoulder
	MatrixLeftMultiplyV(&mAI, NewRotateY(directionAI - 90));//yaw for shoulder
	MatrixLeftMultiplyV(&mAI, NewTranslate(0.0, 1.5, 0.0));
	MatrixLeftMultiplyV(&mAI, NewTranslate(xPosAI, yPosAI, zPosAI));
	MatrixLeftMultiplyV(&mAI, NewTranslate(10.0, 0.0, 0.0));
	VectorLeftMultiply(&vAI, &mAI);


	//for the base of the player's robot to control collision and up/down movement on terrain
	mPlayerBaseAI = NewIdentity();
	vPlayerBaseAI = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&mPlayerBaseAI, NewTranslate(0.0, -2.0, 0.0));
	MatrixLeftMultiplyV(&mPlayerBaseAI, NewTranslate(0.0, 1.5, 0.0));
	MatrixLeftMultiplyV(&mPlayerBaseAI, NewTranslate(10.0, 0.0, 0.0));
	MatrixLeftMultiplyV(&mPlayerBaseAI, NewTranslate(xPosAI, yPosAI, zPosAI));
	VectorLeftMultiply(&vPlayerBaseAI, &mPlayerBaseAI);


	//change the y position of the robot so that it can dip into holes and valleys
	yPosAI = botDip(vPlayerBaseAI.x, vPlayerBaseAI.z);

	///////////////////////////////////////////////////////////////////////////////////////////////




	////This is the modelling for the player's robot--------------------------------------------------------------------------------------------------
	static GLfloat playerDiffuse[] = { 0.9, 0.07,0,1.0 };
	static GLfloat playerSpecular[] = { 1.0,1.0,1.0,1.0 };
	static GLfloat playerAmbient[] = { 0.2,0.2,0.2,0.1 };

	glMaterialfv(GL_FRONT, GL_AMBIENT, playerAmbient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, playerSpecular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, playerDiffuse);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);


	glPushMatrix();

	//this makes sure to turn the robot around if it tries to drive out of the boundaries
	if (vPlayerBase.x > 16 || vPlayerBase.x < -16 || vPlayerBase.z < -16 || vPlayerBase.z > 16)
	{
		direction += 180;
	}


	//directions/angles for rotation for turning the robot
	float xDirection = sin(direction* 3.14159265358979323846 / 180);
	float zDirection = -cos(direction* 3.14159265358979323846 / 180);

	//calculate the new X, Y, Z positions
	xPos -= speed* xDirection;
	zPos += speed* zDirection;

	//move and rotate entire blimp according to the updated X,Y,Z coordinates
	glTranslatef(xPos, yPos, zPos);
	glRotatef(direction + 90, 0, 1, 0);

	//this is to make the wheels on the side of the robot
	glPushMatrix();
	GLUquadricObj *wheelsBot[4];
	GLUquadricObj *rimsBot[4];

	for (int i = 0; i < 4; i++)
	{
		wheelsBot[i] = gluNewQuadric();
		rimsBot[i] = gluNewQuadric();
	}

	//tranformations to place the wheels on the side
	glTranslatef(-2.0, 0.5, 1.5);
	gluCylinder(wheelsBot[0], 0.5, 0.5, 1, 30, 40);
	glutSolidSphere(0.5, 50, 50);
	glTranslatef(4.0, 0.0, 0.0);
	gluCylinder(wheelsBot[1], 0.5, 0.5, 1, 30, 40);
	glTranslatef(0.0, 0.0, -4.0);
	gluCylinder(wheelsBot[2], 0.5, 0.5, 1, 30, 40);
	glTranslatef(-4.0, 0.0, 0.0);
	gluCylinder(wheelsBot[3], 0.5, 0.5, 1, 30, 40);

	glPushMatrix();
	glTranslatef(-2.0, 0.5, 1.5);
	gluDisk(rimsBot[0], 0.5, 0.5, 50, 50);
	glPopMatrix();

	glPopMatrix();




	glTranslatef(0.0, 0.5, 0.0);

	//the body of the machine(the square)
	glPushMatrix();
	glScalef(2.0, 0.5, 2.0);//glScalef(4.0, 0.5, 2.0);
	glutSolidCube(2.0);
	glPopMatrix();


	//the sphere where the whole arms turns around (translate to make sure the arm is attached to it)
	glTranslatef(0.0, 1.0, 0.0);
	glutSolidSphere(1.0, 50, 50);

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
	//-----------------------------------------------------------------------------------------------------------------------------------------

	//This is the forward kinematics code using the 4x4 matrices FOR THE SHOVEL WEAPON
	Matrix3D m = NewIdentity();
	Vector3D v = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&m, NewScale(1.0, 3.0, 1.0));//scale it to tip of the shovel
	MatrixLeftMultiplyV(&m, NewRotateZ(elbowDirection)); //elbow rotation

	MatrixLeftMultiplyV(&m, NewTranslate(0.0, 3.0, 0.0));//bottom of vector is at the tip of the shoulder boom
	MatrixLeftMultiplyV(&m, NewTranslate(0.0, 1.0, 0.0));//translates up
	MatrixLeftMultiplyV(&m, NewRotateZ(shoulderDirection));//pitch for shoulder
	MatrixLeftMultiplyV(&m, NewRotateY(direction + 90));//yaw for shoulder
	MatrixLeftMultiplyV(&m, NewTranslate(0.0, 1.5, 0.0));
	MatrixLeftMultiplyV(&m, NewTranslate(xPos, yPos, zPos));
	VectorLeftMultiply(&v, &m);


	//for the base of the player's robot
	mPlayerBase = NewIdentity();
	vPlayerBase = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&mPlayerBase, NewTranslate(0.0, -2.0, 0.0));
	MatrixLeftMultiplyV(&mPlayerBase, NewTranslate(0.0, 1.5, 0.0));
	MatrixLeftMultiplyV(&mPlayerBase, NewTranslate(xPos, yPos, zPos));
	VectorLeftMultiply(&vPlayerBase, &mPlayerBase);


	//for the base of the robot where the pov camera will be staring at
	mPov = NewIdentity();
	vPov = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&mPov, NewTranslate(0.0, 0.0, -3.0));
	MatrixLeftMultiplyV(&mPov, NewTranslate(0.0, 0.5, 0.0));
	MatrixLeftMultiplyV(&mPov, NewRotateY(direction));//yaw for shoulder
	MatrixLeftMultiplyV(&mPov, NewTranslate(xPos, yPos, zPos));
	VectorLeftMultiply(&vPov, &mPov);


	//for the base of the robot where the pov camera will be placed at
	mPovFirst = NewIdentity();
	vPovFirst = NewVector3D(0.0, 1.0, 0.0);
	MatrixLeftMultiplyV(&mPovFirst, NewTranslate(0.0, 0.0, -2.0));
	MatrixLeftMultiplyV(&mPovFirst, NewTranslate(0.0, 0.5, 0.0));
	MatrixLeftMultiplyV(&mPovFirst, NewRotateY(direction));//yaw for shoulder
	MatrixLeftMultiplyV(&mPovFirst, NewTranslate(xPos, yPos, zPos));
	VectorLeftMultiply(&vPovFirst, &mPovFirst);




	// Draw ground/sea floor
	DrawMeshQM(&groundMesh, meshSize);


	//change the y position of the robot so that it can dip into holes and valleys
	yPos = botDip(vPlayerBase.x, vPlayerBase.z);

	//if the player's robot is not moving forwards or backwards 
	if (previousKey != 'w' || previousKey != 's')
	{
		previousKey = '!';
		glutPostRedisplay(); 
	}


	glutSwapBuffers();   // Double buffering, swap buffers
}

//Calculates the y position/coordinates of the bot when it reaches a hole/dip
float botDip(float x, float z)
{
	//to figure out the dip value of the battlebot, go through all the mesh verticies that are lower than 0
	//then calculate the vertex under 0 that is closest to the base of the battlebot (the x-z plane distance)
	//once you get the closest vertex, take its y-value and this is the battlebot's new y-value
	//run this algorithm every single time something happens (basically run it in the display function)
	
	int i = 0;//counter for the loop
	int counter = 0;//counts the number of vertices that are y < 0

	//the closest vertex and its distance when y<0
	MeshVertex closestPoint;
	float closestDistance;

	//compare every vertex to the bot's current position
	for (i = 0; i < groundMesh.numVertices; i++)
	{
		//only need to check a recalculation of y if the vertex is y < 0 otherwise the robot will not dip 
		if (groundMesh.vertices[i].position.y < 0.0)
		{
			if (counter == 0)
				closestPoint = groundMesh.vertices[i];

			//calculate the distance from the current hole to this vertex
			float r = sqrt(pow((x - groundMesh.vertices[i].position.x), 2) + pow((z - groundMesh.vertices[i].position.z), 2));


			//the robot's new y position is going to be the same y position as the closest vertex with y<0
			if (counter == 0)
			{
				closestDistance = r;
			}
			else if (r < closestDistance)
			{
				closestPoint = groundMesh.vertices[i];
				closestDistance = r;
			}
			counter++;
		}
	}

	//return the closest vertex where y<0 and this will be how much the bot will dip
	return closestPoint.position.y;
}


//bot slowdown function so that the robot will keep moving after key is done being pressed
void timerSpeed(char prevKey)
{
	//if the bot is moving forward
	if (prevKey == 'w')
	{
		//increase speed if w is pressed again
		if (speed < 0.5f) {
			speed = speed + 0.02f;
			Time(2000);
		}
		//cap off speed at the max
		else
			speed = 0.5f;

	}
	//if the bot was moving backwards
	else if (prevKey == 's')
	{
		//decrease speed if s is pressed again
		if (speed > -0.5f) {
			speed = speed - 0.02f;
			Time(2000);
		}
		//cap off speed at the max
		else
			speed = -0.5f;
	}
	//if the player's robot was not moving forwards or backwards
	else if(prevKey == '!')
	{
		//slow down the robot and bring it to a stop
		if (speed > 0.0f)
		{
			speed = speed - 0.02f;
			Time(2000);
		}
		else if (speed < 0.0f)
		{
			speed = speed + 0.02f;
			Time(2000);
		}
		else if (speed <= 0.1f && speed >= -0.1f)
		{
			speed = 0.0f;
			printf("Speed:%f\n", speed);
		}
		
	}
}


//randomizes the bot's movement around the terrain
void botMovement()
{
	speedAI = 0.15f;

	//the robot will randomly change its direction 
	if (rand() % 250 == 0)
	{
		if (dChange == true)
			dChange = false;
		else
			dChange = true;
	}

	if (dChange == false)
	{
		directionAI += 0.2f;
	}
	else if (dChange == true)
	{
		directionAI -= 0.2f;
	}

	//turn the robot around if it is trying to drive outside of the boundaries
	if (vPlayerBaseAI.x > 16 || vPlayerBaseAI.x < -16 || vPlayerBaseAI.z < -16 || vPlayerBaseAI.z > 16)
	{
		directionAI += 180;
	}


}


//calculates where to create hole and depth of the hole
void createHole()
{
	//set a 'b' value to control height
	//set an 'a' value to control the width
	float a = 0.1;
	float b = -1.5;

	float x, z;

	//this is the list of vertices where the holes will be pre-made
	int holeLocations[20];

	holeLocations[0] = 30;
	holeLocations[1] = 230;
	holeLocations[2] = 430;
	holeLocations[3] = 630;
	holeLocations[4] = 830;
	holeLocations[5] = 1030;
	holeLocations[6] = 1230;

	holeLocations[7] = 4260;
	holeLocations[8] = 5270;
	holeLocations[9] = 6280;
	holeLocations[10] = 7290;
	holeLocations[11] = 8300;
	holeLocations[12] = 9310;


	holeLocations[13] = 33000;
	holeLocations[14] = 35000;
	holeLocations[15] = 37000;
	holeLocations[16] = 39000;
	holeLocations[17] = 55000;
	holeLocations[18] = 59000;
	holeLocations[19] = 61000;

	
	int p = 0;
	for (p = 0; p < 20; p++)
	{
		x = groundMesh.vertices[holeLocations[p]].position.x;
		z = groundMesh.vertices[holeLocations[p]].position.z;

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
		//a and d is to turn the robot left and right respectively 
		//s and w is to move the robot forward and backwards
		//k and i is to move the shoulder of the robot weapon
		//o and l is to move the elbow of the robot weapon
	case 'w':
		previousKey = 'w';
		timerSpeed(previousKey);
		previousKey = '!';
		break;
	case 's':
		previousKey = 's';
		timerSpeed(previousKey);
		previousKey = '!';
		break;
	case 'a':
		direction += 2.0f;
		break;
	case 'd':
		direction -= 2.0f;
		break;
	case 'k':
		shoulderDirection = (int)(shoulderDirection + 5) % 360;
		canDig = false;
		break;
	case 'i':
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
		printf("default\n");
		previousKey = '!';
		timerSpeed(previousKey);
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
		printf("W key: Moves the robot forward\n");
		printf("S key: Moves the robot backwards\n");
		printf("A key: Turns the robot left\n");
		printf("D key: Turns the robot right\n");
		printf("I key: Rotates the shoulder downwards\n");
		printf("K key: Rotates the shoulder upwards\n");
		printf("O key: Rotates the elbow downwards\n");
		printf("L key: Rotates the elbow upwards\n");
		printf("Up arrow key: Changes the camera to first person (FPV) mode\n");
		printf("Down arrow key: Changes the camera to the default world camera\n\n");

		printf("Camera Controls:\n");
		printf("Left Mouse Click + Mouse Movement: Rotates the camera around the scene\n");
		printf("Mouse Scroll Up: Camera zoom in\n");
		printf("Mouse Scroll Down: Camera zoom out\n");
		printf("\n");
	}
	else if (key == GLUT_KEY_DOWN)
	{
		isFirstPerson = false;
	}
	else if (key == GLUT_KEY_UP)
	{
		isFirstPerson = true;
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




