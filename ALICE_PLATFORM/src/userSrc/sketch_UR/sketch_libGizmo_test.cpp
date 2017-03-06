

#ifdef _MAIN_
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graphStack.h"
#include "IGizmo.h"

////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

IGizmo *Gz, *Gz_m, *Gz_r, *Gz_s;
float objectMatrix[16];
Matrix4 T;
Mesh M;

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

////// ---------------------------------------------------- MODEL  ----------------------------------------------------


void setup()	
{
	T.identity();
	for (int i = 0; i < 16; i += 1)objectMatrix[i] = T[i];

	Gz_m = CreateMoveGizmo();
	Gz_r = CreateRotateGizmo();
	Gz_s = CreateScaleGizmo();
	Gz = Gz_m;;
	Gz->SetEditMatrix(objectMatrix);
	Gz->SetLocation(IGizmo::LOCATE_WORLD);
	Gz->SetScreenDimension(winW, winH);


	MeshFactory fac;
	M = fac.createPlatonic(1, 6);

	Matrix4 Tr;
	Tr.setColumn(0, vec(1, 1, 0).normalise());
	Tr.setColumn(1, vec(1, -1, 0).normalise());
	Tr.setColumn(2, vec(0, 0, 1));
	Tr.setColumn(3, vec(0, 0, 0.5));
	Tr.invert();
	for (int i = 0; i < 8; i++) M.positions[i] = Tr * M.positions[i];


	
	
}

void update(int value)
{
	T.invert();
	for (int i = 0; i < M.n_v; i++) M.positions[i] = T * M.positions[i];

	
	Gz->SetScreenDimension(winW, winH);
	for (int i = 0; i < 16; i += 1)T[i] = objectMatrix[i];
	T = T.transpose();

	for (int i = 0; i < M.n_v; i++) M.positions[i] = T * M.positions[i];
	
	
}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(0.75);
	drawGrid(20);


	Gz->Draw();

	float viewMat[16];
	float projMat[16];

	glGetFloatv(GL_MODELVIEW_MATRIX, viewMat);
	glGetFloatv(GL_PROJECTION_MATRIX, projMat);

	Gz->SetCameraMatrix(viewMat, projMat);
	
	//
	wireFrameOn();

		M.draw();


	wireFrameOff();
	
	
	char s[200];
	glColor3f(0, 0, 0);
	setup2d();

	
	sprintf(s, "%i", updateCam);
	drawString(s, 50, 300);

	///forward
	vec str(50, 350, 0);
	double width = 4 * 15;
	double ht = 24;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			sprintf(s, "%1.2f", objectMatrix[j * 4 + i]);
			drawString(s, i * width + str.x, j * ht + str.y);
		}

	///inverse
	str = vec(50, 350 + 24*4 + 15, 0);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			sprintf(s, "%1.2f", T[j * 4 + i]);
			drawString(s, i * width + str.x, j * ht + str.y);
		}

	restore3d();




}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{
	//XVTFE reserved 
	if (k == 'A')updateCam = !updateCam;
	if (k == 'w')
	{
		Gz = Gz_m;
		Gz->SetLocation(IGizmo::LOCATE_WORLD);
	}
	if (k == 'e')
	{
		Gz = Gz_r;
		Gz->SetLocation(IGizmo::LOCATE_LOCAL);
	}
	if( k =='r')
	{
		Gz = Gz_s;
		Gz->SetLocation(IGizmo::LOCATE_WORLD);
	}

	Gz->SetEditMatrix(objectMatrix);
	Gz->SetScreenDimension(winW, winH);
}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
		if (Gz->OnMouseDown(x, y))updateCam = false;
			
	if (GLUT_LEFT_BUTTON == b && GLUT_UP == state)
	{
		Gz->OnMouseUp(x, y);
		updateCam = true;
	}
	
}

void mouseMotion(int x, int y)
{
	Gz->SetEditMatrix(objectMatrix);
	Gz->OnMouseMove(x, y);
}




#endif // _MAIN_
