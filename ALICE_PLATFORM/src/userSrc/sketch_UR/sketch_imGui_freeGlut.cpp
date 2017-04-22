//#define _MAIN_

#ifdef _MAIN_
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

#include "imgui.h"
#include "ImGuizmo.h"
////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

float objectMatrix[16];
Matrix4 Tr;
Mesh M;

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------


void setup()	
{
	Tr.identity();
	for (int i = 0; i < 16; i += 1)objectMatrix[i] = Tr[i];



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
	
	
}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(0.75);
	drawGrid(20);


	//
	wireFrameOn();

		M.draw();


		//imgui_draw();

	wireFrameOff();
	
	
	setup2d();
	restore3d();


}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{
	//XVTFE reserved 
	if (k == 'A')updateCam = !updateCam;

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
		/*if (Gz->OnMouseDown(x, y))updateCam = false;*/
			
	if (GLUT_LEFT_BUTTON == b && GLUT_UP == state)
	{
		/*	Gz->OnMouseUp(x, y);
			updateCam = true;*/
	}
	
}

void mouseMotion(int x, int y)
{
	//Gz->SetEditMatrix(objectMatrix);
	//Gz->OnMouseMove(x, y);
}




#endif // _MAIN_
