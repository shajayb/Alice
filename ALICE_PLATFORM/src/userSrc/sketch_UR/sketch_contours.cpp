
#ifdef _MAIN_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;
#include"graph.h"
#include "metaMesh.h"


////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

metaMesh M;
Graph G;

////// --- GUI OBJECTS ----------------------------------------------------

SliderGroup S;
ButtonGroup B;
double threshold = 0.5;
bool tillThreshold = true;
bool zScalars = true;
bool drawMesh = true;

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

////// ---------------------------------------------------- MODEL  ----------------------------------------------------

void setup()
{


	MeshFactory fac;
	Mesh tmp = fac.createFromOBJ("data/in.obj", 10, false, false);
	M = metaMesh(tmp);
	M.assignScalars(zScalars ? "z" : "c");
	M.createIsoContourGraph(threshold);
	double dMn, dMx;
	M.getMinMaxOfScalarField(dMn, dMx);


	///////// ------------------- GUI

	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].attachToVariable(&threshold, dMn, dMx);

	B = *new ButtonGroup(vec(50, 350, 0));
	B.addButton(&tillThreshold, "tillThreshold");
	B.addButton(&zScalars, "zScalars");
	B.addButton(&drawMesh, "drawMesh");
}

void update(int value)
{


}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(0.75);



	M.glPtSize = 5;
	M.display(true, true, drawMesh);

	glColor3f(0, 0, 0);
	if (tillThreshold) M.drawIsoContoursInRange(threshold);


	/////// ------------------- draw GUI

	S.draw();
	B.draw();
}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{



}

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		if (HUDSelectOn)M.createIsoContourGraph(threshold);

		B.performSelection(x, y);
		M.assignScalars(zScalars ? "z" : "c");
	}
}

void mouseMotion(int x, int y)
{
	//if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		if (HUDSelectOn)M.createIsoContourGraph(threshold);

		B.performSelection(x, y);

	}
}




#endif // _MAIN_
