#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;

// model - view - controller (MVC) paradigm / pattern / template 

#include "rigidCube.h"
Mesh M;
rigidCube RC1, RC2, RC3;
vec *P1,*P2;
int RES = 4;
Matrix4 T;
bool run = false;
ButtonGroup B;
SliderGroup S;
Interpolator matProp;

float simTime = 0;
//////////////////////////////////////////////////////////////////////////

void setup()
{

	MeshFactory fac;
	M = fac.createPlatonic(1.0 / sqrt(2), 6);

	Matrix4 Tr;
	vec x, y, z;
	x = vec(1, 1, 0).normalise();
	z = vec(0, 0, 1).normalise();
	y = x.cross(z).normalise();

	Tr.setColumn(0, x);
	Tr.setColumn(1, y);
	Tr.setColumn(2, z);
	Tr.setColumn(3, vec(0, 0, 0.5));
	Tr.invert();
	for (int i = 0; i < 8; i++) M.positions[i] = Tr * M.positions[i];
	
	RC1 = *new rigidCube(M);
	RC2 = *new rigidCube(M);
	RC3 = *new rigidCube(M);

	////----
	Matrix4 transM;
	x = vec(1, 0, 0).normalise();;// vec(0.25, 1, 1).normalise();
	z = vec(0, 0, -1).normalise();
	y = x.cross(z).normalise();
	z = x.cross(y).normalise();
	
	transM.setColumn(0, x);
	transM.setColumn(1, y);
	transM.setColumn(2, z);
	transM.setColumn(3, vec(1.0, 0, 5.1));
	
	RC2.setInitialTransformation(transM);
	//RC2.P = vec(3, 0, 0);

	////----

	transM.identity();
	x = vec(1, 1, -1).normalise();
	z = vec(0, 0, -1).normalise();
	y = x.cross(z).normalise();
	z = x.cross(y).normalise();
	transM.setColumn(0, x);
	transM.setColumn(1, y);
	transM.setColumn(2, z);
	transM.setColumn(3, vec(1.5, 0, -1));
	RC3.setInitialTransformation(transM);

	////----

	P1 = new vec[(RES+1)*(RES + 1)*(RES + 1)];
	P2 = new vec[(RES + 1)*(RES + 1)*(RES + 1)];

	//// ---- 

	B = *new ButtonGroup(vec(50, 350, 0));
	B.addButton(&RC2.b_Fn, "normal force");
	B.addButton(&RC2.b_Fis, "axial");
	B.addButton(&RC2.b_Fit, "tangential");
	B.addButton(&RC2.b_Fid, "damp");
	B.addButton(&RC2.b_Fgrv, "grav");

	RC2.b_Fn = true;
	RC2.b_Fis = true;
	RC2.b_Fit = true;
	RC2.b_Fid = true;
	RC2.b_Fgrv = true;
	run = false;

	S = *new SliderGroup(vec(50, 850, 0));
	S.addSlider(&RC2.kBearing, "kBearing"); 
	S.sliders[0].minVal = 0; S.sliders[0].maxVal = 5 * 5.2;
	
	S.addSlider(&RC2.kAxial, "kAxial");
	S.sliders[1].minVal = 0 ; S.sliders[1].maxVal = 10 * 1.2;

	S.addSlider(&RC2.kTan, "kTan");
	S.sliders[2].minVal = 0; S.sliders[2].maxVal = 1;

	S.addSlider(&RC2.kVelDamp, "kVelDamp");
	S.sliders[3].minVal = 0; S.sliders[3].maxVal = 1;

	S.addSlider(&RC2.dia, "collisionDia");
	S.sliders[4].minVal = 0; S.sliders[4].maxVal = 1;

	RC2.kBearing = 1.0;
	RC2.kAxial = 0.15;
	RC2.kTan = .05; 
	RC2.kVelDamp = 0.15;
	RC2.dia = (1.0 / (RES)) * 1.0;

	//////////////////////////////////////////////////////////////////////////
	#define SEG 4
	float yValues[SEG];
	for (int i = 0; i < SEG; i++)yValues[i] = float(i)/float(SEG);
	
	yValues[0] = 0.001;
	yValues[1] = 0.0015;
	yValues[2] = 1.0;
	yValues[3] = 1.0;
	matProp = *new Interpolator(SEG, vec(50, 750, 0), 350, 100, yValues );
	matProp.dataMin = 0.0;
	matProp.dataMax = 1.0;
	simTime = 0.0;
}


void update(int value)
{
	if (!run)return;
	float parameter = matProp.getValueAt(simTime);

	simTime+= RC2.dt * 30;
	
	RC2.kBearing = ofMap(parameter, 0, 1, S.sliders[0].minVal, S.sliders[0].maxVal);
	RC2.kAxial =   ofMap(parameter, 0, 1, S.sliders[1].minVal, S.sliders[1].maxVal);
	//RC2.kTan =     ofMap(parameter, 0, 1, S.sliders[2].minVal, S.sliders[2].maxVal);
	//RC2.kVelDamp = ofMap(parameter, 0, 1, S.sliders[3].minVal, S.sliders[3].maxVal);

}


void draw()
{

	backGround(0.75);

	if(run)
	for (int i = 0; i < 30; i++)
	{
		RC1.resetForces();
		RC2.resetForces();
		RC3.resetForces();

		RC1.computeGrid(P1, RES);
		RC2.computeGrid(P2, RES);
		RC2.computeContactsAndForces(RC1, P2, P1, RES);

		RC3.computeGrid(P1, RES);
		RC3.drawGridAsPoints(P1, (RES + 1)*(RES + 1)*(RES + 1));;

		RC2.computeContactsAndForces(RC3, P2, P1, RES);
		RC2.updatePositionAndOrientation();

	}

	RC1.computeGrid(P1, RES);
	RC1.drawGridAsPoints(P1, (RES + 1)*(RES + 1)*(RES + 1));;
	RC1.draw(2,vec4(0.5,0.5,0.5,1.0),false);

	RC2.drawGridAsPoints(P2, (RES + 1)*(RES + 1)*(RES + 1));;
	RC2.draw(2, vec4(0.5, 0.5, 0.5, 1.0), false);
	RC2.drawMatrix(RC2.transMatrix, vec(50, 50, 0));

	RC3.draw(2, vec4(0.5, 0.5, 0.5, 1.0), false);

	//////////////////////////////////////////////////////////////////////////

	B.draw();
	S.draw();
	matProp.draw();

	//setup2d();
		drawVector(RC2.F, vec(50, 1100, 0), "RC.F");
		drawVector(RC2.F_it, vec(50, 1125, 0), "RC.Fit");
	//restore3d();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		B.performSelection(x, y);
		S.performSelection(x, y, HUDSelectOn);
		matProp.performselection(x, y, HUDSelectOn);
	}
		
}



void mouseMotion(int x, int y)
{
	{
		S.performSelection(x, y, HUDSelectOn);
		matProp.performselection( x,  y, HUDSelectOn);
	}
}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'c')
	{

		RC1.resetForces();
		RC2.resetForces();
		RC3.resetForces();

		RC1.computeGrid(P1, RES);
		RC2.computeGrid(P2, RES);
		RC2.computeContactsAndForces(RC1, P2, P1, RES);

		RC3.computeGrid(P1, RES);
		RC3.drawGridAsPoints(P1, (RES + 1)*(RES + 1)*(RES + 1));/*;*/

		RC2.computeContactsAndForces(RC3, P2, P1, RES);
		RC2.updatePositionAndOrientation();


	}

	if (k == 'v')
	{

		vec x = vec(rx, rx, 0).normalise();
		vec z = vec(0, 0, 1).normalise();
		vec y = x.cross(z).normalise();

		T.setColumn(0, x);
		T.setColumn(1, y);
		T.setColumn(2, z);
		T.setColumn(3, vec(1, 1, 2));


		RC2.transMatrix.invert();
		RC2.transform();

		RC2.transMatrix = T;
		RC2.transform();
		RC2.computeGrid(P2, RES);
	}

	if (k == 'r') run = !run;
}





#endif // _MAIN_
