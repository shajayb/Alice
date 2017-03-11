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
vector<rigidCube> RBodies;
vector<rigidCube> RSBodies;
int nr = 5; int nrs = 5;
vec *PCur,*PNext;



stack<vec> stk ;
int RES = 8;
#define RES_DEF 8
vec PConvex[(RES_DEF + 1)*(RES_DEF + 1)  *    (RES_DEF + 1) * (RES_DEF + 1)];
int nCol;
//Matrix4 T;
bool run = false;
ButtonGroup B;
SliderGroup S;
Interpolator matProp;

float simTime = 0;

#define rx ofRandom(-1,1)
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
	
	//////////////////////////////////////////////////////////////////////////
	Matrix4 transM;
	float sc[3];
	sc[0] = 1; sc[1] = 1.0; sc[2] = 1.0;

	{
		RBodies.clear();

		for (int i = 0; i < nr ; i++)RBodies.push_back(*new rigidCube(M));


		for (int i = 0; i < RBodies.size(); i++)RBodies[i].setScale(sc);
		for (int i = 0; i < RBodies.size(); i++)RBodies[i].transform();

		for (int i = 0; i < RBodies.size(); i++)
		{
			transM.identity();
			transM.setColumn(3, vec(1.0 * i, 0,0));
			RBodies[i].setInitialTransformation(transM);
		}
		
	}

	////----
	{
		RSBodies.clear();
		for (int i = 0; i < nrs; i++)RSBodies.push_back(*new rigidCube(M));

		for (int i = 0; i < RSBodies.size(); i++)RSBodies[i].setScale(sc);
		for (int i = 0; i < RSBodies.size(); i++)RSBodies[i].transform();

		//
		for (int i = 0; i < RSBodies.size(); i++)
		{
			x = vec(1, rx, 0.0).normalise();;// vec(0.25, 1, 1).normalise();
			z = vec(0, 0, -1).normalise();
			y = x.cross(z).normalise();
			z = x.cross(y).normalise();

			transM.setColumn(0, x);
			transM.setColumn(1, y);
			transM.setColumn(2, z);
			transM.setColumn(3, vec( i * 1.05 + 0.65, 0.55, 1.1));

			RSBodies[i].setInitialTransformation(transM);
		}
	}
	
	//

	////----


	PCur = new vec[(RES+1)*(RES + 1)*(RES + 1)];
	PNext = new vec[(RES + 1)*(RES + 1)*(RES + 1)];

	//// ---- 

	B = *new ButtonGroup(vec(50, 350, 0));
	B.addButton(&RSBodies[0].b_Fn,  "normal force");
	B.addButton(&RSBodies[0].b_Fis, "axial");
	B.addButton(&RSBodies[0].b_Fit, "tangential");
	B.addButton(&RSBodies[0].b_Fid, "damp");
	B.addButton(&RSBodies[0].b_Fgrv,"grav");

	for (int i = 0; i < RSBodies.size(); i++)
	{
		RSBodies[i].b_Fn = true;
		RSBodies[i].b_Fis = true;
		RSBodies[i].b_Fit = true;
		RSBodies[i].b_Fid = true;
		RSBodies[i].b_Fgrv = true;

		RSBodies[i].kBearing = 0.20;
		RSBodies[i].kAxial = -0.05;
		RSBodies[i].kTan = .05;
		RSBodies[i].kVelDamp = 0.005;
		RSBodies[i].dia = 0.2;// (1.0 / (RES)) * 1.0;

	}
	run = false;

	S = *new SliderGroup(vec(50, 850, 0));
	S.addSlider(&RSBodies[0].kBearing, "kBearing");
	S.sliders[0].minVal = 0; S.sliders[0].maxVal = 1.0 / (RES*RES); // 5 * 5.2;
	
	S.addSlider(&RSBodies[0].kAxial, "kAxial");
	S.sliders[1].minVal = 0; S.sliders[1].maxVal = 2.0 / (RES*RES); //  10 * 1.2;

	S.addSlider(&RSBodies[0].kTan, "kTan");
	S.sliders[2].minVal = 0; S.sliders[2].maxVal = 1.0 / (RES*RES);

	S.addSlider(&RSBodies[0].kVelDamp, "kVelDamp");
	S.sliders[3].minVal = 0; S.sliders[3].maxVal = 1.0 / (RES*RES);

	S.addSlider(&RSBodies[0].dia, "collisionDia");
	S.sliders[4].minVal = 0; S.sliders[4].maxVal = 1.0; // 


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
	/*if(RSBodies[0].numCol>0)
	RSBodies[0].kBearing = RSBodies[0].F_grv.mag() * (RSBodies[0].cnt / RSBodies[0].numCol)*1.5;*/

	if (!run)return;
	float parameter = matProp.getValueAt(simTime);

	simTime+= RSBodies[0].dt * 30;
	
	for (int i = 0; i < RSBodies.size(); i++)
	{
		RSBodies[i].kBearing = ofMap(parameter, 0, 1, S.sliders[0].minVal, S.sliders[0].maxVal);
		RSBodies[i].kAxial = ofMap(parameter, 0, 1, S.sliders[1].minVal, S.sliders[1].maxVal);
		RSBodies[i].kTan = ofMap(parameter, 0, 1, S.sliders[2].minVal, S.sliders[2].maxVal);
		RSBodies[i].kVelDamp = ofMap(parameter, 0, 1, S.sliders[3].minVal, S.sliders[3].maxVal);
	}

}


void draw()
{

	backGround(0.75);


	glPushMatrix();
	glScalef(20, 20, 20);

	if(run)
		keyPress('n', 0, 0);


	{
		for (int i = 0; i < RBodies.size(); i++)
		{
			RBodies[i].computeGrid(PCur, RES);
			//RBodies[i].drawGridAsPoints(PCur, (RES + 1)*(RES + 1)*(RES + 1));;
			RBodies[i].draw(2, vec4(0.5, 0.5, 0.5, 1.0), false);
		}

		for (int i = 0; i < RSBodies.size(); i++)
		{
			RSBodies[i].computeGrid(PCur, RES);
			//	RSBodies[i].drawGridAsPoints(PCur, (RES + 1)*(RES + 1)*(RES + 1));;
			RSBodies[i].draw(2, vec4(0.5, 0.5, 0.5, 1.0), false);
		}
	}

	glPopMatrix();

	//////////////////////////////////////////////////////////////////////////

	B.draw();
	S.draw();
	matProp.draw();

	drawVector(RSBodies[0].T, vec(50, 1100, 0), "T");
	drawVector(RSBodies[0].F, vec(50, 1125, 0), "F");
	drawVector(RSBodies[0].F_grv * RSBodies[0].cnt , vec(50, 1075, 0), "F_grv");
	setup2d();
		char s[200];
		sprintf(s, "%i col", RSBodies[0].numCol);
		drawString(s, 50, 1150);

		sprintf(s, "%i cnt", RSBodies[0].cnt);
		drawString(s, 50, 1175);
	restore3d();
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

		for (int i = 0; i < RSBodies.size(); i++) RSBodies[i].dia = RSBodies[0].dia;
	}
}

void keyPress(unsigned char k, int xm, int ym)
{


	if (k == 'r') run = !run;

	if (k == '=')RSBodies[0].kBearing *= 1.1;
	if (k == '-')RSBodies[0].kBearing /= 1.1;

	if (k == 'n')
	{
		for (int it = 0; it < 1; it++)
		{
			for (int i = 0; i < RSBodies.size(); i++)RSBodies[i].resetForces();

			for (int i = 0; i < RSBodies.size(); i++)
			{
				int np = RSBodies[i].computeGrid(PCur, RES);
				RSBodies[i].addSelfWeightAndTorque(PCur);

				for (int n = 0; n < RBodies.size(); n++)
				{
					
					if (RSBodies[i].cog.distanceTo(RBodies[n].cog) > 5)continue;
					RBodies[n].computeGrid(PNext, RES);
					RSBodies[i].computeContactsAndForces(RBodies[n], PCur, PNext, RES, PConvex, stk);
			
				}
				
			}

			//
			//for (int i = 0; i < RSBodies.size(); i++)
			//{
			//	RSBodies[i].computeGrid(PCur, RES);

			//	for (int n = i; n < RSBodies.size(); n++)
			//	{
			//		if (i == n)continue;
			//		
			//		RSBodies[n].computeGrid(PNext, RES);
			//		//RSBodies[i].computeContactsAndForces(RSBodies[n], PCur, PNext, RES,PConvex, stk);

			//	}

			//}

			/*for (int i = 0; i < RSBodies.size(); i++)RSBodies[i].updatePositionAndOrientation();*/
		}



		
	}

	if( k == 'N')for (int i = 0; i < RSBodies.size(); i++)RSBodies[i].updatePositionAndOrientation();
}





#endif // _MAIN_

