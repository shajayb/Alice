#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

Robot_Symmetric nachi;
vec target(10, 0, 0);

importer ptsReader;
SliderGroup S;
ButtonGroup B;

char gcode[600];
ofstream myfile, myfile_write;
vec diff;


physics phy;

bool run = false;

void addParticlesOncircle(vec &cen)
{
	for (int i = 0; i < 32; i++)
	{
		float x = 20.0 * sin(float(i) * 2.0 * PI / 32);
		float y = 20.0 * cos(float(i) * 2.0 * PI / 32);
		int n = phy.makeParticle(vec(x, y, 0) + cen, 1.0, false);
		phy.p[n].v = (phy.p[n].p - cen).normalise();
		phy.makeSpring(n, n - 1, 0.1);

	}
}

void setup()
{

	phy = *new physics(vec(0, 0, -1));
	phy.calcGravity = false;



}


void update(int value)
{
	if (run)
		phy.UpdateParticles(0.5, 2);



	for (int i = 0; i < phy.np; i++)
		for (int j = 0; j < phy.np; j++)
		{
			if (i == j)continue;
			vec d = phy.p[i].p - phy.p[j].p;
			//if (d * d < pow(0.05,2.0) )phy.p[i].fixed = phy.p[j].fixed = true;
		}

}

void draw()
{

	backGround(0.75);
	drawGrid(20.0);

	phy.display(1, 1, false, false);

}


#define att phy.makeAttraction(i, n, 1e-8, 10, 0);

void keyPress(unsigned char k, int xm, int ym)
{


	if (k == 'r')setup();
	if (k == 'u')run = !run;

	if (k == 'a')
	{
		vec a(ofRandom(-2, 2), ofRandom(-2, 2), 0);
		int n = phy.makeParticle(a, 1.0, false);
		int n2 = ofRandom(0, phy.np - 2);
		phy.makeSpring(n, n2, .1);

		for (int i = 0; i < phy.np - 1; i++)att
	}

	if (k == 's')
	{
		vec a(ofRandom(-2, 2), ofRandom(-2, 2), 0);
		int n = phy.makeParticle(a, 1.0, false);
		int n2 = phy.np - 2;
		phy.makeSpring(n, n2, .1);

		for (int i = 0; i < phy.np - 1; i++)att


	}

	if (k == 'b')
	{
		addParticlesOncircle(vec(ofRandom(-20, 20), ofRandom(-20, 20), 0));

	}
	if( k == 'e')phy.calcCharge != phy.calcCharge;

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
}

