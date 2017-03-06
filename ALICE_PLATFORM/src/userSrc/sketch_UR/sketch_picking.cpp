#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
#include "interface.h"

using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;


bool isInRectangle( vec &a, vec &mn , vec &mx )
{
	return (mn < a && a < mx);
}
// model - view - controller (MVC) paradigm / pattern / template 

interface INTRF;

int const dim = 10000;
vec pts[dim];
int msx, msy;

void setup()
{
	INTRF.clkPts.clear();
	for (int i = 0; i < dim; i++)pts[i] = vec( ofRandom(-20, 20), ofRandom(-20, 20), ofRandom(-20, 20));
}

void draw()
{

	double ht = 350;
	double ht_plus = 20;

	backGround(0.75);
	drawGrid(20);

	Matrix4 MV, P;
	GLdouble matModelView[16], matProjection[16];
	int viewport[4];
	// get matrices and viewport:
	glGetFloatv(GL_MODELVIEW_MATRIX, MV.m);
	glGetFloatv(GL_PROJECTION_MATRIX, P.m);
	glGetIntegerv(GL_VIEWPORT, viewport);

	MV.transpose();
	P.transpose();

	/*vec halfDiag = vec(1, 1, 0).normalise() * 20;
	vec mn = vec(msx, msy, 0) - halfDiag;
	vec mx = vec(msx, msy, 0) + halfDiag;*/
	vec mn, mx;
	mx = vec(INTRF.msx, INTRF.msy, 0);
	mn = vec(INTRF.cur_msx, INTRF.cur_msy, 0);
	mx.y *= -1;
	mn.y *= -1;
	
	mn += vec(viewport[2] * 0.5, viewport[3] * 0.5, 0);
	mx += vec(viewport[2] * 0.5, viewport[3] * 0.5, 0);
	
	if (mx.x < mn.x)swap(mx.x, mn.x);
	if (mx.y < mn.y)swap(mx.y, mn.y);
	if (mx.z < mn.z)swap(mx.z, mn.z);
	//////////////////////////////////////////////////////////////////////////

	glPointSize(5);

		setup2d();

			for (int i = 0; i < dim; i++)
			{
				vec pt = worldToScreen(pts[i], MV, P, viewport);
				( isInRectangle(pt,mn,mx) ) ? glColor3f(1, 0, 0): glColor3f(1, 1,1);
				drawPoint(pt);
			}

			//////////////////////////////////////////////////////////////////////////

			wireFrameOn();
				drawRectangle(mn, mx);
			wireFrameOff();


		restore3d();


		drawVector(mn, vec(50, 75, 0), "mn");
		drawVector(mx, vec(50, 90, 0), "mx");

	glPointSize(1);
	/////////


	INTRF.draw();

}



void mousePress(int b, int state, int x, int y)
{
	INTRF.mousePress(b, state, x, y);
	if (glutGetModifiers() == GLUT_ACTIVE_ALT)
	{
		msx = x;// -winW * 0.5;
		msy = y;// winH * 0.5 - y;
		updateCam = false;
	}
	else updateCam = true;
}


///////////////////////////////////////////////////////////////////////////////////////////////
void update(int value)
{

}

void mouseMotion(int x, int y)
{
	INTRF.mouseMotion(x, y);
}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'r')setup();

	INTRF.keyPress(k, xm, ym);
}





#endif // _MAIN_
