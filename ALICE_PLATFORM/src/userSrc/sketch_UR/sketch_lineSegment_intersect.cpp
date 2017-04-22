



#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "graph.h"

using namespace std;
using namespace std::experimental;



#define rx ofRandom(-1,1)
bool run = false;
//////////////////////////////////////////////////////////////////////////
vec pts[4];


void setup()
{
	pts[0] = vec(rx, rx, 0);
	pts[2] = vec(rx, rx, 0);
	pts[1] = pts[0] * -1;
	pts[3] = pts[2] * -1;


}


void update(int value)
{
	if (run)keyPress('x', 0, 0);

}

vec normal(0, 0, 1);
void draw()
{

	backGround(0.8);
	glColor3f(0, 0, 0); 
	drawGrid(10);




	//////////////////////////////////////////////////////////////////////////
	glLineWidth(5);
	glColor3f(1, 0, 0);

	drawLine(pts[0], pts[1]);
	drawLine(pts[2], pts[3]);
	
	glLineWidth(1);
	glColor3f(0, 0, 0);

	double u, v;
	bool closeToVerticalPlane;
	vec pt = Intersect_linesegments(pts, u, v, closeToVerticalPlane);
	glPointSize(10);
	drawPoint(pt);

	drawString("0", pts[0]);
	drawString("1", pts[1]);
	drawString("2", pts[2]);
	drawString("3", pts[3]);

	//////////////////////////////////////////////////////////////////////////




	deferDraw_addElement(float(closeToVerticalPlane)," closeToVert");

	//////////////////////////////////////////////////////////////////////////

	vec str(50, 150, 0);
	for (auto v : deferDrawElements)
	{
		//auto v = deferDrawElements.back();
		(v.TYP == VECTOR) ? drawVector(v.P, str, v.text) : drawString_tmp(v.text, str, true);
		str.y += 15;
	}
	deferDrawElements.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		B.performSelection(x, y);
		S.performSelection(x, y, HUDSelectOn);
	}
}



void mouseMotion(int x, int y)
{
	{
		S.performSelection(x, y, HUDSelectOn);
	}
}


void keyPress(unsigned char k, int xm, int ym)
{

	if (k == ' ')run = !run;

	if (k == 't')
	{
		Matrix4 T;
		vec n(rx, rx, rx);
		n.normalise();
		vec u = n.cross(vec(1, 0, 0)).normalise();
		vec v = u.cross(n).normalise();
		vec cen(rx, rx, rx);
		cen.normalise();
		cen *= 0.1;
		T.setColumn(0, u);
		T.setColumn(1, v);
		T.setColumn(2, n);
		T.setColumn(3, cen);

		for (int i = 0; i < 4; i++)pts[i] = T * pts[i];
	}

	if (k == 'T')
	{
		double u, v;
		bool closeToVerticalPlane;

		int i = 0;
		for ( i = 0; i < 1000; i++)
		{
			keyPress('t', 0, 0);
			vec pt = Intersect_linesegments(pts, u, v, closeToVerticalPlane);
			if (closeToVerticalPlane)break;
		}

		cout << i << " - -coutn" << endl;
	}

	if (k == 'y')
	{
		Matrix4 T;
		T.rotateX(90);

		for (int i = 0; i < 4; i++)pts[i] = T * pts[i];
	}

	if (k == 'Y')
	{
		Matrix4 T;
		T.rotateY(90);

		for (int i = 0; i < 4; i++)pts[i] = T * pts[i];
	}


	if (k == 'i')
	{
	
		vec u =(pts[1] - pts[0]).normalise();
		vec n = (u).cross(pts[3] - pts[2]).normalise();
		vec v = u.cross(n).normalise();
		vec cen;
		for (int i = 0; i < 4; i++)cen += pts[i];
		cen /= 4.0;

		Tr.setColumn(0, u);
		Tr.setColumn(1, v);
		Tr.setColumn(2, n);
		Tr.setColumn(3, cen);

		keyPress('f', 0, 0);

		
	}
	if (k == 'f')
	{
		Tr.invert();
		for (int i = 0; i < 4; i++)pts[i] = Tr * pts[i];
	}


}





#endif // _MAIN_

