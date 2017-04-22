//#define _MAIN_



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
#define RES 6
vector<vec> C_HULL;
vector<vec> subPts;




void setup()
{
	subPts.clear();
	C_HULL.clear();
	float inc = PI * 2.0 / float(RES);
	float rad = 10.0;
	for (int i = 0; i < RES; i++)
	{
		float R = rad;// i * 5;
		float x = cos(inc * i) * R;
		float y = sin(inc * i) * R;
		C_HULL.push_back( vec(x, y, 0) );
	}
		
	////

	vec *ptC;
	ptC = new vec[C_HULL.size()];
	for (int i = 0; i < C_HULL.size(); i++)ptC[i] = C_HULL[i];

	subDivideHull(ptC, C_HULL.size(), subPts,4);

}


void update(int value)
{

}

vec normal(0, 0, 1);
void draw()
{

	backGround(0.8);
	glColor3f(0, 0, 0); 
	drawGrid(10);

	
	glPointSize(4);

	for (auto T : TRIS)T.draw();
	for (auto P : subPts)drawPoint(P);

	glPointSize(1);



	//////////////////////////////////////////////////////////////////////////
	glLineWidth(5);
	glColor3f(1, 0, 0);

	glLineWidth(1);


	//////////////////////////////////////////////////////////////////////////
	deferDraw_addElement(float(TRIS.size()), "tris");
	deferDraw_addElement(float(subPts.size()), "uniq.pts");

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

}





#endif // _MAIN_

