#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include "interpolate.h"
#include "utilities.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;

// model - view - controller (MVC) paradigm / pattern / template 



//////////////////////////////////////////////////////////////////////////
#define rx ofRandom(-10,10)
#define RES 10

vec points[RES*RES];
stack<vec> S;

void setup()
{

		keyPress('a', 0, 0);
	

	//convexHull(points, RES, S);
	//while (!S.empty())
	//{
	//	vec p = S.top();
	//	cout << "(" << p.x << ", " << p.y << ")" << endl;
	//	S.pop();
	//}
}


void update(int value)
{
	
}


void draw()
{

	backGround(0.75);



	glPointSize(5);
	for (int i = 0; i < RES * RES; i++)drawPoint(points[i]);

	vec pts[RES*RES];
	for (int i = 0; i < RES * RES; i++) pts[i] = points[i];

	for (int i = 0; i < 1000; i++)convexHull(pts, RES * RES, S);

	drawConvexHull(S);
	
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{

}



void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{
	float inc = PI * 2.0 / float(RES);

	//for (int i = 0; i < RES; i++)
	//	points[i] = vec( sin(inc*i), cos(i*inc),0).normalise() * rx;
	Matrix3 trans;
	vec x(rx, rx, 0); x.normalise();
	vec z(0, 0, 1); z.normalise();
	vec y = x.cross(z).normalise();
	trans.setColumn(0, x); trans.setColumn(1, y); trans.setColumn(2, z);

	for (int i = 0; i < RES; i++)
		for (int j = 0; j < RES; j++)
		{
			vec P = vec(i * 1, j * 1, 0);
			P = trans * P;
			points[i*RES + j] = P;
		}

}





#endif // _MAIN_
