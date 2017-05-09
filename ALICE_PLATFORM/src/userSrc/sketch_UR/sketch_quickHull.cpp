#define _MAIN_
#define _ALG_LIB_

#ifdef _MAIN_

#include "main.h"
#include "Matrix.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;


//////////////////////////////////////////////////////////////////////////


#define rx ofRandom(-1,1)




//////////////////////////////////////////////////////////////////////////
Mesh M;
vec *P;
int num = 0;
#define MAX_NUM 1000

void setup()
{
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	P = new vec[MAX_NUM];
	keyPress('c', 0, 0);
}

void update(int value)
{
	
	//qh_vertex_t *vertices = new qh_vertex_t[num];

	//for (int i = 0; i < num; ++i) {

	//	vertices[i].z = pts[i].pt.z;
	//	vertices[i].x = pts[i].pt.x;
	//	vertices[i].y = pts[i].pt.y;
	//}

	//mesh = qh_quickhull3d(vertices, num);

}

void draw()
{

	backGround(0.75);
	drawGrid(20);

	M.draw(true);
	M.draw(false);

	glPointSize(5);
		for (int i = 0; i < num; i++) drawPoint(P[i]);
	glPointSize(1);
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
	
	if (k == 'c')
	{

		num = 1000;
		for (int i = 0; i < num; i++) P[i] = vec(rx, rx, rx).normalise() * 10;

		M = quickHull(P, num);
	}


	if (k == 'b')
	{
		num = 1000;
		for (int i = 0; i < num; i++) P[i] = vec(rx, rx, rx) * 10;
		M = quickHull(P, num);

	}

	if (k == 'p')
	{
		num = 4 * 3;
		
		Matrix4 rot;
		vec plPts[4];
		plPts[0] = vec(-1, -1, 0);
		plPts[1] = vec(-1, 1, 0);
		plPts[2] = vec(1, 1, 0);
		plPts[3] = vec(1, -1, 0);

		float inc = PI * 2.0 / float(num);

		for (int i = 0; i < num; i += 4)
		{
			vec n = vec( rx,rx, rx);
			vec u = n.cross(vec(1, 0, 0));
			vec v = n.cross(u);
			u.normalise(); v.normalise(); n.normalise();
			rot.setColumn(0, u);
			rot.setColumn(1, v);
			rot.setColumn(2, n);
			rot.setColumn(3, vec(rx, rx, 0).normalise() * 10);
			for (int j = 0; j < 4; j++)P[i + j] = rot * ( plPts[j] * 1 );
		}

		M = quickHull(P, num);
	}

	if (k == 'w')
		M.writeOBJ("data/conv", "", M.positions, false);
}





#endif // _MAIN_

