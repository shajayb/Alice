
#define _MAIN_


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

Graph G_RB;
Graph G_RBS;


#define rx ofRandom(-1,1)
rigidCube R1, R2;
vec pts[4];
Matrix4 trans;
vec pt(0.9, 0.25, 0);

//////////////////////////////////////////////////////////////////////////


void setup()
{


		R1 = rigidCube(1.0);
		R2 = rigidCube(1.0);


		R2.setScale(1.0);
		R2.transform();

		Matrix4 Tr;
		vec x, y, z;
		x = vec(1, 1, 0).normalise();
		z = vec(0, 0, 1).normalise();
		y = x.cross(z).normalise();

		/*Tr.setColumn(0, x);
		Tr.setColumn(1, y);
		Tr.setColumn(2, z);*/
		Tr.setColumn(3, vec(1.02, 0,0.0));
		R2.setInitialTransformation(Tr);

	
		pts[0] = vec(0, 0, 1);
		pts[1] = vec(1, 0, 1);
		pts[2] = vec(1, 1.0, 1);
		pts[3] = vec(0, 1.0, 1);
		
		pt = vec(5,5,5);
		//for (int i = 0; i < 25; i++)pts[0].y += 0.001;
}


void update(int value)
{
	//pt.z += 0.001;

}

vec normal(0, 0, 1);
void draw()
{

	backGround(0.8);
	glColor3f(0, 0, 0);drawGrid(10);


	glPushMatrix();
	glScalef(12, 12, 12);

		R1.draw();
		R1.isFacetoFace(R2, 0,2);
		R2.draw();
	glPopMatrix();
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
	pt.z += 0.001;
	pt.print();

	if (k == 'n')
	{
		
		trans.rotateX(1);
		trans.rotateZ(1);
		normal = trans * normal;
	}

	if( k == 'b')for (int i = 0; i < 25; i++)pts[1].z -= 0.001;
	if (k == 'v')for (int i = 0; i < 25; i++)pts[1].y -= 0.001;
}





#endif // _MAIN_

