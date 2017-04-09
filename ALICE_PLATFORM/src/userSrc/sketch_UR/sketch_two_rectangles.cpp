
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


Interpolator matProp;

float simTime = 0;

Graph G_RB;
Graph G_RBS;


#define rx ofRandom(-1,1)
rigidCube R1, R2;

rigidCube RCS[RES_DEF][RES_DEF];
Matrix4 trans;
vec pt(0.9, 0.25, 0);

double zTol = 0.2;
double angleR = -45;
vec cenCur(0.0, 0, 1);
bool run = false;
//////////////////////////////////////////////////////////////////////////

void inverse()
{

	R2.inverseTransform();
	R2.transMatrix.identity();
	R2.transform();


}

void setVariables(double ang, vec cen )
{

	inverse();

	R2.transMatrix.identity();
	R2.transMatrix.rotateZ(ang);
	R2.transMatrix.setColumn(3, cen);
	R2.transform();


}



void setup()
{

	int sizeInKB = sizeof(rigidCube) / 1024;
	cout << (sizeInKB * 100000) / 1000 << " sizeInMB : 100,000 micro-units " << endl;

		R1 = rigidCube(1.0);
		R1.setScale(2.0, 2.0, 1);
		R1.transform();
		R1.transMatrix.identity();
		R1.transMatrix.setColumn(3, vec(1, 0, 0));
		R1.transform();

		
		R2 = rigidCube(1.0);

		R2.setScale(2.0, 4.0, 1);
		R2.transform();
		R2.transMatrix.identity();
		R2.transMatrix.rotateZ(angleR);
		R2.transMatrix.setColumn(3, cenCur );
		R2.transform();

		
}


void update(int value)
{
	if (run)keyPress('x', 0, 0);

}

vec normal(0, 0, 1);
void draw()
{

	backGround(0.8);
	glColor3f(0, 0, 0);drawGrid(10);



	glPushMatrix();
	glScalef(5, 5, 5);
	
	R1.draw();
	R2.draw();
//	R1.computeCollisionInterfaces(R2,0.15);
	R1.isFacetoFace(R2, 4, 5, 0.2);
	double area = R1.areaofConvexHUll();
	char s[20];
	sprintf(s, " %1.8f area", area);
	deferDraw_addElement(s);

	sprintf(s, " %1.2f angle", angleR);
	deferDraw_addElement(s);

	deferDraw_addElement(cenCur,"x,y,1");

	glPopMatrix();

	vec str(50, 150, 0);
	for( auto v:deferDrawElements)
	{
		//auto v = deferDrawElements.back();
		(v.TYP == VECTOR ) ? drawVector(v.P,str,v.text ) : drawString_tmp(v.text,str,true);
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

	if( k == 'x')
	{
		

		angleR += 0.01;
		setVariables(angleR,cenCur);
		R1.isFacetoFace(R2, 4, 5, 0.2);
		double areaPlus = R1.areaofConvexHUll();

		angleR -= 0.01 * 2.0;
		setVariables(angleR, cenCur);
		R1.isFacetoFace(R2, 4, 5, 0.2);
		double areaMinus = R1.areaofConvexHUll();
		angleR += 0.01;
		setVariables(angleR, cenCur);
		//////////////////////////////////////////////////////////////////////////
		
		
		//
		setVariables(angleR, cenCur + vec(0.1,0,0));
		R1.isFacetoFace(R2, 4, 5, 0.2);
		double areaXPlus = R1.areaofConvexHUll();

		setVariables(angleR, cenCur + vec(-0.1, 0, 0));
		R1.isFacetoFace(R2, 4, 5, 0.2);
		double areaXMinus = R1.areaofConvexHUll();
		//

		setVariables(angleR, cenCur + vec(0.0, 0.1, 0));
		R1.isFacetoFace(R2, 4, 5, 0.2);
		double areaYPlus = R1.areaofConvexHUll();

		setVariables(angleR, cenCur + vec(0.0, -0.1, 0));
		R1.isFacetoFace(R2, 4, 5, 0.2);
		double areaYMinus = R1.areaofConvexHUll();

		//////////////////////////////////////////////////////////////////////////
		double dA_dAng = (areaPlus - areaMinus) / 0.02;
		angleR += dA_dAng * 100;
		
		double dA_dX = (areaXPlus - areaXMinus) / 0.02;
		//cenCur.x += dA_dX * 1000;
		
		double dA_dY = (areaYPlus - areaYMinus) / 0.02;
	//	cenCur.y += dA_dY * 1000;

		printf("%1.8f,%1.8f,%1.8f \n", dA_dAng, dA_dX, dA_dY);
		setVariables(angleR,cenCur);

	}

	if (k == ' ')run = !run;

	if (k == 'z')inverse();
	if (k == 'a')setVariables(angleR,cenCur);


}





#endif // _MAIN_

