

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



#define rx ofRandom(-1,1)
#define rxx ofRandom( 1,5)
rigidCube R1, R2;


double zTol = 0.2;
double angleR = -45;
vec cenCur(0.0, 0, 1);
bool run = false;
vec sc_r1, sc_r2;
vec gradient;

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

	angleR = +45.;
	cenCur = vec(0.0, 0, 1);
	sc_r1 = vec(rxx, rxx, 1);
	sc_r2 = vec(rxx, rxx, 1);

	R1 = rigidCube(1.0);
	R1.setScale(sc_r1.x,sc_r1.y,sc_r1.z);
	R1.transform();
	R1.transMatrix.identity();
	R1.transMatrix.setColumn(3, vec(rx, rx, 0));
	R1.transform();


	R2 = rigidCube(1.0);

	R2.setScale(sc_r2.x, sc_r2.y, sc_r2.z);
	R2.transform();
	R2.transMatrix.identity();
	R2.transMatrix.rotateZ(angleR);
	R2.transMatrix.setColumn(3, cenCur);
	R2.transform();

}


void update(int value)
{
	if (run)
		for (int i = 0; i < 25; i++)keyPress('x', 0, 0);

}

vec normal(0, 0, 1);
void draw()
{

	backGround(1.0);
	glColor3f(0, 0, 0);//drawGrid(10);



	glPushMatrix();
	glScalef(5, 5, 5);
	
	R1.draw();
	R2.draw();
//	R1.computeCollisionInterfaces(R2,0.15);
	R1.isFacetoFace(R2, 4, 5, 0.2);
	double area = R1.areaofConvexHUll();

	drawLine(R1.cen, R2.cen);
	drawLine(R2.cen, R2.cen + gradient);
	//
	char s[200];
	sprintf(s, "R1 %1.2f w %1.2f h", sc_r1.x, sc_r1.y);
	deferDraw_addElement(s);
	sprintf(s, "R2 %1.2f w %1.2f h", sc_r2.x, sc_r2.y);
	deferDraw_addElement(s);
	
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
		angleR += dA_dAng * 100.0;
		
		double dA_dX = (areaXPlus - areaXMinus) / 0.02;
		cenCur.x += dA_dX * 0.001;
		
		double dA_dY = (areaYPlus - areaYMinus) / 0.02;
		cenCur.y += dA_dY * 0.001;

		gradient = vec(dA_dX, dA_dY, 0.0);

		//printf("%1.8f,%1.8f,%1.8f \n", dA_dAng, dA_dX, dA_dY);
		setVariables(angleR,cenCur);
		deferDrawElements.clear();
	}

	if (k == ' ')run = !run;

	if (k == 'z')inverse();
	if (k == 'a')setVariables(angleR,cenCur);


}





#endif // _MAIN_

