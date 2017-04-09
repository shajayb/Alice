

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

//////////////////////////////////////////////////////////////////////////


void setup()
{

	int sizeInKB = sizeof(rigidCube) / 1024;
	cout << (sizeInKB * 100000) / 1000 << " sizeInMB : 100,000 micro-units " << endl;

		R1 = rigidCube(1.0);
		R2 = rigidCube(1.0);


		R2.setScale(0.1, 1.2, 1.2);
		R2.transform();

		//Matrix4 Tr;
		//vec x, y, z;
		//x = vec(0.978227, 0.15, 0.01).normalise();
		//z = vec(-0.15, 0.97, -0.15).normalise();
		//y = x.cross(z).normalise();

		//Tr.setColumn(0, x);
		//Tr.setColumn(1, y);
		//Tr.setColumn(2, z);
		//Tr.setColumn(3, vec(-0.0, 0,0.0));
		//R2.setInitialTransformation(Tr);

		double x = 0;
		double y = 0;
		double z = 0;

		MeshFactory fac;
		M = fac.createPlatonic(1.0 / sqrt(2), 6);
		Matrix4 Tr;
		vec u, v, n, c;
		u = vec(1, 1, 0).normalise();
		n = vec(0,0, 1).normalise();
		v = u.cross(n).normalise();
		n = v.cross(u).normalise();


		Tr.setColumn(0, u);
		Tr.setColumn(1, v);
		Tr.setColumn(2, n);
		Tr.setColumn(3, vec(0, 0, 0.5));
		Tr.invert();
		for (int i = 0; i < 8; i++)M.positions[i] = M.positions[i] * Tr;
		//Tr.invert();

		for (int i = 0; i < RES_DEF; i++)
		{
			for (int j = 0; j < RES_DEF; j++)
			{
				RCS[i][j] = *new rigidCube(M);

				vec u, v, n, c;
				u = vec(1, 1, 0).normalise();
				n = vec(0,0, 1).normalise();
				v = u.cross(n).normalise();
				n = v.cross(u).normalise();
				c = vec(i * 1, 0.1 * i + j * 0.5, j*1);
				//c = vec(i* 1.05,0,0);

				/*Tr.setColumn(0, u);
				Tr.setColumn(1, v);
				Tr.setColumn(2, n);*/
				Tr.setColumn(3, c);
				RCS[i][j].setInitialTransformation(Tr);
				
				/*RCS[i][j].setScale(.2, 1., 1.);
				RCS[i][j].transform();*/
				//RCS[i][j].transMatrix.identity();
				//RCS[i][j].setInitialTransformation(Tr);
			}

		}

		S = *new SliderGroup();
		S.addSlider(&zTol, "zTol");
		S.sliders[0].max.x += 200;
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
	glScalef(5, 5, 5);

	for (int i = 0; i < RES_DEF; i++)
		for (int j = 0; j < RES_DEF; j++)
			RCS[i][j].draw();


	startTimer();
	//int i, j;
	for (int i = 0; i < RES_DEF; i++)
		for (int j = 0; j < RES_DEF; j++)
		{
			//i = j = 1;
			int prev = (i + RES_DEF - 1) % RES_DEF;
			int next = (i + 1) % RES_DEF;
			int up = (j + RES_DEF - 1) % RES_DEF;
			int down = (j + 1) % RES_DEF;
			
			//RCS[i][j].computeCollisionInterfaces(RCS[next][j], zTol);
			RCS[i][j].computeCollisionInterfaces(RCS[prev][j], zTol);
			//RCS[i][j].computeCollisionInterfaces(RCS[i][up], zTol);
			RCS[i][j].computeCollisionInterfaces(RCS[i][down], zTol);

			char s[200];
			sprintf(s, " --------col check %i & %i", i, next);
			deferDraw_addElement(s);
		}
		

		
	long time = endTimer();
	char s[200];
	sprintf(s, "%i compute time in ms ", time);
	deferDraw_addElement(s);

	glPopMatrix();

	vec str(50, 150, 0);
	//for( auto v:deferDrawElements)
	{
		auto v = deferDrawElements.back();
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
	pt.z += 0.001;
	pt.print();

	if (k == 'n')
	{
		
		trans.rotateX(1);
		trans.rotateZ(1);
		normal = trans * normal;
		R2.transMatrix = trans;
		R2.transform();
		cout << trans << endl;
	}




}





#endif // _MAIN_

