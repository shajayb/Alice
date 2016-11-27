#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graph.h"
#include "newPhysics.h";
#include "ActiveGraph.h"
#include "utilities.h"


rigidCube RG;
activeGraph AG;
//activeGraph AG1, AG2;
vector<activeGraph> AGStack;

void setup()
{
	AGStack.clear();
	RG = *new rigidCube();
	
	importer imp = *new importer("data/curve.txt", 10000, 1.0);
	imp.readPts_p5();

	for (int j = 0; j < 2; j++)
	{

		vec *pts = new vec[imp.nCnt];
		for (int i = 0; i < imp.nCnt; i++)pts[i] = imp.nodes[i].pos + vec(0,0,float(j) * 0.25);

		AG = *new activeGraph();
		AG.constructFromPoints(pts, imp.nCnt);
		AG.fixEnds();
		AG.populateRigidBodies();
		AGStack.push_back(AG);
		
	}


}

bool run = false;

void update(int value)
{
	if(run)
		for (auto &ag : AGStack)
		{
			ag.smoothVertices();
			ag.populateRigidBodies();
		}


	//AGStack[0].RC_curve[0].computeContacts(AGStack[0].RC_curve[1]);
	//vector<rigidCube> nbors;

	//for (int i = 0; i < 1; i++)
	//{
	//	AG1 = AGStack[i];
	//	for (int j = 0; j < 1; j++)
	//	{
	//		if( i == j )
	//		AG2 = AGStack[2];
	//	}
	//}
}


#define rx ofRandom(-20,20)

void draw()
{

	backGround(0.75);
	drawGrid(20.0);

	for (auto &ag : AGStack)ag.display();

	for (auto &ag : AGStack)
		for (auto &rc : ag.RC_curve)rc.drawGrid();

}

int r = 10;
void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'r')run = !run;
	if (k == 'R')setup();

	if (k == ' ')
	{
		AGStack[0].smoothVertices();
		AGStack[0].populateRigidBodies();
	}

	if (k == 'c')
	{
		for (auto &ag : AGStack)
			for (auto &rc : ag.RC_curve)rc.reDimensionComputeGrid(r++);
		
		for (auto &ag : AGStack)
			for (auto &rc : ag.RC_curve)
				for (int i = 0; i < rc.np; i++)rc.pts[i].print();

	}

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
	}
}

void mouseMotion(int x, int y)
{

}



