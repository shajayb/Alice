#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graph.h"
#include "newPhysics.h";
#include "ActiveGraph.h"
#include "utilities.h"
#include "graphStack.h"


////////////////////////////////////////////////////////////////// GLOBAL VARIABLES  //////////////////////////////////////////////////////////////////

rigidCube RG;
activeGraph AG;
vector<activeGraph> AGStack;
int RES = 6;
vec *P1 = new vec[RES*RES*RES];
vec *P2 = new vec[RES*RES*RES];

ButtonGroup B;
bool showPoints = false;
bool showSpheres = false;

bool run = false;
vector<int2> nbor_RCs;
int2 strId(0, 0);

////////////////////////////////////////////////////////////////// temporary utility functions //////////////////////////////////////////////////////////////////


#define rx ofRandom(-20,20)
vec getCenterOfRC(int2 id)
{
	return AGStack[id.l].RCsOnCurve[id.n].transMatrix.getColumn(3);
}
void resetRCForces(int2 id)
{
	AGStack[id.l].RCsOnCurve[id.n].resetForces();
}
void getNBors( int2 &id , vector<int2> &nBors , double D = 0.5 )
{
	nBors.clear();
	vec cen = getCenterOfRC(id);

	for (int l = 0; l < AGStack.size(); l++)
	{
		for (int n = 0; n < AGStack[l].RCsOnCurve.size(); n++)
		{
			int2 id_other(l, n);
			if (id_other == id)continue;
			if (id_other.l == id.l)continue;

			vec cen_other = getCenterOfRC(id_other);
			if ((cen_other - cen) * (cen_other - cen) < D * D)nBors.push_back(id_other);
		}
	}

}

////////////////////////////////////////////////////////////////// MAIN PROGRAM //////////////////////////////////////////////////////////////////

void setup()
{
	AGStack.clear();
	RG = *new rigidCube();
	
	importer imp = *new importer("data/curve.txt", 10000, 1.0);
	imp.readPts_p5();

	//for (int j = 0; j < 3 ; j++)
	{

		vec *pts = new vec[imp.nCnt];
		for (int i = 0; i < imp.nCnt; i++)pts[i] = imp.nodes[i].pos + vec(0,0,float(0) * 0.25);

		AG = *new activeGraph();
		AG.constructFromPoints(pts, imp.nCnt);
		AG.fixEnds();
		AG.populateRigidBodies(0.1);
		AGStack.push_back(AG);
		
	}

	B = *new ButtonGroup();
	B.addButton( &showPoints, "showPts" );
	B.addButton( &showSpheres, "showSpheres" );

}

void update(int value)
{
	if(run)
		for (auto &ag : AGStack)
		{
			ag.smoothVertices();
			ag.populateRigidBodies(0.1);
		}
}

void draw()
{

	backGround(0.9);
	drawGrid(20.0);

	for (auto &ag : AGStack) 
		showPoints ? ag.display(P1, RES) : ag.display();

	//	
	
	{

		//for (auto &ag : AGStack)
		//	for (auto &rc : ag.RCsOnCurve)rc.resetForces();
		

		int2 id = strId;// int2(l, n);
		nbor_RCs.clear();
		getNBors(id, nbor_RCs, 0.5);
		
		resetRCForces(id);
		for (auto &i : nbor_RCs)resetRCForces(i);

		for (auto nbor : nbor_RCs)
		{
			AGStack[id.l].RCsOnCurve[id.n].computeContactsAndForces(AGStack[nbor.l].RCsOnCurve[nbor.n], P1, P2, RES);

			AGStack[nbor.l].RCsOnCurve[nbor.n].draw(2, vec4(1, 0, 0, 1));
			if (showSpheres)AGStack[nbor.l].RCsOnCurve[nbor.n].drawGrid(P2, RES*RES*RES);
		}

		AGStack[strId.l].RCsOnCurve[strId.n].draw(4, vec4(1, 1, 1, 1));
		if (showSpheres)AGStack[strId.l].RCsOnCurve[strId.n].drawGrid(P1, RES*RES*RES);
	}






	/////

	B.draw();
}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'r')run = !run;
	if (k == 'R')setup();

	if (k == '1')
	{
		AGStack[0].smoothVertices();
		AGStack[0].populateRigidBodies(0.1);
	}

	if (k == '2')
	{
		AGStack[1].smoothVertices();
		AGStack[1].populateRigidBodies(0.1);
	}

	if (k == '3')
	{
		AGStack[2].smoothVertices();
		AGStack[2].populateRigidBodies(0.1);
	}

	if (k == 'c')
	{
		RES++;
		P1 = new vec[RES*RES*RES];
		P2 = new vec[RES*RES*RES];
	}

	if (k == 'n')strId.n++;
	if (k == 'l')strId.l++;
	if (k == 'N')strId.n--;
	if (k == 'L')strId.l--;

	
	if (k == 'q')
	{

		cout << "Q" << endl;

		for (auto &ag : AGStack)
			for (auto &rc : ag.RCsOnCurve)rc.resetForces();

		for (int l = 1; l < AGStack.size(); l++)
		{
			for (int n = 0; n < AGStack[0].RCsOnCurve.size(); n++)
			{
				int2 id = int2(l, n);
				nbor_RCs.clear();
				getNBors(id, nbor_RCs, 0.5);

				for (auto nbor : nbor_RCs)
					AGStack[id.l].RCsOnCurve[id.n].computeContactsAndForces(AGStack[nbor.l].RCsOnCurve[nbor.n], P1, P2, RES);
			}
		}
	}

	if (k == 's')
	{

		for (int j = 1; j < 5; j++)
		{
			AG = *new activeGraph();
			AG.constructFromToroidalGraph(AGStack[j - 1]);
			for (int i = 0; i < AG.n_v; i++)AG.positions[i] += vec(0, 0, 0.25);

			AG.fixEnds();
			AG.smoothVertices();
			AG.populateRigidBodies(0.1);
			AGStack.push_back(AG);
		}
	}

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{

}



