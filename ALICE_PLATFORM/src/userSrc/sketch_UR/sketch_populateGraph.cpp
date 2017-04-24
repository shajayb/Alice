

#ifdef _MAIN_
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graph.h"
//#include "newPhysics.h";
#include "ActiveGraph.h"
#include "utilities.h"
#include "graphStack.h"


////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

rigidCube RG;
activeGraph AG;
vector<activeGraph> AGStack;
vector<Graph> displayStack;
Graph base;
metaMesh MM;
Mesh M;
double dMin, dMax;

int RES = 6;
vec *PCur = new vec[RES*RES*RES];
vec *PNext = new vec[RES*RES*RES];
bool run = false;
vector<int2> nbor_RCs;
int2 strId(0, 0);
#define rx ofRandom(-20,20)

////// --- GUI OBJECTS ----------------------------------------------------

//ButtonGroup B;
bool showPoints = false;
bool showSpheres = false;

////////////////////////////////////////////////////////////////// temporary utility functions //////////////////////////////////////////////////////////////////

vec getCenterOfRC(int2 id)
{
	return AGStack[id.l].RCsOnCurve[id.n].transMatrix.getColumn(3);
}
void resetRCForces(int2 id)
{
	AGStack[id.l].RCsOnCurve[id.n].resetForces();
}
void getNBors(int2 &id, vector<int2> &nBors, double D = 0.5)
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


Mesh createMesh( vector<Graph> &stack )
{
	Mesh M;

	int  vertices = stack[0].n_v;
	int layers = stack.size();
	vec minV, maxV;
	/*for (float x = minV.x; x <= maxV.x; x += (maxV.x - minV.x)*0.01)*/
	for (int i = 0; i < layers; i++)
		for (int j = 0; j < vertices; j++)
			M.createVertex(stack[i].positions[j]);


	Vertex *fVerts[4];
	Vertex *fv[3];

	for (int i = 24; i < 25/*layers-1*/; i++)
	{
		for (int j = 1; j < vertices-1; j++)
		{
			if (j == 0)continue;
			fVerts[0] = &M.vertices[i*vertices + j];
			fVerts[1] = &M.vertices[(i + 1)*vertices + j];
			fVerts[2] = &M.vertices[(i + 1)*vertices + j - 1];
			fVerts[3] = &M.vertices[i*vertices + j - 1];
			M.createNGon(fVerts, 4, false);
			/*	fv[0] = &M.vertices[i*colCnt + j];;
			fv[1] = &M.vertices[i*colCnt + j-1];;
			fv[2] = &M.vertices[(i+1)*colCnt + j];;
			M.createFace(fv, 3);


			fv[0] = &M.vertices[(i )*colCnt + j-1];
			fv[1] = &M.vertices[(i+1)*colCnt + j - 1];;
			fv[2] = &M.vertices[(i + 1)*colCnt + j];;
			M.createFace(fv, 3);*/
		}
	}


	//for (int i = 0; i < M.n_f; i++)M.faces[i].faceVertices();// generates face normals ;

	return metaMesh(M);
}

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------
////// ---------------------------------------------------- MODEL  ----------------------------------------------------

void setup()
{
	AGStack.clear();
	RG = *new rigidCube();

	//// graph stack
	displayStack.clear();
	for (int j = 0; j < 5; j++)
	{

		string file = "";
		file += "data/graph";
		file += "_";
		char s[20];
		itoa(j, s, 10);
		file += s;
		file += ".txt";
		importer imp = *new importer(file, 10000, 1.0);
		imp.readPts_p5();
		
		Graph G;
		G.reset();
		for (int i = 0; i < imp.nCnt; i++) G.createVertex( imp.nodes[i].pos + vec(0,0, 0.1 * float(j)) );
		for (int i = 0; i < imp.eCnt; i++) G.createEdge( G.vertices[ imp.edges[i].n0 ], G.vertices[ imp.edges[i].n1 ] );
		displayStack.push_back(G);
		//cout << G.n_v << endl;
	}


	//M = createMesh(displayStack);

	/// base graph

	{
		importer imp = *new importer("data/tree_pts.txt", 10000, 1.0);
		imp.readEdges();

		//---------
		base.reset();
		for (int i = 0; i < imp.nCnt; i++)base.createVertex(imp.nodes[i].pos);
		for (int i = 0; i < imp.eCnt; i++)base.createEdge(base.vertices[imp.edges[i].n0], base.vertices[imp.edges[i].n1]);

		// ------------ scale to fit 
		vec minV, maxV;
		base.boundingbox(minV, maxV);

		Matrix4 trans;
		double preferedDiag = 50;
		trans.scale(preferedDiag / (minV.distanceTo(maxV)));
		trans.translate((minV + maxV) * 0.5);
		for (int i = 0; i <base.n_v; i++) base.positions[i] = trans * base.positions[i];

		cout << " ACUTAL DIAG " << minV.distanceTo(maxV) << endl;
		base.boundingbox(minV, maxV);

		trans.identity();
		trans.translate(vec(45, 0, 0) - (minV + maxV) * 0.5);
		for (int i = 0; i <base.n_v; i++) base.positions[i] = trans *base.positions[i];
		minV = minV * trans;
		maxV = maxV * trans;

		base.boundingbox(minV, maxV);
		minV -= (maxV - minV).normalise() * 2.5;
		maxV += (maxV - minV).normalise() * 2.5;

		//

		MM = MM.createFromPlane(minV, maxV, 100);
		MM.assignScalarsAsLineDistanceField(base, 0, 0.5);
		MM.getMinMaxOfScalarField(dMin, dMax);


	}


	// curve from rhino
	/*importer imp = *new importer("data/curve.txt", 10000, 1.0);
	imp.readPts_p5();

	{

		vec *pts = new vec[imp.nCnt];
		for (int i = 0; i < imp.nCnt; i++)pts[i] = imp.nodes[i].pos + vec(0, 0, float(0) * 0.25);

		AG = *new activeGraph();
		AG.constructFromPoints(pts, imp.nCnt);
		AG.fixEnds();
		AG.populateRigidBodies(0.1);
		AGStack.push_back(AG);

	}*/
	for (int j = displayStack.size() - 5; j < displayStack.size(); j++)
	{
		AG = *new activeGraph();
		AG.constructFromGraph(displayStack[j]);
		for (int i = 0; i < AG.n_v; i++)AG.positions[i] += vec(0, 0, 10);

		AG.fixEnds();
		AG.smoothVertices();
		AG.populateRigidBodies(0.1);
		AGStack.push_back(AG);
	}


	//////////////////////////////////////////////////////////////////////////

	B = *new ButtonGroup();
	B.addButton(&showPoints, "showPts");
	B.addButton(&showSpheres, "showSpheres");


	
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}

void update(int value)
{
	if (run)
		for (auto &ag : AGStack)
		{
			ag.smoothVertices();
			ag.populateRigidBodies(0.1);
		}
}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void disp()
{
	backGround(0.9);
	drawGrid(20.0);

	drawRectangle( vec(0, 0, 0), vec(5, 5, 0) );
}

void draw()
{

	backGround(0.9);
	drawGrid(20);

	{
		glColor3f(0, 0, 0);
		for (auto &g : displayStack)g.draw();

		glColor3f(1, 0, 0);
		base.draw();
	}

	{
		MM.glPtSize = 3.0;
		wireFrameOn();
		MM.display(true, true, false);
		wireFrameOff();
	}


	for (auto &ag : AGStack)
		showPoints ? ag.display(PCur, RES) : ag.display();
	//	
	//for (auto &ag : AGStack)
	//	for (auto &rc : ag.RCsOnCurve)rc.resetForces();
	glLineWidth(4);
	//AGStack[0].draw();
	glLineWidth(1);

	int2 id = strId;// int2(l, n);
	nbor_RCs.clear();
	getNBors(id, nbor_RCs, 0.5);

	resetRCForces(id);
	for (auto &i : nbor_RCs)resetRCForces(i);

	for (auto nbor : nbor_RCs)
	{
//		AGStack[id.l].RCsOnCurve[id.n].computeContactsAndForces(AGStack[nbor.l].RCsOnCurve[nbor.n], PCur, PNext, RES);

		AGStack[nbor.l].RCsOnCurve[nbor.n].draw(2, vec4(1, 0, 0, 1));
		if (showSpheres)AGStack[nbor.l].RCsOnCurve[nbor.n].drawGrid(PNext, RES*RES*RES);
	}

	AGStack[strId.l].RCsOnCurve[strId.n].draw(4, vec4(1, 1, 1, 1));
	if (showSpheres)AGStack[strId.l].RCsOnCurve[strId.n].drawGrid(PCur, RES*RES*RES);


	/////

	B.draw();
}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

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
		PCur = new vec[RES*RES*RES];
		PNext = new vec[RES*RES*RES];
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

		for (int l = 0; l < AGStack.size(); l++)
		{
			for (int n = 0; n < AGStack[0].RCsOnCurve.size(); n++)
			{
				int2 id = int2(l, n);
				nbor_RCs.clear();
				getNBors(id, nbor_RCs, 0.5);

		//		for (auto nbor : nbor_RCs)
//					AGStack[id.l].RCsOnCurve[id.n].computeContactsAndForces(AGStack[nbor.l].RCsOnCurve[nbor.n], PCur, PNext, RES);
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




#endif // _MAIN_
