
#ifdef _MAIN_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"

////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

metaMesh MM;
Graph G;
double dMin, dMax;
double threshold;
bool run = false;

////// --- GUI OBJECTS ----------------------------------------------------

SliderGroup S;
ButtonGroup B;
bool blendPrev, blend;

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

////// ---------------------------------------------------- MODEL  ----------------------------------------------------

void setup()
{


	importer imp = *new importer("data/tree_pts.txt", 10000, 1.0);
	imp.readEdges();


	//---------
	G.reset();
	for (int i = 0; i < imp.nCnt; i++)G.createVertex(imp.nodes[i].pos);
	for (int i = 0; i < imp.eCnt; i++)G.createEdge(G.vertices[imp.edges[i].n0], G.vertices[imp.edges[i].n1]);

	// ------------ scale to fit 
	vec minV, maxV;
	G.boundingbox(minV, maxV);

	Matrix4 trans;
	double preferedDiag = 50;
	trans.scale(preferedDiag / (minV.distanceTo(maxV)));
	trans.translate((minV + maxV) * 0.5);
	for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];

	cout << " ACUTAL DIAG " << minV.distanceTo(maxV) << endl;
	G.boundingbox(minV, maxV);

	trans.identity();
	trans.translate(vec(45, 0, 0) - (minV + maxV) * 0.5);
	for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];
	minV = minV * trans;
	maxV = maxV * trans;

	G.boundingbox(minV, maxV);
	minV -= (maxV - minV).normalise() * 2.5;
	maxV += (maxV - minV).normalise() * 2.5;

	//

	MM = MM.createFromPlane(minV, maxV, 100);
	MM.assignScalarsAsLineDistanceField(G, 0, 0.5);
	MM.getMinMaxOfScalarField(dMin, dMax);

	for (int i = 0; i < MM.n_v; i++)
		if (MM.scalars[i] > 2.0)
			cout << MM.scalars[i] << " " << i << endl;

	//////////////////////////////////////////////////////////////////////////

	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].minVal = dMin;
	S.sliders[0].maxVal = dMax;

	blend = blendPrev = true;
	B = *new ButtonGroup(vec(50, 350, 0));
	B.addButton(&blend, "blend");


}

void update(int value)
{



}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()

{

	backGround(0.75);
	drawGrid(20.0);


	S.draw();
	B.draw();

	G.draw();

	MM.glPtSize = 3.0;
	wireFrameOn();
		MM.display(true, true, false);
	wireFrameOff();



}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'R')setup();


}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		if (HUDSelectOn)MM.createIsoContourGraph(threshold);
		B.performSelection(x, y);

		if (blend != blendPrev)
		{
			MM.assignScalarsAsLineDistanceField(G, 0, 0.5, blend);
			MM.getMinMaxOfScalarField(dMin, dMax);
			MM.createIsoContourGraph(threshold);
			blendPrev = blend;
		}

	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
	if (HUDSelectOn)MM.createIsoContourGraph(threshold);
}




#endif // _MAIN_
