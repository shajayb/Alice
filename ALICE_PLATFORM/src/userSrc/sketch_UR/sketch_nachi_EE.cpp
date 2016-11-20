#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graphStack.h"




pathImporter path;
graphStack GS;
metaMesh MM;
Graph G;
Mesh M;

vec minV, maxV;
double iter;
int currentPointId;

SliderGroup S;
ButtonGroup B;
bool showRobot = true;


void setup()
{

	GS = *new graphStack();
	GS.readGraphAndCreateDataMesh("data/tree_pts.txt", 1.0);

	////

	path.readPath();

	//////////////////////////////////////////////////////////


	S = *new SliderGroup();
	S.addSlider(&path.Nachi_tester.rot[0], "J1");
	S.addSlider(&path.Nachi_tester.rot[1], "J2");
	S.addSlider(&path.Nachi_tester.rot[2], "J3");
	S.addSlider(&path.Nachi_tester.rot[3], "J4");
	S.addSlider(&path.Nachi_tester.rot[4], "J5");
	S.addSlider(&path.Nachi_tester.rot[5], "J6");

	S.sliders[0].attachToVariable(&path.Nachi_tester.rot[0], -170, 170);
	S.sliders[1].attachToVariable(&path.Nachi_tester.rot[1], -170, 170);
	S.sliders[2].attachToVariable(&path.Nachi_tester.rot[2], -170, 170);
	S.sliders[3].attachToVariable(&path.Nachi_tester.rot[3], -170, 170);
	S.sliders[4].attachToVariable(&path.Nachi_tester.rot[4], -170, 170);
	S.sliders[5].attachToVariable(&path.Nachi_tester.rot[5], -170, 170);

	S.addSlider(&GS.threshold, "threshold");
	S.sliders[6].minVal = 0;
	S.sliders[6].maxVal = 400;

	/////////////////////////////

	B = *new ButtonGroup( vec(50, 450, 0) );
	B.addButton(&showRobot, "showRobot");

}

bool run = false;
void update(int value)
{
	path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);

	if (run)
		for (int i = 0; i < 10; i++)GS.smoothCurrentGraph();
			

}

char s[200];
char t[200];
char jts[400];

void draw()
{

	backGround(0.75);
	drawGrid(200.0);


	S.draw();
	B.draw();
	//// ------------------------ draw the path points / Tool orientations 

	if(showRobot)path.draw(false);


	//// graph 

	//GS.draw();


	//////////////////////////////////////////////////////////

	sprintf_s(s, " current point id : %i", path.currentPointId);
	sprintf_s(t, " total points in path : %i", path.actualPathLength - 1);
	int cid = path.currentPointId;

	if (cid < path.actualPathLength - 1 && cid >= 0)
		sprintf_s(jts, "%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f", path.rotations[cid][0], path.rotations[cid][1], path.rotations[cid][2],
			path.rotations[cid][3], path.rotations[cid][4], path.rotations[cid][5]);

	setup2d();

	drawString(s, winW * 0.5, winH - 50);
	drawString(t, winW * 0.5, winH - 75);
	drawString(jts, winW * 0.5, winH - 100);

	restore3d();



}

void keyPress(unsigned char k, int xm, int ym)
{

	if (k == 'n')path.goToNextPoint();
	if (k == 'b')
	{
		path.currentPointId -= 2;;
		path.goToNextPoint();
	}
	if (k == 'N')path.currentPointId = 0;
	if (k == 'q')path.checkPathForReachability();
	if (k == 'w')path.exportGCode();
	if (k == 'r')setup();

	if (k == 'h')
	{
		for (int i = 0; i < DOF; i++)path.Nachi_tester.rot[i] = 0.;
		//path.Nachi_tester.rot[0] = path.Nachi_tester.rot[2] = 0.0;// path.Nachi_tester.rot[3] = 0.0;
		path.Nachi_tester.rot[1] = 90.0;
		//path.Nachi_tester.rot[2] = -90;
		path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);
	}
	/////


	if (k == ' ')GS.smoothCurrentGraph();
	if (k == 'w')GS.writeCurrentGraph();
	if (k == 'R')run = !run;
}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		B.performSelection(x, y);
		if (HUDSelectOn)
		{
			GS.createIsoContourGraph(GS.threshold);
			cout << GS.MM.G.n_v << endl;
		}
	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
	if (HUDSelectOn)GS.createIsoContourGraph(GS.threshold);
}



