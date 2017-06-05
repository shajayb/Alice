
#define _MAIN_
#define _ALG_LIB_





#ifdef _MAIN_
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graphStack.h"


////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------



metaMesh MM;
graphStack GS;
pathImporter path;

Graph G;
Mesh M;

vec minV, maxV;
double iter;
int currentPointId;

bool run = false;
int rCnt = 0;
////// --- GUI OBJECTS ----------------------------------------------------

//SliderGroup S;
//ButtonGroup B;
bool showRobot = false;
bool showGraphStackData = false;

char s[200],text[200], text1[200], jts[400];

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

////// ---------------------------------------------------- MODEL  ----------------------------------------------------

void setup()
{

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	//////////////////////////////////////////////////////////////////////////

	GS = *new graphStack();
	GS.readGraphAndCreateDataMesh("data/tree_pts.txt", 1.0);//circular_pts

	//////////////////////////////////////////////////////////



	//for (int i = 0; i < 50; i++)
	int i = 0;
	{
		path.readPath("data/path.txt", ",", 1.15 + float(i) * 0.1);
		//path.actualPathLength--;
	}

	//////////////////////////////////////////////////////////


	S = *new SliderGroup();
	S.addSlider(&path.Nachi_tester.rot[0], "J1");//etc
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
	S.sliders[6].maxVal = 5;

	/////////////////////////////

	B = *new ButtonGroup(vec(50, 450, 0));
	B.addButton(&showRobot, "showRobot");
	B.addButton(&showGraphStackData, "showGraphData");

}

void update(int value)
{
	path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);

	if (run)
	{
		/*for (int i = 0; i < 10; i++)GS.smoothCurrentGraph();

		rCnt++;
		if (rCnt % 3 == 0)GS.writeCurrentGraph();

		if (rCnt == 200 * 2 )run = !run;*/
		keyPress('p', 0, 0);
		for (int i = 0; i < 5; i++)keyPress(' ', 0, 0);
	}
		


}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(1.0);
	drawGrid(20.0);


	//S.draw();
	//B.draw();
	//// ------------------------ draw the path points / Tool orientations 

	if (showRobot)
	{
		path.draw(false);
		for (auto &g : GS.PrintStack) g.draw();
	}
	else
			GS.draw(showGraphStackData);

	//////////////////////////////////////////////////////////

	sprintf_s(s, " current point id : %i", path.currentPointId);
	sprintf_s(text, " total points in path : %i", path.actualPathLength - 1);
	
	int cid = path.currentPointId;

	if (cid < path.actualPathLength - 1 && cid >= 0)
		sprintf_s(jts, "%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f", path.rotations[cid][0], path.rotations[cid][1], path.rotations[cid][2],
			path.rotations[cid][3], path.rotations[cid][4], path.rotations[cid][5]);

	glColor3f(0, 0, 0);
	setup2d();

	AL_drawString(s, winW * 0.5, winH - 50);
	AL_drawString(text, winW * 0.5, winH - 75);
	AL_drawString(jts, winW * 0.5, winH - 100);


	int hts = 50;
	int wid = winW * 0.75;

	AL_drawString("  SPC :GS.smoothCurrentGraph()", wid, hts); hts += 25;
	AL_drawString("  R :smoothIteration toggle;", wid, hts); hts += 25;
	AL_drawString("  c :GS.convertContourToCyclicGraph();", wid, hts); hts += 25;;
	AL_drawString("  - :GS.reducePointsOnContourGraph(2);", wid, hts); hts += 25;
	AL_drawString("  p :GS.addCurrentContourGraphToPrintStack(0.05, 0.4);", wid, hts); hts += 25;
	AL_drawString("  P :GS.currentStackLayer--;", wid, hts); hts += 25;
	AL_drawString("  O :GS.currentStackLayer = 0", wid, hts); hts += 25;
	AL_drawString("  L :GS.ConvertContourStackToPrintPath(path);", wid, hts); hts += 25;
	AL_drawString("  Q : GS.writeCurrentGraph(); ", wid, hts); hts += 25;
	AL_drawString("  R : run = !run ", wid, hts); hts += 25;
	hts += 25;
	AL_drawString(" n : path.goToNextPoint();", wid, hts); hts += 25;
	AL_drawString(" b : path.goToPrev();", wid, hts); hts += 25;
	AL_drawString(" N : path.currentId = 0;", wid, hts); hts += 25;
	AL_drawString(" w : path.exportGCode();", wid, hts); hts += 25;
	AL_drawString(" r : setup();", wid, hts); hts += 25;
	AL_drawString(" h : path.home();", wid, hts); hts += 25;

	restore3d();



}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{

	///// GRAPH GENERTOR PROGRAM 

	if (k == ' ')GS.smoothCurrentGraph();
	if (k == 'c')GS.convertContourToToroidalGraph();
	if (k == '-')GS.reducePointsOnContourGraph(2);
	if (k == 'p')GS.addCurrentContourGraphToPrintStack(0.2, 1.75);
	if (k == 'P')
	{
		for (int i = 0; i < 1; i++)
		{
			keyPress('p', 0, 0);
			for (int j = 0; j < 1; j++)keyPress(' ', 0, 0);
		}
	}
	if (k == 'P')GS.currentStackLayer--;
	if (k == 'O')GS.currentStackLayer = 0;
	if (k == 'L')GS.ConvertContourStackToPrintPath(path);

	if (k == 'Q')GS.writeCurrentGraph();
	if (k == 'W')
	{
		GS.writeStackToFile("data/PRINT.txt");
	//	GS.writeStackToObj("data/PRINT.obj");
	}

	if (k == 'U')run = !run;

	///// ROBOT PROGRAM / PATH CHECKING 

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
		path.Nachi_tester.rot[1] = 90.0;
		path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);
	}


}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		B.performSelection(x, y);
		if (HUDSelectOn)
		{
			//GS.threshold = 1.5;
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




#endif // _MAIN_
