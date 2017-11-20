


#ifdef _MAIN_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graphStack.h"
#include "largeMesh.h"

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
bool showGraphStackMesh = false;
bool showMeshWire = false;

char s[200],text[200], text1[200], jts[400];

double lightscale = 3.0;
double background = 0.8;
bool flipNormals = false;
vec camPt;
////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

//largeMesh LM;
int fileNum = 0;
string printfile;
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
	//int i = 0;
	//{
	//	path.readPath("data/path.txt", ",", 1.15 + float(i) * 0.1);
	//	//path.actualPathLength--;
	//}
	//char s[200];
	//sprintf(s, "%i", fileNum);
	//printfile = "data/block_";
	//printfile += s;
	//printfile += ".obj";
	//path.readOBJ(printfile);
	//cout << "path read " << printfile.c_str() << endl;

	//cout << filePath[filePath.size() - 1] << endl;
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

	GS.threshold = 0;
	S.addSlider(&GS.threshold, "threshold");
	S.sliders[6].minVal = 0;
	S.sliders[6].maxVal = 1590;
	//S.sliders[6].minVal = -20;
	//S.sliders[6].maxVal = 0;
	S.addSlider(&lightscale, "lightscale");	
	S.sliders[7].maxVal = 20;
	S.addSlider(&background, "background");
	/////////////////////////////

	B = *new ButtonGroup(vec(50, 450, 0));
	B.addButton(&showRobot, "showRobot");
	B.addButton(&showGraphStackData, "showGraphData");
	B.addButton(&showGraphStackMesh, "showStackMesh");
	B.addButton(&showMeshWire, "showMeshWire");
	B.addButton(&flipNormals, "flipNormals");

	//////////////////////////////////////////////////////////////////////////

	//Mesh M;
	//MeshFactory fac;
	//if (inFile.length() > 0)printfile = inFile;
	//M = fac.createFromOBJ(printfile, 100.0, false);

	//{

	//		Matrix3x3 PCA_mat;
	//		vec mean, eigenValues, eigenvecs[3];
	//		PCA_mat.PCA(M.positions, M.n_v, mean, eigenValues, eigenvecs);
	//		M.boundingBox(minV, maxV);

	//		vec x = eigenvecs[0].normalise();
	//		x.z = 0;
	//		vec z = vec(0, 0, 1);
	//		vec y = x.cross(z).normalise();
	//		
	//		Matrix3 trans;
	//		trans.setColumn(0, x);
	//		trans.setColumn(1, y);
	//		trans.setColumn(2, z);
	//		trans.transpose();
	//		for (int i = 0; i < M.n_v; i++)
	//		{
	//			M.positions[i] -= (minV+maxV)*0.5;
	//			//M.positions[i] = trans * M.positions[i];
	//		}
	//}

	//{
	//	vec x, y, z, cen;
	//	cen = vec(73.9327, -2.2114, -17.4015);
	//	x = vec(1, 0, 0).normalise();
	//	z = vec(0, 0, -1);
	//	y = x.cross(z);

	//	Matrix4 fTrans;
	//	fTrans.identity();
	//	fTrans.setColumn(0, x.normalise());
	//	fTrans.setColumn(1, y.normalise());
	//	fTrans.setColumn(2, z.normalise());
	//	fTrans.setColumn(3, cen);


	//	for (int i = 0; i < M.n_v; i++)
	//		M.positions[i] = fTrans * M.positions[i];

	//	vec minV, maxV;
	//	M.boundingBox(minV, maxV);
	//	double diff = cen.z - minV.z;


	//	for (int i = 0; i < M.n_v; i++)
	//	M.positions[i].z += diff;
	//}


	//MM = *new metaMesh(M);;
	//MM.assignScalars("z");
	//MM.createIsoContourGraph(1.0);
	//MM.G.computeIslandsAsEdgeAndVertexList();
	//MM.convertContourToToroidalGraph();
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
		/*keyPress('p', 0, 0);
		for (int i = 0; i < 5; i++)keyPress(' ', 0, 0);*/
		keyPress('.', 0, 0);
	}
		


}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(background);
	drawGrid(20.0);

	glColor3f(1, 0, 0);
	MM.display(false, true, true);

	//vec A = vec(73.9327, -2.2114, -17.4);
	//wireFrameOn();
	//	drawPlane(vec(0, 0, 1), A, 15.0);
	//wireFrameOff();

	S.draw();
	B.draw();
	// ------------------------ draw the path points / Tool orientations 

	if (showRobot)
	{
		
		path.draw(false);
		for (auto &g : GS.PrintStack) g.draw();

		path.drawFrame(path.fTrans, 3);
	}
	else
			GS.draw(showGraphStackMesh, showMeshWire,showGraphStackData);

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
	AL_drawString(printfile.c_str(), winW * 0.5, winH - 125);
	
	

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

	drawVector(camPt, vec(wid, hts + 25, 0), "cam");

}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{

	///// GRAPH GENERTOR PROGRAM 
	if (k == 'i')setCamera(15, -40, 60, -2, 4);
	

	if (k == ' ')
	{
		//GS.smoothCurrentGraph();
		//GS.MM.G.redistribute_toroidal(0.2);
		GS.MM.G.smoothGraph(180);

		for (int i = 0; i < 10; i++)MM.G.smoothGraph(10);
	}
	
	if (k == 'c')
	{
		GS.convertContourToToroidalGraph();
		MM.convertContourToToroidalGraph();
	}

	if (k == 'n')
	{
		GS.MM.G.redistribute_toroidal(0.05);
		for (int i = 0; i < 10; i++)GS.MM.G.smoothGraph(90);

		MM.G.redistribute_toroidal(0.5);
		
	}
	if (k == '-')GS.reducePointsOnContourGraph(2);
	if (k == 'b')
	{
		GS.addCurrentContourGraphToPrintStack(0.2, 0.0);
		//
	}
	if (k == 'B')
	{
		for (int i = 0; i < 10; i++)
		{
			keyPress('b', 0, 0);
			
			for (int j = 0; j < 1; j++)GS.inflateCurrentGraph();
			for (int j = 0; j < 30; j++)keyPress(' ', 0, 0);
			for (int j = 0; j < 1; j++)GS.MM.G.inflateVertices();
		}

		GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}
	if (k == '<')GS.currentStackLayer--;
	if (k == 'O')
	{
		showGraphStackMesh = false; 
		GS.convertedToToroidal = false;
		GS.currentStackLayer = 0;
		GS.LM.n_v = GS.LM.n_f = 0;
	}
	if (k == 'L')GS.ConvertContourStackToPrintPath(path);

	if (k == 'Q')GS.writeCurrentGraph();
	if (k == 'W')
	{
		GS.writeStackToFile("data/PRINT.txt");
		GS.writeStackToObj("data/PRINT.obj");
		GS.LM.writeOBJ("data/largeMesh.obj");
	}

	if (k == 'e')
	{
		GS.readGraphAndCreateDataMesh("data/tree_pts.txt", 1.0);//circular_pts

	}
	if (k == 'U')run = !run;

	///// ROBOT PROGRAM / PATH CHECKING 

	if (k == '.')path.goToNextPoint();
	if (k == 'b')
	{
		path.currentPointId -= 2;;
		path.goToNextPoint();
	}
	if (k == 'N')path.currentPointId = 0;

	if (k == 'w')path.exportGCode_3dp();// path.exportGCode();


	if (k == 'h')
	{
		for (int i = 0; i < DOF; i++)path.Nachi_tester.rot[i] = 0.;
		path.Nachi_tester.rot[1] = 90.0;
		path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);
	}

	if (k == 'a')
	{
		MM.G.smoothGraph(10);
		MM.G.renumber(MM.G.positions[4]);
	}

	if (k == 'm')
	{
		
			cout << "path read " << inFile.c_str() << endl;
			vector<string>filePath = path.splitString(inFile, "\\");
			vector<string>fileNameParts = path.splitString(filePath[filePath.size()-1], ".");
			string fileName = fileNameParts[0];
			cout << fileName << endl;
			string exportFileName = "Robot2_";
			exportFileName += fileName;
			exportFileName += ".src";

			path.readOBJ(inFile);
			path.exportGCode_3dp(exportFileName);



	}

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		
		B.performSelection(x, y);

		S.performSelection(x, y, HUDSelectOn);

		if (HUDSelectOn & !GS.convertedToToroidal)
		{
			//GS.threshold = 1.5;
			GS.createIsoContourGraph(GS.threshold);

			{
				
				MM.createIsoContourGraph(GS.threshold);
				MM.G.computeIslandsAsEdgeAndVertexList();
				MM.convertContourToToroidalGraph();
			}
			cout << GS.MM.G.n_v << endl;

		}
	}

	if((GLUT_LEFT_BUTTON == b && GLUT_UP == state) || (GLUT_RIGHT_BUTTON == b && GLUT_UP == state))
	{
		int cur_msx = winW * 0.5;
		int cur_msy = winH * 0.5;
		camPt = screenToCamera(cur_msx, cur_msy, 0.2);

		GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
	if (HUDSelectOn & !GS.convertedToToroidal)
	{
		//GS.createIsoContourGraph(GS.threshold);
	
		{

			MM.createIsoContourGraph(GS.threshold);
			MM.G.computeIslandsAsEdgeAndVertexList();
			MM.convertContourToToroidalGraph();
		}

		//GS.LM.updateColorArray(lightscale, flipNormals, camPt);
	}

	bool dragging = (glutGetModifiers() == GLUT_ACTIVE_ALT) ? true : false;
	int cur_msx = winW * 0.5;
	int cur_msy = winH * 0.5;
	camPt = screenToCamera(cur_msx, cur_msy, 0.2);

	//if( dragging)GS.LM.updateColorArray(lightscale, flipNormals, camPt);

}




#endif // _MAIN_
