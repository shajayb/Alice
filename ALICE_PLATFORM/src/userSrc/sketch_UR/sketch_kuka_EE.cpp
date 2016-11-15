#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;


class EndEffector
{
public:

	Mesh M;
	Robot Kuka_tester;

	EndEffector()
	{
		MeshFactory fac;
		M = fac.createFromOBJ("data/EE.obj", 1, false, false);

		invertMeshToLocal();
	}

	void invertMeshToLocal()
	{
		double rot[6];
		for (int i = 0; i < 6; i++)rot[i] = 0; // Kuka_tester.rot[i];
		rot[2] -= 90;
		Kuka_tester.ForwardKineMatics(rot);

		Matrix4 transformMatrix = Kuka_tester.Bars_to_home_matrices[5];
		transformMatrix.invert();

		for (int i = 0; i < M.n_v; i++)M.positions[i] = transformMatrix * M.positions[i];
	}

	void draw()
	{
		//Nachi_tester.draw();
		M.draw();
	}
};


// derive your class from a base / existing class 
class pathImporter:public importer
{

#define maxPts 9999 // max nachi controller limit
public:

	////////////////////////////////////////////////////////////////////////// CLASS VARIABLES 
	// path related class variables i.e arrays 

	vec path[maxPts][4]; // array to store TOOL locations ( from input text file ) or other method
	double rotations[maxPts][6];// array for joint rotations at each point along path 
	bool *reachable; // array to booleans
	int actualPathLength;
	vec min, max; // to store min - max of path bbox

	//////////////////////////////////////////////////////////////////////////
	Matrix4 TOOL, EE;

	// point related class variables 

	vec tcp, tcp_x, tcp_y, tcp_z;
	int currentPointId = 0;

	//
	Robot Kuka_tester;
	EndEffector E;

	////////////////////////////////////////////////////////////////////////// CLASS METHODS 

	pathImporter()
	{
		currentPointId = 0;
		
		// ------- set rotations to match exported links configuration of robot
		double rot[6];
		for (int i = 0; i < 6; i++)rot[i] = 0; // Kuka_tester.rot[i];
		rot[2] -= 90;
		vec pt = Kuka_tester.ForwardKineMatics(rot);
		//// ------- import link meshes
		Kuka_tester.addMeshes();
		
	}

	////////////////////////////////////////////////////////////////////////// UTILITY METHODS

	void assignDefaultFrame()
	{
		tcp = vec(0, 0, 0);
		tcp_x = vec(1, 0, 0);
		tcp_y = vec(0, 1, 0);
		tcp_z = vec(0, 0, -1);
	}

	vec extractVecFromStringArray( int id, vector<string> &content )
	{
		return vec(atof(content[id].c_str()), atof(content[id + 1].c_str()), atof(content[id+2].c_str()));
	}

	void readPath( string fileToRead = "data/path.txt" , string delimiter = "," )
	{
		
		////////////////////////////////////////////////////////////////////////// read file
		
		std::fstream fs(fileToRead.c_str(), ios::in);
		

		if (fs.fail())
		{
			cout << " error in file reading " << fileToRead << endl;
			return;
		}

		actualPathLength = 0;
		while (!fs.eof() && actualPathLength < maxPts)
		{
			char str[2000];
			fs.getline(str, 2000);
			vector<string> content = splitString(str, ",");

			assignDefaultFrame();

			if (content.size() >= 3)tcp = extractVecFromStringArray(0, content);
			if (content.size() >= 6)tcp_x = extractVecFromStringArray(3, content).normalise();
			if (content.size() >= 9)tcp_y = extractVecFromStringArray(6, content).normalise();
			if (content.size() >= 12)tcp_z = extractVecFromStringArray(9, content).normalise();

			path[actualPathLength][0] = tcp * 1 ; ///was 2 from sb
			path[actualPathLength][1] = tcp_x * 1 ; /// was -1 from sb
			path[actualPathLength][2] = tcp_y;
			path[actualPathLength][3] = tcp_z;
			actualPathLength++;


		}

		fs.close();

		reachable = new bool[actualPathLength];

		////////////////////////////////////////////////////////////////////////// bounding box of file

		getBoundingBox();

		checkReach();


	}


	void getBoundingBox()
	{
		min = vec(pow(10, 10), pow(10, 10), pow(10, 10));
		max = min * -1;

		for (int i = 0; i < actualPathLength; i++)
		{
			tcp = path[i][0];
			max.x = MAX(tcp.x, max.x);
			max.y = MAX(tcp.y, max.y);
			max.z = MAX(tcp.z, max.z);

			min.x = MIN(tcp.x, min.x);
			min.y = MIN(tcp.y, min.y);
			min.z = MIN(tcp.z, min.z);
		}
	}


	void getToolLocation(int id, Matrix4 &TOOL)
	{
		TOOL.setColumn(0, path[id][1]); // tcp_x
		TOOL.setColumn(1, path[id][2]); //tcp_y
		TOOL.setColumn(2, path[id][3]); // tcp_z
		TOOL.setColumn(3, path[id][0]); // tcp
	}

	Matrix4 getToolLocation(int id)
	{
		Matrix4 _TOOL;
		getToolLocation(id, _TOOL);
		return _TOOL;
	}

	void goToNextPoint()
	{
		
		EE = TOOL = getToolLocation(currentPointId);

		if (E.M.n_v != 0 )
		{

			for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];

			vec offsetpt = EE.getColumn(3);// +(EE.getColumn(2).normalise() * E.M.positions[3].distanceTo(E.M.positions[2]));
			TOOL.setColumn(3,offsetpt); 

			EE.invert();
			for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];
		}

		Kuka_tester.inverseKinematics_analytical(TOOL, false);
		vec pt = Kuka_tester.ForwardKineMatics(Kuka_tester.rot);

		currentPointId++;
		if (currentPointId >= actualPathLength)currentPointId = 0;

	}

	////////////////////////////////////////////////////////////////////////// COMPUTE METHODS

	void checkReach()
	{
		getBoundingBox();

		//----- print global warnings if any

		cout << " ------------------------------ START - PATH RELATED  WARNINGS ------------------------------ " << endl;

		if (min.mag() >= 70 || max.mag() >= 70) cout << " some or all points out of range" << endl;
		if (min.mag() <= 20 || max.mag() <= 20) cout << " some or all points are too close to robot " << endl;

		cout << " ------------------------------  END - PATH RELATED  WARNINGS ------------------------------ " << endl;

		//----- print point specific warnings if any

		cout << " ------------------------------ START - POINT-SPECIFIC   WARNINGS ------------------------------ " << endl;
		checkPathForReachability();
		cout << " ------------------------------ END - POINT-SPECIFIC   WARNINGS ------------------------------ " << endl;
	}

	void checkPathForReachability()
	{
		Matrix4 TOOL,EE;
		vec pt;
		for (int i = 0; i < actualPathLength-1; i++)
		{
			// get TOOL information at current point i
			EE = TOOL = getToolLocation(i);

			if (E.M.n_v != 0)
			{

				for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];

				//vec offsetpt = EE.getColumn(3) - (EE.getColumn(2) * E.M.positions[6].distanceTo(E.M.positions[4]));
				vec offsetpt = EE.getColumn(3);
				TOOL.setColumn(3, offsetpt);

				EE.invert();
				for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];
			}

			Kuka_tester.inverseKinematics_analytical(TOOL, false);
			pt = Kuka_tester.ForwardKineMatics(Kuka_tester.rot);

			// test if current TOOL location is reachable
			reachable[i] = true;
			if (pt.distanceTo(TOOL.getColumn(3)) > 1e-04)
			{
				printf(" point id %i is unreachable \n", i);
				reachable[i] = false;
			}

			// test if current TOOL orientation axes are ortho-normal / perpendicular to each other
			if (TOOL.getColumn(0)*TOOL.getColumn(1) >1e-04)printf(" point id %i axis x & y are not ortho-normal \n", i);
			if (TOOL.getColumn(1)*TOOL.getColumn(2) >1e-04)printf(" point id %i axis y & z are not ortho-normal \n", i);
			if (TOOL.getColumn(2)*TOOL.getColumn(0) >1e-04)printf(" point id %i axis z & x are not ortho-normal \n", i);

			
	


			// store corresponding rotations at each point along path
			// for later use such as gcode export & graph-generation etc.

			
			for (int n = 0; n < 6; n++)rotations[i][n] = Kuka_tester.rot[n];
			
		}

	}

	//"C:/FD_ONDESK/NRA2011/WORK/PROGRAM/MZ07-01-A.083"
	void exportGCode( string fileToWrite = "data/out.src" )
	{
		//----- check & compute all necessary values
	//	checkReach();
		
		cout << "--------------------------- EXPORT GCODE ----------------- " << endl;
		cout << "Ensure you have inspected robot reach at all points previously " << endl;
		cout << "un-reachable points revert to previous reach-able points" << endl;

		//----- instance ofstream for file IO
		ofstream myfile_write;
		char gcode[600];
		
		//- open file
		myfile_write.open(fileToWrite.c_str(), ios::out);
		if (myfile_write.fail())cout << " error in opening file  " << fileToWrite << endl;

		float e_x, e_y, e_z, r, p, y;
		//- iterate through path
		for (int i = 0; i < actualPathLength - 1; i++)
		{
			// get corresponding joint rotations
				e_x = rotations[i][0];
				e_y = rotations[i][1];
				e_z = rotations[i][2];
				  r = rotations[i][3];
				  p = rotations[i][4];
				  y = rotations[i][5];

			//format as per kuka language
			/*sprintf_s(gcode, "MOVEX A=1,AC=0,SM=0,M1X,L,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),S=1200.0,H=2,MS, CONF=0001", e_x * 10, e_y * 10, e_z * 10, r+90, p, y);*/
			//sprintf_s(gcode, "MOVEX A=8P,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),T=1.0,H=3,MS, CONF=0001", e_x, e_y, e_z, r, p, y);
			//perfect for cardiff robot!!
			sprintf_s(gcode, "PTP {AXIS: A1 %1.8f,A2 %1.8f,A3 %1.8f,A4 %1.8f,A5 %1.8f,A6 %1.8f}C_PTP ", e_x * -1, e_y, (e_z + 90), r *-1, (p - 360.0), y * -1);
			// output string to open file
			myfile_write << gcode << endl;
		}

		//-close file
		myfile_write.close();
		cout << "--------------------------- EXPORT GCODE COMPLETE ----------------- " << endl;
	}

	////////////////////////////////////////////////////////////////////////// DISPLAY METHODS

	void draw( bool wireFrame = true , bool showSphere = false )
	{
		// ------------------- TCP locations

		glPointSize(3);
		for (int i = 0; i < actualPathLength; i++)
		{
			reachable[i] ? glColor3f(0, 0, 1) : glColor3f(1, 0, 0);
			drawPoint(path[i][0]);
		}
		glPointSize(1);

		// ------------------- TCP orientation axes

		for (int i = 0; i < actualPathLength; i++)
		{
			glColor3f(1, 0, 0); drawLine(path[i][0], path[i][0] + path[i][1]);
			glColor3f(0, 1, 0); drawLine(path[i][0], path[i][0] + path[i][2]);
			glColor3f(0, 0, 1); drawLine(path[i][0], path[i][0] + path[i][3]);
		}


		//////////////////////////////////////////////////////////////////////////
		// get TOOL information at current point i
		Matrix4 EE = Kuka_tester.Bars_to_home_matrices[5];
		for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];

		wireFrameOn();
			E.draw();
		wireFrameOff();
		EE.invert();
		for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];
		

		//////////////////////////////////////////////////////////////////////////
		// ------------------- draw bounding box ;

		wireFrameOn();
			drawCube(min, max);
		wireFrameOff();

		// ------------------- draw Robot ;

		if (!wireFrame)Kuka_tester.draw(false, false); // updates AO render points ;
		Kuka_tester.draw(true, false);

		
	}

	void drawGraphs( float z, int jointNum)
	{
		float min, max;
		min = 1e10;
		max = min * -1;

		for (int i = 0; i < actualPathLength - 1; i++)
		{
			min = MIN(rotations[i][jointNum], min);
			max = MAX(rotations[i][jointNum], max);
		}

		glLineWidth(1);
		setup2d();
		
			for (int i = 0; i < actualPathLength - 1; i++)
			{
				// get corresponding joint rotations
				float r = rotations[i][jointNum];
				float ht = ofMap(r, -360, 360, -1, 1) * 25;
				vec a = vec(float(i) * 2, winH - z, 0);
				drawLine( a , a + vec(0, ht, 0) );
			}
		
			vec a = vec(float(currentPointId) * 2, winH - z, 0);
			glColor3f(1, 0, 0);
			drawLine(a, a + vec(0, -30, 0));
			char s[200];
			sprintf(s, "%f", rotations[currentPointId][jointNum] );
			drawString(s, a + vec(0, -35, 0));

		restore3d();

	}


};


pathImporter path;
int currentPointId;
SliderGroup S;
ButtonGroup B;

char s[200];
char t[200];
char jts[400];
double xAngDiff, yAngDiff, zAngDiff;
bool drawWire = true;

void setup()
{

	path.readPath();

	// ---------------------------------------------

	S = *new SliderGroup();
	S.addSlider(&path.Kuka_tester.rot[0], "J0");
	S.addSlider(&path.Kuka_tester.rot[1], "J1");
	S.addSlider(&path.Kuka_tester.rot[2], "J2");
	S.addSlider(&path.Kuka_tester.rot[3], "J3");
	S.addSlider(&path.Kuka_tester.rot[4], "J4");
	S.addSlider(&path.Kuka_tester.rot[5], "J5");

	S.sliders[0].attachToVariable(&path.Kuka_tester.rot[0], -360, 360);
	S.sliders[1].attachToVariable(&path.Kuka_tester.rot[1], -360, 360);
	S.sliders[2].attachToVariable(&path.Kuka_tester.rot[2], -360, 360);
	S.sliders[3].attachToVariable(&path.Kuka_tester.rot[3], -360, 360);
	S.sliders[4].attachToVariable(&path.Kuka_tester.rot[4], -360, 360);
	S.sliders[5].attachToVariable(&path.Kuka_tester.rot[5], -360, 360);

	B = *new ButtonGroup( vec(50,550,0) ) ;
	B.addButton(&drawWire, "drawWire");

	// ---------------------------------------------
}

void update(int value)
{
	path.Kuka_tester.ForwardKineMatics(path.Kuka_tester.rot);
	//path.Kuka_tester.ForwardKineMatics(path.Kuka_tester.rot);
}



void draw()
{

	backGround(0.75);
	drawGrid(200.0);


	//// ------------------------ draw Sliders

	glColor3f(0, 0, 0);
	S.draw();
	B.draw();
	
	//// ------------------------ draw the path points / Tool orientations 

	path.draw(drawWire);
	path.drawGraphs(50,5);
	//E.draw();
	//M.draw();

	//// ------------------------ draw the variables


	sprintf_s(s,"current point id : %i", path.currentPointId);
	sprintf_s(t,"total points in path : %i", path.actualPathLength - 1);
	int cid = path.currentPointId;

	if (cid < path.actualPathLength - 1 && cid >= 0)
		sprintf_s(jts, "joint angles:%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f", path.rotations[cid][0], path.rotations[cid][1], path.rotations[cid][2],path.rotations[cid][3], path.rotations[cid][4], path.rotations[cid][5]);

	glColor3f(0, 0, 0);
	setup2d();

		drawString(s, winW * 0.5, winH - 50);
		drawString(t, winW * 0.5, winH - 75);
		drawString(jts, winW * 0.5, winH - 100);

		sprintf_s(s, "xAngDiff %1.2f, yAngDiff %1.2f, zAngDiff %1.2f", xAngDiff,yAngDiff,zAngDiff );
			drawString(s, winW * 0.5, winH - 125);
		sprintf_s(s, "n - go to next point ;q - check path for reachability;w - export g code to data_out.src;r - reset");
			 drawString(s, winW * 0.5, winH - 150);
			

	restore3d();


}

void keyPress(unsigned char k, int xm, int ym)
{
	
	
	if (k == 'n')
	{
		path.goToNextPoint();

		vec ax, ay, az;
		ax = path.Kuka_tester.Bars_to_home_matrices[5].getColumn(0);
		ay = path.Kuka_tester.Bars_to_home_matrices[5].getColumn(1);
		az = path.Kuka_tester.Bars_to_home_matrices[5].getColumn(2);

		xAngDiff = ax.angle(path.TOOL.getColumn(0));
		yAngDiff = ay.angle(path.TOOL.getColumn(1));
		zAngDiff = az.angle(path.TOOL.getColumn(2));

	}

	if (k == 'N')path.currentPointId -= 1;
	

	if (k == 'q')path.checkPathForReachability();
	if (k == 'w')path.exportGCode();
	if (k == 'r')setup();

	if (k == 'c')
	{
		//// rotation corrections...
		for (int i = 0; i < path.actualPathLength -1 ; i++)
		{
			//if (i > 0)
			//	if ((path.rotations[i][5] - path.rotations[i - 1][5]) > 180 ) path.rotations[i][5] -= 360;

			if (i > 0)
				if ( (path.rotations[i][5] - path.rotations[i - 1][5]) < -180 ) path.rotations[i][5] += 360;
		}
	}
}

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{
//	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
	}
}



