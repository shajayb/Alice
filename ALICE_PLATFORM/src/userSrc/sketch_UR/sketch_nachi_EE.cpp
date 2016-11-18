#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;


class EndEffector
{
public:

	Mesh M;
	Robot_Symmetric Nachi_tester;

	EndEffector()
	{
		MeshFactory fac;
		M = fac.createFromOBJ("data/EE.obj", 1, false, false);
		invertMeshToLocal();
	}

	void invertMeshToLocal()
	{
		Nachi_tester.ForwardKineMatics(Nachi_tester.rot);

		Matrix4 transformMatrix = Nachi_tester.Bars_to_world_matrices[5];
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
class pathImporter :public importer
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

				  // point related class variables 

	vec tcp, tcp_x, tcp_y, tcp_z;
	int currentPointId = 0;

	//
	Robot_Symmetric Nachi_tester;
	EndEffector E;

	////////////////////////////////////////////////////////////////////////// CLASS METHODS 

	pathImporter()
	{
		currentPointId = 0;

		//double DH[6][4];
		////DH[0][0] = 034.5; DH[0][1] = 005.0;    DH[0][2] = 90.0; DH[0][3] = 0.0;
		////DH[1][0] = 000.0; DH[1][1] = 033.0;    DH[1][2] = 00.0; DH[1][3] = 90.0;
		////DH[2][0] = 000.0; DH[2][1] = 04.5;  DH[2][2] = 90.0; DH[2][3] = 0.0;
		////DH[3][0] = 034.0;   DH[3][1] = .000;  DH[3][2] = -90.0; DH[3][3] = 0.0;
		////DH[4][0] = 0.000; DH[4][1] = .000;  DH[4][2] = 90.0; DH[4][3] = 0.0;
		////DH[5][0] = 007.3; DH[5][1] = .000;  DH[5][2] = 00.0; DH[5][3] = 0.0;

		//DH[0][0] = 33.0;  DH[0][1] = 8.8;   DH[0][2] = 90.0;  DH[0][3] = 0;
		//DH[1][0] = 000.0; DH[1][1] = 31.0;  DH[1][2] = 0.;    DH[1][3] = 90;
		//DH[2][0] = 000.0; DH[2][1] = 4.0;   DH[2][2] = 90.0;  DH[2][3] = 0;
		//DH[3][0] = 30.5;  DH[3][1] = .000;  DH[3][2] = -90.0; DH[3][3] = 0;
		//DH[4][0] = 0.000; DH[4][1] = .000;  DH[4][2] = 90.0;  DH[4][3] = 0;
		//DH[5][0] = 8.65;  DH[5][1] = .000;  DH[5][2] = 00.0;  DH[5][3] = 0;


		////Nachi_tester.Bars[0] = Link(034.5, 05.0, 90., 0.); //1 l-axis = blue -> next axis = red, bcos alpha = 90
		////Nachi_tester.Bars[1] = Link(0.000, 33.0, 0., 90.);//2 l-axis = red ->  next axis = red bcos alpha = 0
		////Nachi_tester.Bars[2] = Link(0.000, 04.5, 90., 0.);//3 l-axis = red ->  next axis = blue bcos alpha = 90
		////Nachi_tester.Bars[3] = Link(034.0, .000, -90., 0.);//4 l-axis = blue
		////Nachi_tester.Bars[4] = Link(0.000, .000, 90., 00.);//5 l-axis = blue
		////Nachi_tester.Bars[5] = Link(007.3, .000, 00., 00.);//6 l-axis = blue

		//Nachi_tester.Bars[0] = Link(DH[0][0], DH[0][1], DH[0][2], DH[0][3]);
		//Nachi_tester.Bars[1] = Link(DH[1][0], DH[1][1], DH[1][2], DH[1][3]);
		//Nachi_tester.Bars[2] = Link(DH[2][0], DH[2][1], DH[2][2], DH[2][3]);
		//Nachi_tester.Bars[3] = Link(DH[3][0], DH[3][1], DH[3][2], DH[3][3]);
		//Nachi_tester.Bars[4] = Link(DH[4][0], DH[4][1], DH[4][2], DH[4][3]);
		//Nachi_tester.Bars[5] = Link(DH[5][0], DH[5][1], DH[5][2], DH[5][3]);

		//for (int i = 0; i < DOF; i++)Nachi_tester.rot[i] = 0.;
		//Nachi_tester.rot[1] = 90;
		////Nachi_tester.rot[1] = -90;

		//Nachi_tester.constructRobot(DH);
		//vec pt = Nachi_tester.ForwardKineMatics(Nachi_tester.rot);
		Nachi_tester.addMeshes();



	}

	////////////////////////////////////////////////////////////////////////// UTILITY METHODS

	void assignDefaultFrame()
	{
		tcp = vec(0, 0, 0);
		tcp_x = vec(-1, 0, 0);
		tcp_y = vec(0, 1, 0);
		tcp_z = vec(0, 0, -1);
	}

	vec extractVecFromStringArray(int id, vector<string> &content)
	{
		return vec(atof(content[id].c_str()), atof(content[id + 1].c_str()), atof(content[id + 2].c_str()));
	}

	void readPath(string fileToRead = "data/path.txt", string delimiter = ",")
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

			if (content.size() >= 3)tcp = extractVecFromStringArray(0, content) * 1.0;
			if (content.size() >= 6)tcp_x = extractVecFromStringArray(3, content).normalise() * 1;
			if (content.size() >= 9)tcp_y = extractVecFromStringArray(6, content).normalise() * 1;
			if (content.size() >= 12)tcp_z = extractVecFromStringArray(9, content).normalise() * 1;


			path[actualPathLength][0] = tcp;
			path[actualPathLength][1] = tcp_z * -1;
			path[actualPathLength][2] = tcp_y * 1;
			path[actualPathLength][3] = tcp_x * 1;
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
		Matrix4 TOOL, EE;
		EE = TOOL = getToolLocation(currentPointId);


		double rot_prev[6];
		for (int i = 0; i < 6; i++)rot_prev[i] = Nachi_tester.rot[i];

		Nachi_tester.inverseKinematics_analytical(TOOL, false);


		vec pt = Nachi_tester.ForwardKineMatics(Nachi_tester.rot);

		cout << rot_prev[3] - Nachi_tester.rot[3] << " J3 diff " << endl;
		cout << rot_prev[3] << " J3_prev " << endl;


		angleCorrection(rot_prev);





		//Nachi_tester.rot[0] = ofClamp(Nachi_tester.rot[0], -170, 170);
		//Nachi_tester.rot[1] = ofClamp(Nachi_tester.rot[1], -65, 150);
		//Nachi_tester.rot[2] = ofClamp(Nachi_tester.rot[2], -70, 90);
		//Nachi_tester.rot[3] = ofClamp(Nachi_tester.rot[3], -150, 150);
		//Nachi_tester.rot[4] = ofClamp(Nachi_tester.rot[4], -109, 109);
		//Nachi_tester.rot[5] = ofClamp(Nachi_tester.rot[5], -360, 360);

		cout << " -- current point -- " << currentPointId << endl;
		vec ax, ay, az;
		ax = Nachi_tester.Bars_to_world_matrices[5].getColumn(0);
		ay = Nachi_tester.Bars_to_world_matrices[5].getColumn(1);
		az = Nachi_tester.Bars_to_world_matrices[5].getColumn(2);
		cout << pt.mag() << " lenghth " << endl;

		cout << ax.angle(TOOL.getColumn(0)) << endl;
		cout << ay.angle(TOOL.getColumn(1)) << endl;
		cout << az.angle(TOOL.getColumn(2)) << endl;
		vec tcp_ret = Nachi_tester.Bars_to_world_matrices[5].getColumn(3);
		cout << Nachi_tester.joints[5].distanceTo(TOOL.getColumn(3)) << " in-out - tcp diff " << endl;


		currentPointId++;
		if (currentPointId >= actualPathLength - 1)currentPointId = 0;

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
		Matrix4 TOOL, EE;
		vec pt;
		for (int i = 0; i < actualPathLength - 1; i++)
		{
			// get TOOL information at current point i
			EE = TOOL = getToolLocation(i);

			double rot_prev[6];
			for (int i = 0; i < 6; i++)rot_prev[i] = Nachi_tester.rot[i];

			Nachi_tester.inverseKinematics_analytical(TOOL, false);

			//-170	170
			//	65 - 150
			//	- 70.001	90
			//	190.001 - 190.001
			//	- 109.106	109.106
			////	360.001 - 360.001

			angleCorrection(rot_prev);


			//Nachi_tester.rot[0] = ofClamp(Nachi_tester.rot[0], -170, 170);
			//Nachi_tester.rot[1] = ofClamp(Nachi_tester.rot[1], -65, 150);
			//Nachi_tester.rot[2] = ofClamp(Nachi_tester.rot[2], -70, 90);
			//Nachi_tester.rot[3] = ofClamp(Nachi_tester.rot[3], -170, 170);
			//Nachi_tester.rot[4] = ofClamp(Nachi_tester.rot[4], -109, 109);
			//Nachi_tester.rot[5] = ofClamp(Nachi_tester.rot[5], -360, 360);

			pt = Nachi_tester.ForwardKineMatics(Nachi_tester.rot);



			// test if current TOOL location is reachable
			reachable[i] = true;
			if (pt.distanceTo(TOOL.getColumn(3)) > 1e-04)
			{
				printf(" point id %i is unreachable \n", i);
				reachable[i] = false;
			}

			/////// check if tool orinetationis the same as input 

			vec ax, ay, az;
			ax = Nachi_tester.Bars_to_world_matrices[5].getColumn(0);
			ay = Nachi_tester.Bars_to_world_matrices[5].getColumn(1);
			az = Nachi_tester.Bars_to_world_matrices[5].getColumn(2);

			cout << " -- current point -- " << i << endl;
			if (fabs(ax.angle(TOOL.getColumn(0))) > 1e-04) printf(" point id %i axis ax & tcp_x do not match within tolernace \n", i);
			if (fabs(ay.angle(TOOL.getColumn(1))) > 1e-04) printf(" point id %i axis ay & tcp_y do not match within tolernace \n", i);
			if (fabs(az.angle(TOOL.getColumn(2))) > 1e-04) printf(" point id %i axis az & tcp_z do not match within tolernace \n", i);

			// test if current TOOL orientation axes are ortho-normal / perpendicular to each other
			if (TOOL.getColumn(0)*TOOL.getColumn(1) >1e-04)printf(" point id %i axis x & y are not ortho-normal \n", i);
			if (TOOL.getColumn(1)*TOOL.getColumn(2) >1e-04)printf(" point id %i axis y & z are not ortho-normal \n", i);
			if (TOOL.getColumn(2)*TOOL.getColumn(0) >1e-04)printf(" point id %i axis z & x are not ortho-normal \n", i);

			// store corresponding rotations at each point along path
			// for later use such as gcode export & graph-generation etc.
			for (int n = 0; n < 6; n++)rotations[i][n] = Nachi_tester.rot[n];

		}

	}

	void angleCorrection(double * rot_prev)
	{
		if (fabs(rot_prev[3] - Nachi_tester.rot[3]) > 180)
		{
			cout << Nachi_tester.rot[3] << " J3_fk " << endl;

			if (rot_prev[3] < 0 && Nachi_tester.rot[3] > 0) Nachi_tester.rot[3] -= 360;
			if (rot_prev[3] > 0 && Nachi_tester.rot[3] < 0) Nachi_tester.rot[3] += 360;

			cout << Nachi_tester.rot[3] << " J3_new " << endl;
		}

		cout << Nachi_tester.rot[3] << " J4_new_after " << endl;


		if (fabs(rot_prev[5] - Nachi_tester.rot[5]) > 180)
		{

			cout << rot_prev[5] << " J5_prev " << endl;
			cout << Nachi_tester.rot[5] << " J5_fk " << endl;

			if (rot_prev[5] < 0 && Nachi_tester.rot[5] > 0) Nachi_tester.rot[5] -= 360;
			if (rot_prev[5] > 0 && Nachi_tester.rot[5] < 0) Nachi_tester.rot[5] += 360;

			cout << Nachi_tester.rot[5] << " J5_new " << endl;
		}
	}

	void exportGCode(string fileToWrite = "data/MZ07-01-A.083")
	{
		//----- check & compute all necessary values
		checkReach();

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
		//for (int i = actualPathLength - 2; i >= 0; i--) // reverse order
		for (int i = 0; i < actualPathLength - 1; i++)
		{
			// get corresponding joint rotations
			e_x = rotations[i][0];
			e_y = rotations[i][1];
			e_z = rotations[i][2];
			r = rotations[i][3];
			p = rotations[i][4];
			y = rotations[i][5];

			//if (y >= 0 && y < 180) y -= 180; 
			//else y += 180; 

			//if (r >= 0 ) r -= 360;
			//else r += 360;

			//y += (y >= 0 ) ? -180 : 180 ;
			//format as per nachi language
			/*sprintf_s(gcode, "MOVEX A=1,AC=0,SM=0,M1X,L,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),S=1200.0,H=2,MS, CONF=0001", e_x * 10, e_y * 10, e_z * 10, r+90, p, y);*/
			/*sprintf_s(gcode, "MOVEX A=6,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),T=0.1,H=3,MS, CONF=0001", e_x, e_y, e_z, r, p, y);*/
			sprintf_s(gcode, "MOVEX A=6,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),S=30.0,H=3,MS, CONF=0001", e_x, e_y, e_z, r, p, y);

			// output string to open file
			myfile_write << gcode << endl;
		}

		//-close file
		myfile_write.close();
		cout << "--------------------------- EXPORT GCODE COMPLETE ----------------- " << endl;
	}

	////////////////////////////////////////////////////////////////////////// DISPLAY METHODS

	void draw(bool wireFrame = true, bool showSphere = false)
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
		Matrix4 EE = Nachi_tester.Bars_to_world_matrices[5];
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

		if (wireFrame)wireFrameOn();
		Nachi_tester.draw(true); // updates AO render points ;
		if (wireFrame)wireFrameOff();

		if (showSphere) glutSolidSphere(78, 32, 32);
	}


};




pathImporter path;
int currentPointId;
Mesh M;
SliderGroup S;

void setup()
{
	MeshFactory fac;

	//M = fac.createFromOBJ("data/hwc.obj", 1, false, false);

	path.readPath();


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

}

void update(int value)
{
	path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);
}

char s[200];
char t[200];
char jts[400];

void draw()
{

	backGround(0.75);
	drawGrid(200.0);


	S.draw();
	//// ------------------------ draw the path points / Tool orientations 

	path.draw(false);
	//E.draw();
	M.draw();



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
		path.Nachi_tester.rot[1] = 90.0;
		//path.Nachi_tester.rot[2] = -90;
		path.Nachi_tester.ForwardKineMatics(path.Nachi_tester.rot);
	}

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		//B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
}



