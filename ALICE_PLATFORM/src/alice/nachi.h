
#ifndef _NACHI_

#define _NACHI_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

#include "graph.h"
#include "Matrix.h"

class EndEffector
{
public:

	Mesh M;
	Robot_Symmetric Nachi_tester;
	vec XA_f, YA_f, ZA_f;
	vec XA, YA, ZA;
	vec cen, cen_f;
	Matrix3 rot, rotf;
	Matrix4 transformMatrix;

	EndEffector() {};
	EndEffector(string file)
	{
		MeshFactory fac;
		M = fac.createFromOBJ(file, 1, false);
		invertMeshToLocal();

		int i, j, k;
		i = 5;
		j = 7;
		k = 11;

		//invertTCP_toLocalOrigin();
	}

	void invertTCP_toLocalOrigin()
	{
		//
		vec cen = (M.positions[5] + M.positions[1]) * 0.5;
		vec xAxis = M.positions[5] - M.positions[3];
		vec yAxis = M.positions[1] - M.positions[3];
		vec zAxis = xAxis.cross(yAxis);
		Matrix4 T;
		T.setColumn(0, xAxis.normalise() * -1);
		T.setColumn(1, yAxis.normalise() * 1);
		T.setColumn(2, zAxis.normalise()* -1);
		T.setColumn(3, cen);
		T.invert();

		for (int i = 0; i < M.n_v; i++)M.positions[i] = T * M.positions[i];
	}

	void invertMeshToLocal()
	{
		Nachi_tester.ForwardKineMatics(Nachi_tester.rot);

		transformMatrix = Nachi_tester.Bars_to_world_matrices[5];

		XA_f = transformMatrix.getColumn(0).normalise();
		YA_f = transformMatrix.getColumn(1).normalise();
		ZA_f = transformMatrix.getColumn(2).normalise();
		cen_f = transformMatrix.getColumn(3);

		cen = (M.positions[1] + M.positions[5]) * 0.5;

		YA = M.positions[5] - M.positions[7];
		XA = M.positions[1] - M.positions[7];
		//XA *= -1;
		ZA = YA.cross(XA);

		XA.normalise();
		YA.normalise();
		ZA.normalise();

		rot.setColumn(0, XA); rotf.setColumn(0, XA_f);
		rot.setColumn(1, YA);  rotf.setColumn(1, YA_f);
		rot.setColumn(2, ZA); rotf.setColumn(2, ZA_f);
	}

	void drawAtLocation(Matrix4 &EE, bool wireframe = true)
	{
		for (int i = 0; i < M.n_v; i++)M.positions[i] = EE * M.positions[i];// to tcip

		//if (wireframe)wireFrameOn();
		draw();
		//if (wireframe)wireFrameOff();

		EE.invert();
		for (int i = 0; i < M.n_v; i++)M.positions[i] = EE * M.positions[i];// to tcip
	}

	void draw()
	{
		//Nachi_tester.draw();
		M.draw(true);
		//int i = 1 + 4 ;
		//char c[200];
		//sprintf(c, "%i ", i);
		//string s = "";
		//s += c;
		//drawString(s, M.positions[i]);

		//i = 3 + 4 ;
		//sprintf(c, "%i ", i);
		//s = "";
		//s += c;
		//drawString(s, M.positions[i]);


		//i = 7 + 4;
		//sprintf(c, "%i ", i);
		//s = "";
		//s += c;
		//drawString(s, M.positions[i]);

		//i = 0;
		//sprintf(c, "%i ", i);
		//s = "";
		//s += c;
		//drawString(s, (M.positions[7] + M.positions[11]) * 0.5);

		int i = 1;
		char c[200];
		sprintf(c, "%i ", i);
		string s = "";
		s += c;
		drawString(s, M.positions[i]);

		i = 7;
		sprintf(c, "%i ", i);
		s = "";
		s += c;
		drawString(s, M.positions[i]);


		i = 5;
		sprintf(c, "%i ", i);
		s = "";
		s += c;
		drawString(s, M.positions[i]);


		glColor3f(1, 0, 0); drawLine(cen, cen + XA);
		glColor3f(0, 1, 0); drawLine(cen, cen + YA);
		glColor3f(0, 0, 1); drawLine(cen, cen + ZA);

		glColor3f(1, 0, 0); drawLine(cen_f, cen_f + XA_f);
		glColor3f(0, 1, 0); drawLine(cen_f, cen_f + YA_f);
		glColor3f(0, 0, 1); drawLine(cen_f, cen_f + ZA_f);

	}
};


// derive your class from a base / existing class 
class pathImporter :public importer
{

#define maxPts 59999 // max nachi controller limit
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
	EndEffector E_disp;
	Graph taskGraph;
	Matrix4 fTrans;
	Mesh M;
	metaMesh MM;
	////////////////////////////////////////////////////////////////////////// CLASS METHODS 

	pathImporter()
	{
		currentPointId = 0;
		Nachi_tester.addMeshes();
		E = *new EndEffector("data/EE_disp.obj");
		E_disp = *new EndEffector("data/EE_disp.obj");

		Matrix4 EE = E.transformMatrix;
		EE.invert();
		for (int i = 0; i < E.M.n_v; i++)E.M.positions[i] = EE * E.M.positions[i];// to tcip

		actualPathLength = 0;
		taskGraph = *new Graph();
		taskGraph.reset();

		reachable = new bool[maxPts];
	}
	void readPath(string fileToRead = "data/path.txt", string delimiter = ",", float inc = 0)
	{
		cout << "reading file for path " << fileToRead << endl;

		////////////////////////////////////////////////////////////////////////// read file

		std::fstream fs(fileToRead.c_str(), ios::in);

		if (fs.fail())
		{
			cout << " error in file reading " << fileToRead << endl;
			return;
		}

		//actualPathLength = 0;
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

			//tcp.x += 5.0;
			tcp.z += inc;
			addPoint(tcp);
		}

		fs.close();

		reachable = new bool[actualPathLength];

		////////////////////////////////////////////////////////////////////////// bounding box of file

		getBoundingBox();

		//	checkReach();
		copyPathToGraph();

	}

	void readOBJ(string fileToRead = "data/path.txt" , float zHt = 0.2 , double spacing = 1.0, double distTol = 0.05)
	{
		
		MeshFactory fac;
		M = fac.createFromOBJ(fileToRead, 100.0, false);

		cout << " nachi" << fileToRead << endl;
		{
			vec minV, maxV;
			Matrix3x3 PCA_mat;
			vec mean, eigenValues, eigenvecs[3];
			PCA_mat.PCA(M.positions, M.n_v, mean, eigenValues, eigenvecs);
			M.boundingBox(minV, maxV);

			for (int i = 0; i < M.n_v; i++)
				M.positions[i] -= (minV + maxV)*0.5;
			
			vec x = eigenvecs[0].normalise();
			if( fabs(x.x) > 0.1 && x.y > 0.1)
			{
				
				x.z = 0;
				vec z = vec(0, 0, 1);
				vec y = x.cross(z).normalise();

				Matrix3 trans;
				trans.setColumn(0, x);
				trans.setColumn(1, y);
				trans.setColumn(2, z);
				trans.transpose();

				for (int i = 0; i < M.n_v; i++)
					M.positions[i] = trans * M.positions[i];

			}

		}

		cout << "invert done" << endl;
		////////////////////////////////////////////////////////////////////////// transform to base plate
		{
			vec x, y, z, cen;
			cen = vec(53.9327, -2.2114, -17.4015);
			x = vec(1, 0, 0).normalise();
			z = vec(0, 0, -1);
			y = x.cross(z);

			Matrix4 fTrans;
			fTrans.identity();
			fTrans.setColumn(0, x.normalise());
			fTrans.setColumn(1, y.normalise());
			fTrans.setColumn(2, z.normalise());
			fTrans.setColumn(3, cen);


			for (int i = 0; i < M.n_v; i++)
				M.positions[i] = fTrans * M.positions[i];

			vec minV, maxV;
			M.boundingBox(minV, maxV);
			double diff = cen.z - minV.z;


			for (int i = 0; i < M.n_v; i++)
				M.positions[i].z += diff;
		}
		cout << "trans done" << endl;
		//////////////////////////////////////////////////////////////////////////
		metaMesh MM(M);;

		MM.assignScalars("z");
		double minZ, maxZ;
		//MM.getMinMaxOfScalarField(minZ, maxZ);

		minZ = -17.4015; maxZ = 0;
		///////////
		double z = 0; 
		actualPathLength = 0;
		vec refPt_renumber;
		vector<vec> startPts;
		for (double  z = minZ ; z < maxZ; z+= zHt )
		{
			MM.createIsoContourGraph(z);
			
			cout << z << endl;
				if (!MM.G.n_v > 0)continue;
			cout << MM.G.n_v << endl;

			MM.G.computeIslandsAsEdgeAndVertexList();
			MM.convertContourToToroidalGraph();
			MM.G.redistribute_toroidal(spacing * 0.5);

			int iterations = 0;
			//while (fabs(MM.G.averageEdgeLenght() - spacing) < distTol && iterations < 1000)
			{
				for (int i = 0; i < 10; i++)MM.G.smoothGraph(10);
					
				//iterations++;
			}

			// renumber
			if (z == minZ)refPt_renumber = MM.G.positions[0];
			refPt_renumber.z = z;
			MM.G.renumber(refPt_renumber);
			
			for (int i = 0; i < MM.G.n_v; i++)addPoint(MM.G.positions[i]);
			startPts.push_back(MM.G.positions[0]);
		}

		for (int i = 0; i < startPts.size(); i++)
			printf("%1.2f,%1.2f,%1.2f \n", startPts[i].x, startPts[i].y, startPts[i].z);

		//////////////////////////////////////////////////////////////////////////
		for (int i = 0; i < actualPathLength; i++)reachable[i] = true;
	}
	////////////////////////////////////////////////////////////////////////// UTILITY METHODS

	vec extractVecFromStringArray(int id, vector<string> &content)
	{
		return vec(atof(content[id].c_str()), atof(content[id + 1].c_str()), atof(content[id + 2].c_str()));
	}
	void assignDefaultFrame()
	{
		tcp = vec(0, 0, 0);
		tcp_x = vec(-1, 0, 0);
		tcp_y = vec(0, 1, 0);
		tcp_z = vec(0, 0, -1);
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
	void angleBetweenFrames(Matrix3 rotA, Matrix3 rotB)
	{
		for (int i = 0; i < 3; i++)
			cout << rotA.getColumn(i).angle(rotB.getColumn(i)) << " ";
		cout << endl;
	}
	void addPoint(vec tcp, vec tcp_x = vec(1, 0, 0), vec tcp_y = vec(0, 1, 0), vec tcp_z = vec(0, 0, -1))
	{

		path[actualPathLength][0] = tcp;
		path[actualPathLength][1] = tcp_x * 1;
		path[actualPathLength][2] = tcp_y * 1;
		path[actualPathLength][3] = tcp_z * 1;
		actualPathLength++;
		if (actualPathLength > maxPts)actualPathLength = 0;
	}
	void copyPathToGraph()
	{
		for (int i = 0; i < actualPathLength; i++)
			taskGraph.createVertex(path[i][0]);
		for (int i = 0; i < actualPathLength; i++)
			taskGraph.createEdge(taskGraph.vertices[taskGraph.Mod(i, actualPathLength)], taskGraph.vertices[taskGraph.Mod(i + 1, actualPathLength)]);
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
	void changeTool(Matrix4 EE, Matrix4 &TOOL, int n)
	{
		vec x = E_disp.XA;
		vec y = E_disp.YA;
		vec z = E_disp.ZA;
		vec cen = E_disp.cen;

		vec xf = E_disp.XA_f;
		vec yf = E_disp.YA_f;
		vec zf = E_disp.ZA_f;
		vec cenf = E_disp.cen_f;


		////  ------------------ inert to origin ;

		Matrix3 trans = E_disp.rot;
		trans.transpose();

		x = trans * x; y = trans * y; z = trans * z;
		xf = trans * xf; yf = trans * yf; zf = trans * zf;

		Matrix4 T;
		T.identity();
		T.setColumn(3, cen);
		T.invert();
		cenf = T * cenf;
		cen = T * cen;
		cenf = cen + z.normalise() * 20.85;


		// -------------- forward to tool location
		trans.setColumn(0, EE.getColumn(0).normalise());
		trans.setColumn(1, EE.getColumn(1).normalise());
		trans.setColumn(2, EE.getColumn(2).normalise());

		x = trans * x; y = trans * y; z = trans * z;
		xf = trans * xf; yf = trans * yf; zf = trans * zf;

		T.identity();
		T.setColumn(3, EE.getColumn(3));
		//cenf += EE.getColumn(3);
		cen += EE.getColumn(3);
		cenf = cen - z.normalise() * 20.85;

		TOOL.setColumn(0, xf.normalise());
		TOOL.setColumn(1, yf.normalise());
		TOOL.setColumn(2, zf.normalise());
		TOOL.setColumn(3, cenf);
	}
	void goToNextPoint()
	{
		Matrix4 TOOL, EE;
		EE = TOOL = getToolLocation(currentPointId);

		changeTool(EE, TOOL, currentPointId);

		double rot_prev[6];
		for (int i = 0; i < 6; i++)rot_prev[i] = Nachi_tester.rot[i];

		Nachi_tester.inverseKinematics_analytical(TOOL, false);

		//angleCorrection(rot_prev);

		vec pt = Nachi_tester.ForwardKineMatics(Nachi_tester.rot);

		//cout << rot_prev[3] - Nachi_tester.rot[3] << " J3 diff " << endl;
		//cout << rot_prev[3] << " J3_prev " << endl;


		//cout << " -- current point -- " << currentPointId << endl;
		vec ax, ay, az;
		ax = Nachi_tester.Bars_to_world_matrices[5].getColumn(0);
		ay = Nachi_tester.Bars_to_world_matrices[5].getColumn(1);
		az = Nachi_tester.Bars_to_world_matrices[5].getColumn(2);
		//cout << pt.mag() << " lenghth " << endl;

		//cout << ax.angle(TOOL.getColumn(0)) << endl;
		//cout << ay.angle(TOOL.getColumn(1)) << endl;
		//cout << az.angle(TOOL.getColumn(2)) << endl;
		vec tcp_ret = Nachi_tester.Bars_to_world_matrices[5].getColumn(3);
	//	cout << Nachi_tester.joints[5].distanceTo(TOOL.getColumn(3)) << " in-out - tcp diff " << endl;


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

			changeTool(EE, TOOL, i);

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
				//printf(" point id %i is unreachable \n", i);
				reachable[i] = false;
			}

			/////// check if tool orinetationis the same as input 

			vec ax, ay, az;
			ax = Nachi_tester.Bars_to_world_matrices[5].getColumn(0);
			ay = Nachi_tester.Bars_to_world_matrices[5].getColumn(1);
			az = Nachi_tester.Bars_to_world_matrices[5].getColumn(2);

		//	cout << " -- current point -- " << i << endl;
			//if (fabs(ax.angle(TOOL.getColumn(0))) > 1e-04) printf(" point id %i axis ax & tcp_x do not match within tolernace \n", i);
			//if (fabs(ay.angle(TOOL.getColumn(1))) > 1e-04) printf(" point id %i axis ay & tcp_y do not match within tolernace \n", i);
			//if (fabs(az.angle(TOOL.getColumn(2))) > 1e-04) printf(" point id %i axis az & tcp_z do not match within tolernace \n", i);

			//// test if current TOOL orientation axes are ortho-normal / perpendicular to each other
			//if (TOOL.getColumn(0)*TOOL.getColumn(1) > 1e-04)printf(" point id %i axis x & y are not ortho-normal \n", i);
			//if (TOOL.getColumn(1)*TOOL.getColumn(2) > 1e-04)printf(" point id %i axis y & z are not ortho-normal \n", i);
			//if (TOOL.getColumn(2)*TOOL.getColumn(0) > 1e-04)printf(" point id %i axis z & x are not ortho-normal \n", i);

			// store corresponding rotations at each point along path
			// for later use such as gcode export & graph-generation etc.
			for (int n = 0; n < 6; n++)rotations[i][n] = Nachi_tester.rot[n];

		}

	}
	void angleCorrection(double * rot_prev)
	{
		if (fabs(rot_prev[3] - Nachi_tester.rot[3]) > 180)
		{
		//	cout << Nachi_tester.rot[3] << " J3_fk " << endl;

			if (rot_prev[3] < 0 && Nachi_tester.rot[3] > 0) Nachi_tester.rot[3] -= 360;
			if (rot_prev[3] > 0 && Nachi_tester.rot[3] < 0) Nachi_tester.rot[3] += 360;

		//	cout << Nachi_tester.rot[3] << " J3_new " << endl;
		}

	//	cout << Nachi_tester.rot[3] << " J4_new_after " << endl;


		if (fabs(rot_prev[5] - Nachi_tester.rot[5]) > 180)
		{

			//cout << rot_prev[5] << " J5_prev " << endl;
			//cout << Nachi_tester.rot[5] << " J5_fk " << endl;

			if (rot_prev[5] < 0 && Nachi_tester.rot[5] > 0) Nachi_tester.rot[5] -= 360;
			if (rot_prev[5] > 0 && Nachi_tester.rot[5] < 0) Nachi_tester.rot[5] += 360;

			//cout << Nachi_tester.rot[5] << " J5_new " << endl;
		}

		/*Nachi_tester.rot[0] = ofClamp(Nachi_tester.rot[0], -170, 170);
		Nachi_tester.rot[1] = ofClamp(Nachi_tester.rot[1], -65, 150);
		Nachi_tester.rot[2] = ofClamp(Nachi_tester.rot[2], -70, 90);
		Nachi_tester.rot[3] = ofClamp(Nachi_tester.rot[3], -150, 150);
		Nachi_tester.rot[4] = ofClamp(Nachi_tester.rot[4], -109, 109);
		Nachi_tester.rot[5] = ofClamp(Nachi_tester.rot[5], -360, 360);*/
	}
	void exportGCode(string fileToWrite = "data/MZ07-01-A.080")
	{
		
		int counter = 0;
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


			//format as per nachi language
			//sprintf_s(gcode, "MOVEX A=6,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),T=0.1,H=3,MS, CONF=0001", e_x, e_y, e_z, r, p, y);
			sprintf_s(gcode, "MOVEX A=6,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),S=300.0,H=3,MS, CONF=0001", e_x, e_y, e_z, r, p, y);

			// output string to open file
			myfile_write << gcode << endl;

			if (i % 1000 == 0)
			{
				myfile_write.close();

				string file = "data/MZ07-01-A.08";
				char s[20];
				itoa(counter, s, 10);
				file += s;

				//- open file
				myfile_write.open(file, ios::out);
				if (myfile_write.fail())cout << " error in opening file  " << fileToWrite << endl;

				counter++;
			}
		}

		//-close file
		myfile_write.close();
		cout << "--------------------------- EXPORT GCODE COMPLETE ----------------- " << endl;
	}
	void exportGCode_3dp(string fileToWrite = "data/bl_1.src")
	{

		checkReach();
		int counter = 0;
	
		//----- instance ofstream for file IO
		ofstream myfile_write;
		char gcode[600];

		//- open file
		myfile_write.open(fileToWrite.c_str(), ios::out);
		if (myfile_write.fail())cout << " error in opening file  " << fileToWrite << endl;

		// --------------------------- copy header from base file
		std::fstream fs("data/3DP_header.txt", ios::in);

			if (fs.fail())cout << " error in file reading " << fileToRead << endl;

		int lcnt = 0;
		while (!fs.eof() && lcnt < 1000)
		{
			char str[2000];
			fs.getline(str, 2000);

			myfile_write << str << endl;
			lcnt++;
		}
		fs.close();
		
		// --------------------------- 
		//- iterate through path
		
		vec tcp;
		double scale = 10;
		tcp = path[0][0]; tcp *= scale;
		sprintf_s(gcode, "PTP{ E6POS: X %1.4f, Y %1.4f, Z %1.4f, A 0, B 90, C 0, E1 0, E2 0, E3 0, E4 0, S 'B 110' } C_PTP", tcp.x,tcp.y,tcp.z);
		myfile_write << gcode << endl;
		
		tcp = path[1][0]; tcp *= scale;
		sprintf_s(gcode, "LIN{ E6POS: X %1.4f, Y %1.4f, Z %1.4f, A 0, B 90, C 0, E1 0, E2 0, E3 0, E4 0 }", tcp.x, tcp.y, tcp.z);
		myfile_write << gcode << endl;

		for (int i = 2; i < actualPathLength - 1; i++)
		{
		
			tcp = path[i][0]; tcp *= scale;
			sprintf_s(gcode, "LIN{ E6POS: X %1.4f, Y %1.4f, Z %1.4f, A 0, B 90, C 0, E1 0, E2 0, E3 0, E4 0 } C_DIS", tcp.x, tcp.y, tcp.z);
			myfile_write << gcode << endl; 
		}

		//------close file
		myfile_write << "HALT" << endl;
		myfile_write << "END" << endl;
		myfile_write.close();
		cout << "--------------------------- EXPORT GCODE COMPLETE ----------------- " << endl;
	}


	////////////////////////////////////////////////////////////////////////// DISPLAY METHODS
	void drawHistograms( int j ,vec cen = vec (50, winH - 50,0), float r = 5.0)
	{
		AL_glLineWidth(1);
		
		float mn, mx;
		mn = 1e10;
		mx = mn * -1;
		for (int i = 0; i < actualPathLength - 1; i++)
		{
			mn = MIN(mn, rotations[i][j]);
			mx = MAX(mn, rotations[i][j]);
		}
		
		setup2d();



		//glBegin(GL_LINES);
			for (int i = 0; i < actualPathLength - 1; i+= 10 )
			{
				double ang = ((PI * 2.0 / actualPathLength) * float(i)) * 1.0;
				vec4 clr = getColour(rotations[i][j], mn, mx);
				double d = ofMap(rotations[i][j], mn, mx, 0, r);
				d = ofClamp(d,0, r);
				glColor3f(clr.r, clr.g, clr.b);
				drawLine(cen, cen + (vec(sin(ang), cos(ang), 0).normalise() * d));
				vec radial = cen + vec(sin(ang), cos(ang), 0).normalise() * d;
				//glVertex2f(cen.x, cen.y);
				//glVertex2f(radial.x, radial.y);
			}
		//glEnd();

		restore3d();
	}
	void drawFrame(Matrix4 &tool, float sz)
	{

		tcp = tool.getColumn(3);
		tcp_x = tool.getColumn(0); tcp_y = tool.getColumn(1); tcp_z = tool.getColumn(2);

		glColor3f(1, 0, 0); drawLine(tcp, tcp + tcp_x.normalise() * sz);
		glColor3f(0, 1, 0); drawLine(tcp, tcp + tcp_y.normalise() * sz);
		glColor3f(0, 0, 1); drawLine(tcp, tcp + tcp_z.normalise() * sz);
	}
	void draw(bool wireFrame = true, bool showSphere = false)
	{
		// ------------------- taskGraph
		//taskGraph.draw();

		// ------------------- TCP locations
		glPointSize(3);
		for (int i = 0; i < actualPathLength; i++)
		{
			reachable[i] ? glColor3f(0, 0, 1) : glColor3f(1, 0, 0);
			drawPoint(path[i][0]);
			/*char c[200];
			sprintf(c, "%i ", i);
			string s = "";
			s += c;
			drawString(s, path[i][0]+ vec(0,0,0.2));*/
		}
		glPointSize(1);

		// ------------------- TCP orientation axes

		for (int i = 0; i < actualPathLength; i++)drawFrame(getToolLocation(i), .1);



		//////////////////////////////////////////////////////////////////////////
		// get TOOL information at current point i

		Matrix4 EE = Nachi_tester.Bars_to_world_matrices[5];
		E.drawAtLocation(EE);

		//////////////////////////////////////////////////////////////////////////
		// ------------------- draw bounding box ;

		wireFrameOn();
			drawCube(min, max);
		wireFrameOff();
		// ------------------- draw Robot ;

		if (wireFrame)wireFrameOn();
			Nachi_tester.draw(false); // updates AO render points ;
		if (wireFrame)wireFrameOff();

		if (showSphere) glutSolidSphere(78, 32, 32);

		for (int i = 0; i < 6; i++)
			drawHistograms(i, vec( i*200 + 150 , winH - 300,0),75);

	}


};
#endif // !_NACHI_

