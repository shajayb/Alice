#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;

#include"graph.h"

Graph G;


class metaMesh : public Mesh
{
public:

	Graph G;
	array<double, MAX_VERTS> scalars;
	metaMesh()
	{
		
	};
	metaMesh( Mesh &in)
	{
		G = *new Graph();
		
		///
		
		for (int i = 0; i < in.n_v; i++) positions[i] = in.positions[i];
		for (int i = 0; i < in.n_v; i++) createVertex(positions[i]);
		Vertex *f_v[MAX_VALENCE];
		for (int i = 0; i < in.n_f; i++)
		{
			int *face_verts = in.faces[i].faceVertices();
			for (int j = 0; j < in.faces[i].n_e; j++)f_v[j] = &vertices[face_verts[j]];
			createFace(f_v, in.faces[i].n_e);
		}

		for (int i = 0; i < n_f; i++)faces[i].faceVertices();
	}

	void assignScalars(string component = "y")
	{
		for (int i = 0; i < n_v; i++)scalars[i] = positions[i].z;// vertices[i].getMeanCurvatureGradient(positions).mag(); // 
		//(&m.positions[i])* DEG_TO_RAD ; //// m.positions[i].y; // distanceTo(vec(0, 0, 0));

	}

	void createGraph( double threshold)
	{
		int a, b;
		vec diff;
		double interp;
		Vertex v;
		int *edge_vertex_ids = new int[n_e];

		G.reset();

		for (int i = 0; i < n_e; i++)
		{
			a = edges[i].vStr->id;
			b = edges[i].vEnd->id;

			diff = (positions[b] - positions[a]);// .normalise();
			interp = ofMap(threshold, scalars[a], scalars[b], 0, 1);
			if (interp >= 0.0 && interp <= 1.0)
			{
				v = * ( G.createVertex((positions[a] + diff * interp)) );
				//v.clr = getColour(interp,0,1);
				edge_vertex_ids[i] = v.id;
			}
			else
				edge_vertex_ids[i] = -1;

		}


		////

		int e_id;
		int e_v_ids[3];

		for (int i = 0; i < n_f; i++)
		{

			e_v_ids[0] = e_v_ids[1] = e_v_ids[2] = -1;;

			for (int j = 0; j < 3 /*m.faces[i].n_e*/; j++)
			{
				e_id = faces[i].edgePtrs[j]->id;
				e_v_ids[j] = edge_vertex_ids[e_id];
			}

			
			if (e_v_ids[0] >= 0 && e_v_ids[1] >= 0)G.createEdge(G.vertices[e_v_ids[0]], G.vertices[e_v_ids[1]]);
			if (e_v_ids[1] >= 0 && e_v_ids[2] >= 0)G.createEdge(G.vertices[e_v_ids[1]], G.vertices[e_v_ids[2]]);
			if (e_v_ids[2] >= 0 && e_v_ids[0] >= 0)G.createEdge(G.vertices[e_v_ids[2]], G.vertices[e_v_ids[0]]);
			

		}

		cout << G.n_e << " " << n_v << endl;

		delete[]edge_vertex_ids;
	}

};


////////////////////////////////////////////////////////////////////////////////////////////////

SliderGroup S;
double threshold = 0.5;
metaMesh M;
importer OBJ;

void setup()
{


	vec min, max;
	MeshFactory fac;
	
	//Mesh tmp = fac.createFromOBJ("data/in.obj", 10, true, false);
	
	OBJ = * new importer("data/in.obj", MAX_VERTS, 1.0);
	OBJ.read_obj();
	
	vec *p = new vec[OBJ.nCnt];
	for (int i = 0; i < OBJ.nCnt; i++)p[i] = OBJ.nodes[i].pos;


	Mesh OBJMesh;
	Vertex *f_verts[MAX_VALENCE];
	// ------------------------------------ make vertices 

	for (int i = 0; i < OBJ.nCnt; i++)OBJMesh.createVertex(p[i]);

	// ------------------------------------ make faces ;

	for (int i = 0, strt = 0, num; i < OBJ.fCnt; i++, strt += num)
	{
		num = OBJ.faceCounts[i]; // number of vertices per face - tri,quad,nGon ;

							 // --- collect face vertices from global vertex list ;
		for (int j = strt, f_v_cnt = 0; j < strt + num; j++, f_v_cnt++)
			f_verts[f_v_cnt] = &OBJMesh.vertices[OBJ.faces[j]];

		OBJMesh.createNGon(f_verts, num, true);
	}

	// check euler characteristics .

	// recent addition .
	//for (int i = 0; i < OBJ.n_f; i++)OBJ.faces[i].faceVertices();


	//fac.createFromArrays(p, OBJ.nCnt, OBJ.faceCounts, OBJ.fCnt, OBJ.faces, true );
//	M = metaMesh( tmp );
	//M.assignScalars();
//	M.boundingBox(min, max);
	

	//S = *new SliderGroup();
	//S.addSlider(&threshold, "threshold");
	//S.sliders[0].attachToVariable(&threshold,min.z, max.z);


}

void update(int value)
{


}

void timeStats(long &strt, long& end, string str)
{
	elapsedTime = end - strt;
	cout << elapsedTime / 1000.0f << " - " << str << endl;
}


void draw()
{

	backGround(0.75);

	glLineWidth(1.0);
	//M.draw(true);

	glPointSize(5);

	glColor3f(1, 0, 0);

		
		
	long start, end;
	start = GetTickCount();
		
	//	M.createGraph(threshold);
		//M.G.draw();

		
	for (int i = 0; i < OBJ.nCnt; i++)drawPoint(OBJ.nodes[i].pos);

			//M.G.drawIslands();

		end = GetTickCount();
		//timeStats(start, end, " graph + islands ");

	//	cout << " -------------------------------------------- " << endl;
		//

	glColor3f(0, 0, 0);
	

	//S.draw();

}
void keyPress(unsigned char k, int xm, int ym)
{

}



void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		//		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{
	//if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
	}
}




