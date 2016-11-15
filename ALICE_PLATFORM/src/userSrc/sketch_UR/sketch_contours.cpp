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

struct E
{
	vec t, f;

	E() {}
	E( vec &_t,vec &_f)
	{
		t = _t;
		f = _f;
	}
};



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

	vec assignScalars(string component = "z")
	{
		if( component == "z")
		for (int i = 0; i < n_v; i++)scalars[i] = positions[i].z;// vertices[i].getMeanCurvatureGradient(positions).mag(); // 

		if (component == "m")
			for (int i = 0; i < n_v; i++)scalars[i] =  vertices[i].getMeanCurvatureGradient(positions).mag(); // 
		//(&m.positions[i])* DEG_TO_RAD ; //// m.positions[i].y; // distanceTo(vec(0, 0, 0));

		vec ret;
		ret.x = 1e+10;
		ret.y = 1e-10;
		for (int i = 0; i < n_v; i++)
		{
			ret.x = MIN(scalars[i], ret.x);
			ret.y = MAX(scalars[i], ret.y);

		}

		return ret;
	}

	void createGraph( double threshold)
	{
		int a, b;
		vec diff;
		double interp;
		Vertex v;
		int *edge_vertex_ids = new int[n_e];
		
		
		G.n_v = G.n_e = 0;

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

		delete[]edge_vertex_ids;
	}

};

////////////////////////////////////////////////////////////////////////////////////////////////

SliderGroup S;
double threshold = 0.5;
metaMesh M;

ButtonGroup B;
bool tillThreshold = false;
bool showMesh = true;
bool scalarsCurv = false;

void setup()
{
	

	MeshFactory fac;
	Mesh tmp = fac.createFromOBJ("data/in.obj", 1, false, false);
	M = metaMesh( tmp );
	

	vec min, max;
	M.boundingBox(min, max);
	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].attachToVariable(&threshold, min.z, max.z);

	B = *new ButtonGroup(vec(50, 350, 0));
	B.addButton(&tillThreshold, "tillThreshold");
	B.addButton(&showMesh, "showMesh");
	B.addButton(&scalarsCurv, "scalarsCurv");

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
	if(showMesh)M.draw(true);

	vec mnx = (scalarsCurv) ? M.assignScalars("m") : M.assignScalars("z");
	S.sliders[0].maxVal = mnx.y;
	S.sliders[0].minVal = mnx.x;
	double inc = (mnx.y - mnx.x) * 0.005;
	glPointSize(5);

	
	for( double t = (tillThreshold ? S.sliders[0].minVal : threshold ) ; t <= threshold ; t += inc )
	{
		M.createGraph(t);
		glColor3f(1, 0, 0);
		M.G.draw();
		if( !tillThreshold )M.G.drawIslands();
	}


	glColor3f(0, 0, 0);
	
	S.draw();
	B.draw();

}
void keyPress(unsigned char k, int xm, int ym)
{

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
	//if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
	}
}




