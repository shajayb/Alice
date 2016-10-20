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



//
generator<E> faceEdges( metaMesh &m , double threshold)
{
	int a, b;
	vec ePt[3];
	bool e[3];
	vec diff;
	double interp;

	for (int i = 0; i < m.n_f; i++)
	{
		
		for (int j = 0; j < 3 /*m.faces[i].n_e*/; j++)
		{
			a = m.faces[i].edgePtrs[j]->vEnd->id;
			b = m.faces[i].edgePtrs[j]->vStr->id;
		
			diff = (m.positions[b] - m.positions[a]);// .normalise();
			interp = ofMap(threshold, m.scalars[a], m.scalars[b], 0, 1);

			ePt[j] = (interp >= 0.0 && interp <= 1.0) ?  (m.positions[a] + diff * interp) :  ( vec(0, 0, 0) );
			e[j] = (interp >= 0.0 && interp <= 1.0) ? 1 : 0;
		}

		
		if( e[0] && e[1])yield E(ePt[0], ePt[1]);
		if (e[1] && e[2])yield E(ePt[1], ePt[2]);
		if (e[2] && e[0])yield E(ePt[2], ePt[0]);
	
	}
}




////////////////////////////////////////////////////////////////////////////////////////////////

SliderGroup S;
double threshold = 0.5;
metaMesh M;

void setup()
{
	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].attachToVariable(&threshold, 0, 100);

	MeshFactory fac;
	Mesh tmp = fac.createFromOBJ("data/in.obj", 10, false, false);
	M = metaMesh( tmp );
	M.assignScalars();


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
	M.draw(true);

	glPointSize(5);

	glColor3f(1, 0, 0);
	//for (double i = 0; i < threshold; i+= threshold * 0.05)
		for (auto &c : faceEdges(M, threshold))
		{
			//drawLine(c.t, c.f);
			//drawPoint((vec)c.t);
			//drawPoint((vec)c.f);
		
		}

		M.G.draw();
		
	
		M.createGraph(threshold);
		

		long start, end;
		
		start = GetTickCount();

			M.G.drawIslands();

		end = GetTickCount();
		timeStats(start, end, " yield islands ");

		cout << " -------------------------------------------- " << endl;
		//

	glColor3f(0, 0, 0);
	

	S.draw();

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




