#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "graph.h"

class metaMesh : public Mesh
{
public:

	Graph G;
	array<double, MAX_VERTS> scalars;
	metaMesh()
	{

	};
	metaMesh(Mesh &in)
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

	void createGraph(double threshold)
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
				v = *(G.createVertex((positions[a] + diff * interp)));
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

		//cout << G.n_e << " " << n_v << endl;

		delete[]edge_vertex_ids;
	}

};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mesh M;
metaMesh MM;
Graph G;
vec minV, maxV;
double dMin, dMax;
double threshold;
SliderGroup S;

double nearestPointOnEdge(vec &a, vec&b, vec &p, vec&pt)
{
	vec n = (b - a).cross(vec(0, 0, 1));
	n.normalise();
	pt = n * ((a - p)*n);
	vec ed = (b - a).normalise();

	if ( (p-a) * (ed) > 0 && (p - a) * (ed) < (b-a).mag())
		return (a - p)*n;
	else
	{
		float d1 = pt.distanceTo(a);
		float d2 = pt.distanceTo(b);
		return MIN(d1, d2);
	}

	//return  ((a - pt) * (a - pt) < (b - a) * (b - a)) ? (a - p)*n : 1e10;
}

//////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
	G.createVertex(vec(-2, -1, 0));
	G.createVertex(vec(-1, 1, 0));
	G.createVertex(vec(1, 1, 0));
	G.createVertex(vec(1, -1, 0));
//	G.createVertex(vec(-2, 3, 0));

	G.createEdge(G.vertices[0], G.vertices[1]);
	G.createEdge(G.vertices[1], G.vertices[2]);
	G.createEdge(G.vertices[2], G.vertices[3]);
	//G.createEdge(G.vertices[4], G.vertices[0]);
	
	
	for (int i = 0; i < G.n_v; i++) G.positions[i] *= 10;
	
	G.boundingbox(minV, maxV);
	
	Matrix4 trans;
	trans.translate((minV + maxV) * 0.5);
	trans.scale(1.4);
	minV = minV * trans;
	maxV = maxV * trans;
	

	int rowCnt = 0;
	int colCnt = 0;
	for (float x = minV.x; x <= maxV.x; x += (maxV.x - minV.x)*0.02)
	{
		rowCnt = 0;
			for (float y = minV.y; y <= maxV.y; y += (maxV.y - minV.y)*0.02)
			{
				M.createVertex(vec(x, y, 0));
				rowCnt++;
			}
		colCnt++;
	}
		
	
	Vertex *fVerts[4];
	for (int i = 0; i < rowCnt-1; i++)
	{
		for (int j = 0; j < colCnt; j++)
		{
			if (j == 0)continue;
			fVerts[0] = & M.vertices[i*colCnt + j];
			fVerts[1] = &M.vertices[(i+1)*colCnt + j];
			fVerts[2] = &M.vertices[(i+1)*colCnt + j-1];
			fVerts[3] = &M.vertices[i*colCnt + j-1];
			M.createNGon(fVerts, 4,true);
		}
	}


	for (int i = 0; i < M.n_f; i++)M.faces[i].faceVertices();

	
	MM = metaMesh(M);
	
	dMin = 1e10;
	dMax = dMin * -1;

	for (int i = 0; i < MM.n_v; i++)
	{
		
		vec pt;
		double d = 1e10;
		double dChk;
			//= nearestPointOnEdge(G.positions[1], G.positions[0], p, pt);

		for (int j = 0; j < G.n_e; j++)
		{
			int e0, e1;
			e0 = G.edges[j].vEnd->id;
			e1 = G.edges[j].vStr->id;
			dChk = nearestPointOnEdge(G.positions[e0], G.positions[e1], MM.positions[i], pt);
		
			d = MIN(dChk, d);
		}

		MM.scalars[i] = d;
		
		dMin = MIN(dMin, d);
		dMax = MAX(dMax, d);
	}


	//
	threshold = 0; ;; (dMin);

	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].minVal = -1 ;
	S.sliders[0].maxVal = 1.0;
	

	
}



void update(int value)
{

	MeshFactory fac;

	MM.createGraph((dMin + dMax) * threshold);
}

void draw()
{

	backGround(0.75);

	G.draw();

	S.draw();

	vec p = minV;
	vec pt;
	double d = nearestPointOnEdge(G.positions[0], G.positions[1], p,pt);

	glPointSize(10);
	drawPoint(p);
	drawLine(p, p + pt);


	wireFrameOn();
		//MM.draw();
		MM.G.draw();
		//MM.G.drawIslands();
	wireFrameOff();

	for (int i = 0; i < M.n_v; i++)
	{
		vec4 clr = getColour(MM.scalars[i], dMin, dMax);
		glColor3f(clr.r, clr.g, clr.b);
		drawPoint(MM.positions[i]);
	}


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

