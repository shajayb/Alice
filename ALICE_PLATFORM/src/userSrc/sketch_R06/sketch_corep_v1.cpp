


#ifdef _MAIN_

#include "main.h"
#include "Matrix.h"
#include "MODEL.h"
#include "interpolate.h"
#include "metaMesh.h"
#include "rigidCube.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;


//////////////////////////////////////////////////////////////////////////


#define rx ofRandom(-1,1)




//////////////////////////////////////////////////////////////////////////




class dataMesh : public Mesh
{
public:
	dataMesh() {};
	~dataMesh() {};

	double scalars[MAX_VERTS];
};

//metaMesh CombineMesh;

void combineMeshes(Mesh &sub, metaMesh &parent)
{
	int P_nv = parent.n_v;
	for (int i = 0; i < sub.n_v; i++)parent.createVertex(sub.positions[i]);

	Vertex *FV[6];
	for (int i = 0; i < sub.n_f; i++)
	{
		int *f_v = sub.faces[i].faceVertices();

		for (int j = 0; j < sub.faces[i].n_e; j++)
		{
			int id = sub.vertices[f_v[j]].id;
			id += P_nv;
			FV[j] = &parent.vertices[id];
		}

		parent.createFace(FV, sub.faces[i].n_e);
	}

	for (int i = 0; i < parent.n_f; i++)parent.faces[i].faceVertices();
}

#define MAX_NUM 1000
vec Pts[100];
void meshFromGraph(Graph &G, metaMesh &CombinedMesh, double endOffset = 0.2, double wid = 0.2)
{

	MeshFactory fac;

	Mesh Prim = fac.createFromOBJ("data/cube_tri.obj", 1.0, false);
	Mesh Prim_copy = fac.createFromOBJ("data/cube_tri.obj", 1.0, false);
	vec *P = new vec[MAX_NUM];

	//////////////////////////////////////////////////////////////////////////
	vec u, v, n, cen;
	float Scale[3];
	Matrix4 T;

	////////////////////////////////////////////////////////////////////////// iterate through edges

	for (int i = 0; i < G.n_e; i++)
	{

		//transform matrix
		n = G.positions[G.edges[i].vEnd->id] - G.positions[G.edges[i].vStr->id];
		u = n.cross(vec(1, 0, 0));
		v = n.cross(u);
		cen = (G.positions[G.edges[i].vEnd->id] + G.positions[G.edges[i].vStr->id]) * 0.5;

		Scale[2] = n.mag() - (endOffset * 4.0);
		Scale[0] = Scale[1] = wid;
		u.normalise(); v.normalise(); n.normalise();

		T.setColumn(0, u * Scale[0]);
		T.setColumn(1, v* Scale[1]);
		T.setColumn(2, n* Scale[2]);
		T.setColumn(3, cen);

		//transform
		for (int n = 0; n < Prim.n_v; n++)Prim.positions[n] = T * Prim_copy.positions[n];

		//////////////////////////////////////////////////////////////////////////
		int nv = CombinedMesh.n_v;
		combineMeshes(Prim, CombinedMesh);

		for (int o = nv; o < CombinedMesh.n_v; o++)
		{
			double val;
			vec p = CombinedMesh.positions[o];
			val = (p - cen) * n;
			CombinedMesh.scalars[o] = val;
		}
	}

	////////////////////////////////////////////////////////////////////////// iterate through vertices
	int num = 0;
	vec plPts[4];
	plPts[0] = vec(-1, -1, 0); plPts[1] = vec(-1, 1, 0);
	plPts[2] = vec(1, 1, 0); plPts[3] = vec(1, -1, 0);

	for (int vv = 0; vv < G.n_v; vv++)
	{

		int valence = G.vertices[vv].n_e;
		if (valence < 2)continue;
		num = 4 * valence;

		int cnt = 0;
		for (int i = 0; i < valence; i += 1)
		{
			Edge E = *G.vertices[vv].edgePtrs[i];
			int other = E.vEnd->id == vv ? E.vStr->id : E.vEnd->id;
			n = G.positions[other] - G.positions[vv];
			u = n.cross(vec(1, 0, 0));
			v = n.cross(u);
			u.normalise(); v.normalise(); n.normalise();
			T.setColumn(0, u);
			T.setColumn(1, v);
			T.setColumn(2, n);
			T.setColumn(3, G.positions[vv] + n.normalise() * endOffset);
			for (int j = 0; j < 4; j++)P[cnt++] = T * (plPts[j] * wid * 0.5);

		}

		Prim = quickHull(P, num);
		//M = quickHull(Pts, num);

		//////////////////////////////////////////////////////////////////////////
		int nv = CombinedMesh.n_v;
		combineMeshes(Prim, CombinedMesh);

		for (int o = nv; o < CombinedMesh.n_v; o++)
		{
			double val = 1e10;
			vec p = CombinedMesh.positions[o];

			// get distance to neares plane, for each of hte newly added vertices..
			for (int i = 0; i < 1; i += 1)
			{
				Edge E = *G.vertices[vv].edgePtrs[i];
				int other = E.vEnd->id == vv ? E.vStr->id : E.vEnd->id;
				n = G.positions[other] - G.positions[vv];
				n.normalise();
				cen = G.positions[vv] + (n * 0.1);
				val = MIN((p - cen) * n, val);
			}

			CombinedMesh.scalars[o] = val;
		}
	}

}

//////////////////////////////////////////////////////////////////////////

metaMesh MM;
double threshold = 1.0;

Graph G;
Mesh M;
double Eoff = 0.2;
double Width = 0.2;

//////////////////////////////////////////////////////////////////////////

void setup()
{
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


	//////////////////////////////////////////////////////////////////////////

	//G = *new Graph();
	//G.createVertex(vec(0, 0, 0)); //0
	//G.createVertex(vec(0, 0, 1)); // 1
	//G.createVertex(vec(1, 0, 2)); // 2
	//G.createVertex(vec(-1, 0, 2)); //3
	//G.createVertex(vec(-1, 0, 3)); //4

	//G.createEdge(G.vertices[0], G.vertices[1]);
	//G.createEdge(G.vertices[1], G.vertices[2]);
	//G.createEdge(G.vertices[1], G.vertices[3]);
	//G.createEdge(G.vertices[3], G.vertices[4]);
	string file = "data/corep_graph_0.txt";
	importer imp = *new importer(file, 10000, 1.0);
	imp.readEdges();


	//---------
	G.reset();
	for (int i = 0; i < imp.nCnt; i++)G.createVertex(imp.nodes[i].pos);
	for (int i = 0; i < G.n_v; i++) swap(G.positions[i].y, G.positions[i].z);

	for (int i = 0; i < imp.eCnt; i++)G.createEdge(G.vertices[imp.edges[i].n0], G.vertices[imp.edges[i].n1]);

	// ------------ scale to fit 
	vec minV, maxV;
	G.boundingbox(minV, maxV);

	Matrix4 trans;
	//double preferedDiag = 50;
	//trans.scale(preferedDiag / (minV.distanceTo(maxV)));
	//for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];

	//G.boundingbox(minV, maxV);

	//trans.identity();
	//trans.translate((minV + maxV) * -0.5);
	//for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];


	//////////////////////////////////////////////////////////////////////////

	meshFromGraph(G, MM);

	//////////////////////////////////////////////////////////////////////////
	S.addSlider(&threshold, "threshold");

	S.addSlider();
	S.addSlider();
	S.addSlider(&Eoff, "edgeOffset");
	S.addSlider(&Width, "width");
	S.sliders[4].maxVal = 5; //S.sliders[1].maxVal = 0.05;
}

void update(int value)
{

	//qh_vertex_t *vertices = new qh_vertex_t[num];

	//for (int i = 0; i < num; ++i) {

	//	vertices[i].z = pts[i].pt.z;
	//	vertices[i].x = pts[i].pt.x;
	//	vertices[i].y = pts[i].pt.y;
	//}

	//mesh = qh_quickhull3d(vertices, num);

}

void draw()
{

	backGround(0.75);
	drawGrid(20);

	glPushMatrix();
	glScalef(5, 5, 5);
	MM.display(true,true,true);
	MM.drawIsoContoursInRange(threshold, 0.1);

	glColor4f(1, 1, 1, 1);

	//CombineMesh.draw(true);
	M.draw();
	G.draw();

	//glPointSize(3);
	//for (int i = 0; i < 10; i++)drawPoint(Pts[i]);
	//glPointSize(1);

	glPopMatrix();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{
	if (HUDSelectOn)
	{

		keyPress('S', 0, 0);
	}
}

void keyPress(unsigned char k, int xm, int ym)
{

	//if (k == 'c')
	//{

	//	num = 1000;
	//	for (int i = 0; i < num; i++) P[i] = vec(rx, rx, rx).normalise() * 10;

	//	M = quickHull(P, num);
	//	/*MeshFactory fac;
	//	M = fac.createPlatonic(1, 6);*/
	//}


	//if (k == 'b')
	//{
	//	num = 1000;
	//	for (int i = 0; i < num; i++) P[i] = vec(rx, rx, rx) * 10;
	//	M = quickHull(P, num);



	//}

	//if (k == 'p')
	//{
	//	for (int vv = 1; vv < G.n_v; vv++)
	//	{
	//		//int vv = 1;
	//		int valence = G.vertices[vv].n_e;
	//		if (valence < 2)continue;
	//		num = 4 * valence;

	//		Matrix4 rot;
	//		vec plPts[4];
	//		plPts[0] = vec(-1, -1, 0);
	//		plPts[1] = vec(-1, 1, 0);
	//		plPts[2] = vec(1, 1, 0);
	//		plPts[3] = vec(1, -1, 0);

	//		float inc = PI * 2.0 / float(num);
	//		int cnt = 0;
	//		for (int i = 0; i < valence; i += 1)
	//		{
	//			Edge E = * G.vertices[vv].edgePtrs[i];
	//			int other = E.vEnd->id == vv ? E.vStr->id : E.vEnd->id;
	//			vec n = G.positions[other] - G.positions[vv];
	//			//vec n = vec(sin(float(i) * (PI * 2.0 / num)), cos(float(i) * (PI * 2.0 / num)), rx * 1);
	//			vec u = n.cross(vec(1, 0, 0));
	//			vec v = n.cross(u);
	//			u.normalise(); v.normalise(); n.normalise();
	//			rot.setColumn(0, u);
	//			rot.setColumn(1, v);
	//			rot.setColumn(2, n);
	//			rot.setColumn(3, G.positions[vv] + n.normalise() * 0.1);
	//			for (int j = 0; j < 4; j++)P[cnt++] = rot * (plPts[j] * 0.1);
	//		}

	//		M = quickHull(P, num);
	//		combineMeshes(M, CombineMesh);
	//	}
	//}

	//if (k == 'w')
	//	M.writeOBJ("data/conv", "", M.positions, false);

	if (k == 's')
	{
		MM.n_v = MM.n_f = MM.n_e = 0;
		meshFromGraph(G, MM, Eoff, Width);
	}
	if (k == 'S')
	{
		double mn, mx;
		MM.getMinMaxOfScalarField(mn, mx);
		S.sliders[0].maxVal = mx * 1.5;
		S.sliders[0].minVal = 0.0;
		MM.createIsoContourGraph(threshold);
	}

	if (k == 'w')
	{
		MM.writeOBJ("data/convex.obj", "", MM.positions, false);
		M.writeOBJ("data/convex_tmp.obj", "", M.positions, false);
	}
}





#endif // _MAIN_
