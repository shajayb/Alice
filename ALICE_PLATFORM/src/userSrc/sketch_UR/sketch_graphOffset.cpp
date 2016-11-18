#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include"metaMesh.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mesh M;
metaMesh MM;
Graph G;
vec minV, maxV;
double dMin, dMax;
double threshold;
double iter;
SliderGroup S;


//////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
	//G.createVertex(vec(-2, -1, 0));
	//G.createVertex(vec(-1, 1, 0));
	//G.createVertex(vec(1, 1, 0));
	//G.createVertex(vec(1, -1, 0));
	//G.createVertex(vec(-2, 3, 0));
	//G.createVertex(vec(2.1, 3, 0));

	//G.createEdge(G.vertices[0], G.vertices[1]);
	//G.createEdge(G.vertices[1], G.vertices[2]);
	//G.createEdge(G.vertices[2], G.vertices[3]);
	//G.createEdge(G.vertices[4], G.vertices[0]);
	//G.createEdge(G.vertices[5], G.vertices[2]);

	importer imp = *new importer("data/tree_pts.txt", 10000, 5.0);
	imp.readEdges();

	G.reset();
	for (int i = 0; i < imp.nCnt; i++)G.createVertex( imp.nodes[i].pos );
	for (int i = 0; i < imp.eCnt; i++)G.createEdge( G.vertices[ imp.edges[i].n0 ], G.vertices[ imp.edges[i].n1 ] );


	for (int i = 0; i < G.n_v; i++) G.positions[i] *= 1;
	
	G.boundingbox(minV, maxV);
	
	Matrix4 trans;
	trans.translate((minV + maxV) * 0.5);
	trans.scale(1.5);
	minV = minV * trans;
	maxV = maxV * trans;
	

	MeshFactory fac;
	//MM = metaMesh(fac.createFromOBJ("data/in.obj", 1.0, false, false));
	//MM.assignScalars("z");
	MM = MM.createFromPlane(minV, maxV, 100);
	MM.assignScalarsAsLineDistanceField(G);
	MM.getMinMaxOfScalarField(dMin, dMax);

	//
	threshold = 0; ; 

	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].minVal = 0;
	S.sliders[0].maxVal = 400;
	
	cout << dMin << " " << dMax << endl;

}


bool run = false;
void update(int value)
{

	if(run)
		for (int i = 0; i < 10; i++)
			MM.G.smooth_connectedVertices();
			
}

void draw()
{

	backGround(0.75);


	S.draw();

	G.draw();
	wireFrameOn();
	
		////MM.draw();
		MM.G.computeIslandsAsEdgeAndVertexList();
	
		//glColor3f(1, 0, 0);
		MM.G.drawConnectedEdgeList();
	
		//glColor3f(0, 0, 0);
		MM.G.draw();

	wireFrameOff();

	for (int i = 0; i < MM.n_v; i++)
	{
		vec4 clr = getColour(MM.scalars[i], dMin, dMax);
		glColor3f(clr.r, clr.g, clr.b);
		drawPoint(MM.positions[i]);
	}


}
void keyPress(unsigned char k, int xm, int ym)
{
	if( k == ' ')MM.G.smooth_connectedVertices();
	if (k == 'w')MM.G.writeGraph(1.0);
	if (k == 'r')run = !run;
}

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		if(HUDSelectOn)MM.createIsoContourGraph(threshold);

	}
}

void mouseMotion(int x, int y)
{
	//if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		if (HUDSelectOn)MM.createIsoContourGraph(threshold);
		
	}
}

