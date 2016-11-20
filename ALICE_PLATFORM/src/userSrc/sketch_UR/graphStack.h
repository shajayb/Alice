
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

#include "graph.h"
#include "metaMesh.h"

class graphStack
{
public:

	vector<Graph> ContourStack;
	Graph G;
	vec minV, maxV;
	double dMin, dMax;
	double threshold;
	int currentStackLayer;
	metaMesh MM;

	////---------------------------------------------------- CONSTRUCTOR --------------------------------------
	graphStack( )
	{
		//---------
		ContourStack.reserve(25);
	}

	////---------------------------------------------------- COMPUTE  --------------------------------------
	void readGraphAndCreateDataMesh(string fileToImport, float scale = 1.0)
	{
		//---------
		importer imp = *new importer(fileToImport, 10000, 5.0);
		imp.readEdges();

		//---------
		Graph G;
		for (int i = 0; i < imp.nCnt; i++)G.createVertex(imp.nodes[i].pos);
		for (int i = 0; i < imp.eCnt; i++)G.createEdge(G.vertices[imp.edges[i].n0], G.vertices[imp.edges[i].n1]);
		for (int i = 0; i < G.n_v; i++) G.positions[i] *= 1;
		//
		G.boundingbox(minV, maxV);
		//
		Matrix4 trans;
		trans.translate((minV + maxV) * 0.5);
		trans.scale(1.5);
		minV = minV * trans;
		maxV = maxV * trans;

		//---------
		createDataMeshGrid(G);
		createIsoContourGraph(0.1);

	}


	void createDataMeshGrid( Graph &G)
	{
		MeshFactory fac;
		//MM = metaMesh(fac.createFromOBJ("data/in.obj", 1.0, false, false));
		//MM.assignScalars("z");
		MM = MM.createFromPlane(minV, maxV, 100);
		MM.assignScalarsAsLineDistanceField(G);
		MM.getMinMaxOfScalarField(dMin, dMax);
	}

	void createIsoContourGraph( double threshold )
	{
		MM.createIsoContourGraph(threshold);
	}

	void smoothCurrentGraph()
	{
		MM.G.smooth_connectedVertices();
	}

	////---------------------------------------------------- UTILITIES  --------------------------------------


	void setCurrentGraphAsBase()
	{
		//---------
		currentStackLayer = 0;
		ContourStack[currentStackLayer].reset();
		ContourStack[currentStackLayer] = MM.G;
		currentStackLayer++;
	}


	void writeCurrentGraph()
	{
		MM.G.writeGraph(1.0);
	}

	////---------------------------------------------------- DISPLAY  --------------------------------------

	void draw()
	{
		
		for (auto &G : ContourStack)G.draw();
		
		wireFrameOn();

			////MM.draw();
		//G = stack[currentStackLayer];
		//G.computeIslandsAsEdgeAndVertexList();
		//G.drawConnectedEdgeList();
		MM.G.draw();

		wireFrameOff();

		for (int i = 0; i < MM.n_v; i++)
		{
			vec4 clr = getColour(MM.scalars[i], dMin, dMax);
			glColor3f(clr.r, clr.g, clr.b);
			drawPoint(MM.positions[i]);
		}

	}

};

