
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

#include "graph.h"
#include "metaMesh.h"

class graphStack
{
public:

	Graph PrintStack[25];
	Graph G;
	vec minV, maxV;
	double dMin, dMax;
	double threshold;
	int currentStackLayer;
	metaMesh MM;
	bool convertedTocyclic;
	////---------------------------------------------------- CONSTRUCTOR --------------------------------------
	graphStack( )
	{
		//---------
		//PrintStack.reserve(25);
		//PrintStack = * new Graph[25]
	}

	////---------------------------------------------------- COMPUTE  --------------------------------------
	void readGraphAndCreateDataMesh(string fileToImport, float scale = 1.0)
	{
		//---------
		importer imp = *new importer(fileToImport, 10000, 1.0);
		imp.readEdges();

		//---------
		G.reset();
		for (int i = 0; i < imp.nCnt; i++)G.createVertex(imp.nodes[i].pos);
		for (int i = 0; i < imp.eCnt; i++)G.createEdge(G.vertices[imp.edges[i].n0], G.vertices[imp.edges[i].n1]);
		
		//
		G.boundingbox(minV, maxV);

		Matrix4 trans;
		double preferedDiag = 50;
		trans.scale( preferedDiag / (minV.distanceTo(maxV)) );
		trans.translate((minV + maxV) * 0.5);
		for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];
		cout << " ACUTAL DIAG " << minV.distanceTo(maxV) << endl;
		G.boundingbox(minV, maxV);

		trans.identity();
		trans.translate( vec(40,0,0) - (minV + maxV) * 0.5);
		for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];
		minV = minV * trans;
		maxV = maxV * trans;

		G.boundingbox(minV, maxV);
		minV -= (maxV - minV).normalise() * 1.5;
		maxV += (maxV - minV).normalise() * 1.5;

		//---------

		createDataMeshGrid(G);
		createIsoContourGraph(0.1);

		//
		currentStackLayer = 0;
		convertedTocyclic = false; 
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
	void convertContourToCyclicGraph()
	{
		MM.convertContourToCyclicGraph();
		convertedTocyclic = true;
	}

	void smoothCurrentGraph()
	{
		MM.G.smooth_connectedVertices();
	}

	////---------------------------------------------------- UTILITIES  --------------------------------------

	void reducePointsOnContourGraph( int inc = 3)
	{
		Graph A;
		for (int i = 0; i < MM.G.n_v; i += inc)
			A.createVertex( MM.G.positions[i]);

		for (int i = 0; i < A.n_v; i += 1)
			A.createEdge( A.vertices[i], A.vertices[ A.Mod(i + 1, A.n_v) ]);

		MM.G.reset();

		for (int i = 0; i < A.n_v; i += 1)
			MM.G.createVertex(A.positions[i]);

		for (int i = 0; i < A.n_v; i += 1)
			MM.G.createEdge( MM.G.vertices[ A.edges[i].vStr->id], MM.G.vertices[A.edges[i].vEnd->id]);
	}

	void addCurrentContourGraphToPrintStack( float layersize = 0.1 , float baseOffset = 0.4)
	{
		//---------
		for (int i = 0; i < MM.G.n_v; i++)
			MM.G.positions[i].z += currentStackLayer * layersize + baseOffset;

		PrintStack[currentStackLayer] = *new Graph();
		PrintStack[currentStackLayer].reset();
		
		for (int i = 0; i < MM.G.n_v; i++)
			PrintStack[currentStackLayer].createVertex(MM.G.positions[i]);

		for (int i = 0; i < MM.G.n_e; i++)
			PrintStack[currentStackLayer].createEdge( PrintStack[currentStackLayer].vertices[ MM.G.edges[i].vStr->id ], PrintStack[currentStackLayer].vertices[MM.G.edges[i].vEnd->id]);

		currentStackLayer++;
		if (currentStackLayer >= 25)currentStackLayer = 0;
	}

	void ConvertContourStackToPrintPath(pathImporter &path)
	{
		path.actualPathLength = 0;
		for (int i = 0; i < currentStackLayer; i++)
			for (int j = 0; j < PrintStack[i].n_v; j++)
				path.addPoint( PrintStack[i].positions[j] );
	}

	void writeCurrentGraph()
	{
		MM.G.writeGraph(1.0);
	}

	////---------------------------------------------------- DISPLAY  --------------------------------------

	void draw()
	{
		

		
		wireFrameOn();

				////MM.draw();
		G.draw();

			MM.G.computeIslandsAsEdgeAndVertexList();
			MM.G.drawConnectedEdgeList();
			
			convertedTocyclic ? glColor3f(1, 0, 0) : glColor3f(0, 0, 0);
			MM.G.draw();

		wireFrameOff();

		//----------------- drawDataGridMesh
		for (int i = 0; i < MM.n_v; i++)
		{
			vec4 clr = getColour(MM.scalars[i], dMin, dMax);
			glColor3f(clr.r, clr.g, clr.b);
			drawPoint(MM.positions[i]);
		}

		//----------------- drawDataGridMesh
		glColor3f(1, 0, 0);
		for (int i = 0; i < currentStackLayer; i++)PrintStack[i].draw();

		//draw stats
		char s[200];
		sprintf_s(s, " num points in contour : %i", MM.G.n_v);
		setup2d();
			drawString(s, vec(winW * 0.5, 50, 0));
			sprintf_s(s, " num points in stack : %i", MM.G.n_v * currentStackLayer);
			drawString(s, vec(winW * 0.5, 75, 0));
		restore3d();
	}

};

