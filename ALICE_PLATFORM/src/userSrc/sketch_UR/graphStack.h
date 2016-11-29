
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

#include "graph.h"
#include "ActiveGraph.h"
#include "metaMesh.h"

class graphStack
{
public:

	activeGraph PrintStack[25];
	Graph G;
	vec minV, maxV;
	double dMin, dMax;
	double threshold;
	int currentStackLayer;
	metaMesh MM;
	bool convertedToToroidal;
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

		// ------------ scale to fit 
		G.boundingbox(minV, maxV);

		Matrix4 trans;
		double preferedDiag = 50;
		trans.scale( preferedDiag / (minV.distanceTo(maxV)) );
		trans.translate((minV + maxV) * 0.5);
		for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];
		cout << " ACUTAL DIAG " << minV.distanceTo(maxV) << endl;
		G.boundingbox(minV, maxV);

		trans.identity();
		trans.translate( vec(45,0,0) - (minV + maxV) * 0.5);
		for (int i = 0; i < G.n_v; i++) G.positions[i] = trans * G.positions[i];
		minV = minV * trans;
		maxV = maxV * trans;

		G.boundingbox(minV, maxV);
		minV -= (maxV - minV).normalise() * 2.5;
		maxV += (maxV - minV).normalise() * 2.5;

		//--------- dateGrid

		createDataMeshGrid(G);
		//createIsoContourGraph(0.1);

		//
		currentStackLayer = 0;
		convertedToToroidal = false; 
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
	void convertContourToToroidalGraph()
	{
		MM.convertContourToToroidalGraph();
		convertedToToroidal = true;
	}

	void smoothCurrentGraph()
	{
		MM.G.smooth_connectedVertices();
	}

	////---------------------------------------------------- UTILITIES  --------------------------------------

	void reducePointsOnContourGraph( int inc = 3)
	{
		toroidalGraph A;
		A.constructFromGraph(MM.G , inc);

		MM.G.reset();
		MM.G.constructFromGraph(A);

	}

	void addCurrentContourGraphToPrintStack( float layersize = 0.1 , float baseOffset = 1.0 )
	{
		//---------
		for (int i = 0; i < MM.G.n_v; i++)
			MM.G.positions[i].z = currentStackLayer * layersize + baseOffset;

		//---------
		activeGraph AG;
		AG = * new activeGraph();
		AG.reset();
		AG.constructFromGraph(MM.G);
		//AG.fixEnds();
		AG.populateRigidBodies(0.1);
		
		PrintStack[currentStackLayer] = AG;

		///
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

	void draw( bool showData = false )
	{
		
		
		wireFrameOn();

			/// ------------------------- TODO : clean this section -> encapsulate

			G.draw();

			MM.G.computeIslandsAsEdgeAndVertexList();
			MM.G.drawConnectedEdgeList();
			
			convertedToToroidal ? glColor3f(1, 0, 0) : glColor3f(0, 0, 0);
			MM.G.draw();

			/// ------------------------- TODO : clean this section

		wireFrameOff();

		//----------------- drawDataGridMesh
		if(showData)
		for (int i = 0; i < MM.n_v; i++)
		{
			vec4 clr = getColour(MM.scalars[i], dMin, dMax);
			glColor3f(clr.r, clr.g, clr.b);
			drawPoint(MM.positions[i]);
		}

		//----------------- drawDataGridMesh

		glColor3f(1, 0, 0);
		for (int i = 0; i < currentStackLayer; i++)PrintStack[i].display();// draw();

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

