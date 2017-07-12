// DESCRIPTION:
//      Winged edge mesh 
//

#pragma once
#include <zVector\zVector.h>
#include <zGraph\zGraph.h>
#include <vector>
#include <algorithm>    // std::sort




#define MAX_POLYCONNECTS 100000

namespace zSpace
{
	//
	// CLASS MESH

	struct zCurvature
	{
		double k1;
		double k2;
			
	};

	class zMesh
	{
	private:
		int n_v; // number of vertices
		int n_e; // number of edges
		int n_f; // number of faces
				
	public:
		zVertex vertices[MAX_VERTS];	   /* vertex list			*/
		zEdge edges[MAX_EDGES];		  /* edge list				*/
		zFace faces[MAX_FACES];		 /* face list				*/
		//vector<zVector> positions;	/* vertex positions			*/

		zVector minBB, maxBB;
		
		//---- CONSTRUCTOR

		// default constructor
		zMesh();

		// over loaded constructor
		//zMesh(int _numVertices, int _numEdges, int _numFaces, zVertex *_vertices, zEdge *_edges, zFace *_faces, zVector *_positions);

		// over loaded constructor create from arrays
		//zMesh(int &_num_vertices, int &_num_edges, int &_num_polygons, int &_num_polyconnects, vector<zVector>(&_positions), vector<int>(&_polyCounts), vector<int>(&_polyConnects));
		
		zMesh(int &_num_vertices, int &_num_edges, int &_num_polygons, int &num_polyconnects, zVector(&_positions)[MAX_VERTS], int(&polyCounts)[MAX_FACES], int(&polyConnects)[MAX_POLYCONNECTS]);

		//---- DESTRUCTOR

		// default destructor
		~zMesh();

		//---- METHODS

		// create a copy
		zMesh meshCopy();

		// print Info
		void printMeshInfo();

		//compute poly-connects and poly-counts
		void computePolyConnects_PolyCount(int (&polyConnects)[MAX_POLYCONNECTS], int (&polyCounts)[MAX_FACES], int &n_PC);

		//get mesh triangles
		void getTriangles(int(&polyConnects_tris)[MAX_POLYCONNECTS], int(&tris_perPolygon)[MAX_FACES], int &n_PC);
		
		//mesh triangulate
		zMesh meshTriangulate();

		//calculate bounding box
		void calculateBoundingBox();
				
		
		//---- VERTEX QUERIES

		// number of vertices
		int numVertices();

		//get connected edges to input vertex
		vector<zEdge> getConnectedEdgestoVertex(int vertId);

		//get connected vertices to input vertex
		vector<zVertex> getConnectedVerticestoVertex(int vertId);

		//get connected faces to input vertex
		vector<zFace> getConnectedFacestoVertex(int vertId);

		//get vertex Angle in the input face
		double getVertexAngle(zVertex &v, zFace &f);

		// vertex on boundary
		bool onBoundaryVertex(int vertId);

		// calculate principal curvature
		vector<zCurvature> calculatePrincipalCurvature();

		//---- EDGE QUERIES

		// number of edges
		int numEdges();

		// edge on boundary
		bool onBoundaryEdge(int edgeId);

		//get connected edges to input edge
		vector<zEdge> getConnectedEdgestoEdge(int edgeId);

		//---- FACE QUERIES

		// number of polygons
		int numPolygons();
		
		//get edges of input face
		vector<zEdge> getFaceEdges(int faceId);

		//get vertices of input face
		vector<zVertex> getFaceVertices(int faceId);

		//---- DISPLAY METHODS

		// Draw Vertex
		void drawVertex(zVertex &v1, zColor &col);

		// Draw Edge
		void drawEdge(zVertex &v0, zVertex &v1, zColor &edgeCol, bool borderEdge);

		// Draw Polygon
		void drawPolygon(vector<zVertex> &v1, zColor &faceCol);

		//DrawMesh
		void drawMesh(zColor &vertCol, zColor &edgeCol, zColor &faceCol, bool showVerts, bool showEdges, bool showFaces);
		
	};
}
