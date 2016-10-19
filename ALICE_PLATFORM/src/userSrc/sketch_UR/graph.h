#ifndef _GRAPH_
#define _GRAPH_


#include "nvec.h"
//#include "matrices.h"

class Graph
{

public:
	// ------------- topology 

	Vertex *vertices;
	Edge *edges;
	int n_e, n_v;

	// ------------- attributes ;

	vec * positions;

	Vertex *v_v[MAX_VALENCE];



	// ------------- ------------- ------------- -------------------------- CONSTRUCTORS  ;
	Graph()
	{
		n_e = n_v = 0;
		vertices = new Vertex[MAX_VERTS];
		edges = new Edge[MAX_EDGES];
		positions = new vec[MAX_VERTS];
	}


	// ------------- ------------- ------------- -------------------------- TOPOLOGY
	// VERTEX
	Vertex * createVertex(vec &p)
	{

		vertices[n_v] = Vertex(n_v);
		positions[n_v] = p;
		n_v++;

		return &vertices[n_v - 1];
	}

	// EDGE
	Edge * createEdge(Vertex &str, Vertex &end)
	{
		edges[n_e] = Edge(str, end, n_e);
		str.addEdge(&edges[n_e]);
		end.addEdge(&edges[n_e]);

		n_e++;
		return &edges[n_e - 1];
	}

	//FACE 		


	// ------------- ------------- ------------- -------------------------- UTILITIES - TOPOLOGY
	Edge * edgeExists(Vertex &str, Vertex &end, bool &found)
	{

		for (int i = 0; i < str.n_e; i++)
		{
			found = true;
			Edge e = *(str.edgePtrs[i]);
			//cout << &(str.edges[i]) <<  " " <<  &(str.edges[i].vStr) << " --- c -- " <<  &end <<  " " << &str << endl ;
			if (e.vStr == &end || e.vEnd == &end)return str.edgePtrs[i];

		}

		//printf( " none found , creating new %i %i \n " , str.id , end.id ) ;
		found = false;
		return createEdge(str, end);
	}


	// ------------- ------------- ------------- -------------------------- UTILITIES
	void writeGraph(float scaleBack, string outFileName = "data/graph.txt")
	{
		printf(" ----------- writing \n ");

		ofstream myfile;
		myfile.open(outFileName.c_str(), ios::out);


		if (myfile.fail())
		{
			myfile << " error in opening file  " << outFileName.c_str() << endl;
			return;
		}

		// vertices
		for (int i = 0; i < n_v; i++)
		{

			char s[200];
			sprintf(s, "%1.4f,%1.4f,%1.4f,%i", positions[i].x * scaleBack, positions[i].y * scaleBack, positions[i].z * scaleBack, (vertices[i].n_e == 1) ? 1 : 0);

			myfile << s << endl;
		}

		for (int i = 0; i < n_e; i++)
		{

			char s[200];
			sprintf(s, "%i,%i", edges[i].vStr->id, edges[i].vEnd->id);

			myfile << s << endl;
		}


		myfile.close();
		return;

	}


	// ------------- ------------- ------------- -------------------------- DISPLAY

	//int getVertexEdges(int id, vec &normal, int *v_vertIds)
	//{
	//	vec *pos = new vec[vertices[id].n_e];

	//	vec eVecs[3];
	//	vec eVals, mean;

	//	// get vertices
	//	int N = vertices[id].getVertices(v_v);
	//	// get vertex positions
	//	for (int i = 0; i < N; i++)pos[i] = positions[v_v[i]->id];

	//	//get PCA plane
	//	Matrix::PCA(pos, N, mean, eVals, eVecs);
	//	eVecs[0].normalise();
	//	vertices[id].norm = normal = eVecs[0];
	//	// project points to PCA plane 
	//	for (int i = 0; i < N; i++)
	//	{
	//		pos[i] -= eVecs[0] * ((pos[i] - positions[id])*eVecs[0]);
	//		drawPoint(pos[i]);;
	//	}
	//	//

	//	// ------------ sort vec
	//	vec firstEdge = pos[0] - positions[id];
	//	list<float> angles;
	//	map< float, int > angle_E_Map;

	//	// ---- calc angles
	//	for (int i = 0; i < N; i++)
	//	{
	//		vec ed = (pos[i] - positions[id]);
	//		float ang = vec::angle_2D(ed.x, ed.y, firstEdge.x, firstEdge.y);
	//		angles.push_back(ang);
	//		angle_E_Map[ang] = i;
	//	}

	//	// ---- sort angles , get corresponding edge / v_id 
	//	list<float>::iterator it_f;
	//	int cnt = 0;
	//	angles.sort();


	//	for (it_f = angles.begin(); it_f != angles.end(); it_f++)
	//	{
	//		int v_id = angle_E_Map[*it_f];
	//		v_vertIds[cnt] = v_v[v_id]->id;
	//		cnt++;
	//	}


	//	delete[]pos;
	//	return N; // eVecs[0] ;

	//}

	//int cyclicSortPositions(vec*pos, int N, vec &normal, int *v_vertIds, vec cen = vec(0, 0, 0))
	//{
	//	vec eVecs[3];
	//	vec eVals, mean;

	//	Matrix::PCA(pos, N, mean, eVals, eVecs);
	//	eVecs[0].normalise();
	//	normal = eVecs[0];


	//	// calc. centroid if not provided.
	//	if (cen == vec(0, 0, 0))
	//	{
	//		for (int i = 0; i < N; i++)cen += pos[i];
	//		cen /= N;
	//	}
	//	if (eVals.x > 0.2)
	//	{
	//		normal = vec(0, 0, 0);
	//		for (int i = 0; i < N; i++)normal += pos[i] - cen;
	//		normal /= N;
	//	}

	//	normal.normalise();

	//	// project points to best fit plane 
	//	for (int i = 0; i < N; i++)pos[i] -= normal*((pos[i] - cen)*normal);



	//	// ------------ sort vec

	//	vec firstEdge = pos[0] - cen;
	//	firstEdge.normalise();
	//	list<float> angles;
	//	map< float, int > angle_E_Map;

	//	// ---- calc angles
	//	for (int i = 0; i < N; i++)
	//	{
	//		vec ed = (pos[i] - cen);
	//		ed.normalise();
	//		float ang = vec::angle_2D(ed.x, ed.y, firstEdge.x, firstEdge.y);
	//		if ( /*fabs(fmod (double(ang),double(PI))*/fabs(ang) < 10e-02)ang = ofRandom(-0.01, 0.01);
	//		angles.push_back(ang);
	//		angle_E_Map[ang] = i;
	//	}
	//	// ---- sort angles , get corresponding edge / v_id 
	//	list<float>::iterator it_f;
	//	list<float>::reverse_iterator it_f_rev;
	//	int cnt = 0;
	//	angles.sort();


	//	vec bi = firstEdge.cross(normal);
	//	vec ed = (pos[1] - cen);
	//	if (bi*ed > 0)
	//	{
	//		for (it_f_rev = angles.rbegin(); it_f_rev != angles.rend(); it_f_rev++)
	//		{
	//			int v_id = angle_E_Map[*it_f_rev];
	//			v_vertIds[cnt] = v_id; //v_v[v_id]->id ;
	//			cnt++;
	//		}
	//	}
	//	else
	//	{
	//		for (it_f = angles.begin(); it_f != angles.end(); it_f++)
	//		{
	//			int v_id = angle_E_Map[*it_f];
	//			v_vertIds[cnt] = v_id; //v_v[v_id]->id ;
	//			cnt++;
	//		}
	//	}


	//}


	void draw()
	{
		glPointSize(5);
		for (int i = 0; i < n_v; i++)drawPoint(positions[i]);
		glPointSize(1);
		for (int i = 0; i < n_e; i++)drawLine(positions[edges[i].vStr->id], positions[edges[i].vEnd->id]);
	}


	void draw( nvec &dists, vector<vec> &nPts)
	{
		vec4 clr;
		vec pt;
		float minD,maxD;
		minD = dists.min();
		maxD = dists.max();

		glEnable(GL_LINE_SMOOTH);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


		for (int i = 0; i < n_e; i++)
		{
			float d = 4;
			glLineWidth(d);
			glBegin(GL_LINES);
			glLineWidth(ofMap(dists.x[edges[i].vStr->id], minD, maxD, 0, 2));

				clr = getColour(dists.x[edges[i].vStr->id], minD, maxD);

				pt = positions[edges[i].vStr->id];
				glColor3f(clr.r, clr.g, clr.b);
				glVertex3f(pt.x, pt.y, pt.z);

				clr = getColour(dists.x[edges[i].vEnd->id], minD, maxD);
				pt = positions[edges[i].vEnd->id];
				glColor3f(clr.r, clr.g, clr.b);
				glVertex3f(pt.x, pt.y, pt.z);
			
			glEnd();


		}


		glDisable(GL_BLEND);

		//////////////////////////////////////////////////////////////////////////
		glLineWidth(1);
		lineStyle(5);
			for (int i = 0; i < n_v; i++)drawLine(positions[i], nPts[i]);

		glDisable(GL_LINE_STIPPLE);

	
	}











};


#endif