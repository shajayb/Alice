#ifndef _GRAPH_
#define _GRAPH_


#include "nvec.h"
//#include "matrices.h"
#define _use_vector_
//#define _push_back_

class Graph
{

public:
	// ------------- topology 
	


#ifdef _use_vector_

	vector<Vertex>vertices;
	vector<Edge>edges;
	int n_e, n_v;

	// ------------- attributes ;

	vector<vec>positions;



	// ------------- ------------- ------------- -------------------------- CONSTRUCTORS  ;
	Graph()
	{
		n_e = n_v = 0;
		
		vertices.reserve(MAX_VERTS);
		edges.reserve(MAX_EDGES);
		positions.reserve(MAX_VERTS);
	}

#else


	Vertex *vertices;
	Edge *edges;
	int n_e, n_v;

	// ------------- attributes ;
	vec * positions;

	// ------------- ------------- ------------- -------------------------- CONSTRUCTORS  ;
	Graph()
	{
		n_e = n_v = 0;
		vertices = new Vertex[MAX_VERTS];
		edges = new Edge[MAX_EDGES];
		positions = new vec[MAX_VERTS];
	}

#endif // !_use vector_

	// ------------- ------------- ------------- -------------------------- TOPOLOGY
#ifdef _push_back_


	// VERTEX
	Vertex * createVertex(vec &p)
	{
		vertices.push_back(Vertex(vertices.size()));
		positions.push_back(p);
		n_v = vertices.size() -1;

		return &vertices[n_v];
	}

	// EDGE
	Edge * createEdge(Vertex &str, Vertex &end)
	{
		edges.push_back( Edge(str, end, edges.size()) );
		n_e = edges.size() -1;
		str.addEdge(&edges[n_e]);
		end.addEdge(&edges[n_e]);
		
		return &edges[n_e];
	}

#else 

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

#endif // !_push_back_


	// ------------- ------------- ------------- -------------------------- UTILITIES - TOPOLOGY
	
	void reset()
	{

		n_v = n_e = 0;
		
#ifdef _use_vector_

		vertices.clear();
		edges.clear();
		positions.clear();

#endif // _use_vector_



	}
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

	generator<Edge> getNextEdge(Edge &e)
	{
		//cout << " getNextEdge called on " << e.id << endl;
		for (int i = 0; i < e.vEnd->n_e; i++)
			if (e.vEnd->edgePtrs[i] != &e)yield *(e.vEnd->edgePtrs[i]);

		for (int i = 0; i < e.vStr->n_e; i++)
			if (e.vStr->edgePtrs[i] != &e)yield *(e.vStr->edgePtrs[i]);

	}



	array<bool, MAX_EDGES> eChecked;
	// generator<Edge>  !! IMP : co-routine functions cannot be recursive. !
	void recurseEdges(Edge &startEdge, int &sum)
	{

		//startEdge.draw(&positions, 6.0);
		glLineWidth(1.0);
		int cnt = 0;;

		vector<Edge> con_edges;
		for (auto e : getNextEdge(startEdge))
			if (!eChecked[e.id])
			{
				//e.draw(positions, 2.0);
				con_edges.push_back(e);
			}

		eChecked[startEdge.id] = true;
		sum += con_edges.size();
		//for (auto e : con_edges) cout << e.id << ",";
		//cout << endl;

		for (auto e : con_edges)
			if (!eChecked[e.id])recurseEdges(e, sum);
	}

	void drawIslands()
	{
		for (auto &c : eChecked)c = false;

		for (int i = 0; i < n_e; i++)
			if (!eChecked[i])
			{
				int sum = 0;
				//vec4 clr = clrs[i];;
				//glColor3f(clr.r, clr.g, clr.b);
				recurseEdges(edges[i], sum);
			}

	}

	array<bool, MAX_VERTS>vChecked;
	void subGraphs()
	{
		for (int i = 0; i < n_v; i++);
		
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

	void draw_edge( Edge &e )
	{

	}

	void draw()
	{
		glPointSize(5);
		for (int i = 0; i < n_v; i++)
		{
			char s[200];
			itoa(i, s, 10);
			//drawString(s, positions[i]+ vec(0,0,.1));
			drawPoint(positions[i]);
		}
		glPointSize(1);
		for (int i = 0; i < n_e; i++)
		{
			char s[200];
			itoa(i, s, 10);
			//drawString(s, (positions[edges[i].vStr->id] + positions[edges[i].vEnd->id]) * 0.5 + vec(0, 0, .1));
			drawLine(positions[edges[i].vStr->id], positions[edges[i].vEnd->id]);
		}

		//for (auto &v : vertices) drawPoint(positions[v.id]);
		//for (auto &e : edges) drawLine(positions[e.vEnd->id], positions[e.vStr->id]);
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