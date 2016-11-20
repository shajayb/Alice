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


#ifndef _META_MESH_

#define  _META_MESH_



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

	metaMesh createFromPlane( vec minV,vec maxV , int divs)
	{
		Mesh M;
		int rowCnt = 0;
		int colCnt = 0;
		//int divs = 100;
		/*for (float x = minV.x; x <= maxV.x; x += (maxV.x - minV.x)*0.01)*/
		for (int i = 0; i < divs; i++)
		{
			float x = minV.x + (maxV.x - minV.x) / float(divs) * float(i);
			rowCnt = 0;
			for (int j = 0; j < divs; j++)
			{
				float y = minV.y + (maxV.y - minV.y) / float(divs) * float(j);
				M.createVertex(vec(x, y, 0));
				rowCnt++;
			}
			colCnt++;
		}


		Vertex *fVerts[4];
		Vertex *fv[3];

		for (int i = 0; i < rowCnt - 1; i++)
		{
			for (int j = 0; j < colCnt; j++)
			{
				if (j == 0)continue;
				fVerts[0] = &M.vertices[i*colCnt + j];
				fVerts[1] = &M.vertices[(i + 1)*colCnt + j];
				fVerts[2] = &M.vertices[(i + 1)*colCnt + j - 1];
				fVerts[3] = &M.vertices[i*colCnt + j - 1];
				M.createNGon(fVerts, 4, true);

				/*	fv[0] = &M.vertices[i*colCnt + j];;
				fv[1] = &M.vertices[i*colCnt + j-1];;
				fv[2] = &M.vertices[(i+1)*colCnt + j];;
				M.createFace(fv, 3);


				fv[0] = &M.vertices[(i )*colCnt + j-1];
				fv[1] = &M.vertices[(i+1)*colCnt + j - 1];;
				fv[2] = &M.vertices[(i + 1)*colCnt + j];;
				M.createFace(fv, 3);*/
			}
		}


		for (int i = 0; i < M.n_f; i++)M.faces[i].faceVertices();

		return metaMesh(M);
	}

	void assignScalars(string component = "y")
	{
		for (int i = 0; i < n_v; i++)scalars[i] = positions[i].z;// vertices[i].getMeanCurvatureGradient(positions).mag(); // 
																 //(&m.positions[i])* DEG_TO_RAD ; //// m.positions[i].y; // distanceTo(vec(0, 0, 0));

	}

	double distanceAndNearestPointOnEdge(vec &a, vec&b, vec &p, vec&pt)
	{
		vec n = (b - a).cross(vec(0, 0, 1));
		n.normalise();
		pt = n * ((a - p)*n);
		pt += p;


		float len = (a - b).mag();

		vec ed = (a - b) / len;
		double param = (pt - b) * ed;

		param = ofClamp(param, 0, len);
		pt = b + ed * param;

		return (p - pt) * (p - pt);// p.distanceTo(pt);
	}

	void assignScalarsAsLineDistanceField( Graph &G)
	{

		for (int i = 0; i < n_v; i++)
		{

			vec pt;
			double d = 1e10;
			double dChk = 0.0;

			for (int j = 0; j < G.n_e; j++)
			{
				int e0, e1;
				e0 = G.edges[j].vEnd->id;
				e1 = G.edges[j].vStr->id;
				float Di = distanceAndNearestPointOnEdge(G.positions[e0], G.positions[e1], positions[i], pt);
				//dChk = Di;
				//dChk -= 1.0;
				dChk += 1.0 / (pow((Di + 0.01), 2.0));

				//d= MIN(dChk, d);
			}

			d = dChk;
			scalars[i] = d;
			//dMin = MIN(dMin, d);
			//dMax = MAX(dMax, d);
		}
	}

	void getMinMaxOfScalarField( double &dMin, double &dMax )
	{
		dMin = 1e10;
		dMax = dMin * -1;
		
		for (int i = 0; i < n_v; i++)
		{
			dMin = MIN(dMin, scalars[i]);
			dMax = MAX(dMax, scalars[i]);
		}
		
	}

	void createIsoContourGraph(double threshold)
	{
		int a, b;
		vec diff;
		double interp;
		Vertex v;
		int *edge_vertex_ids = new int[n_e];

		G.reset();
		int eCnt = 0;
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
				eCnt++;
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

		cout << eCnt << " -- metaMesh Graph " << G.n_v << endl;
	}

};


#endif // !_META_MESH_

