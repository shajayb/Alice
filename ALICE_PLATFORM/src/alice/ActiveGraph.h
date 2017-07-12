#ifndef _ACTIVE_GRAPH_
#define _ACTIVE_GRAPH_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graph.h"
#include "rigidCube.h"
#include "largeMesh.h"



class activeGraph : public toroidalGraph
{
public:

	rigidCube RC;
	vector<rigidCube> RCsOnCurve;


	activeGraph()
	{
		/*MeshFactory fac;
		Mesh M = fac.createPlatonic(1.0 / sqrt(2.0), 6);
		RC = *new rigidCube(M);*/
	}

	void constructFromToroidalGraph(toroidalGraph &TG)
	{
		reset();
		for (int i = 0; i < TG.n_v; i++) createVertex(TG.positions[i]);

		for (int i = 0; i < n_v; i++) createEdge(vertices[Mod(i, n_v)], vertices[Mod(i + 1, n_v)]);
		fixed.assign(n_v, false);

	}

	void populateRigidBodies(float depth = 0.25, float ht = 0.25)
	{
		RCsOnCurve.clear();
		vec S, E, cen, XA, YA, ZA;
		ZA = vec(0, 0, 1);

		MeshFactory fac;
		Mesh M = fac.createPlatonic(1.0 / sqrt(2), 6);

		for (int i = 0; i < n_e; i++)
		{
			if (fixed[edges[i].vStr->id] && fixed[edges[i].vEnd->id]) continue;

			S = positions[edges[i].vStr->id];
			E = positions[edges[i].vEnd->id];
			cen = (S + E)*0.5;
			XA = (E - S).normalise();
			YA = ZA.cross(XA).normalise();

			float Scale[3];
			Scale[0] = S.distanceTo(E)* 1.0;
			Scale[1] = depth; Scale[2] = ht;

			///
			RC = rigidCube(M);

			RC.setScale(Scale);
			RC.transform();

			Matrix4 T;
			T.setColumn(0, XA);
			T.setColumn(1, YA);
			T.setColumn(2, ZA);
			T.setColumn(3, cen);
			RC.setInitialTransformation(T);

			RCsOnCurve.push_back(RC);

		}


	}

	void populateRigidBodies(RenderMesh &LM , plane prevPl,float depth = 0.25, float ht = 0.25  )
	{

		vec S, E, cen, XA, YA, ZA;
		ZA = vec(0, 0, 1);



		for (int i = 0; i < n_e; i++)
		{
			if (fixed[edges[i].vStr->id] && fixed[edges[i].vEnd->id]) continue;

			S = positions[edges[i].vStr->id];
			E = positions[edges[i].vEnd->id];
			cen = (S + E)*0.5;
			XA = (E - S).normalise();
			YA = ZA.cross(XA).normalise();

			float Scale[3];
			Scale[0] = S.distanceTo(E)* 1.0;
			Scale[1] = depth; Scale[2] = ht;

	
			cen = cen - XA* Scale[0] * 0.25;;
			///
			Matrix4 T;
			T.setColumn(0, XA * Scale[0] * 0.5);
			T.setColumn(1, YA * Scale[1]* 0.5);
			T.setColumn(2, ZA* Scale[2]* 0.5);
			T.setColumn(3, cen);
			//T.setColumn(0, XA );
			//T.setColumn(1, YA );
			//T.setColumn(2, ZA );
			//T.setColumn(3, cen);
			//T.scale(Scale[0], Scale[1], Scale[2]);
			/*plane prevPl;
			prevPl.cen = cen - vec(0, 0, 1) * ht;
			prevPl.normal = vec(0, 0, 1);*/
			LM.addCube(T);

		}


	}



	void display()
	{
		for (auto &rc : RCsOnCurve) rc.draw();
	}

	void display(vec *P, int RES = 10)
	{

		for (auto &rc : RCsOnCurve) rc.draw();


		for (auto &rc : RCsOnCurve)
		{
			rc.computeGrid(P, RES);
			rc.drawGridAsPoints(P, RES*RES*RES);
		}


	}

	void computeContacts()
	{

		//cout << " ------------------- " << P <<  endl;
		//for (int i = 0; i <RES*RES*RES; i++)P[i].print();
		//
		////P = NULL;
		////delete P;
	}
};




#endif // !_ACTIVE_GRAPH_
