#ifndef _ACTIVE_GRAPH_
#define _ACTIVE_GRAPH_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graph.h"
#include "newPhysics.h";
#include "rigidCube.h"



class activeGraph : public toroidalGraph
{
public:

	rigidCube RC;
	vector<rigidCube> RC_curve;

	activeGraph()
	{
		

	}

	void constructFromToroidalGraph(toroidalGraph &TG)
	{
		reset();
		for (int i = 0; i < TG.n_v; i++) createVertex(TG.positions[i]);

		for (int i = 0; i < n_v; i++) createEdge(vertices[Mod(i, n_v)], vertices[Mod(i + 1, n_v)]);
		fixed.assign(n_v, false);
		
	}

	void populateRigidBodies( float depth =0.25, float ht = 0.25 )
	{
		RC_curve.clear();
		vec S, E, cen, XA, YA, ZA;
		ZA = vec(0, 0, 1);

		MeshFactory fac;
		Mesh M = fac.createPlatonic(1.0 / sqrt(2.0), 6);

		for (int i = 0; i < n_e; i++)
		{
			if (fixed[edges[i].vStr->id] && fixed[edges[i].vEnd->id]) continue;

			S = positions[ edges[i].vStr->id ];
			E = positions[ edges[i].vEnd->id ];
			cen = (S + E)*0.5;
			XA = (E - S).normalise();
			YA = ZA.cross(XA).normalise();

			float Scale[3];
			Scale[0] = S.distanceTo(E)* 1.0;
			Scale[1] = depth; Scale[2] = ht ;

			//RC = *new rigidCube(M);
			RC.setTransformation(XA, YA, ZA, cen);
			RC.setScale(Scale);
			RC.transform();
			RC.computeGrid();
				RC_curve.push_back(RC);
			RC.inverseTransform();
		}

		
	}

	void display()
	{

		//for (int i = 0; i < n_e; i++)
		//{
		//	if ( fixed[ edges[i].vStr->id ]  && fixed[ edges[i].vEnd->id ]) continue; 

		//	S = positions[ edges[i].vStr->id ];
		//	E = positions[edges[i].vEnd->id];
		//	cen = (S + E)*0.5;
		//	XA = (E - S).normalise();
		//	YA = ZA.cross(XA).normalise();

		//	RC.setOrientation(XA,YA,ZA,cen);
		//	float Scale[3];
		//	Scale[0] = S.distanceTo(E)* 1.0;
		//	Scale[1] = Scale[2] = 0.25;

		//	vec4 clr = getColour(i, 0, n_e);
		//	float l = 0.0; // clr.r * 0.57 + clr.g * 0.23 + clr.b * 0.11;
		//	RC.setScale(Scale);
		//	RC.draw(4, vec4(l,l,l,1.0) );

		//	drawLine(S, E);
		//	
		//	glPointSize(5); drawPoint(cen);
		//}
	
		//glPointSize(1);
		for (auto &rc : RC_curve) rc.draw();
	}

	void computeContacts()
	{
		//auto rc = RC_curve[0];

		//int RES = 10;
		////vec *P;
		//vec *P = new vec[RES*RES*RES];
		//rc.computeGrid(P, RES);
		//cout << " ------------------- " << P <<  endl;
		//for (int i = 0; i <RES*RES*RES; i++)P[i].print();
		//
		////P = NULL;
		////delete P;
	}
};




#endif // !_ACTIVE_GRAPH_
