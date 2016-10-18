#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;


struct E
{
	vec t, f;

	E() {}
	E( vec &_t,vec &_f)
	{
		t = _t;
		f = _f;
	}
};



class metaMesh : public Mesh
{
public:

	array<double, 1000> scalars;
	metaMesh() {};
	metaMesh( Mesh &in)
	{

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
};

//
generator<E> faceEdges( metaMesh &m , double threshold)
{
	int a, b;
	vec ePt[3];
	bool e[3];
	vec diff;
	double interp;

	for (int i = 0; i < m.n_f; i++)
	{
		
		for (int j = 0; j < 3 /*m.faces[i].n_e*/; j++)
		{
			a = m.faces[i].edgePtrs[j]->vEnd->id;
			b = m.faces[i].edgePtrs[j]->vStr->id;
		
			diff = (m.positions[b] - m.positions[a]);// .normalise();
			interp = ofMap(threshold, m.scalars[a], m.scalars[b], 0, 1);

			ePt[j] = (interp >= 0.0 && interp <= 1.0) ?  (m.positions[a] + diff * interp) :  ( vec(0, 0, 0) );
			e[j] = (interp >= 0.0 && interp <= 1.0) ? 1 : 0;
		}

		
		if( e[0] && e[1])yield E(ePt[0], ePt[1]);
		if (e[1] && e[2])yield E(ePt[1], ePt[2]);
		if (e[2] && e[0])yield E(ePt[2], ePt[0]);
	
	}
}


SliderGroup S;
double threshold = 0.5;
metaMesh M;

void assignScalars(metaMesh &m , string component = "y")
{
	for (int i = 0; i < M.n_v; i++)m.scalars[i] = m.vertices[i].getMeanCurvatureGradient(m.positions).mag();
		//(&m.positions[i])* DEG_TO_RAD ; //// m.positions[i].y; // distanceTo(vec(0, 0, 0));

}

void setup()
{
	S = *new SliderGroup();
	S.addSlider(&threshold, "threshold");
	S.sliders[0].attachToVariable(&threshold, -PI, PI);

	MeshFactory fac;
	Mesh tmp = fac.createFromOBJ("data/in.obj", 10, false, false);
	M = metaMesh( tmp );
	assignScalars(M);

}

void update(int value)
{


}

void draw()
{

	backGround(0.75);

	M.draw(true);

	glPointSize(5);
	
	for (double i = 0; i < threshold; i+= threshold * 0.05)
		for (auto &c : faceEdges(M, i))
		{
			drawLine(c.t, c.f);
			//drawPoint((vec)c.t);
			//drawPoint((vec)c.f);
		
		}


	S.draw();

}
void keyPress(unsigned char k, int xm, int ym)
{


}



void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		//		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{
	//if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
	}
}




