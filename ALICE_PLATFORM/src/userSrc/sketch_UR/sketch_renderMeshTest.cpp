

#ifdef _MAIN_

#include "main.h"
#include "Matrix.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "largeMesh.h"



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
RenderMesh LM;
RenderMesh ground;
#define  rx ofRandom(-1,1)

void setup()
{
	Matrix4 T;
	T.identity();

	for ( int i = 0 ; i < 10 ; i ++)
	{
		T.identity();
		T.scale(fabs(rx)*0.5);
		T.translate( vec(rx, rx, fabs(rx)) * 20 );
		LM.addCube(T);
	}


	MeshFactory fac;
	Mesh M;
	M = fac.createFromOBJ("data/cube.obj", 10.0, false);
	//LM.addMesh(M);

	LM.createGroundPlane();
	//ground.addMesh(LM.ground);
	
	
	
}

void update(int value)
{
	
}

void draw()
{

	backGround(1.0);
	drawGrid(20);

	LM.draw();
	LM.draw(true);

	//LM.projectUntoGroundPlane();
	//wireFrameOn();
	ground.draw();
//	wireFrameOff();


	setup2d();
	
	glColor3f(0, 0, 0);
	double dmx, dmn;
	char s[200];
	LM.ground.getMinMaxOfScalarField(dmn, dmx);
	sprintf(s, " min %1.2f max %1.2f", dmn, dmx);
	drawString(s, winW*0.5, 50);

	restore3d();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{
	//LM.updateColorArray(10,false,vec(20,20,20));
	if ((GLUT_LEFT_BUTTON == b && GLUT_UP == state) || (GLUT_RIGHT_BUTTON == b && GLUT_UP == state))
	{
		int cur_msx = winW * 0.5;
		int cur_msy = winH * 0.5;
		vec camPt = screenToCamera(cur_msx, cur_msy, 0.2);

		if (glutGetModifiers() == GLUT_ACTIVE_ALT)
		LM.updateColorArray(10.0, false, camPt);
	}
}

void mouseMotion(int x, int y)
{
//	if ((GLUT_LEFT_BUTTON == b && GLUT_UP == state) || (GLUT_RIGHT_BUTTON == b && GLUT_UP == state))
	{
		int cur_msx = winW * 0.5;
		int cur_msy = winH * 0.5;
		vec camPt = screenToCamera(cur_msx, cur_msy, 0.2);

		if (glutGetModifiers() == GLUT_ACTIVE_ALT)
			LM.updateColorArray(10.0, false, camPt);
	}
}

void keyPress(unsigned char k, int xm, int ym)
{
	
	

	ground.n_v_tris = ground.n_f_tris = 0;
	ground.addMesh(LM.ground);

	vec minBB, maxBB;
	LM.ground.boundingBox(minBB, maxBB);
	minBB -= (maxBB - minBB).normalise() * 2.0;
	maxBB += (maxBB - minBB).normalise() * 2.0;

	for (int i = 0; i < ground.n_v_tris; i += 3)
	{
		int nv = i;// n_v;
		vec P;
		ground.getVectorFromArray(nv, P, true);
		vec projectedP = rayPlaneIntersection(P, vec(1, 1, 5) * -1, vec(0, 0, 1), 0);

		int vid = ground.getNearestVertexOnGroundPlane(projectedP, 100, minBB, maxBB);

		double l = 1.0 - LM.ground.scalars[vid];
		//if ( l > 0.9 )l = 0.75;
		
		
		
		vec4 clr(l, l, l, 1.0);
		ground.lm_colors_tris[nv + 0] = clr.r;
		ground.lm_colors_tris[nv + 1] = clr.g;
		ground.lm_colors_tris[nv + 2] = clr.b;// clr.z;
	}

	for (int i = 0; i < 1; i += 1)LM.ground.smoothData_laplace();

}





#endif // _MAIN_

