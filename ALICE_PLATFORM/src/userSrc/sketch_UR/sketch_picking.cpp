#define _MAIN_

#ifdef _MAIN_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;

// model - view - controller (MVC) paradigm / pattern / template 
void drawVector(vec&a, vec loc, string suffix)
{
	setup2d();

	char s[200];
	sprintf(s, "%1.2f,%1.2f,%1.6f : ", a.x, a.y, a.z);
	string str = s;
	str += suffix;
	drawString(str, loc);

	restore3d();
}
void drawMatrix(Matrix4 &T, vec str)
{

	char s[200];
	glColor3f(0, 0, 0);
	setup2d();

	double width = 4 * 20;
	double ht = 24;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			sprintf(s, "%1.2f", T[j * 4 + i]);
			drawString(s, i * width + str.x, j * ht + str.y);
		}

	restore3d();

}

vec rayPlaneIntersection(vec P0, vec ray, vec N, float d = 0)
{
	double t = -(P0 * N + d) / (ray * N);
	return P0 + ray * t;
}

vec screenToWorld(int x, int y, double zPlane = 0)
{
	double camera_pos[3];
	GLdouble matModelView[16], matProjection[16];
	int viewport[4];
	// get matrixs and viewport:
	glGetDoublev(GL_MODELVIEW_MATRIX, matModelView);
	glGetDoublev(GL_PROJECTION_MATRIX, matProjection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	int scrCenX = (viewport[2] - viewport[0]) / 2;
	int scrCenY = (viewport[3] - viewport[1]) / 2;
	gluUnProject
	(
		scrCenX + x, scrCenY + y, zPlane, //screen coords
		matModelView, matProjection, viewport, //mvp matrices
		&camera_pos[0], &camera_pos[1], &camera_pos[2] // return pos
	);

	return vec(camera_pos[0], camera_pos[1], camera_pos[2]);
}

/*Matrix4*/vec worldToScreen( vec &a , Matrix4 &MV, Matrix4 &P, int *viewport )
{

	int x, y, w, h;
	x = viewport[0]; y = viewport[1];
	w = viewport[2]; h = viewport[3];

	Vector4 inPt(a.x, a.y, a.z, 1.0);
	Vector4 eyeSpcPt = (MV * inPt);
	Vector4 clipSpcPt = P * eyeSpcPt;


	vec ndcPt(clipSpcPt.x, clipSpcPt.y, clipSpcPt.z);
	if(fabs(clipSpcPt.w)> 1e-04)
			ndcPt /= clipSpcPt.w;
	ndcPt.y *= -1;

	float wx = ofMap(ndcPt.x, -1, 1, x,x+w);
	float wy = ofMap(ndcPt.y, -1, 1, y, y+h);

	return  vec(wx, wy, 0.0);

}


//////////////////////////////////////////////////////////////////////////////
int msx, msy;
vector<vec> clkPts;
vec camPos_near, camPos_far, ray, pt;

void setup()
{
	msx = msy = 0;
	clkPts.clear();

}

void draw()
{

	double ht = 350;
	double ht_plus = 20;

	backGround(0.75);
	drawGrid(20);

	Matrix4 MV, P;
	GLdouble matModelView[16], matProjection[16];
	int viewport[4];
	// get matrixs and viewport:
	glGetFloatv(GL_MODELVIEW_MATRIX, MV.m);
	glGetFloatv(GL_PROJECTION_MATRIX, P.m);
	glGetIntegerv(GL_VIEWPORT, viewport);

	//////////////////////////////////////////////////////////////////////////


	MV.transpose();
	P.transpose();

	vec scr0 = worldToScreen(vec(20, 20, 0) ,MV,P,viewport);
	vec scr1 = worldToScreen(vec(20, -20, 0), MV, P, viewport);
	vec scr2 = worldToScreen(vec(-20, -20, 0), MV, P, viewport);
	vec scr3 = worldToScreen(vec(-20, 20, 0), MV, P, viewport);

	
	glPointSize(8);

	setup2d();

	for (int i = 0; i < 100; i++)
		drawPoint(worldToScreen(vec(ofRandom(-20, 20), ofRandom(-20, 20), ofRandom(-20, 20)), MV, P, viewport));

	restore3d();
	/////////




	drawVector(vec(msx, msy, 0), vec(50, ht, 0), "screen coords"); ht += ht_plus;
	drawVector(camPos_near, vec(50, ht, 0), "camPos N"); ht += ht_plus;
	drawVector(camPos_far, vec(50, ht, 0), "camPos F"); ht += ht_plus;
	drawVector(pt, vec(50, ht, 0), "intersection"); ht += ht_plus;

	setup2d();
	drawCircle(vec(msx + winW*0.5, winH*0.5 - msy, 0), 10, 64);
	restore3d();

	glBegin(GL_LINE_STRIP);
	for (auto &p : clkPts) glVertex3f(p.x, p.y, p.z);
	glEnd();
}



void mousePress(int b, int state, int x, int y)
{
	if (glutGetModifiers() == GLUT_ACTIVE_ALT)
	{
		msx = x - winW * 0.5;
		msy = winH * 0.5 - y;


		camPos_near = screenToWorld(msx, msy, 0.2);
		camPos_far = screenToWorld(msx, msy, 0.9);
		ray = camPos_far - camPos_near;
		pt = rayPlaneIntersection(camPos_near, ray, vec(0, 0, 1), 0);
		clkPts.push_back(pt);
	}

}


///////////////////////////////////////////////////////////////////////////////////////////////
void update(int value)
{

}

void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 'r')setup();
	if (k == 'v')msx += 1;
	if (k == 'c')msx -= 1;
	if (k == 'b')msy -= 1;
	if (k == 'n')msy += 1;

}





#endif // _MAIN_
