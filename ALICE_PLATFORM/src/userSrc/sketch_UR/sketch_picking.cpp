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
	sprintf(s, "%1.2f,%1.2f,%1.2f : ", a.x, a.y, a.z);
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

vec worldToScreen(vec &worldSpacePosition)
{
	float z, rx, ry, _tx, _ty;
	getCamera(z, rx, ry, _tx, _ty);

	Matrix4 T;
	vec a(0, 0, 0);
	T.identity();
	T.translate(_tx, _ty, z);
	T.rotateX(-rx);
	T.rotateZ(-ry);
	a = T * a;

	GLdouble matModelView[16], matProjection[16];
	int viewport[4];
	// get matrixs and viewport:
	glGetDoublev(GL_MODELVIEW_MATRIX, matModelView);
	glGetDoublev(GL_PROJECTION_MATRIX, matProjection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	T.transpose();
	drawMatrix(T, vec(50, 100, 0));

	for (int i = 0; i < 16; i++)matModelView[i] = T[i];


	double screenSpacePoint[3];
	gluProject
	(
		worldSpacePosition.x, worldSpacePosition.y, worldSpacePosition.z,
		matModelView, matProjection, viewport,
		&screenSpacePoint[0], &screenSpacePoint[1], &screenSpacePoint[2]
	);

	int scrCenX = (viewport[2] - viewport[0]) / 2;
	int scrCenY = (viewport[3] - viewport[1]) / 2;
	int msx = screenSpacePoint[0];// +scrCenX;
	int msy = screenSpacePoint[1];// +screenSpacePoint[1];

	//Vector4 homog_pt(worldSpacePosition.x, worldSpacePosition.y, worldSpacePosition.z, 1.0);
	//Vector4 clickSpacePos = P.transpose() * (MV.transpose() * homog_pt);
	//
	//vec ndcSpacePos(clickSpacePos.x, clickSpacePos.y, clickSpacePos.z);
	////if (fabs(clickSpacePos.w) < 1e-4)clickSpacePos.w = 1.0;
	//ndcSpacePos /= clickSpacePos.w;

	/*float winX = ( (ndcSpacePos.x + 1.0) / 2.0 ) * viewport[2] + viewport[0];
	float winY = (( ndcSpacePos.y + 1.0) / 2.0) * viewport[3] + viewport[1];*/

	return  vec(msx, msy, 0); //MV * worldSpacePosition + vec(scrCenX, scrCenY, 0);// vec(msx, msy, 0);// +vec(scrCenX, -scrCenY, 0);
}



//////////////////////////////////////////////////////////////////////////////
int msx, msy;
vector<vec> clkPts;
vec camPos_near, camPos_far, ray, pt;
vector<vec> pts;
vec aScr;

void setup()
{
	msx = msy = 0;
	clkPts.clear();

	for (int i = 0; i < 100; i++)pts.push_back( vec(ofRandom(-10, 10), ofRandom(-10, 10), ofRandom(-10, 10) ) ) ;
}

void draw()
{

	double ht = 350;
	double ht_plus = 20;



	//{
	//	//{
	//	//	
	//	//	
	//	//	float z, rx, ry, _tx, _ty;
	//	//	getCamera(z, rx, ry, _tx, _ty);

	//	//	
	//	//	glLoadIdentity(); // reset your paper to default scale, rotations, translations

	//	//	Matrix4 T;
	//	//	vec a(0, 0, 0);
	//	//	T.identity();
	//	//	T.translate(_tx, _ty, z);
	//	//	T.rotateX(-rx);
	//	//	T.rotateZ(-ry);
	//	//	a = T * a;

	//	////	glLoadMatrixf(T.transpose().m);
	//	//	drawMatrix(T.transpose(), vec(50, 500, 0));
	//	//	glTranslatef(_tx, _ty, -z); // zooming
	//	//	glRotatef(rx, 1, 0, 0);
	//	//	glRotatef(ry, 0, 0, 1);

	//	//	drawVector(a, vec(50, 200, 0), " a ");
	//	//}

	//	glColor3f(1, 1, 1);
	//	drawGrid(gridSz);

	//	vec worldSpacePosition(20, 20, 0);
	//	GLdouble matModelView[16], matProjection[16];
	//	int viewport[4];
	//	// get matrixs and viewport:
	//	glGetDoublev(GL_MODELVIEW_MATRIX, matModelView);
	//	glGetDoublev(GL_PROJECTION_MATRIX, matProjection);
	//	glGetIntegerv(GL_VIEWPORT, viewport);



	//	double screenSpacePoint[3];
	//	gluProject
	//	(
	//		worldSpacePosition.x, worldSpacePosition.y, worldSpacePosition.z,
	//		matModelView, matProjection, viewport,
	//		&screenSpacePoint[0], &screenSpacePoint[1], &screenSpacePoint[2]
	//	);

	//	int scrCenX = (viewport[2] - viewport[0]) / 2;
	//	int scrCenY = (viewport[3] - viewport[1]) / 2;
	//	int msx = screenSpacePoint[0];// +scrCenX;
	//	int msy = screenSpacePoint[1];// +screenSpacePoint[1];

	//	/*setup2d();
	//	drawPoint(vec(msx, msy, 0));
	//	restore3d();*/

	//	Matrix4 MV;
	//	for (int i = 0; i < 16; i++)MV[i] = matModelView[i];
	//	//cout << MV.transpose() << endl;
	//	
	//}


	/////////

	aScr = vec(20, 20, 0);
	aScr.z = 0;
	aScr = worldToScreen(aScr);

		setup2d();
		drawPoint(aScr);
		restore3d();

	glPointSize(5);

	drawVector(vec(msx, msy, 0), vec(50, ht, 0), "screen coords wrt screen center"); ht += ht_plus;
	drawVector(camPos_near, vec(50, ht, 0), "camPos N"); ht += ht_plus;
	drawVector(camPos_far, vec(50, ht, 0), "camPos F"); ht += ht_plus;
	drawVector(pt, vec(50, ht, 0), "intersection"); ht += ht_plus;
	drawVector(aScr, vec(50, ht, 0), "screen coords wrt bottom left origin"); ht += ht_plus;

	setup2d();
	drawCircle(vec(msx + winW*0.5, winH*0.5 - msy, 0), 10, 64);
	restore3d();

	//glBegin(GL_LINE_STRIP);
	//for (auto &p : clkPts) glVertex3f(p.x, p.y, p.z);
	//glEnd();


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

