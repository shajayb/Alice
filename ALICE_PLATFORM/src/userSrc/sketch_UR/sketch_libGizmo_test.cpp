#define _MAIN_

#ifdef _MAIN_
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include "metaMesh.h"
#include "nachi.h"
#include "graphStack.h"
#include "IGizmo.h"
#include "imgui.h"
////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

IGizmo *Gz, *Gz_m, *Gz_r, *Gz_s;
float objectMatrix[16];
Matrix4 T;
Mesh M;


void zNearAndFar( Matrix4 &PM, double &zNear, double &zFar )
{

	float m22 = -PM[10];// .m22;
	float m32 = -PM[14];// .m32;

	zNear = (2.0f*m32) / (2.0f*m22 - 2.0f);
	zFar = ((m22 - 1.0f)*zNear) / (m22 + 1.0);

}

void ExtractPlane(vec &planeN, double &planeD, Matrix4 &mat, int row)

{

	int scale = (row < 0) ? -1 : 1;

	row = abs(row) - 1;

	planeN.x = mat[3] + scale * mat[row];
	planeN.y = mat[7] + scale * mat[row + 4];
	planeN.z = mat[11] + scale * mat[row + 8];
	planeD = mat[15] + scale * mat[row + 12];



	float length = sqrtf(planeN*planeN);

	planeN.normalise();
	planeD /= length;

}

vec GetOGLPos(int x, int y, vec &pt1, vec &pt2 )
{
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;

	
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	vec pt = vec(posX, posY, posZ);
	
	winZ = 0;
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	pt1 = vec(posX, posY, posZ);
	
	winZ = 1;
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	pt2 = vec(posX, posY, posZ);

	return pt ;
}
////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------

////// ---------------------------------------------------- MODEL  ----------------------------------------------------
vec clkPt;
int msx, msy;

void setup()	
{
	T.identity();
	for (int i = 0; i < 16; i += 1)objectMatrix[i] = T[i];

	Gz_m = CreateMoveGizmo();
	Gz_r = CreateRotateGizmo();
	Gz_s = CreateScaleGizmo();
	Gz = Gz_m;;
	Gz->SetEditMatrix(objectMatrix);
	Gz->SetLocation(IGizmo::LOCATE_WORLD);
	Gz->SetScreenDimension(winW, winH);


	MeshFactory fac;
	M = fac.createPlatonic(1, 6);

	Matrix4 Tr;
	Tr.setColumn(0, vec(1, 1, 0).normalise());
	Tr.setColumn(1, vec(1, -1, 0).normalise());
	Tr.setColumn(2, vec(0, 0, 1));
	Tr.setColumn(3, vec(0, 0, 0.5));
	Tr.invert();
	for (int i = 0; i < 8; i++) M.positions[i] = Tr * M.positions[i];


	msx = 1440; msy = 770;
	
}

void update(int value)
{
	T.invert();
	for (int i = 0; i < M.n_v; i++) M.positions[i] = T * M.positions[i];

	
	Gz->SetScreenDimension(winW, winH);
	for (int i = 0; i < 16; i += 1)T[i] = objectMatrix[i];
	T = T.transpose();

	for (int i = 0; i < M.n_v; i++) M.positions[i] = T * M.positions[i];
	
	
}


void printMatrix4( Matrix4 &M)
{
	char s[200];
	cout << "------------" << endl;
	for (int i = 0; i < 16; i+=4)
	{
		for (int j = i; j < i + 4; j++)
		{
			sprintf(s, " %1.2f", M[j]);
			cout << s << ",";
		}

		cout << endl;
	}

}

void drawMatrix( vec str , Matrix4 &Mat)
{
	char s[200];
	double width = 4 * 20;
	double ht = 24;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			sprintf(s, "%1.2f", Mat[j * 4 + i]);
			drawString(s, i * width + str.x, j * ht + str.y);
		}
}
////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{
	char s[200];
	backGround(0.75);
	drawGrid(20);
	drawGrid(200);
	

	Gz->Draw();

	float viewMat[16];
	float projMat[16];

	glGetFloatv(GL_MODELVIEW_MATRIX, viewMat);
	glGetFloatv(GL_PROJECTION_MATRIX, projMat);

	Gz->SetCameraMatrix(viewMat, projMat);
	
	Matrix4 T, P, MV;
	glGetFloatv(GL_MODELVIEW_MATRIX, MV.m);
	glGetFloatv(GL_MODELVIEW_MATRIX, P.m);



	{
		float _z, _rx, _ry, _tx, _ty;
		getCamera(_z, _rx, _ry, _tx, _ty);
	
	//	printf("%1.2f,%1.2f,%1.2f \n", _z, _rx, _ry);

		

		{	
			
			setup2d();

				sprintf(s, "screen %i,%i,%i", msx, msy, 0.5);
				drawString(s, 50, 260);


				vec screenCen(winW*0.5, winH*0.5, 0);
				vec screenCoord = vec(msx, msy, 0) - screenCen;
				clkPt = screenCoord;
				//screenCoord /= _z;
				//clkPt = P.invert().transpose() * clkPt;
				sprintf(s, "screen space coordinates w.r.t screen cen %1.2f,%1.2f,%1.2f", screenCoord.x, screenCoord.y, screenCoord.z);
				drawString(s, 50, 275);
				
				// cam matrix = MV.transpose();: transforms worldspace to camera;
				MV = MV.transpose();
				//MV.setDiag(1.0);
				drawMatrix(vec(50,50,0),MV);
				vec camX = MV.getColumn(0).normalise();
				vec camY = MV.getColumn(1).normalise();
				vec camZ = MV.getColumn(2).normalise();
				vec camPos = MV.getColumn(3);

				//Matrix4 R4;
				//R4.identity();
				//R4.translate(0, 0,-_z);
				//R4.translate(_tx, _ty, 0);
				//R4.rotateX(_rx);
				//R4.rotateY(_ry);
				//R4.invert();
				//MV.setDiag(-1);
				Matrix3 R;
				for (int i = 0; i < 3; i++)R.setColumn(i, MV.getColumn(i).normalise());

				R.setColumn(0, camX * -1);
				R.setColumn(1, camY * -1);
				R.setColumn(2, camZ );

				camPos = R.transpose() * camPos;
				camPos.z *= -1;
				MV.setColumn(3, camPos);
				clkPt += camPos;
				
				//cam inverted matrix : transforms camera to worldspace;
				glColor3f(1, 1, 1);
				Matrix4 MV_inv = MV;
				//Matrix3 rot;
				//rot.setColumn(0, camX * -1);
				//rot.setColumn(1, camY * -1);
				//rot.setColumn(2, camZ);
				//rot.transpose();
				//for (int i = 0; i < 3; i++)MV_inv.setColumn(i, rot.getColumn(i));
				//MV_inv.setColumn(3, MV.getColumn(3)*-1);
				//MV_inv.setDiag(1.0);
				MV_inv.invert();
				drawMatrix(vec(50, 150, 0), MV_inv);

				double ht = 500;
				sprintf(s, "camPos %1.2f,%1.2f,%1.2f", camPos.x, camPos.y, camPos.z);
				glColor3f(0, 0, 0); drawString(s, 50, ht); ht += 25;
				
				sprintf(s, "camX %1.2f,%1.2f,%1.2f", camX.x, camX.y, camX.z);
				glColor3f(1,0,0); drawString(s, 50, ht); ht += 25;
				
				sprintf(s, "camY %1.2f,%1.2f,%1.2f", camY.x, camY.y, camY.z);
				glColor3f(0,1,0); drawString(s, 50, ht); ht += 25;
				
				sprintf(s, "camZ %1.2f,%1.2f,%1.2f", camZ.x, camZ.y, camZ.z);
				glColor3f(0, 0, 1); drawString(s, 50, ht); ht += 25;
				
				///
				vec ray,pt1,pt2;
				vec plane1, plane2;
				double d1, d2;
				//GetOGLPos(msx, msy, pt1,pt2);

				//////////////////////////////////////////////////////////////////////////
				vec P0 = clkPt;
				vec V = camPos - vec(0,0,0);//cam focus
				V.normalise(); V *= -1;
				vec camDir = V;
				sprintf(s, "camDir : %1.2f,%1.2f,%1.2f", camDir.x, camDir.y, camDir.z);
				drawString(s, 50, 305);
				glLineWidth(10);
				drawLine(vec(0, 0, 0), V * 10);
				glLineWidth(1);

				V =  camPos + camDir * 0.1;
				V += camX * screenCoord.x + camY * screenCoord.y; //  vec(msx, msy, 0);
				vec rayPt = V;
				
				sprintf(s, "rayPt on near Plane :  %1.2f,%1.2f,%1.2f", V.x, V.y, V.z);
				drawString(s, 50, 320);

				V =  V - camPos ;
				V.normalise();
				
				sprintf(s, "rayDir :  %1.2f,%1.2f,%1.2f", V.x, V.y, V.z);
				drawString(s, 50, 335);
				
				///
				P0 = rayPt;
				V *= -1;
				vec N = vec(0, 0, 1);
				double t = -1 * (P0 * N) / (V*N);
				vec ws_clkPt = P0 + V * t;// MV_inv * clkPt;
				//ws_clkPt.z = 0;
				ws_clkPt *= 0.1;
				////
				sprintf(s, "ray intersection pt on ground plane : %1.2f,%1.2f,%1.2f", ws_clkPt.x, ws_clkPt.y, ws_clkPt.z);
				drawString(s, 50, 350);


			restore3d();
			
			glPointSize(15);
			setup2d();
				glColor3f(1, 1, 1); drawPoint( vec(msx,msy,0)  );
			restore3d();

				glColor3f(0,0,0); drawPoint(ws_clkPt);
			glPointSize(1);
			//drawLine(ws_clkPt, vec(0, 0, 0));

			
		}


	}
	
	//
	wireFrameOn();

		M.draw();


	wireFrameOff();
	
	

	glColor3f(0, 0, 0);
	setup2d();

	
		//sprintf(s, "%i", updateCam);
		//drawString(s, 50, 300);

		//r (int i = 0; i < 16; i++) proj_matrix[i] = 0;

		///forward
		vec str(50, 400, 0);
		Matrix4 OM(objectMatrix);
		drawMatrix(str, OM);


	restore3d();

	

}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------
bool altPressed = false;
void keyPress(unsigned char k, int xm, int ym)
{
	//XVTFE reserved 
	if (k == 'A')updateCam = !updateCam;
	if (k == 'P')perspCamera();
	if (k == 'w')
	{
		Gz = Gz_m;
		Gz->SetLocation(IGizmo::LOCATE_WORLD);
	}
	if (k == 'e')
	{
		Gz = Gz_r;
		Gz->SetLocation(IGizmo::LOCATE_LOCAL);
	}
	if( k =='r')
	{
		Gz = Gz_s;
		Gz->SetLocation(IGizmo::LOCATE_WORLD);
	}

	Gz->SetEditMatrix(objectMatrix);
	Gz->SetScreenDimension(winW, winH);

	if (k == 's')
	{
		msx = winW * 0.5 ;
		msy = winH * 0.5;
	}

	if (k == 'x')msx += 1;
	if (k == 'z')msx -= 1;
	if (k == 'c')msy -= 1;
	if (k == 'v')msy += 1;
}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
		if (Gz->OnMouseDown(x, y))updateCam = false;
			
	if (GLUT_LEFT_BUTTON == b && GLUT_UP == state)
	{
		Gz->OnMouseUp(x, y);
		updateCam = true;
	}
	
	
	int mod = glutGetModifiers();
	if (mod == GLUT_ACTIVE_ALT)
	{
		msx = x; msy = y;

		altPressed = true;
	}
}

void mouseMotion(int x, int y)
{
	Gz->SetEditMatrix(objectMatrix);
	Gz->OnMouseMove(x, y);
//	clkPt = GetOGLPos(x, y);
}




#endif // _MAIN_
