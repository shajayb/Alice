#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

Robot_Symmetric nachi;
vec target(10, 0, 0);

importer ptsReader;
SliderGroup S;
ButtonGroup B;

char gcode[600];
ofstream myfile, myfile_write;
vec diff;

void setup()
{


	//////////////////////////////////////////////////////////////////////////

	//nachi = *new Robot();
	//vec pt = nachi.ForwardKineMatics(nachi.rot);
	nachi.addMeshes();


	S = *new SliderGroup();
	S.addSlider(&nachi.rot[0], "J1");
	S.addSlider(&nachi.rot[1], "J2");
	S.addSlider(&nachi.rot[2], "J3");
	S.addSlider(&nachi.rot[3], "J4");
	S.addSlider(&nachi.rot[4], "J5");
	S.addSlider(&nachi.rot[5], "J6");


	S.sliders[0].attachToVariable(&nachi.rot[0], -170, 170);
	S.sliders[1].attachToVariable(&nachi.rot[1], -45, 170);
	S.sliders[2].attachToVariable(&nachi.rot[2], -67, 120);
	S.sliders[3].attachToVariable(&nachi.rot[3], -190, 190);
	S.sliders[4].attachToVariable(&nachi.rot[4], -120, 120);
	S.sliders[5].attachToVariable(&nachi.rot[5], -370, 370);



	ptsReader = *new importer("data/inPts.txt", 10000, 1.0);
	ptsReader.readPts_p5();

	//////////////////////////////////////////////////////////////////////////

}


void update(int value)
{


	nachi.rot[0] = ofClamp(nachi.rot[0], -170, 170);
	nachi.rot[1] = ofClamp(nachi.rot[1], -45, 170);
	nachi.rot[2] = ofClamp(nachi.rot[2], -67, 120);
	nachi.rot[3] = ofClamp(nachi.rot[3], -190, 190);
	nachi.rot[4] = ofClamp(nachi.rot[4], -120, 120);
	nachi.rot[5] = ofClamp(nachi.rot[5], -370, 370);
	vec pt = nachi.ForwardKineMatics(nachi.rot);
	//nachi.inverseKinematics_analytical(target, false);
}

void draw()
{
	backGround(0.75);

	drawGrid(20.0);


	S.draw();
	B.draw();


	//nachi.draw(true);
	glColor3f(1, 0, 1);
	drawCircle(target, 5, 32);


	glPointSize(3);
	glColor3f(0, 0, 1);
	//for (int i = 0; i < ptsReader.nCnt; i++)drawPoint(ptsReader.nodes[i].pos);


	drawLine(nachi.joints[5], nachi.joints[5] - diff);


	char s[200];
	double  l = 0.;
	for (int i = 1; i < DOF; i++) l += nachi.joints[i].distanceTo(nachi.joints[i - 1]);
	sprintf_s(s, " link lengths %1.4f", l);
	setup2d();
	drawString(s, winW - 250, winH - 50);
	//drawString(ang0, winW * 0.5, 25);
	//drawString(ang1, winW * 0.5, 50);
	//drawString(ang2, winW * 0.5, 75);
	//drawString(ang3, winW * 0.5, 100);
	//drawString(ang4, winW * 0.5, 125);
	//drawString(ang5, winW * 0.5, 150);

	glColor3f(1, 0, 0);
	sprintf_s(s, " TCP_X %1.4f %1.4f %1.4f", nachi.TCP_x.x, nachi.TCP_x.y, nachi.TCP_x.z);
	drawString(s, winW * 0.5, 25);

	glColor3f(0, 1, 0);
	sprintf_s(s, " TCP_Y %1.4f %1.4f %1.4f", nachi.TCP_y.x, nachi.TCP_y.y, nachi.TCP_y.z);
	drawString(s, winW * 0.5, 50);

	glColor3f(0, 0, 1);
	sprintf_s(s, " TCP_Z %1.4f %1.4f %1.4f", nachi.TCP_z.x, nachi.TCP_z.y, nachi.TCP_z.z);
	drawString(s, winW * 0.5, 75);

	vec pp = nachi.joints[5] + vec(0, 0, -24.98);
	sprintf_s(s, " %1.4f %1.4f %1.4f", pp.x, pp.y, pp.z);
	drawString(s, winW * 0.5, 125);
	drawString(gcode, winW * 0.5, 100);

	restore3d();

}


int ptcnt = 0;

void keyPress(unsigned char k, int xm, int ym)
{



	if (k == 'n')
	{

		/*target = vec(rx, rx, rx);*/
		target = ptsReader.nodes[ptcnt].pos;
		target.z += 0;

		vec x(1, 0, 0);
		vec y(0, 1, 0);
		vec z(0, 0, -1);

		Matrix4 TOOL;
		TOOL.setColumn(0, x);
		TOOL.setColumn(1, y);
		TOOL.setColumn(2, z);
		TOOL.setColumn(3, target);
		nachi.inverseKinematics_analytical(TOOL, false);

		ptcnt++;
		if (ptcnt >= ptsReader.nCnt)ptcnt = 0;
	}

	if (k == 'N')
	{
		myfile_write.open("C:/FD_ONDESK/NRA2011/WORK/PROGRAM/MZ07-01-A.083", ios::out);
		if (myfile_write.fail())cout << " error in opening file  " << "MZ07-01-A.083" << endl;

		for (int i = 0; i < 1000; i++)
		{
			cout << target.z << endl;
			target = ptsReader.nodes[i].pos;
			//			target.z += Z;

			vec x(1, 0, 0);
			vec y(0, 1, 0);
			vec z(0, 0, -1);

			Matrix4 TOOL;
			TOOL.setColumn(0, x);
			TOOL.setColumn(1, y);
			TOOL.setColumn(2, z);
			TOOL.setColumn(3, target);
			nachi.inverseKinematics_analytical(TOOL, false);
			vec pt = nachi.ForwardKineMatics(nachi.rot);
			{
				float e_x, e_y, e_z, r, p, y;
				e_x = nachi.rot[0];
				e_y = nachi.rot[1];
				e_z = nachi.rot[2];
				r = nachi.rot[3];
				p = nachi.rot[4];
				y = 0;// nachi.rot[5];

					  /*sprintf_s(gcode, "MOVEX A=1,AC=0,SM=0,M1X,L,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),S=1200.0,H=2,MS, CONF=0001", e_x * 10, e_y * 10, e_z * 10, r+90, p, y);*/
				sprintf_s(gcode, "MOVEX A=8P,AC=0,SM=0,M1J,P,( %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f),T=0.1,H=3,MS, CONF=0001", e_x, e_y, e_z, r, p, y);
				myfile_write << gcode << endl;

			}
		}

		myfile_write.close();

	}


}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		S.performSelection(x, y, HUDSelectOn);
		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
}


