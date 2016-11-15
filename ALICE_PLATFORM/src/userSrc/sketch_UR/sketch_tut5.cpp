#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;

////////////////////////////////////////////////////////////////////
vec P,V, gravity;

class particle
{
public:

	vec P, V, gravity, F;

	void setup()
	{
		P = vec( ofRandom(-20,20), ofRandom(-20, 20), 0);
		V = P / P.mag(); //  vec(1, 1, 0);
		gravity = vec(0, 0, -1);
	}


	void update( vec &Pc, vector<particle> &_COP  ) // and symbol
	{
		//1. reset force
		//2. add one or more repulsions
		// 3. update P & V
		F = gravity;
		// calculate F1
		addRepulsion(Pc);
		// add interparticle repulsion
		addInterParticleRepulsion(_COP);

		//////////////////////////////// update 
		updatePositionAndVelocity();
	}

	void addInterParticleRepulsion( vector<particle> _COP)
	{
		for (auto &pedro : _COP)
		{
			vec Pn = pedro.P;
			float d = (Pn - P).mag();
			vec e_unit = (Pn - P)/ d ;
			vec F_pedro = e_unit * -1.0 / (d *d);
			if (d > 0.01 && d < 2 )F = F + F_pedro;
		}
	}

	void addRepulsion( vec Pc )
	{
		vec PC_unit = (P - Pc) / (P - Pc).mag();
		float d = (P - Pc).mag();
		float dSq = (d * d);
		vec F2 = PC_unit * 1.0 / dSq;
		// add F1 to whatever is F;
		if (dSq >= 0.75)F = F + F2;
	}

	void updatePositionAndVelocity()
	{
		vec A = F / 1.0;
		//A = deltaV / deltaT ; let say deltaT = 0.1
		float deltaT = 0.1;
		vec deltaV = A * deltaT;
		// V = deltaP / deltaT ;
		vec deltaP = V * deltaT;

		P = P + deltaP;
		V =  deltaV;
	}

	void draw()
	{
		drawPoint(P);
		drawLine(P, P + V);
	}

};

///////////////////////////////////////////////////////////////
vector<particle> COP;
vec C = vec(-5, 2, 0);
bool run = false;

void setup()
{
	for (int i = 0; i < 500; i++)
	{
		particle a;
		COP.push_back(a);
	}


	for (auto &prt : COP)prt.setup();



}


void update(int value)
{
	if(run == true)
		for (auto &p : COP)p.update( C, COP );//pass by copy

}

void draw()
{

	backGround(0.75);

	glPointSize(5);

	for (auto &prt : COP)prt.draw();

	glColor3f(1, 0, 0);
	drawPoint(C);
	drawCircle(C, 2, 64);
}


void mousePress(int b, int state, int x, int y)
{


}


///////////////////////////////////////////////////////////////////////////////////////////////

void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{

	if (k == 'r')setup();
	if (k == 'p')run = !run;

}




