#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;

#define  rx ofRandom(-1,1)
#define  rxx ofRandom(0.75,1)
class particle
{
public:

	vec P, V,F,A;
	vec deltaV, deltaP;


	void setup()
	{
		P = vec(rx * 0.1, rx * 0.1, 0) * 10;
		V = vec(rx * 0.6,rx * 0.6, rxx);
		V.normalise();
		V *= 20 * rxx;
	}

	void resetForces()
	{
		F = vec(0, 0, 0);
	}
	void addForces()
	{

		F += vec(0, 0, -10);
	}

	void addCollisions_ground()
	{
		if (P.z < 0)
		{
			vec normal(0, 0, 1);
			vec Vn = normal * (V * normal);
			vec Vt = V - Vn;
			
			V = Vt + (Vn * -1);
			//V *= 0.75;
		}
	}

	void addInterParticleForces( vector<particle> &particles )
	{

		for (auto &p : particles)
		{
			
			if (&p == this) continue;
			
			vec d = P - p.P;
			float dSq = d * d;
			if (dSq >= 1)F += d *1 / (dSq);

			//d.normalise();
			//d *= 10;
			
		}
	}

	void calculatedelta()
	{
		float dt = 0.05;
		A = F / 1.0; // F = mA ;
		deltaV = A * dt; // dvdt = a -> dv = a*dt
		deltaP = V * dt;// dpdt = v --> dp = v * dt
	}

	void update()
	{
		V += deltaV;
		//V *= 0.9;
		P += deltaP;
	}

	void draw()
	{
		drawPoint(P);
		drawLine(P, P + (V / V.mag()) * .2 );
	}
};

////////////////////////////////////////////////////////////////////

vector<particle> particles;


void setup()
{

	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	particles.clear();

	for (int i = 0; i < 1; i++)
	{
		particle a;
		a.setup();
		particles.push_back(a);
	}
}


void update(int value)
{


}

void draw()
{

	backGround(0.75);

	glPointSize(10);
	for (auto &p : particles)p.draw();

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

	if (k == 'u')
	{
		for (auto &p : particles)
		{
			p.resetForces();
			p.addForces();
			p.addCollisions_ground();
			//p.addInterParticleForces(particles);
			p.calculatedelta();
			p.update();
		}
	}
}




