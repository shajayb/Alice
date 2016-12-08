

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
#include "graph.h"
#include "newPhysics.h"

////////////////////////////////////////////////////////////////////////// TEMPORARY CLASS DEFINITIONS

#define numFrames 500
class circle
{
public:
	vector<int> particles;
	vec trail[numFrames * 64];
	int frame;
	int trailCnt;

	circle() {};
	circle(vec &cen, float radius, newPhysics &phy)
	{
		frame = trailCnt = 0;
		particles.clear();
		for (int i = 0; i < 64; i++)
		{
			float x = radius * sin(float(i) * 2.0 * PI / 64);
			float y = radius * cos(float(i) * 2.0 * PI / 64);
			int n = phy.makeParticle(vec(x, y, 0) + cen, 1.0, false);
			particles.push_back(n);
			phy.p[n].v = (phy.p[n].p - cen).normalise() * 5.0;
			//if (i > 0)phy.makeSpring(n, n - 1, 0.1);
		}
	}

	void checkCollide(circle &other, newPhysics &phy)
	{
		for (int i = 0; i < particles.size(); i++)
		{
			for (int j = 0; j < other.particles.size(); j++)
			{
				int id1 = particles[i];
				int id2 = other.particles[j];
				vec a = phy.p[id1].p;
				vec b = phy.p[id2].p;
				vec c = b - a;
				if (c * c < 2)
				{
					phy.p[id1].fixed = true;
					phy.p[id2].fixed = true;
				}
			}
		}


	}

	void draw(newPhysics &phy , bool showStack = false)
	{
		for (int i = 1; i < particles.size(); i++)drawLine(phy.p[particles[i]].p, phy.p[particles[i - 1]].p);
		//for (int i = 0; i < trailCnt; i++)drawPoint(trail[i]);

		if(showStack)
		for (int i = 0; i < frame * 64; i += 64)
			for (int j = i; j < i + particles.size(); j++)
			{
				vec4 clr = getColour(i, 0, frame * 64);
				glColor3f(clr.r, clr.g, clr.b);
				drawLine(trail[j], trail[j - 1]);
			}
	}

	void updateTrail(newPhysics &phy)
	{

		for (int i = 0; i < particles.size(); i++)
		{

			trail[trailCnt] = phy.p[particles[i]].p;
			trail[trailCnt].z = float(frame) * 0.2;
			trailCnt++;
			if (trailCnt >= numFrames * 64)trailCnt = 0;
		}

		frame++;
		if (frame >= numFrames)frame = 0;
	}

};

////////////////////////////////////////////////////////////////////////// GLOBALS


newPhysics phy;
vector<circle>circles;
int j = 0;
Graph G;
importer imp;

bool importFromFile = false;
bool showstack = false;

ButtonGroup B;


////////////////////////////////////////////////////////////////////////// MAIN PROGRAM //

void setup()
{
	j = 0;
	phy = *new newPhysics();

	circles.clear();

	for (int i = 0; i < 6; i++)
	{
		float x = 20 * sin(float(i) * 2.0 * PI / 6);
		float y = 20 * cos(float(i) * 2.0 * PI / 6);

		if(!importFromFile)circles.push_back(circle(vec(x, y, 0), 1, phy));
	}

	//////////////////////////////////////////////////////////////////////////

	imp = *new importer("data/inPts.txt", 100000, 1.0);
	imp.readPts_p5();

	vec minV, maxV;
	imp.boundingBox(minV, maxV);

	vec cen = (minV + maxV)*0.5;
	double diag = (maxV - minV).mag();

	for (int i = 0; i < imp.nCnt; i++)imp.nodes[i].pos -= cen;
	for (int i = 0; i < imp.nCnt; i++)imp.nodes[i].pos *= 100.0 / diag;


	G = *new Graph();

	for (int i = 0; i < imp.nCnt; i++)G.createVertex(imp.nodes[i].pos);


	for (int i = 0; i < imp.nCnt; i++)
	{
		G.positions[i].z = 0;
		if (importFromFile)circles.push_back(circle(G.positions[i], ofRandom(1, 5), phy));
	}

	//////////////////////////////////////////////////////////////////////////
	B = *new ButtonGroup(vec(50, 350, 0));
	B.addButton(&importFromFile, "importFile");
	B.addButton(&showstack, "showStack");

	//////////////////////////////////////////////////////////////////////////

	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void update(int value)
{

	phy.UpdateParticles(0.1, 2);

	for (auto &cir : circles)
		for (auto &cir2 : circles)
		{
			if (&cir == &cir2) continue;
			cir.checkCollide(cir2, phy);
		}

	for (auto &cir : circles)cir.updateTrail(phy);
}

void draw()
{

	backGround(0.95);

	//phy.display();
	//G.draw();

	glColor3f(0, 0, 0);
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(2);
	for (auto &cir : circles)cir.draw(phy, showstack);
	glLineWidth(1);


	B.draw();
}

void keyPress(unsigned char k, int xm, int ym)
{

	if (k == 'r')setup();

	if (k == 'c')
	{
		circle c = *new circle(vec(j * 10, ofRandom(-10, 10), 0), 4, phy);
		circles.push_back(c);
		j++;
	}
	if (k == 'a')
	{

		int n = phy.makeParticle(vec(ofRandom(-10, 10), 0, 0));
		int n2 = ofRandom(0, phy.np - 1);
		phy.makeSpring(n, n2, 0.1);
	}
}

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		B.performSelection(x, y);
	}
}

void mouseMotion(int x, int y)
{

}


#endif // _MAIN_
