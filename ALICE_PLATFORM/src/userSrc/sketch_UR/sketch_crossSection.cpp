#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "newPhysics.h"
#include "circle.h"

// INTERFACE: HEADER & mvc PATTERN;
/////////////////////////////


newPhysics phy; // declare an empty object;
importer imp;
vector<circle> circles;
float Cx = 0.0;

void setup()
{
	// constructor ;
	phy = * new newPhysics(); // make a new instance of class physics, using its constructor
	Cx = 0.0;
	circles.clear();

	imp = *new importer("data/inPts.txt", 10000, 1.0);
	imp.readPts_p5();

	for (int i = 0; i < imp.nCnt; i++)
	{
		circle c = *new circle(phy, 1.0, imp.nodes[i].pos);
		circles.push_back(c);
	}
}

void update(int value)
{
	//update newPhysics
	phy.UpdateParticles(0.15,2);

	for (int i = 0; i < circles.size(); i++)
	{
		for (int j = 0; j < circles.size(); j++)
		{
			if (i == j)continue;

			circles[i].checkCollisions(circles[j], phy);
		}
	}
}

void draw()
{
	
	backGround(0.75);
	glPointSize(5);

	//display physics
	//phy.display();
	for (int i = 0; i < circles.size(); i++)circles[i].draw( phy );
}

void keyPress(unsigned char k, int xm, int ym)
{

	if (k == 'r')setup();

	if (k == 'c')
	{
		circle c = *new circle(phy, 2.0, vec(Cx, 0, 0));
		Cx += 6.0;
		circles.push_back(c);
	}
	
}

void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{
	
}


