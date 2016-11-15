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

vec A = vec(10, 10, 10);
vec B = vec(10, 20, 5);
vec C;
vec D;

void setup()
{
	 

}

void draw()
{

	backGround(0.75);

	glPointSize(10);
	
	drawPoint(A);
	drawPoint(B);

	drawLine(vec(0, 0, 0), C);
}



void mousePress(int b, int state, int x, int y)
{


}


///////////////////////////////////////////////////////////////////////////////////////////////
void update(int value)
{
	// Object oriented programming
	// object.property
	// object.action()
	// object.operation

	C = A - B;
	
	vec C_unit;
	C_unit = C / C.mag();
	C_unit = C_unit *  0.1;
	D = B + C_unit;
	B = D;

	float distanceA_B = C.mag();

	if (distanceA_B < 0.12 )
	{
		A = vec(ofRandom(-10, 10), 10, 0);
	}
}

void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{

	if (k == 'n')
	{
		//code to make A move towards B
		// i.e go to D
		//vec C;
		C = A - B;
		vec C_unit;
		C_unit = C / C.mag();
		D = B + C_unit;
		B = D;

		
	}

	if (k == 'h')
	{
		A = vec(ofRandom(-10,10), 10, 0);
	}

	
}




