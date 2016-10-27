#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
using namespace std;


#define dim 10

class ConstChecker
{
public:

	/*vec P[100];*/
	array<vec, dim> P;

	void fill()
	{
		for (size_t i = 0; i < dim; i++)P[i] = { ofRandom(-1,1),ofRandom(-1,1), ofRandom(-1,1) };

	}

	void print()
	{
		for (size_t i = 0; i < dim; i++)P[i].print();
	}

	//auto /*const vec * */rawArray()
	//{
	//	return P.data();
	//}

	auto /*const vec * */rawArray()
	{
		return &P;
	}

};

#define  rx ofRandom(-1,1)
ConstChecker CC;

void setup()
{
	glEnable(GL_POINT_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	
	
	auto p = CC.rawArray();
	
	*p.print();

	for (size_t i = 0; i < dim; i++)
	{
		
		
		
		////p = & vec(rx,rx,rx);
		//p->print();
		//p++;
	}

	CC.print();
}

void update(int value)
{

}

void draw()
{
	backGround(0.75);
}
void keyPress(unsigned char k, int xm, int ym)
{

}
void mousePress(int b, int state, int x, int y)
{

}
void mouseMotion(int x, int y)
{

}



