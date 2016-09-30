#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
using namespace std;




#define dim 10000


//////

class Tvec
{
public:
	double x, y, z;
	double A[100];

	double distanceTo( Tvec &other)
	{
		return (sqrt(other.x * x + other.y * y + other.z * z));
	}
};

auto distanceTo( Tvec &a,  Tvec&b)
{
	return (sqrt(a.x * b.x + a.y * b.y + a.z * b.z));
}

Tvec AAA[dim], BBB[dim];
//array<Tvec, dim> A;
//array<Tvec, dim> B;
//Tvec AA[dim], BB[dim];

//////

array<double, dim*dim> dists;
double dists2[dim*dim];

void timeStats(long &strt,long& end, string str)
{
	elapsedTime = end - strt;
	cout << elapsedTime / 1000.0f << " - " << str << endl;
}

void setup()
{

	for (int i = 0; i < dim; i++)AAA[i] = { ofRandom(0,1),ofRandom(0,1) ,ofRandom(0,1) };
	for (int i = 0; i < dim; i++)BBB[i] = { ofRandom(0,1),ofRandom(0,1) ,ofRandom(0,1) };
	long start, end;

	{
		//////////////////////////////////////////////////////////////////////////////////// LINEAR ACCESS 

		//////// sequential access of a std:array
		
		start = GetTickCount();;

			for (int i = 0, cnt = 0; i < dim*dim; i++) dists[i] = ofRandom(0, 1);
			
		end = GetTickCount();
		timeStats(start, end, " sequential access of a std:array ");

		//////// sequential access of a c-style array

		start = GetTickCount();;

			for (int i = 0, cnt = 0; i < dim * dim; i++)dists2[i] = ofRandom(0, 1);

		end = GetTickCount();
		timeStats(start, end, " sequential access of a c-style array ");

		////////////////////////////////////////////////////////////////////////////////////

		//////// n-n distance check , object field method , no storage

		start = GetTickCount();;
		int count = 0;
		for (int i = 0, cnt = 0; i < dim; i++)
			for (int j = 0, cnt = 0; j < dim; j++, cnt++)
			{
				double d = AAA[i].distanceTo(BBB[j]);
				count++;
			}
			
		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , object field method , no storage ");


		//////// n-n distance check , function argument method , no storage

		start = GetTickCount();;

		for (int i = 0, cnt = 0; i < dim; i++)
			for (int j = 0, cnt = 0; j < dim; j++, cnt++) distanceTo(AAA[i],BBB[j]);


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , function argument method , no storage ");



	}


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



