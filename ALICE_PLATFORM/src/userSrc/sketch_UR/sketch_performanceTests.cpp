#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;



#define dim 1000


//////

class Tvec
{
public:
	double x, y, z;
	double A[100];

	double distanceTo(Tvec &other)
	{
		return (sqrt(other.x * x + other.y * y + other.z * z));
	}
};



auto distanceTo(Tvec &a, Tvec&b)
{
	return (sqrt(a.x * b.x + a.y * b.y + a.z * b.z));
}

Tvec AAA[dim], BBB[dim];
array<Tvec, dim> A,B;
//array<Tvec, dim> B;
//Tvec AA[dim], BB[dim];

//////

array<double, dim*dim> dists;
double dists2[dim*dim];

void timeStats(long &strt, long& end, string str)
{
	elapsedTime = end - strt;
	cout << elapsedTime / 1000.0f << " - " << str << endl;
}

template<std::size_t SIZE>
generator<double> distanceTO(  array< Tvec,SIZE> &A,  array< Tvec, SIZE> &B )
{
	for (auto &a : A)
		for (auto &b : A)yield a.distanceTo(b);
}


void setup()
{

	for (int i = 0; i < dim; i++)AAA[i] = { ofRandom(0,1),ofRandom(0,1) ,ofRandom(0,1) };
	for (int i = 0; i < dim; i++)BBB[i] = { ofRandom(0,1),ofRandom(0,1) ,ofRandom(0,1) };

	for (int i = 0; i < dim; i++)A[i] = { ofRandom(0,1),ofRandom(0,1) ,ofRandom(0,1) };
	for (int i = 0; i < dim; i++)B[i] = { ofRandom(0,1),ofRandom(0,1) ,ofRandom(0,1) };



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

		//////////////////////////////////////////////////////////////////////////////////// RANDOM ACCESS 

		//////// sequential access of a std:array

		start = GetTickCount();;

		for (int i = 0, cnt = 0; i < dim*dim; i++) dists[int(ofRandom(0, dim*dim))] = ofRandom(0, 1);

		end = GetTickCount();
		timeStats(start, end, " random access of a std:array ");

		//////// sequential access of a c-style array

		start = GetTickCount();;

		for (int i = 0, cnt = 0; i < dim * dim; i++)dists2[int(ofRandom(0, dim*dim))] = ofRandom(0, 1);

		end = GetTickCount();
		timeStats(start, end, " random access of a c-style array ");

		//////////////////////////////////////////////////////////////////////////////////// FOR EACH TO EACH OTHER 

		//////// n-n distance check , object field method , no storage

		start = GetTickCount();;
		for (int i = 0, cnt = 0; i < dim; i++)
			for (int j = 0, cnt = 0; j < dim; j++, cnt++)double d = AAA[i].distanceTo(BBB[j]);


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , object field method :  a.distanceTo(b) , no storage ");


		//////// n-n distance check , function argument method , no storage

		start = GetTickCount();;

		for (int i = 0, cnt = 0; i < dim; i++)
			for (int j = 0, cnt = 0; j < dim; j++, cnt++) distanceTo(AAA[i], BBB[j]);


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , function argument method : distanceTo(b,a), no storage ");

		//////// n-n distance check , object field method , no storage

		start = GetTickCount();
		for (auto &a:A)
			for (auto &b : A) a.distanceTo(b);


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , auto &a:A | auto &b:B, no storage ");

		//////////////////////////////////////////////////////////////////////////////////// FOR EACH TO EACH OTHER , store results

		//////// n-n distance check , object field method , store results

		start = GetTickCount();;
		for (int i = 0, cnt = 0; i < dim; i++)
			for (int j = 0, cnt = 0; j < dim; j++, cnt++)dists[cnt] = AAA[i].distanceTo(BBB[j]);


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , object field method : a.distanceTo(b) , store results ");


		//////// n-n distance check , function argument method , store results

		start = GetTickCount();;

		for (int i = 0, cnt = 0; i < dim; i++)
			for (int j = 0, cnt = 0; j < dim; j++, cnt++)dists[cnt]= distanceTo(AAA[i], BBB[j]);


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , function argument method : distanceTo(a,b) , store results ");

		//////// n-n distance check , object field method , store results

		start = GetTickCount();
		int count = 0;
		for (auto &a : A)
			for (auto &b : A)
			{
				dists[count] = a.distanceTo(b);
				dists[count] += 1.0;
				count++;
			}

		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , auto &a:A | auto &b:B,store results ");

		//////// n - n distance check, object field method, yield

		start = GetTickCount();
		count = 0;
		for (auto c : distanceTO(A, B)) c += 1.0;


		end = GetTickCount();
		timeStats(start, end, "  n-n distance check , auto &a:A | auto &b:B,yield");

		///////////////////////////////////////////////////////////////
		
		start = GetTickCount();
		
		for (size_t i = 0; i < 1000; i++)
		{
			count = 0;


			for (int i = 0, cnt = 0; i < dim; i++)
				for (int j = 0, cnt = 0; j < dim; j++, cnt++)
				{
					dists[count] = distanceTo(AAA[i], BBB[j]);
					dists[count] += 1.0;
					count++;
				}
		}


		end = GetTickCount();
		timeStats(start, end, "  M (= 1000) iterations of n-n distance check , auto &a:A | auto &b:B,store results");
		
		///

		start = GetTickCount();
		count = 0;
		for (size_t i = 0; i < 1000; i++)
			for (auto c : distanceTO(A, B)) c += 1.0;


		end = GetTickCount();
		timeStats(start, end, "   M (= 1000)  iterations of n-n distance check , auto &a:A | auto &b:B,yield");



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



