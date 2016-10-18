#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
//#include<experimental/generator> 
//using namespace std;
//using namespace std::experimental;
using namespace std;

class HalfVert;
class HalfEdge;
class HalfFace;


class HalfEdge
{
public:

	HalfVert *ve;
	HalfEdge *twin;
	HalfEdge *next;
	HalfFace *f;
};

class HalfVert
{
public:

	HalfEdge he;
};

class HalfFace
{
public:
	HalfEdge *e;
};

const int HE_MAX = 60000;
const int HV_MAX = 60000;
const int HF_MAX = 60000;

class HEMesh
{
public:
	
	array<vec, HV_MAX> positions;
	array<HalfEdge, HE_MAX> HE;
	array<HalfVert, HV_MAX> HV;
	array<HalfFace, HF_MAX> HF;

};

void timeStats(long &strt, long& end, string str)
{
	elapsedTime = end - strt;
	cout << elapsedTime / 1000.0f << " - " << str << endl;
}

const int sz = 200000;

//array<vec, sz> A;
//array<vec, sz> B;


void setup()
{

	//cout << sizeof(HalfVert) << endl;
	//cout << sizeof(HalfEdge) << endl;
	//cout << sizeof(HalfFace) << endl;
	//cout << float(sizeof(HEMesh)) / (1024.0 * 1024.0) << endl;

	//HEMesh M;

	vec *AA = new vec[sz];
	vec *BB = new vec[sz];
	cout << float(sizeof(vec)) * sz / (1024.0 * 1024.0) << endl;

	long start, end;

	//start = GetTickCount();;

	//for( auto &a: A)
	//	for (auto &b : B)
	//		double d = a*(b);

	//end = GetTickCount();
	//timeStats(start, end, "  n-n distance check static array L1 , object field method : a.distanceTo(b) , store results ");

	//// 

	start = GetTickCount();;
	for (int i = 0; i < sz; i++)
		for (int j = 0; j < sz; j++)
			double d = AA[i]*(BB[j]);

	end = GetTickCount();
	timeStats(start, end, "  n-n distance check dyn array, object field method : a.distanceTo(b) , store results ");


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



