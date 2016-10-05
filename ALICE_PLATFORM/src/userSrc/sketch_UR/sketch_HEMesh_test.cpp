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

void setup()
{

	cout << sizeof(HalfVert) << endl;
	cout << sizeof(HalfEdge) << endl;
	cout << sizeof(HalfFace) << endl;
	cout << float(sizeof(HEMesh)) / (1024.0 * 1024.0) << endl;

	HEMesh M;

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



