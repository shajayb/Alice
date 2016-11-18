#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;


class particle
{
public:
	vec P, V;
	vector<vec> Pos;

	void setup()
	{

	}

	void inverskeKinematics(Matrix4 tool)
	{

	}

	int draw()
	{

		return 1 ;
	}

	vector<int> outputCollection()
	{
		vector<int> A;
		vector<int> B;
		for (int i = 0; i < 100; i++)A.push_back( ofRandom(-5,5) );

		return A;
	}

};
