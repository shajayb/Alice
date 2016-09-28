#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
//#include<iostream>
//#include <experimental\coroutine>
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

/*
NOTE: all experimental features require that you add "/await" to the command line options in  ( properties -> c++ -> commandline options)

https://blogs.msdn.microsoft.com/vcblog/2015/04/29/more-about-resumable-functions-in-c/
https://blogs.msdn.microsoft.com/vcblog/2015/11/30/coroutines-in-visual-studio-2015-update-1/
*/



auto strip(const char *p)
{
	while (*p)
		yield *p++;
}


generator<int> anonymousList()
{
	for (int i = 0; i < 10; i++)yield i;
}





void setup()
{

	for (auto c : strip("myName")) cout << c << ".";
	cout << endl;
	
	for (auto c : anonymousList() ) cout << c << ".";
	cout << endl;
	
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



