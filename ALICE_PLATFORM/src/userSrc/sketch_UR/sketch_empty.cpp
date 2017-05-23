
#define _MAIN_
#define _ALG_LIB_


#ifdef _MAIN_

#include "main.h"
#include "Matrix.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;


//////////////////////////////////////////////////////////////////////////


// MODEL - VIEW - CONTROLLER 


//////////////////////////////////////////////////////////////////////////
Mesh M_bldg;
Mesh site;

void setup()
{
	MeshFactory fac;
	M_bldg = fac.createFromOBJ("data/bldg.obj", 1.0, false);
	site = fac.createFromOBJ("data/site.obj", 1.0, false);
}

void update(int value)
{
	
}

void draw()
{

	backGround(0.75);
	drawGrid(20);

	M_bldg.draw();
	site.draw(true);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{
		
}

void mouseMotion(int x, int y)
{
	
}

void keyPress(unsigned char k, int xm, int ym)
{
	



}





#endif // _MAIN_

