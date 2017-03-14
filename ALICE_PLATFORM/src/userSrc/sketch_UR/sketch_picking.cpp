

#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"



#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;



// model - view - controller (MVC) paradigm / pattern / template 



int const dim = 4;
vec pts[dim];
int msx, msy;


class derivedOBJECT:public OBJECT
{
public:
	Mesh m;
	derivedOBJECT()
	{
		MeshFactory fac;
		m = fac.createPlatonic(20, 6);
		positions = m.positions;
		nv = m.n_v;
		
	}

	virtual void draw()
	{
		display();
		wireFrameOn();
			m.draw();
		wireFrameOff();
	}


};




OBJECT tmp;
derivedOBJECT MO;



//////////////////////////////////////////////////////////////////////////

void setup()
{
	double inc = 2 * PI / float(dim);
	for (int i = 0; i < dim; i++)
	{
		pts[i] = vec( sin(inc * i), cos(inc * i), 0) * 10;
	}

	tmp.positions = pts;
	tmp.nv = dim;
	/*tmp.transformationMatrix.setColumn(0, vec(1, 1, 0).normalise());
	tmp.transformationMatrix.setColumn(1, vec(1, -1, 0).normalise());
	tmp.transformationMatrix.setColumn(2, vec(0,0,1).normalise());
	tmp.transformationMatrix.transpose();*/
	//SCENE.addObject(tmp);

	MO = *new derivedOBJECT();
	MO.transformationMatrix.identity();
	SCENE.addObject(MO);

	MO.transformationMatrix.setColumn(0, vec(1, 1, 0).normalise());
	MO.transformationMatrix.setColumn(1, vec(1, -1, 0).normalise());
	MO.transformationMatrix.setColumn(2, vec(0, 0, 1).normalise());
	MO.transformationMatrix.transpose();
//	SCENE.addObject(  MO );

	
	
}

void draw()
{

	double ht = 350;
	double ht_plus = 20;

	backGround(0.75);
	glColor3f(1,0,0);
	drawGrid(20);

	

}



void mousePress(int b, int state, int x, int y)
{
}


///////////////////////////////////////////////////////////////////////////////////////////////
void update(int value)
{
}

void mouseMotion(int x, int y)
{
}

void keyPress(unsigned char k, int xm, int ym)
{
	
}





#endif // _MAIN_
