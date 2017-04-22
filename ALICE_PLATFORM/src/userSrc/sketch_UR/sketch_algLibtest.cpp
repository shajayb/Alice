//#define _MAIN_
//#define _ALG_LIB_


#ifdef _MAIN_

#include "main.h"
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


void setup()
{

	//
	int r = 2, c = 2, i, j;
	real_2d_array Amat;
	Amat.setlength(r, c);
	Amat[0][0] = 2; Amat[0][1] = 0;
	Amat[1][0] = 0; Amat[1][1] = 2;

	real_1d_array b;// = "[-6,-4]";
	b.setlength(2);
	b[0] = -6; b[1] = -4;

	real_1d_array x;
	
	QP_SOLVE_dense(Amat, b, x);
	printf("%s ------ \n", x.tostring(2).c_str()); // EXPECTED: [3,2]
}


void update(int value)
{

}

vec normal(0, 0, 1);
void draw()
{

	backGround(0.8);
	glColor3f(0, 0, 0); 
	drawGrid(10);

	
	

	vec str(50, 150, 0);
	for (auto v : deferDrawElements)
	{
		//auto v = deferDrawElements.back();
		(v.TYP == VECTOR) ? drawVector(v.P, str, v.text) : drawString_tmp(v.text, str, true);
		str.y += 15;
	}
	deferDrawElements.clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{
	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
	{
		B.performSelection(x, y);
		S.performSelection(x, y, HUDSelectOn);
	}
}



void mouseMotion(int x, int y)
{
	{
		S.performSelection(x, y, HUDSelectOn);
	}
}



void keyPress(unsigned char k, int xm, int ym)
{

}





#endif // _MAIN_

