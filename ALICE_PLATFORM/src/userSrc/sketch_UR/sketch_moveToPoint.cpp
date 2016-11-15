#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 

using namespace std;
using namespace std::experimental;



// model - view - controller (MVC) paradigm / pattern / template 
vec A = vec( 1, 1, 0);
vec B = vec( 1, -1, 0);
vec C = vec(-1, -1, 0);
vec D = vec( -1, 1, 0);
vec tmp;

// arrayList ~ vector<>;
vector<vec> collectionOfPoints;
vector<vec> BFR;

void update(int value)
{

}


void setup()
{
	collectionOfPoints.clear(); 
	BFR.clear();

	 A = vec(1, 1, 0);
	 B = vec(1, -1, 0);
	 C = vec(-1, -1, 0);
	 D = vec(-1, 1, 0);
	// divide AB into 5 segments
	// add points to collectionOfPoints

	for (int i = 0; i <= 6; i+= 1)
	{
		vec e = B - A; // calculate difference vector
		vec unitC = (B - A) / (B - A).mag(); // calc. unit vector
		float totalDistance = e.mag(); // calc total distance
		float incrementalDistance = (totalDistance / 6.0) * i; // totalDist times i
		tmp = A + unitC * incrementalDistance;// tmp = new point 
		// put tmp into global collection of points
		collectionOfPoints.push_back(tmp);
		BFR.push_back(tmp);
	}

	//
	for (int i = 0; i <= 6; i += 1)
	{
		vec e = C - B; // calculate difference vector
		vec unitC = (C - B) / (C - B).mag(); // calc. unit vector
		float totalDistance = e.mag(); // calc total distance
		float incrementalDistance = (totalDistance / 6.0) * i; // totalDist times i
		tmp = B + unitC * incrementalDistance;// tmp = new point 
											  // put tmp into global collection of points
		collectionOfPoints.push_back(tmp);
		BFR.push_back(tmp);
	}

	//

	for (int i = 0; i <= 6; i += 1)
	{
		vec e = D - C; // calculate difference vector
		vec unitC = (D - C) / (D - C).mag(); // calc. unit vector
		float totalDistance = e.mag(); // calc total distance
		float incrementalDistance = (totalDistance / 6.0) * i; // totalDist times i
		tmp = C + unitC * incrementalDistance;// tmp = new point 
											  // put tmp into global collection of points
		collectionOfPoints.push_back(tmp);
		BFR.push_back(tmp);
	}
	
	
}

void draw()
{

	backGround(0.75);

	glPointSize(10);
	//
	//drawPoint(A);
	//drawPoint(B);
	//drawPoint(C);
	//drawPoint(D);



	glColor3f(1, 0, 0);
	for (auto pt : collectionOfPoints)
	{
		drawPoint(pt);
	}

	
	//collectionOfPoints[idOfNborFront]


}



void mousePress(int b, int state, int x, int y)
{


}


///////////////////////////////////////////////////////////////////////////////////////////////

void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{

	///for each pt in collection
	// check if both neighbors exist
	//	get location of the forward and backward neighbor
	//	call the locations Pf,Pb
	//	calculate average (Pf + Pb) * 0.5 ;
	//	calculate deltaA ;
	//  A = A + deltaA

	if (k == 'r')
	{
		setup();
	}

	if (k == 'a')
	{

		for (int i = 1; i < collectionOfPoints.size() - 1; i++)
		{
			int idOfNborFront = i + 1;
			int idOfNborBack = i - 1;
			vec Pf = collectionOfPoints[idOfNborFront];// cop[2] 
			vec Pb = collectionOfPoints[idOfNborBack];// cop[0]
			vec A = collectionOfPoints[i]; //cop[1] ;
			vec Avg = (Pf + Pb) * 0.5;
			vec diff = Avg - A;
			vec unitDiff = diff / diff.mag();

			if (diff.mag() < 0.001) continue;

			A = A + unitDiff * 0.1;
			// put back / update the collectionOfPoints
			// because.. what is being drawin is the collection
			// not what we are calculating within the FOR loop.
			collectionOfPoints[i] = A;
		}

		//collectionOfPoints = BFR;
	}
}




