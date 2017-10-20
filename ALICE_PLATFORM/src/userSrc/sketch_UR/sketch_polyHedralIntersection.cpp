


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

#include "largeMesh.h"
#include "rigidCube.h"

/////////////////////////////////////////////////////////////////////////
class polyhedralIntersections
{
#define MAX_HULL_PTS 100
public:
	int hullCnt = 0;
	vec ptConvex[MAX_HULL_PTS];
	vector<vec> pipA;
	vector<vec> pipB;
	vector<vec> lineSeg;

	polyhedralIntersections()
	{

	}

	void reset()
	{
		hullCnt = 0;
		pipA.clear();
		pipB.clear();
		lineSeg.clear();
	}

	void addPtsToConvexHull(vec &pt)
	{
		for (int i = 0; i < hullCnt; i++)
			if (pt == ptConvex[i])return;

		ptConvex[hullCnt] = pt;
		hullCnt++;
	}

	int GetIntersection(vec * segmentEndPoints, vec &pt)
	{
		double L1, L2;

		bool closeToVertPlane;
		pt = Intersect_linesegments(segmentEndPoints, L1, L2, closeToVertPlane);
		L1 += 1e-08; L2 += 1e-08;

		//printf("%1.2f,%1.2f \n", L1,L2);
		if (L1 < 1e-04 && L2 < 1e-04)return 0; // parallel
		if (L1 <= 1.01 && L2 <= 1.01  && L1 >= 0.0 &&  L2 >= 0.0)
		{
			//addPtsToConvexHull(pt);
			return 1;
		}
		return 0;

	}

	vec centroid(vec * pts_faceI, int nI)
	{
		vec cen;
		for (int i = 0; i < nI; i++) cen += pts_faceI[i] * 1.0 / nI;

		return cen;
	}

	vec normal(vec * pts_faceI , int nI)
	{

		vec cen = centroid(pts_faceI, nI);
		vec norm;
		for (int i = 0; i < nI; i++)
		{
			int next = (i + 1) % nI;
			norm += (pts_faceI[i] - cen).cross(pts_faceI[next] - pts_faceI[i]);
		}
		
		return norm.normalise();
	}

	void ComputeFaceToFaceIntersection(vec * pts_faceI, vec * pts_faceJ, int nI = 4,int nJ = 4, double incidenceTolernace = 0.05)
	{
		reset();

		// check for winding directions
		

		//// CASE 1 : COINCIDENCE 
		bool coincident = true;
		int numCoincident = 0;
		for (int a = 0; a < nI; a++)
		{
			bool found = false;
			for (int b = 0; b < nJ; b++)
				if (areClose(pts_faceI[a], pts_faceJ[b], incidenceTolernace))found = true;

			if (!found) coincident = false;
			else numCoincident++;

		}

		if (coincident) // !!!change this to append smaller face
		{
			for (int a = 0; a < nI; a++) ptConvex[hullCnt++] = pts_faceI[a];
			return;
		}


		///// CASE 2 : edge INTERSECTION/S 


		vec segmentEndPoints[4];
		vec intersectionSegment[2];
		
		
		int iCnt = 0;
		for (int a = 0; a < nI; a++)
		{

			//if (pointInPolygon(pts_faceI[a], pts_faceJ, nJ))addPtsToConvexHull(pts_faceI[a]);// if Vb_i is PIP( face_Bj) , add Vb_j to convex hull
			for (int b = 0; b < nI; b++)
			{
				if (pointInPolygon(pts_faceI[b], pts_faceJ, nJ))
				{
					//
				//	addPtsToConvexHull(pts_faceI[b]);// if Vb_j is PIP( face_Bi) , add Vb_j to convex hull
				}
			}

			
			vec pt;
			
			for (int b = 0; b < nJ; b++)
			{
				segmentEndPoints[0] = pts_faceI[a];
				segmentEndPoints[1] = pts_faceI[(a + 1) % nI];

				segmentEndPoints[2] = pts_faceJ[b];
				segmentEndPoints[3] = pts_faceJ[(b + 1) % nJ];
				int found = GetIntersection(segmentEndPoints, pt); // if edges intersect, add intersection pt to convex hull

																				/* {
																				 	glLineWidth(5);
																				 	glColor3f(1, 0, 0);drawLine(segmentEndPoints[0], segmentEndPoints[1]);
																				 	glColor3f(0, 0, 1);drawLine(segmentEndPoints[2], segmentEndPoints[3]);
																				 	glLineWidth(1);
																				 }*/

				//if (pointInPolygon(pts_faceJ[b], pts_faceI, nI))addPtsToConvexHull(pts_faceJ[b]);// if Vb_j is PIP( face_Bi) , add Vb_j to convex hull
				//if (found)break;
				if (found) lineSeg.push_back(pt);
				
			}

		}

		

		vec lastPt = ptConvex[hullCnt - 1];
		//if (pointInPolygon(ptConvex[0], pts_faceJ, nJ))hullCnt--;

		for (int b = 0; b < nJ; b++)
			if (pointInPolygon(pts_faceJ[b], pts_faceI, nI))pipA.push_back(pts_faceJ[b]);

		for (int b = 0; b < nI; b++)
			if (pointInPolygon(pts_faceI[b], pts_faceJ, nJ))pipB.push_back(pts_faceI[b]);

		//addPtsToConvexHull(intersectionSegment[1]);
		//addPtsToConvexHull(intersectionSegment[0]);
		
		
		///// !!CASE 3 : shared edge : should deal with this in face-edge intersection
		if (numCoincident == 2)
		{
			hullCnt = 0;
		}
	}

	void drawPoly( vec *pts, int n , bool drawIds = true )
	{
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i < n; i++)
			glVertex3f(pts[i].x, pts[i].y, pts[i].z);

		glVertex3f(pts[0].x, pts[0].y, pts[0].z);
		glEnd();

		drawLine( centroid(pts, n), centroid(pts, n) + normal(pts, n) );

		char s[20];
		if(drawIds)
		for (int i = 0; i < n; i++)
		{
			sprintf(s, "%i", i);
			drawString(s, pts[i] + vec(0,0,1));
		}
	}

	void drawPoly(vector<vec> pts , bool drawIds = false )
	{
		if (!pts.size() > 0)return;
		
		glBegin(GL_LINE_STRIP);
		for(auto pt : pts)
			glVertex3f(pt.x, pt.y, pt.z);

		glVertex3f(pts[0].x, pts[0].y, pts[0].z);
		glEnd();


		char s[20];
		if (drawIds)
			for (int i = 0; i < pts.size(); i++)
			{
				sprintf(s, "%i", i);
				drawString(s, pts[i] + vec(0, 0, 1));
			}
	}
};





//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define  rx ofRandom(-1,1)

polyhedralIntersections polyI;
#define ni 8
#define nj 5
vec ptsA[ni], ptsB[nj];

void setup()
{
	
	for (int i = 0; i < ni; i++)ptsA[i] = vec(cos(1.0 / float(ni) * i * 2 * PI), sin(1.0 / float(ni) * i * 2 * PI), 0) * 3;

	for (int i = 0; i < nj; i++)ptsB[i] = vec(cos(1.0 / float(nj) * i * 2 * PI), sin(1.0 / float(nj) * i * 2 * PI), 0) * 3 + vec(3, 0, 0);
	
	polyI.ComputeFaceToFaceIntersection(ptsA, ptsB, ni, nj, 0.05);
}

void update(int value)
{

}

void draw()
{

	glLineWidth(1);
	backGround(0.7);
	drawGrid(20);

	polyI.ComputeFaceToFaceIntersection(ptsA, ptsB, ni, nj, 0.05);

	polyI.drawPoly(ptsA, ni, false);
	polyI.drawPoly(ptsB, nj, false);


	glColor3f(0, 0, 1);
	glLineWidth(5);
	polyI.drawPoly(polyI.pipA,false);

	glColor3f(0, 1, 0);
	//glLineWidth(1);
	polyI.drawPoly(polyI.pipB, false);
	
	glColor3f(1, 0, 0);
	glLineWidth(5);
	polyI.drawPoly(polyI.lineSeg, false);



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
	
	Matrix4 T;
	T.rotateZ(5);
	for (int i = 0; i < nj; i++)ptsB[i] = T * ptsB[i];

}





#endif // _MAIN_

