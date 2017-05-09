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
int num = 1000;
#define rx ofRandom(-1,1)

struct augmentedPoint
{
	vec pt;
	int faceId;
	double normDis;
	augmentedPoint()
	{
		faceId = -1;
	}

	augmentedPoint( vec &_pt )
	{
		pt = _pt;
		faceId = -1;
	}
};

augmentedPoint *pts , *o_pts;
vec minV, maxV;
vec mean, eigenVal;
vec eigenVecs[3];
Matrix4 T, T_inv;
tri base;
vec EP[6];
vector<tri>hull;
vector<tri>hull_newFacets;

void axisAlign( augmentedPoint *pts , int num)
{
	Matrix3x3 pca;
	vec *points = new vec[num];
	for (int i = 0; i < num; i++)points[i] = pts[i].pt;

	pca.PCA(points, int(num), mean, eigenVal, eigenVecs);
	eigenVal.print();

	for (int i = 0; i < 3; i++)T.setColumn(i, eigenVecs[i].normalise());
	T.setColumn(3, mean);
	T_inv = T;
	T_inv.invert();

	for (int i = 0; i < num; i++) pts[i].pt = T_inv * pts[i].pt;
}

void minMaxAlongDir( vec dir, vec&mn, vec &mx)
{

	double min_coord = 1e10;
	double max_coord = min_coord * -1.0;
	double coord;
	for (int i = 0; i < num; i++)
	{
		coord = pts[i].pt * dir;
		if ( coord < min_coord)
		{
			min_coord = coord;
			mn = pts[i].pt;
		}

		if (coord > max_coord)
		{
			max_coord = coord;
			mx = pts[i].pt;
		}
	}
}


void cull()
{
	cout << " num before -- " << num;
	vec tri_pts[3];
	vector<augmentedPoint> includePts;

	for (int i = 0; i < num; i++)
	{
		int n = hull.size();
		for (int j = 0; j < n; j++)
		{
			double normDis = (pts[i].pt - hull[j].centroid()) * hull[j].norm();
			vec ptOnPlane = pts[i].pt - (hull[j].norm() * normDis);
			if( ! pointInPolygon(ptOnPlane, hull[j].pts, 3) )continue;

			if ( normDis > 0)
			{
				pts[i].faceId = j;
				pts[i].normDis = normDis;
				includePts.push_back(pts[i]);
				break;
			}

		}
	}

	pts = NULL;
	pts = new augmentedPoint[includePts.size()];
	for (int i = 0; i < includePts.size(); i++)pts[i] = includePts[i];
	num = includePts.size();

	cout << " num after -- " << num;
}

int findFarthestPointFromHullFace( int faceId )
{
	double maxD = 1e-10;
	int pointID_max = 0;

	for (int i = 0; i < num; i++)
	{
		if (pts[i].faceId != faceId)continue;

		if (pts[i].normDis > maxD)
		{
			maxD = pts[i].normDis;
			pointID_max = i;
		}
	}

	return maxD == 1e-10 ? -1 : pointID_max;
}

void subDivideHUllFace( int j)
{
	int n = hull.size();
	int pointId = findFarthestPointFromHullFace(j);
	if(pointId !=-1)hull[j].subDivide_center(hull_newFacets, pts[pointId].pt);
	else hull_newFacets.push_back(hull[j]);

}
//////////////////////////////////////////////////////////////////////////

void setup()
{
	num = 1000;
	pts = new augmentedPoint[num];
	o_pts = new augmentedPoint[num];
	for (int i = 0; i < num; i++) pts[i].pt = o_pts[i].pt = vec(rx, rx, rx) * 10 ;// vec(rx, rx, rx).normalise() * ofRandom(2, 10);
	

	axisAlign(pts,num);

	

	Mesh M;
	
	M.n_v = num;
	for (int i = 0; i < num; i++)M.positions[i] = pts[i].pt;
	M.boundingBox(minV, maxV);

	cout << maxV.x - minV.x << endl;
	cout << maxV.y - minV.y << endl;
	cout << maxV.z - minV.z << endl;


	minMaxAlongDir(vec(0, 0, 1), EP[0], EP[1]);
	minMaxAlongDir(vec(0, 1, 0), EP[2], EP[3]);
	minMaxAlongDir(vec(1, 0, 0), EP[4], EP[5]);

	// initial hull
	hull_newFacets.clear();
	hull.clear();
	hull.push_back(tri(EP[0], EP[1], EP[2]));
	//hull.push_back(tri(EP[1], EP[3], EP[5]));
	//hull.push_back(tri(EP[2], EP[1], EP[5]));
	//hull.push_back(tri(EP[0], EP[2], EP[5]));

	//hull.push_back(tri(EP[0], EP[1], EP[2]));
	//hull.push_back(tri(EP[0], EP[2], EP[5]));
	//hull.push_back(tri(EP[2], EP[1], EP[5]));
	////hull.push_back(tri(EP[3], EP[0], EP[6]));

	//cull();
}


void update(int value)
{
	

}


void draw()
{

	backGround(0.75);
	drawGrid(20);

	wireFrameOn();
	drawCube(minV, maxV);
	wireFrameOff();

	for (int i = 0; i < num; i++)drawPoint(o_pts[i].pt);

	glPointSize(4);
	glColor3f(0, 0, 0);
	for (int i = 0; i < num; i++)
	{
		vec4 clr = getColour( pts[i].faceId, 0, hull.size());
		glColor3f(clr.r, clr.g, clr.b);
		drawPoint(pts[i].pt);
		
		if (pts[i].faceId != -1);
		drawLine(pts[i].pt, pts[i].pt - (hull[pts[i].faceId].norm() * pts[i].normDis ));
	}
	glPointSize(1);
	glColor3f(0,0,0);



	//glLineWidth(1);
	for (int i = 0; i < 6; i += 1)
	{
		char s[200];
		sprintf(s, "%i", i);
		drawString(s, EP[i]);
	}

	wireFrameOn();
		for (auto t : hull_newFacets)t.draw( vec4(0,0,0,1));
	wireFrameOff();


	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glLineWidth(4);
	for (int i = 0; i < hull.size(); i++)
	{
		vec4 clr = getColour(i, 0, hull.size());
		glColor3f(clr.r, clr.g, clr.b);
		hull[i].draw(clr);
	}
	glLineWidth(1);

	glDisable(GL_BLEND);


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
	/*int n = hull.size();
	for (int i = 0; i < n; i++)hull[i].subDivide_center(hull, vec(rx,rx,rx));*/
	// include pts;
	if (k == 's')
	{
		hull_newFacets.clear();

		int n = hull.size();
		for (int i = 0; i < n; i++)
			subDivideHUllFace(i);
		
		//hull.erase(hull.begin(), hull.begin() + n);
		
		cout << n << " " << hull.size() << endl;
	}

	if (k == 'r')hull = hull_newFacets;
	if (k == 'c')
	{

		cull();
	}

	if (k == 'w')
	{
		Mesh M;
		for (int i = 0; i < hull.size(); i++)
			for (int j = 0; j < 3; j++)M.createVertex(hull[i].pts[j]);

		for (int i = 0; i < M.n_v -3; i += 3)
		{
			Vertex *fv[3];
			
			fv[0] = &M.vertices[i];
			fv[1] = &M.vertices[i+1];
			fv[2] = &M.vertices[i+2];
			M.createFace(fv,3);
		}

		M.writeOBJ("data/conv", "", M.positions, false);
			
	}
}





#endif // _MAIN_

