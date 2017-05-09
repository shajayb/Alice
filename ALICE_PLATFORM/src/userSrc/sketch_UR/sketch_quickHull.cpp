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
int num = 0;
int o_num = 0;
#define rx ofRandom(-1,1)
qh_mesh_t mesh;

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
int2 diag;
vec apexPt;
int furthestPointOnEP;
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
			double normDis = ( pts[i].pt - hull[j].centroid() ) * hull[j].norm();
			vec ptOnPlane = pts[i].pt - (hull[j].norm() * normDis);

			//if( ! pointInPolygon(ptOnPlane, hull[j].pts, 3) )continue;

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
	//else hull_newFacets.push_back(hull[j]);

}
//////////////////////////////////////////////////////////////////////////
Mesh M;
vec *P;

void setup()
{
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	num = o_num = 1000;
	pts = new augmentedPoint[num];
	o_pts = new augmentedPoint[num];

	for (int i = 0; i < num; i++) pts[i].pt = o_pts[i].pt = vec(rx, rx, rx).normalise() * 10 ;// vec(rx, rx, rx).normalise() * ofRandom(2, 10);
	
	P = new vec[num];
	for (int i = 0; i < num; i++) P[i] = vec(rx, rx, rx).normalise() * 10;

	M = quickHull(P, num);
	////////////////////////////////////////////////////////
	//num = o_num = 4 * 3;
	//pts = new augmentedPoint[num];
	//o_pts = new augmentedPoint[num];

	//vec plPts[4];
	//plPts[0] = vec(-1, -1, 0);
	//plPts[1] = vec(-1, 1, 0);
	//plPts[2] = vec(1, 1, 0);
	//plPts[3] = vec(1, -1, 0);
	////for (int i = 0; i < 4; i++)plPts[i] *= 0.1;

	//Matrix4 rot;

	//for (int i = 0; i < num; i+=4)
	//{
	//	vec n = vec(rx, rx, rx);
	//	vec u = n.cross(vec(1, 0, 0));
	//	vec v = n.cross(u);
	//	u.normalise(); v.normalise(); n.normalise();
	//	rot.setColumn(0, u);
	//	rot.setColumn(1, v );
	//	rot.setColumn(2, n );
	//	rot.setColumn(3, vec(rx, rx, rx).normalise() * 10);
	//	for (int j = 0; j < 4; j++)pts[i + j].pt = rot * plPts[j];
	//	for (int j = 0; j < 4; j++) o_pts[i + j].pt = rot * plPts[j];
	//}
	////////////////////////////////////////////////////////

	//axisAlign(pts,num); // the PCA algorithm faisl under some conditions / features of the point cloud.. such as pints being on a plane already ?

	

	Mesh M;
	
	M.n_v = num;
	for (int i = 0; i < num; i++)M.positions[i] = pts[i].pt;
	M.boundingBox(minV, maxV);

	cout << maxV.x - minV.x << endl;
	cout << maxV.y - minV.y << endl;
	cout << maxV.z - minV.z << endl;

	// 6 extreme points..
	minMaxAlongDir(vec(0, 0, 1), EP[0], EP[1]);
	minMaxAlongDir(vec(0, 1, 0), EP[2], EP[3]);
	minMaxAlongDir(vec(1, 0, 0), EP[4], EP[5]);

	// find the points forming the longest diagonal within the six EPs

	float maxD = -1 * 1e10;
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
		{
			if (i == j)continue;
			float d = EP[j].distanceTo(EP[i]);
			if (d > maxD)
			{
				d = maxD;
				diag.l = i;
				diag.n = j;
			}
		}

	// find the point in the EP furthest from the base-line / diag
	maxD = -1 * 1e10;
	vec a = EP[diag.l];
	vec b = EP[diag.n];
	vec ed = (b - a).normalise();

	for (int i = 0; i < 6; i++)
	{
		if (i == diag.l)continue;
		if (i == diag.n)continue;
		vec nearestP = a + ed * ((EP[i] - a) * ed);
		float d = EP[i].distanceTo(nearestP);

		if (d > maxD)
		{
			maxD = d;
			furthestPointOnEP = i;
		}
	}

	/// find apex = furthest point in point cloud to base traingle
	tri base = tri(a, b, EP[furthestPointOnEP]);
	maxD = -1 * 1e10;
	int apex = 0;
	for (int i = 0; i < num; i++)
	{
			double normDis = (pts[i].pt - base.centroid()) * base.norm();
			vec ptOnPlane = pts[i].pt - (base.norm() * normDis);
			//if (!pointInPolygon(ptOnPlane, base.pts, 3))continue;

			if (normDis > maxD)
			{
				apex = i;
				maxD = normDis;
			}
	}
	apexPt = pts[apex].pt;
	// initial hull
	hull_newFacets.clear();
	hull.clear();
	hull.push_back(tri( b,a, EP[furthestPointOnEP])); //flip
	hull.push_back( tri(a, b, apexPt));
	hull.push_back(tri(b, EP[furthestPointOnEP], apexPt));
	hull.push_back(tri( EP[furthestPointOnEP],a, apexPt));


	//////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////
}


void update(int value)
{
	
	//qh_vertex_t *vertices = new qh_vertex_t[num];

	//for (int i = 0; i < num; ++i) {

	//	vertices[i].z = pts[i].pt.z;
	//	vertices[i].x = pts[i].pt.x;
	//	vertices[i].y = pts[i].pt.y;
	//}

	//mesh = qh_quickhull3d(vertices, num);
	for (int i = 0; i < num; i++) P[i] = vec(rx, rx, rx).normalise() * 10;

	M = quickHull(P, num);
}


void draw()
{

	backGround(0.75);
	drawGrid(20);

	//wireFrameOn();
	//	drawCube(minV, maxV);
	//wireFrameOff();

	//////////////////////////////////////////////////////////////////////////// original points 
	//glColor3f(1, 1, 1);
	//glPointSize(10);
	////for (int i = 0; i < o_num; i++)
	////{
	////	drawCircle(o_pts[i].pt, .1, 30);
	////	drawPoint(o_pts[i].pt);
	////}

	//////////////////////////////////////////////////////////////////////////// apex 
	//glLineWidth(6);
	//	//drawCircle(apexPt,0.5, 4);
	//	//glColor3f(1, 0, 0); drawLine(EP[diag.l], EP[diag.n]);
	//	//glColor3f(0, 0, 1);	drawLine(EP[diag.l], EP[furthestPointOnEP]);
	//glLineWidth(1);
	//glPointSize(1);


	//////////////////////////////////////////////////////////////////////////// per face points

	//glPointSize(10);
	//glColor3f(0, 0, 0);
	//for (int i = 0; i < num; i++)
	//{
	//	vec4 clr = getColour( pts[i].faceId, 0, hull.size());
	//	glColor3f(clr.r, clr.g, clr.b);
	//	drawPoint(pts[i].pt);
	//	
	//	if (pts[i].faceId != -1 );
	//	drawLine(pts[i].pt, pts[i].pt - ( hull[ pts[i].faceId ].norm() * pts[i].normDis ));
	//}
	//glPointSize(1);
	//glColor3f(0,0,0);


	///// EP points
	////glLineWidth(1);
	//for (int i = 0; i < 6; i += 1)
	//{
	//	char s[200];
	//	sprintf(s, "%i", i);
	//	drawString(s, EP[i]);
	//}

	//// hull facets - new
	//wireFrameOn();
	//	for (auto t : hull_newFacets)t.draw();
	//wireFrameOff();


	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//// hull facets - previous ;
	//glLineWidth(4);
	////wireFrameOn();

	//	for (int i = 0; i < hull.size(); i++)
	//	{
	//		vec4 clr = getColour(i, 0, hull.size());
	//		glColor3f(clr.r, clr.g, clr.b);
	//		hull[i].draw();
	//	}


	////wireFrameOff();
	//glLineWidth(1);

	//glDisable(GL_BLEND);


	//// -----------------------------------------
	//glColor3f(0, 0, 0);
	//wireFrameOn();
	//	glBegin(GL_TRIANGLES);
	//	for (int i = 0, j = 0; i < mesh.nindices; i += 3, j++) 
	//	{
	//	
	//		int a = mesh.indices[i + 0];
	//		int b = mesh.indices[i + 1];
	//		int c = mesh.indices[i + 2];
	//		glVertex3f(mesh.vertices[a].x, mesh.vertices[a].y, mesh.vertices[a].z);
	//		glVertex3f(mesh.vertices[b].x, mesh.vertices[b].y, mesh.vertices[b].z);
	//		glVertex3f(mesh.vertices[c].x, mesh.vertices[c].y, mesh.vertices[c].z);
	//	}
	//	glEnd();
	//wireFrameOff();

	wireFrameOn();
		M.draw();
	wireFrameOff();

	glPointSize(5);
	for (int i = 0; i < num; i++) drawPoint(P[i]);
		glPointSize(1);
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

	if (k == ' ')
	{
		for (int i = 0; i < num; i++) pts[i].pt = o_pts[i].pt = vec(rx, rx, rx).normalise() * 10;// vec(rx, rx, rx).normalise() * ofRandom(2, 10);

	}
}





#endif // _MAIN_

