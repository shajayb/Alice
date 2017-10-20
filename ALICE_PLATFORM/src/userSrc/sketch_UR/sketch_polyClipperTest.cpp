


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


#include "clipper.hpp"
using namespace ClipperLib;

ClipType     ct = ctIntersection;
PolyFillType pft = pftEvenOdd;
JoinType jt = jtSquare;
bool show_clipping = true;
int scale = 1e4;
double delta = 0.0;

Paths sub, clp, sol;
Matrix4 T1, T2;

#define rx ofRandom( 0.1,1)
#define MAX_EDGES_IN_INTERSECTION 2000

void MakeRandomPoly(Path &p, int width, int height, int edgeCount)
{
	float r = 1;// ofRandom(8, 10);
	vec cen(rx, rx, rx);
	cen.normalise();
	cen *= r;;
	float phi = 2.0f * PI / float(edgeCount);

	p.resize(edgeCount);
	for (int i = 0; i < edgeCount; i++)
	{
		/*p[i].X = (rand() % (width - 20) + 10)*scale;
		p[i].Y = (rand() % (height - 20) + 10)*scale;*/

		p[i].X = (r * cos(phi * float(i)) + cen.x ) * scale ;
		p[i].Y = (r * sin(phi * float(i)) + cen.y) * scale ;
	}
}

void drawPath( Path &p )
{
	//for (Paths::size_type i = 0; i < pp.size(); ++i)
	//{
	//	of << pp[i].size() << "\n";
	//	if (scale > 1.01 || scale < 0.99)
	//		of << fixed << setprecision(6);
	//	for (Path::size_type j = 0; j < pp[i].size(); ++j)
	//		of << (double)pp[i][j].X / scale << ", " << (double)pp[i][j].Y / scale << ",\n";
	//}
	for (Path::size_type j = 0; j < p.size(); ++j)
	{
		int next = (j + 1) % p.size();
		drawLine( vec((double)p[j].X / scale, (double)p[j].Y / scale, 0), vec((double)p[next].X / scale, (double)p[next].Y / scale, 0) );
	}
}

void subDividePoly( vec *&p, int &n)
{
	vec *tmp = new vec[n * 2 + 1];
	int cnt = 0;
	tmp[cnt] = p[0]; cnt++;

	for (int i = 0; i < n; i++)
	{
		int next = (i + 1) % n;
		tmp[cnt] = (p[i] + p[next]) * 0.5; cnt++;
		tmp[cnt] = p[next]; cnt++;
	}

	p = tmp;
	n = cnt-1;
}

vec getAverageCentroid(vec *p1, int n1, vec *p2, int n2)
{
	vec cen;
	for (int i = 0; i < n1; i++) cen += p1[i];
	for (int i = 0; i < n2; i++) cen += p2[i];

	cen /= n1 + n2;

	return cen;
}

Matrix4 alignPolyCanonical( vec *p, int n , vec &cen)
{
	

	Matrix4 T, T_inv;



	vec u, v, norm;
	u = p[1] - p[0];
	v = p[2] - p[0];
	norm = u.cross(v).normalise();
	v = u.cross(norm).normalise();
	u.normalise();
	T.setColumn(0, u);
	T.setColumn(1, v);
	T.setColumn(2, norm);
	T.setColumn(3, cen);
	
	T_inv = T;
	T_inv.invert();
	for (int i = 0; i < n; i++) p[i] = T_inv * p[i];
	
	return T;
}

void getPolyRotFrame(Matrix3 &rot, vec * p , int n)
{
	if (n < 3)return;

	vec u, v, norm;
	u = p[1] - p[0];
	v = p[2] - p[0];
	norm = u.cross(v).normalise();
	v = u.cross(norm).normalise();
	u.normalise();

	rot.setColumn(0, u);
	rot.setColumn(1, v);
	rot.setColumn(2, norm);
}

void makePathfromPoly( Path &path,vec *p, int n )
{
	path.resize(n);
	for (int i = 0; i < n; i++)
	{
		path[i].X = p[i].x * scale;
		path[i].Y = p[i].y * scale;
	}
}

void makePolyfromPath(Path &path, vec *p) // assume P is already sized to path.size()
{
	for (int i = 0; i < path.size(); i++)
		p[i] = vec((double)path[i].X / scale, (double)path[i].Y / scale, 0);
}

void makePoly( vec *p , int n)
{
	float r = 50;
	vec cen(rx, rx, 0) ;
	cen *= 5.0;
	cen.normalise();
	cen *= r;;
	float phi = 2.0f * PI / float(n);

	for (int i = 0; i < n; i++)
		p[i] = vec( r * cos(phi * float(i) ), r * sin(phi * float(i)),0) /** ofRandom(1,2)*/ + cen;
}

void drawPoly(vec *p, int n)
{
	if (n < 3)return;

	for (int i = 0; i < n; i++)drawLine(p[i], p[(i + 1) % n]);
	for (int i = 0; i < n; i++)
	{
		char s[20];
		itoa(i, s, 10);
		drawString(s, p[i]);
	}
}


void polyIntersect(vec *p1, int n1, vec *p2, int n2 , vec *soln, int &n_edges_sol) // MAX_EDGES_IN_INTERSECTION is assumed to be greater than expected n_edges_sol
{
	
	////////////////////////////////////////////////////////////////////////// prepare polygons for clipperLib
	// get global frame of the two polygons 
	vec cen = getAverageCentroid(p1, n1, p2, n2);
	//
	vec u, v, n;
	Matrix3 rot;
	getPolyRotFrame(rot, p1, n1);
	Matrix4 T;
	for (int i = 0; i < 3; i++)T.setColumn(i, rot.getColumn(i));
	T.setColumn(3, cen);
	
	// align polygons to canonincalAxes
	T.invert();
	for (int i = 0; i < n1; i++)p1[i] = T * p1[i];
	for (int i = 0; i < n2; i++)p2[i] = T * p2[i];

	////////////////////////////////////////////////////////////////////////// execute clipper
	sub.resize(1);
	clp.resize(1);
	makePathfromPoly(sub[0], p1, n1);
	makePathfromPoly(clp[0], p2, n2);

	////

	Clipper c;
	
	delta = 0.0;

	c.AddPaths(sub, ptSubject, true);
	c.AddPaths(clp, ptClip, true);
	c.Execute(ct, sol, pft, pft);

	////////////////////////////////////////////////////////////////////////// prepare solution for Alice
	//// convert solution from Path (clipper lib) to poly
	n_edges_sol = 0;
	//cout << sol.size() << "-------soln pieces" << endl;

	if ( sol.size() > 0)	n_edges_sol = sol[0].size();

	if (n_edges_sol > MAX_EDGES_IN_INTERSECTION || n_edges_sol <= 0)
	{
		string errMsg;
		errMsg = " number of points in intersection greater than MAX_EDGES_IN_INTERSECTION ";
		n_edges_sol = 0;
		cout << errMsg.c_str() << endl;
	
	}
	else makePolyfromPath(sol[0], soln);

	//// transform all polys to global frame
	T.invert();
	for (int i = 0; i < n1; i++)p1[i] = T * p1[i];
	for (int i = 0; i < n2; i++)p2[i] = T * p2[i];

	for (int i = 0; i < n_edges_sol; i++)soln[i] = T * soln[i];

}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

vec *p1, *p2;
int n1 = 4;
int n2 = 9;


vec soln[MAX_EDGES_IN_INTERSECTION];
int ns = 0;

void setup()
{
	n1 = ofRandom(5, 50);
	n2 = ofRandom(5, 50);
	p1 = new vec[n1];
	p2 = new vec[n2];

	makePoly(p1, n1);
	makePoly(p2, n2);


}

void update(int value)
{

}

void draw()
{

	glLineWidth(1);
	backGround(0.7);
	drawGrid(20);

	//glLineWidth(1);
	//glColor3f(1, 0, 0); if(sub.size() > 0)drawPath(sub[0]);
	//glColor3f(0, 1, 0); if (clp.size() > 0)drawPath(clp[0]);


	glLineWidth(5);
	//glColor3f(0, 0, 1); if(sol.size() > 0 )drawPath(sol[0]);

	glLineWidth(1);
	glColor3f(1, 0, 0); drawPoly(p1, n1);
	glColor3f(1, 1, 0); drawPoly(p2, n2);

	glLineWidth(5);
	glColor3f(1, 0, 1); drawPoly(soln, ns);
	
	//for (int i = 0; i < 10000; i++)
	//{
	//	keyPress('R', 0, 0);
	//	keyPress('i', 0, 0);
	//}


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

	if (k == 'W')
	{
		startTimer();

		for (int i = 0; i < 10000; i++)
		{
			srand(time(NULL));
			setup();
			keyPress('i', 0, 0);
		}

		long elapse = endTimer();
		cout << elapse << endl;
	}
	
	if (k == '+')
	{
		scale *= 10;
		keyPress('i', 0, 0);
	}

	if (k == 's')
	{
		subDividePoly(p1, n1);
		subDividePoly(p2, n2);
		
	}
	
	if (k == 't')
	{
		Matrix4 T;
		vec u, v, norm;
		u = vec(rx, rx, rx);
		v = vec(rx, rx, rx);
		norm = u.cross(v).normalise();
		v = u.cross(norm).normalise();
		u.normalise();
		T.setColumn(0, u);
		T.setColumn(1, v);
		T.setColumn(2, norm);
		T.setColumn(3, vec(rx, rx, rx).normalise() * 5);
		

		for (int i = 0; i < n1; i++) p1[i] = T * p1[i];
		for (int i = 0; i < n2; i++) p2[i] = T * p2[i];
	}

	if (k == '=')
	{
		n1++;
		setup();
	}




	if (k == 'i')
	{
		polyIntersect(p1, n1, p2, n2, soln, ns);
		//cout << ns << "----" << endl;
	}

	
	
}





#endif // _MAIN_

