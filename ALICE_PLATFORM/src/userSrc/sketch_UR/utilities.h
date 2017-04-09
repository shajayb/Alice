#ifndef _UTILITIES_

#include "ALICE_DLL.h"
#include "matrices.h"

inline float SIGN(float x) { return (x >= 0.0f) ? +1.0f : -1.0f; }
inline float NORM(float a, float b, float c, float d) { return sqrt(a * a + b * b + c * c + d * d); }



struct E
{
	vec t, f;

	E() {}
	E(vec &_t, vec &_f)
	{
		t = _t;
		f = _f;
	}
};


class quaternion
{
public:
	double rot;
	vec a;


	quaternion() {};
	~quaternion() {};
	quaternion(double _rot, vec _axis)
	{
		rot = _rot; a = _axis;
	}

	Matrix3 quatToRotationMatrix()
	{

		Matrix3 rotMatrix;
		double x, y, z;

		x = 1 - 2 * a.y*a.y - 2 * a.z*a.z;
		y = 2 * a.x*a.y + 2 * rot*a.z;
		z = 2 * a.x*a.z - 2 * rot*a.y;
		vec col0(x, y, z);

		x = 2 * a.x*a.y - 2 * rot*a.z;
		y = 1 - 2 * a.x*a.x - 2 * a.z*a.z;
		z = 2 * a.y*a.z + 2 * rot*a.x;
		vec col1(x, y, z);

		x = 2 * a.x*a.z + 2 * rot*a.y;
		y = 2 * a.y*a.z - 2 * rot*a.x;
		z = 1 - 2 * a.x*a.x - 2 * a.y*a.y;
		vec col2(x, y, z);


		rotMatrix.setColumn(0, col0);
		rotMatrix.setColumn(1, col1);
		rotMatrix.setColumn(2, col2);

		return rotMatrix;
	}

	void print()
	{
		printf(" s %1.2f  x %1.2f y %1.2f z %1.2f \n", rot, a.x, a.y, a.z);
	}

	quaternion operator ^ (quaternion &other)
	{
		return quaternion(rot * other.rot - a * other.a, other.a*rot + a * other.rot + a.cross(other.a));
	}

	bool operator == (quaternion &other)
	{
		return ( fabs(rot - other.rot) < 1e-4 && (a - other.a).mag() < 1e-4 );
	}

	bool operator != (quaternion &other)
	{
		return !(fabs(rot - other.rot) < 1e-4 && (a - other.a).mag() < 1e-4);
	}
};


quaternion rotMatrixToQuaternion(Matrix3 rotMatrix)
{
	double q0, q1, q2, q3;
	double r11, r22, r33, r32, r23, r13, r31, r21, r12;
	r11 = rotMatrix[0];
	r22 = rotMatrix[4];
	r33 = rotMatrix[8];
	r32 = rotMatrix[7];
	r23 = rotMatrix[5];
	r13 = rotMatrix[2];
	r31 = rotMatrix[6];
	r21 = rotMatrix[3];
	r12 = rotMatrix[1];
	//q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
	//q1 = 1 / (4.0 * q0) * (r32 - r23);
	//q2 = 1 / (4.0 * q0) * (r13 - r31);
	//q3 = 1 / (4.0 * q0) * (r21 - r12);

	q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) q0 = 0.0f;
	if (q1 < 0.0f) q1 = 0.0f;
	if (q2 < 0.0f) q2 = 0.0f;
	if (q3 < 0.0f) q3 = 0.0f;
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= SIGN(r32 - r23);
		q2 *= SIGN(r13 - r31);
		q3 *= SIGN(r21 - r12);
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= SIGN(r32 - r23);
		q1 *= +1.0f;
		q2 *= SIGN(r21 + r12);
		q3 *= SIGN(r13 + r31);
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= SIGN(r13 - r31);
		q1 *= SIGN(r21 + r12);
		q2 *= +1.0f;
		q3 *= SIGN(r32 + r23);
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= SIGN(r21 - r12);
		q1 *= SIGN(r31 + r13);
		q2 *= SIGN(r32 + r23);
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}
	double r = NORM(q0, q1, q2, q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;
	return quaternion(q0, vec(q1, q2, q3));
}

void drawSphere(vec &a, vec rotate, vec scale = vec(1, 1, 1), float r = 1.0, float alpha = 1.0)
{
	GLfloat light_pos[] = { 20, 20, 100, 1.0 };

	glPushMatrix();
	glTranslatef(a.x, a.y, a.z);

	glPushMatrix();

	//
	glRotatef(rotate.x, 1, 0, 0);
	glRotatef(rotate.y, 0, 1, 0);
	glRotatef(rotate.z, 0, 0, 1);
	glScalef(scale.x, scale.y, scale.z);



	glPushAttrib(GL_CURRENT_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	static const GLdouble equation[] = { 0, 0, 1.0, 0.0 };
	glClipPlane(GL_CLIP_PLANE0, equation);
//	glEnable(GL_CLIP_PLANE0);

	resetProjection();
	enableLight(light_pos);
	restore3d();

	glColor4f(1.0, 0.0, 0.5, alpha);
	glutSolidSphere(r, 3, 3);


	glPopMatrix();
	glPopMatrix();

	glDisable(GL_CLIP_PLANE0);
	glDisable(GL_LIGHTING);
	glDisable(GL_BLEND);

}

struct int2
{
	int n;
	int l;

	int2() {};
	int2(int _l, int _n)
	{
		
		l = _l;
		n = _n;

	}
	bool operator == (int2 &other)
	{
		return (other.n == n && other.l == l);
	}
};

////////////////////////////////////////////////////////////////////////// interface
void lineStyle(int lineType)
{
	glEnable(GL_LINE_STIPPLE);
	switch (lineType)
	{
	case 0: glLineStipple(2, 0xffff); break;
	case 1: glLineStipple(2, 0x00ff); break;
	case 2: glLineStipple(2, 0xffff); break;
	case 3: glLineStipple(2, 0x0c0f); break;
	case 4: glLineStipple(2, 0x0c0f); break;
	case 5: glLineStipple(2, 0xaaaa); break;
	case 6: glLineStipple(2, 0xaaaa); break;
	default:glLineStipple(3, 0xaaaa); break;
	}

}

void drawVector(vec&a, vec loc, string suffix)
{
	setup2d();

	char s[200];
	sprintf(s, "%1.2f,%1.2f,%1.6f : ", a.x, a.y, a.z);
	string str = s;
	str += suffix;
	drawString(str, loc);

	restore3d();
}

void drawMatrix(Matrix4 &T, vec str)
{

	char s[200];
	glColor3f(0, 0, 0);
	setup2d();

	double width = 4 * 20;
	double ht = 24;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
		{
			sprintf(s, "%1.2f", T[j * 4 + i]);
			drawString(s, i * width + str.x, j * ht + str.y);
		}

	restore3d();

}

vec rayPlaneIntersection(vec P0, vec ray, vec N, float d = 0)
{
	double t = -(P0 * N + d) / (ray * N);
	return P0 + ray * t;
}

vec screenToCamera(int x, int y, double zPlane = 0)
{
	double camera_pos[3];
	GLdouble matModelView[16], matProjection[16];
	int viewport[4];
	// get matrices and viewport:
	glGetDoublev(GL_MODELVIEW_MATRIX, matModelView);
	glGetDoublev(GL_PROJECTION_MATRIX, matProjection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	int scrCenX = (viewport[2] - viewport[0]) / 2;
	int scrCenY = (viewport[3] - viewport[1]) / 2;
	gluUnProject
	(
		scrCenX + x, scrCenY + y, zPlane, //screen coords
		matModelView, matProjection, viewport, //mvp matrices
		&camera_pos[0], &camera_pos[1], &camera_pos[2] // return pos
	);

	return vec(camera_pos[0], camera_pos[1], camera_pos[2]);
}

vec screenToWorld( vec &inPt)
{
	vec camPos_near = screenToCamera(inPt.x, inPt.y, 0.2);
	vec camPos_far = screenToCamera(inPt.x, inPt.y, 0.9);
	vec ray = camPos_far - camPos_near;
	return rayPlaneIntersection(camPos_near, ray, vec(0, 0, 1), 0);
}


vec worldToScreen(vec &a, Matrix4 &MV, Matrix4 &P, int *viewport)
{


	Vector4 inPt(a.x, a.y, a.z, 1.0);
	inPt = (MV * inPt); // inPt -> eyeSpace
	inPt = P * inPt; // eyeSpace -> clipSpace

					 // clipSpace -> normalised device coordinates
	if (fabs(inPt.w) > 1e-04)
	{
		float inv = 1.0 / inPt.w;
		inPt.x *= inv;
		inPt.y *= inv;
		inPt.z *= inv;
	}
	inPt.y *= -1; // strange inversion needed.

				  //NDC -> window coordinates
	return  vec
	(
		ofMap(inPt.x, -1, 1, viewport[0], viewport[0] + viewport[2]), // map ndc.x -> x , x+w
		ofMap(inPt.y, -1, 1, viewport[1], viewport[1] + viewport[3]), // map ndc.y -> y , y+h
		0.0
	);

}

vec worldToScreen(vec &a)
{

	Matrix4 MV, P;
	int viewport[4];
	// get matrices and viewport:
	glGetFloatv(GL_MODELVIEW_MATRIX, MV.m);
	glGetFloatv(GL_PROJECTION_MATRIX, P.m);
	glGetIntegerv(GL_VIEWPORT, viewport);
	MV.transpose();
	P.transpose();

	return worldToScreen(a, MV, P, viewport);
}

bool isInRectangle(vec &a, vec &mn, vec &mx)
{
	return (mn < a && a < mx);
}

////////////////////////////////////////////////////////////////////

// A C++ program to find convex hull of a set of points. Refer
// http://www.geeksforgeeks.org/orientation-3-ordered-points/
// for explanation of orientation()
#include <iostream>
#include <stack>
#include <stdlib.h>
using namespace std;


// A globle point needed for  sorting points with reference
// to  the first point Used in compare function of qsort()
vec p0;

// A utility function to find next to top in a stack
vec nextToTop(stack<vec> &S)
{

	vec p = S.top();
	S.pop();
	vec res = S.top();
	S.push(p);
	return res;
}

// A utility function to swap two points
void swap(vec &p1, vec &p2)
{
	vec temp = p1;
	p1 = p2;
	p2 = temp;
}

// A utility function to return square of distance
// between p1 and p2
int distSq(vec p1, vec p2)
{
	return (p1.x - p2.x)*(p1.x - p2.x) +
		(p1.y - p2.y)*(p1.y - p2.y);
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(vec p, vec q, vec r)
{
	double val = (q.y - p.y) * (r.x - q.x) -
		(q.x - p.x) * (r.y - q.y);

	if (fabs(val) < 1e-8) return 0;  // colinear
	return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// A function used by library function qsort() to sort an array of
// points with respect to the first point
int compare(const void *vp1, const void *vp2)
{
	vec *p1 = (vec *)vp1;
	vec *p2 = (vec *)vp2;

	// Find orientation
	int o = orientation(p0, *p1, *p2);
	if (o == 0)
		return (distSq(p0, *p2) >= distSq(p0, *p1)) ? -1 : 1;

	return (o == 2) ? -1 : 1;
}

// Prints convex hull of a set of n points.
void convexHull(vec points[], int n, stack<vec> &S)
{
	
	// Find the bottommost point
	int ymin = points[0].y, min = 0;
	for (int i = 1; i < n; i++)
	{
		int y = points[i].y;

		// Pick the bottom-most or chose the left
		// most point in case of tie
		if ((y < ymin) || (ymin == y &&
			points[i].x < points[min].x))
			ymin = points[i].y, min = i;
	}

	// Place the bottom-most point at first position
	swap(points[0], points[min]);

	// Sort n-1 points with respect to the first point.
	// A point p1 comes before p2 in sorted output if p2
	// has larger polar angle (in counterclockwise
	// direction) than p1
	p0 = points[0];
	qsort(&points[1], n - 1, sizeof(vec), compare);

	// If two or more points make same angle with p0,
	// Remove all but the one that is farthest from p0
	// Remember that, in above sorting, our criteria was
	// to keep the farthest point at the end when more than
	// one points have same angle.
	int m = 1; // Initialize size of modified array
	for (int i = 1; i < n; i++)
	{
		// Keep removing i while angle of i and i+1 is same
		// with respect to p0
		while (i < n - 1 && orientation(p0, points[i],points[i + 1]) == 0)i++;


		points[m] = points[i];
		m++;  // Update size of modified array
	}

//	cout << " CHULL " << m << " " << n << endl;
	// If modified array of points has less than 3 points,
	// convex hull is not possible
	if (m < 3) return;

	// Create an empty stack and push first three points
	// to it.
	//stack<vec> S;
	S.push(points[0]);
	S.push(points[1]);
	S.push(points[2]);



	// Process remaining n-3 points
	for (int i = 3; i < m; i++)
	{
		// Keep removing top while the angle formed by
		// points next-to-top, top, and points[i] makes
		// a non-left turn
		vec prev;
		while (orientation(S.size() > 1 ? nextToTop(S) : vec(0, 0, 0), S.size() > 0 ? S.top() : vec(0, 0, 0), points[i]) != 2)
			S.pop();

		S.push(points[i]);
	}

	// Now stack has the output points, print contents of stack
	//while (!S.empty())
	//{
	//	vec p = S.top();
	//	cout << "(" << p.x << ", " << p.y << ")" << endl;
	//	S.pop();
	//}
}

void drawConvexHull( stack<vec> &S, vec4 clr = vec4(1, 1, 1, 1))
{
	glColor3f(clr.r, clr.g, clr.b);

	glLineWidth(5);
	glBegin(GL_LINE_STRIP);

	int cnt = 0;
	vec bottomPt;
	while (!S.empty())
	{
		vec p = S.top();
		if (cnt == 0)
		{
			bottomPt = p;
			cnt++;
		}
		glVertex3f(p.x, p.y, p.z);

		S.pop();

	}

	glVertex3f(bottomPt.x, bottomPt.y, bottomPt.z);
	glEnd();
	glLineWidth(1);
}

void drawConvexHull(stack<vec> &S, stack<vec> &copy , vec4 clr = vec4(1, 1, 1, 1))
{
	glColor3f(clr.r, clr.g, clr.b);

	glLineWidth(5);
	glBegin(GL_LINE_STRIP);

	int cnt = 0;
	vec bottomPt;
	while (!S.empty())
	{
		vec p = S.top();
		if (cnt == 0)
		{
			bottomPt = p;
			cnt++;
		}
		glVertex3f(p.x, p.y, p.z);

		copy.push(S.top());
		S.pop();

	}

	glVertex3f(bottomPt.x, bottomPt.y, bottomPt.z);
	glEnd();
	glLineWidth(1);
}

void drawConvexHull_withMetadata(stack<vec> &S , vec *metaData, vec4 clr = vec4(1, 1, 1, 1))
{
	
	glLineWidth(5);
	glColor3f(clr.r, clr.g, clr.b);

	int cnt = 0;
	vec bottomPt;
	while (!S.empty())
	{
		vec p = S.top();
		drawLine(p, p + metaData[cnt]);
		S.pop();
		cnt++;
	}


	glLineWidth(1);
}

void drawConvexHull_withMetadata(stack<vec> &S, vec *metaData, stack<vec> &copy , vec4 clr = vec4(1,1,1,1) )
{
	glLineWidth(5);

	
	glColor3f(clr.r, clr.g, clr.b);
	int cnt = 0;
	vec bottomPt;
	while (!S.empty())
	{
		vec p = S.top();
		drawLine(p, p + metaData[cnt]);
		copy.push(S.top());
		S.pop();
		cnt++;
	}


	glLineWidth(1);
}
////////////////////////////////////////////////////////////////////////// 
// barycentric coordinates

void barycentric(vec &p, vec *q, int n, float *w )
{
	int j, prev, next;
	float weightSum = 0.f;

	// For each vertex q[j] of Q:

	//   Grab the previous and next q's and compute the barycentric weight

	for (j = 0; j < n; j++)
	{
		prev = (j + n - 1) % n;
		next = (j + 1) % n;

		float lenSq = (p - q[j]) * (p - q[j]);
		if (fabs(lenSq) < 1e-8)lenSq = 1e-4;

		w[j] = (cotangent(p, q[j], q[prev]) + cotangent(p, q[j], q[next])) / lenSq;
		weightSum += w[j];
	}

	// Normalize the weights
	if(fabs(weightSum) > 0 )
		for (j = 0; j < n; j++)w[j] /= weightSum;
		
}

enum IntersectResult { PARALLEL, COINCIDENT, NOT_INTERESECTING, INTERESECTING };
IntersectResult Intersect_segment2d(double *x, double *y, double &u, double &v)
{
	double denom = ((y[3] - y[2]) * (x[1] - x[0])) - ((x[3] - x[2]) * (y[1] - y[0]));
	double nume_a = ((x[3] - x[2]) * (y[0] - y[2])) - ((y[3] - y[2]) * (x[0] - x[2]));
	double nume_b = ((x[1] - x[0]) * (y[0] - y[2])) - ((y[1] - y[0]) * (x[0] - x[2]));

	if (fabs(denom) < 1e-06)
	{
		if (fabs(nume_a) < 1e-06 && fabs(nume_b) < 1e-06)return COINCIDENT;

		u = v = -0.0; 
		return PARALLEL; // lines are parallel
	}
	u = nume_a;// ((x[3] - x[2]) * (y[0] - y[2])) - ((y[3] - y[2]) * (x[0] - x[2]));
	u /= denom;// ((y[3] - y[2]) * (x[1] - x[0])) - ((x[3] - x[2]) * (y[1] - y[0]));

	v = nume_b;// ((x[1] - x[0]) * (y[0] - y[2])) - ((y[1] - y[0]) * (x[0] - x[2]));
	v /= denom;// ((y[3] - y[2]) * (x[1] - x[0])) - ((x[3] - x[2]) * (y[1] - y[0]));

	if (u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f)return INTERESECTING;

		return NOT_INTERESECTING;
}

vec Intersect_linesegments( vec *pts, vec &normal , double &u, double &v,bool project = true )
{
	double x[4], y[4];
	for (int i = 0; i < 4; i++)
	{
		if(project)pts[i] = pts[i] - normal * (pts[i] * normal); // project to face i
		x[i] = pts[i].x;
		y[i] = pts[i].y;

	}

	Intersect_segment2d(x, y, u, v);
	return pts[0] + (pts[1]-pts[0])* u; 
}

vec Intersect_linesegments(vec *pts, double &u, double &v, bool project = true) // pts is of length 4
{

	/*for (int i = 0; i < 4; i++)swap(pts[i].x, pts[i].z);*/

	vec normal = (pts[1] - pts[0]).cross(pts[3] - pts[2]).normalise();
	vec pt = Intersect_linesegments( pts, normal, u,v, project) ;
	
	//swap(pt.x, pt.z);
	//for (int i = 0; i < 4; i++)swap(pts[i].x, pts[i].z);
	
	return pt;
}

vec Intersect_linesegments(vec *pts , double &lambda)
{
	vec v1 = (pts[1] - pts[0]);// .normalise();
	vec v2 = (pts[3] - pts[2]);// .normalise();

	if (fabs(v1.angle(v2)) < 1e-04)
	{
		lambda = -1; // parallel;
		return vec(0,0,0);
	}


	double l1 = v1.mag();
	double l2 = v2.mag();
	v1 *= 1.0/l1; v2 *= 1.0/l2;

	vec LHS = (v1.cross(v2));
	vec RHS = (pts[2] - pts[0]).cross(v2);
	double denom = LHS.mag();
	lambda = denom > 0 ? ( RHS.mag() / denom ) : -1.0;
	
	if (fabs(LHS.angle(RHS) - 180.0) < 1e-04 && fabs(lambda) > 1e-04 )lambda *= -1;
	vec intPt = pts[0] + v1 * lambda;

	double param1, param2;

	param1 = intPt.distanceTo(pts[2]) / l2;
	param2 = intPt.distanceTo(pts[3]) / l2;
	/*param1 = (intPt - pts[2])*(intPt - pts[2]) / (l1*l1);
	param1 = (intPt - pts[3])*(intPt - pts[3]) / (l2*l2);*/
	//printf("%1.8f,%1.8f, %1.8f, %1.8f,%1.8f \n", param1,param2, LHS.angle(RHS), denom,lambda);
	
	if ( param1 > 1.0f + EPS || param2 > 1.0 + EPS )lambda = -1.0; // greater than computes to true without the EPS addition
	return (intPt);
}
vec pointInNewBasis( vec inPt , vec &u, vec &v, vec &n, vec &c)
{
	vec rpt = inPt;
	rpt -= c; // translate
	rpt = vec(rpt*u, rpt*v, rpt * n); // project to bases to get coordinates
	return rpt;
}

vec pointInNewBasis(vec inPt,Matrix4 T)
{
	vec bases[4];
	for (int i = 0; i < 4; i++)bases[i] = T.getColumn(i);

	return pointInNewBasis(inPt, bases[0], bases[1], bases[2], bases[3]);;
}

bool pointInPolygon(vec &pt,vec *pts , int n)
{
	float *wts = new float[n];
	barycentric(pt, pts, n, wts);
	
	double sum = 0.0;
	for (int i = 0; i < n; i++)sum += wts[i];

	vec rpt;
	for (int i = 0; i < n; i++) rpt += pts[i] * wts[i]; // this is a weird test , as the barycentric weights seem to add up to 1, even if p is outside.

	return ((pt - rpt)*(pt - rpt) <  pow(1e-4,2) ) ? true : false;
}


//////////////////////////////////////////////////////////////////////////

void drawString_tmp(string &s, vec pt , bool twoD = false)
{
	if (twoD)setup2d();

	unsigned int i;
	glRasterPos3f(pt.x, pt.y, pt.z);

	for (i = 0; i < s.length(); i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s[i]);

	if (twoD)restore3d();
}


void drawString_tmp(double a, vec pt)
{
	char c[200];
	sprintf(c, "%1.8f ", a);
	string s = "";
	s += c;

	drawString_tmp(s, pt);
}

void drawString_tmp(int a, vec pt)
{
	char c[200];
	sprintf(c, "%i ", a);
	string s = "";
	s += c;

	drawString_tmp(s, pt);
}


enum Elementtype { VECTOR, TEXT };
struct deferDrawElement
{
	vec P;
	string text;
	Elementtype TYP;
	deferDrawElement(Elementtype _typ,vec &p, string _suffix = "")
	{
		TYP = _typ;
		P = p;
		text = _suffix;
	}

	deferDrawElement(Elementtype _typ,string str = " string : ")
	{
		TYP = _typ;
		text = str;
	}
};
vector<deferDrawElement> deferDrawElements;

void deferDraw_addElement( vec p , string suffix = "" )
{
	deferDrawElements.push_back( deferDrawElement(VECTOR,p,suffix) );
}

void deferDraw_addElement( string suffix = "")
{
	deferDrawElements.push_back( deferDrawElement(TEXT,suffix) );
}

void deferDraw_addElement(double r , string suffix = "" )
{
	char s[20];
	sprintf(s, " %1.2f ", r);
	string str = s;
	str += suffix;

	deferDrawElements.push_back(deferDrawElement(TEXT, suffix));
}

//////////////////////////////////////////////////////////////////////////

bool areClose( vec &a,vec &b, double tol)
{
	vec c = b - a;
	return(fabs(c.x) < tol && fabs(c.y) < tol && fabs(c.z) < tol);
}

//////////////////////////////////////////////////////////////////////////

long start_time;
long end_time;

void startTimer()
{
	start_time = GetTickCount();
}

long endTimer()
{
	end_time = GetTickCount();
	long elapsedTime = end_time - start_time;
	if (elapsedTime < 10)elapsedTime = 10;
	return elapsedTime;
}


//////////////////////////////////////////////////////////////////////////

vector<string> splitString(const string& str, const string& delimiter)
{
	vector<string> elements;
	// Skip delimiters at beginning.
	string::size_type lastPos = str.find_first_not_of(delimiter, 0);
	// Find first "non-delimiter".
	string::size_type pos = str.find_first_of(delimiter, lastPos);

	while (string::npos != pos || string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		elements.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiter, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiter, lastPos);
	}
	return elements;
}

#define _UTILITIES_
#endif // !_UTILITIES_


