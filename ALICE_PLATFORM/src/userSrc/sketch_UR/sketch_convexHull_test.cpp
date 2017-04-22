


#ifdef _MAIN_

#include "main.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "graph.h"

using namespace std;
using namespace std::experimental;

// model - view - controller (MVC) paradigm / pattern / template 
#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cassert>

using namespace std;

#define MAXN 1010

typedef double vtype;

/* Basic 3D vector implementation */
struct vec3 {
	vec3() { X[0] = X[1] = X[2] = 0; }
	vec3(vtype x, vtype y, vtype z) { X[0] = x; X[1] = y; X[2] = z; }

	/* 3D cross product */
	vec3 operator*(const vec3& v) const {
		return vec3(X[1] * v.X[2] - X[2] * v.X[1],
			X[2] * v.X[0] - X[0] * v.X[2],
			X[0] * v.X[1] - X[1] * v.X[0]);
	}

	vec3 operator-(const vec3& v) const {
		return vec3(X[0] - v.X[0], X[1] - v.X[1], X[2] - v.X[2]);
	}

	vec3 operator-() const {
		return vec3(-X[0], -X[1], -X[2]);
	}

	vtype dot(const vec3& v) const {
		return X[0] * v.X[0] + X[1] * v.X[1] + X[2] * v.X[2];
	}

	vtype X[3];
};

/* Original points in the input. */
vec3 A[MAXN];

/* E[i][j] indicates which (up to two) other points combine with the edge i and
* j to make a face in the hull.  Only defined when i < j.
*/
struct twoset {
	void insert(int x) { (a == -1 ? a : b) = x; }
	bool contains(int x) { return a == x || b == x; }
	void erase(int x) { (a == x ? a : b) = -1; }
	int size() { return (a != -1) + (b != -1); }
	int a, b;
} E[MAXN][MAXN];

struct face {
	vec3 norm;
	vtype disc;
	int I[3];
};

/* Compute the half plane {x : c^T norm < disc}
* defined by the three points A[i], A[j], A[k] where
* A[inside_i] is considered to be on the 'interior' side of the face. */
face make_face(int i, int j, int k, int inside_i) {
	E[i][j].insert(k); E[i][k].insert(j); E[j][k].insert(i);

	face f;
	f.I[0] = i; f.I[1] = j; f.I[2] = k;
	f.norm = (A[j] - A[i]) * (A[k] - A[i]);
	f.disc = f.norm.dot(A[i]);
	if (f.norm.dot(A[inside_i]) > f.disc) {
		f.norm = -f.norm;
		f.disc = -f.disc;
	}
	return f;
}

////////////////////////////////////////////////////////////////////////// ////////////////////////////////////////////////////////////////////////// //////////////////////////////////////////////////////////////////////////
vector<face> faces;
int N = 10;
#define rx ofRandom(-5,5)
Mesh M;

void setup()
{
	faces.clear();

	MeshFactory fac;
	M = fac.createFromOBJ("data/convex.obj",10.0, false);

	N = M.n_v;
	for (int i = 0; i < N; i++) A[i] = vec3( M.positions[i].x, M.positions[i].y, M.positions[i].z);

	/* Initially construct the hull as containing only the first four points. */
	face f;
	
	memset(E, -1, sizeof(E));
	for (int i = 0; i < 4; i++)
		for (int j = i + 1; j < 4; j++)
			for (int k = j + 1; k < 4; k++) {
				faces.push_back(make_face(i, j, k, 6 - i - j - k));
			}

	/* Now add a point into the hull one at a time. */
	for (int i = 4; i < N; i++) {
		/* Find and delete all faces with their outside 'illuminated' by this
		* point. */
		for (int j = 0; j < faces.size(); j++) {
			f = faces[j];
			if (f.norm.dot(A[i]) > f.disc) {
				E[f.I[0]][f.I[1]].erase(f.I[2]);
				E[f.I[0]][f.I[2]].erase(f.I[1]);
				E[f.I[1]][f.I[2]].erase(f.I[0]);
				faces[j--] = faces.back();
				faces.resize(faces.size() - 1);
			}
		}
		/* Now for any edge still in the hull that is only part of one face
		* add another face containing the new point and that edge to the hull. */
		int nfaces = faces.size();
		for (int j = 0; j < nfaces; j++) {
			f = faces[j];
			for (int a = 0; a < 3; a++) for (int b = a + 1; b < 3; b++) {
				int c = 3 - a - b;
				if (E[f.I[a]][f.I[b]].size() == 2) continue;
				faces.push_back(make_face(f.I[a], f.I[b], i, f.I[c]));
			}
		}
	}
	

}


void update(int value)
{
	

}


void draw()
{

	backGround(0.8);
	glColor3f(0, 0, 0);drawGrid(10);

	
	M.draw();

	glPointSize(5);
	for (int i = 0; i < N; i++) drawPoint(vec(A[i].X[0], A[i].X[1], A[i].X[2]));
	glPointSize(1);

	wireFrameOn();
	for (int f = 0; f < faces.size(); f++)
	{
		int i;
		glBegin(GL_TRIANGLES);
			i = faces[f].I[0];glVertex3f(A[i].X[0], A[i].X[1], A[i].X[2]);
			i = faces[f].I[1]; glVertex3f(A[i].X[0], A[i].X[1], A[i].X[2]);
			i = faces[f].I[2]; glVertex3f(A[i].X[0], A[i].X[1], A[i].X[2]);
		glEnd();
	}



	wireFrameOff();

	vec str(50, 150, 0);
	for( auto v:deferDrawElements)
	{
		//auto v = deferDrawElements.back();
		(v.TYP == VECTOR ) ? drawVector(v.P,str,v.text ) : drawString_tmp(v.text,str,true);
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






int Amain() {
	int N;
	for (cin >> N; N; cin >> N) {
		for (int i = 0; i < N; i++) {
			cin >> A[i].X[0] >> A[i].X[1] >> A[i].X[2];
		}

		/* Initially construct the hull as containing only the first four points. */
		face f;
		vector<face> faces;
		memset(E, -1, sizeof(E));
		for (int i = 0; i < 4; i++)
			for (int j = i + 1; j < 4; j++)
				for (int k = j + 1; k < 4; k++) {
					faces.push_back(make_face(i, j, k, 6 - i - j - k));
				}

		/* Now add a point into the hull one at a time. */
		for (int i = 4; i < N; i++) {
			/* Find and delete all faces with their outside 'illuminated' by this
			* point. */
			for (int j = 0; j < faces.size(); j++) {
				f = faces[j];
				if (f.norm.dot(A[i]) > f.disc) {
					E[f.I[0]][f.I[1]].erase(f.I[2]);
					E[f.I[0]][f.I[2]].erase(f.I[1]);
					E[f.I[1]][f.I[2]].erase(f.I[0]);
					faces[j--] = faces.back();
					faces.resize(faces.size() - 1);
				}
			}
			/* Now for any edge still in the hull that is only part of one face
			* add another face containing the new point and that edge to the hull. */
			int nfaces = faces.size();
			for (int j = 0; j < nfaces; j++) {
				f = faces[j];
				for (int a = 0; a < 3; a++) for (int b = a + 1; b < 3; b++) {
					int c = 3 - a - b;
					if (E[f.I[a]][f.I[b]].size() == 2) continue;
					faces.push_back(make_face(f.I[a], f.I[b], i, f.I[c]));
				}
			}
		}

		/* Answer each of the queries.  Compute the minimum distance of each query
		* point to each face of the hull. */
		int Q; cin >> Q;
		for (int i = 0; i < Q; i++) {
			vec3 v; cin >> v.X[0] >> v.X[1] >> v.X[2];

			double dist = 1e300;
			for (int i = 0; i < faces.size(); i++) {
				vtype d = faces[i].disc - faces[i].norm.dot(v);
				dist = min(dist, 1. * d / sqrt(faces[i].norm.dot(faces[i].norm)));
			}
			printf("%.4f\n", dist);
		}
	}
}


#endif // _MAIN_

