#ifndef _RIGID_CUBE_
#include "main.h"
#include "ALICE_DLL.h"
#include "matrices.h"
#include "utilities.h"

class rigidCube
{
public:

	vec pos[8];
	int faces[24];
	vec XA, YA, ZA, cen;
	Matrix4 transMatrix;
	vec minBB, maxBB;

	//// forces
	vec *pts;
	int np;
	int RES = 10;

	double dt = 0.01;
	vec cog;
	vec F, T;
	vec P; // linear momentum
	vec vel;
	double mass = 1.0;

	quaternion q;
	vec L; // angular momentum
	vec w;// angular vel;
	Matrix3 inertiaTensor;

	
	//////////////////////////////// CONSTRUCTORS -------------------------------------------------------------------------------------------- 

	rigidCube()
	{
		//pts = new vec[RES*RES*RES];
	}

	~rigidCube()
	{
		//if(pts != NULL)delete[] pts;
	}

	rigidCube( Mesh &M )
	{

		for (int i = 0; i < 8; i++)pos[i] = M.positions[i];

		int eCnt = 0;
		int *fv;

		for (int i = 0; i < M.n_f; i++)
		{
			int n = M.faces[i].n_e;
			// = new int[n];
			fv = M.faces[i].faceVertices();

			for (int j = 0; j < n; j++)
			{
				faces[eCnt] = fv[j];
				eCnt++;
			}
			cout << endl;
		}

		///
		transMatrix.identity();

		Matrix4 T;
		T.setColumn(0, vec(1, 1, 0).normalise());
		T.setColumn(1, vec(1, -1, 0).normalise());
		T.setColumn(2, vec(0, 0, 1));
		T.setColumn(3, vec(0, 0, 0.5));
		T.invert();
		for (int i = 0; i < 8; i++) pos[i] = T * pos[i];

		////

		getOBB(minBB, maxBB);

		///
		pts = new vec[RES*RES*RES];
		np = RES*RES*RES;
	}

	//////////////////////////////// COMPUTE -------------------------------------------------------------------------------------------- 
	void getOBB(vec &mn, vec &mx)
	{
		mn = vec(1e12, 1e12, 1e12);
		mx = mn * -1;
		vec x, y, z, c;
		transMatrix.getBasisVectors(x, y, z, c);
		x.normalise(); y.normalise(); z.normalise();

		for (int i = 0; i < 8; i++)
		{

			vec ptInOrientedBasis = pos[i];
			ptInOrientedBasis -= c;
			ptInOrientedBasis = vec(ptInOrientedBasis * x, ptInOrientedBasis * y, ptInOrientedBasis*z);


			mn.x = MIN(mn.x, ptInOrientedBasis.x);
			mn.y = MIN(mn.y, ptInOrientedBasis.y);
			mn.z = MIN(mn.z, ptInOrientedBasis.z);

			mx.x = MAX(mx.x, ptInOrientedBasis.x);
			mx.y = MAX(mx.y, ptInOrientedBasis.y);
			mx.z = MAX(mx.z, ptInOrientedBasis.z);
		}
	}

	//////////////////////////////// UTILITIES  -------------------------------------------------------------------------------------------- 

	void setTransformation(Matrix4 &_transMatrix)
	{
		transMatrix = _transMatrix;
	}

	void setTransformation(vec &XA_, vec &YA_, vec &ZA_, vec &cen_)
	{
		transMatrix.setColumn(0, XA_);
		transMatrix.setColumn(1, YA_);
		transMatrix.setColumn(2, ZA_);
		transMatrix.setColumn(3, cen_);
	}

	void extractFrame()
	{
		transMatrix.getBasisVectors(XA, YA, ZA, cen);
	}

	void setScale(float scale[3])
	{
		for (int i = 0; i < 3; i++)
			transMatrix.setColumn(i, transMatrix.getColumn(i).normalise()*scale[i]);

	}

	void transform()
	{
		for (int i = 0; i < 8; i++) pos[i] = transMatrix * pos[i];
	}

	void inverseTransform()
	{
		transMatrix.invert();
		transform();
		transMatrix.identity();
	}

	//////////////////////////////// DISPLAY  -------------------------------------------------------------------------------------------- 
	void reDimensionComputeGrid( int _res)
	{
		RES = _res;
		pts = new vec[RES*RES*RES];

	}
	
	void computeContacts( rigidCube &R2 )
	{
		computeGrid();
		R2.computeGrid();
	}
	void computeGrid()
	{

		vec x, y, z, c;
		transMatrix.getBasisVectors(x, y, z, c);
		x.normalise(); y.normalise(); z.normalise();

		getOBB(minBB, maxBB);

		vec mn = x * minBB.x + y * minBB.y + z * minBB.z;
		vec mx = x * maxBB.x + y * maxBB.y + z * maxBB.z;

		mn += c;
		mx += c;

		float xInc, yInc, zInc;
		vec diff = mx - mn;
		diff /= float(RES);
		xInc = diff * x; yInc = diff* y; zInc = diff * z;

		glPointSize(1);
		glColor3f(.2, .2, .2);
		int cnt = -1;
		for (int i = 0; i < RES; i++)
			for (int j = 0; j < RES; j++)
				for (int k = 0; k < RES; k++)
				{
					vec pt = (x * float(i) * xInc + y * float(j) * yInc + z * float(k) * zInc);
					pt += mn;
					pts[cnt++] = pt; 
				}
		///
		np = cnt-1;
	}

	void drawGrid()
	{
		//computeGrid();
		for (int i = 0; i < np; i++)drawPoint(pts[i]);

	}

	void drawAxes(float scale = 1.0)
	{
		glColor3f(1, 0, 0); drawLine(cen, cen + XA * scale);
		glColor3f(0, 1, 0); drawLine(cen, cen + YA * scale);
		glColor3f(0, 0, 1); drawLine(cen, cen + ZA * scale);
	}

	void draw(int lineWt = 1.0, vec4 clr = vec4(0, 0, 0, 1), bool debug = false)
	{

		glColor3f(clr.r, clr.g, clr.b);
		glLineWidth(lineWt);
		for (int i = 0; i < 24; i += 4)
		{
			glBegin(GL_LINE_STRIP);

			for (int j = i; j < i + 4; j++)
				glVertex3f(pos[faces[j]].x, pos[faces[j]].y, pos[faces[j]].z);

			glEnd();
		}


		if (debug)
			for (int i = 0; i < 8; i++)
			{
				char c[200];
				sprintf(c, "%i ", i);
				string s = "";
				s += c;
				drawString(s, pos[i]);

			}

	
		///
		glLineWidth(1.0);
			extractFrame();
			drawAxes(.2);
		
		///
		drawGrid();

	}
};


#define _RIGID_CUBE_
#endif // !_RIGID_CUBE_
