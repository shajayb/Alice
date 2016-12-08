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
	}

	~rigidCube()
	{
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

		Matrix4 Tr;
		Tr.setColumn(0, vec(1, 1, 0).normalise());
		Tr.setColumn(1, vec(1, -1, 0).normalise());
		Tr.setColumn(2, vec(0, 0, 1));
		Tr.setColumn(3, vec(0, 0, 0.5));
		Tr.invert();
		for (int i = 0; i < 8; i++) pos[i] = Tr * pos[i];

		////

		getOBB(minBB, maxBB);

		////
		F = P = T = L = w = vec(0, 0, 0);

		inertiaTensor.identity();
		//inertiaTensor = pow(10,5) * inertiaTensor;
		Matrix3 rotMatrix;
		rotMatrix.identity();
		q = rotMatrixToQuaternion(rotMatrix);// quaternion(0, vec(0, 0, 1));

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

	void resetForces()
	{
		F = T = vec(0, 0, 0);
	}

	void computeGrid(vec *P, int RES)
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
		int cnt = 0;
		for (int i = 0; i < RES; i++)
			for (int j = 0; j < RES; j++)
				for (int k = 0; k < RES; k++)
				{
					vec pt = (x * float(i) * xInc + y * float(j) * yInc + z * float(k) * zInc);
					pt += mn;
					P[cnt] = pt;
					cnt++;
				}
		///

	}

	void computeContactsAndForces(rigidCube &R2, vec *P1, vec *P2, int RES)
	{
		
		computeGrid(P1, RES);
		R2.computeGrid(P2, RES);

		double k, kShear, kFriction, damp, dia;
		cog = transMatrix.getColumn(3);

		int np = RES*RES*RES;
		for (int i = 0; i < np; i++)
		{

			k = 0.55;// pow(10, 3.3);
			damp = 0.05;
			dia = 0.1;
			kShear = 0.01; // pow(10, 1);
			k = 0.002;
			

			for (int j = 0; j < np; j++)
			{
				vec relPos_ij = P1[i] - P2[j];

				if (relPos_ij * relPos_ij < pow(1e-6, 2))continue;
				if (relPos_ij * relPos_ij > pow(dia * 0.9, 2))continue;

				vec F_is, F_id, F_it, F_if;
				vec relVel_ij = vec(0,0, 0);// vel - R.vel;
				vec relPos_normalised = relPos_ij / relPos_ij.mag(); /// normalise();
				vec relVel_tan = relVel_ij - relPos_normalised * (relVel_ij * relPos_normalised);

				double dist = dia - relPos_ij.mag();
				/*F_is = (relPos_normalised)*  k * (1.0 / double(np)) * (1.0 / (dist * dist));*/
				F_is = (relPos_normalised)*  k * (dist);// *(1.0 / (dist * dist));
				F_id = relVel_ij * damp;
				F_it = relVel_tan * kShear;

				vec totalF = F_is + F_id + F_it; // F_is is parallel to relPos_ij , without F_id or F_it, this force will not cause torque
				/*particles[i].f_is = F_is;
				particles[i].f_id = F_id;
				particles[i].f_it = F_it;*/

				F += totalF;
				T += (P1[i] - cog ).cross(totalF);

			}
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
	
	void drawGrid( vec *P , int n)
	{
		for (int i = 0; i < n; i++)drawSphere(P[i],vec(0,0,0),vec(1,1,1),.02,1.0);
	}

	void drawGridAsPoints(vec *P, int n)
	{
		glPointSize(3);
		for (int i = 0; i < n; i++)drawLine(P[i], P[i]*1.0001); // drawPoint(P[i]);
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
		
		glLineWidth(4.0);
		glColor3f(1, 0, 0);drawLine(cen, cen + F);
		glColor3f(0, 1, 0);	drawLine(cen, cen + T);

		glLineWidth(1.0);
		///


	}
};


#define _RIGID_CUBE_
#endif // !_RIGID_CUBE_
