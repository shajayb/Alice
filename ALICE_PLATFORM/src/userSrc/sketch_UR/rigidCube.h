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
	bool b_Fgrv,b_Fn, b_Fis, b_Fit, b_Fid;
	vec F_grv,F_is, F_id, F_it, F_if;
	double kAxial, kTan, kFriction, kBearing, kVelDamp, dia;
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

	//
	int cnt;
	int numCol;
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



		////

		getOBB(minBB, maxBB);

		////
		F = P = T = L = w = vel = vec(0, 0, 0);

		inertiaTensor.identity();
		//inertiaTensor = pow(10,5) * inertiaTensor;
		Matrix3 rotMatrix;
		rotMatrix.identity();
		q = rotMatrixToQuaternion(rotMatrix);// quaternion(0, vec(0, 0, 1));

	}

	void setInitialTransformation( Matrix4 &trans)
	{
		transMatrix = trans;
		transform();
		getOBB(minBB, maxBB);

		////
		F = P = T = L = w = vec(0, 0, 0);

		for (int i = 0; i < 3; i++) inertiaTensor.setColumn(i, trans.getColumn(i));
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

	void divideFaceIntoGrid( vec &fmn,vec axis1_diff, vec axis2_diff, int RES, vec *P)
	{

		for (int i = 0; i <= RES; i++)
		{
			for (int j = 0; j <= RES; j++)
			{
				vec pt = axis1_diff * i + axis2_diff * j;
				pt +=fmn;
				P[cnt] = pt;
				cnt++;
			}
		}
	}

	int computeGrid(vec *P, int RES)
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

		cnt = 0;
		//for (int i = 0; i <= RES; i+= RES)
		//	for (int j = 0; j <= RES; j+= RES)
		//		for (int k = 0; k <= RES; k+= RES)
		//		{
		//			//if (k != 0 && k != RES  && j!=0 && j != RES  && i != 0 && i != RES)continue;
		//			{
		//				vec pt = (x * float(i) * xInc + y * float(j) * yInc + z * float(k) * zInc);
		//				pt += mn;
		//				P[cnt] = pt;
		//				cnt++;
		//			}
		//		}

		divideFaceIntoGrid(/*vec &fmn*/mn, x*xInc, y*yInc, RES, P);
		divideFaceIntoGrid(/*vec &fmn*/mn, x*xInc, z*zInc, RES, P);
		divideFaceIntoGrid(/*vec &fmn*/mn, y*yInc, z*zInc, RES, P);

		divideFaceIntoGrid(/*vec &fmn*/mx, x*-xInc, y*-yInc, RES, P);
		divideFaceIntoGrid(/*vec &fmn*/mx, x*-xInc, z*-zInc, RES, P);
		divideFaceIntoGrid(/*vec &fmn*/mx, y*-yInc, z*-zInc, RES, P);


		////
		F_grv = vec(0, 0, -1.0);
		// per particle gravity
		F_grv /= cnt;

		return cnt;

	}

	void addSelfWeightAndTorque(vec *P)
	{
		for (int i = 0; i < cnt; i++)
		{
			if (b_Fgrv)
			{
				F += F_grv;
				T += (P[i] - cog).cross(F_grv);
			}
		}
	}

	int computeconvexHull( int face, vec *P1, vec *P2, int RES, vec *ptsConvex, stack<vec> &S)
	{
		int np = cnt;
		numCol = 0;
			for (int i = face * (RES + 1) * (RES + 1); i < (face + 1) * (RES + 1) * (RES + 1); i += 1)
			{

				bool colliding = false;
				for (int j = 0; j < np; j++)
				{
					vec relPos_ij = P1[i] - P2[j];

					if (relPos_ij * relPos_ij > pow(dia * 0.9, 2))continue;
					if (relPos_ij * relPos_ij < pow(1e-6, 2))continue;
						
					colliding = true;
					break;
				}

				if (colliding)
				{
					ptsConvex[numCol] = P1[i];
					numCol++;
				}
			}

			if (numCol > 0)convexHull(ptsConvex, numCol, S);
		
			return numCol;
	}

	vec NormalforcesOnConvexHull[50];
	vec TangentforcesOnConvexHull[50];
	float wtsOnConvex[50];
	stack<vec> Scopy; //!! REMOVE
	void computeContactsAndForces(rigidCube &R2, vec *P1, vec *P2, int RES, vec *ptsConvex , stack<vec> &S)
	{
		

		cog = transMatrix.getColumn(3);

		numCol = 0;
		vec colScale(1,1,1);

		int np = cnt;;// (RES + 1)*(RES + 1)*(RES + 1);

		int face = 0;

		for (int face = 0; face < 6; face += 1)
		{
			computeconvexHull(face, P1, P2, RES, ptsConvex, S);
			for (int i = 0; i < S.size(); i++)NormalforcesOnConvexHull[i] = TangentforcesOnConvexHull[i] = vec(0, 0, 0);
			


			for (int i = face * (RES+1) * (RES + 1); i < (face + 1) * (RES + 1) * (RES + 1); i += 1) // points on face i
			{


				for (int j = 0; j < np; j++) // points in rigidCube 2
				{
					
					//////////////////////////////////////////////////////////////////////////
					vec relPos_ij = P1[i] - P2[j];

					double dist;
					if (relPos_ij * relPos_ij > pow(dia * 0.9, 2))continue;
					if (relPos_ij * relPos_ij < pow(1e-6, 2))continue;
				

					dist = dia - relPos_ij.mag();

					vec relVel_ij = R2.vel - vel;
					vec relPos_normalised = relPos_ij / relPos_ij.mag(); /// normalise();
					vec relVel_tan = relVel_ij - relPos_normalised * (relVel_ij * relPos_normalised);


					/*F_is = (relPos_normalised)*  k * (1.0 / double(np)) * (1.0 / (dist * dist));*/
					F_is = (relPos_normalised)*  kAxial * (dist);// *(1.0 / (dist * dist));
					F_id = relVel_ij * kVelDamp;
					F_it = relVel_tan * kTan;

					// normal force;
					vec normal = transMatrix.getColumn(2);
					normal.normalise();
					vec F_n = normal * kBearing *(normal * relPos_ij);

					vec totalF;
					if (b_Fis)totalF += F_is;
					if (b_Fid)totalF += F_id;
					if (b_Fit)totalF += F_it;
					if (b_Fn)totalF += F_n; // F_is is parallel to relPos_ij , without F_id or F_it, this force will not cause torque

					F += totalF;
					T += (P1[i] - cog).cross(totalF);

					//lumping
					//barycentric(P1[i], ptsConvex, S.size(), wtsOnConvex);
					//float sum = 0;

					
					vec normalComp = normal * ((totalF- F_n) * normal);
					vec tanComp = (totalF - F_n) - normalComp;
					for (int k = 0; k < S.size(); k++)NormalforcesOnConvexHull[k] += (normalComp);// *wtsOnConvex[k];
					for (int k = 0; k < S.size(); k++)TangentforcesOnConvexHull[k] += (tanComp);// *wtsOnConvex[k];

					/*drawSphere(P1[i], vec(0,0,0),colScale * 0.05, 1, .25);*/
					//drawCircle(P1[i], 0.1, 32);
					//////////////////////////////////////////////////////////////////////////
					float scale = 0.1;
					/*F_n.normalise();
					F_it.normalise();
					F_is.normalise();
					F_id.normalise();
					glColor3f(1,0,0);   if(b_Fn)drawLine(P1[i], P1[i] + F_n * scale);
					glColor3f(0, 1, 0); if (b_Fit) drawLine(P1[i], P1[i] + F_it * scale);
					glColor3f(0, 0, 1); if (b_Fis)drawLine(P1[i], P1[i] + F_is * scale);
					glColor3f(1, 1, 0); if (b_Fid)drawLine(P1[i], P1[i] + F_id * scale);

					glColor3f(0,0, 0); drawLine(P1[i], P2[j]);*/

				}

			}

			for (int k = 0; k < S.size(); k++) drawLine(ptsConvex[k], ptsConvex[k] + NormalforcesOnConvexHull[k]);
			for (int k = 0; k < S.size(); k++) drawLine(ptsConvex[k], ptsConvex[k] + TangentforcesOnConvexHull[k]);

			drawConvexHull(S,Scopy); // this empties the stack, i.e S.size() = = , after this call.
			drawConvexHull_withMetadata(Scopy, NormalforcesOnConvexHull,S,vec4(0,0,1,1) );
			drawConvexHull_withMetadata(S, TangentforcesOnConvexHull, vec4(1, 0, 0, 1));
			

			//for (int n = 0; n < numCol; n++)drawPoint(ptsConvex[n]);
		}
		
	}

	void updatePositionAndOrientation()
	{
		
		
		// ------------- position

		vec delP = F * dt;
		P += delP;
		vel = P / mass;
		vec delX = vel * dt;
		cog += delX;
		P *= 0.99;
		// ------------- orientation

		// update angular momentum
		vec delL;
		delL = T * dt;
		L += delL;
		L *= 0.99;
		
		
		// update angular velocity
		Matrix3 R_atT;
		R_atT = q.quatToRotationMatrix();

		Matrix3 I_inverse = inertiaTensor.invert();
		Matrix3 I_atT_inverse = R_atT*I_inverse*(R_atT.transpose());
		w = I_atT_inverse * L;// *1e-04;;

		// update orientation

		double theta = (w*dt).mag();
		vec a;
		if (L.mag() > 1e-8)a = w / w.mag();
		quaternion delq = quaternion(cos(theta*0.5 * 1), a*sin(theta*0.5 * 1));
		quaternion prevQ = q;
		q = delq ^ q;

		// ------------- update rigid body

		
		//////////////////////////////////////////////////////////////////////////
		Matrix3 rotMatrix;
		for (int i = 0; i < 3; i++)rotMatrix.setColumn(i, transMatrix.getColumn(i));

		transMatrix.invert();
		transform();
		//--
		if (q != prevQ)	rotMatrix = q.quatToRotationMatrix();

		for (int i = 0; i < 3; i++)transMatrix.setColumn(i, rotMatrix.getColumn(i));
		transMatrix.setColumn(3, cog);

		transform();
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
		for (int i = 0; i < n; i++) drawPoint(P[i]);// drawLine(P[i], P[i] * 1.0001); // hack for EPS output
	}

	void drawAxes(float scale = 1.0)
	{
		glColor3f(1, 0, 0); drawLine(cen, cen + XA * scale);
		glColor3f(0, 1, 0); drawLine(cen, cen + YA * scale);
		glColor3f(0, 0, 1); drawLine(cen, cen + ZA * scale);
	}

	void drawMatrix( Matrix4 &T, vec str)
	{
		char s[200];
		glColor3f(0, 0, 0);
		setup2d();

		double width = 4 * 15;
		double ht = 24;
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
			{
				sprintf(s, "%1.2f", T[j * 4 + i]);
				drawString(s, i * width + str.x, j * ht + str.y);
			}

		restore3d();

	}
	void draw(int lineWt = 1.0, vec4 clr = vec4(0, 0, 0, 1), bool debug = false)
	{

		glColor3f(clr.r, clr.g, clr.b);
		glLineWidth(lineWt);
		//for (int i = 0; i < 24; i += 4)
		//{
		//	glBegin(GL_QUADS);

		//	for (int j = i; j < i + 4; j++)
		//		glVertex3f(pos[faces[j]].x, pos[faces[j]].y, pos[faces[j]].z);

		//	glEnd();
		//}

		glColor3f(0, 0, 0);
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
			drawAxes(1);
		//glPointSize(8); drawPoint( cen );
		glPointSize(1);

		glLineWidth(4.0);
		//glColor3f(1, 0, 0);drawLine(cen, cen + F);
		//glColor3f(0, 1, 0);	drawLine(cen, cen + T);

		glLineWidth(1.0);
		///


	}
};


#define _RIGID_CUBE_
#endif // !_RIGID_CUBE_
