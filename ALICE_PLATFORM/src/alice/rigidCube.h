

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

	//
	vec NormalforcesOnConvexHull[50];
	vec TangentforcesOnConvexHull[50];
	float wtsOnConvex[50];
	stack<vec> Scopy; //!! REMOVE
	vec ptConvex[20];
	int hullCnt = 0;

	//////////////////////////////// CONSTRUCTORS -------------------------------------------------------------------------------------------- 

	rigidCube()
	{
	}

	~rigidCube()
	{
	}

	rigidCube( float radius , int sides = 6 )
	{
		MeshFactory fac;
		Mesh M = fac.createPlatonic( radius / sqrt(2), sides);
	

		extractFromMesh(M);
	}

	rigidCube( Mesh &M )
	{
		extractFromMesh(M);
	}

	void extractFromMesh( Mesh &M)
	{
		Matrix4 transM;
		getTopology(M);
		getInvertRotateToAABB(M, transM);// cube needs to be rotated back to align with cardinal axes


		for (int i = 0; i < 8; i++) pos[i] = M.positions[i];
		for (int i = 0; i < 8; i++) pos[i] = transM * pos[i];

		setInitialTransformation(transMatrix);
		//transMatrix = transM.identity();
		transMatrix.identity();
	}

	void setInitialTransformation( Matrix4 &trans)
	{
		transMatrix = trans;
		transform();
		getOBB(minBB, maxBB);

		////
		F = P = T = L = w = vec(0, 0, 0);
		cog = transMatrix.getColumn(3);
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
				//F += F_grv;
				//T += (P[i] - cog).cross(F_grv);
				F += vec(0,0,-1e-04);
			}
		}
	}
	int computeconvexHull( int face, vec *P1, vec *P2, int RES, vec *ptsConvex, stack<vec> &S)
	{
		
		//transMatrix.getBasisVectors(XA, YA, ZA, cen);
		//vec v1, v2;
		//getBasisOfFace(face, v1, v2);
		//v1.normalise(); v2.normalise();

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
					ptsConvex[numCol] = P1[i]; // v1 * (P1[i] * v1) + v2 * (P1[i] * v2);
					numCol++;
				}
			}

			if (numCol > 0)convexHull(ptsConvex, numCol, S);
		
			return numCol;
	}
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
					barycentric(P1[i], ptsConvex, S.size(), wtsOnConvex);
					//float sum = 0;

					
					vec normalComp = normal * ((totalF- F_n) * normal);
					vec tanComp = (totalF - F_n) - normalComp;
					for (int k = 0; k < S.size(); k++)NormalforcesOnConvexHull[k] += (normalComp) * wtsOnConvex[k] * 0.01;
					for (int k = 0; k < S.size(); k++)TangentforcesOnConvexHull[k] += (tanComp) * wtsOnConvex[k] * 0.01;

					/*drawSphere(P1[i], vec(0,0,0),colScale * 0.05, 1, .25);*/
					//drawCircle(P1[i], 0.1, 32);
					//////////////////////////////////////////////////////////////////////////
					float scale = 0.1;
					F_n.normalise();
					F_it.normalise();
					F_is.normalise();
					F_id.normalise();
					
					//if( i == face * (RES + 1) * (RES + 1) + RES  * RES * 0.5 )
					{
						/*glColor3f(1, 0, 0);   if (b_Fn && F_n.mag() > 0.1)drawLine(P1[i], P1[i] + F_n * scale);
						glColor3f(0, 1, 0); if (b_Fit  && F_it.mag() > 0.1) drawLine(P1[i], P1[i] + F_it * scale);
						glColor3f(0, 0, 1); if (b_Fis && F_is.mag() > 0.1)drawLine(P1[i], P1[i] + F_is * scale);
						glColor3f(1, 1, 0); if (b_Fid && F_id.mag() > 0.1)drawLine(P1[i], P1[i] + F_id * scale);*/

						glColor3f(0, 0, 0); drawLine(P1[i], P2[j]);
					}

				}

			}

			/*for (int k = 0; k < S.size(); k++) drawLine(ptsConvex[k], ptsConvex[k] + NormalforcesOnConvexHull[k]);
			for (int k = 0; k < S.size(); k++) drawLine(ptsConvex[k], ptsConvex[k] + TangentforcesOnConvexHull[k]);*/

			drawConvexHull(S, Scopy,vec4(.25, .25, 0.,1.)); // this empties the stack, i.e S.size() = = , after this call.
			//drawConvexHull_withMetadata(Scopy, NormalforcesOnConvexHull,S,vec4(0,0,1,1) );
			//drawConvexHull_withMetadata(S, TangentforcesOnConvexHull, vec4(1, 0, 0, 1));
			
			glColor3f(0.1, 0, 0.1);
			for (int n = 0; n < numCol; n++)drawLine(ptsConvex[n], ptsConvex[n]); // for eps out
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
		P *= 0.9;
		// ------------- orientation

		// update angular momentum
		vec delL;
		delL = T * dt;
		L += delL;
		L *= 0.9;
		
		
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
	//////////////////////////////////////////////////////////////////////////
	vec FunctionAtPoint( vec &P , vector<vec> &HullPts , vec &normal /*unit normal expected*/)
	{

		vec F_is;
		for (auto pOther : HullPts)
		{
			//pOther += normal * 0.1; 
			vec relPos_ij = (P - pOther) /** -1.0*/;;

			double dist;
			/*dia = 0.5;
			if (relPos_ij * relPos_ij > pow(dia * 0.9, 2))continue;
			if (relPos_ij * relPos_ij < pow(1e-6, 2))continue;*/
			if (relPos_ij * relPos_ij < pow(1e-6, 2))continue;

			dist = relPos_ij.mag();
			vec relPos_normalised = relPos_ij / dist; /// normalise();
			F_is += (relPos_normalised);// *(dist);// *(1.0 / (dist * dist));
			F_is *= 0.1; 
		}

		return F_is;
	}
	
	//------

	vec getAngularMomentumContribution( vec &torque , vec &ra )
	{

		Matrix3 R_atT;
		R_atT = q.quatToRotationMatrix();

		Matrix3 I_inverse = inertiaTensor.invert();
		Matrix3 I_atT_inverse = R_atT*I_inverse*(R_atT.transpose());
		return ( (I_atT_inverse )* torque.cross(ra) );// *1e-04;;
	}
	void updateCOG()
	{
		extractFrame();
		cog = cen;
	}

	void computeRestingForces(real_1d_array &x , vec &n, rigidCube &r2 , float currentmatStrength)
	{
		real_2d_array Amat;
		real_1d_array b;
		r2.updateCOG();
		//updateCOG();


		Amat.setlength(hullCnt, hullCnt);
		b.setlength(hullCnt);
		x.setlength(hullCnt);

		vec Wa, Wb; //weight vector of Body A,B
		Wa = Wb = vec(0, 0, -1);
		
		for (int i = 0; i < hullCnt; i++)b[i] = (Wa + Wb) * n;// *((i % 2 == 0) ? 1 : 1); Wa
		

		for (int i = 0; i < hullCnt; i++)
			for (int j = 0; j < hullCnt; j++)
			{
		/*		if (i == j)
				{
					Amat[i][j] = 0.0;
					continue;
				}*/

				vec forceOnA, forceOnB;
				vec torqueOnA, torqueOnB;
				vec aAng, bAng;
				vec ra, rb;
				//vec PA, PB;
				//PA = ptConvex[i]; PB = ptConvex[j];

				forceOnA = n;
				forceOnB = n * -1;

				ra = ptConvex[i] - cog;
				rb = ptConvex[i] - r2.cog ;

				torqueOnA = (ptConvex[j] - cog).cross(forceOnA);
				torqueOnB = (ptConvex[j] - r2.cog).cross(forceOnB);

				aAng = (torqueOnA).cross(ra);//  getAngularMomentumContribution(torqueOnA, ra);
				bAng = (torqueOnB).cross(rb);// r2.getAngularMomentumContribution(torqueOnB, rb);

				Amat[i][j] = n * ( forceOnA - forceOnB + aAng - bAng);
			
			}

		//
		//for (int i = 0; i < hullCnt; i++)ptConvex[i] += cog;
		//r2.cog += cog;

		QP_SOLVE_dense(Amat, b, x);// solves 0.5 * Xt *A * x + bt *x ;
		for (int i = 0; i < hullCnt; i++)x[i] *= 2.0;
		//printf("\n%s\n ---- a\n ", x.tostring(hullCnt).c_str()); // EXPECTED: [3,2]
		
		vec netT,netF;
		glLineWidth(4);
		
		for (int i = 0; i < hullCnt; i++)
		{
			netT += (n * (x[i])).cross(ptConvex[i] - r2.cog);
			netF += (n * (x[i]));
			vec fi = (n * ofClamp(x[i], -1, 1) );
			glColor3f(1, 0.65, 0);  drawLine(ptConvex[i], ptConvex[i] + fi);

			char s[200];
			sprintf(s, "f");
			drawString(s, ptConvex[i] + (n * (ofClamp(x[i], -1, 1) - 0.1)));

			glColor3f(0, 1, 0); drawLine(ptConvex[i], ptConvex[i] - fi * currentmatStrength * 1.1 );
		}
		glLineWidth(1);
		glColor3f(0, 0, 0);
		netF += (Wa + Wb);
		F = netF;
		T = netT;

		//cog.print();
		//cout << "cog" << endl;
		//r2.cog.print();
		//cout << "r2Cog" << endl;
		//netT.print();
		//netF.print();

		deferDraw_addElement(netT, "net T on r2");
	/*	for (int i = 0; i < hullCnt; i++)
		{
			cout << endl;
			for (int j = 0; j < hullCnt; j++)
			{
				cout << Amat[i][j] << ",";
			}
		}*/
	}
	void computeIntegratedContactForces( vec &n  )
	{
		if (hullCnt < 3)return;
		// subDivide convex hull to compute F(x) at gauss quadrature points;
		vector<vec> subPts;
		vector<tri> Ts = subDivideHull(ptConvex, hullCnt, subPts, 2);
		
		subPts.clear();
		for (auto Tr : Ts)subPts.push_back(Tr.centroid());

		for (auto P : subPts)drawPoint(P);
		//for (auto Tr : Ts)Tr.draw();

		// gauss quadrature integration per triangle 
		n.normalise();
		vector<tri> Tris;
		for (int i = 1; i < hullCnt - 1; i++)
			Tris.push_back(tri(ptConvex[i], ptConvex[(i + 1) % hullCnt], ptConvex[0]));
		for (auto T : Tris)T.draw();


		vec C;
		for (int i = 0; i < hullCnt; i++)C += ptConvex[i];
		C /= hullCnt;
		C += n * 0.1;
		drawCircle(C, .02, 32);


		/*for (int i = 1; i < hullCnt - 1; i++)
		{
			vec P0 = (ptConvex[i] + ptConvex[0]) * 0.5;
			vec P1 = (ptConvex[i + 1] + ptConvex[0]) * 0.5;
			vec P2 = (ptConvex[i] + ptConvex[i + 1]) * 0.5;
			vec Force;
			Force += FunctionAtPoint(P0, subPts, n) * 0.33;
			Force += FunctionAtPoint(P1, subPts, n) * 0.33;
			Force += FunctionAtPoint(P2, subPts, n) * 0.33;

			vec cen = (ptConvex[i] + ptConvex[0] + ptConvex[i + 1]) / 3.0;

			F += Force;
			double mult = 1000.0;
			T += Force.cross(cog - cen) * mult;

			glLineWidth(4);
			glColor3f(1, 0, 0);
			drawLine(cen, cen + Force * 30);
			glLineWidth(1);
			glColor3f(0, 0, 0);

			drawCircle(P0, .02, 32);
			drawCircle(P1, .02, 32);
			drawCircle(P2, .02, 32);
		}*/
		cen = transMatrix.getColumn(3);
		T = vec(0, 0, 0);
		
		//for (int i = 0; i < hullCnt; i++)
		for (auto P : subPts)
		{
			//vec P = ptConvex[i];
			vec Force;
			Force = vec(0, 0, 0.1);// FunctionAtPoint(P, subPts, n);// vector-valued function F(p) ;
			T += Force.cross(P - cog);
			glColor3f(1, 0, 0);; drawLine(P, P + (Force.cross(P - cen)));
			//glColor3f(0,0,1); drawLine(P, P + (Force));
		}

		deferDraw_addElement(T, "torqueNet");
	}

	void addPtsToConvexHull( vec &pt)
	{
		for (int i = 0; i < hullCnt; i++)
			if (pt == ptConvex[i])return;

		ptConvex[hullCnt] = pt;
		hullCnt++;
	}

	vec pts_faceI[4], pts_faceJ[4];
	bool computeRestingContacts = false;
	bool isFacetoFace( rigidCube &r2, int i, int j, double distancetoPlaneTolerance = 0.2 , float currentmatStrength = 0.2 )
	{

		// -----------------------------------------  get face i data;
		vec u, v, n, c; Matrix4 transFace;
		getTransformationToFace(i, transFace);
		transFace.getBasisVectors(u, v, n, c);

		// ----------------------------------------- ---- cast face J from global unto frame of face I
		// return false if all points are NOT within tolerance along normal direction.
		//  iterate through pts of face j
		
		r2.getFacePoints(j , pts_faceJ);
		
		bool found = true; 
		for (int o = 0; o < 4; o++)
			if (fabs(pointInNewBasis(pts_faceJ[o], transFace).z) > distancetoPlaneTolerance)found = false;
		

		if (!found)return false; 

		char s[200];
		sprintf(s, "Within zRange %i & %i", i, j);
		deferDraw_addElement(s);
		// -----------------------------------------  collect facePts_I , project facePts_J to face i , etc
		getFacePoints(i, pts_faceI);
		//for (int o = 0; o < 4; o++)pts_faceJ[o] = pointInNewBasis(pts_faceJ[o], transFace);
		for (int o = 0; o < 4; o++)pts_faceJ[o] -= n * ( n * (pts_faceJ[o]-c) ) ; // project facePts_J to plane of facePts_j;
		

		//glLineWidth(5);
		//glColor3f(1, 0, 0); for (int p = 0; p < 4; p++)drawLine(pts_faceI[p], pts_faceI[(p + 1) % 4]);
		//glColor3f(0, 0, 1); for (int p = 0; p < 4; p++)drawLine(pts_faceJ[p], pts_faceJ[(p + 1) % 4]);
		//glLineWidth(1);
		//return false;

		// ----------------------------------------- computer convex hull of edge-edge intersections and pointsInPolygons;
		ComputeFaceToFaceIntersection(pts_faceI, pts_faceJ,0.0051);
		ComputeConvexHull(transFace);

		// ----------------------------------------- computer convex hull of edge-edge intersections and pointsInPolygons;

		//computeIntegratedContactForces(n);
		if( hullCnt >=3 )
		{
			
			real_1d_array x;
			if(computeRestingContacts)computeRestingForces(x, n, r2 , currentmatStrength);


		}
		else
		{
			glLineWidth(4);
			glColor3f(1, 0, 1);
			if (computeRestingContacts)drawLine(cen, cen+vec(0, 0, -0.75));
			char s[200];
			sprintf(s, "W");
			//drawString(s, cen + vec(0, 0, -0.8));
			glLineWidth(1);

			
		}

		glLineWidth(5);
		glColor3f(1, 0.25, 0.25);
			DrawConvexHull();
		glLineWidth(1);
		
		// ----------------------------------------- drawConvex hull
	


		return true;
	}

	double areaofConvexHUll()
	{
		double area = 0.0;
		for (int i = 1; i < hullCnt-1; i++)
		{
			area += 0.5 * (ptConvex[i] - ptConvex[0]).cross(ptConvex[i+1] - ptConvex[0]).mag();
		}
		return area;
	}
	void ComputeFaceToFaceIntersection(vec * pts_faceI, vec * pts_faceJ , double incidenceTolernace = 0.05)
	{
		hullCnt = 0;

		//// CASE 1 : COINCIDENCE 
		bool coincident = true;
		int numCoincident = 0;
		for (int a = 0; a < 4; a++)
		{
			bool found = false;
			for (int b = 0; b < 4; b++)
				if (areClose(pts_faceI[a], pts_faceJ[b], incidenceTolernace))found = true;

			if (!found) coincident = false;
			else numCoincident++;

		}

		if ( coincident )
		{
			for (int a = 0; a < 4; a++) ptConvex[hullCnt++] = pts_faceI[a];
			return;
		}

		
		deferDraw_addElement( "	not coincident");
		///// CASE 2 : edge INTERSECTION/S 
	/*	{
			for (int a = 0; a < 4; a++)
			{
				char s[200];
				sprintf( s, "%i" , a );
				drawString(s, (pts_faceI[a] + pts_faceI[(a + 1) % 4])* 0.5);
			}

			for (int a = 0; a < 4; a++)
			{
				char s[200];
				sprintf( s, "%i", a );
				drawString(s, (pts_faceJ[a] + pts_faceJ[(a + 1) % 4]) * 0.5 );
			}
		}*/

		vec segmentEndPoints[4];

		for (int a = 0; a < 4; a++)
		{

			if (pointInPolygon(pts_faceI[a], pts_faceJ, 4))addPtsToConvexHull(pts_faceI[a]);// if Vb_i is PIP( face_Bj) , add Vb_j to convex hull

			for (int b = 0; b < 4; b++)
			{
				segmentEndPoints[0] = pts_faceI[a];
				segmentEndPoints[1] = pts_faceI[(a + 1) % 4];

				segmentEndPoints[2] = pts_faceJ[b];
				segmentEndPoints[3] = pts_faceJ[(b + 1) % 4];
				CheckAndAddToConvexHull(segmentEndPoints, pts_faceJ, pts_faceI); // if edges intersect, add intersection pt to convex hull

				//{
				//	glLineWidth(5);
				//	glColor3f(1, 0, 0);drawLine(segmentEndPoints[0], segmentEndPoints[1]);
				//	glColor3f(0, 0, 1);drawLine(segmentEndPoints[2], segmentEndPoints[3]);
				//	glLineWidth(1);
				//}

				if (pointInPolygon(pts_faceJ[b], pts_faceI, 4))addPtsToConvexHull(pts_faceJ[b]);// if Vb_j is PIP( face_Bi) , add Vb_j to convex hull
			}

		}

		char s[200];
		sprintf(s, "     %i pts added to hull", hullCnt);
		deferDraw_addElement(s);

		///// !!CASE 3 : shared edge : should deal with this in face-edge intersection
		if (numCoincident == 2)
		{
			hullCnt = 0;
			deferDraw_addElement("     shared edge ");
		}
	}

	void ComputeConvexHull(Matrix4 &T)
	{
		if (!(hullCnt >=3 ))return;

		for (int i = 0; i < hullCnt; i++)ptConvex[i] = pointInNewBasis(ptConvex[i], T); // transform to 2d;
		
		
		convexHull(ptConvex, hullCnt, Scopy);

		hullCnt = 0;
		while (!Scopy.empty())
		{
			vec p = Scopy.top();
			p = T * p;
			ptConvex[hullCnt++] = p; 
			Scopy.pop();
		}
	}

	void DrawConvexHull()
	{
		if (hullCnt < 3)return;

		for (int i = 0; i < hullCnt; i++)
		{
			//drawCircle(ptConvex[i], 0.01, 32);
			drawLine(ptConvex[i], ptConvex[(i + 1) % hullCnt]);
			char s[200];
			sprintf(s, "%i", i);
			//drawString(s, ptConvex[i]);
		}
	}

	void CheckAndAddToConvexHull(vec * segmentEndPoints, vec * pts_faceJ, vec * pts_faceI)
	{
		double L1, L2;
		
		bool closeToVertPlane;
		vec pt = Intersect_linesegments(segmentEndPoints, L1, L2, closeToVertPlane);
		L1 += 1e-08; L2 += 1e-08;

		//printf("%1.2f,%1.2f \n", L1,L2);
		if (L1 < 1e-04 && L2 < 1e-04)return; // parallel
		if (L1 <= 1.01 && L2 <= 1.01  && L1 >= 0.0 &&  L2 >= 0.0)addPtsToConvexHull(pt);
		
		//---------------------------------------------------------------------------------
		//double lambda;
		//vec pt =Intersect_linesegments(segmentEndPoints, lambda);
		//
		//if (( lambda) <= 1.0 + EPS &&  lambda >= 0.0 )addPtsToConvexHull(pt); // comparison of floats seems to need EPS addition !!!
	}

	void computeCollisionInterfaces(rigidCube &R2, double zTol = 0.2)
	{

		for (int i = 0; i < 6; i++)
			for (int j = 0; j < 6; j++)
			{
				if (isFacetoFace(R2, i, j, zTol));
				//else if (/*isFaceToEdge*/)
				
			}
		
		//isFacetoFace(R2, 2, 0, zTol);

	}
	//////////////////////////////// UTILITIES  -------------------------------------------------------------------------------------------- 

	void getFacePoints(int face, vec *pts_face)
	{
		face *= 4;
		for (int o = face, cnt = 0; o < face + 4; o++, cnt++)pts_face[cnt] = pos[faces[o]];
	}
	void getTransformationToFace(int i, Matrix4 &transFace)
	{
		vec u, v, n, c;
		getBasisOfFace(i, n, v);
		u = v.cross(n);
		c = getfaceCenter(i);

		transFace.setColumn(0, u);
		transFace.setColumn(1, v);
		transFace.setColumn(2, n);
		transFace.setColumn(3, c);
	}
	void getBasisOfFace(int face, vec &v1, vec &v2)
	{
		extractFrame();

		if (face == 0)
		{
			v1 = XA; v2 = ZA;
		}
		if (face == 1)
		{
			v1 = YA; v2 = ZA;
		}
		if (face == 2)
		{
			v1 = XA*-1; v2 = ZA;
		}
		if (face == 3)
		{
			v1 = YA*-1; v2 = ZA;
		}
		if (face == 4)
		{
			v1 = ZA * 1; v2 = XA;
		}
		if (face == 5)
		{
			v1 = ZA * -1; v2 = XA;
		}
	}
	vec getfaceCenter(int i)
	{
		i *= 4;
		vec fc;
		for (int j = i; j < i + 4; j++) fc += pos[faces[j]];
		fc /= 4;
		return fc;
	}
	void getInvertRotateToAABB( Mesh &M , Matrix4 &Tr )
	{
		
		vec x, y, z;
		x = vec(1, 1, 0).normalise();
		z = vec(0, 0, 1).normalise();
		y = x.cross(z).normalise();

		Tr.setColumn(0, x);
		Tr.setColumn(1, y);
		Tr.setColumn(2, z);
		Tr.setColumn(3, vec(0, 0, 0.5));
		Tr.invert();
		
	}
	void getTopology( Mesh &M)
	{
		for (int i = 0; i < 8; i++)pos[i] = M.positions[i];

		int eCnt = 0;
		int *fv;

		for (int i = 0; i < M.n_f; i++)
		{
			int n = M.faces[i].n_e;
			fv = M.faces[i].faceVertices();

			for (int j = 0; j < n; j++)
			{
				faces[eCnt] = fv[j];
				eCnt++;
			}
		}

	}
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
	void setScale(float uniform)
	{
		for (int i = 0; i < 3; i++)
			transMatrix.setColumn(i, transMatrix.getColumn(i).normalise()*uniform);

	}
	void setScale(double x, double y, double z)
	{
		float sc[3];
		sc[0] = x; sc[1] = y; sc[2] = z;
		setScale(sc);
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
	void writeOBJ()
	{
		MeshFactory fac;
		Mesh M = fac.createPlatonic(1.0 / sqrt(2), 6);

		for (int i = 0; i < 8; i++)M.positions[i] = pos[i];
		M.writeOBJ("data/test.obj", "", M.positions, false);
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
		/*glColor3f(1, 0, 0); drawLine(cen, cen + XA * scale);
		glColor3f(0, 1, 0); drawLine(cen, cen + YA * scale);
		glColor3f(0, 0, 1); drawLine(cen, cen + ZA * scale);*/
		drawAxes(XA, YA, ZA, cen,scale);
	}
	void drawAxes( vec &u, vec &v, vec &n, vec &c, float scale = 1.0 )
	{
		glColor3f(1, 0, 0); drawLine(c, c + u * scale);
		glColor3f(0, 1, 0); drawLine(c, c + v * scale);
		glColor3f(0, 0, 1); drawLine(c, c + n * scale);

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


		glColor3f(0, 0, 0);
		for (int i = 0; i < 24; i += 4)
		{
			glBegin(GL_LINE_STRIP);//glBegin(GL_QUADS);

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
				drawString_tmp(s, pos[i]);

			}


	
		///
		glLineWidth(1.0);
			extractFrame();
			drawAxes(0.1);
		//glPointSize(8); drawPoint( cen );
		glPointSize(1);

		glLineWidth(4.0);
		
		/*glColor3f(1, 0, 0.6);drawLine(cen, cen + F);
		glColor3f(1, 0, 0.6);drawLine(cen, cen + T);*/

		glLineWidth(1.0);
		///

		/*vec u, v, n;
		glPointSize(5);
		glLineWidth(2.0);

		for (int i = 0; i < 6; i++)
		{
			getBasisOfFace(i, n, v);
			u = v.cross(n);

			drawAxes(u, v, n, getfaceCenter(i), 0.2);
			drawPoint( getfaceCenter(i));

			char chr[200];
			sprintf(chr, "%i ", i);
			string str = "";
			str += chr;
			drawString_tmp(str, getfaceCenter(i)+ vec(0.01,0.01,0.01));
			drawCircle(getfaceCenter(i), .025, 32);
		}*/

		glPointSize(1.0);
		glLineWidth(1.0);

	}
};


#define _RIGID_CUBE_
#endif // !_RIGID_CUBE_
