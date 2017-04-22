

#include "main.h"
//#include "ao.h"
//#include "mesh.h" // AO works with older alice_dll.h, without mesh included 
//#include "particlelib.h"
//#include "slider.h"
//#include "particlelib.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/barycenter.h>
#include <igl/cotmatrix.h>
#include <igl/massmatrix.h>
#include <igl/grad.h>
#include <igl/doublearea.h>
#include <igl/repdiag.h>
#include <igl/jet.h>
#include <igl/per_vertex_normals.h>

#include <igl/boundary_facets.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/jet.h>
#include <igl/massmatrix.h>
#include <igl/min_quad_with_fixed.h>


#include <igl/barycenter.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/per_face_normals.h>
#include <igl/slice.h>
#include <igl/avg_edge_length.h>
#include <igl/file_dialog_open.h>
#include <igl/local_basis.h>


using namespace std;
using namespace Eigen;



Eigen::MatrixXd V;
Eigen::MatrixXi F;


typedef complex<double> Cdouble;
typedef SparseMatrix<Cdouble> SpMat;
typedef Triplet<Cdouble> T;
vector<int> freeFaces, fixedFaces;
vector<int> freeVerts, fixedVerts;
vector<vec> fixedVectors;
Eigen::SparseMatrix<double> G;

SpMat A;
MatrixXcd Ui;
MatrixXd fv;
double minV;
double maxV;
Mesh M;
MeshFactory fac;
//SliderGroup S;

Cdouble test;
vector<vec> fieldVectors;
vector<vec> face_centers;
RowVector3d bb_min, bb_max, center;

double numConstraints = 3;

vec a, b, c;
vec TP1, TP2;
MatrixXd B1, B2, B3;

# define EIGEN_ERROR_SPARSE(solver) \
if (solver.info() != Eigen::Success)\
{\
	cout << " sparse solver error " << endl; \
}\



void scaleToFitUnitBox(MatrixXd &P)
{

	bb_min = P.colwise().minCoeff();
	bb_max = P.colwise().maxCoeff();

	RowVector3d  diag = bb_max - bb_min;
	double scaleFactor = (2.0 * sqrtf(3.0)) / diag.norm();
	RowVector3d center = (bb_min + bb_max) * 0.5;

	//translate to 0,0,0 ;
	for (int i = 0; i < P.rows(); i++)P.row(i) -= center;

	MatrixXd scalingMatrix;
	scalingMatrix.resize(3, 3);
	scalingMatrix.setIdentity();
	scalingMatrix *= scaleFactor * 10;

	//scale to fit unit box ;
	P *= scalingMatrix;


}

Cdouble C_conjugate(Cdouble &in)
{
	return Cdouble(in.real(), -1 * in.imag());
}

double C_Modulus(Cdouble &in)
{
	return (pow(in.real(), 2.0) + pow(in.imag(), 2.0));
}

void matricesToMesh(Mesh &M, MatrixXd &V, MatrixXi &F)
{

	vec *p = new vec[V.rows()];
	int *polyCounts = new int[F.rows()];
	int *polyConects = new int[F.rows() * 3];

	for (int i = 0; i < V.rows(); i++)
	{
		p[i] = vec(V(i, 0), V(i, 1), V(i, 2));
		p[i] *= 0.1;
	}
	int cnt = 0;
	for (int i = 0; i < F.rows(); i++)
	{
		polyCounts[i] = 3;
		polyConects[cnt] = F(i, 0); cnt++;
		polyConects[cnt] = F(i, 1); cnt++;
		polyConects[cnt] = F(i, 2); cnt++;
	}


	M = fac.createFromArrays(p, V.rows(), polyCounts, F.rows(), polyConects, false);

}


Cdouble getComplexEdgeBasis(int f_id, Mesh &M, vec &commonEdge, vec &b1 = vec(0, 0, 0), vec &b2 = vec(0, 0, 0), vec &b3 = vec(0, 0, 0))
{
	//int fv[3];
	//for (int i = 0; i < 3; i++) fv[i] = M.faces[f_id].f_v[i];
	//vec a = M.positions[fv[0]];
	//vec b = M.positions[fv[1]];
	//vec c = M.positions[fv[2]];

	// b1 = (b-a).normalise();
	// b3 = b1.cross(c-a).normalise();
	// b2 = b3.cross(b1).normalise();
	int index = f_id;
	b1 = vec(B1(index, 0), B1(index, 1), B1(index, 2));
	b2 = vec(B2(index, 0), B2(index, 1), B2(index, 2));
	b3 = vec(B3(index, 0), B3(index, 1), B3(index, 2));

	vec newEd = vec(commonEdge * b1, commonEdge * b2, commonEdge * b3);

	Cdouble ed_complex(newEd.x, newEd.y);
	return ed_complex;
}

vec getVecFromComplexEdgeAndTriangle(Cdouble &u, int f_id, Mesh &M, vec &b1 = vec(0, 0, 0), vec &b2 = vec(0, 0, 0), vec &b3 = vec(0, 0, 0))
{
	//int fv[3];
	//for (int i = 0; i < 3; i++) fv[i] = M.faces[f_id].f_v[i];
	//vec a = M.positions[fv[0]];
	//vec b = M.positions[fv[1]];
	//vec c = M.positions[fv[2]];

	//b1 = (b - a).normalise();
	//b3 = b1.cross(c - a).normalise();
	//b2 = b3.cross(b1).normalise();

	int index = f_id;

	b1 = vec(B1(index, 0), B1(index, 1), B1(index, 2));
	b2 = vec(B2(index, 0), B2(index, 1), B2(index, 2));
	b3 = vec(B3(index, 0), B3(index, 1), B3(index, 2));

	vec ed_r3 = b1 * (u.real()) + b2 * (u.imag()) + b3 * 0.;
	//ed_r3 += a;
	return ed_r3;
}

SpMat subMatrix(SpMat &C, vector<int> &nodes)
{
	SpMat C_sub(C.rows(), nodes.size());
	for (int i = 0; i < nodes.size(); i++)C_sub.col(i) = C.col(nodes[i]);
	return C_sub;
}

Eigen::SparseMatrix<double> subMatrix(Eigen::SparseMatrix<double> &C, vector<int> &nodes)
{
	Eigen::SparseMatrix<double> C_sub(C.rows(), nodes.size());
	for (int i = 0; i < nodes.size(); i++)C_sub.col(i) = C.col(nodes[i]);
	return C_sub;
}

SpMat subMatrix_rows(SpMat &C, vector<int> &nodes)
{
	SpMat C_sub(nodes.size(), C.cols());
	for (int i = 0; i < nodes.size(); i++)C_sub.row(i) = C.row(nodes[i]);
	return C_sub;
}

MatrixXcd subMatrix_rows(MatrixXcd &C, vector<int> &nodes)
{
	MatrixXcd C_sub(nodes.size(), C.cols());
	for (int i = 0; i < nodes.size(); i++)C_sub.row(i) = C.row(nodes[i]);
	return C_sub;
}

MatrixXd subMatrix_rows(MatrixXd &C, vector<int> &nodes)
{
	MatrixXd C_sub(nodes.size(), C.cols());
	for (int i = 0; i < nodes.size(); i++)C_sub.row(i) = C.row(nodes[i]);
	return C_sub;
}

MatrixXd subMatrix(MatrixXd &C, vector<int> &nodes)
{
	MatrixXd C_sub(C.rows(), nodes.size());
	for (int i = 0; i < nodes.size(); i++)C_sub.col(i) = C.col(nodes[i]);
	return C_sub;
}

MatrixXcd subMatrix(MatrixXcd &C, vector<int> &nodes)
{
	MatrixXcd C_sub(C.rows(), nodes.size());
	for (int i = 0; i < nodes.size(); i++)C_sub.col(i) = C.col(nodes[i]);
	return C_sub;
}

bool boundaryConstraint = true;
void interpolateField(int numConstraints)
{


	/////////////////////////////////// ///////////////////////////////////  construct fixed and free face index sets ;

	fixedFaces.clear();
	freeFaces.clear();
	int cnt = 0;
	if (boundaryConstraint)
		for (int i = 0; i < M.n_f; i++)
			(M.faces[i].onBoundary() && fixedFaces.size() < numConstraints) ? fixedFaces.push_back(i) : freeFaces.push_back(i);
	else
	{
		for (int i = 0; i < numConstraints; i++)fixedFaces.push_back(i); //int(ofRandom(0, M.n_f)

		for (int i = 0; i < M.n_f; i++)
		{
			bool found = false;
			for (int j = 0; j < fixedFaces.size(); j++)
				if (fixedFaces[j] == i)
				{
					found = true;
					break;
				}
			if (!found)freeFaces.push_back(i);
		}
	}

	printf(" free %i, fixed %i, sum %i \n", freeFaces.size(), fixedFaces.size(), M.n_f);


	/////////////////////////////////// ///////////////////////////////////  get & set constraint vectors in local face basis ;

	MatrixXcd B;
	B.resize(fixedFaces.size(), 1);

	for (int i = 0; i < fixedFaces.size(); i++)
	{
		vec constraintVec;
		int f = fixedFaces[i];

		if (boundaryConstraint)
			for (int j = 0; j < M.faces[fixedFaces[i]].n_e; j++)
			{
				if (!M.faces[fixedFaces[i]].edgePtrs[j]->onBoundary()) continue;

				int vStr_Id = M.faces[fixedFaces[i]].edgePtrs[j]->vStr->id;
				int vEnd_Id = M.faces[fixedFaces[i]].edgePtrs[j]->vEnd->id;
				constraintVec = M.positions[vEnd_Id] - M.positions[vStr_Id];
				break;
			}
		else
			constraintVec = vec(B1(f, 0), B1(f, 1), B1(f, 2)); // first basis vector ;

		constraintVec = constraintVec.cross(vec(B3(f, 0), B3(f, 1), B3(f, 2))); // rotate constraintVec by 90 in the plane
		Cdouble e_f = getComplexEdgeBasis(f, M, constraintVec);
		B.row(i) << e_f; // C_conjugate(e_f);
	}

	/////////////////////////////////// INTERPOLATION ON MESH FACES /////////////////////////////////// 
	///////////////////////// minimize squared sum of smoothness energy per-face pair across each edge /////////////////////////////////// 
	/////////////////////////////////// /////////////////////////////////// construct A_fg * Conjugate Transpose (A_fg)
	/////////////////////////////////// /////////////////////////////////// this amounts to product : (#F x #e) * ( #e x #F )

	vector<T> coefs_Fe, coefs_eF;
	coefs_Fe.clear(); coefs_eF.clear();

	SpMat sp_Fe, sp_eF;
	sp_Fe.resize(M.n_f, M.n_e);
	sp_eF.resize(M.n_e, M.n_f);


	for (int i = 0; i < M.n_e; i++)
	{
		if (M.edges[i].lFace == NULL || M.edges[i].rFace == NULL)continue;

		int f = M.edges[i].lFace->id;
		int g = M.edges[i].rFace->id;

		vec commonEdge = M.positions[M.edges[i].vEnd->id] - M.positions[M.edges[i].vStr->id];;
		Cdouble e_f = getComplexEdgeBasis(f, M, commonEdge);
		Cdouble e_g = getComplexEdgeBasis(g, M, commonEdge);

		coefs_Fe.push_back(T(f, i, e_f));
		coefs_Fe.push_back(T(g, i, e_g*-1.0));

		coefs_eF.push_back(T(i, f, C_conjugate(e_f)));
		coefs_eF.push_back(T(i, g, C_conjugate(e_g)*-1.0));

	}

	sp_Fe.setFromTriplets(coefs_Fe.begin(), coefs_Fe.end());
	sp_eF.setFromTriplets(coefs_eF.begin(), coefs_eF.end());

	SpMat An = sp_Fe * sp_eF; // (A_fg) * (A_fg).conjugate transpose 
	An += An.transpose();
	MatrixXcd denseAn(An); // this conversion is needed currently, because I don't know how to slice a sparse,column major matrix along its rows.
	// i.e subMatrix_rows : doesn't work for sparse matrices ; also IGL::slice doesn't seem to be supported of SparseMatrix< std::complex<double> >
	SpMat An_stripped = subMatrix_rows(denseAn, freeFaces).sparseView(); //remove known equations i.e keep equations related to unknown face indices
	SpMat A_ii = subMatrix(An_stripped, freeFaces); // separate the known and unknown coefs / columns [ A_ii A_ib] = [ X_ii X_ib]^T ;
	SpMat A_ib = subMatrix(An_stripped, fixedFaces);// separate the known and unknown coefs / columns
	MatrixXcd D = A_ib * B * -1.0; // constraint equations = RHS  :  A_ii * U_i = - A_ib * B ;

	SimplicialLLT< SpMat > solver; // sparse cholesky solver
	solver.compute(A_ii); // compute cholesky factors
	Ui = solver.solve(D); // solve AX = B ;

	/////////////////////////////////// /////////////////////////////////// 
	///////////////////////////////////END / INTERPOLATION /////////////////////////////////// 

	/////////////////////////////////// ///////////////////////////////////  update display 
	// interpolated vectors 
	face_centers.clear();
	fieldVectors.clear();
	vec b1, b2, b3;
	for (int i = 0; i < Ui.rows(); i++)
	{
		Cdouble u = Ui(i, 0);
		vec cen = M.faces[freeFaces[i]].centroid(M.positions);
		vec ed_r3 = getVecFromComplexEdgeAndTriangle(u, freeFaces[i], M, b1, b2, b3);
		ed_r3.normalise();
		fieldVectors.push_back(ed_r3);
		face_centers.push_back(cen + b3.normalise()*0.1);
	}
	// constraint vectors ;
	for (int i = 0; i < B.rows(); i++)
	{
		Cdouble u = B(i, 0);
		vec cen = M.faces[fixedFaces[i]].centroid(M.positions);
		vec ed_r3 = getVecFromComplexEdgeAndTriangle(u, fixedFaces[i], M, b1, b2, b3);
		ed_r3.normalise();
		fieldVectors.push_back(ed_r3);
		face_centers.push_back(cen + b3.normalise()*0.1);
	}

	/////////////////////////////////// /////////////////////////////////// scalar inter.
	VectorXd U(fieldVectors.size() * 3, 1);
	for (int i = 0; i < Ui.rows(); i++)
	{
		U(i * 3, 0) = fieldVectors[i].y;
		U((i * 3) + 1, 0) = fieldVectors[i].x;
		U((i * 3) + 2, 0) = fieldVectors[i].z;
	}

	for (int i = Ui.rows(); i < fieldVectors.size(); i++)
	{
		U(i * 3, 0) = fieldVectors[i].y;
		U(i * 3 + 1, 0) = fieldVectors[i].z;
		U(i * 3 + 2, 0) = fieldVectors[i].z;
	}



	freeVerts.clear();
	fixedVerts.clear();
	for (int i = 0; i < V.rows(); i++)
		(i >= 1) ? freeVerts.push_back(i) : fixedVerts.push_back(i);

	Eigen::VectorXd Xb(1);
	Xb(0) = 0; //Xb(1) = 1; Xb(2) = 2;

	Eigen::VectorXd a;
	igl::doublearea(V, F, a);

	Eigen::VectorXd Ar(a.rows() * 3);

	for (int i = 0; i < a.rows(); i++)
	{
		Ar(i * 3, 0) = a(i);
		Ar((i * 3) + 1, 0) = a(i);
		Ar((i * 3) + 2, 0) = a(i);
	}


	//Eigen::SparseMatrix<double> A;
	Eigen::DiagonalMatrix<double, Eigen::Dynamic, Eigen::Dynamic> W = Ar.asDiagonal();

	Eigen::SparseMatrix<double> Q = G.transpose() * W * G;
	Eigen::MatrixXd denseQ = Eigen::MatrixXd(Q);
	Eigen::SparseMatrix<double> Q_stripped = subMatrix_rows(denseQ, freeVerts).sparseView();
	Eigen::SparseMatrix<double> Q_ii = subMatrix(Q_stripped, freeVerts);
	Eigen::SparseMatrix<double> Q_ib = subMatrix(Q_stripped, fixedVerts);

	Eigen::MatrixXd UGt = G.transpose() * W * U;
	Eigen::MatrixXd UGt_stripped = subMatrix_rows(UGt, freeVerts);
	Eigen::MatrixXd Q_ib_X_ib = Q_ib * Xb;
	Eigen::MatrixXd RHS = (UGt_stripped - Q_ib_X_ib);

	printf(" UtG_stripped : %i ,%i  \n", UGt_stripped.rows(), UGt_stripped.cols());
	printf(" Q_ib_X_ib : %i ,%i  \n", Q_ib_X_ib.rows(), Q_ib_X_ib.cols());
	printf(" RHS : %i ,%i  \n", (RHS).rows(), (RHS).cols());
	printf(" Q_ii : %i ,%i  \n", (Q_ii).rows(), (Q_ii).cols());


	Eigen::SparseLU< Eigen::SparseMatrix<double> > scalar_solver; // sparse cholesky solver
	scalar_solver.compute(Q_ii); // compute cholesky factors
	fv = scalar_solver.solve(RHS); // solve AX = B ;

	minV = fv.minCoeff();
	maxV = fv.maxCoeff();


	//dense_Qii.ldlt();

}



void setup()
{


	// Load a mesh in OFF format
	std::size_t foundOFF, foundOBJ;
	bool off = true;
	bool obj = true;
	string str = inFile;

	foundOFF = str.find('.off');
	if (foundOFF == std::string::npos) off = false;
	foundOBJ = str.find('.obj');
	if (foundOBJ == std::string::npos) obj = false;

	cout << foundOFF << " ----ff " << foundOBJ << endl;
	cout << off << " ----ff " << obj << endl;

	if (!off && !obj)exit(0);

	if (obj) igl::readOBJ(inFile, V, F);
	if (off) igl::readOFF(inFile, V, F);

	scaleToFitUnitBox(V);
	MatrixXd scale(3, 3);
	scale.setIdentity();
	scale *= 15;
	V *= scale;

	B1.resize(F.rows(), 3);
	B2.resize(F.rows(), 3);
	B3.resize(F.rows(), 3);
	igl::local_basis(V, F, B1, B2, B3);

	igl::grad(V, F, G);

	//cout << G << endl;
	matricesToMesh(M, V, F);

	A.resize(F.rows(), F.rows());
	A.setIdentity();

	S = *new SliderGroup();
	S.addSlider();
	S.sliders[0].attachToVariable(&numConstraints, 3., 50.0);

}



void update(int value)
{


}

void draw()
{


	backGround(0.75);

	S.draw();



	glLineWidth(1);
	glColor3f(0, 0, 0);
	for (int i = 0; i < Ui.rows(); i++) drawLine(face_centers[i], face_centers[i] + fieldVectors[i]);

	glLineWidth(5);
	glColor3f(1, 0, 0);
	for (int i = Ui.rows(); i < fieldVectors.size(); i++) drawLine(face_centers[i], face_centers[i] + fieldVectors[i] * 2);

	glPointSize(4);
	for (int i = 0; i < fieldVectors.size(); i++) drawPoint(face_centers[i]);

	glPushAttrib(GL_CURRENT_BIT);
	wireFrameOn();
	M.draw();
	wireFrameOff();

	VectorXd min = fv.colwise().minCoeff();
	VectorXd max = fv.colwise().maxCoeff();
	glPointSize(15);
	if (minV < maxV)
	{
		cout << minV << " " << freeVerts.size() << endl;
		for (int i = 0; i < freeVerts.size(); i++)
		{
			vec4 clr = getColour(fv(i, 0), minV, maxV);
			glColor3f(clr.r, clr.g, clr.b);
			drawPoint(M.positions[ freeVerts[i] ]);
		}
	}

	setup2d();
	char sc[300];
	sprintf(sc, " bc on/ off %i, #constraint ff vectors %i, \n", boundaryConstraint, int(numConstraints));
	drawString(sc, winW - 450, winH - 150);
	restore3d();

	//for (int i = 0; i < M.n_f; i++)
	//{
	//	vec b1, b2, b3;
	//	vec commonEdge;

	//	getComplexEdgeBasis(i, M, commonEdge, b1, b2, b3);
	//	vec cen = M.faces[i].centroid(M.positions);
	//	drawLine(cen, cen + b1);
	//	//drawLine(cen, cen + b2);
	//	drawLine(cen, cen + b3);
	//}

}




void keyPress(unsigned char k, int xm, int ym)
{

	if (k == '=')numConstraints += 1.0;
	if (k == '-')numConstraints -= 1.0;

	numConstraints = ofClamp(numConstraints, 1, 150);

	if (k == 's')setup();
	if (k == 'b')boundaryConstraint = !boundaryConstraint;
	if (k == '=' || k == '-' || k == 'b')interpolateField(numConstraints);

	printf(" bc on/ off %i, #constraint vectors %i, \n", boundaryConstraint, numConstraints);
}



void mousePress(int b, int s, int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
}

void mouseMotion(int x, int y)
{
	S.performSelection(x, y, HUDSelectOn);
	//interpolateField(numConstraints);
}



