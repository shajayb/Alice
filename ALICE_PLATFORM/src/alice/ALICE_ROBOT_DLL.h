#ifndef __ALICE_ROBOT_DLL__
#define __ALICE_ROBOT_DLL__


#include "ALICE_DLL.h"
using namespace Alice;

#include "matrices.h"
#include <iostream>
//#include <Eigen/Dense>
//#include <Eigen/Core>
//#include <Eigen/Eigen>
//#include <Eigen/Sparse>
//#include <bench/BenchTimer.h>
//using namespace Eigen;

#ifdef DLL_EXPORT
#  define DLL_API __declspec(dllexport)
//#include "jpeglib.h"
#else
#  define DLL_API __declspec(dllimport)
#endif
//__cdecl
#define DLL_CALL  __stdcall 




namespace ROBOTICS 
{

	struct DLL_API Link
	{
	public:

		Matrix4 T;
		double alpha, a, d, theta;

		Link();
		Link(double _d, double _a, double _alpha, double _theta);
		void updateTransform();
		void draw(vec str);
		void print();
	};

	#define DOF 6 
	class DLL_API Robot
	{
	public:

		Link Bars[DOF];
		vec joints[DOF];
		//MatrixXd Jacobian;
		Matrix4 Bars_to_home_matrices[DOF];
		Matrix4 homeBasis, homeBasisInverse, transformMatrix;

		double rot[6];
		//double home_rot[6];
		//VectorXd inv_rot;
		Mesh link_meshes[6];// base, link0, link1, link2, link3, link4;
		Mesh base;
		vec TCP, TCP_x, TCP_y, TCP_z;
		double fac = 5.0;
		Matrix4 Tbase;
		vec pt;


		Robot();
		//Robot(MatrixXd DH);
		~Robot();
		//void constructRobot(MatrixXd DH);
		void constructRobot(double DH[6][4]);
		void addMeshes(string meshNames[6], bool triangulate = true); 
		void addMeshes(bool triangulate = true );
		vec ForwardKineMatics(double rot[DOF]);
		vec inverseKinematics_analytical(Matrix4 Tool, bool showAlt = false);
		void calcJoints();
		void invertTransformMeshesToLocal();		
		void draw(bool wire = false, bool drawMatrix = false, vec4 clr = vec4(1, 1, 1, 1));
		void drawJoints();
		void drawAxes(vec cen, vec x, vec y, vec z);

	};

	class DLL_API Robot_Symmetric
	{
	public:

		double scale;
		Link Bars[DOF];
		vec joints[DOF];
		//MatrixXd Jacobian;
		Matrix4 Bars_to_world_matrices[DOF];

		vec TCP_x, TCP_y, TCP_z, TCP;
		vec wrist;
		Matrix4 transformMatrix;
		//inv
		double rot[6];
		//VectorXd inv_rot;
		Mesh link_meshes[5];// base, link0, link1, link2, link3, link4;
		Mesh base;
		vec origPositions[20000];
		Robot_Symmetric();
		~Robot_Symmetric();
	
		void addMeshes();
		vec ForwardKineMatics(double rot[DOF]);
		void invertTransformMeshesToLocal();
		void constructRobot(double DH[6][4]);
		//vec ForwardKineMatics(VectorXd rot);
		//vec inverseKinematics(vec ePt, bool fast = false);
		/*vec inverseKinematics_analytical(vec ePt, bool showAlt = false);*/
		vec inverseKinematics_analytical(Matrix4 TOOL, bool showAlt);
		void calcJoints();
		void draw(bool shaded = false);

		void drawAxes(int i);
		void drawAxes(vec &x, vec &y, vec &z, vec &cen);
		void drawMatrices();
		void drawLinks();


	};
}






#endif 