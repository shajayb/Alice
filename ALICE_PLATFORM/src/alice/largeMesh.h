#ifndef _LARGEMESH_
#define _LARGEMESH_

#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;


#define _LM_VERTICES 100000
#define _LM_FACECOUNTS_ 100000
#define _LM_FACECONNECTS_ 100000

#define MAX_CUBES 1000000

class largeMesh
{
public:

	GLfloat lm_positions[MAX_CUBES * 72];
	GLfloat lm_normals[MAX_CUBES * 72];
	GLfloat lm_colors[MAX_CUBES * 72];
	GLubyte lm_indices[_LM_FACECONNECTS_];


	//int polyCounts[_LM_FACECOUNTS_];
	int n_v, n_f, n_e;
	largeMesh()
	{
		n_v = n_f = n_e = 0;
	}

	void lightsOn(GLfloat light_pos[4])
	{
		// set position and enable light0
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

		//
		light_pos[0] *= -1;
		glEnable(GL_LIGHT1);
		glLightfv(GL_LIGHT1, GL_POSITION, light_pos);

		light_pos[1] *= -1;
		glEnable(GL_LIGHT2);
		glLightfv(GL_LIGHT2, GL_POSITION, light_pos);


		//glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 2.0);
		//glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 1.0);
		//glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.5);

		// enable material color
		glEnable(GL_COLOR_MATERIAL);
		glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
	}
	/*void addFace(vec *p, int nv)
	{
		for (int i = 0; i < nv ; i++)
		{
			positions[n_v++] = p[i].x;
			positions[n_v++] = p[i].y;
			positions[n_v++] = p[i].z;

			polyConnects[n_e++] = n_e;
		}

		polyCounts[n_f++] = nv;

	}*/

	/*  - 0.500000 - 0.500000 0.500000
		0.500000 - 0.500000 0.500000
		- 0.500000 0.500000 0.500000
		0.500000 0.500000 0.500000
		- 0.500000 0.500000 - 0.500000
		0.500000 0.500000 - 0.500000
		- 0.500000 - 0.500000 - 0.500000
		0.500000 - 0.500000 - 0.500000
	*/

	/*  f 0 1 3 2
		f 2 3 5 4
		f 4 5 7 6
		f 6 7 1 0
		f 1 7 5 3
		f 6 0 2 4
	*/
	int offset = 0;

	void addCube(Matrix4 &T = Matrix4())
	{
		

		for (int i = 0; i < 72; i+= 3)
		{
			vec P = T * vec(vertices[i], vertices[i + 1], vertices[i + 2]);
			int nv = n_v;
			lm_normals[nv + 0] = normals[i] * 1 ;
			lm_normals[nv + 1] = normals[i+1] * 1;
			lm_normals[nv + 2] = normals[i+2] * 1;

			// normals
			vec light(10, 10, 10);
			double ambO = (P - light).angle(vec(normals[i], normals[i + 1], normals[i + 2]));

			ambO = ofMap(ambO, -180, 180, 0, 1);
				
			lm_colors[nv + 0] = ambO;
			lm_colors[nv + 1] = ambO;
			lm_colors[nv + 2] = ambO;


			lm_positions[nv + 0] = P.x;
			lm_positions[nv + 1] = P.y;
			lm_positions[nv + 2] = P.z;

			n_v += 3;
			if (n_v > MAX_CUBES * 72)n_v = 0;
		}

		//for(int i = 0; i < 72; i++)
		//		lm_positions[n_v++] = GLfloat( vertices[i]);
		//for (int i = 0; i < 24; i++) lm_indices[n_e++] = offset + indices[i];
		n_f += 6;
		offset += 24;
	}

	void writeOBJ(string outFileName)
	{
		printf(" ----------- writing largemesh \n ");

		float scaleBack = 1.0;

		ofstream myfile;
		myfile.open(outFileName.c_str(), ios::out);


		if (myfile.fail())
		{
			myfile << " error in opening file  " << outFileName.c_str() << endl;
			return;
		}


		// vertices
		for (int i = 0; i < n_v; i+=3)
		{

			char s[200];
			sprintf(s, "v %1.4f %1.4f %1.4f ", lm_positions[i], lm_positions[i+1] , lm_positions[i+2] );

			myfile << s << endl;

		}

		// faces
		int numCubes = n_f / 6;
		for (int i = 0; i < numCubes  ; i++)
		{


			string str;
			str = "f ";
			for (int j = 0; j < 24; j++)
			{
				char s[200];
				itoa(indices[j] + i * 24 + 1, s, 10);
				str += s;
				str += "//";
				str += s;
				str += " ";
				if ((j+1) % 4 == 0 /*&& j > 0*/ )
				{
					
					myfile << str.c_str() << endl;
					str = "f ";
				}
			}

			

		}




		myfile.close();

		return;
	}

	void draw()
	{
		//
		//wireFrameOn();
		//glColor3f(1, 0, 0);
		//glPointSize(1);


		//wireFrameOff();

		//////////////////////////////////////////////////////////////////////////
		glPushAttrib(GL_CURRENT_BIT);

		//vec lightPos = vec(50, 0, 200);
		//GLfloat light_pos[] = { lightPos.x, lightPos.y, lightPos.z, 1.0 };

		//GLfloat qaAmbient[] = { 0.0, 0.0, 0.0, 1.0 }; //Black Color
		//GLfloat qaDiffuse[] = { 0.5, 0.5, 0.5, 1.0 }; //Green Color
		//GLfloat qaWhite[] = { 1.0, 1.0, 1.0, 1.0 }; //White Color
		//GLfloat qaRed[] = { 1.0, 0.0, 0.0, 1.0 }; //White Color

		//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, qaAmbient);
		//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, qaDiffuse);
		//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, qaWhite);
		//glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 1);

		//glPushAttrib(GL_CURRENT_BIT);
		//glEnable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		////resetProjection();
		//	glColor4f(1.0, 1.0, 1.0, 1.0);
		//	lightsOn(light_pos);
		////restore3d();


			//drawCube(vec(30, 0, 0), vec(60, 30, 1));

		{
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_NORMAL_ARRAY);
			glEnableClientState(GL_COLOR_ARRAY);

			glVertexPointer(3, GL_FLOAT, 0, lm_positions);
			glNormalPointer(GL_FLOAT, 0, lm_normals);
			glColorPointer(3, GL_FLOAT, 0, lm_colors);
			//glDrawElements(GL_QUADS, n_f * 6, GL_UNSIGNED_BYTE, lm_indices);// index hopping isnt workign currently - 140617
			glDrawArrays(GL_QUADS, 0, n_f * 6);


			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_NORMAL_ARRAY);
			glDisableClientState(GL_COLOR_ARRAY);
		}

		glPopMatrix();
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);

				



	}
};




#endif // !_LARGEMESH_
