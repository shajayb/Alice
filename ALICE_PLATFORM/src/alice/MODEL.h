
#ifndef _INTERACTION_
#define _INTERACTION_

#include "ALICE_DLL.h"
#include "utilities.h"
#include "object.h"
//#include "Controller.h"

class OBJECT
{
public:

	vec *positions;
	int nv;
	Matrix4 transformationMatrix;


	virtual void display()
	{
		// activate and specify pointer to vertex array
		glEnableClientState(GL_VERTEX_ARRAY);
		//glVertexPointer(3, GL_FLOAT, sizeof(vec), positions);
		glVertexPointer(3, GL_DOUBLE, 0, positions);

		glDrawArrays(GL_POINTS, 0, nv);

		// deactivate vertex arrays after drawing
		glDisableClientState(GL_VERTEX_ARRAY);

	};


	virtual void draw()
	{
		display();
	}
};

class COMMAND
{
public:
	OBJECT *obj = NULL;
	virtual ~COMMAND() {};
	virtual void doIt() 
	{
		cout << " default doIt" << endl;
	};
	virtual void undoIt() 
	{
		cout << " default undoIt" << endl;
	};
};



class MODEL
{

public:


	vector<OBJECT *> objectsInScene; 
	/*It is necessary to store object pointers to force compiler to
	force late binding of over-ridden(virtual) functions of derived classes.*/
	vector<COMMAND *> commandStack;


	void addObject(OBJECT &obj)
	{
		objectsInScene.push_back(&obj);
	}

	void addCommandToStack(COMMAND &cmd)
	{
		
		commandStack.push_back( &cmd );
		//cmd.doIt();
		commandStack.back()->doIt();

		//cout << commandStack.back() << endl;
	}

	void popCommandFromStack()
	{
		if (!commandStack.size() > 0)return;
		
		cout << "popping" << endl;

		commandStack.back()->undoIt();
		commandStack.pop_back();
	}


	////////////////////////////////////////////////////////////////////////// UTILITIES

	void correctScreenCoordinates( vec &mn, vec &mx , float w,float h )
	{

		mx.y *= -1;
		mn.y *= -1;

		mn += vec(w * 0.5, h * 0.5, 0);
		mx += vec(w * 0.5, h * 0.5, 0);

		if (mx.x < mn.x)swap(mx.x, mn.x);
		if (mx.y < mn.y)swap(mx.y, mn.y);
		if (mx.z < mn.z)swap(mx.z, mn.z);
	}
	void getCurrentMVPMatrices( Matrix4 &MV, Matrix4 &P, int (&viewport)[4] )
	{
	
		// get matrices and viewport:
		glGetFloatv(GL_MODELVIEW_MATRIX, MV.m);
		glGetFloatv(GL_PROJECTION_MATRIX, P.m);
		glGetIntegerv(GL_VIEWPORT, viewport);
	}

	////////////////////////////////////////////////////////////////////////// COMPUTE

	/*void performWindowSelection(CONTROLLER &ctrl)
	{
		vec mn, mx;
		mx = vec(ctrl.msx, ctrl.msy, 0);
		mn = vec(ctrl.cur_msx, ctrl.cur_msy, 0);
		performWindowSelection(mn, mx);

	}
*/
	void performWindowSelection( vec mn,vec mx )
	{
		Matrix4 MV, P;
		int viewport[4];
		getCurrentMVPMatrices( MV,P,viewport);
		
		MV.transpose();	P.transpose();

		//////////////////////////////////////////////////////////////////////////

		correctScreenCoordinates(mn, mx, viewport[2],viewport[3]);

		//////////////////////////////////////////////////////////////////////////
		

		setup2d();
		
			glPointSize(6);

				Matrix4 transform;
				for (auto obj : objectsInScene)
				{
					transform = obj->transformationMatrix.transpose();
			
					for (int i = 0; i < obj->nv; i++)
					{
						vec pt = worldToScreen( transform * obj->positions[i],MV,P,viewport);
						(isInRectangle(pt, mn, mx)) ? glColor3f(1, 0, 0) : glColor3f(1, 1, 1);
						drawPoint(pt);
					}
				}
			glPointSize(1);

			wireFrameOn();
				drawRectangle(mn, mx);
			wireFrameOff();
		
		restore3d();
	}

	////////////////////////////////////////////////////////////////////////// DISPLAY

	void draw()
	{

		glPointSize(6);

		for (auto obj : objectsInScene)
		{
			glPushMatrix();
				glMultMatrixf(obj->transformationMatrix.m);
			
					obj->draw();

			glPopMatrix();

		}

		glPointSize(1);
	}

};





#endif // _INTERFACE_
