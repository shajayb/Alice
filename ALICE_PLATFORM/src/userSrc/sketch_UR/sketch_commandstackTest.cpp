

#ifdef _MAIN_

#include "main.h"
#include "Matrix.h"
#include "MODEL.h"
#include "interpolate.h"

#include <array>
#include <memory>
#include<time.h>
#include<experimental/generator> 
#include<experimental/generator> 
using namespace std;
using namespace std::experimental;

#include "largeMesh.h"

/////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define  rx ofRandom(-1,1)


class scaleCommand : public COMMAND
{
public:

	bool undone = false;
	double scaleFactor = 1.1;
	
	scaleCommand()
	{
		obj = NULL;
		scaleFactor = 1.1;
	};
	~scaleCommand()
	{
		cout << "scaleCommand destroyed" << this <<  endl;
	}

	scaleCommand( OBJECT *_obj , double _scaleFactor = 1.1) 
	{
		obj = _obj;
		scaleFactor = _scaleFactor;
	};

	virtual void doIt()
	{
		if (!obj) return;
		obj->transformationMatrix.scale(scaleFactor);
		undone = false; 
	}

	virtual void undoIt()
	{
		if (!obj) return;

		cout << "scale cmd undo" << endl;

		if (!undone)
		{
			obj->transformationMatrix.scale(1.0 / scaleFactor);
			undone = true;
		}
		
	}
};


class moveCommand : public COMMAND
{
public:

	bool undone = false;
	vec moveBy;

	moveCommand()
	{
		obj = NULL;
		moveBy = vec(1,1,1);
	};
	~moveCommand()
	{
		cout << "scaleCommand destroyed" << this << endl;
	}

	moveCommand(OBJECT *_obj, vec _moveBy = vec(1,1,1))
	{
		obj = _obj;
		moveBy = _moveBy;
	};

	virtual void doIt()
	{
		if (!obj) return;
		obj->transformationMatrix.translate(moveBy);
		undone = false;
	}

	virtual void undoIt()
	{
		if (!obj) return;

		cout << "scale cmd undo" << endl;

		if (!undone)
		{
			obj->transformationMatrix.translate(moveBy*-1);
			undone = true;
		}

	}
};


class derivedOBJECT :public OBJECT
{
public:
	Mesh m;
	derivedOBJECT()
	{
		MeshFactory fac;
		m = fac.createPlatonic(20, 6);
		positions = m.positions;
		nv = m.n_v;

	}

	virtual void draw()
	{
		display();
		wireFrameOn();
		m.draw();
		wireFrameOff();
	}


};


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

derivedOBJECT MO;
scaleCommand cmd_sc;
vector<COMMAND> cmds;

void setup()
{

	MO = *new derivedOBJECT();
	MO.transformationMatrix.identity();
	SCENE.addObject(MO);

	cmd_sc = *new scaleCommand(&MO, 2.0);
}

void update(int value)
{
	
}

void draw()
{

	backGround(0.9);
	drawGrid(20);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{

}

void keyPress(unsigned char k, int xm, int ym)
{
	if (k == 's')
	{
		//scaleCommand s(&MO, 2.0);
		//cmds.push_back( * new scaleCommand(&MO, 2.0) );
		SCENE.addCommandToStack(*new scaleCommand(&MO, 2.0));
	}


	if (k == 'm')
	{
		//scaleCommand s(&MO, 2.0);
		//cmds.push_back( * new scaleCommand(&MO, 2.0) );
		SCENE.addCommandToStack(*new moveCommand(&MO));
	}
}





#endif // _MAIN_

