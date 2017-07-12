
#ifndef _CONTROLLER_
#define _CONTROLLER_

#include "ALICE_DLL.h"
#include "utilities.h"
#include "MODEL.h"

class CONTROLLER
{
public:

	int msx, msy;
	int cur_msx, cur_msy;
	vec curPt;
	vec anchorPt;

	vector<vec> clkPts;
	string keySeq;
	bool dragging = false;
	bool rightMouseDown = false;
	bool lefttMouseDown = false;

	long timeAtClick;

	CONTROLLER()
	{
		clkPts.clear();
	}


	bool checkForDuplicates( vec &pt , vector<vec> &pts )
	{
		if ( ! clkPts.size() > 0)return false;
		
		vector<vec>::reverse_iterator rit = pts.rbegin();
		for (; rit != pts.rend(); ++rit)
			if (pt.distanceTo(*rit) < 1e-4)return true;
		
		return false;
	}
	
	////////////////////////////////////////////////////////////////////////// event - callback functions

	void mousePress(int b, int state, int x, int y)
	{
		if (glutGetModifiers() == GLUT_ACTIVE_ALT)
		{
			cur_msx = msx = x - winW * 0.5;
			cur_msy = msy = winH * 0.5 - y;

			vec pt = screenToWorld( vec(msx, msy, 0) );
			anchorPt = pt;
			if ( !checkForDuplicates(pt, clkPts) ) clkPts.push_back(pt);
		}

	}
	void mouseMotion(int x, int y)
	{
		dragging = ( glutGetModifiers() == GLUT_ACTIVE_ALT ) ? true : false ;
		if (!dragging) return;

		cur_msx = x - winW * 0.5;
		cur_msy = winH * 0.5 - y;
		curPt = screenToWorld(vec(cur_msx, cur_msy, 0));
		
	}
	void keyPress(unsigned char k, int xm, int ym, MODEL &_model)
	{
		//printf("ASCII value of %c = %d \n", k, k);
		if (glutGetModifiers() == GLUT_ACTIVE_ALT)keySeq += "ALT_";
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL)keySeq += "CTRL_";

		keySeq += k;
		keySeq += ",";
		
		if ( k == 13 /*enter*/)keySeq.clear();

		//////////////////////////////////////////////////////////////////////////
		if (glutGetModifiers() == GLUT_ACTIVE_CTRL && k == 26 /* z when CTRL is pressed*/ )
		{
			printf("ASCII value of %c = %d \n", k, k);
			//executeOnModel_commandUndo(_model);
			_model.popCommandFromStack();
		}
	}

	void update( MODEL &_model )
	{
		executeOnModel_windowSelection(_model);
	}

	void draw( MODEL &model )
	{
		if (!dragging)return;

		lineStyle(4);

		GLfloat c[4];
		glGetFloatv(GL_CURRENT_COLOR, c);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(0, 0, 0, 0.3);

		
			setup2d();

				drawCircle(vec(msx + winW*0.5, winH*0.5 - msy, 0), 10, 64);
				drawString(keySeq, 50, 50);
			
				char s[20];
				sprintf(s, "%i", dragging);
				drawString(s, 50, 65);

			
			restore3d();

			//glBegin(GL_LINE_STRIP);
			//for (auto &p : clkPts) glVertex3f(p.x, p.y, p.z);
			//glEnd();

			wireFrameOn();
				drawRectangle(anchorPt, curPt);
			wireFrameOff();
		
		lineStyle(0);
		glDisable(GL_BLEND);
		glColor4f(c[0],c[1],c[2],c[3]);
		
	}

	////////////////////////////////////////////////////////////////////////// actions on MODEL

	void executeOnModel_windowSelection(MODEL &_model)
	{
		vec mx = vec(msx, msy, 0);
		vec mn = vec(cur_msx, cur_msy, 0);
		_model.performWindowSelection(mn, mx);
	}
	void executeOnModel_commandUndo(MODEL &_model)
	{
		_model.popCommandFromStack();
	}

};


#endif // _CONTROLLER_
