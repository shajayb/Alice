#ifndef _APP
#define _APP 

#include "ALICE_DLL.h"
using namespace Alice;

#include "AL_gl2psUtils.h" // vector screen capture 

#include "ALICE_ROBOT_DLL.h" // robot kinematics
using namespace ROBOTICS;

#include "CONTROLLER.h" // keyboard and mouse tracker
#include "MODEL.h"// picking and selection

#include "utilities.h"

//------------------------------------------------------------------------------- FORWARD DECLARATIONS for functions

void setup();
void update(int value);
void draw() ;
void keyPress(unsigned char k, int xm, int ym);
void mousePress(int b,int s,int x,int y) ;
void mouseMotion( int x, int y ) ;

//------------------------------------------------------------------------------- APPLICATION VARIABLES


bool HUDSelectOn = false;
bool updateCam = true;
int counter = 0;
string inFile = "";
CONTROLLER CONTROLLERS;
MODEL SCENE;
ButtonGroup B;
SliderGroup S;
//------------------------------------------------------------------------------- CALLBACKS

void updateCallBack( int value )
{
	long start = GetTickCount();

		update( value ) ;

		frame ++ ;
		winW = glutGet(GLUT_WINDOW_WIDTH);
		winH = glutGet(GLUT_WINDOW_HEIGHT);
		
		glutPostRedisplay() ;// refresh your screen i.e internally this will call draw() ;
		glutTimerFunc(10,updateCallBack,0); // call update every 10 millisecs ;

	long end = GetTickCount();
	elapsedTime = end - startTime ;
	long time = (end-start) ;
	if( time < 10 )time = 10 ;

}

void drawCallBack()
{
	
	long start = GetTickCount();
	
		float currentColor[4];
		glGetFloatv(GL_CLEAR,currentColor) ;

		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT); // background(255) ;
		if(saveF)
		{
			buffer.Init( screenW , screenH ); // specify dimensions of the image file, max : 4000 x 4000 pixels ;	
			buffer.beginRecord(0.95); // record to offScreen texture , additionally specify a background clr, default is black
	
		}
		/*else
			glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0 );*/
		//////////////////////////////////////////////////////////////////////////

			if(updateCam)updateCamera() ;
			glColor3f(1,1,1);
			//drawGrid( gridSz );	

			draw() ;



			SCENE.performWindowSelection(CONTROLLERS);
			SCENE.draw();
			CONTROLLERS.draw();
			S.draw();
			B.draw();
		//////////////////////////////////////////////////////////////////////////

		if (saveF)
		{
			buffer.endRecord(true, numFrames); // stop recording, additionally specify save texture to disk, and a max number of frames to save ;
			buffer.drawTexture(proj_matrix, mv_matrix); // NOT WORKING draw recorded off-screen texture as image unto current screen
		}

		glutSwapBuffers();
	
	long end = GetTickCount();
	elapsedTime = end - startTime ;
	long time = (end-start) ;
	if( time < 10 )time = 10 ;

	
}

void keyPressCallBack(unsigned char k, int xm, int ym)
{

	CONTROLLERS.keyPress(k, xm, ym);

	if (k == 'R')setup();
	if( k == 'X' )exit(0) ;
	if( k == 'V' )resetCamera() ;
	if( k == 'T' )topCamera();
	if( k == 'F' )
	{
		numFrames = 1200 ; 
		int nf = 25 ;
		screenW = winW * 1 ;
		screenH = winH * 1 ;
		setPrintScreenAttribs(screenW,screenH,nf,true);
		
		saveF = !saveF ;
		setSaveFrame(saveF);

		if( saveF ) cout << " printing screen " << endl ;
		else cout << " NOT printing screen " << endl ;
	}

	if (k == 'E')
	{
		FILE *fp;
		int state = GL2PS_OVERFLOW, buffsize = 0;


		string file = "";
		file += "data/out";
		file += "_";
		char s[20];
		itoa(counter, s, 10);
		file += s;
		file += ".eps";

		fp = fopen(file.c_str(), "w");
		cout << file.c_str() << endl;
		printf("Writing 'out.eps'... ");

		while (state == GL2PS_OVERFLOW)
		{
			buffsize += winW * winH;
			gl2psBeginPage("test", "gl2psTestSimple", NULL, GL2PS_EPS, GL2PS_NO_SORT,
				GL2PS_USE_CURRENT_VIEWPORT,
				GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, file.c_str());

			draw();



			SCENE.performWindowSelection(CONTROLLERS);
			SCENE.draw();
			CONTROLLERS.draw();

			state = gl2psEndPage();
		}

		fclose(fp);
		printf("Done!\n");

		counter++;

	}
	
	
	keyPress(k,xm,ym);
}

void mousePressCallBack(int b,int s,int x,int y)
{

	 CONTROLLERS.mousePress(b, s, x, y);


	 S.performSelection(x, y, HUDSelectOn);
	 B.performSelection(x, y);

	 mousePress( b, s, x, y) ;

	 updateCam = (glutGetModifiers() == GLUT_ACTIVE_ALT) ? false : true;
	 
	 if(updateCam )Mouse( b, s, x, y) ;
}

void motionCallBack( int x, int y )
{

	CONTROLLERS.mouseMotion(x, y);

	S.performSelection(x, y, HUDSelectOn);
	B.performSelection(x, y);

	mouseMotion(x,y);
	if(!HUDSelectOn && updateCam)Motion( x, y) ;
}

//------------------------------------------------------------------------------- ENTRY POINT 

int main(int argc,char** argv)
{
	

	if (argc > 1)inFile = argv[1];

	

	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
	glutInitWindowSize(winW,winH);
	glutInitWindowPosition(200,100);
	glutCreateWindow("Soft-Rigid-Block Equilibrium");

	// register event methods ;
	glutDisplayFunc(drawCallBack); // register a drawing code 
	glutReshapeFunc(reshape); // register a reshape 
	glutMouseFunc(mousePressCallBack);
	glutMotionFunc(motionCallBack);
	glutIdleFunc(idle);//(NEW) calls our idle function
	glutTimerFunc(0,updateCallBack,0); // call update once in main
	glutKeyboardFunc( keyPressCallBack ); // register keypress function;

	{
		glewInit();

		if (glewIsSupported("GL_VERSION_2_0"))
		{
			printf("Ready for OpenGL 2.0\n");
			//glewInited = true ;
		}
		else 
			printf("OpenGL 2.0 not supported\n");
	}

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SMOOTH);


	setup();
	glutMainLoop();

	
}

//#include <qapplication.h>
//
//int main(int argc, char** argv)
//{
//	// Read command lines arguments.
//	QApplication application(argc, argv);
//
//	// Instantiate the viewer.
//	Viewer viewer;
//
//	viewer.setWindowTitle("simpleViewer");
//
//	// Make the viewer window visible on screen.
//	viewer.show();
//
//	// Run main loop.
//	return application.exec();
//}
//
//
#endif