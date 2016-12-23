#ifndef _APP
#define _APP 

#include "ALICE_DLL.h"

using namespace Alice;
#include "AL_gl2psUtils.h"

//------------------------------------------------------------------------------- FORWARD DECLARATIONS for functions

void keyPress (unsigned char k, int xm, int ym);
void draw() ;
void update( int value );
void setup() ;
void mousePress(int b,int s,int x,int y) ;
void mouseMotion( int x, int y ) ;
bool HUDSelectOn = false;
bool updateCam = true;

float  					gTotalTimeElapsed 	= 0;
int 					gTotalFrames		= 0;
GLuint 					gTimer;

void init_timer()
{
	glGenQueries(1, &gTimer);
}

void start_timing()
{
	glBeginQuery(GL_TIME_ELAPSED, gTimer);
}

float stop_timing()
{
	glEndQuery(GL_TIME_ELAPSED);

	GLint available = GL_FALSE;
	while (available == GL_FALSE)
		glGetQueryObjectiv(gTimer, GL_QUERY_RESULT_AVAILABLE, &available);

	GLint result;
	glGetQueryObjectiv(gTimer, GL_QUERY_RESULT, &result);

	float timeElapsed = result / (1000.0f * 1000.0f * 1000.0f);
	return timeElapsed;
}

void displayFPS( float timeElapsed )
{
	gTotalFrames++;
	gTotalTimeElapsed += timeElapsed;
	float fps = gTotalFrames / gTotalTimeElapsed;
	char string[1024] = {0};
	sprintf(string, "FPS: %0.2f FPS", fps);
	glutSetWindowTitle(string);
}


void lineStyle(int lineType)
{
	glEnable(GL_LINE_STIPPLE);
	switch (lineType)
	{
	case 0: glLineStipple(2, 0xffff); break;
	case 1: glLineStipple(2, 0x00ff); break;
	case 2: glLineStipple(2, 0xffff); break;
	case 3: glLineStipple(2, 0x0c0f); break;
	case 4: glLineStipple(2, 0x0c0f); break;
	case 5: glLineStipple(2, 0xaaaa); break;
	case 6: glLineStipple(2, 0xaaaa); break;
	default:glLineStipple(3, 0xaaaa); break;
	}

}

//------------------------------------------------------------------------------- CALLBACKS
float FPS = 0 ;
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
	FPS = 1000.0f / float(time) ;
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

	backGround(0.75);

	if(updateCam)updateCamera() ;
	drawGrid(gridSz);

	draw() ;

	
	if(saveF)
	{
		buffer.endRecord( true , numFrames  ) ; // stop recording, additionally specify save texture to disk, and a max number of frames to save ;
		buffer.drawTexture( proj_matrix , mv_matrix  ) ; // NOT WORKING draw recorded off-screen texture as image unto current screen
	}




	glutSwapBuffers();
	
	long end = GetTickCount();
	elapsedTime = end - startTime ;
	long time = (end-start) ;
	if( time < 10 )time = 10 ;
	FPS = 1000.0f / float(time) ;
	
}

int counter = 0;
void keyPressCallBack(unsigned char k, int xm, int ym)
{
	
	if( k == 'X' )exit(0) ;
	if( k == 'V' )resetCamera() ;
	if( k == 'T' )topCamera();
	if( k == 'F' )
	{
		numFrames = 1200 ; 
		int nf = 25 ;
		screenW = winW * 2 ;
		screenH = winH * 2 ;
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
		printf("Writing 'out.eps'... ");

		while (state == GL2PS_OVERFLOW)
		{
			buffsize += winW * winH;
			gl2psBeginPage("test", "gl2psTestSimple", NULL, GL2PS_EPS, GL2PS_NO_SORT,
				GL2PS_USE_CURRENT_VIEWPORT,
				GL_RGBA, 0, NULL, 0, 0, 0, buffsize, fp, file.c_str());

			draw();

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

	 mousePress( b, s, x, y) ;
	 if(updateCam)Mouse( b, s, x, y) ;
}

void motionCallBack( int x, int y )
{
	mouseMotion(x,y);
	if(!HUDSelectOn && updateCam)Motion( x, y) ;
}

string inFile = "";
int main(int argc,char** argv)
{
	

	if (argc > 1)inFile = argv[1];

	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
	glutInitWindowSize(winW,winH);
	glutInitWindowPosition(200,100);
	glutCreateWindow("Maya Camera");

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
	startTime = GetTickCount();
	init_timer();
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