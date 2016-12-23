

// ImGui - standalone example application for Glut + OpenGL, using programmable pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.
#include "ALICE_DLL.h"
using namespace Alice;

#include "imgui_freeglut.h"
//#include "glew.h"
//#include "freeglut.h"

#include "imgui.h"
//#include "imgui_impl_glut.h"
//IMGUI_API bool        ImGui_ImplGLUT_Init();
//IMGUI_API void        ImGui_ImplGLUT_NewFrame(int w, int h);
//IMGUI_API void        ImGui_ImplGLUT_Shutdown();

#include <iostream>
using namespace std;

unsigned int screenWidth = 1280;
unsigned int screenHeight = 720;
bool show_test_window = true;
bool show_another_window = false;

void drawGUI()
{
	ImGui_ImplGLUT_NewFrame(screenWidth, screenHeight);

	// 1. Show a simple window
	// Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug"
	{
		static float f = 0.0f;
		ImGui::Text("Hello, world!");
		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
		if (ImGui::Button("Test Window")) show_test_window ^= 1;
		if (ImGui::Button("Another Window")) show_another_window ^= 1;
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	}

	// 2. Show another simple window, this time using an explicit Begin/End pair
	if (show_another_window)
	{
		ImGui::SetNextWindowSize(ImVec2(200, 100), ImGuiSetCond_FirstUseEver);
		ImGui::Begin("Another Window", &show_another_window);
		ImGui::Text("Hello");
		ImGui::End();
	}

	// 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	/*if (show_test_window)
	{
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}*/

	ImGui::Render();
}

void drawScene()
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//// add code here to draw scene objects

	//// draw gui
	

	//glutSwapBuffers();

	long start = GetTickCount();

	float currentColor[4];
	glGetFloatv(GL_CLEAR, currentColor);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // background(255) ;
	//if (saveF)
	//{
	//	buffer.Init(screenW, screenH); // specify dimensions of the image file, max : 4000 x 4000 pixels ;	
	//	buffer.beginRecord(0.95); // record to offScreen texture , additionally specify a background clr, default is black

	//}
	/*else
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0 );*/


	/*if (updateCam)*/
	updateCamera();
	glColor3f(1, 1, 1);
	drawGrid( 20 );	

	//draw();


	//if (saveF)
	//{
	//	buffer.endRecord(true, numFrames); // stop recording, additionally specify save texture to disk, and a max number of frames to save ;
	//	buffer.drawTexture(proj_matrix, mv_matrix); // NOT WORKING draw recorded off-screen texture as image unto current screen
	//}

	drawGUI();
	glutSwapBuffers();

	
}

bool keyboardEvent(unsigned char nChar, int nX, int nY)
{
	ImGuiIO& io = ImGui::GetIO();

	io.AddInputCharacter(nChar);



	return true;
}

bool mouseEvent(int button, int state, int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);

	if (state == GLUT_DOWN && (button == GLUT_LEFT_BUTTON))
		io.MouseDown[0] = true;
	else
		io.MouseDown[0] = false;

	if (state == GLUT_DOWN && (button == GLUT_RIGHT_BUTTON))
		io.MouseDown[1] = true;
	else
		io.MouseDown[1] = false;

	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if (state == GLUT_DOWN && button == 3) // It's a wheel event
		io.MouseWheel = 1.0;
	else
		io.MouseWheel = 0.0;

	if (state == GLUT_DOWN && button == 4) // It's a wheel event
		io.MouseWheel = -1.0;
	else
		io.MouseWheel = 0.0;


	return true;
}

//void reshape(int w, int h)
//{
//	screenWidth = w;
//	screenHeight = h;
//
//	glViewport(0, 0, screenWidth, screenHeight);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	if (screenWidth > screenHeight)
//	{
//		float fWidth(screenWidth / screenHeight);
//		float fOffset((fWidth - 1.0f)*0.5f);
//		gluOrtho2D(0 - fOffset, fWidth - fOffset, 1.0f, 0.0f);
//	}
//	else
//	{
//		float fHeight(screenWidth / screenHeight);
//		float fOffset((fHeight - 1.0f)*0.5f);
//		gluOrtho2D(0, 1.0f, fHeight - fOffset, 0 - fOffset);
//	}
//	glMatrixMode(GL_MODELVIEW);
//}

int counter = 0;
void keyboardCallback(unsigned char k, int x, int y)
{
	if (k == 'X')exit(0);
	if (k == 'V')resetCamera();
	if (k == 'T')topCamera();
	if (k == 'F')
	{
		numFrames = 1200;
		int nf = 25;
		screenW = winW * 2;
		screenH = winH * 2;
		setPrintScreenAttribs(screenW, screenH, nf, true);

		saveF = !saveF;
		setSaveFrame(saveF);


		if (saveF) cout << " printing screen " << endl;
		else cout << " NOT printing screen " << endl;
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

			//draw();

			state = gl2psEndPage();
		}

		fclose(fp);
		printf("Done!\n");

		counter++;

	}


	//keyPress(k, xm, ym);
	
	if (keyboardEvent(k, x, y))
	{
		glutPostRedisplay();
	}
}

void mouseCallback(int button, int state, int x, int y)
{
	//mousePress(b, s, x, y);
	/*if (updateCam)*/Mouse(button, state, x, y);

	if (mouseEvent(button, state, x, y))
	{
		glutPostRedisplay();
	}
}

void mouseDragCallback(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);

	//mouseMotion(x, y);
	/*if (!HUDSelectOn && updateCam)*/Motion(x, y);

	glutPostRedisplay();
}

void mouseMoveCallback(int x, int y)
{
	ImGuiIO& io = ImGui::GetIO();
	io.MousePos = ImVec2((float)x, (float)y);

	glutPostRedisplay();
}

// initialize ogl and imgui
void init()
{
	glEnable(GL_MULTISAMPLE);
	glClearColor(0.447f, 0.565f, 0.604f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	ImGui_ImplGLUT_Init();
}

//int main(int argc, char **argv)
//{
//	glutInit(&argc, argv);
//	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
//	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);
//
//	glutInitWindowSize(1280, 720);
//	glutInitWindowPosition(200, 200);
//	glutCreateWindow("imgui FreeGlut Example");
//
//	// callback
//	glutDisplayFunc(drawScene);
//	glutReshapeFunc(reshape);
//	glutKeyboardFunc(keyboardCallback);
//	glutMouseFunc(mouseCallback);
//	glutMotionFunc(mouseDragCallback);
//	glutPassiveMotionFunc(mouseMoveCallback);
//
//	init();
//	glutMainLoop();
//
//	return 0;
//}



