#ifndef _APP
#define _APP 

#include "ALICE_DLL.h"
using namespace Alice;
#include "AL_gl2psUtils.h"

//------------------------------------------------------------------------------- FORWARD DECLARATIONS for functions

void keyPress(unsigned char k, int xm, int ym);
void draw();
void update(int value);
void setup();
void mousePress(int b, int s, int x, int y);
void mouseMotion(int x, int y);
bool HUDSelectOn = false;
bool updateCam = true;

float  					gTotalTimeElapsed = 0;
int 					gTotalFrames = 0;
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

void displayFPS(float timeElapsed)
{
	gTotalFrames++;
	gTotalTimeElapsed += timeElapsed;
	float fps = gTotalFrames / gTotalTimeElapsed;
	char string[1024] = { 0 };
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
float FPS = 0;
void updateCallBack(int value)
{
	long start = GetTickCount();

	update(value);

	frame++;
	winW = glutGet(GLUT_WINDOW_WIDTH);
	winH = glutGet(GLUT_WINDOW_HEIGHT);

	glutPostRedisplay();// refresh your screen i.e internally this will call draw() ;
	glutTimerFunc(10, updateCallBack, 0); // call update every 10 millisecs ;

	long end = GetTickCount();
	elapsedTime = end - startTime;
	long time = (end - start);
	if (time < 10)time = 10;
	FPS = 1000.0f / float(time);
}


void drawCallBack()
{

	long start = GetTickCount();

	float currentColor[4];
	glGetFloatv(GL_CLEAR, currentColor);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // background(255) ;
	if (saveF)
	{
		buffer.Init(screenW, screenH); // specify dimensions of the image file, max : 4000 x 4000 pixels ;	
		buffer.beginRecord(0.95); // record to offScreen texture , additionally specify a background clr, default is black

	}
	/*else
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0 );*/


	if (updateCam)updateCamera();
	glColor3f(1, 1, 1);
	//drawGrid( gridSz );	

	draw();


	if (saveF)
	{
		buffer.endRecord(true, numFrames); // stop recording, additionally specify save texture to disk, and a max number of frames to save ;
		buffer.drawTexture(proj_matrix, mv_matrix); // NOT WORKING draw recorded off-screen texture as image unto current screen
	}

	glutSwapBuffers();

	long end = GetTickCount();
	elapsedTime = end - startTime;
	long time = (end - start);
	if (time < 10)time = 10;
	FPS = 1000.0f / float(time);

}

int counter = 0;
void keyPressCallBack(unsigned char k, int xm, int ym)
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

			draw();

			state = gl2psEndPage();
		}

		fclose(fp);
		printf("Done!\n");

		counter++;

	}


	keyPress(k, xm, ym);
}

void mousePressCallBack(int b, int s, int x, int y)
{

	mousePress(b, s, x, y);
	if (updateCam)Mouse(b, s, x, y);
}

void motionCallBack(int x, int y)
{
	mouseMotion(x, y);
	if (!HUDSelectOn && updateCam)Motion(x, y);
}

string inFile = "";
//int main(int argc,char** argv)
//{
//	
//
//	if (argc > 1)inFile = argv[1];
//
//	glutInit(&argc,argv);
//	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
//	glutInitWindowSize(winW,winH);
//	glutInitWindowPosition(200,100);
//	glutCreateWindow("Maya Camera");
//
//	// register event methods ;
//	glutDisplayFunc(drawCallBack); // register a drawing code 
//	glutReshapeFunc(reshape); // register a reshape 
//	glutMouseFunc(mousePressCallBack);
//	glutMotionFunc(motionCallBack);
//	glutIdleFunc(idle);//(NEW) calls our idle function
//	glutTimerFunc(0,updateCallBack,0); // call update once in main
//	glutKeyboardFunc( keyPressCallBack ); // register keypress function;
//
//	{
//		glewInit();
//
//		if (glewIsSupported("GL_VERSION_2_0"))
//		{
//			printf("Ready for OpenGL 2.0\n");
//			//glewInited = true ;
//		}
//		else 
//			printf("OpenGL 2.0 not supported\n");
//	}
//
//	glEnable(GL_DEPTH_TEST);
//	glEnable(GL_POINT_SMOOTH);
//
//
//	setup();
//	startTime = GetTickCount();
//	init_timer();
//	glutMainLoop();
//
//	
//}
#include "imgui.h"
#include "ImGuizmo.h"

void imGui_drawFunc(ImDrawData* draw_data)
{
	// Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
	ImGuiIO& io = ImGui::GetIO();
	int fb_width = (int)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	int fb_height = (int)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (fb_width == 0 || fb_height == 0)
		return;
	draw_data->ScaleClipRects(io.DisplayFramebufferScale);

	// We are using the OpenGL fixed pipeline to make the example code simpler to read!
	// Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, vertex/texcoord/color pointers.
	GLint last_texture; glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	GLint last_viewport[4]; glGetIntegerv(GL_VIEWPORT, last_viewport);
	GLint last_scissor_box[4]; glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TRANSFORM_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_SCISSOR_TEST);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnable(GL_TEXTURE_2D);
	//glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context

	// Setup viewport, orthographic projection matrix
	glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0f, io.DisplaySize.x, io.DisplaySize.y, 0.0f, -1.0f, +1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	// Render command lists
#define OFFSETOF(TYPE, ELEMENT) ((size_t)&(((TYPE *)0)->ELEMENT))
	for (int n = 0; n < draw_data->CmdListsCount; n++)
	{
		const ImDrawList* cmd_list = draw_data->CmdLists[n];
		const ImDrawVert* vtx_buffer = cmd_list->VtxBuffer.Data;
		const ImDrawIdx* idx_buffer = cmd_list->IdxBuffer.Data;
		glVertexPointer(2, GL_FLOAT, sizeof(ImDrawVert), (const GLvoid*)((const char*)vtx_buffer + OFFSETOF(ImDrawVert, pos)));
		glTexCoordPointer(2, GL_FLOAT, sizeof(ImDrawVert), (const GLvoid*)((const char*)vtx_buffer + OFFSETOF(ImDrawVert, uv)));
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(ImDrawVert), (const GLvoid*)((const char*)vtx_buffer + OFFSETOF(ImDrawVert, col)));

		for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
		{
			const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
			if (pcmd->UserCallback)
			{
				pcmd->UserCallback(cmd_list, pcmd);
			}
			else
			{
				glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
				glScissor((int)pcmd->ClipRect.x, (int)(fb_height - pcmd->ClipRect.w), (int)(pcmd->ClipRect.z - pcmd->ClipRect.x), (int)(pcmd->ClipRect.w - pcmd->ClipRect.y));
				glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer);
			}
			idx_buffer += pcmd->ElemCount;
		}
	}
#undef OFFSETOF

	// Restore modified state
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	glBindTexture(GL_TEXTURE_2D, (GLuint)last_texture);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
	glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3]);
}


int main(int argc, char** argv)
{
	// ... app code
	glutInitDisplayMode(GLUT_RGB | GLUT_ALPHA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_MULTISAMPLE);
	glutInitWindowSize(winW, winH);
	int screenWidth = glutGet(GLUT_SCREEN_WIDTH);
	glutInitWindowPosition(5 * screenWidth / 12, 0);
	glutInit(&argc, argv);
	glutCreateWindow("A");

	// init opengl
	glEnable(GL_DEPTH_TEST);
	//setTransparencyEnabled(true); function's content in braces:
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}

	// set the background color
	// rgba=(0.25098, 0.501961, 0.12549, 1)
	//double r, g, b, a; bgColor.getComponents_double(r, g, b, a);
	glClearColor(1,1,1,1);

	// Set up glut callbacks
	glutDisplayFunc(drawCallBack); // register a drawing code 
	glutReshapeFunc(reshape); // register a reshape 
	glutMouseFunc(mousePressCallBack);
	glutMotionFunc(motionCallBack);
	glutIdleFunc(idle);//(NEW) calls our idle function
	glutTimerFunc(0, updateCallBack, 0); // call update once in main
	glutKeyboardFunc(keyPressCallBack); // register keypress function;

										// Set up ImGui
	ImGuiIO& guiIO = ImGui::GetIO();
	guiIO.DisplaySize.x = winW; // 1680.0f;
	guiIO.DisplaySize.y = winH;// .resolution; // 1050.0f;
	guiIO.DeltaTime = 1.0f / 60.0f;
	guiIO.IniFilename = "imgui.ini";
	guiIO.RenderDrawListsFn = imGui_drawFunc;
	guiIO.KeyMap[0] = 9;    // tab
	guiIO.KeyMap[1] = GLUT_KEY_LEFT;    // Left
	guiIO.KeyMap[2] = GLUT_KEY_RIGHT;   // Right
	guiIO.KeyMap[3] = GLUT_KEY_UP;      // Up
	guiIO.KeyMap[4] = GLUT_KEY_DOWN;    // Down
	guiIO.KeyMap[5] = GLUT_KEY_HOME;    // Home
	guiIO.KeyMap[6] = GLUT_KEY_END;     // End
	guiIO.KeyMap[7] = 127;  // Delete
	guiIO.KeyMap[8] = 8;    // Backspace
	guiIO.KeyMap[9] = 13;   // Enter
	guiIO.KeyMap[10] = 27;  // Escape
	guiIO.KeyMap[11] = 1;   // ctrl-A
	guiIO.KeyMap[12] = 3;   // ctrl-C
	guiIO.KeyMap[13] = 22;  // ctrl-V
	guiIO.KeyMap[14] = 24;  // ctrl-X
	guiIO.KeyMap[15] = 25;  // ctrl-Y
	guiIO.KeyMap[16] = 26;  // ctrl-Z

							//glGenTextures(1, &fontTex);
							//glBindTexture(GL_TEXTURE_2D, fontTex);
							//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
							//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

							//const void* png_data;
							//unsigned int png_size;
							//ImGui::GetDefaultFontData(NULL, NULL, &png_data, &png_size);
							//int tex_x, tex_y, tex_comp;
							//void* tex_data = stbi_load_from_memory((const unsigned char*)png_data, (int)png_size, &tex_x, &tex_y, &tex_comp, 0);
							//IM_ASSERT(tex_data != NULL);
							//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex_x, tex_y, 0, GL_RGBA, GL_UNSIGNED_BYTE, tex_data);
							//stbi_image_free(tex_data);


	glutMainLoop();     // GLUT has its own main loop, which calls display();
}



void draw() {
	glFlush();
	glClear(GL_COLOR_BUFFER_BIT);
	ImGui::NewFrame();


	// Draw application's things
	glViewport(0, 0, 800, 800);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-100, 100, -100, 100, -1, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//e.g. some polygons
	//glDepthFunc(GL_ALWAYS);
	//glPolygonMode(GL_FRONT, GL_FILL);
	//glColor4ub(polygon.color.r, polygon.color.g, polygon.color.b, polygon.color.a);

	//glBegin(GL_POLYGON);
	//for (vector<Point>::const_iterator it = polygon.pts.begin(); it != polygon.pts.end(); ++it) {
	//	glVertex2d(it->x, it->y);
	//}
	//glEnd();

	glViewport(0, 0, 800, 800);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 800, 0, 800, -1, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//e.g. text "Simulation time: 164.3s"

	/*glDepthFunc(GL_ALWAYS);
	glColor4ub(text.color.r, text.color.g, text.color.b, text.color.a);
	glRasterPos2f(text.pos.x, text.pos.y);

	int n = (int)text.text.length();
	for (int i = 0; i < n; ++i) {
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, text.text[i]);
	}*/

	// Draw ImGui (toggled using tab-key)
	/*if (guiDrawing)*/ {
		static bool show_another_window = true;
		ImGui::Begin("Another Window", &show_another_window, ImVec2(200, 100));
		ImGui::Text("Hello");
		ImGui::End();

		static bool testOpen = true;
		//ImGui::ShowTestWindow(&testOpen);


		ImGui::Render();
	}
	glutSwapBuffers();
}


void update(int value)
{


}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------



////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{


}

void mousePress(int b, int state, int x, int y)
{

}

void mouseMotion(int x, int y)
{

}




//
#endif