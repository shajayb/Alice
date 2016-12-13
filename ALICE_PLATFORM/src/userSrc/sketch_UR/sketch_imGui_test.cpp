#define _MAIN_

#ifdef _MAIN_
#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;

#include "imgui.h"
#include "ImGuizmo.h"
////////////////////////////////////////////////////////////////////////// GLOBAL VARIABLES ----------------------------------------------------
////// --- MODEL OBJECTS ----------------------------------------------------

float objectMatrix[16];
Matrix4 T;
Mesh M;

////////////////////////////////////////////////////////////////////////// MAIN PROGRAM : MVC DESIGN PATTERN  ----------------------------------------------------
ImGuiIO io;//
static GLuint       g_FontTexture = 0;
////// ---------------------------------------------------- MODEL  ----------------------------------------------------

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

void imgui_draw()
{
	
	{
		// 1) get low-level inputs (e.g. on Win32, GetKeyboardState(), or poll your events, etc.)
		// TODO: fill all fields of IO structure and call NewFrame
		ImGuiIO& io = ImGui::GetIO();
		io.DeltaTime = 1.0f / 100.0f;
		//io.MousePos = mouse_pos;
		/*io.MouseDown[0] = mouse_button_0;
		io.MouseDown[1] = mouse_button_1;
		io.KeysDown[i] = ...*/

			// 2) call NewFrame(), after this point you can use ImGui::* functions anytime
			ImGui::NewFrame();

		// 3) most of your application code here
	//	MyGameUpdate(); // may use any ImGui functions, e.g. ImGui::Begin("My window"); ImGui::Text("Hello, world!"); ImGui::End();
	//	MyGameRender(); // may use any ImGui functions

						// 4) render & swap video buffers
		ImGui::Render();
		//SwapBuffers();
	}
}

void setup()	
{
	T.identity();
	for (int i = 0; i < 16; i += 1)objectMatrix[i] = T[i];



	MeshFactory fac;
	M = fac.createPlatonic(1, 6);

	Matrix4 Tr;
	Tr.setColumn(0, vec(1, 1, 0).normalise());
	Tr.setColumn(1, vec(1, -1, 0).normalise());
	Tr.setColumn(2, vec(0, 0, 1));
	Tr.setColumn(3, vec(0, 0, 0.5));
	Tr.invert();
	for (int i = 0; i < 8; i++) M.positions[i] = Tr * M.positions[i];


	
	{
		// Set up ImGui
		ImGuiIO& guiIO = ImGui::GetIO();
		guiIO.DisplaySize.x = winW;// 1680.0f;
		guiIO.DisplaySize.y = winH;// MA.resolution; // 1050.0f;
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

		{
			ImGuiIO& io = ImGui::GetIO();
			unsigned char* pixels;
			int width, height;
			io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   // Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) because it is more likely to be compatible with user's existing shaders. If your ImTextureId represent a higher-level concept than just a GL texture id, consider calling GetTexDataAsAlpha8() instead to save on GPU memory.

																	  // Upload texture to graphics system
			GLint last_texture;
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
			glGenTextures(1, &g_FontTexture);
			glBindTexture(GL_TEXTURE_2D, g_FontTexture);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

			// Store our identifier
			io.Fonts->TexID = (void *)(intptr_t)g_FontTexture;

			// Restore state
			glBindTexture(GL_TEXTURE_2D, last_texture);
		}

	}

	
}

void update(int value)
{
	
	
}

////// ---------------------------------------------------- VIEW  ----------------------------------------------------

void draw()
{

	backGround(0.75);
	drawGrid(20);

	ImGui::NewFrame();

	//
	wireFrameOn();

		M.draw();


		//imgui_draw();

	wireFrameOff();
	
	
	setup2d();

	 {
		static bool show_another_window = true;
		ImGui::Begin("Another Window", &show_another_window, ImVec2(200, 100));
		ImGui::Text("Hello");
		ImGui::End();

		//static bool testOpen = true;
		//ImGui::ShowTestWindow(&testOpen);


		ImGui::Render();
	}

	 restore3d();


}

////// ---------------------------------------------------- CONTROLLER  ----------------------------------------------------

void keyPress(unsigned char k, int xm, int ym)
{
	//XVTFE reserved 
	if (k == 'A')updateCam = !updateCam;

}

void mousePress(int b, int state, int x, int y)
{

	if (GLUT_LEFT_BUTTON == b && GLUT_DOWN == state)
		/*if (Gz->OnMouseDown(x, y))updateCam = false;*/
			
	if (GLUT_LEFT_BUTTON == b && GLUT_UP == state)
	{
		/*	Gz->OnMouseUp(x, y);
			updateCam = true;*/
	}
	
}

void mouseMotion(int x, int y)
{
	//Gz->SetEditMatrix(objectMatrix);
	//Gz->OnMouseMove(x, y);
}




#endif // _MAIN_
