#ifndef _AL_Gl2PS_UTILS
#define _AL_Gl2PS_UTILS

#include "ALICE_DLL.h"
#include "gl2ps.h"

void AL_glLineWidth(float wid)
{
	gl2psLineWidth((wid));
	glLineWidth((wid));
}

void AL_glPointSize(float wid)
{
	gl2psPointSize((wid));
	glPointSize((wid));
}
void AL_glEnable_lineStipple()
{
	glEnable(GL_LINE_STIPPLE);
	gl2psEnable(GL2PS_LINE_STIPPLE);
	glLineStipple(0.25, 0x0c0f); // default
}

void AL_glDisable_lineStipple()
{

	glDisable(GL_LINE_STIPPLE);
	gl2psDisable(GL2PS_LINE_STIPPLE);
}

void AL_drawString( const char *string, float x, float y)
{
	float angle = 0.0;
	unsigned int i;
	const char *fonts[] =
	{ "Times-Roman", "Times-Bold", "Times-Italic", "Times-BoldItalic",
	"Helvetica", "Helvetica-Bold", "Helvetica-Oblique", "Helvetica-BoldOblique",
	"Courier", "Courier-Bold", "Courier-Oblique", "Courier-BoldOblique",
	"Symbol", "ZapfDingbats" };

	/* call gl2psText before the glut function since glutBitmapCharacter
	changes the raster position... */
	glRasterPos2d(x, y);
	gl2psTextOpt(string, fonts[3], 14, GL2PS_TEXT_BL, angle);

	for (i = 0; i < strlen(string); i++)
		glutBitmapCharacter(GLUT_BITMAP_9_BY_15, string[i]);

	//drawString(string, x, y);
}



#endif