#ifndef _ALICE_DLL
#define _ALICE_DLL


#include <GL/glew.h>
#include <iostream>
#include <fstream>
using namespace std;
#include <freeglut.h>
#include <list>
#include <string.h>
#include "stdio.h"
#include <math.h>
#include <vector>
#include <string>
#include <map>
#include <set>
//#include "isoSurfaceConstants.h"
#include "gl2ps.h"
#include "AL_gl2psUtils.h"

#ifdef DLL_EXPORT
#  define DLL_API __declspec(dllexport)
//#include "jpeglib.h"
#else
#  define DLL_API __declspec(dllimport)
#endif
//__cdecl
#define DLL_CALL  __stdcall 



struct vec4
{
	double r, g, b, a;
	vec4(double _x, double _y, double _z, double _w)
	{
		r = _x; g = _y; b = _z; a = _w;
	}
	vec4(){};

};


#ifndef EPS
#define EPS 0.0001 
#endif
#ifndef PI
#define PI       3.14159265358979323846
#endif

#ifndef TWO_PI
#define TWO_PI   6.28318530717958647693
#endif

#ifndef M_TWO_PI
#define M_TWO_PI   6.28318530717958647693
#endif

#ifndef FOUR_PI
#define FOUR_PI 12.56637061435917295385
#endif

#ifndef HALF_PI
#define HALF_PI  1.57079632679489661923
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI/180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0/PI)
#endif

#ifndef MIN
#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef CLAMP
#define CLAMP(val,min,max) (MAX(MIN(val,max),min))
#endif

#ifndef ABS
#define ABS(x) (((x) < 0) ? -(x) : (x))
#endif


// buffer
#define MAX_W 3000
#define MAX_H 3000

// importer
#define MAX_NBORS 15 
#define MAX_DIVS 20 
#define MAX_POINTS	15000

// ao shader
#define DIM 256 

//particle lib

#define SPR_CONST 0.8
#define SPR_DAMP_CONST 0.8
#define SPR_RL 20

#define R_SPR_CONST 1.1
#define R_SPR_DAMP_CONST 1
#define R_SPR_RA (M_PI / 2 * RAD_TO_DEG) 

#define C_DIST 100.001 
#define C_CHARGE 1 
#define C_DAMP 0.1 
#define COLOUMB ( 1 * pow(100.0f,2.0f) )  

#define MAX_PARTICLES 5500 
#define MAX_SPRINGS 5500 
#define MAX_ROTATE_SPRINGS 1500 * 10 
#define MAX_CHARGES 1500 * 1500 

#define DIM_PS 500 
// tris have to be a multiple of 3, to make 1d/2d array access easier ;
#define DIM_PS_TRIS 501


#define maxDataSize 200 

static const float a2fVertexOffset[8][3] =
{
	{0.0, 0.0, 0.0},{1.0, 0.0, 0.0},{1.0, 1.0, 0.0},{0.0, 1.0, 0.0},
	{0.0, 0.0, 1.0},{1.0, 0.0, 1.0},{1.0, 1.0, 1.0},{0.0, 1.0, 1.0}
};

//a2iEdgeConnection lists the index of the endpoint vertices for each of the 12 edges of the cube
static const GLint a2iEdgeConnection[12][2] = 
{
	{0,1}, {1,2}, {2,3}, {3,0},
	{4,5}, {5,6}, {6,7}, {7,4},
	{0,4}, {1,5}, {2,6}, {3,7}
};

//a2fEdgeDirection lists the direction vector (vertex1-vertex0) for each edge in the cube
static const float a2fEdgeDirection[12][3] =
{
	{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{-1.0, 0.0, 0.0},{0.0, -1.0, 0.0},
	{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{-1.0, 0.0, 0.0},{0.0, -1.0, 0.0},
	{0.0, 0.0, 1.0},{0.0, 0.0, 1.0},{ 0.0, 0.0, 1.0},{0.0,  0.0, 1.0}
};

//a2iTetrahedronEdgeConnection lists the index of the endpoint vertices for each of the 6 edges of the tetrahedron
static const GLint a2iTetrahedronEdgeConnection[6][2] =
{
	{0,1},  {1,2},  {2,0},  {0,3},  {1,3},  {2,3}
};

//a2iTetrahedronEdgeConnection lists the index of verticies from a cube 
// that made up each of the six tetrahedrons within the cube
static const GLint a2iTetrahedronsInACube[6][4] =
{
	{0,5,1,6},
	{0,1,2,6},
	{0,2,3,6},
	{0,3,7,6},
	{0,7,4,6},
	{0,4,5,6},
};


GLint aiTetrahedronEdgeFlags[16]=
{
	0x00, 0x0d, 0x13, 0x1e, 0x26, 0x2b, 0x35, 0x38, 0x38, 0x35, 0x2b, 0x26, 0x1e, 0x13, 0x0d, 0x00, 
};


// For each of the possible vertex states listed in aiTetrahedronEdgeFlags there is a specific triangulation
// of the edge intersection points.  a2iTetrahedronTriangles lists all of them in the form of
// 0-2 edge triples with the list terminated by the invalid value -1.
//
// I generated this table by hand

GLint a2iTetrahedronTriangles[16][7] =
{
	{-1, -1, -1, -1, -1, -1, -1},
	{ 0,  3,  2, -1, -1, -1, -1},
	{ 0,  1,  4, -1, -1, -1, -1},
	{ 1,  4,  2,  2,  4,  3, -1},

	{ 1,  2,  5, -1, -1, -1, -1},
	{ 0,  3,  5,  0,  5,  1, -1},
	{ 0,  2,  5,  0,  5,  4, -1},
	{ 5,  4,  3, -1, -1, -1, -1},

	{ 3,  4,  5, -1, -1, -1, -1},
	{ 4,  5,  0,  5,  2,  0, -1},
	{ 1,  5,  0,  5,  3,  0, -1},
	{ 5,  2,  1, -1, -1, -1, -1},

	{ 3,  4,  2,  2,  4,  1, -1},
	{ 4,  1,  0, -1, -1, -1, -1},
	{ 2,  3,  0, -1, -1, -1, -1},
	{-1, -1, -1, -1, -1, -1, -1},
};

// For any edge, if one vertex is inside of the surface and the other is outside of the surface
//  then the edge intersects the surface
// For each of the 8 vertices of the cube can be two possible states : either inside or outside of the surface
// For any cube the are 2^8=256 possible sets of vertex states
// This table lists the edges intersected by the surface for all 256 possible vertex states
// There are 12 edges.  For each entry in the table, if edge #n is intersected, then bit #n is set to 1

GLint aiCubeEdgeFlags[256]=
{
	0x000, 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c, 0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00, 
	0x190, 0x099, 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c, 0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90, 
	0x230, 0x339, 0x033, 0x13a, 0x636, 0x73f, 0x435, 0x53c, 0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30, 
	0x3a0, 0x2a9, 0x1a3, 0x0aa, 0x7a6, 0x6af, 0x5a5, 0x4ac, 0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0, 
	0x460, 0x569, 0x663, 0x76a, 0x066, 0x16f, 0x265, 0x36c, 0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60, 
	0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0x0ff, 0x3f5, 0x2fc, 0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0, 
	0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x055, 0x15c, 0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950, 
	0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0x0cc, 0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0, 
	0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc, 0x0cc, 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0, 
	0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c, 0x15c, 0x055, 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650, 
	0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc, 0x2fc, 0x3f5, 0x0ff, 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0, 
	0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c, 0x36c, 0x265, 0x16f, 0x066, 0x76a, 0x663, 0x569, 0x460, 
	0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac, 0x4ac, 0x5a5, 0x6af, 0x7a6, 0x0aa, 0x1a3, 0x2a9, 0x3a0, 
	0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c, 0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x033, 0x339, 0x230, 
	0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c, 0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x099, 0x190, 
	0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c, 0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x000
};

//  For each of the possible vertex states listed in aiCubeEdgeFlags there is a specific triangulation
//  of the edge intersection points.  a2iTriangleConnectionTable lists all of them in the form of
//  0-5 edge triples with the list terminated by the invalid value -1.
//  For example: a2iTriangleConnectionTable[3] list the 2 triangles formed when corner[0] 
//  and corner[1] are inside of the surface, but the rest of the cube is not.
//
//  I found this table in an example program someone wrote long ago.  It was probably generated by hand

GLint a2iTriangleConnectionTable[256][16] =  
{
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
	{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
	{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
	{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
	{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
	{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
	{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
	{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
	{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
	{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
	{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
	{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
	{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
	{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
	{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
	{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
	{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
	{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
	{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
	{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
	{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
	{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
	{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
	{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
	{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
	{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
	{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
	{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
	{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
	{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
	{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
	{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
	{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
	{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
	{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
	{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
	{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
	{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
	{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
	{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
	{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
	{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
	{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
	{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
	{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
	{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
	{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
	{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
	{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
	{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
	{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
	{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
	{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
	{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
	{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
	{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
	{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
	{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
	{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
	{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
	{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
	{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
	{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
	{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
	{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
	{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
	{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
	{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
	{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
	{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
	{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
	{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
	{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
	{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
	{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
	{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
	{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
	{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
	{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
	{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
	{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
	{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
	{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
	{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
	{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
	{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
	{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
	{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
	{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
	{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
	{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
	{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
	{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
	{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
	{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
	{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
	{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
	{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
	{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
	{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
	{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
	{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
	{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
	{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
	{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
	{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
	{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
	{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
	{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
	{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
	{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
	{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
	{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
	{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
	{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
	{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
	{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
	{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
	{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
	{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
	{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
	{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
	{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
	{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
	{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
	{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
	{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
	{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};



namespace Alice 
{

	////////////////////////////////////////////////////////////////////////// MATH UTILS 

	DLL_API int     DLL_CALL	ofNextPow2 ( int a );
	DLL_API void    DLL_CALL	ofSeedRandom();
	DLL_API void 	DLL_CALL	ofSeedRandom(int val);
	DLL_API float 	DLL_CALL	ofRandom(float val0, float val1);	
	DLL_API float   DLL_CALL	ofRandomf();						
	DLL_API float 	DLL_CALL	ofRandomuf();						
	DLL_API float	DLL_CALL	ofNormalize(float value, float min, float max);
	DLL_API float	DLL_CALL	ofMap(float value, float inputMin, float inputMax, float outputMin, float outputMax);
	DLL_API float	DLL_CALL	ofClamp(float value, float min, float max);
	DLL_API float	DLL_CALL	ofLerp(float start, float stop, float amt);
	DLL_API float	DLL_CALL	ofDist(float x1, float y1, float x2, float y2);
	DLL_API float	DLL_CALL	ofDistSquared(float x1, float y1, float x2, float y2);
	DLL_API int		DLL_CALL	ofSign(float n);
	DLL_API bool	DLL_CALL	ofInRange(float t, float min, float max);
	DLL_API float	DLL_CALL	ofRadToDeg(float radians);
	DLL_API float	DLL_CALL	ofDegToRad(float degrees);
	DLL_API float	DLL_CALL	ofRandomWidth();
	DLL_API float	DLL_CALL	ofRandomHeight();
	

	////////////////////////////////////////////////////////////////////////// GL FUNCTIONS 

	DLL_API bool DLL_CALL checkFramebufferStatus() ;
	DLL_API int DLL_CALL printOglError(char *file, int line)  ; 
	DLL_API void DLL_CALL printShaderInfoLog(GLuint obj) ;
	DLL_API void DLL_CALL printProgramInfoLog(GLuint obj) ;
	DLL_API void DLL_CALL checkGLErrors(const char *label) ;
	DLL_API void DLL_CALL createTextures(GLuint &texNum, GLvoid *pixels, int width, int height,	GLenum internalFormat, GLenum uploadType, GLenum uploadFormat) ;
	DLL_API void DLL_CALL copyToTexture(GLuint &texNum, GLvoid *pixels, int width, int height,GLenum internalFormat, GLenum uploadType,	GLenum uploadFormat) ;
	DLL_API char * DLL_CALL  textFileRead(char *fn)  ;
	DLL_API void DLL_CALL setShaders( char *fn_vs, char *fn_fs , GLuint &prog , GLuint &vs_id , GLuint &fs_id ) ;
	DLL_API void DLL_CALL processPixels( int wd,int ht ) ;
	DLL_API void DLL_CALL attachTexturesToFBO( GLuint &fbo , GLenum attachment , GLuint texture ) ;
	DLL_API void DLL_CALL generateBuffer( GLuint &vbo , GLuint sz ) ;
	DLL_API void DLL_CALL transferToGPU( float data[][4] , int wd , GLuint &texture ,bool _texturesInited);
	DLL_API void DLL_CALL unPackBufferToVbo( GLenum &readBuffer , GLuint &vbo , int dim );
	DLL_API void DLL_CALL drawTexture( GLuint texture , int w );

	////////////////////////////////////////////////////////////////////////// VECTOR CLASS

	class DLL_API vec
	{
	public:
		double x,y,z ;
		vec();
		vec(double _x, double _y , double _z );	
		bool operator == (vec other) ;
		void operator += (vec other) ;
		void operator -= (vec other) ;
		vec operator + (vec other) ;
		vec operator - (vec other) ;
		void operator *= (double fac) ;
		vec operator * (double fac) ;
		bool vec::operator < (vec &other);
		vec operator / (double fac);
		void operator /= (double fac) ;
		vec operator /= (double &fac) ;
		double operator *= (vec &other) ;
		double operator * (vec &other) ;
		double mag() ;
		vec normalise() ;
		double distanceTo( vec other ) ;
		vec cross( vec &b ) ;
		double angle(vec b) ;
		double static angle_2D(double x1, double y1, double x2, double y2);		
		vec moveAlongDir( float d , vec a = vec(0,0,0));
		void print() ;
	};

	DLL_API double DLL_CALL cotangent( vec &a,  vec &b,  vec &c) ;
	////////////////////////////////////////////////////////////////////////// VECTOR2 CLASS // sfrom song ho
	class DLL_API Vector2
	{
		float x;
		float y;

		// ctors
		Vector2() : x(0), y(0) {};
		Vector2(float x, float y) : x(x), y(y) {};

		// utils functions
		void        set(float x, float y);
		float       length() const;                         //
		float       distance(const Vector2& vec) const;     // distance between two vectors
		Vector2&    normalize();                            //
		float       dot(const Vector2& vec) const;          // dot product
		bool        equal(const Vector2& vec, float e) const; // compare with epsilon

		// operators
		Vector2     operator-() const;                      // unary operator (negate)
		Vector2     operator+(const Vector2& rhs) const;    // add rhs
		Vector2     operator-(const Vector2& rhs) const;    // subtract rhs
		Vector2&    operator+=(const Vector2& rhs);         // add rhs and update this object
		Vector2&    operator-=(const Vector2& rhs);         // subtract rhs and update this object
		Vector2     operator*(const float scale) const;     // scale
		Vector2     operator*(const Vector2& rhs) const;    // multiply each element
		Vector2&    operator*=(const float scale);          // scale and update this object
		Vector2&    operator*=(const Vector2& rhs);         // multiply each element and update this object
		Vector2     operator/(const float scale) const;     // inverse scale
		Vector2&    operator/=(const float scale);          // scale and update this object
		bool        operator==(const Vector2& rhs) const;   // exact compare, no epsilon
		bool        operator!=(const Vector2& rhs) const;   // exact compare, no epsilon
		bool        operator<(const Vector2& rhs) const;    // comparison for sort
		float       operator[](int index) const;            // subscript operator v[0], v[1]
		float&      operator[](int index);                  // subscript operator v[0], v[1]

		friend Vector2 operator*(const float a, const Vector2 vec);
		friend std::ostream& operator<<(std::ostream& os, const Vector2& vec);
	};
	////////////////////////////////////////////////////////////////////////// VECTOR4 CLASS // from song ho

	class DLL_API Vector4
	{
	public:
		float x;
		float y;
		float z;
		float w;

		// ctors
		Vector4() : x(0), y(0), z(0), w(0) {};
		Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {};

		// utils functions
		void        set(float x, float y, float z, float w);
		float       length() const;                         //
		float       distance(const Vector4& vec) const;     // distance between two vectors
		Vector4&    normalize();                            //
		float       dot(const Vector4& vec) const;          // dot product
		bool        equal(const Vector4& vec, float e) const; // compare with epsilon

		// operators
		Vector4     operator-() const;                      // unary operator (negate)
		Vector4     operator+(const Vector4& rhs) const;    // add rhs
		Vector4     operator-(const Vector4& rhs) const;    // subtract rhs
		Vector4&    operator+=(const Vector4& rhs);         // add rhs and update this object
		Vector4&    operator-=(const Vector4& rhs);         // subtract rhs and update this object
		Vector4     operator*(const float scale) const;     // scale
		Vector4     operator*(const Vector4& rhs) const;    // multiply each element
		Vector4&    operator*=(const float scale);          // scale and update this object
		Vector4&    operator*=(const Vector4& rhs);         // multiply each element and update this object
		Vector4     operator/(const float scale) const;     // inverse scale
		Vector4&    operator/=(const float scale);          // scale and update this object
		bool        operator==(const Vector4& rhs) const;   // exact compare, no epsilon
		bool        operator!=(const Vector4& rhs) const;   // exact compare, no epsilon
		bool        operator<(const Vector4& rhs) const;    // comparison for sort
		float       operator[](int index) const;            // subscript operator v[0], v[1]
		float&      operator[](int index);                  // subscript operator v[0], v[1]

		friend Vector4 operator*(const float a, const Vector4 vec);
		friend std::ostream& operator<<(std::ostream& os, const Vector4& vec);
	};



	////////////////////////////////////////////////////////////////////////// GEOMETRIC UTILITIES

	DLL_API void DLL_CALL closestPointsOn3dLines( vec &a,vec &b,vec &c,vec &d, vec &p1, vec &p2 ) ;
	
	////////////////////////////////////////////////////////////////////////// DRAW UTILITIES
	
	DLL_API void DLL_CALL wireFrameOn();
	DLL_API void DLL_CALL wireFrameOff();
	DLL_API vec4 DLL_CALL getColour(double v,double vmin,double vmax) ;
	DLL_API void DLL_CALL drawCube(  vec &min , vec &max , vec &origin = vec(0,0,0) , bool lines = false , vec4 clr = vec4(1,1,1,1)) ;
	DLL_API void DLL_CALL drawPoint( vec &a ) ;
	DLL_API void DLL_CALL drawLine( vec a , vec b ) ;
	DLL_API void DLL_CALL drawCircle( vec cen , float r , int n ) ;
	DLL_API void DLL_CALL drawRectangle( vec &min, vec &max ) ;
	DLL_API void DLL_CALL drawString ( string s, vec pt ) ;
	DLL_API void DLL_CALL drawString ( string s, float x, float y ) ;
	DLL_API void DLL_CALL drawPlane( vec &norm , vec &cen , float scale = 1.0 ,vec &B = vec(0,0,0) , vec &T = vec(0,0,0) );
	
	////////////////////////////////////////////////////////////////////////// VIEWPORT UTILITIES
	
	DLL_API void DLL_CALL setGridSize( float sz ) ;
	DLL_API void DLL_CALL drawGrid( int sz ) ;
	DLL_API vec4 DLL_CALL backGround( float r = 1 , float g = 1 , float b = 1 , float a = 1 ) ;
	DLL_API void DLL_CALL updateCamera(vec lookAt = vec(0,0,0)) ;
	DLL_API void DLL_CALL resetCamera() ;
	DLL_API void DLL_CALL topCamera() ;
	DLL_API void DLL_CALL perspCamera() ;
	DLL_API void DLL_CALL setCamera( float z , float rx, float ry, float _tx,float _ty) ;
	DLL_API void DLL_CALL getCamera(float &z, float &rx, float &ry, float &_tx, float &_ty);
	DLL_API void DLL_CALL enableLight( GLfloat light_pos[4] ) ;
	DLL_API void DLL_CALL resetProjection() ;
	DLL_API void DLL_CALL setup2d()  ;
	DLL_API void DLL_CALL restore3d() ;
	DLL_API void DLL_CALL processHits( GLint numHits, GLuint *nameBuffer , int * hits ) ;
	DLL_API void DLL_CALL processHits( GLint numHits, GLuint *nameBuffer , int *hits ,  int *selSeq , int &seqCnt  ) ;
	DLL_API void DLL_CALL setupSelection( GLuint *nameBuffer, GLint x, GLint y ) ;
	DLL_API void DLL_CALL resetSelection( GLuint numHits , GLuint *nameBuffer , int *hits );
	DLL_API void DLL_CALL resetSelection( GLuint numHits , GLuint *nameBuffer , int *hits ,  int *selSeq , int &seqCnt );

	DLL_API void DLL_CALL reshape(int w, int h);
	DLL_API void DLL_CALL Motion(int x,int y);
	DLL_API void DLL_CALL Mouse(int b,int s,int x,int y);
	DLL_API void DLL_CALL idle();


	////////////////////////////////////////////////////////////////////////// FRAME BUFFER
	
	class DLL_API frameBuffer
	{

	public:

		// -------------------  internal storage 
		int w,h,prev_w,prev_h ;
		float arr[MAX_W*MAX_H][4];

		//frame buffer object
		GLuint fbo;
		GLuint fboTex,depthTex; 
		//GLuint mDepth ;
		
		//vertex buffer object - currently not used ;should be used to (debug)display the saved resulting texture etc.
		// or for post-process glsl shaders etc ;
		GLuint vbo , colVbo , indicesVbo;

		int cnt ;
		BYTE* bmpBuffer ;
		// -------------------  state storage 

		bool glewInited ;
		bool buffersInited ;
		bool texturesInited ;
		bool bmpBufferInited ;

		//////////////////////////////////////////////////////////////////////////  

		frameBuffer() ;
		~frameBuffer();
		void Init( int _w , int _h );
		int initBuffers();
		void beginRecord( float r = 1 , float g = 1 , float b = 1 , float a = 1   );		
		void endRecord( bool save , int maxFramesToSave = 2 , GLuint fboToBind = 0 );
		void snapshot( char* filename , int max , GLuint G_fbo );
		void drawTexture( GLfloat *proj_matrix , GLfloat *mv_matrix );
		
	};

	frameBuffer buffer ;
	DLL_API void DLL_CALL setPrintScreenAttribs( int &w , int &h , int &nf , bool reset = true ) ;
	DLL_API void DLL_CALL setSaveFrame( bool _saveFrame );


	
	//DLL_API int DLL_CALL read_jpeg_file( char *filename , unsigned char *raw_image );
	////////////////////////////////////////////////////////////////////////// IMPORTER

	struct node
	{
		vec pos ; // global pos
		vec normal ;
		int edgesIds[MAX_NBORS] ;
		int numEdges ;
		int fixed ;

		node(){};

	};

	struct edge
	{
		int n0,n1 ; // node indices
		edge(){};
	};


	class DLL_API importer
	{

	public : 

		// ------------------ importer variables 

		string Line;
		string fileToRead ;
		int pt_cnt;
		int max_pts ;
		int max_edges ;
		int max_faces ;
		float scaleFac ;

		// ------------------ storage arrays 

		multimap<int,int> node_edge_map;
		node *nodes ; int nCnt ;
		edge *edges ; int eCnt ;
		int *faces ; int fCnt ; int fIdCount ;
		int *faceCounts ;


		////////////////////////////////////////////////////////////////////////// CONTRUCTORS / DESTRUCTORS 

		importer() ;
		~importer() ;
		importer( string _fileToRead , int _max_pts , float _scaleFac ) ;
		vector<string> splitString(const string& str, const string& delimiter = " ") ;
		

		////////////////////////////////////////////////////////////////////////// COMPUTE methods 
		
		void buildNodeEdgeMap( bool verbose ) ;	
		void readEdges() ;
		void readCurves() ;
		void readPts_p5() ;
		void readPts_normals() ;
		void read_obj() ;
		void writeFile( string outFileName = "C:/randPoints.txt" , string header = "" ) ;
		void boundingBox( vec &min, vec &max );

	};

	////////////////////////////////////////////////////////////////////////// AO SHADER

	class DLL_API AO_shader
	{

	public:

		// -------------------- properties ;

		float ao_fac ;

		// -----shader compile attributes ;
		GLuint v_glsl,f_glsl,p_aoGlsl;

		// -------------------- CPU memory 
		int w ; //= DIM ;
		int h ;// = DIM ;
		float	pos[DIM*DIM][4];
		float	norm[DIM*DIM][4];
		GLuint  indices[DIM*DIM*3]; // 3 ties the edges as pts ;
		int iCnt , vCnt ,vCntSqrt ;
		//float  norms[w*h][3] ;

		// -------------------- GPU memory  / textures
		// one texture each for positions, normals and vertex area ;
		// area is 4th element in normal texture.
		//p ={ { x,y,z},  { x,y,z} , ..}
		//n ={ { x,y,z,area},  { x,y,z,area} , ..}
		GLuint aoTextures[2];
		GLuint vertTex[2];

		// ----- frame buffer object
		GLuint fbo;
		// ----- vertex buffer object
		GLuint posVbo , colVbo ; 

		// -------------------- GPU memory  / textures / colorAttachments 
		GLenum writeBuffers[2] ; //= { GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT };
		GLenum readBuffers[2];
		// ----- ping pong toogle
		unsigned int pp_Index ;
		// ----- state booleans
		bool glewInited ;
		bool buffersInited ;
		bool texturesInited ;

		// debug output 
		GLfloat *txtBuffer ;

		// ----------------------------------------------------------------------------- CONSTRUCTORS
		AO_shader();
		AO_shader( float _ao_fac );
		
		int initGPUBuffers( );
		int createDataOnGPU( int wd, int ht );

		// ----------------------------------------------------------------------------- COMPUTE
		
		void unPackBuffersToVbo() ;
		void process( int wind_w , int wind_h , GLuint fboToBind = 0  );

		// ----------------------------------------------------------------------------- UTILITIES

		void addVertexPoint(vec &p ,vec &n , float &ar );
		void addIndex( int &id );
		void updateGPUData();
		void snapshot( const char *filename , int max = 0 );
		
		// ----------------------------------------------------------------------------- DISPLAY
		void display( bool dispAOData = false , GLuint fboToBind = 0 );

	};



	
	////////////////////////////////////////////////////////////////////////// PARTICLE LIB

	typedef struct {
		double m;   /* Mass                          */
		vec ori_p;
		vec p;      /* Position                      */
		vec v;      /* Velocity                      */
		vec f;      /* Force                         */
		int fixed;  /* Fixed point or free to move   */
	} PARTICLE;
	typedef struct {
		vec dpdt;
		vec dvdt;
	} PARTICLEDERIVATIVES;
	typedef struct {
		int from;
		int to;
		double springconstant;
		double dampingconstant;
		double restlength;
		bool off ;
	} PARTICLESPRING;
	typedef struct {
		int otherId0;
		int otherId1;
		int nid ;
		double springconstant;
		double dampingconstant;
		double restAngle;
		vec f_dir0 ;
		vec f_dir1 ;
	} PARTICLEROTATESPRING;
	typedef struct {
		int from;
		int to;
		int fromCharge ;
		int toCharge ;
		bool repel ;
		float dist ;
	} PARTICLECHARGE;

	class DLL_API physics
	{

	public : 
		
		// ---------------------  sim storage
		
		PARTICLE *p;
		PARTICLESPRING *s;
		PARTICLEROTATESPRING *rs;
		PARTICLECHARGE *c;

		double gravity , viscousdrag;
		vec down;

		int np ,ns , nrs, nc ;
		multimap<int,int> n_s_map;
		
		bool calcRotate,calcCharge,calcSprings,calcGravity,calcCustom ;

		/*voxGrid spaceDiv ;*/
		vec *particlePositions;
		bool collisions ;


		// ---------------------  display & calcs storage 

		float minD,maxD,minAng,maxAng,minL,maxL;
		double *linearDisps ;
		double *angDiffs ;
		double *rlDiffs ;
		float totalDisp ;
		bool dispCalculated ;

		// ---------------------  selection storage 

		int *particleHits ;
		int *selSeq ;
		int seqCnt ;

		// ---------------------  performance 

		float fps ;
		////////////////////////////////////////////////////////////////////////// INITIALISE 

		physics( vec _gravity = vec(0,0,-1) ) ;
		~physics() ;
		void Setup(int np,int ns , int nc , int nrs );

		////////////////////////////////////////////////////////////////////////// COMPUTE 
		void calcSpringForces( int ns, PARTICLESPRING * s, PARTICLE * p ) ;
		void calcChargeForces(  PARTICLECHARGE * c, PARTICLE * p , int nc  ) ;
		void calcRotationalForces( PARTICLE * p ) ;
		virtual void calcCustomForces_pre(PARTICLE *p);
		virtual void calcCustomForces_post(PARTICLE *p);
		void CalculateForces(PARTICLE *p, int np, PARTICLESPRING *s, int ns, PARTICLECHARGE *c, int nc);
		void UpdateParticles(double dt = 0.1 ,int method = 2);
		virtual void calcCollisions(PARTICLEDERIVATIVES *deriv);
		void CalculateDerivatives( PARTICLE *p,int np,PARTICLEDERIVATIVES *deriv);
		
		////////////////////////////////////////////////////////////////////////// UTILITIES 

		int makeParticle(  vec pos = vec(0,0,0) , float mass = 1.0 , bool fixed = false );
		int makeSpring( int to , int from , float rl = SPR_RL , float springConstant = SPR_CONST , float dampingConstant = SPR_DAMP_CONST );
		int makeSpring( int to , int from , float &rl_fac );
		int makeAttraction( int to, int from, float charge = C_CHARGE ,float dist = C_DIST , int repel = true );
		int makeRotateSpring( int n , int otherId0 , int otherId1 , float restAngle = 0.0 ,float springConst = R_SPR_CONST  );
		void reset( int _ns = 0 );
		double edgeLength( int i );
		float rotateSpringAngle( int i);
		void buildNodeEdgeMap( bool verbose );
		int * getSprings( int nodeId , int &cnt );
		int getOtherEnd( int springId , vec fromPt );
		void cyclicSortSprings( int *s_ids , int numEdges , vec node );

		////////////////////////////////////////////////////////////////////////// DATA 

		void calcEdgeDisplacements();
		void calcLinearDisplacements();
		void calcAngularDisplacements() ;
		void calcAllDisplacements();

		
		////////////////////////////////////////////////////////////////////////// DISPLAY 
		void drawSprings( int lineWidth );
		void drawHinges();
		void drawHingeForces();
		void displayStats();
		void display( int lineWidth = 1 , int pointSize = 10 ,bool showIds = false , bool showHinges = false );
		void drawParticles( int pSz = 10 );
		void drawDeviations(  vec origin ,float x , float y ,float ht , int _typ );
		void drawGraphs( float x, float y , vec origin , float ht , double *vals , int n  , float minl, float maxl, string txt );
		
		////////////////////////////////////////////////////////////////////////// SELECTION
		
		void makeSpringsBetWeenSelected( int *ids = NULL );
		void makeSelectedFixed( int *ids = NULL , int n = 0 );
		void displaceSelected( vec disp = vec(0,0,1) , int *ids = NULL, int n = 0 );
		void clearSelection();
		/*void processHits( GLint hits, GLuint nameBuffer[] );*/
		void performSelection( GLint x, GLint y );

		////////////////////////////////////////////////////////////////////////// OUTPUT
		void writeFile( const char *filename , bool springs = false ) ;
	


	};


	////////////////////////////////////////////////////////////////////////// PS SHADER
	class Mesh;

	class DLL_API ParticleShader
	{

		public:

			// ------------ glsl programs
			GLuint v_glsl,f_glsl,p_psGlsl;
			static const int w  = DIM_PS ;
			static const int h  = DIM_PS ;

			// ------------ texture storage ;
			float	pos[w*h][4];
			float	vel[w*h][4];
			int sqrt_np  ; 
			
			float centers[w*h][4] ; // xyz & wt ;
			int sqrt_nc ;

			float triVerts[DIM_PS_TRIS*DIM_PS_TRIS][4] ; // p0,p1,p2..p0,p1,p2...p0,p1,p2.. ;
			int sqrt_ntv ;

			// ------------ textures , fbo,vbo
			GLuint posTex[2]; // position
			GLuint velTex[2]; // velocity
			GLuint cenTex ; // field sources
			GLuint triTex ; // col. tris
			//frame buffer object
			GLuint fbo;
			//vertex buffer object
			GLuint vbo , colVbo , indicesVbo;
			// ping-pong color attachments : position & velocity
			GLenum MRTBuffers[2][2] ; 

			// ------------ booleans & toggles
			unsigned int pp_Index ;
			bool glewInited ;
			bool buffersInited ;
			bool texturesInited ;
			bool cenTexInited; // text. for field source
			bool triTexInited; // text for collision triangles

			// ------------ ps system ;

			int np ; // num points
			int nc ; // num charges
			int nTrv ; // num collision Triangles
			int fr ; // frame count

			// ------------ fragment shader booleans

			int E,G,R,C,A;
			float attractRadius ;
			float gravityConstant ;
			float electricForceMultiplier;
			float  gravityAcceleration;
			float bounce;

			// output txt
			GLfloat* txtBuffer ;

			ParticleShader(){}
			ParticleShader( float ) ;			

			// ----------- buffers & CPU _ GPU - data transfer 
			
			int createDataOnGPU_centers( int wd, int ht ) ;
			int createDataOnGPU_tris( int wd, int ht ) ;
			int createDataOnGPU_particles( int wd, int ht ) ;
			int initGPUBuffers( ) ;
			// ----------- compute ;
			void updatePS( int wind_w , int wind_h ) ;
			// ----------- utilities
			void updateParams( float ar = 10.0 ,float gc = 0.001 ,float eFM = 0.5 , float gA = -1.1, float b = 2.0 ) ;
			void addTriVert( vec p , float w = 1, bool updateGPU = true ) ;
			void addCenter( vec p , float w = 1, bool updateGPU = true ) ;
			void addParticle( vec p , vec v = vec(0,0,0), bool updateGPU = false ) ;
			void resetParticle( int i , vec &p , vec &v = vec(0,0,0) ) ;
			void addTris( int num , vec *p  ) ;
			void addTrisFromOBJ( importer *obj  ) ;
			void addTrisFromMesh( Mesh &in  ) ;
			void addParticles( int num , vec *p , vec *v = NULL ) ;
			void addParticlesFromOBJ( importer *obj  ) ;
			void addParticlesOnSphere( vec cen , float r , int num , bool  updateGPU = false ) ;
			void addParticlesInBox( vec min , vec max , int xDivs , int yDivs , int zDivs  ) ;
			void addParticlesInBox( vec min , vec max , int num ) ;
			void addParticlesOnSpheresFromObj( importer *obj , float r , int numPerSphere  ) ;
			void addCenters( int num , vec *p , float *w = NULL ) ;
			void addCentersFromOBJ( importer *obj , float *wts = NULL ) ;
			void addCentersFromMesh( Mesh &M , float *wts = NULL ) ;
			void resetParticles( /*int num , vec *p , vec *v = NULL*/ ) ;
			// ------------------ output
			void snapshot( const char *filename , int max = 0 ) ;
			// ------------------ display
			void display( int winW, int winH , GLuint G_Fbo = 0 ) ;
	};


	////////////////////////////////////////////////////////////////////////// ISO-SURFACING 

	class DLL_API isoSurf
	{
		public:
			
			int     iDataSetSize ;
			float   fStepSize ;
			double   fTargetValue ;
			float boxDim ;
			int num ;
			vec  *charges ;
			float afVertexCubeValue[maxDataSize*maxDataSize*maxDataSize][8];
			float radius  ;

			vec b_min , b_max ;  ;
			bool accurateNormals ;
			bool colors ;
			bool filledCubeValues ;

			isoSurf() ;
			isoSurf( int _iDataSetSize , float _fTargetValue , int _numCharges  , vec *pts = NULL );
			isoSurf( int _iDataSetSize , float _fTargetValue  );
			~isoSurf() ;
			////////////////////////////////////////////////////////////////////////// --------------- UTILITIES 
			void addPoint( vec &pt ) ;
			int getPtsAndNormals( vec *iso_pts , vec *iso_norms );
			void writeFile( string outFileName = "C:/iso.obj");
			////////////////////////////////////////////////////////////////////////// --------------- COMPUTE 
			virtual void boundingBox();
			void increaseDensity();
			void decreaseDensity();
			void increaseThreashold() ;
			void decreaseThreashold();

			// calcs approx normal using cross products ;
			vec calcTriangleNormal( GLint iFlagIndex, GLint iTriangle, vec * asEdgeVertex ) ;
			void getNormal(vec &rfNormal, float fX, float fY, float fZ);
			//getNormal() finds the gradient of the scalar field at a point
			//This gradient can be used as a very accurate vertex normal for lighting calculations
			void normalizeVector(vec &rfVectorResult, vec &rfVectorSource);
			//getColor generates a color from a given position and normal of a point
			void getColor(vec &rfColor, vec &rfPosition, vec &rfNormal);
			// cube point calculation utility	
			float fGetOffset(float fValue1, float fValue2, float fValueDesired);
			//fSample finds the distance of (fX, fY, fZ) from three moving points
			virtual float  fSample(float x, float y, float z) ;
			float fillCubeValue( int i , float fX, float fY, float fZ, float fScale );
			void fillCubeValues( bool setTarget = true );
			void vMarchCube( int i , float fX, float fY, float fZ, float fScale ,  int &n, vec *pts = NULL , vec *clrs = NULL   );
			vec * getTrianglePoints( vec *iso_pts , int &n ) ;
			////////////////////////////////////////////////////////////////////////// --------------- DISPLAY  
			//vMarchingCubes iterates over the entire dataset, calling vMarchCube on each cube
			void display( bool _accNormals = false , bool _colors = false );

	};

	
	////////////////////////////////////////////////////////////////////////// SLIDER & SLIDER GROUP
	
	#define MAX_SLIDERS_PER_GROUP 50 
	#define DEFAULT_SLIDER_WIDTH 200 
	#define DEFAULT_SLIDER_HEIGHT 40
	#define DEFAULT_PADDING 5

	class DLL_API Slider
	{

	public:

		vec min,max;
		double val,minVal,maxVal ;
		double *referenceVar ;
		string varName ;
		bool attached ;
		int Id ;


		Slider();
		~Slider() ;			
		////////////////////////////////////////////////////////////////////////// CONSTRUCTORS

		Slider( vec _min, vec _max , int _Id = 0 );
		Slider( vec _min, float width , float height , int _Id = 0 , string _varName = "" );
		//////////////////////////////////////////////////////////////////////// COMPUTE

		int hit( int &x,int &y );
		void attachToVariable( double *_referenceVar , float _minVal = 0.0 , float _maxVal = 1.0 );
		void updateValue( int &x, int &y) ;// update dir - slider to variable 
		////////////////////////////////////////////////////////////////////////// DRAW
		void draw();
		////////////////////////////////////////////////////////////////////////// SELECTION



	};

	/////////////////// SLIDERGROUP

	class DLL_API SliderGroup
	{

	public:


		int *SliderHits ;
		Slider *sliders ;
		int numSliders;
		vec min,max;

		////////////////////////////////////////////////////////////////////////// CONSTRUCTORS
		
		SliderGroup( vec _min = vec(50,DEFAULT_PADDING*2,0) );
		~SliderGroup() ;
		////////////////////////////////////////////////////////////////////////// DRAW

		void addSlider();
		void addSlider( double *refVar, string _varName = "" );
		void addSlider( vec _min, vec _max );
		void addSlider( vec _min, float width , float height );
		void draw();
		////////////////////////////////////////////////////////////////////////// SELECTION

		void clearSelection();
		void performSelection(GLint x,GLint y,bool &HUDSelectOn);


	};
	

	#define MAX_BUTTONS_PER_GROUP 50 
	#define DEFAULT_BUTTON_WIDTH 150
	#define DEFAULT_BUTTON_HEIGHT 40
	#define DEFAULT_BUTTON_PADDING 10 

	class DLL_API Button
	{
		public:

		vec min,max;
		bool *referenceVar ;
		bool val ;
		bool attached ;
		string varName ;
		int Id ;


		Button();
		~Button();

		////////////////////////////////////////////////////////////////////////// CONSTRUCTORS

		Button( vec _min, vec _max , int _Id = 0, string _varName = "" ) ;
		Button( vec _min, float width , float height , int _Id = 0,string _varName = "" ) ;

		////////////////////////////////////////////////////////////////////////// COMPUTE

		int hit( int &x,int &y );
		void attachToVariable( bool *_referenceVar );
		void updateValue() ;
		////////////////////////////////////////////////////////////////////////// DRAW
		void draw();
		////////////////////////////////////////////////////////////////////////// SELECTION

	};

	class DLL_API ButtonGroup
	{

	public:


		int *ButtonHits ;
		Button *buttons ;
		int numButtons;
		vec min,max;

		////////////////////////////////////////////////////////////////////////// CONSTRUCTORS

		ButtonGroup( vec _min = vec(50,DEFAULT_BUTTON_PADDING*2,0) ) ;
		~ButtonGroup() ;

		////////////////////////////////////////////////////////////////////////// COMPUTE

		void addButton( string _varName = "") ;
		void addButton( bool *refVar , string _varName = "") ;
		void addButton( vec _min, vec _max ) ;
		void addSlider( vec _min, float width , float height ) ;
		////////////////////////////////////////////////////////////////////////// DRAW
		void draw() ;
		////////////////////////////////////////////////////////////////////////// SELECTION
		void clearSelection();
		void performSelection(GLint x,GLint y) ;


	};


	////////////////////////////////////////////////////////////////////////// MESH
	
	#define  MAX_VERTS 20000 
	#define  MAX_EDGES 10000 
	#define  MAX_FACES 20000 
	#define  MAX_VALENCE 30 
	#define MAX_SEL_VERTS 20

	class Vertex;
	class Edge;
	class Face;
	//////////////----------------------------------------- VERTEX
	class DLL_API Vertex
	{

	public :
		// --------------- 
		int id ;
		//Edge *edges ;
		Edge *edgePtrs[MAX_VALENCE] ; // array of pointers to edges of the mesh ;
		Face *vF[MAX_VALENCE] ;
		int n_e ;
		vec norm ;
		vec4 clr ;
		float curvature ;
		float planeDis ;

		//vec vPos, vPosCur, vPos1, vPos2, vPos3, vPos4;
		//vec vVel, vVelCur, vVel1, vVel2, vVel3, vVel4;
		//vec vForce;
		//double mass;
		// ------------- ------------- ------------- -------------------------- CONSTRUCTORS 
		Vertex();
		Vertex( int _id ) ;
		
		// ------------- ------------- ------------- -------------------------- TOPOLOGY 
		void addEdge( Edge *e ) ;

		// ------------- ------------- ------------- -------------------------- UTILITIES 
		int getFaces( Face *vFaces[] ) ;
		int getEdges(Edge *vEdges[] );
		int getVertices(Vertex *verts[]);
		bool onBoundary();
		vec vertexNormal(vec *positions   );
		vec vertexNormal(vec *positions , float *ar  );
		vec getAngleGradient( vec *positions ) ;
		float gaussCurvature(vec *positions );
		vec getMeanCurvatureGradient(vec *positions );
		vec getAngleGradient_cotangent( vec *positions );
		vec getDirichletGradient( vec *pos_3d , vec *pos_2d , double &E );
		vec getChiGradient( vec *pos_3d , vec *pos_2d , double &E );
		void draw( vec *positions );

	};

	//////////////----------------------------------------- EDGE

	class DLL_API Edge
	{

	public :
		int id ;
		Vertex *vStr;
		Vertex *vEnd ;
		Face *lFace ;
		Face *rFace ;

		// ------------- ------------- ------------- -------------------------- CONSTRUCTORS 
		Edge() ;
		Edge( Vertex &_vStr, Vertex &_vEnd , int _id) ;
		// ------------- ------------- ------------- -------------------------- TOPOLOGY 
		bool addFace( Face *f , bool addAsLeft ) ;
		// --------------------------------------------------------------------- UTILITIES
		float edgeLength( vec *p ) ;
		float edgeLengthSq( vec* p) ;
		// ------------- ------------- ------------- -------------------------- DISPLAY 
		void draw( vec *pos , float lw = 1 ) ;
		bool onBoundary() ;

	};

	//////////////----------------------------------------- FACE	
	class DLL_API Face
	{

	public :
		// --------------- 
		int id ;  
		Edge *edgePtrs[MAX_VALENCE] ;
		Vertex *fVerts[MAX_VALENCE] ;
		int n_e ;

		// --------------- 

		int f_v[MAX_VALENCE]  ;

		// ------------- ------------- ------------- -------------------------- CONTRUCTORS 
		Face() ;
		Face( int _id ) ;
		// ------------- ------------- ------------- -------------------------- TOPOLOGY 
		void addEdge( Edge *e ) ;
		// ------------- ------------- ------------- -------------------------- UTILITIES 
		int * faceVertices( /*int *n = NULL*/ ); 
		vec centroid( vec *positions ) ;
		vec normal( vec *positions , float *ar = NULL ) ;
		bool onBoundary() ;

		// ------------- ------------- ------------- -------------------------- DISPLAY
		void draw(vec *positions, bool wire, bool flipNormals = false, bool faceColors = false);
	};

	//////////////----------------------------------------- MESH

	class DLL_API Mesh
	{

	public:
		// ------------- topology 

		Vertex *vertices;
		Edge *edges ;
		Face *faces ;
		int n_e , n_v, n_f;  

		// ------------- attributes ;

		vec * positions ;
		vec * faceCenters ;

		// ------------- selection

		int *particleHits ;
		int *selSeq ;
		int seqCnt ; 

		// 
		float minC,maxC ;

		// ------------- ------------- ------------- -------------------------- CONSTRUCTORS  ;
		Mesh() ;
		
		// ------------- ------------- ------------- -------------------------- TOPOLOGY
		// VERTEX
		Vertex * createVertex( vec &p ) ;

		// EDGE
		Edge * createEdge( Vertex &str , Vertex &end ) ;

		//FACE 		
		Face *createFace( Vertex *f_verts[] , int n_f_v   ) ;
		void createNGon( Vertex *f_verts[] , int n_f_v , bool tri ) ;
		
		// ------------- ------------- ------------- -------------------------- UTILITIES - TOPOLOGY
		Edge * edgeExists( Vertex &str , Vertex &end , bool &found ) ;
		bool edgeExists_testOnly( Vertex &str , Vertex &end ) ;
		void printEuler() ;
		void calcCurvatures( vec *pos , int typ = 0 ) ;
		// ------------- ------------- ------------- -------------------------- TOPOLOGY - CONWAY ;
		// for creating duals ;
		void fillFaceCenters() ;
		Mesh dual() ;
		Mesh chamfer( float factor  ) ;
		Mesh triangulate(  ) ;
		Mesh triangulate( bool addCenter = false ) ;
		Mesh copy();
		void smoothBoundary( Vertex *b_verts[] , int n , vec *pos ) ;
		int getBoundary( /*vec *pos */  Vertex *b_verts[] ) ;

		// ------------- ------------- ------------- -------------------------- UTILITIES - I/O

		void boundingBox( vec &min, vec &max ) ;
		void writeOBJ( string outFileName , string header , vec *pos , bool removeBorder = false) ;

		// ------------- ------------- ------------- -------------------------- DISPLAY
		
		void drawVertices(  vec *pos , bool showIds = true ,int pSz = 4 ) ;
		void draw(bool wire = false);
		

		// -------------------------------------------------------------------- SELECTION

		void clearSelection() ;
		void processHits( GLint hits, GLuint nameBuffer[] ) ;
		void performSelection( GLint x, GLint y , vec *pos ) ;
		



	};

	//////////////----------------------------------------- MESH FACTOR

	class DLL_API MeshFactory
	{

	public:

		MeshFactory();
		Mesh createPrism( int nSides , float radius , float nTwist , bool tri ) ;
		Mesh createFromArrays( vec *p , int n_v , int *polyCounts , int n_f , int *polyConnects , bool triangulate = false );
		Mesh createFromOBJ(string file, float scale, bool center = false);
		Mesh createIcosahedron( float radius );
		Mesh createTetra( float radius );
		Mesh createPlatonic( float _radius , int n );
		

	};

	////////////////////////////////////////////////////////////////////////// 




	int numFrames = 25000 ;
	float zoom = 150.0f;
	float rotx = 0;
	float roty = 0.001f;
	float tx = 0;
	float ty = 0;
	int lastx=0;
	int lasty=0;
	unsigned char Buttons[3] = {0};
	int win = 0;
	int gridSz = 20 ;
	vec4 clearColor ;

	int winW = 800;
	int winH = 600 ;
	long frame = 0;

	GLfloat proj_matrix[16] , mv_matrix[16] ;
	bool saveF = false ;
	int screenW = winW ;
	int screenH = winH ;

	long startTime ;
	long elapsedTime ;
	float fps = 0 ;

	GLfloat vertices[] = {1,1,1,  -1,1,1,  -1,-1,1,  1,-1,1,   //TOPFACE
							1,1,1,  1,-1,1,  1,-1,-1,  1,1,-1,  
							1,1,1,  1,1,-1,  -1,1,-1,  -1,1,1,   
							-1,1,1,  -1,1,-1,  -1,-1,-1,  -1,-1,1,  
							-1,-1,-1,  1,-1,-1,  1,-1,1,  -1,-1,1,  
							1,-1,-1,  -1,-1,-1,  -1,1,-1,  1,1,-1 // bottom FACE
		};  
	// normal array
	GLfloat normals[] = {0,0,1,  0,0,1,  0,0,1,  0,0,1,  1,0,0,  1,0,0,  1,0,0, 1,0,0,  0,1,0,  0,1,0,  0,1,0, 0,1,0,
		-1,0,0,  -1,0,0, -1,0,0,  -1,0,0,  0,-1,0,  0,-1,0,  0,-1,0,  0,-1,0,  0,0,-1,  0,0,-1,  0,0,-1,  0,0,-1};        // v4-v7-v6-v5
	// color array
	GLfloat colors[] = {1,1,1,  1,1,0,  1,0,0,  1,0,1, 1,1,1,  1,0,1,  0,0,1,  0,1,1,  1,1,1,  0,1,1,  0,1,0,  1,1,0,  1,1,0,  0,1,0,  0,0,0,  1,0,0,              // v1-v6-v7-v2
		0,0,0,  0,0,1,  1,0,1,  1,0,0, 0,0,1,  0,0,0,  0,1,0,  0,1,1};             // v4-v7-v6-v5
	GLubyte indices[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
	

}






#endif 