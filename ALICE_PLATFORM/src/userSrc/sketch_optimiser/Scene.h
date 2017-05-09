#include "sketch_FDM_constraints/optimiser.h"
#include "sketch_FDM_constraints/linAlg.h"


bool runPSO = false ;
bool runGA  = false ;
bool runGR  = false ;
bool runGSA = false;

#define PENALTY_SCORE  pow(10.0,10.0)

void AL_glLineWidth( float wid) 
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

void AL_drawString(float angle, const char *string, float x, float y)
{
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

int numCoefs = 3;
void fillBxi(VectorXd &Bxi, RowVector3d &pt)
{
		Bxi(0) = 0;
		Bxi(1) = 1;
		Bxi(2) = pt(0) * pt(0);
		//Bxi(3) = 0;

}


double wendlandRadius = 0.1;


class Scene : public Model
{
	
public : 
	
	
	// scene objects needed for evaluation and candidate set-up
	Mesh imp ;
	int num_e ;
	vector<g_Vert> GV;
	vector<g_Edge> GE;
	double min_q,max_q;
	float dAbovePlane;
	float dAbovePlane_threshold;

	nvec plane_distances;
	nvec orig_plane_distances;
	nvec initialQ;

	VectorXd Bxi, GradXi, Ax;
	RowVector3d X, pt;

	Scene(){} ;
	Scene( double _min_q,double _max_q )
	{
	
		MeshFactory fac;
		imp = fac.createFromOBJ("data/in_ccf_o.obj", 10, false ,true);
		//for (int i = 0; i < imp.n_v; i++)imp.positions[i].z = 0;

		//for

		//meshToGraph(imp,GV,GE);
		textGraphToGraph("data/in_dualNet.txt", GV, GE);
		num_e = GE.size() ;
		cout << " paramSize " << num_e << endl ;
		min_q = _min_q ;
		max_q = _max_q ;
		dAbovePlane = 12.01; 
		dAbovePlane_threshold = dAbovePlane * 0.1;

		plane_distances.resize(GV.size());
		plane_distances = 0.0;
		orig_plane_distances.resize(GV.size());
		orig_plane_distances = 0.0;
		// ------------------------- 

		UBound = nvec(num_e);
			UBound.x.assign(num_e,max_q);
		LBound = nvec(num_e);
			LBound.x.assign(num_e,min_q);
		
		numFnCalls = 0 ;
	}
	
	void generateCandidate( Candidate &C )
	{
		C.P  = nvec(num_e);
		C.PB = nvec(num_e);
		C.V  = nvec(num_e);

		for( int i =0 ; i < num_e ; i++ )C.P.x[i]  = ( ofRandom( LBound.x[i] , UBound.x[i] ) );
		C.PB = C.P ;

		for( int i =0 ; i < num_e ; i++ )C.V.x[i]  = ( ofRandom( LBound.x[i] , UBound.x[i] )  );
		
	}

	float evalGraphScore(vector<g_Vert> &graphVerts , nvec &dists  )
	{

		float score = 0.0;
		vec norm;
		for (int i = 0; i < graphVerts.size(); i++)
		{

			if (graphVerts[i].boundary)continue;

			vec o = graphVerts[i].pos;
			vec cen;
			Vertex *verts[6];
			int n = imp.vertices[i].getVertices(verts);
			for (int j = 0 , next = 0; j < n; j++)
			{
				next = (j + 1) % n;
				norm += (graphVerts[verts[j]->id].pos - o).cross(graphVerts[verts[next]->id].pos - o);
				cen += graphVerts[verts[j]->id].pos;
			}

			cen /= n;
			norm.normalise();

			if (norm * vec(0, 0, 1) < 0)norm *= -1;
			float d = (o - cen)*norm;

				dists.x[i] = d;

			score += (d);

		}

		//for (int i = 0; i < graphVerts.size(); i++)score -= 0.5 * (graphVerts[i].orig - graphVerts[i].pos).mag();
		//cout << "score" << score << endl;
		return score;
	}
	void evaluate( Candidate &C )
	{
		
		float score = 0.0;
		
		// ----------------------------------------------------------------------- 

		for( int i =0 ; i < GV.size() ; i++ )GV[i].pos = GV[i].orig ;
		int suc = FDM(GV, GE, C.P, -1.0, NULL);

		iterativeFDM(GV, GE, C.P, -1.1 * 1.5, imp);;//
		if( !suc)
		{
			
			numFnCalls ++ ;
			C.P.mag(); // lazy eval ? without print/mag the scores are weird !
			C.score = C.actual_score = PENALTY_SCORE ;// over-ridden function has to ensure that individual cadidate scores are updated ;
			return ;
		}

		/*float sqDistSum;
		for (int i = 0; i < GV.size(); i++)sqDistSum += ((GV[i].pos - GV[i].orig) * (GV[i].pos - GV[i].orig));
		score = sqDistSum ;*/

		score = evalGraphScore(GV,plane_distances);


		// ----------------------------------------------------------------------- 

		C.score = C.actual_score = -score ; // over-ridden function has to ensure that individual cadidate scores are updated ;
		numFnCalls ++ ;
	}
	double getFunctionAtX( RowVector3d &X , nvec vals)
	{


		Bxi.resize(numCoefs, 1);
		Bxi.setZero(numCoefs, 1);

		MatrixXd BT, Fv, A, B;
		VectorXd wt(vals.size());
		Fv.resize(vals.size(), 1);
		BT.resize(vals.size(), Bxi.rows());

		int nBor = 2;
		int str = X(0) - nBor > 0 ? X(0) - nBor : 0;
		int end = (X(0) + nBor) < vals.size() ? (X(0) + nBor) : vals.size();
		
		vals.normalise();

		for (int i = str; i < end ; i++)
		{
			RowVector3d xi(i, 0, 0); //  = Xi.row(N[i]);
			RowVector3d pt = xi - X;

			double d = pt.norm(); // sqrtf(pt * pt.transpose());

			fillBxi(Bxi, xi);

			wt[i] =  1.0 - vals.x[i];// pow(1.0 - d / wendlandRadius, 4) * (4.0 * d / wendlandRadius + 1);;
			BT.row(i) = Bxi.transpose();
			Fv.row(i) << vals.x[i];// Fi(N[i]);
		}


		Diag W = wt.asDiagonal();
		A = BT.transpose() * W * BT;
		B = BT.transpose() * W * Fv;
		//Ax = A.ldlt().solve(B);

		Ax = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B); //A.colPivHouseholderQr().solve(B);


		fillBxi(Bxi, X);
		double d = Bxi.dot(Ax);
		return d;
	}

	void draw( vector<g_Vert> & GV , vector<g_Edge> &GE , nvec densities = NULL , bool f_clr = true )
	{
		
		min_q = 1e32;
		max_q = -1e32;
		for (int i = 0; i < densities.size(); i++)
		{
			min_q = MIN(densities.x[i], min_q);
			max_q = MAX(densities.x[i], max_q);
		}
		max_q += 0.001;

		// --------------------- draw interior nodes of graph;
		//wireFrameOn();
		//glEnable(GL_POINT_SMOOTH);
		glColor3f(0,0,0);
		AL_glPointSize(6);

		glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
		glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glEnable(GL_POINT_SMOOTH);
		glEnable(GL_LINE_SMOOTH);

		AL_glLineWidth(2);
		for( int i =0 ; i < GV.size() ; i++ )
			if (!GV[i].boundary)
			{
				drawPoint(GV[i].pos);
				if (plane_distances.x[i] < dAbovePlane_threshold)drawCircle(GV[i].pos, 0.55, 32);
			}
				

		// --------------------- draw boundary nodes of graph ;
		AL_glPointSize(8);
		glColor3f(1,0,0);
		for( int i =0 ; i < GV.size() ; i++ )
			if (GV[i].boundary)drawPoint(GV[i].pos);

		//// --------------------- draw edges of graph
		AL_glLineWidth(4);
		glColor3f(0.2,0.2,0.2);
		for( int i =0 ; i < GE.size() ; i++ )
		{
			if(densities.size() > 0 && f_clr )
			{
				vec4 clr = getColour( densities.x[i],min_q,max_q);
				glColor3f(clr.r,0,0);
				float l = clr.r * 0.57 + clr.g * 0.33 + clr.b * 0.1;
				glColor3f(clr.r > 0.5 ? clr.r : l, 0 , l);
				glColor3f(clr.r, clr.g, clr.b);
			}


			//AL_glLineWidth(ofMap(densities.x[i], min_q, max_q, 1, 8));
				drawLine( GV[ GE[i].to ].pos, GV[ GE[i].from ].pos) ;
		}

		AL_glLineWidth(1);
		AL_glPointSize(1);
	}

	void drawGraph(vec origin, float wd, float ht, nvec &vals )
	{
		double min_v = vals.min();
		double max_v = vals.max();
		int n = vals.size();
		float xInc = float(wd) / float(n);
		

		setup2d();
			glColor3f(0, 0, 0);
			AL_glLineWidth(1);
		
			glBegin(GL_LINE_STRIP);
				for (int i = 0; i < n; i++)glVertex2f(float(i)*xInc + origin.x, origin.y - ofMap((vals.x[i]), min_v, max_v, 0.1, ht*0.9));
			glEnd();
		restore3d();
	}

	void drawHistogram(vec &origin, float wd, float ht, nvec & vals, string txt  )
	{

		
		setup2d( /* */);

		//maxVal
		glColor3f(0, 0, 0);
		//drawString(txt, origin.x, origin.y - ht);
		AL_drawString(0., txt.c_str(), origin.x, origin.y - ht);
			//draw
			int n = vals.size();
			float xInc = float(wd) / float(n);
			float base = origin.y;


			// draw graph
			AL_glLineWidth(3);
			glEnable(GL_LINE_SMOOTH);

		
			double max_v = vals.max();
			double min_v = vals.min();
			max_v += 0.001;

			for (int i = 0; i < n; i++)
			{

				float d = (vals.x[i]);
				vec4 clr = getColour(d, min_v, max_v);

				glBegin(GL_LINES);

					glColor3f(clr.r, clr.g, clr.b);
					float l = clr.r * 0.57 + clr.g * 0.33 + clr.b * 0.1;
					glColor3f(clr.r > 0.5 ? clr.r : l, 0,l);
					glColor3f(clr.r, clr.g, clr.b);
					glVertex2f(float(i)*xInc + origin.x, origin.y - ofMap(d, min_v, max_v, ht*0.05, ht*0.9));
					glVertex2f(float(i)*xInc + origin.x, origin.y);

				glEnd();

			}

		glDisable(GL_LINE_SMOOTH);
		
			glBegin(GL_LINES);

				glColor3f(0,0,0);
				glVertex2f( origin.x, origin.y );
				glVertex2f( float(n)*xInc + origin.x, origin.y);

			glEnd();

		AL_glLineWidth(1);
		// restore 3d projection ;


		restore3d();

	}

	void drawScene( Optimiser &solver, int lineStyle = 6 )
	{
		

		// --------------------- draw embedding of topological graph ; 
		//FDM(GV, GE, solver.GB.P, -1.0, NULL);
		draw(GV,GE,solver.GB.P);
		
		//drawGraphStats(GV, GE, imp, dAbovePlane, plane_distances);
		// --------------------- draw force density histogram ; 
		vec origin(50,winH *0.50,0);
		float wd = solver.GB.P.size() * 15;
		float ht = 150;
		int n = solver.GB.P.x.size();
		
			//drawHistogram(origin, wd,ht, solver.GB.P, "force_densities per bar");
		AL_glEnable_lineStipple();
			//drawGraph(origin, wd, ht, initialQ );
		AL_glDisable_lineStipple();
		// --------------------- draw MLS smoothing graph of force densities  ; 

		/*nvec mls_vals(solver.GB.P.size());
			for (int i = 0; i < n; i++)mls_vals.x[i] = getFunctionAtX(RowVector3d(i, 0, 0), solver.GB.P);
		drawGraph(origin, wd, ht, mls_vals);
		*/
		
		// --------------------- draw vertex-plane distance histo-gram & graph mesh ; 
		origin.y += 250;

		float threshold_remap = ofMap(dAbovePlane_threshold, plane_distances.min(), plane_distances.max(), 0, ht*0.9);
		plane_distances.normalise()*= 0.5;
		setup2d();
			//drawLine(vec(origin.x, origin.y - threshold_remap, 0), vec(origin.x+wd, origin.y - threshold_remap, 0) );
		restore3d();

			//drawHistogram(origin, wd,ht, plane_distances,"distance_from_plane per node");
		AL_glEnable_lineStipple();
			drawGraph(origin, wd,ht,orig_plane_distances);
		AL_glDisable_lineStipple();
		// --------------------- draw voronoi-area weights for self-weight / loading ; 
		origin.y += 250;

			//drawHistogram(origin, wd, ht, voronoi_areas, "voronoi_area_weighted_loads per node");
		AL_glEnable_lineStipple();
			//drawGraph(origin, wd, ht, voronoi_areas);
		AL_glDisable_lineStipple();
		// --------------------- draw input mesh ; 

		AL_glEnable_lineStipple();
		switch (lineStyle)
		{
		case 0: glLineStipple(2, 0xffff); break; //doesnt work with gl2ps
		case 1: glLineStipple(2, 0x00ff); break; //doesnt work with gl2ps
		case 2: glLineStipple(2, 0xffff); break; //doesnt work with gl2ps
		case 3: glLineStipple(2, 0x0c0f); break;
		case 4: glLineStipple(2, 0x0c0f); break;
		case 5: glLineStipple(2, 0xaaaa); break; //doesnt work with gl2ps
		case 6: glLineStipple(15, 0xaaaa); break;//doesnt work with gl2ps
		default:glLineStipple(3, 0xaaaa); break;
		}

		glLineStipple(0.25, 0x0c0f); 
		//glLineStipple(15, 0xaaaa);
		//

		//for (int i = 0; i < imp.n_v; i++)imp.positions[i] = GV[i].orig;
		//for (int i = 0; i < imp.n_v; i++)imp.vertices[i].clr = vec4(0, 0, 0, 1);
		
				glColor3f(0, 0, 0);
				for (int i = 0; i < GE.size(); i++)
				{
					drawLine(GV[GE[i].from].orig, GV[GE[i].to].orig);
				}
				//for (int i = 0; i < imp.n_e; i++)imp.edges[i].draw(imp.positions, 1);
			
		AL_glDisable_lineStipple();
		AL_glLineWidth(1);
			
			wireFrameOn();
				//imp.draw(false);
			wireFrameOff();

		for (int i = 0; i < imp.n_v; i++)imp.positions[i] = GV[i].pos;

	}

	void drawSearchSpace( Optimiser &solver )
	{
		int totalCandidates = solver.numSearches ;// solver.numC * solver.iteration ;
		float min = pow(10.0,20.0);
		float max = -min ;
		for( int i =0 ; i < totalCandidates ; i++ )
		{
			if( solver.searchSpace[i].actual_score == PENALTY_SCORE )continue ;
			if( solver.searchSpace[i].actual_score < min )min = solver.searchSpace[i].actual_score ;
			if( solver.searchSpace[i].actual_score > max )max = solver.searchSpace[i].actual_score ;
		}


		float phi = 2.0f * PI / float(totalCandidates);

		setup2d();

		float currentColor[4];
		glGetFloatv(GL_CURRENT_COLOR,currentColor) ;

		float maxRadius = 100 ;
		vec cen(winW - maxRadius*1.5,maxRadius*1.5,0);

		glPointSize(1);
		glBegin(GL_POINTS);

		for( int i = 0 ; i < totalCandidates ; i++ )
		{
				if( solver.searchSpace[i].actual_score == PENALTY_SCORE )continue ;
			float r = ofMap(solver.searchSpace[i].actual_score,min,max,1,maxRadius)  ;
			vec4 clr = getColour(solver.searchSpace[i].actual_score,min,max) ;
			glColor3f(clr.r,clr.g,clr.b);
			float x = r * cos( phi * float(i) ) ;
			float y = r * sin( phi * float(i) ) ;
			glVertex3f(cen.x +x ,cen.y + y ,0);
		}

		glEnd();

		glPointSize(1);
		glBegin(GL_LINES);

		vec str(winW - maxRadius *2.5 , winH - maxRadius * 3.0 , 0 );
		float xInc = maxRadius *2.0 / float(solver.numC) ;
		if( solver.iteration > 0 )
			for( int i = totalCandidates - solver.numC , cnt = 0; i < totalCandidates ; i+= 1, cnt++ )
			{
				if( solver.searchSpace[i].actual_score == PENALTY_SCORE )continue ;

				float r = ofMap(solver.searchSpace[i].actual_score,min,max,1,maxRadius )  ;
				float x = str.x + xInc*(float(cnt)) ;
				vec4 clr = getColour(solver.searchSpace[i].actual_score,min,max) ;
				glColor3f(clr.r,clr.g,clr.b);

				glVertex3f( x ,str.y - r ,0);
				glVertex3f( x ,str.y ,0);
				//glVertex3f( cen.x + x,cen.y + maxRadius * 2.5 + y + r ,0);

			}

			glEnd();


			restore3d() ;



			glColor3f(currentColor[0],currentColor[1],currentColor[2]);
			glPointSize(1);

	}

	
};

