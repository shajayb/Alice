#ifndef _INTERP
#define  _INTERP

#include "ALICE_DLL.h"
#define nPtsSegment 8

class Interpolator

{
public:

	vector<vec>anchors ;
	vec origin ;
	int w, h ;
	int numPtsPerSegment ;
	float dataMax, dataMin ;
	float paramMin, paramMax ;
	float xInc ;
	float requestedVal ;

	Interpolator()
	{
		origin = vec(0,0,0);
		numPtsPerSegment = nPtsSegment ;
	}

	Interpolator( int _n , vec _origin = vec(0,0,0), int _w = 550 , int _h = 50 , float *y =  NULL )
	{
		anchors.resize(_n);
		anchors.assign(_n, _origin);
		origin = _origin;
		w = _w ;h = _h ;
		numPtsPerSegment = nPtsSegment ;
		
		for( int i = 0 ; i < anchors.size() ; i++ )
		{
			anchors[i].x = origin.x + (float(w)/float(anchors.size()-1)) * float(i) ;
			anchors[i].y = origin.y ;
			anchors[i].y -= (y == NULL) ? ofRandom(0,h) : ofMap( y[i],0,1.0,0,h);
			printf(" data : %1.2f ,%1.2f \n" , anchors[i].x ,anchors[i].y ) ;
		}

		
		 
		dataMax = h ; 
		dataMin = 0.0 ;
		paramMin = 0.0 ; 
		paramMax = w ;
		requestedVal = 0 ;
	}

	void setDataAndParams( float _dataMin , float _dataMax, float _paramMin , float _paramMax )
	{
		dataMin = _dataMin ;
		dataMax = _dataMax ; 
		paramMin = _paramMin ; 
		paramMax = _paramMax ;
	}

	double CosineInterpolate( double y1,double y2,double mu )
	{
		double mu2;
		mu2 = (1-cos(mu*PI))/2;
		return(y1*(1-mu2)+y2*mu2);
	}

	float getValueAt(float paramX )
	{
		
		paramX = ofClamp(paramX,paramMin,paramMax);
		float x = ofMap(paramX,paramMin,paramMax,0,float(w));
	
		int a0,a1 ;	
		float xInc = float(1.0) / float( anchors.size()-1 ) ;
		
		int div = floor( (x/w) / xInc ) ;
		if( div == int(anchors.size()-1))div=anchors.size()-2;

		a0 = div ;
		a1 = div+1;
		
		x = requestedVal = ofMap(paramX,paramMin,paramMax,origin.x,origin.x+float(w));
		float mu = ( x - anchors[a0].x)/ ( anchors[a1].x - anchors[a0].x);
		float y = CosineInterpolate(anchors[a0].y-origin.y , anchors[a1].y-origin.y , mu );
		

		return fabs( ofMap(y,0,h,dataMin,dataMax) );
	}


	void performselection(int x, int y, bool &HUDSelecton)
	{

		int id = -1;
		float nearestD = 1e12;

		for (int i = 0; i < anchors.size(); i++)
		{
			float d = anchors[i].distanceTo(vec(x, y, 0) );

			if (d < nearestD && d < 25 ) id = i;
		}

		if (id != -1 )anchors[id] = vec(x, y, 0);

		HUDSelectOn = (id != -1) ? 1 : 0;
	}

	void draw()
	{
		
		setup2d();
		
		//data points
		glPointSize(4);
		char s[200];
		glColor3f(0.75,0.75,0.75);

			for( int i = 0 ; i < anchors.size() ; i++ )
			{
				drawPoint(anchors[i]);
				sprintf(s,"%1.2f", ofMap(fabs(anchors[i].y-origin.y),0,h,0,dataMax));
				AL_drawString(s,anchors[i].x , anchors[i].y - 15 );
			}
		glPointSize(1);
		
		// interp graph
		glColor3f(0,0,0);	
		glBegin(GL_LINE_STRIP);

			for( int i = 1 ; i < anchors.size() ; i++ )
			{
				float xInc = (anchors[i].x - anchors[i-1].x )/float(numPtsPerSegment);
				for( int j =0 ; j <= numPtsPerSegment ; j++ )
				{
					float x = anchors[i-1].x + xInc * j;
					float mu = j * 1.0/float(numPtsPerSegment);
					float y = CosineInterpolate(anchors[i-1].y , anchors[i].y , mu );
					glVertex2f(x,y);
				}
			}

		glEnd();

		// border
		vec or ;
		or = origin;
		or.y += h*0.1;
		glColor3f(0.75, 0.75, 0.75);
		drawLine(or,or+vec(w,0,0));
		
		or = origin;
		or.y-=h*1.1;
		drawLine(or,or+vec(w,0,0));

		//requestedVal
		drawLine( vec(requestedVal,origin.y-h*0.05,0) , vec(requestedVal,origin.y-h*0.95,0));



		restore3d();
	}

};

#endif