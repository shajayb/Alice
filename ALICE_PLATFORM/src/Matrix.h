#ifndef _MATRIX
#define  _MATRIX

struct eigenPair
{
	vec eVec;
	double eVal;
};

class Matrix3x3
{

public:
	double entries[3][3];
	double a1,a2,a3,b1,b2,b3,c1,c2,c3;

	////////////////////////////////////////////////////////////////////////// CONSTRUCTORS
	Matrix3x3()
	{
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ )entries[i][j] = 0;
		entriesToCoef() ;
	}

	Matrix3x3( vec cl1, vec cl2, vec cl3 )
	{
		entries[0][0] = a1 = cl1.x ;
		entries[1][0] = a2 = cl1.y ;
		entries[2][0] = a3 = cl1.z ;

		entries[0][1] = b1 = cl2.x ;
		entries[1][1] = b2 = cl2.y ;
		entries[2][1] = b3 = cl2.z ;

		entries[0][2] = c1 = cl3.x ;
		entries[1][2] = c2 = cl3.y ;
		entries[2][2] = c3 = cl3.z ;
	}


	////////////////////////////////////////////////////////////////////////// SUB-MATRICIES & PROPERTIES

	double det()
	{
	 /*
	  a1 b1 c1 a1 b1 
	  a2 b2 c2 a2 b2 
	  a3 b3 c3 a3 b3
	 */ 
		return( a1*b2*c3 + b1*c2*a3 + c1*a2*b3 - a3*b2*c1 - b3*c2*a1 - c3*a2*b1 ) ;
	}

	Matrix3x3 static identity()
	{
		Matrix3x3 ret;
		for( int i = 0 ; i < 3 ; i++ )
			for( int j = 0 ; j < 3 ; j++ )ret.entries[i][j] = ( i == j ) ? 1 : 0 ;

		ret.entriesToCoef();
		return ret;
	}

	double trace()
	{
		return (a1+b2+c3);
	}

	double principalMinors()
	{
	  /*
	  a1 b1 c1  
	  a2 b2 c2 
	  a3 b3 c3 
	  */
		return ( (b2*c3-b3*c2) + (a1*c3 - a3*c1) + (a1*b2 - a2*b1));
		//return (a3*c1 + b3*c2 + a2*b1 - a1*b2 - a1*c3 - b2*c3);
	}


	////////////////////////////////////////////////////////////////////////// OPERATORS

	void operator -= (Matrix3x3 &other)
	{
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ )entries[i][j] -= other.entries[i][j];
		entriesToCoef();
	}

	Matrix3x3 operator - (Matrix3x3 &other)
	{
		Matrix3x3 ret;
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ ) ret.entries[i][j] = entries[i][j] - other.entries[i][j];

		ret.entriesToCoef();
		return ret ;
	}


	void operator += (Matrix3x3 &other)
	{
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ )entries[i][j] += other.entries[i][j];
		entriesToCoef();
	}

	Matrix3x3 operator + (Matrix3x3 &other)
	{
		Matrix3x3 ret;
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ ) ret.entries[i][j] = entries[i][j] + other.entries[i][j];

		ret.entriesToCoef();
		return ret ;
	}


	void operator *= (double mult)
	{
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ )entries[i][j] *= mult ;
		entriesToCoef();
	}

	Matrix3x3 operator * (double mult)
	{
		Matrix3x3 ret;
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ ) ret.entries[i][j] = entries[i][j] * mult ;
		
		ret.entriesToCoef();
		return ret ;
	}

	void operator /= (double mult)
	{
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ )entries[i][j] /= mult ;
		entriesToCoef();
	}

	Matrix3x3 operator / (double mult)
	{
		Matrix3x3 ret;
		for( int i =0 ; i < 3 ; i++ )
			for( int j =0 ; j < 3 ; j++ ) ret.entries[i][j] = entries[i][j] / mult ;

		ret.entriesToCoef();
		return ret ;
	}

	
	////////////////////////////////////////////////////////////////////////// MANIPULATORS
	
	void replace( vec cl , int col )
	{
		entries[0][col] = cl.x ;
		entries[1][col] = cl.y ;
		entries[2][col] = cl.z ;
		entriesToCoef();
	}


	
	////////////////////////////////////////////////////////////////////////// COMPUTE
	
	vec solve( vec R ) // solve non-homogenous via cramer's method
	{
		Matrix3x3 tmp ;

		tmp = *this ;
		tmp.replace(R,0);
		double Dx = tmp.det();

		tmp = *this ;
		tmp.replace(R,1);
		double Dy = tmp.det();

		tmp = *this ;
		tmp.replace(R,2);
		double Dz = tmp.det();

		double D = det() ;

			if( fabs(D) < EPS )return vec(0,0,0) ;

		return vec( Dx/D , Dy/D , Dz / D );
	}

	vec solve_gauss() // solve homogeneous via Gaussian elimination method
	{
		if( fabs(det()) > EPS )return vec(0,0,0); // only trivial solutions

		Matrix3x3 augM = *this ;
		int n = 3 ;

		for( int i=0 ; i < n ; i++ ) 
		{
			// Search for maximum in this column
			double maxEl = fabs(augM.entries[i][i]);
			int maxRow = i;
			for (int k=i+1; k < n; k++ ) 
			{
				if (fabs(augM.entries[k][i]) > maxEl) 
				{
					maxEl = fabs(augM.entries[k][i]);
					maxRow = k;
				}
			}

			// Swap maximum row with current row (column by column)
			for ( int k=i; k < n; k++ ) 
			{
				double tmp = augM.entries[maxRow][k];
				augM.entries[maxRow][k] = augM.entries[i][k];
				augM.entries[i][k] = tmp;
			}

			// Make all rows below this one 0 in current column
			for ( int k=i+1; k < n; k++ ) 
			{
				double c = -augM.entries[k][i]/ augM.entries[i][i];
				for ( int j=i; j < n; j++ ) 
				{
					if (i==j) 
						augM.entries[k][j] = 0;
					else 
						augM.entries[k][j] += c * augM.entries[i][j];
				}
			}
		}

		augM.entriesToCoef();
		/*cout << " augM " << endl ;
		augM.print() ;*/

		double x,y,z;
		if( fabs(augM.b2) < EPS )
		{
			z = 0 ;
			x = -augM.b1/augM.a1 ;
			y = 1.0 ;
		}
		else
		{
			y = -1 * augM.c2/augM.b2 ;
			x = -1 * (augM.b1*y +augM.c1)/augM.a1 ;
			z = 1.0 ;
		}

		return vec(x,y,z) ;

	}



	static Matrix3x3  covarianceMatrix( vec *points , vec &mean ,int N , double *wts=NULL )
	{
		Matrix3x3 cov;

		// calc means in x,y,z ;
		double sumW ;
		mean = vec(0,0,0);
		sumW=0;
		for (int i = 0; i < N; i++)mean += points[i] * ( wts == NULL ? 1.0 : wts[i]);
		if( wts !=NULL )for (int i = 0; i < N; i++)sumW += wts[i];
		mean /= ( wts == NULL ? double(N): sumW) ;

		// calc co-variance 
		for (int i = 0; i < 3; i++) 	
			for (int j = 0; j < 3; j++) 
			{
				for (int k = 0; k < N; k++)
					cov.entries[i][j] += ( getMean(mean,i) - getCoord(points[k],i)*( wts == NULL ? 1.0 : wts[k]) ) * 
					                     ( getMean(mean,j) - getCoord(points[k],j)*( wts == NULL ? 1.0 : wts[k]) );

				/*if( fabs(cov.entries[i][j]) > EPS )cov.entries[i][j] /= double(N - 1);
				else
					cov.entries[i][j] = 0.0 ;*/
				cov.entries[i][j] = ( fabs(cov.entries[i][j]) > EPS ) ? cov.entries[i][j]/double(N - 1) : 0.0 ;

			}
		cov.entriesToCoef();
		return cov ;
	}

	static vec  solveCubic_allReal(double a, double b, double c, double d)
	{

		vec ret ;

		b /= a;
		c /= a;
		d /= a;
		double disc, q, r, dum1, s, t, term1, r13;
		q = (3.0*c - (b*b))/9.0;
		r = -(27.0*d) + b*(9.0*c - 2.0*(b*b));
		r /= 54.0;
		disc = q*q*q + r*r;
		term1 = (b/3.0);

	/*	printf(" disc : %1.2f \n" , disc ) ;*/

		if( disc < 0 ) // all roots are real 
		{
			q = -q ;
			r13= 2 * sqrt(q);
			term1 = b/3.0 ; // powf((-t + s)/2, 1/3.);
			double q3 = pow(q,3.0) ;
			q3 = acos(r/sqrt(q3));
			ret.x = (-1*term1 +( r13 * cos(q3/3.0)));
			ret.y = (-1*term1 + r13 * cos( (q3 + 2*PI)/3.0) );
			ret.z = (-1*term1 + r13 * cos( (q3 + 4*PI)/3.0) );
		}

		if( disc == 0.0 /*fabs(disc) < EPS*/ )
		{
			r13 = ((r < 0) ? -1 * pow(-r,(1.0/3.0)) : pow(r,(1.0/3.0)));
			term1 = b/3.0 ; // powf((-t + s)/2, 1/3.);
			ret.x = -term1 + 2.0*r13;
			ret.y = ret.z = -(r13 + term1);
		}

		return ret ;
	}

	static void sortEigenPairs( vec *eigenVectors, vec &eigenValues )
	{
		eigenPair ev[3];
		for( int i =0 ; i < 3 ; i++ )
		{
			ev[i].eVal = getCoord(eigenValues,i);
			ev[i].eVec = eigenVectors[i] ;
		}

		for( int i =0 ; i < 3 ; i++ )
		{
			int next =  (i+1)%3 ;
			if( ev[next].eVal > ev[i].eVal )swap(ev[next],ev[i]) ;
		}

		eigenValues.x = ev[0].eVal;
		eigenValues.y = ev[1].eVal;
		eigenValues.z = ev[2].eVal;

		for( int i =0 ; i < 3 ; i++ )eigenVectors[i]=ev[i].eVec ;
	}
	static void findEigenVectors( Matrix3x3 &A, vec &eigenValues, vec* eigenVectors  ) 
	{
		
		// solve characteristic (cubic)polynomial for the eigenValues ;
		eigenValues = solveCubic_allReal( -1,A.trace(),-A.principalMinors(),A.det() );

		// calculate the eigen vectors for each of three eigenValues;
		Matrix3x3 I , AMinusLambdaI ;
		for( int i = 0 ; i < 3 ; i++ )
		{
			I = identity();
			I *= getCoord(eigenValues,i); // lambdaI
			AMinusLambdaI = A - I ; // A-lambdaI
			//if( fabs( AMinusLambdaI.det()) < EPS  )
			eigenVectors[i] = AMinusLambdaI.solve_gauss() ;

		}

		if( eigenVectors[1] == eigenVectors[2] )
		{
			eigenVectors[1] = eigenVectors[0].cross(vec(1,0,0));
			eigenVectors[2] = eigenVectors[0].cross(eigenVectors[1]);
			eigenValues.y = eigenValues.z = eigenValues.x+1;
			cout << "repeated eigen vals" << endl ;
		}

		sortEigenPairs(eigenVectors,eigenValues);
	}
	static vec *  PCA( string ptsFile,int &N,vec &mean, vec &eigenValues , vec* eigenVectors , double *wts=NULL )
	{
		
		// read points
		importer ptsReader( ptsFile ,  100000 , 1.0  );
		ptsReader.readPts_p5();
		vec *points  = new vec[ptsReader.nCnt]; // !!! TODO : this is a memory leak ... remove this method.
		N = ptsReader.nCnt ;

		// construct point array ;
		for( int i =0 ; i < N ; i++ )points[i] = ptsReader.nodes[i].pos ;

		// calculate covariance matrix
		Matrix3x3 cov  = covarianceMatrix(points,mean,N,wts);
		findEigenVectors(cov,eigenValues,eigenVectors);
		
		return points;
	}

	static void PCA( vec *points,int N, vec &mean, vec &eigenValues , vec* eigenVectors , double *wts=NULL )
	{
		// calculate covariance matrix
		Matrix3x3 cov  = covarianceMatrix(points,mean,N,wts);
		findEigenVectors(cov,eigenValues,eigenVectors);
	}


	

	////////////////////////////////////////////////////////////////////////// UTILITIES 

	void entriesToCoef()
	{
		a1 = entries[0][0] ;
		a2 = entries[1][0] ;
		a3 = entries[2][0] ;

		b1 = entries[0][1] ;
		b2 = entries[1][1] ;
		b3 = entries[2][1] ;

		c1 = entries[0][2] ;
		c2 = entries[1][2] ;
		c3 = entries[2][2] ;
	}

	static double  getMean( vec &mean , int &i )
	{
		if( i == 0 )return mean.x ;
		if( i == 1 )return mean.y ;
		if( i == 2 )return mean.z ;
	}

	static double  getCoord( vec &point , int &i )
	{
		if( i == 0 )return point.x ;
		if( i == 1 )return point.y ;
		if( i == 2 )return point.z ;
	}
	void print()
	{
		printf(" row1 : %1.16f %1.16f %1.16f \n" , a1 , b1 , c1 ) ;
		printf(" row2 : %1.16f %1.16f %1.16f \n" , a2 , b2 , c2 ) ;
		printf(" row3 : %1.16f %1.16f %1.16f \n" , a3 , b3 , c3 ) ;
	}


};


#endif