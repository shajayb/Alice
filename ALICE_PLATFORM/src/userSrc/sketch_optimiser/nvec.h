

#include <math.h>
#include <vector>
#include <string>


////////////////////////////////////////////////////////////////////////// --------------------------- N-VECTOR 

#ifndef _NVEC
#define  _NVEC

class nvec
{
public :
	vector<float>x ;

	nvec(){};

	nvec( int n)
	{
		x.resize(n);
		x.assign(n,0.0);
	}	
	int size()
	{
		return x.size() ;
	}
	void resize( int n )
	{
		x.resize(n);
	}
	void copy( nvec &other )
	{
			if( x.size() != other.x.size() )other.resize(x.size());
		for( int i =0 ; i < x.size() ; i++ ) x[i] = other.x[i] ;

	}
	int operator += ( nvec &other)
	{
			if( x.size() != other.x.size() )return 0 ;
		for( int i =0 ; i < x.size() ; i++ )x[i] += other.x[i] ;
	}
	int operator -= (nvec &other)
	{
			if( x.size() != other.x.size() )return 0 ;
		for( int i =0 ; i < x.size() ; i++ )x[i] -= other.x[i] ;
	}

	nvec operator + (nvec &other)
	{
		nvec ret(x.size());
		ret.copy(*this);
		ret += other ;
		return ret  ;
		/**this += other ;
		return *this  ;*/
	}

	nvec operator - (nvec &other)
	{
		nvec ret(x.size());
		ret.copy(*this);
		ret -= other ;
		return ret ;
		/**this -= other ;
		return *this  ;*/
	}

	void operator *= (double fac)
	{
		for( int i =0 ; i < x.size() ; i++ )x[i] *= fac ;
	}


	void operator = (double fac)
	{
		for( int i =0 ; i < x.size() ; i++ )x[i] = fac ;
	}

	nvec operator * (double fac)
	{
		nvec ret(x.size());
		ret.copy(*this);
		ret *= fac ;
		return ret  ;

	}

	void operator /= (double fac)
	{
		for( int i =0 ; i < x.size() ; i++ )x[i] /= fac ;
	}

	nvec operator / (double fac)
	{
		/*nvec ret(x.size());
		ret.copy(*this);
		ret /= fac ;*/
		*this /= fac ;
		return ( *this )  ;
	}


	double operator *= (nvec &other)
	{
			if( x.size() != other.x.size() )return 0.0 ;
		float dotP = 0 ;
		for( int i =0 ; i < x.size() ; i++ ) dotP += x[i] * other.x[i] ;
		return ( dotP );
	}

	double operator * (nvec &other)
	{
		if( x.size() != other.x.size() )return 0.0 ;
		float dotP = 0 ;
		for( int i =0 ; i < x.size() ; i++ ) dotP += x[i] * other.x[i] ;
		return ( dotP );
	}

	double mag()
	{	
		return sqrt( (*this) * (*this) ) ;
	}
	
	nvec normalise()
	{
		double d = this->mag();
		for( int i =0 ; i < x.size() ; i++ ) x[i] /= d ;

		return *this ;
	}

	double distanceTo( nvec &other )
	{
		nvec ret(x.size()) ;
		ret = *this - other ;
		return ret.mag() ;
	}

	void print()
	{
		for( int i =0 ; i < x.size() ; i++ )cout << x[i] << " "  ;
		cout << " " << endl ;
	}

	double max()
	{
		double max_v = -1e32;
		for (int i = 0; i < x.size(); i++)max_v = MAX(max_v, x[i]);
		return max_v;
	}

	double min()
	{
		double min_v = 1e32;
		for (int i = 0; i < x.size(); i++)min_v = MIN(min_v, x[i]);
		return min_v;
	}
};



#endif

