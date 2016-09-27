#include "main.h"




void setup() 
{
	
	////////////////////////////////////////////////////////////////////////// OBJECTS , REFERENCES and DE-REFERENCE OPERATOR ;

	int a = 20 ;
	cout << &a << endl ; // address or where ? -- reference operator
	cout << a << endl ; // value of object ---- what is inside ?
	cout << *(&a) << endl ; // value of object --- what is inside ? -- dereference operator ;

	// get address from object ---- use &(nameOfObject)  - where do you live ;
	// get object stored at address ---- use *(nameOfPointer) - what is inside ?

	// example of common use of dereference operator ;
	vec d = *new vec(1,2,3) ; // the new keyword returns a pointer NOT an object .. therefore, it has to be dereferenced to make LHS = RHS ;
	d.print() ;
	// constructors  return objects ; so all below are valid ;
	vec e = vec(1,2,3) ; 
	vec f(1,2,3) ;
	e.print();
	f.print();

	////////////////////////////////////////////////////////////////////////// POINTERS , POINTER TO AN ARRAY and ARRAY OF POINTERS ;
	
	// -------------------------------------------------------- pointer ;
	int *p ;  
	p = &a ;
	cout << p << endl ;
	cout << *p << endl;
	a = 10 ;
	cout << p << endl ;
	cout << *p << endl;

	// example of pointer and object syntax ;
	vec d = *new vec(1,2,3) ;
	vec *ptr_to_d ;
	ptr_to_d = &d ;
	ptr_to_d->print();
	d.print() ;

	// --------------------------------------------------------  pointer to an array ;

	int *q ; // variable of type pointer .. ie. a variable to store an address ;
	q = new int[10];// assign array memory at that address ;
	// q now stores the address of the first element of the array ;
	
	for( int i =0 ; i < 10 ; i++ )q[i] = i ; // store values in array 
	
	// retrieving values from array using array access ;
	
	for( int i =0 ; i < 10 ; i++ )
	{
		cout << q[i]  << " -- array access v" << endl ;
	}

	// retrieving values from array using pointer access ;
	for( int i =0 ; i < 10 ; i++ )
	{
		cout << *q << " -- pointer access v" << endl ;
		q++ ;
	}
	q -= 10 ; // puts pointer back to start of array ; .. this is important, otherwise the pointer is storing empty or invalid addresses ;

	// -------------------------------------------------------- array of pointers ;

	int *ptrs[10] ; // an array of 10 addresses / pointers ;
	for( int i =0 ; i < 10 ; i++ )ptrs[i] = new int(i) ;// assign a new integer value to each address ;
	for( int i =0 ; i < 10 ; i++ )cout << *ptrs[i] << " --- value at address " << endl ; // retrieve values at each address using dereference operator ;
	for( int i =0 ; i < 10 ; i++ )cout << ptrs[i] << " ---  address " << endl ; // access the addresses themselves ;

	//////////////////////////////////////////////////////////////////////////

	
}


void update( int value )
{
}

void draw()	
{
	backGround(0.75) ;

	
}

void keyPress (unsigned char k, int xm, int ym)
{

}

void mousePress(int b,int s,int x,int y) 
{
}