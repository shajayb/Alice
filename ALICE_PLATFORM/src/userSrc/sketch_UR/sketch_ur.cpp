#include "main.h"
#include "ALICE_ROBOT_DLL.h"
using namespace ROBOTICS;
#include <array>
#include <memory>
using namespace std;

// testing snippets described in this video :
 // https://www.youtube.com/watch?v=U6mgsPqV32A&index=1&list=PL5jc9xFGsL8FWtnZBeTqZBbniyw0uHyaH

class newvec: public vec
{
public:
	string name;

	newvec() {};
	newvec(string _name) { name = _name; }
	~newvec() { cout << "destructor called " << name << endl; }

};

class Myclass
{
public:

	double x, y, z;
	string name;
	int type;// = 10; value initialisation will disable defualt initialiser list below.
	double decay;// = 10.0;

	auto print()
	{
		printf(" contents %1.2f,%1.2f,%1.2f \n", x, y, z);
		cout << name << endl;
		cout << type << endl;
		cout << decay << endl;
	}
	// change made at home
};




template<std::size_t SIZE>
void printArray(std::array<newvec, SIZE>& arr) // this function knows arg 1 is an std:: array, therefore all code compeletion etc will work
{

	cout << " -- printing inside array -- " << endl;
	for (auto& e : arr) e.print();
}

template< typename T>
void printArray_generictype(T & arr)// this function does not know arg 1 is an std:: array, therefore all code compeletion etc will work
{
	cout << " -- printing inside generic array -- " << endl;
	for (auto& e : arr) e.print();
}

void setup()
{
	// --------------------------// ------------------------------  dangling pointers tests;
	
	// objects are assigned to the heap and are deleted as soon as the scope expires.
	// pointers on the other hand have to be deleted.
	// smart pointers - unique_ptr<> and shared_ptr<> take care of the deletion automatically as soon as the scope expires.
	// i.e smart-pointer is an object that holds the raw pointer, and so you can use .to notation to access methods / fields of the smart-pointer itself
	// and -> notation, to access methods and fields of the object pointed at, by the smart-pointer.
	
	{
		newvec obj = *new newvec("test");
	}// destructor of obj / "test" is called here;

	newvec *ptr_to_a = new newvec(" testPointer ");
	delete  ptr_to_a;
	// destructor of  "test-pointer" is called here .. 
	// if delete not implemented here, *a will become a dangling pointer, because object of a (test pointer) will be deleted at the end of setup;
	
	newvec c("c");//destructor of "c" is called called when setup() completes;
	newvec("d"); // destructor called immediately as the object is created.. so useful, only if the ccontents are copied to LHS using = operator; g = newvec("d") ;

	// --------------------------// ------------------------------  shared pointers tests;

	// shared pointers - requires memory.h
	shared_ptr<newvec> f = make_shared<newvec>(" test shared ptr"); // arguments of constructor inside parenthesis
	shared_ptr<newvec> g = make_shared<newvec>(" shared ptr reciever "); // arguments of constructor inside parenthesis
	
	g->print(); // accessing methods of the object of 'g';
	g.get(); // accessing methods of the msart-pointer 'g' ;


	g = move(f); 
	// move ownership of the object owned by f, to g; 
	// thus the object owned by g originally (shared ptr reciever) is deleted immeidately.
	// g then recieves ownership of the object of f ( test shared ptr) ;
	// g itself is finally deleted when scope of setup() expires.
	// incidentally moving ownership is beteer especially with arrays, as copies are avoided.

	// --------------------------  // ------------------------------ initialiser list tests

	// uniform initilisation  - aggregate initilisation, initialiser lists, etc combined in unified curly brace syntax;
	Myclass H = { 10.0,1.0,0.0," instance",0,5.0 };
	H.print();
	
	// ------------------------------// ------------------------------ static fixed array container versus c-style arrays

	//c++ 11
	// array container is a thin-wrapper to c-style array, 
	// but is compatible with iterators and other enw features of c++11 such as auto;

	array<newvec, 10> T;
	for (auto &a : T)a.print(); // a iterator can be used to access the elements
	for (int i = 0; i < 10; i++)T[i].x += 1; // [] operators can still be used.

	// passing an array on non-static length or without explicitly requiring the length as a argument,,
	// requires using templates in the function definition - see printArray above.
	// template stuff seems an over-kill, but apparently thats the the lay of the land now.
	// too many templates is likely to slow down compilation (?)

	printArray(T);
	printArray_generictype(T);

	// c -style
	
	newvec Tprime[10];
	for (int i = 0; i < 10; i++)Tprime[i].print();

	// both std::array T and c-style array Tprime are deleted as the scope expires below.
}
	
void update(int value)
{


}

void draw()
{

	backGround(0.75);

}
void keyPress(unsigned char k, int xm, int ym)
{

		
}
void mousePress(int b, int state, int x, int y)
{

}
void mouseMotion(int x, int y)
{
	
}



