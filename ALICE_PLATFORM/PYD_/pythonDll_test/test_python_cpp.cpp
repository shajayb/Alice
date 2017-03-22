#include "pybind11.h"
#include "constructor_stats.h"
#include "stl.h"
namespace py = pybind11;

// call_function.c - A sample of calling 
// python functions from C code
// 
#include <Python.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eval.h>

#include <cstdint>
#include <vector>

using namespace std;
using arr = py::array;
using arr_t = py::array_t<uint16_t, 0>;

class ExamplePythonTypes 
{
public:
	static ExamplePythonTypes *new_instance() {
		auto *ptr = new ExamplePythonTypes();
		print_created(ptr, "via new_instance");
		return ptr;
	}
	~ExamplePythonTypes() { print_destroyed(this); }

	/* Create and return a Python dictionary */
	py::dict get_dict() {
		py::dict dict;
		dict[py::str("key")] = py::str("value");
		return dict;
	}

	/* Create and return a Python set */
	py::set get_set() {
		py::set set;
		set.add(py::str("key1"));
		set.add(py::str("key2"));
		return set;
	}

	/* Create and return a C++ dictionary */
	std::map<std::string, std::string> get_dict_2() {
		std::map<std::string, std::string> result;
		result["key"] = "value";
		return result;
	}

	/* Create and return a C++ set */
	std::set<std::string> get_set_2() {
		std::set<std::string> result;
		result.insert("key1");
		result.insert("key2");
		return result;
	}

	/* Create, manipulate, and return a Python list */
	py::list get_list() {
		py::list list;
		list.append(py::str("value"));
		py::print("Entry at position 0:", list[0]);
		list[0] = py::str("overwritten");
		return list;
	}

	/* C++ STL data types are automatically casted */
	std::vector<std::wstring> get_list_2() {
		std::vector<std::wstring> list;
		list.push_back(L"value");
		return list;
	}

	/* C++ STL data types are automatically casted */
	std::array<std::string, 2> get_array() {
		return std::array<std::string, 2> { { "array entry 1", "array entry 2"}};
	}

	/* Easily iterate over a dictionary using a C++11 range-based for loop */
	void print_dict(py::dict dict) {
		for (auto item : dict)
			py::print("key: {}, value={}"_s.format(item.first, item.second));
	}

	/* Easily iterate over a set using a C++11 range-based for loop */
	void print_set(py::set set) {
		for (auto item : set)
			py::print("key:", item);
	}

	/* Easily iterate over a list using a C++11 range-based for loop */
	void print_list(py::list list) {
		int index = 0;
		for (auto item : list)
			py::print("list item {}: {}"_s.format(index++, item));
	}

	/* STL data types (such as maps) are automatically casted from Python */
	void print_dict_2(const std::map<std::string, std::string> &dict) {
		for (auto item : dict)
			py::print("key: {}, value={}"_s.format(item.first, item.second));
	}

	/* STL data types (such as sets) are automatically casted from Python */
	void print_set_2(const std::set<std::string> &set) {
		for (auto item : set)
			py::print("key:", item);
	}

	/* STL data types (such as vectors) are automatically casted from Python */
	void print_list_2(std::vector<std::wstring> &list) {
		int index = 0;
		for (auto item : list)
			py::print("list item {}: {}"_s.format(index++, item));
	}

	/* pybind automatically translates between C++11 and Python tuples */
	std::pair<std::string, bool> pair_passthrough(std::pair<bool, std::string> input) {
		return std::make_pair(input.second, input.first);
	}

	/* pybind automatically translates between C++11 and Python tuples */
	std::tuple<int, std::string, bool> tuple_passthrough(std::tuple<bool, std::string, int> input) {
		return std::make_tuple(std::get<2>(input), std::get<1>(input), std::get<0>(input));
	}

	/* STL data types (such as arrays) are automatically casted from Python */
	void print_array(std::array<std::string, 2> &array) {
		int index = 0;
		for (auto item : array)
			py::print("array item {}: {}"_s.format(index++, item));
	}

	void throw_exception() {
		throw std::runtime_error("This exception was intentionally thrown.");
	}

	py::bytes get_bytes_from_string() {
		return (py::bytes) std::string("foo");
	}

	py::bytes get_bytes_from_str() {
		return (py::bytes) py::str("bar", 3);
	}

	py::str get_str_from_string() {
		return (py::str) std::string("baz");
	}

	py::str get_str_from_bytes() {
		return (py::str) py::bytes("boo", 3);
	}

	void test_print(const py::object& obj) {
		py::print(py::str(obj));
		py::print(py::repr(obj));
	}

	static int value;
	static const int value2;
};

void printPythonArray(py::array_t<double> array)
{
	py:print(array);
}

struct double3
{
	double x[3];
};

struct float3
{
	float x[3];
};

struct int3
{
	int x[3];
};

int
main(int argc, char *argv[])
{

	Py_Initialize();

		printf(" \n");
		printf(" \n");
		printf(" \n");
		printf(" -   ------------------------------------------------------------------------------------------------------------ \n");
		printf(" -   ------------------------------------------------------------------------------------------------------------ \n");
		printf(" -   ------------------------------------------------------------------------------------------------------------ \n");

		//// calling a python function defined in a python module, and processing the result in C++
		//printf(" -  function calling --------------------------- \n");
		//{

		//	// get c++ pointer to the function (forceD), defined in module funcTest;
		//	py::object func_ptr = py::module::import("funcTest").attr("forceD");
		//	// use function pointer to call the function, the RHS is the pointer to the return value of the function.
		//	py::object result_py = func_ptr.call();
		//	// cast return pointer as appropriate type - in this case an array (numpy)
		//	py::array resA = result_py.cast<py::array>();;

		//	// use pointer to numpy array to access the underlying buffer - info_res.ptr
		//	py::buffer_info info_res = resA.request();
		//	if ((info_res.ndim != 1))
		//				std::cout << "Number of dimensions must be one" << std::endl;
		//	else
		//	{
		//		std::vector<double> ret(info_res.shape[0]);
		//		for (unsigned int idx = 0; idx < info_res.shape[0]; idx++)
		//			ret[idx] = ((double*)info_res.ptr)[idx];

		//		for (auto c : ret)printf("%1.2f,\n", c);
		//		printf(" ---------------------------------------- \n");
		//	}

		//}

		////alternatively you can use the derived class of py::function 
		printf(" -  alternate function calling , with 1D numpy return --------------------------- \n");
		{
			//cast function (forceD), defined in module funcTest, as py::function
			py::function f = py::module::import("funcTest").attr("forceD");

			// use function object to call function, as per argument signature
			py::object result_py = f();
			// cast return pointer as appropriate type - in this case an array (numpy)
			py::array resA = result_py.cast<py::array>();;

			// use pointer to numpy array to access the underlying buffer - info_res.ptr
			py::buffer_info info_res = resA.request();
			if ((info_res.ndim != 1))
				std::cout << "Number of dimensions must be one" << std::endl;
			else
			{
				std::vector<double> ret(info_res.shape[0]);
				for (unsigned int idx = 0; idx < info_res.shape[0]; idx++)
					ret[idx] = ((double*)info_res.ptr)[idx];

				for (auto c : ret)printf("%1.2f,\n", c);
			
			}
		}

		printf(" -  numpy 2D array function return --------------------------- \n");
		// 2D numpy array -> c++ vector<int3> / vector<double3> depending of dtype.kind() : see next ;
		{
			//cast function (forceD), defined in module funcTest, as py::function
			py::function f = py::module::import("funcTest").attr("forceD");

			// use 
			py::object result_py = f();
			// cast return pointer as appropriate type - in this case an array (numpy)
			py::array resA = result_py.cast<py::array>();;

			// use pointer to numpy array to access the underlying buffer - info_res.ptr
			py::buffer_info info_res = resA.request();
			if ((info_res.ndim != 2))
				std::cout << "Number of dimensions must be two" << std::endl;
			else
			{
				std::vector<double3> ret(info_res.shape[0]); // determining the dtype.kind of the returned numpy array is important, else iteration below will fail.
															// additionally, it appears python dtype = 'f' --> c++ type double 
				for (unsigned int idx = 0, cnt = 0; idx < info_res.shape[0] * 3; idx += 3, cnt++)
				{
					ret[cnt].x[0] = ((double*)info_res.ptr)[idx];
					ret[cnt].x[1] = ((double*)info_res.ptr)[idx + 1];
					ret[cnt].x[2] = ((double*)info_res.ptr)[idx + 2];
				}

				for (auto c : ret)printf("%1.2f,%1.2f,%1.2f \n", c.x[0], c.x[1], c.x[2]);
				printf(" ---------------------------------------- \n");
			}
		}

		//getting the type of a numpy array
		printf(" -  type of numpy array --------------------------- \n");
		{
			py::function f = py::module::import("funcTest").attr("forceD");
			py::object result_py = f();
			//// cast return pointer as appropriate type - in this case an array (numpy)
			py::array resA = result_py.cast<py::array>();;
			py::dtype dt = resA.dtype();
			char typ = dt.kind();
			cout << typ << endl;
		}

	Py_Finalize();
	return 1;
	

}


//
////from http://pybind11.readthedocs.io/en/master/advanced/pycpp/numpy.html
//auto buf1 = input1.request(), buf2 = input2.request();
//double *ptr1 = (double *)buf1.ptr,
//*ptr2 = (double *)buf2.ptr,
//for (size_t idx = 0; idx < buf1.shape[0]; idx++)
//	ptr3[idx] = ptr1[idx] + ptr2[idx];
