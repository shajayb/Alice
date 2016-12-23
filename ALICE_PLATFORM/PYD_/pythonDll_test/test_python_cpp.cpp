#include "pybind11.h"
#include "constructor_stats.h"
#include "stl.h"
namespace py = pybind11;

// call_function.c - A sample of calling 
// python functions from C code
// 
#include <Python.h>

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

int
main(int argc, char *argv[])
{
	PyObject *pName, *pModule, *pDict, *pFunc;
	PyObject *pArgs, *pValue;
	int i;

	if (argc < 3) {
		fprintf(stderr, "Usage: call pythonfile funcname [args]\n");
		return 1;
	}

	Py_Initialize();
	pName = PyUnicode_DecodeFSDefault(argv[1]);
	/* Error checking of pName left out */

	pModule = PyImport_Import(pName);
	Py_DECREF(pName);

	if (pModule != NULL) {
		pFunc = PyObject_GetAttrString(pModule, argv[2]);
		/* pFunc is a new reference */

		if (pFunc && PyCallable_Check(pFunc)) {
			pArgs = PyTuple_New(argc - 3);
			for (i = 0; i < argc - 3; ++i) {
				pValue = PyLong_FromLong(atoi(argv[i + 3]));
				if (!pValue) {
					Py_DECREF(pArgs);
					Py_DECREF(pModule);
					fprintf(stderr, "Cannot convert argument\n");
					return 1;
				}
				/* pValue reference stolen here: */
				PyTuple_SetItem(pArgs, i, pValue);
			}
			pValue = PyObject_CallObject(pFunc, pArgs);
			Py_DECREF(pArgs);
			if (pValue != NULL) {
				//printf("Result of call: %ld\n", PyLong_AsLong(pValue));
				Py_DECREF(pValue);

				ExamplePythonTypes P;
				py::int_ I;
				I = *new py::int_(PyLong_AsLong(pValue));
				

				//
				py::handle h_PO(pValue); // PyObject_CallObject(pFunc, pArgs);
				py::object result_py(h_PO, true);
				py::print(result_py);
				int res = result_py.cast<int>();
				std::cout << res << std::endl;
			}
			else {
				Py_DECREF(pFunc);
				Py_DECREF(pModule);
				PyErr_Print();
				fprintf(stderr, "Call failed\n");
				return 1;
			}
		}
		else {
			if (PyErr_Occurred())
				PyErr_Print();
			fprintf(stderr, "Cannot find function \"%s\"\n", argv[2]);
		}
		Py_XDECREF(pFunc);
		Py_DECREF(pModule);
	}
	else {
		PyErr_Print();
		fprintf(stderr, "Failed to load \"%s\"\n", argv[1]);
		return 1;
	}
	Py_Finalize();
	return 0;
}