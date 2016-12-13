#include "pybind11.h"
namespace py = pybind11;

#include "ALICE_DLL.h"
using namespace Alice;


vec add(int i, int j) {

	vec a(5, 10, 0);
	vec b(10, 0, 0);
	vec c = a + b;

	return c;
}

namespace py = pybind11;

PYBIND11_PLUGIN(example) {
	py::module m("example", "pybind11 example plugin");

	m.def("add", &add, "A function which adds two numbers");

	return m.ptr();
}