#ifndef IGL_UNIQUE_EDGE_MAP_H
#define IGL_UNIQUE_EDGE_MAP_H
#include "igl_inline.h"
#include <Eigen/Dense>
#include <vector>
namespace igl
{
  // Constuct relationships between facet "half"-(or rather "viewed")-edges E
  // to unique edges of the mesh seen as a graph.
  //
  // Inputs:
  //   F  #F by 3  list of simplices
  // Outputs:
  //   E  #F*3 by 2 list of all of directed edges
  //   uE  #uE by 2 list of unique undirected edges
  //   EMAP #F*3 list of indices into uE, mapping each directed edge to unique
  //     undirected edge
  //   uE2E  #uE list of lists of indices into E of coexisting edges
  template <
    typename DerivedF,
    typename DerivedE,
    typename DeriveduE,
    typename DerivedEMAP,
    typename uE2EType>
  IGL_INLINE void unique_edge_map(
    const Eigen::PlainObjectBase<DerivedF> & F,
    Eigen::PlainObjectBase<DerivedE> & E,
    Eigen::PlainObjectBase<DeriveduE> & uE,
    Eigen::PlainObjectBase<DerivedEMAP> & EMAP,
    std::vector<std::vector<uE2EType> > & uE2E);

}
#ifndef IGL_STATIC_LIBRARY
#  include "unique_edge_map.cpp"
#endif

#endif
