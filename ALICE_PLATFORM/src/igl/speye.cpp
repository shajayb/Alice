// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include "speye.h"

template <typename T>
IGL_INLINE void igl::speye(const int m, const int n, Eigen::SparseMatrix<T> & I)
{
  // size of diagonal
  int d = (m<n?m:n);
  I = Eigen::SparseMatrix<T>(m,n);
  I.reserve(d);
  for(int i = 0;i<d;i++)
  {
    I.insert(i,i) = 1.0;
  }
  I.finalize();
}

template <typename T>
IGL_INLINE void igl::speye(const int n, Eigen::SparseMatrix<T> & I)
{
  return igl::speye(n,n,I);
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template specialization
// generated by autoexplicit.sh
template void igl::speye<double>(int, Eigen::SparseMatrix<double, 0, int>&);
#endif
