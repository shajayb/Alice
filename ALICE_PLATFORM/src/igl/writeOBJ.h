// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_WRITEOBJ_H
#define IGL_WRITEOBJ_H
#include "igl_inline.h"
// History:
//  return type changed from void to bool  Alec 20 Sept 2011

#include <Eigen/Core>
#include <string>

namespace igl 
{
  // Write a mesh in an ascii obj file
  // Inputs:
  //   str  path to outputfile
  //   V  eigen double matrix #V by 3 (mesh vertices)
  //   F  eigen int matrix #F by 3 (mesh indices)
  // Returns true on success, false on error
  template <typename DerivedV, typename DerivedF>
  IGL_INLINE bool writeOBJ(
    const std::string str,
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F);
  
  template <typename DerivedV, typename DerivedF, typename DerivedT>
  IGL_INLINE bool writeOBJ(
    const std::string str,
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedV>& CN,
    const Eigen::PlainObjectBase<DerivedF>& FN,
    const Eigen::PlainObjectBase<DerivedT>& TC,
    const Eigen::PlainObjectBase<DerivedF>& FTC);

}

#ifndef IGL_STATIC_LIBRARY
#  include "writeOBJ.cpp"
#endif

#endif
