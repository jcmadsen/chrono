// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// Base Class for all opengl related classes
// =============================================================================

#ifndef CHOPENGLBASE_H
#define CHOPENGLBASE_H

#include <GL/glew.h>

#ifdef __APPLE__
#define GLFW_INCLUDE_GLCOREARB
#define GL_DO_NOT_WARN_IF_MULTI_GL_VERSION_HEADERS_INCLUDED  // fixes warnings
#endif

#define GLM_FORCE_RADIANS
#define _CRT_SECURE_NO_WARNINGS

#include <assert.h>
#include <iostream>
//#include <string>
//#include <iomanip>
//#include <fstream>
//#include <sstream>
//#include <limits>
//#include <time.h>
//#include <math.h>
//#include <vector>

#include "chrono_opengl/core/ChApiOpenGL.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
namespace chrono {
namespace opengl {

#ifndef BAD_GL_VALUE
#define BAD_GL_VALUE GLuint(-1)
#endif

class CH_OPENGL_API ChOpenGLBase {
 public:
  ChOpenGLBase() {}
  virtual ~ChOpenGLBase() {}

  // Children must implement this function
  virtual void TakeDown() = 0;

  // Check for opengl Errors and output if error along with input char strings
  bool GLReturnedError(const char* s) {
    bool return_error = false;
    GLenum glerror;
    // Go through list of errors until no errors remain
    while ((glerror = glGetError()) != GL_NO_ERROR) {
      return_error = true;
      std::cerr << s << ": " << gluErrorString(glerror) << std::endl;
    }
    return return_error;
  }
};
}
}
#pragma GCC diagnostic pop

#endif  // END of CHOPENGLBASE_H
