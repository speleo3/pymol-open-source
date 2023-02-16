/*
 * Surface Intersection Tesselator
 */

#include "CGO.h"

#include <memory>

std::unique_ptr<CGO> SurfaceZIntersection(PyMOLGlobals* G,
    const float* vertices,     // vertices
    const int* triindices,     // triangle indices
    int n_triangles,           // number of triangles
    float clip_z,              // z-axis clipping value
    const float* clip_normal); // orientation of the clipping plane
