/*
 * Surface Intersection Tesselator
 */

// #define POLYLINEJIGSAW_OPTIMIZATIONS
#define POLYLINEJIGSAW_DEBUG

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/vec3.hpp>

#include <algorithm>
#include <cassert>
#include <deque>
#include <map>

#ifdef POLYLINEJIGSAW_DEBUG
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#endif

#include "os_gl.h"
#include "os_std.h"

#ifdef _PYMOL_OSX
#import <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include "CGO.h"
#include "Matrix.h"
#include "Scene.h"
#include "Vector.h"

// missing on OS X
typedef void (*_GLUfuncptr)(void);

// maps a key to a polyline (series of points)
typedef std::map<int, std::deque<glm::vec3>> polylines_t;

class PolylinesTesselator
{
  typedef PolylinesTesselator this_type;

  GLUtesselator* m_tobj = nullptr;
  CGO* m_cgo = nullptr;

  // must be a data structure which provides pointers to float[3] data
  // which are not invalidated when adding more elements (see cbCombine)
  std::vector<std::vector<float>> m_extra_data;

  // CALLBACK FUNCTIONS
  //
  // The GLU_TESS_BEGIN function must be able to handle modes GL_TRIANGLES,
  // GL_TRIANGLE_FAN and GL_TRIANGLE_STRIP.

  static void cbBeginCGO(GLenum mode, this_type* this_)
  { //
    CGOBegin(this_->m_cgo, mode);
  }

  static void cbEndCGO(this_type* this_)
  { //
    CGOEnd(this_->m_cgo);
  }

  static void cbVertexCGO(const float* vertex, this_type* this_)
  { //
    CGOVertexv(this_->m_cgo, vertex);
  }

  /*
   * The "combine" function needs to allocate memory for a single vertex. The
   * data pointer will later be passed to the GLU_TESS_VERTEX or
   * GLU_TESS_VERTEX_DATA callback.
   */
  static void cbCombine(const GLdouble* coords, const void* d[4],
      const GLfloat* w, void** dataOut, this_type* this_)
  {
    this_->m_extra_data.emplace_back(coords, coords + 3);
    *dataOut = this_->m_extra_data.back().data();
  }

  static void cbError(GLenum err)
  {
    const GLubyte* msg = gluErrorString(err);
    printf("tesselation error %d: '%s'\n", err,
        msg ? (const char*) msg : "<NULL>");
  }

public:
  PolylinesTesselator()
  {
    m_tobj = gluNewTess();

    gluTessCallback(m_tobj, GLU_TESS_ERROR, (_GLUfuncptr) &cbError);
    gluTessCallback(m_tobj, GLU_TESS_COMBINE_DATA, (_GLUfuncptr) &cbCombine);

    gluTessProperty(m_tobj, GLU_TESS_WINDING_RULE, GLU_TESS_WINDING_ODD);
  }

  ~PolylinesTesselator()
  {
    gluDeleteTess(m_tobj);
    CGOFree(m_cgo);
  }

#ifndef PURE_OPENGL_ES_2
  /*
   * Setup callbacks for immediate mode rendering
   */
  void setupImmediate()
  {
    gluTessCallback(m_tobj, GLU_TESS_VERTEX, (_GLUfuncptr) &glVertex3fv);
    gluTessCallback(m_tobj, GLU_TESS_BEGIN, (_GLUfuncptr) &glBegin);
    gluTessCallback(m_tobj, GLU_TESS_END, (_GLUfuncptr) &glEnd);
  }
#endif

  /*
   * Setup callbacks for rendering to a CGO
   */
  void setupCGO(PyMOLGlobals* G)
  {
    assert(!m_cgo);
    m_cgo = CGONew(G);
    gluTessCallback(m_tobj, GLU_TESS_VERTEX_DATA, (_GLUfuncptr) &cbVertexCGO);
    gluTessCallback(m_tobj, GLU_TESS_BEGIN_DATA, (_GLUfuncptr) &cbBeginCGO);
    gluTessCallback(m_tobj, GLU_TESS_END_DATA, (_GLUfuncptr) &cbEndCGO);
  }

  /*
   * Tesselation
   */
  void tesselate(const polylines_t& polylines)
  {
    gluTessBeginPolygon(m_tobj, (GLvoid*) this);

    for (auto const& poly : polylines) {
      if (poly.second.empty())
        continue;

      gluTessBeginContour(m_tobj);

      for (auto const& vec : poly.second) {
        const float* fxyz = glm::value_ptr(vec);
        GLdouble dxyz[3];
        copy3(fxyz, dxyz);
        gluTessVertex(m_tobj,
            dxyz,            // (transient) vertex coordinates
            (GLvoid*) fxyz); // (persistent) vertex data for GLU_TESS_VERTEX
      }
      gluTessEndContour(m_tobj);
    }

    gluTessEndPolygon(m_tobj);
  }

  /*
   * Returns the generated CGO and releases the ownership. Calling this
   * function a second time will return NULL.
   */
  CGO* releaseCGO()
  {
    CGO* cgo = m_cgo;
    m_cgo = nullptr;
    return cgo;
  }
};

class PolylineJigsaw
{
  struct vec3_less_t {
    bool operator()(const glm::vec3& lhs, const glm::vec3& rhs) const
    {
      return lhs.x < rhs.x ||
             (lhs.x == rhs.x &&
                 (lhs.y < rhs.y || (lhs.y == rhs.y && lhs.z < rhs.z)));
    }
  };

  typedef std::map<glm::vec3, std::pair<int, bool>, vec3_less_t> tips_t;
  typedef tips_t::iterator tips_iter_t;
  typedef polylines_t::mapped_type polyline_t;

  tips_t tips;
  polylines_t polylines;
  int next_key = 0;

  /*
   * Return true if the iterator represents the end of a polyline
   */
  static bool IS_END(tips_iter_t& it) { return it->second.second; }

  /*
   * almost equal comparison, with consideration of magnitude of the operands
   */
  static bool almostequal(float lhs, float rhs, float eps = 1e-6)
  {
    eps = std::max(eps, std::abs(lhs * eps));
    return std::abs(lhs - rhs) <= eps;
  }

  static bool almostequal(
      const glm::vec3& lhs, const glm::vec3& rhs, float eps = 1e-6)
  {
    return almostequal(lhs.x, rhs.x, eps) && almostequal(lhs.y, rhs.y, eps) &&
           almostequal(lhs.z, rhs.z, eps);
  }

  /*
   * Optimization: approximate key lookup. Checks two keys for beeing almost
   * equal to vec: the first existing key which is >=vec and its predecessor
   * (last one which is <vec).
   */
  tips_iter_t tips_find(const glm::vec3& vec)
  {
#ifndef POLYLINEJIGSAW_OPTIMIZATIONS
    return tips.find(vec);
#else
    auto it = tips.lower_bound(vec);
    if (it != tips.end() && !almostequal(it->first, vec)) {
      if (it == tips.begin()) {
        it = tips.end();
      } else {
        --it;
        if (it != tips.end() && !almostequal(it->first, vec)) {
          it = tips.end();
        }
      }
    }
    return it;
#endif
  }

  /*
   * Optimization: Check if this polygon is closed and if yes, unregister
   * its tips from further lookups.
   */
  bool check_close(polyline_t& poly)
  {
#ifdef POLYLINEJIGSAW_OPTIMIZATIONS
    if (almostequal(poly.front(), poly.back())) {
      tips.erase(poly.front());
      tips.erase(poly.back());
      return true;
    }
#endif
    return false;
  }

  /*
   * Add a point to the end of a polyline
   */
  void poly_grow(tips_iter_t& it, const glm::vec3& vec)
  {
    bool end = IS_END(it);
    auto key = it->second.first;
    auto& poly = polylines[key];

    if (end) {
      poly.push_back(vec);
    } else {
      poly.push_front(vec);
    }

    tips.erase(it);

    if (!check_close(poly))
      tips[vec] = std::make_pair(key, end);
  }

  /*
   * Merge two polylines
   */
  void poly_merge(tips_iter_t& it, tips_iter_t& it_other)
  {
    bool prepend = false;
    bool reversed = IS_END(it_other);

    if (!IS_END(it)) {
      if (reversed) {
        std::swap(it, it_other);
        reversed = false;
      } else {
        prepend = true;
      }
    }

    auto key = it->second.first;
    auto key_other = it_other->second.first;
    auto& poly = polylines[key];

    if (key != key_other) {
      auto& poly_other = polylines[key_other];

      if (prepend) {
        for (auto& vec : poly_other)
          poly.push_front(vec);
      } else if (reversed) {
        poly.insert(poly.end(), poly_other.rbegin(), poly_other.rend());
      } else {
        poly.insert(poly.end(), poly_other.begin(), poly_other.end());
      }

      polylines.erase(key_other);
    } else {
      // close polyline
      poly.push_back(poly.front());
    }

    tips.erase(it);
    tips.erase(it_other);

    if (key != key_other && !check_close(poly)) {
      // overwrites existing key
      if (prepend) {
        tips[poly.front()] = std::make_pair(key, false);
      } else {
        tips[poly.back()] = std::make_pair(key, true);
      }
    }
  }

  /*
   * Create a new polyline from two points
   */
  void poly_new(const glm::vec3& vec0, const glm::vec3& vec1)
  {
    auto& poly = polylines[next_key];
    poly.push_back(vec0);
    poly.push_back(vec1);

    tips[vec0] = std::make_pair(next_key, false);
    tips[vec1] = std::make_pair(next_key, true);

    ++next_key;
  }

  /*
   * In principle, the `tips` map should be empty after all segments got
   * inserted. But since the keys have float precision and mismatches
   * are likely, we expect some splitted polylines that yet have to be
   * merged. Call this functions after all points have been added and
   * before using the polylines for anything else.
   */
  void finish()
  {
    const float eps = 1e-5;

#ifdef xPOLYLINEJIGSAW_DEBUG
    auto s_before = tips.size();
    std::cout << "finish remaining: " << tips.size() << std::endl;
#endif

  repeat:
    if (tips.empty())
      return;

    // merging polygons will invalidate the iterators,
    // so need to break and repeat from the beginning.

    for (auto it0 = tips.begin(); it0 != tips.end(); ++it0) {
      const auto& vec0 = it0->first;
      for (auto it1 = it0; (++it1) != tips.end();) {
        const auto& vec1 = it1->first;

        // tips sorted by x (before y/z), so look ahead along x for close points
        if (!almostequal(vec0.x, vec1.x, eps))
          break;

        if (almostequal(vec0.y, vec1.y, eps) &&
            almostequal(vec0.z, vec1.z, eps)) {

#ifdef xPOLYLINEJIGSAW_DEBUG
          if (!almostequal(vec0, vec1)) {
            std::cout << "equal with 20 but not 5: " //
                      << glm::to_string(vec0) << " " //
                      << glm::to_string(vec1) << " " //
                      << glm::to_string(vec1 - vec0) << std::endl;
          }
#endif

          poly_merge(it0, it1);
          goto repeat;
        }
      }
    }

#ifdef xPOLYLINEJIGSAW_DEBUG
    if (!tips.empty()) {
      std::cout << "finish remaining: " << tips.size() << " (" << s_before
                << ")" << std::endl;
      for (auto it0 = tips.begin(); it0 != tips.end(); ++it0) {
        const auto& vec0 = it0->first;
        for (auto it1 = it0; (++it1) != tips.end();) {
          const auto& vec1 = it1->first;

          auto eps3 = glm::abs(vec0) * eps;
          auto absdiff = glm::abs(vec0 - vec1);

          std::cout << glm::to_string(vec0)
                    << " dist: " << glm::distance(vec0, vec1) << std::endl
                    << glm::to_string(vec1) //
                    << " e: " << glm::to_string(eps3)
                    << " d: " << glm::to_string(absdiff) << std::endl;
        }
      }
    }
#endif
  }

public:
  /*
   * Get the polylines (call after all segments have been added)
   */
  const polylines_t& getPolylines()
  {
    finish();
    return polylines;
  }

  /*
   * Add a segment (consisting of two points)
   */
  void add(const glm::vec3& vec0, const glm::vec3& vec1)
  {
    auto it_tips0 = tips_find(vec0);
    auto it_tips1 = tips_find(vec1);

    bool found_tip0 = (it_tips0 != tips.end());
    bool found_tip1 = (it_tips1 != tips.end());

    if (found_tip0) {
      if (it_tips0 == it_tips1)
        return;

      if (found_tip1) {
        poly_merge(it_tips0, it_tips1);
      } else {
        poly_grow(it_tips0, vec1);
      }
    } else if (found_tip1) {
      poly_grow(it_tips1, vec0);
    } else {
      poly_new(vec0, vec1);
    }
  }
};

/*
 * Given a set of triangles which describe a surface, and a Z clipping
 * value, generate the intersecting CGO.
 */
std::unique_ptr<CGO> SurfaceZIntersection(PyMOLGlobals* G,
    const float* vertices,    // vertices
    const int* triindices,    // triangle indices
    int n_triangles,          // number of triangles
    float clip_z,             // z-axis clipping value
    const float* clip_normal) // orientation of the clipping plane
{
  const int* t = triindices;
  const float* v = vertices;
  const float* matrix = SceneGetModelViewMatrix(G);
  float transformed[3][3];
  glm::vec3 intersect[2];
  glm::mat4 gmatrix;
  PolylineJigsaw jigsaw;

  // support rotated clipping plane
  if (clip_normal) {
    auto gnorm = glm::normalize(glm::make_vec3(clip_normal));
    auto gup = glm::vec3(0.f, 0.f, 1.f);

    if (glm::dot(gnorm, gup) < 0.99f) {
      gmatrix = glm::orientation(gnorm, gup) *
                glm::translate(glm::vec3(0.f, 0.f, -clip_z)) *
                glm::make_mat4(matrix);

      clip_z = 0;
      matrix = glm::value_ptr(gmatrix);
    }
  }

  for (int c = n_triangles; c; --c, t += 3) {
    for (int i = 0; i < 3; ++i) {
      MatrixTransformC44f3f(matrix, v + t[i] * 3, transformed[i]);
    }

    bool sign[3] = {
        (transformed[0][2] < clip_z),
        (transformed[1][2] < clip_z),
        (transformed[2][2] < clip_z),
    };

    if (sign[0] == sign[1] && sign[0] == sign[2])
      continue;

    glm::vec3 gv[3] = {
        glm::make_vec3(v + t[0] * 3),
        glm::make_vec3(v + t[1] * 3),
        glm::make_vec3(v + t[2] * 3),
    };

    // index of point which is alone on one side of the surface
    int i = (sign[0] == sign[1]) ? 2 : (sign[0] == sign[2]) ? 1 : 0;

    for (int k = 0; k < 2; ++k) {
      // index of one of the two points on the other side of the surface
      int j = (i + k + 1) % 3;

      // relative position between i and j
      // (p=0.0: plane cuts at i, p=1.0: plane cuts at j)
      float p = (clip_z - transformed[i][2]) /
                (transformed[j][2] - transformed[i][2]);

      // intersection points
      intersect[k] = glm::mix(gv[i], gv[j], p);
    }

    jigsaw.add(intersect[0], intersect[1]);
  }

  PolylinesTesselator tess;
  tess.setupCGO(G);
  tess.tesselate(jigsaw.getPolylines());
  return std::unique_ptr<CGO>(tess.releaseCGO());
}
