/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

/**
 * GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
 */
class _SimplexVertex {
  final Vector2 wA = new Vector2.zero(); // support point in shapeA
  final Vector2 wB = new Vector2.zero(); // support point in shapeB
  final Vector2 w = new Vector2.zero(); // wB - wA
  double a = 0.0; // barycentric coordinate for closest point
  int indexA = 0; // wA index
  int indexB = 0; // wB index

  void set(_SimplexVertex sv) {
    wA.set(sv.wA);
    wB.set(sv.wB);
    w.set(sv.w);
    a = sv.a;
    indexA = sv.indexA;
    indexB = sv.indexB;
  }
}

class SimplexCache {
  /** length or area */
  double metric = 0.0;
  int count = 0;
  /** vertices on shape A */
  final List<int> indexA = BufferUtils.allocClearIntList(3);
  /** vertices on shape B */
  final List<int> indexB = BufferUtils.allocClearIntList(3);

  SimplexCache() {
    indexA[0] = Settings.INTEGER_MAX_VALUE;
    indexA[1] = Settings.INTEGER_MAX_VALUE;
    indexA[2] = Settings.INTEGER_MAX_VALUE;
    indexB[0] = Settings.INTEGER_MAX_VALUE;
    indexB[1] = Settings.INTEGER_MAX_VALUE;
    indexB[2] = Settings.INTEGER_MAX_VALUE;
  }

  void set(SimplexCache sc) {
    BufferUtils.arraycopy(sc.indexA, 0, indexA, 0, indexA.length);
    BufferUtils.arraycopy(sc.indexB, 0, indexB, 0, indexB.length);
    metric = sc.metric;
    count = sc.count;
  }
}

class _Simplex {
  final _SimplexVertex m_v1 = new _SimplexVertex();
  final _SimplexVertex m_v2 = new _SimplexVertex();
  final _SimplexVertex m_v3 = new _SimplexVertex();
  final List<_SimplexVertex> vertices = new List<_SimplexVertex>(3);
  int m_count = 0;

  _Simplex() {
    vertices[0] = m_v1;
    vertices[1] = m_v2;
    vertices[2] = m_v3;
  }

  void readCache(SimplexCache cache, DistanceProxy proxyA, Transform transformA,
      DistanceProxy proxyB, Transform transformB) {
    assert(cache.count <= 3);

    // Copy data from cache.
    m_count = cache.count;

    for (int i = 0; i < m_count; ++i) {
      _SimplexVertex v = vertices[i];
      v.indexA = cache.indexA[i];
      v.indexB = cache.indexB[i];
      Vector2 wALocal = proxyA.getVertex(v.indexA);
      Vector2 wBLocal = proxyB.getVertex(v.indexB);
      Transform.mulToOutUnsafeVec2(transformA, wALocal, v.wA);
      Transform.mulToOutUnsafeVec2(transformB, wBLocal, v.wB);
      v.w.set(v.wB).sub(v.wA);
      v.a = 0.0;
    }

    // Compute the new simplex metric, if it is substantially different than
    // old metric then flush the simplex.
    if (m_count > 1) {
      double metric1 = cache.metric;
      double metric2 = getMetric();
      if (metric2 < 0.5 * metric1 ||
          2.0 * metric1 < metric2 ||
          metric2 < Settings.EPSILON) {
        // Reset the simplex.
        m_count = 0;
      }
    }

    // If the cache is empty or invalid ...
    if (m_count == 0) {
      _SimplexVertex v = vertices[0];
      v.indexA = 0;
      v.indexB = 0;
      Vector2 wALocal = proxyA.getVertex(0);
      Vector2 wBLocal = proxyB.getVertex(0);
      Transform.mulToOutUnsafeVec2(transformA, wALocal, v.wA);
      Transform.mulToOutUnsafeVec2(transformB, wBLocal, v.wB);
      v.w.set(v.wB).sub(v.wA);
      m_count = 1;
    }
  }

  void writeCache(SimplexCache cache) {
    cache.metric = getMetric();
    cache.count = m_count;

    for (int i = 0; i < m_count; ++i) {
      cache.indexA[i] = (vertices[i].indexA).toInt();
      cache.indexB[i] = (vertices[i].indexB).toInt();
    }
  }

  final Vector2 _e12 = new Vector2.zero();

  void getSearchDirection(final Vector2 out) {
    switch (m_count) {
      case 1:
        out.set(m_v1.w).negate();
        return;
      case 2:
        _e12.set(m_v2.w).sub(m_v1.w);
        // use out for a temp variable real quick
        out.set(m_v1.w).negate();
        double sgn = Vector2.cross(_e12, out);

        if (sgn > 0.0) {
          // Origin is left of e12.
          Vector2.crossToOutUnsafeDblVec2(1.0, _e12, out);
        } else {
          // Origin is right of e12.
          Vector2.crossToOutUnsafeVec2Dbl(_e12, 1.0, out);
        }
        return;
      default:
        assert(false);
        out.setZero();
        return;
    }
  }

  // djm pooled
  final Vector2 _case2 = new Vector2.zero();
  final Vector2 _case22 = new Vector2.zero();

  /**
   * this returns pooled objects. don't keep or modify them
   * 
   * @return
   */
  void getClosestPoint(final Vector2 out) {
    switch (m_count) {
      case 0:
        assert(false);
        out.setZero();
        return;
      case 1:
        out.set(m_v1.w);
        return;
      case 2:
        _case22.set(m_v2.w).mul(m_v2.a);
        _case2.set(m_v1.w).mul(m_v1.a).add(_case22);
        out.set(_case2);
        return;
      case 3:
        out.setZero();
        return;
      default:
        assert(false);
        out.setZero();
        return;
    }
  }

  // djm pooled, and from above
  final Vector2 _case3 = new Vector2.zero();
  final Vector2 _case33 = new Vector2.zero();

  void getWitnessPoints(Vector2 pA, Vector2 pB) {
    switch (m_count) {
      case 0:
        assert(false);
        break;

      case 1:
        pA.set(m_v1.wA);
        pB.set(m_v1.wB);
        break;

      case 2:
        _case2.set(m_v1.wA).mul(m_v1.a);
        pA.set(m_v2.wA).mul(m_v2.a).add(_case2);
        // m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
        // *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
        _case2.set(m_v1.wB).mul(m_v1.a);
        pB.set(m_v2.wB).mul(m_v2.a).add(_case2);

        break;

      case 3:
        pA.set(m_v1.wA).mul(m_v1.a);
        _case3.set(m_v2.wA).mul(m_v2.a);
        _case33.set(m_v3.wA).mul(m_v3.a);
        pA.add(_case3).add(_case33);
        pB.set(pA);
        // *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
        // *pB = *pA;
        break;

      default:
        assert(false);
        break;
    }
  }

  // djm pooled, from above
  double getMetric() {
    switch (m_count) {
      case 0:
        assert(false);
        return 0.0;

      case 1:
        return 0.0;

      case 2:
        return MathUtils.distance(m_v1.w, m_v2.w);

      case 3:
        _case3.set(m_v2.w).sub(m_v1.w);
        _case33.set(m_v3.w).sub(m_v1.w);
        // return Vec2.cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
        return Vector2.cross(_case3, _case33);

      default:
        assert(false);
        return 0.0;
    }
  }

  // djm pooled from above
  /**
   * Solve a line segment using barycentric coordinates.
   */
  void solve2() {
    // Solve a line segment using barycentric coordinates.
    //
    // p = a1 * w1 + a2 * w2
    // a1 + a2 = 1
    //
    // The vector from the origin to the closest point on the line is
    // perpendicular to the line.
    // e12 = w2 - w1
    // dot(p, e) = 0
    // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
    //
    // 2-by-2 linear system
    // [1 1 ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    //
    // Define
    // d12_1 = dot(w2, e12)
    // d12_2 = -dot(w1, e12)
    // d12 = d12_1 + d12_2
    //
    // Solution
    // a1 = d12_1 / d12
    // a2 = d12_2 / d12
    final Vector2 w1 = m_v1.w;
    final Vector2 w2 = m_v2.w;
    _e12.set(w2).sub(w1);

    // w1 region
    double d12_2 = -Vector2.dot(w1, _e12);
    if (d12_2 <= 0.0) {
      // a2 <= 0, so we clamp it to 0
      m_v1.a = 1.0;
      m_count = 1;
      return;
    }

    // w2 region
    double d12_1 = Vector2.dot(w2, _e12);
    if (d12_1 <= 0.0) {
      // a1 <= 0, so we clamp it to 0
      m_v2.a = 1.0;
      m_count = 1;
      m_v1.set(m_v2);
      return;
    }

    // Must be in e12 region.
    double inv_d12 = 1.0 / (d12_1 + d12_2);
    m_v1.a = d12_1 * inv_d12;
    m_v2.a = d12_2 * inv_d12;
    m_count = 2;
  }

  // djm pooled, and from above
  final Vector2 _e13 = new Vector2.zero();
  final Vector2 _e23 = new Vector2.zero();
  final Vector2 _w1 = new Vector2.zero();
  final Vector2 _w2 = new Vector2.zero();
  final Vector2 _w3 = new Vector2.zero();

  /**
   * Solve a line segment using barycentric coordinates.<br/>
   * Possible regions:<br/>
   * - points[2]<br/>
   * - edge points[0]-points[2]<br/>
   * - edge points[1]-points[2]<br/>
   * - inside the triangle
   */
  void solve3() {
    _w1.set(m_v1.w);
    _w2.set(m_v2.w);
    _w3.set(m_v3.w);

    // Edge12
    // [1 1 ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    _e12.set(_w2).sub(_w1);
    double w1e12 = Vector2.dot(_w1, _e12);
    double w2e12 = Vector2.dot(_w2, _e12);
    double d12_1 = w2e12;
    double d12_2 = -w1e12;

    // Edge13
    // [1 1 ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    _e13.set(_w3).sub(_w1);
    double w1e13 = Vector2.dot(_w1, _e13);
    double w3e13 = Vector2.dot(_w3, _e13);
    double d13_1 = w3e13;
    double d13_2 = -w1e13;

    // Edge23
    // [1 1 ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    _e23.set(_w3).sub(_w2);
    double w2e23 = Vector2.dot(_w2, _e23);
    double w3e23 = Vector2.dot(_w3, _e23);
    double d23_1 = w3e23;
    double d23_2 = -w2e23;

    // Triangle123
    double n123 = Vector2.cross(_e12, _e13);

    double d123_1 = n123 * Vector2.cross(_w2, _w3);
    double d123_2 = n123 * Vector2.cross(_w3, _w1);
    double d123_3 = n123 * Vector2.cross(_w1, _w2);

    // w1 region
    if (d12_2 <= 0.0 && d13_2 <= 0.0) {
      m_v1.a = 1.0;
      m_count = 1;
      return;
    }

    // e12
    if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
      double inv_d12 = 1.0 / (d12_1 + d12_2);
      m_v1.a = d12_1 * inv_d12;
      m_v2.a = d12_2 * inv_d12;
      m_count = 2;
      return;
    }

    // e13
    if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
      double inv_d13 = 1.0 / (d13_1 + d13_2);
      m_v1.a = d13_1 * inv_d13;
      m_v3.a = d13_2 * inv_d13;
      m_count = 2;
      m_v2.set(m_v3);
      return;
    }

    // w2 region
    if (d12_1 <= 0.0 && d23_2 <= 0.0) {
      m_v2.a = 1.0;
      m_count = 1;
      m_v1.set(m_v2);
      return;
    }

    // w3 region
    if (d13_1 <= 0.0 && d23_1 <= 0.0) {
      m_v3.a = 1.0;
      m_count = 1;
      m_v1.set(m_v3);
      return;
    }

    // e23
    if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
      double inv_d23 = 1.0 / (d23_1 + d23_2);
      m_v2.a = d23_1 * inv_d23;
      m_v3.a = d23_2 * inv_d23;
      m_count = 2;
      m_v1.set(m_v3);
      return;
    }

    // Must be in triangle123
    double inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
    m_v1.a = d123_1 * inv_d123;
    m_v2.a = d123_2 * inv_d123;
    m_v3.a = d123_3 * inv_d123;
    m_count = 3;
  }
} // Class _Simplex

class DistanceProxy {
  final List<Vector2> m_vertices;
  int m_count;
  double m_radius;
  final List<Vector2> m_buffer;

  DistanceProxy()
      : m_vertices = new List<Vector2>(Settings.maxPolygonVertices),
        m_buffer = new List<Vector2>(2) {
    for (int i = 0; i < m_vertices.length; i++) {
      m_vertices[i] = new Vector2.zero();
    }
    m_count = 0;
    m_radius = 0.0;
  }

  /**
   * Initialize the proxy using the given shape. The shape must remain in scope while the proxy is
   * in use.
   */
  void set(final Shape shape, int index) {
    switch (shape.getType()) {
      case ShapeType.CIRCLE:
        final CircleShape circle = shape;
        m_vertices[0].set(circle.m_p);
        m_count = 1;
        m_radius = circle.radius;

        break;
      case ShapeType.POLYGON:
        final PolygonShape poly = shape;
        m_count = poly.m_count;
        m_radius = poly.radius;
        for (int i = 0; i < m_count; i++) {
          m_vertices[i].set(poly.m_vertices[i]);
        }
        break;
      case ShapeType.CHAIN:
        final ChainShape chain = shape;
        assert(0 <= index && index < chain.m_count);

        m_buffer[0] = chain.m_vertices[index];
        if (index + 1 < chain.m_count) {
          m_buffer[1] = chain.m_vertices[index + 1];
        } else {
          m_buffer[1] = chain.m_vertices[0];
        }

        m_vertices[0].set(m_buffer[0]);
        m_vertices[1].set(m_buffer[1]);
        m_count = 2;
        m_radius = chain.radius;
        break;
      case ShapeType.EDGE:
        EdgeShape edge = shape;
        m_vertices[0].set(edge.m_vertex1);
        m_vertices[1].set(edge.m_vertex2);
        m_count = 2;
        m_radius = edge.radius;
        break;
      default:
        assert(false);
    }
  }

  /**
   * Get the supporting vertex index in the given direction.
   * 
   * @param d
   * @return
   */
  int getSupport(final Vector2 d) {
    int bestIndex = 0;
    double bestValue = Vector2.dot(m_vertices[0], d);
    for (int i = 1; i < m_count; i++) {
      double value = Vector2.dot(m_vertices[i], d);
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
    }

    return bestIndex;
  }

  /**
   * Get the supporting vertex in the given direction.
   * 
   * @param d
   * @return
   */
  Vector2 getSupportVertex(final Vector2 d) {
    int bestIndex = 0;
    double bestValue = Vector2.dot(m_vertices[0], d);
    for (int i = 1; i < m_count; i++) {
      double value = Vector2.dot(m_vertices[i], d);
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
    }

    return m_vertices[bestIndex];
  }

  /**
   * Get the vertex count.
   * 
   * @return
   */
  int getVertexCount() {
    return m_count;
  }

  /**
   * Get a vertex by index. Used by Distance.
   * 
   * @param index
   * @return
   */
  Vector2 getVertex(int index) {
    assert(0 <= index && index < m_count);
    return m_vertices[index];
  }
} // Class _DistanceProxy.

class Distance {
  static const int MAX_ITERS = 20;

  static int GJK_CALLS = 0;
  static int GJK_ITERS = 0;
  static int GJK_MAX_ITERS = 20;

  _Simplex _simplex = new _Simplex();
  List<int> _saveA = BufferUtils.allocClearIntList(3);
  List<int> _saveB = BufferUtils.allocClearIntList(3);
  Vector2 _closestPoint = new Vector2.zero();
  Vector2 _d = new Vector2.zero();
  Vector2 _temp = new Vector2.zero();
  Vector2 _normal = new Vector2.zero();

  /**
   * Compute the closest points between two shapes. Supports any combination of: CircleShape and
   * PolygonShape. The simplex cache is input/output. On the first call set SimplexCache.count to
   * zero.
   * 
   * @param output
   * @param cache
   * @param input
   */
  void distance(final DistanceOutput output, final SimplexCache cache,
      final DistanceInput input) {
    GJK_CALLS++;

    final DistanceProxy proxyA = input.proxyA;
    final DistanceProxy proxyB = input.proxyB;

    Transform transformA = input.transformA;
    Transform transformB = input.transformB;

    // Initialize the simplex.
    _simplex.readCache(cache, proxyA, transformA, proxyB, transformB);

    // Get simplex vertices as an array.
    List<_SimplexVertex> vertices = _simplex.vertices;

    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    // (pooled above)
    int saveCount = 0;

    _simplex.getClosestPoint(_closestPoint);
    double distanceSqr1 = _closestPoint.lengthSquared();
    double distanceSqr2 = distanceSqr1;

    // Main iteration loop
    int iter = 0;
    while (iter < MAX_ITERS) {

      // Copy simplex so we can identify duplicates.
      saveCount = _simplex.m_count;
      for (int i = 0; i < saveCount; i++) {
        _saveA[i] = vertices[i].indexA;
        _saveB[i] = vertices[i].indexB;
      }

      switch (_simplex.m_count) {
        case 1:
          break;
        case 2:
          _simplex.solve2();
          break;
        case 3:
          _simplex.solve3();
          break;
        default:
          assert(false);
      }

      // If we have 3 points, then the origin is in the corresponding triangle.
      if (_simplex.m_count == 3) {
        break;
      }

      // Compute closest point.
      _simplex.getClosestPoint(_closestPoint);
      distanceSqr2 = _closestPoint.lengthSquared();

      // ensure progress
      if (distanceSqr2 >= distanceSqr1) {
        // break;
      }
      distanceSqr1 = distanceSqr2;

      // get search direction;
      _simplex.getSearchDirection(_d);

      // Ensure the search direction is numerically fit.
      if (_d.lengthSquared() < Settings.EPSILON * Settings.EPSILON) {
        // The origin is probably contained by a line segment
        // or triangle. Thus the shapes are overlapped.

        // We can't return zero here even though there may be overlap.
        // In case the simplex is a point, segment, or triangle it is difficult
        // to determine if the origin is contained in the CSO or very close to it.
        break;
      }
      /*
       * SimplexVertex* vertex = vertices + simplex.m_count; vertex.indexA =
       * proxyA.GetSupport(MulT(transformA.R, -d)); vertex.wA = Mul(transformA,
       * proxyA.GetVertex(vertex.indexA)); Vec2 wBLocal; vertex.indexB =
       * proxyB.GetSupport(MulT(transformB.R, d)); vertex.wB = Mul(transformB,
       * proxyB.GetVertex(vertex.indexB)); vertex.w = vertex.wB - vertex.wA;
       */

      // Compute a tentative new simplex vertex using support points.
      _SimplexVertex vertex = vertices[_simplex.m_count];

      Rot.mulTransUnsafeVec2(transformA.q, _d.negate(), _temp);
      vertex.indexA = proxyA.getSupport(_temp);
      Transform.mulToOutUnsafeVec2(
          transformA, proxyA.getVertex(vertex.indexA), vertex.wA);
      // Vec2 wBLocal;
      Rot.mulTransUnsafeVec2(transformB.q, _d.negate(), _temp);
      vertex.indexB = proxyB.getSupport(_temp);
      Transform.mulToOutUnsafeVec2(
          transformB, proxyB.getVertex(vertex.indexB), vertex.wB);
      vertex.w.set(vertex.wB).sub(vertex.wA);

      // Iteration count is equated to the number of support point calls.
      ++iter;
      ++GJK_ITERS;

      // Check for duplicate support points. This is the main termination criteria.
      bool duplicate = false;
      for (int i = 0; i < saveCount; ++i) {
        if (vertex.indexA == _saveA[i] && vertex.indexB == _saveB[i]) {
          duplicate = true;
          break;
        }
      }

      // If we found a duplicate support point we must exit to avoid cycling.
      if (duplicate) {
        break;
      }

      // New vertex is ok and needed.
      ++_simplex.m_count;
    }

    GJK_MAX_ITERS = Math.max(GJK_MAX_ITERS, iter);

    // Prepare output.
    _simplex.getWitnessPoints(output.pointA, output.pointB);
    output.distance = MathUtils.distance(output.pointA, output.pointB);
    output.iterations = iter;

    // Cache the simplex.
    _simplex.writeCache(cache);

    // Apply radii if requested.
    if (input.useRadii) {
      double rA = proxyA.m_radius;
      double rB = proxyB.m_radius;

      if (output.distance > rA + rB && output.distance > Settings.EPSILON) {
        // Shapes are still no overlapped.
        // Move the witness points to the outer surface.
        output.distance -= rA + rB;
        _normal.set(output.pointB).sub(output.pointA);
        _normal.normalize();
        _temp.set(_normal).mul(rA);
        output.pointA.add(_temp);
        _temp.set(_normal).mul(rB);
        output.pointB.sub(_temp);
      } else {
        // Shapes are overlapped when radii are considered.
        // Move the witness points to the middle.
        // Vec2 p = 0.5f * (output.pointA + output.pointB);
        output.pointA.add(output.pointB).mul(.5);
        output.pointB.set(output.pointA);
        output.distance = 0.0;
      }
    }
  }
}
