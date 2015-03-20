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
 * A convex polygon shape. Polygons have a maximum number of vertices equal to _maxPolygonVertices.
 * In most cases you should not need many vertices for a convex polygon.
 */
class PolygonShape extends Shape {
  /** Dump lots of debug information. */
  static const bool _m_debug = false;

  /**
   * Local position of the shape centroid in parent body frame.
   */
  final Vector2 m_centroid = new Vector2.zero();

  /**
   * The vertices of the shape. Note: use getVertexCount(), not m_vertices.length, to get number of
   * active vertices.
   */
  final List<Vector2> m_vertices = new List<Vector2>(Settings.maxPolygonVertices);

  /**
   * The normals of the shape. Note: use getVertexCount(), not m_normals.length, to get number of
   * active normals.
   */
  final List<Vector2> m_normals = new List<Vector2>(Settings.maxPolygonVertices);

  /**
   * Number of active vertices in the shape.
   */
  int m_count = 0;

  // pooling
  final Vector2 _pool1 = new Vector2.zero();
  final Vector2 _pool2 = new Vector2.zero();
  final Vector2 _pool3 = new Vector2.zero();
  final Vector2 _pool4 = new Vector2.zero();
  Transform _poolt1 = new Transform.zero();

  PolygonShape() : super(ShapeType.POLYGON) {
    for (int i = 0; i < m_vertices.length; i++) {
      m_vertices[i] = new Vector2.zero();
    }
    ;
    for (int i = 0; i < m_normals.length; i++) {
      m_normals[i] = new Vector2.zero();
    }
    setRadius(Settings.polygonRadius);
  }

  Shape clone() {
    PolygonShape shape = new PolygonShape();
    shape.m_centroid.set(this.m_centroid);
    for (int i = 0; i < shape.m_normals.length; i++) {
      shape.m_normals[i].set(m_normals[i]);
      shape.m_vertices[i].set(m_vertices[i]);
    }
    shape.setRadius(this.getRadius());
    shape.m_count = this.m_count;
    return shape;
  }

  /**
   * Create a convex hull from the given array of points. The count must be in the range [3,
   * Settings.maxPolygonVertices].
   *
   * @warning the points may be re-ordered, even if they form a convex polygon.
   * @warning collinear points are removed.
   */
  void set(final List<Vector2> vertices, final int count) {
    setWithPools(vertices, count, null, null);
  }

  /**
   * Create a convex hull from the given array of points. The count must be in the range [3,
   * Settings.maxPolygonVertices]. This method takes an arraypool for pooling.
   *
   * @warning the points may be re-ordered, even if they form a convex polygon.
   * @warning collinear points are removed.
   */
  void setWithPools(final List<Vector2> verts, final int num,
      final Vec2Array vecPool, final IntArray intPool) {
    assert(3 <= num && num <= Settings.maxPolygonVertices);
    if (num < 3) {
      setAsBoxXY(1.0, 1.0);
      return;
    }

    int n = Math.min(num, Settings.maxPolygonVertices);

    // Perform welding and copy vertices into local buffer.
    List<Vector2> ps = (vecPool != null)
        ? vecPool.get(Settings.maxPolygonVertices)
        : new List<Vector2>(Settings.maxPolygonVertices);
    int tempCount = 0;
    for (int i = 0; i < n; ++i) {
      Vector2 v = verts[i];
      bool unique = true;
      for (int j = 0; j < tempCount; ++j) {
        if (MathUtils.distanceSquared(v, ps[j]) < 0.5 * Settings.linearSlop) {
          unique = false;
          break;
        }
      }

      if (unique) {
        ps[tempCount++] = v;
      }
    }

    n = tempCount;
    if (n < 3) {
      // Polygon is degenerate.
      assert(false);
      setAsBoxXY(1.0, 1.0);
      return;
    }

    // Create the convex hull using the Gift wrapping algorithm
    // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

    // Find the right most point on the hull
    int i0 = 0;
    double x0 = ps[0].x;
    for (int i = 1; i < n; ++i) {
      double x = ps[i].x;
      if (x > x0 || (x == x0 && ps[i].y < ps[i0].y)) {
        i0 = i;
        x0 = x;
      }
    }

    List<int> hull = (intPool != null)
        ? intPool.get(Settings.maxPolygonVertices)
        : BufferUtils.allocClearIntList(Settings.maxPolygonVertices);
    int m = 0;
    int ih = i0;

    while (true) {
      hull[m] = ih;

      int ie = 0;
      for (int j = 1; j < n; ++j) {
        if (ie == ih) {
          ie = j;
          continue;
        }

        Vector2 r = _pool1.set(ps[ie]).sub(ps[hull[m]]);
        Vector2 v = _pool2.set(ps[j]).sub(ps[hull[m]]);
        double c = Vector2.cross(r, v);
        if (c < 0.0) {
          ie = j;
        }

        // Collinearity check
        if (c == 0.0 && v.lengthSquared() > r.lengthSquared()) {
          ie = j;
        }
      }

      ++m;
      ih = ie;

      if (ie == i0) {
        break;
      }
    }

    this.m_count = m;

    // Copy vertices.
    for (int i = 0; i < m_count; ++i) {
      if (m_vertices[i] == null) {
        m_vertices[i] = new Vector2.zero();
      }
      m_vertices[i].set(ps[hull[i]]);
    }

    final Vector2 edge = _pool1;

    // Compute normals. Ensure the edges have non-zero length.
    for (int i = 0; i < m_count; ++i) {
      final int i1 = i;
      final int i2 = i + 1 < m_count ? i + 1 : 0;
      edge.set(m_vertices[i2]).sub(m_vertices[i1]);

      assert(edge.lengthSquared() > Settings.EPSILON * Settings.EPSILON);
      Vector2.crossToOutUnsafeVec2Dbl(edge, 1.0, m_normals[i]);
      m_normals[i].normalize();
    }

    // Compute the polygon centroid.
    computeCentroidToOut(m_vertices, m_count, m_centroid);
  }

  /**
   * Build vertices to represent an axis-aligned box.
   *
   * @param hx the half-width.
   * @param hy the half-height.
   */
  void setAsBoxXY(final double hx, final double hy) {
    m_count = 4;
    m_vertices[0].setValues(-hx, -hy);
    m_vertices[1].setValues(hx, -hy);
    m_vertices[2].setValues(hx, hy);
    m_vertices[3].setValues(-hx, hy);
    m_normals[0].setValues(0.0, -1.0);
    m_normals[1].setValues(1.0, 0.0);
    m_normals[2].setValues(0.0, 1.0);
    m_normals[3].setValues(-1.0, 0.0);
    m_centroid.setZero();
  }

  /**
   * Build vertices to represent an oriented box.
   *
   * @param hx the half-width.
   * @param hy the half-height.
   * @param center the center of the box in local coordinates.
   * @param angle the rotation of the box in local coordinates.
   */
  void setAsBox(
      final double hx, final double hy, final Vector2 center, final double angle) {
    m_count = 4;
    m_vertices[0].setValues(-hx, -hy);
    m_vertices[1].setValues(hx, -hy);
    m_vertices[2].setValues(hx, hy);
    m_vertices[3].setValues(-hx, hy);
    m_normals[0].setValues(0.0, -1.0);
    m_normals[1].setValues(1.0, 0.0);
    m_normals[2].setValues(0.0, 1.0);
    m_normals[3].setValues(-1.0, 0.0);
    m_centroid.set(center);

    final Transform xf = _poolt1;
    xf.p.set(center);
    xf.q.setAngle(angle);

    // Transform vertices and normals.
    for (int i = 0; i < m_count; ++i) {
      Transform.mulToOutVec2(xf, m_vertices[i], m_vertices[i]);
      Rot.mulToOut(xf.q, m_normals[i], m_normals[i]);
    }
  }

  int getChildCount() {
    return 1;
  }

  bool testPoint(final Transform xf, final Vector2 p) {
    double tempx, tempy;
    final Rot xfq = xf.q;

    tempx = p.x - xf.p.x;
    tempy = p.y - xf.p.y;
    final double pLocalx = xfq.c * tempx + xfq.s * tempy;
    final double pLocaly = -xfq.s * tempx + xfq.c * tempy;

    if (_m_debug) {
      print("--testPoint debug--");
      print("Vertices: ");
      for (int i = 0; i < m_count; ++i) {
        print(m_vertices[i]);
      }
      print("pLocal: $pLocalx, $pLocaly");
    }

    for (int i = 0; i < m_count; ++i) {
      Vector2 vertex = m_vertices[i];
      Vector2 normal = m_normals[i];
      tempx = pLocalx - vertex.x;
      tempy = pLocaly - vertex.y;
      final double dot = normal.x * tempx + normal.y * tempy;
      if (dot > 0.0) {
        return false;
      }
    }

    return true;
  }

  void computeAABB(final AABB aabb, final Transform xf, int childIndex) {
    final Vector2 lower = aabb.lowerBound;
    final Vector2 upper = aabb.upperBound;
    final Vector2 v1 = m_vertices[0];
    final double xfqc = xf.q.c;
    final double xfqs = xf.q.s;
    final double xfpx = xf.p.x;
    final double xfpy = xf.p.y;
    lower.x = (xfqc * v1.x - xfqs * v1.y) + xfpx;
    lower.y = (xfqs * v1.x + xfqc * v1.y) + xfpy;
    upper.x = lower.x;
    upper.y = lower.y;

    for (int i = 1; i < m_count; ++i) {
      Vector2 v2 = m_vertices[i];
      // Vec2 v = Mul(xf, m_vertices[i]);
      double vx = (xfqc * v2.x - xfqs * v2.y) + xfpx;
      double vy = (xfqs * v2.x + xfqc * v2.y) + xfpy;
      lower.x = lower.x < vx ? lower.x : vx;
      lower.y = lower.y < vy ? lower.y : vy;
      upper.x = upper.x > vx ? upper.x : vx;
      upper.y = upper.y > vy ? upper.y : vy;
    }

    lower.x -= radius;
    lower.y -= radius;
    upper.x += radius;
    upper.y += radius;
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
   * Get a vertex by index.
   *
   * @param index
   * @return
   */
  Vector2 getVertex(final int index) {
    assert(0 <= index && index < m_count);
    return m_vertices[index];
  }

  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    double xfqc = xf.q.c;
    double xfqs = xf.q.s;
    double tx = p.x - xf.p.x;
    double ty = p.y - xf.p.y;
    double pLocalx = xfqc * tx + xfqs * ty;
    double pLocaly = -xfqs * tx + xfqc * ty;

    double maxDistance = -double.MAX_FINITE;
    double normalForMaxDistanceX = pLocalx;
    double normalForMaxDistanceY = pLocaly;

    for (int i = 0; i < m_count; ++i) {
      Vector2 vertex = m_vertices[i];
      Vector2 normal = m_normals[i];
      tx = pLocalx - vertex.x;
      ty = pLocaly - vertex.y;
      double dot = normal.x * tx + normal.y * ty;
      if (dot > maxDistance) {
        maxDistance = dot;
        normalForMaxDistanceX = normal.x;
        normalForMaxDistanceY = normal.y;
      }
    }

    double distance;
    if (maxDistance > 0) {
      double minDistanceX = normalForMaxDistanceX;
      double minDistanceY = normalForMaxDistanceY;
      double minDistance2 = maxDistance * maxDistance;
      for (int i = 0; i < m_count; ++i) {
        Vector2 vertex = m_vertices[i];
        double distanceVecX = pLocalx - vertex.x;
        double distanceVecY = pLocaly - vertex.y;
        double distance2 =
            (distanceVecX * distanceVecX + distanceVecY * distanceVecY);
        if (minDistance2 > distance2) {
          minDistanceX = distanceVecX;
          minDistanceY = distanceVecY;
          minDistance2 = distance2;
        }
      }
      distance = Math.sqrt(minDistance2);
      normalOut.x = xfqc * minDistanceX - xfqs * minDistanceY;
      normalOut.y = xfqs * minDistanceX + xfqc * minDistanceY;
      normalOut.normalize();
    } else {
      distance = maxDistance;
      normalOut.x = xfqc * normalForMaxDistanceX - xfqs * normalForMaxDistanceY;
      normalOut.y = xfqs * normalForMaxDistanceX + xfqc * normalForMaxDistanceY;
    }

    return distance;
  }

  bool raycast(
      RayCastOutput output, RayCastInput input, Transform xf, int childIndex) {
    final double xfqc = xf.q.c;
    final double xfqs = xf.q.s;
    final Vector2 xfp = xf.p;
    double tempx, tempy;
    // b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
    // b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
    tempx = input.p1.x - xfp.x;
    tempy = input.p1.y - xfp.y;
    final double p1x = xfqc * tempx + xfqs * tempy;
    final double p1y = -xfqs * tempx + xfqc * tempy;

    tempx = input.p2.x - xfp.x;
    tempy = input.p2.y - xfp.y;
    final double p2x = xfqc * tempx + xfqs * tempy;
    final double p2y = -xfqs * tempx + xfqc * tempy;

    final double dx = p2x - p1x;
    final double dy = p2y - p1y;

    double lower = 0.0,
        upper = input.maxFraction;

    int index = -1;

    for (int i = 0; i < m_count; ++i) {
      Vector2 normal = m_normals[i];
      Vector2 vertex = m_vertices[i];
      // p = p1 + a * d
      // dot(normal, p - v) = 0
      // dot(normal, p1 - v) + a * dot(normal, d) = 0
      double tempxn = vertex.x - p1x;
      double tempyn = vertex.y - p1y;
      final double numerator = normal.x * tempxn + normal.y * tempyn;
      final double denominator = normal.x * dx + normal.y * dy;

      if (denominator == 0.0) {
        if (numerator < 0.0) {
          return false;
        }
      } else {
        // Note: we want this predicate without division:
        // lower < numerator / denominator, where denominator < 0
        // Since denominator < 0, we have to flip the inequality:
        // lower < numerator / denominator <==> denominator * lower >
        // numerator.
        if (denominator < 0.0 && numerator < lower * denominator) {
          // Increase lower.
          // The segment enters this half-space.
          lower = numerator / denominator;
          index = i;
        } else if (denominator > 0.0 && numerator < upper * denominator) {
          // Decrease upper.
          // The segment exits this half-space.
          upper = numerator / denominator;
        }
      }

      if (upper < lower) {
        return false;
      }
    }

    assert(0.0 <= lower && lower <= input.maxFraction);

    if (index >= 0) {
      output.fraction = lower;
      // normal = Mul(xf.R, m_normals[index]);
      Vector2 normal = m_normals[index];
      Vector2 out = output.normal;
      out.x = xfqc * normal.x - xfqs * normal.y;
      out.y = xfqs * normal.x + xfqc * normal.y;
      return true;
    }
    return false;
  }

  void computeCentroidToOut(
      final List<Vector2> vs, final int count, final Vector2 out) {
    assert(count >= 3);

    out.setValues(0.0, 0.0);
    double area = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    final Vector2 pRef = _pool1;
    pRef.setZero();

    final Vector2 e1 = _pool2;
    final Vector2 e2 = _pool3;

    final double inv3 = 1.0 / 3.0;

    for (int i = 0; i < count; ++i) {
      // Triangle vertices.
      final Vector2 p1 = pRef;
      final Vector2 p2 = vs[i];
      final Vector2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

      e1.set(p2).sub(p1);
      e2.set(p3).sub(p1);

      final double D = Vector2.cross(e1, e2);

      final double triangleArea = 0.5 * D;
      area += triangleArea;

      // Area weighted centroid
      e1.set(p1).add(p2).add(p3).mul(triangleArea * inv3);
      out.add(e1);
    }

    // Centroid
    assert(area > Settings.EPSILON);
    out.mul(1.0 / area);
  }

  void computeMass(final MassData massData, double density) {
    // Polygon mass, centroid, and inertia.
    // Let rho be the polygon density in mass per unit area.
    // Then:
    // mass = rho * int(dA)
    // centroid.x = (1/mass) * rho * int(x * dA)
    // centroid.y = (1/mass) * rho * int(y * dA)
    // I = rho * int((x*x + y*y) * dA)
    //
    // We can compute these integrals by summing all the integrals
    // for each triangle of the polygon. To evaluate the integral
    // for a single triangle, we make a change of variables to
    // the (u,v) coordinates of the triangle:
    // x = x0 + e1x * u + e2x * v
    // y = y0 + e1y * u + e2y * v
    // where 0 <= u && 0 <= v && u + v <= 1.
    //
    // We integrate u from [0,1-v] and then v from [0,1].
    // We also need to use the Jacobian of the transformation:
    // D = cross(e1, e2)
    //
    // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
    //
    // The rest of the derivation is handled by computer algebra.

    assert(m_count >= 3);

    final Vector2 center = _pool1;
    center.setZero();
    double area = 0.0;
    double I = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    final Vector2 s = _pool2;
    s.setZero();
    // This code would put the reference point inside the polygon.
    for (int i = 0; i < m_count; ++i) {
      s.add(m_vertices[i]);
    }
    s.mul(1.0 / m_count);

    final double k_inv3 = 1.0 / 3.0;

    final Vector2 e1 = _pool3;
    final Vector2 e2 = _pool4;

    for (int i = 0; i < m_count; ++i) {
      // Triangle vertices.
      e1.set(m_vertices[i]).sub(s);
      e2
          .set(s)
          .negate()
          .add(i + 1 < m_count ? m_vertices[i + 1] : m_vertices[0]);

      final double D = Vector2.cross(e1, e2);

      final double triangleArea = 0.5 * D;
      area += triangleArea;

      // Area weighted centroid
      center.x += triangleArea * k_inv3 * (e1.x + e2.x);
      center.y += triangleArea * k_inv3 * (e1.y + e2.y);

      final double ex1 = e1.x,
          ey1 = e1.y;
      final double ex2 = e2.x,
          ey2 = e2.y;

      double intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
      double inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

      I += (0.25 * k_inv3 * D) * (intx2 + inty2);
    }

    // Total mass
    massData.mass = density * area;

    // Center of mass
    assert(area > Settings.EPSILON);
    center.mul(1.0 / area);
    massData.center.set(center).add(s);

    // Inertia tensor relative to the local origin (point s)
    massData.I = I * density;

    // Shift to center of mass then to original body origin.
    massData.I += massData.mass * (Vector2.dot(massData.center, massData.center));
  }

  /**
   * Validate convexity. This is a very time consuming operation.
   *
   * @return
   */
  bool validate() {
    for (int i = 0; i < m_count; ++i) {
      int i1 = i;
      int i2 = i < m_count - 1 ? i1 + 1 : 0;
      Vector2 p = m_vertices[i1];
      Vector2 e = _pool1.set(m_vertices[i2]).sub(p);

      for (int j = 0; j < m_count; ++j) {
        if (j == i1 || j == i2) {
          continue;
        }

        Vector2 v = _pool2.set(m_vertices[j]).sub(p);
        double c = Vector2.cross(e, v);
        if (c < 0.0) {
          return false;
        }
      }
    }

    return true;
  }

  /** Get the vertices in local coordinates. */
  List<Vector2> getVertices() {
    return m_vertices;
  }

  /** Get the edge normal vectors. There is one for each vertex. */
  List<Vector2> getNormals() {
    return m_normals;
  }

  /** Get the centroid and apply the supplied transform. */
  Vector2 centroid(final Transform xf) {
    return Transform.mulVec2(xf, m_centroid);
  }

  /** Get the centroid and apply the supplied transform. */
  Vector2 centroidToOut(final Transform xf, final Vector2 out) {
    Transform.mulToOutUnsafeVec2(xf, m_centroid, out);
    return out;
  }
}
