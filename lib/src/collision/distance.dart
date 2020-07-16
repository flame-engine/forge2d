part of box2d;

/// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
class _SimplexVertex {
  final Vector2 wA = Vector2.zero(); // support point in shapeA
  final Vector2 wB = Vector2.zero(); // support point in shapeB
  final Vector2 w = Vector2.zero(); // wB - wA
  double a = 0.0; // barycentric coordinate for closest point
  int indexA = 0; // wA index
  int indexB = 0; // wB index

  void set(_SimplexVertex sv) {
    wA.setFrom(sv.wA);
    wB.setFrom(sv.wB);
    w.setFrom(sv.w);
    a = sv.a;
    indexA = sv.indexA;
    indexB = sv.indexB;
  }
}

class SimplexCache {
  /// length or area
  double metric = 0.0;
  int count = 0;

  /// vertices on shape A
  final List<int> indexA = BufferUtils.intList(3);

  /// vertices on shape B
  final List<int> indexB = BufferUtils.intList(3);

  SimplexCache() {
    indexA[0] = Settings.INTEGER_MAX_VALUE;
    indexA[1] = Settings.INTEGER_MAX_VALUE;
    indexA[2] = Settings.INTEGER_MAX_VALUE;
    indexB[0] = Settings.INTEGER_MAX_VALUE;
    indexB[1] = Settings.INTEGER_MAX_VALUE;
    indexB[2] = Settings.INTEGER_MAX_VALUE;
  }

  void set(SimplexCache sc) {
    BufferUtils.arrayCopy(sc.indexA, 0, indexA, 0, indexA.length);
    BufferUtils.arrayCopy(sc.indexB, 0, indexB, 0, indexB.length);
    metric = sc.metric;
    count = sc.count;
  }
}

class _Simplex {
  final _SimplexVertex v1 = _SimplexVertex();
  final _SimplexVertex v2 = _SimplexVertex();
  final _SimplexVertex v3 = _SimplexVertex();
  final List<_SimplexVertex> vertices = List<_SimplexVertex>(3);
  int count = 0;

  _Simplex() {
    vertices[0] = v1;
    vertices[1] = v2;
    vertices[2] = v3;
  }

  void readCache(SimplexCache cache, DistanceProxy proxyA, Transform transformA,
      DistanceProxy proxyB, Transform transformB) {
    assert(cache.count <= 3);

    // Copy data from cache.
    count = cache.count;

    for (int i = 0; i < count; ++i) {
      _SimplexVertex v = vertices[i];
      v.indexA = cache.indexA[i];
      v.indexB = cache.indexB[i];
      Vector2 wALocal = proxyA.getVertex(v.indexA);
      Vector2 wBLocal = proxyB.getVertex(v.indexB);
      v.wA.setFrom(Transform.mulVec2(transformA, wALocal));
      v.wB.setFrom(Transform.mulVec2(transformB, wBLocal));
      v.w
        ..setFrom(v.wB)
        ..sub(v.wA);
      v.a = 0.0;
    }

    // Compute the new simplex metric, if it is substantially different than
    // old metric then flush the simplex.
    if (count > 1) {
      double metric1 = cache.metric;
      double metric2 = getMetric();
      if (metric2 < 0.5 * metric1 ||
          2.0 * metric1 < metric2 ||
          metric2 < Settings.EPSILON) {
        // Reset the simplex.
        count = 0;
      }
    }

    // If the cache is empty or invalid ...
    if (count == 0) {
      _SimplexVertex v = vertices[0];
      v.indexA = 0;
      v.indexB = 0;
      Vector2 wALocal = proxyA.getVertex(0);
      Vector2 wBLocal = proxyB.getVertex(0);
      v.wA.setFrom(Transform.mulVec2(transformA, wALocal));
      v.wB.setFrom(Transform.mulVec2(transformB, wBLocal));
      v.w
        ..setFrom(v.wB)
        ..sub(v.wA);
      count = 1;
    }
  }

  void writeCache(SimplexCache cache) {
    cache.metric = getMetric();
    cache.count = count;

    for (int i = 0; i < count; ++i) {
      cache.indexA[i] = (vertices[i].indexA).toInt();
      cache.indexB[i] = (vertices[i].indexB).toInt();
    }
  }

  final Vector2 _e12 = Vector2.zero();

  void getSearchDirection(final Vector2 out) {
    switch (count) {
      case 1:
        out
          ..setFrom(v1.w)
          ..negate();
        return;
      case 2:
        _e12
          ..setFrom(v2.w)
          ..sub(v1.w);
        // use out for a temp variable real quick
        out
          ..setFrom(v1.w)
          ..negate();
        double sgn = _e12.cross(out);

        if (sgn > 0.0) {
          // Origin is left of e12.
          _e12.scaleOrthogonalInto(1.0, out);
        } else {
          // Origin is right of e12.
          _e12.scaleOrthogonalInto(-1.0, out);
        }
        return;
      default:
        assert(false);
        out.setZero();
        return;
    }
  }

  // djm pooled
  final Vector2 _case2 = Vector2.zero();
  final Vector2 _case22 = Vector2.zero();

  /// This returns pooled objects. don't keep or modify them
  void getClosestPoint(final Vector2 out) {
    switch (count) {
      case 0:
        assert(false);
        out.setZero();
        return;
      case 1:
        out.setFrom(v1.w);
        return;
      case 2:
        _case22
          ..setFrom(v2.w)
          ..scale(v2.a);
        _case2
          ..setFrom(v1.w)
          ..scale(v1.a)
          ..add(_case22);
        out.setFrom(_case2);
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
  final Vector2 _case3 = Vector2.zero();
  final Vector2 _case33 = Vector2.zero();

  void getWitnessPoints(Vector2 pA, Vector2 pB) {
    switch (count) {
      case 0:
        assert(false);
        break;

      case 1:
        pA.setFrom(v1.wA);
        pB.setFrom(v1.wB);
        break;

      case 2:
        _case2
          ..setFrom(v1.wA)
          ..scale(v1.a);
        pA
          ..setFrom(v2.wA)
          ..scale(v2.a)
          ..add(_case2);
        // v1.a * v1.wA + v2.a * v2.wA;
        // *pB = v1.a * v1.wB + v2.a * v2.wB;
        _case2
          ..setFrom(v1.wB)
          ..scale(v1.a);
        pB
          ..setFrom(v2.wB)
          ..scale(v2.a)
          ..add(_case2);

        break;

      case 3:
        pA
          ..setFrom(v1.wA)
          ..scale(v1.a);
        _case3
          ..setFrom(v2.wA)
          ..scale(v2.a);
        _case33
          ..setFrom(v3.wA)
          ..scale(v3.a);
        pA..add(_case3)..add(_case33);
        pB.setFrom(pA);
        // *pA = v1.a * v1.wA + v2.a * v2.wA + v3.a * v3.wA;
        // *pB = *pA;
        break;

      default:
        assert(false);
        break;
    }
  }

  // djm pooled, from above
  double getMetric() {
    switch (count) {
      case 0:
        assert(false);
        return 0.0;

      case 1:
        return 0.0;

      case 2:
        return MathUtils.distance(v1.w, v2.w);

      case 3:
        _case3
          ..setFrom(v2.w)
          ..sub(v1.w);
        _case33
          ..setFrom(v3.w)
          ..sub(v1.w);
        // return Vec2.cross(v2.w - v1.w, v3.w - v1.w);
        return _case3.cross(_case33);

      default:
        assert(false);
        return 0.0;
    }
  }

  // djm pooled from above
  /// Solve a line segment using barycentric coordinates.
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
    final Vector2 w1 = v1.w;
    final Vector2 w2 = v2.w;
    _e12
      ..setFrom(w2)
      ..sub(w1);

    // w1 region
    double d12_2 = -w1.dot(_e12);
    if (d12_2 <= 0.0) {
      // a2 <= 0, so we clamp it to 0
      v1.a = 1.0;
      count = 1;
      return;
    }

    // w2 region
    double d12_1 = w2.dot(_e12);
    if (d12_1 <= 0.0) {
      // a1 <= 0, so we clamp it to 0
      v2.a = 1.0;
      count = 1;
      v1.set(v2);
      return;
    }

    // Must be in e12 region.
    double inv_d12 = 1.0 / (d12_1 + d12_2);
    v1.a = d12_1 * inv_d12;
    v2.a = d12_2 * inv_d12;
    count = 2;
  }

  // djm pooled, and from above
  final Vector2 _e13 = Vector2.zero();
  final Vector2 _e23 = Vector2.zero();
  final Vector2 _w1 = Vector2.zero();
  final Vector2 _w2 = Vector2.zero();
  final Vector2 _w3 = Vector2.zero();

  /// Solve a line segment using barycentric coordinates.<br/>
  /// Possible regions:<br/>
  /// - points[2]<br/>
  /// - edge points[0]-points[2]<br/>
  /// - edge points[1]-points[2]<br/>
  /// - inside the triangle
  void solve3() {
    _w1.setFrom(v1.w);
    _w2.setFrom(v2.w);
    _w3.setFrom(v3.w);

    // Edge12
    // [1 1 ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    _e12
      ..setFrom(_w2)
      ..sub(_w1);
    double w1e12 = _w1.dot(_e12);
    double w2e12 = _w2.dot(_e12);
    double d12_1 = w2e12;
    double d12_2 = -w1e12;

    // Edge13
    // [1 1 ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    _e13
      ..setFrom(_w3)
      ..sub(_w1);
    double w1e13 = _w1.dot(_e13);
    double w3e13 = _w3.dot(_e13);
    double d13_1 = w3e13;
    double d13_2 = -w1e13;

    // Edge23
    // [1 1 ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    _e23
      ..setFrom(_w3)
      ..sub(_w2);
    double w2e23 = _w2.dot(_e23);
    double w3e23 = _w3.dot(_e23);
    double d23_1 = w3e23;
    double d23_2 = -w2e23;

    // Triangle123
    double n123 = _e12.cross(_e13);

    double d123_1 = n123 * _w2.cross(_w3);
    double d123_2 = n123 * _w3.cross(_w1);
    double d123_3 = n123 * _w1.cross(_w2);

    // w1 region
    if (d12_2 <= 0.0 && d13_2 <= 0.0) {
      v1.a = 1.0;
      count = 1;
      return;
    }

    // e12
    if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0) {
      double inv_d12 = 1.0 / (d12_1 + d12_2);
      v1.a = d12_1 * inv_d12;
      v2.a = d12_2 * inv_d12;
      count = 2;
      return;
    }

    // e13
    if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0) {
      double inv_d13 = 1.0 / (d13_1 + d13_2);
      v1.a = d13_1 * inv_d13;
      v3.a = d13_2 * inv_d13;
      count = 2;
      v2.set(v3);
      return;
    }

    // w2 region
    if (d12_1 <= 0.0 && d23_2 <= 0.0) {
      v2.a = 1.0;
      count = 1;
      v1.set(v2);
      return;
    }

    // w3 region
    if (d13_1 <= 0.0 && d23_1 <= 0.0) {
      v3.a = 1.0;
      count = 1;
      v1.set(v3);
      return;
    }

    // e23
    if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0) {
      double inv_d23 = 1.0 / (d23_1 + d23_2);
      v2.a = d23_1 * inv_d23;
      v3.a = d23_2 * inv_d23;
      count = 2;
      v1.set(v3);
      return;
    }

    // Must be in triangle123
    double inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
    v1.a = d123_1 * inv_d123;
    v2.a = d123_2 * inv_d123;
    v3.a = d123_3 * inv_d123;
    count = 3;
  }
} // Class _Simplex

class DistanceProxy {
  final List<Vector2> vertices;
  int _count;
  double radius;
  final List<Vector2> buffer;

  DistanceProxy()
      : vertices = List<Vector2>(Settings.maxPolygonVertices),
        buffer = List<Vector2>(2) {
    for (int i = 0; i < vertices.length; i++) {
      vertices[i] = Vector2.zero();
    }
    _count = 0;
    radius = 0.0;
  }

  /// Initialize the proxy using the given shape. The shape must remain in scope while the proxy is
  /// in use.
  void set(final Shape shape, int index) {
    switch (shape.shapeType) {
      case ShapeType.CIRCLE:
        final circle = shape as CircleShape;
        vertices[0].setFrom(circle.position);
        _count = 1;
        radius = circle.radius;

        break;
      case ShapeType.POLYGON:
        final poly = shape as PolygonShape;
        _count = poly.count;
        radius = poly.radius;
        for (int i = 0; i < _count; i++) {
          vertices[i].setFrom(poly.vertices[i]);
        }
        break;
      case ShapeType.CHAIN:
        final chain = shape as ChainShape;
        assert(0 <= index && index < chain._count);

        buffer[0] = chain._vertices[index];
        if (index + 1 < chain._count) {
          buffer[1] = chain._vertices[index + 1];
        } else {
          buffer[1] = chain._vertices[0];
        }

        vertices[0].setFrom(buffer[0]);
        vertices[1].setFrom(buffer[1]);
        _count = 2;
        radius = chain.radius;
        break;
      case ShapeType.EDGE:
        final edge = shape as EdgeShape;
        vertices[0].setFrom(edge.vertex1);
        vertices[1].setFrom(edge.vertex2);
        _count = 2;
        radius = edge.radius;
        break;
      default:
        assert(false);
    }
  }

  /// Get the supporting vertex index in the given direction.
  int getSupport(final Vector2 d) {
    int bestIndex = 0;
    double bestValue = vertices[0].dot(d);
    for (int i = 1; i < _count; i++) {
      double value = vertices[i].dot(d);
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
    }

    return bestIndex;
  }

  /// Get the supporting vertex in the given direction.
  Vector2 getSupportVertex(final Vector2 d) {
    int bestIndex = 0;
    double bestValue = vertices[0].dot(d);
    for (int i = 1; i < _count; i++) {
      double value = vertices[i].dot(d);
      if (value > bestValue) {
        bestIndex = i;
        bestValue = value;
      }
    }

    return vertices[bestIndex];
  }

  /// Get the vertex count.
  int getVertexCount() {
    return _count;
  }

  /// Get a vertex by index. Used by Distance.
  Vector2 getVertex(int index) {
    assert(0 <= index && index < _count);
    return vertices[index];
  }
} // Class _DistanceProxy.

class Distance {
  static const int MAX_ITERS = 20;

  static int GJK_CALLS = 0;
  static int GJK_ITERS = 0;
  static int GJK_MAX_ITERS = 20;

  final _Simplex _simplex = _Simplex();
  final List<int> _saveA = BufferUtils.intList(3);
  final List<int> _saveB = BufferUtils.intList(3);
  final Vector2 _closestPoint = Vector2.zero();
  final Vector2 _d = Vector2.zero();
  final Vector2 _temp = Vector2.zero();
  final Vector2 _normal = Vector2.zero();

  /// Compute the closest points between two shapes. Supports any combination of: CircleShape and
  /// PolygonShape. The simplex cache is input/output. On the first call set SimplexCache.count to
  /// zero.
  void compute(final DistanceOutput output, final SimplexCache cache,
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
    double distanceSqr1 = _closestPoint.length2;
    double distanceSqr2 = distanceSqr1;

    // Main iteration loop
    int iter = 0;
    while (iter < MAX_ITERS) {
      // Copy simplex so we can identify duplicates.
      saveCount = _simplex.count;
      for (int i = 0; i < saveCount; i++) {
        _saveA[i] = vertices[i].indexA;
        _saveB[i] = vertices[i].indexB;
      }

      switch (_simplex.count) {
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
      if (_simplex.count == 3) {
        break;
      }

      // Compute closest point.
      _simplex.getClosestPoint(_closestPoint);
      distanceSqr2 = _closestPoint.length2;

      // ensure progress
      if (distanceSqr2 >= distanceSqr1) {
        // break;
      }
      distanceSqr1 = distanceSqr2;

      // get search direction;
      _simplex.getSearchDirection(_d);

      // Ensure the search direction is numerically fit.
      if (_d.length2 < Settings.EPSILON * Settings.EPSILON) {
        // The origin is probably contained by a line segment
        // or triangle. Thus the shapes are overlapped.

        // We can't return zero here even though there may be overlap.
        // In case the simplex is a point, segment, or triangle it is difficult
        // to determine if the origin is contained in the CSO or very close to it.
        break;
      }
      /*
       * SimplexVertex* vertex = vertices + simplex.count; vertex.indexA =
       * proxyA.GetSupport(MulT(transformA.R, -d)); vertex.wA = Mul(transformA,
       * proxyA.GetVertex(vertex.indexA)); Vec2 wBLocal; vertex.indexB =
       * proxyB.GetSupport(MulT(transformB.R, d)); vertex.wB = Mul(transformB,
       * proxyB.GetVertex(vertex.indexB)); vertex.w = vertex.wB - vertex.wA;
       */

      // Compute a tentative new simplex vertex using support points.
      _SimplexVertex vertex = vertices[_simplex.count];

      _temp.setFrom(Rot.mulTransVec2(transformA.q, _d..negate()));
      vertex.indexA = proxyA.getSupport(_temp);
      vertex.wA.setFrom(
          Transform.mulVec2(transformA, proxyA.getVertex(vertex.indexA)));
      // Vec2 wBLocal;
      _temp.setFrom(Rot.mulTransVec2(transformB.q, _d..negate()));
      vertex.indexB = proxyB.getSupport(_temp);
      vertex.wB.setFrom(
          Transform.mulVec2(transformB, proxyB.getVertex(vertex.indexB)));
      (vertex.w..setFrom(vertex.wB)).sub(vertex.wA);

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
      ++_simplex.count;
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
      double rA = proxyA.radius;
      double rB = proxyB.radius;

      if (output.distance > rA + rB && output.distance > Settings.EPSILON) {
        // Shapes are still no overlapped.
        // Move the witness points to the outer surface.
        output.distance -= rA + rB;
        _normal
          ..setFrom(output.pointB)
          ..sub(output.pointA);
        _normal.normalize();
        _temp
          ..setFrom(_normal)
          ..scale(rA);
        output.pointA.add(_temp);
        _temp
          ..setFrom(_normal)
          ..scale(rB);
        output.pointB.sub(_temp);
      } else {
        // Shapes are overlapped when radii are considered.
        // Move the witness points to the middle.
        // Vec2 p = 0.5f * (output.pointA + output.pointB);
        output.pointA
          ..add(output.pointB)
          ..scale(.5);
        output.pointB.setFrom(output.pointA);
        output.distance = 0.0;
      }
    }
  }
}
