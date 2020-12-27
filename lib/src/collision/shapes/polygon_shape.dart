part of forge2d;

/// A convex polygon shape. Polygons have a maximum number of vertices equal to _maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class PolygonShape extends Shape {
  /// Dump lots of debug information.
  static const bool _debug = false;

  /// Local position of the shape centroid in parent body frame.
  final Vector2 centroid = Vector2.zero();

  /// The vertices of the shape. Note: use vertexCount, not _vertices.length, to get number of
  /// active vertices.
  final List<Vector2> vertices = [];

  /// The normals of the shape. Note: use vertexCount, not _normals.length, to get number of
  /// active normals.
  final List<Vector2> normals = [];

  PolygonShape() : super(ShapeType.POLYGON) {
    radius = settings.polygonRadius;
  }

  @override
  Shape clone() {
    final PolygonShape shape = PolygonShape();
    shape.centroid.setFrom(centroid);
    normals.forEach((normal) => shape.normals.add(normal.clone()));
    vertices.forEach((vertex) => shape.vertices.add(vertex.clone()));
    shape.radius = radius;
    return shape;
  }

  /// Create a convex hull from the given array of points. The length of the list
  /// must be in the range [3, Settings.maxPolygonVertices].
  /// @warning the points may be re-ordered, even if they form a convex polygon.
  /// @warning collinear points are removed.
  void set(final List<Vector2> updatedVertices) {
    final updatedCount = updatedVertices.length;
    assert(updatedCount >= 3, "Too few vertices to form polygon");
    assert(updatedCount <= settings.maxPolygonVertices, "Too many vertices");
    if (updatedCount < 3) {
      setAsBoxXY(1.0, 1.0);
      return;
    }

    // Perform welding and copy vertices into local buffer.
    final List<Vector2> points = [];
    for (Vector2 v in updatedVertices) {
      bool unique = true;
      for (int j = 0; j < points.length; ++j) {
        if (v.distanceToSquared(points[j]) < 0.5 * settings.linearSlop) {
          unique = false;
          break;
        }
      }

      if (unique) {
        points.add(v.clone());
        if (points.length == settings.maxPolygonVertices) {
          break;
        }
      }
    }

    if (points.length < 3) {
      assert(false, "Too few vertices to be a polygon");
      setAsBoxXY(1.0, 1.0);
      return;
    }

    // Create the convex hull using the Gift wrapping algorithm
    // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

    // Find the right most point on the hull
    Vector2 rightMostPoint = points.first;
    for (Vector2 point in points) {
      final double x = point.x;
      final double y = point.y;
      final double x0 = rightMostPoint.x;
      final double y0 = rightMostPoint.y;
      if (x > x0 || (x == x0 && y < y0)) {
        rightMostPoint = point;
      }
    }
    print(rightMostPoint);

    final List<Vector2> hull = [rightMostPoint];
    Vector2 pointToAdd = rightMostPoint;
    print(points);

    for (Vector2 point1 in points) {
      Vector2 lastCheckedPoint = pointToAdd;
      for (Vector2 point2 in points) {
        if (lastCheckedPoint == pointToAdd) {
          lastCheckedPoint = point2;
          continue;
        }

        final Vector2 r = lastCheckedPoint.clone()..sub(point1);
        final Vector2 v = point2.clone()..sub(point1);
        final double c = r.cross(v);
        if (c < 0.0) {
          lastCheckedPoint = point2;
        }

        // Collinearity check
        if (c == 0.0 && v.length2 > r.length2) {
          lastCheckedPoint = point2;
        }
      }

      pointToAdd = lastCheckedPoint;
      if (!hull.contains(pointToAdd)) {
        hull.add(pointToAdd);
      }
    }

    // Copy vertices.
    vertices.clear();
    vertices.addAll(hull);
    vertices.forEach((_) => normals.add(Vector2.zero()));

    final Vector2 edge = Vector2.zero();

    // Compute normals. Ensure the edges have non-zero length.
    print(hull);
    print(vertices);
    for (int i = 0; i < vertices.length; ++i) {
      final int i1 = i;
      final int i2 = (i + 1) % vertices.length;
      edge
        ..setFrom(vertices[i2])
        ..sub(vertices[i1]);

      assert(edge.length2 > settings.EPSILON * settings.EPSILON);
      edge.scaleOrthogonalInto(-1.0, normals[i]);
      normals[i].normalize();
    }

    // Compute the polygon centroid.
    computeCentroid(vertices, vertices.length);
  }

  /// Build vertices to represent an axis-aligned box.
  ///
  /// @param hx the half-width.
  /// @param hy the half-height.
  void setAsBoxXY(final double hx, final double hy) {
    vertices.clear();
    vertices.addAll([
      Vector2(-hx, -hy),
      Vector2(hx, -hy),
      Vector2(hx, hy),
      Vector2(-hx, hy),
    ]);
    normals.clear();
    normals.addAll([
      Vector2(0.0, -1.0),
      Vector2(1.0, 0.0),
      Vector2(0.0, 1.0),
      Vector2(-1.0, 0.0),
    ]);
    centroid.setZero();
  }

  /// Build vertices to represent an oriented box.
  ///
  /// @param hx the half-width.
  /// @param hy the half-height.
  /// @param center the center of the box in local coordinates.
  /// @param angle the rotation of the box in local coordinates.
  void setAsBox(final double hx, final double hy, final Vector2 center,
      final double angle) {
    setAsBoxXY(hx, hy);
    centroid.setFrom(center);

    final Transform xf = Transform.zero();
    xf.p.setFrom(center);
    xf.q.setAngle(angle);

    // Transform vertices and normals.
    for (int i = 0; i < vertices.length; ++i) {
      vertices[i].setFrom(Transform.mulVec2(xf, vertices[i]));
      normals[i].setFrom(Rot.mulVec2(xf.q, normals[i]));
    }
  }

  /// Set this as a single edge.
  void setAsEdge(Vector2 v1, Vector2 v2) {
    vertices.clear();
    vertices.add(v1.clone());
    vertices.add(v2.clone());
    centroid
      ..setFrom(v1)
      ..add(v2)
      ..scale(0.5);
    normals.clear();
    normals.add(v2 - v1);
    normals[0].scaleOrthogonalInto(-1.0, normals[0]);
    normals[0].normalize();
    normals.add(-normals[0]);
  }

  @override
  int getChildCount() {
    return 1;
  }

  @override
  bool testPoint(final Transform xf, final Vector2 p) {
    final Rot xfq = xf.q;

    double tempX = p.x - xf.p.x;
    double tempY = p.y - xf.p.y;
    final double pLocalx = xfq.c * tempX + xfq.s * tempY;
    final double pLocaly = -xfq.s * tempX + xfq.c * tempY;

    if (_debug) {
      print("--testPoint debug--");
      print("Vertices: ");
      vertices.forEach(print);
      print("pLocal: $pLocalx, $pLocaly");
    }

    for (int i = 0; i < vertices.length; ++i) {
      final Vector2 vertex = vertices[i];
      final Vector2 normal = normals[i];
      tempX = pLocalx - vertex.x;
      tempY = pLocaly - vertex.y;
      final double dot = normal.x * tempX + normal.y * tempY;
      if (dot > 0.0) {
        return false;
      }
    }

    return true;
  }

  @override
  void computeAABB(final AABB aabb, final Transform xf, int childIndex) {
    final Vector2 lower = aabb.lowerBound;
    final Vector2 upper = aabb.upperBound;
    final Vector2 v1 = vertices[0];
    final double xfqc = xf.q.c;
    final double xfqs = xf.q.s;
    final double xfpx = xf.p.x;
    final double xfpy = xf.p.y;
    lower.x = (xfqc * v1.x - xfqs * v1.y) + xfpx;
    lower.y = (xfqs * v1.x + xfqc * v1.y) + xfpy;
    upper.x = lower.x;
    upper.y = lower.y;

    for (int i = 1; i < vertices.length; ++i) {
      final Vector2 v2 = vertices[i];
      // Vec2 v = Mul(xf, _vertices[i]);
      final double vx = (xfqc * v2.x - xfqs * v2.y) + xfpx;
      final double vy = (xfqs * v2.x + xfqc * v2.y) + xfpy;
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

  @override
  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    final double xfqc = xf.q.c;
    final double xfqs = xf.q.s;
    double tx = p.x - xf.p.x;
    double ty = p.y - xf.p.y;
    final double pLocalx = xfqc * tx + xfqs * ty;
    final double pLocaly = -xfqs * tx + xfqc * ty;

    double maxDistance = -double.maxFinite;
    double normalForMaxDistanceX = pLocalx;
    double normalForMaxDistanceY = pLocaly;

    for (int i = 0; i < vertices.length; ++i) {
      final Vector2 vertex = vertices[i];
      final Vector2 normal = normals[i];
      tx = pLocalx - vertex.x;
      ty = pLocaly - vertex.y;
      final double dot = normal.x * tx + normal.y * ty;
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
      for (int i = 0; i < vertices.length; ++i) {
        final Vector2 vertex = vertices[i];
        final double distanceVecX = pLocalx - vertex.x;
        final double distanceVecY = pLocaly - vertex.y;
        final double distance2 =
            distanceVecX * distanceVecX + distanceVecY * distanceVecY;
        if (minDistance2 > distance2) {
          minDistanceX = distanceVecX;
          minDistanceY = distanceVecY;
          minDistance2 = distance2;
        }
      }
      distance = math.sqrt(minDistance2);
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

  @override
  bool raycast(
      RayCastOutput output, RayCastInput input, Transform xf, int childIndex) {
    final double xfqc = xf.q.c;
    final double xfqs = xf.q.s;
    final Vector2 xfp = xf.p;
    double tempX = input.p1.x - xfp.x;
    double tempY = input.p1.y - xfp.y;
    final double p1x = xfqc * tempX + xfqs * tempY;
    final double p1y = -xfqs * tempX + xfqc * tempY;

    tempX = input.p2.x - xfp.x;
    tempY = input.p2.y - xfp.y;
    final double p2x = xfqc * tempX + xfqs * tempY;
    final double p2y = -xfqs * tempX + xfqc * tempY;

    final double dx = p2x - p1x;
    final double dy = p2y - p1y;

    double lower = 0.0, upper = input.maxFraction;

    int index = -1;

    for (int i = 0; i < vertices.length; ++i) {
      final Vector2 normal = normals[i];
      final Vector2 vertex = vertices[i];
      final double tempX = vertex.x - p1x;
      final double tempY = vertex.y - p1y;
      final double numerator = normal.x * tempX + normal.y * tempY;
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
      final Vector2 normal = normals[index];
      final Vector2 out = output.normal;
      out.x = xfqc * normal.x - xfqs * normal.y;
      out.y = xfqs * normal.x + xfqc * normal.y;
      return true;
    }
    return false;
  }

  void computeCentroid(final List<Vector2> vs, final int count) {
    assert(count >= 3);

    centroid.setZero();
    double area = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    final Vector2 pRef = Vector2.zero();

    final Vector2 e1 = Vector2.zero();
    final Vector2 e2 = Vector2.zero();

    const double inv3 = 1.0 / 3.0;

    for (int i = 0; i < count; ++i) {
      // Triangle vertices.
      final Vector2 p1 = pRef;
      final Vector2 p2 = vs[i];
      final Vector2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

      e1
        ..setFrom(p2)
        ..sub(p1);
      e2
        ..setFrom(p3)
        ..sub(p1);

      final double D = e1.cross(e2);

      final double triangleArea = 0.5 * D;
      area += triangleArea;

      // Area weighted centroid
      e1
        ..setFrom(p1)
        ..add(p2)
        ..add(p3)
        ..scale(triangleArea * inv3);
      centroid.add(e1);
    }

    // Centroid
    assert(area > settings.EPSILON);
    centroid.scale(1.0 / area);
  }

  @override
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

    assert(vertices.length >= 3);

    final Vector2 center = Vector2.zero();
    double area = 0.0;
    double I = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    final Vector2 s = Vector2.zero();
    // This code would put the reference point inside the polygon.
    for (int i = 0; i < vertices.length; ++i) {
      s.add(vertices[i]);
    }
    s.scale(1.0 / vertices.length.toDouble());

    const double kInv3 = 1.0 / 3.0;

    final Vector2 e1 = Vector2.zero();
    final Vector2 e2 = Vector2.zero();

    for (int i = 0; i < vertices.length; ++i) {
      // Triangle vertices.
      e1
        ..setFrom(vertices[i])
        ..sub(s);
      e2
        ..setFrom(s)
        ..negate()
        ..add(i + 1 < vertices.length ? vertices[i + 1] : vertices[0]);

      final double D = e1.cross(e2);

      final double triangleArea = 0.5 * D;
      area += triangleArea;

      // Area weighted centroid
      center.x += triangleArea * kInv3 * (e1.x + e2.x);
      center.y += triangleArea * kInv3 * (e1.y + e2.y);

      final double ex1 = e1.x, ey1 = e1.y;
      final double ex2 = e2.x, ey2 = e2.y;

      final double intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
      final double inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

      I += (0.25 * kInv3 * D) * (intx2 + inty2);
    }

    // Total mass
    massData.mass = density * area;

    // Center of mass
    assert(area > settings.EPSILON);
    center.scale(1.0 / area);
    massData.center
      ..setFrom(center)
      ..add(s);

    // Inertia tensor relative to the local origin (point s)
    massData.I = I * density;

    // Shift to center of mass then to original body origin.
    massData.I += massData.mass * (massData.center.dot(massData.center));
  }

  /// Validate convexity. This is a very time consuming operation.
  bool validate() {
    for (int i = 0; i < vertices.length; ++i) {
      final int i1 = i;
      final int i2 = i < vertices.length - 1 ? i1 + 1 : 0;
      final Vector2 p = vertices[i1];
      final Vector2 e = Vector2.copy(vertices[i2])..sub(p);

      for (int j = 0; j < vertices.length; ++j) {
        if (j == i1 || j == i2) {
          continue;
        }

        final Vector2 v = Vector2.copy(vertices[j])..sub(p);
        final double c = e.cross(v);
        if (c < 0.0) {
          return false;
        }
      }
    }

    return true;
  }

  /// Get the centroid and apply the supplied transform.
  Vector2 applyToCentroid(final Transform xf) {
    return Transform.mulVec2(xf, centroid);
  }
}
