import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// A convex polygon shape. Polygons have a maximum number of vertices equal to
/// _maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class PolygonShape extends Shape {
  /// Local position of the shape centroid in parent body frame.
  final Vector2 centroid = Vector2.zero();

  /// The vertices of the shape. Note: use vertexCount, not _vertices.length,
  /// to get number of active vertices.
  final List<Vector2> vertices = [];

  /// The normals of the shape. Note: use vertexCount, not _normals.length, to
  /// get number of active normals.
  final List<Vector2> normals = [];

  PolygonShape() : super(ShapeType.polygon) {
    radius = settings.polygonRadius;
  }

  @override
  Shape clone() {
    final shape = PolygonShape();
    shape.centroid.setFrom(centroid);
    normals.forEach((normal) => shape.normals.add(normal.clone()));
    vertices.forEach((vertex) => shape.vertices.add(vertex.clone()));
    shape.radius = radius;
    return shape;
  }

  /// Create a convex hull from the given array of points. The length of the
  /// list must be in the range [3, Settings.maxPolygonVertices].
  /// Warning: the points may be re-ordered, even if they form a convex polygon.
  /// Warning: collinear points are removed.
  void set(List<Vector2> updatedVertices) {
    final updatedCount = updatedVertices.length;
    assert(updatedCount >= 3, 'Too few vertices to form polygon');
    assert(updatedCount <= settings.maxPolygonVertices, 'Too many vertices');
    if (updatedCount < 3) {
      setAsBoxXY(1.0, 1.0);
      return;
    }

    // Perform welding and copy vertices into local buffer.
    final points = <Vector2>[];
    for (final v in updatedVertices) {
      var unique = true;
      for (var j = 0; j < points.length; ++j) {
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
      assert(false, 'Too few vertices to be a polygon');
      setAsBoxXY(1.0, 1.0);
      return;
    }

    // Create the convex hull using the Gift wrapping algorithm
    // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

    // Find the right most point on the hull
    var rightMostPoint = points.first;
    for (final point in points) {
      final x = point.x;
      final y = point.y;
      final x0 = rightMostPoint.x;
      final y0 = rightMostPoint.y;
      if (x > x0 || (x == x0 && y < y0)) {
        rightMostPoint = point;
      }
    }

    final hull = <Vector2>[rightMostPoint];
    var pointOnHull = rightMostPoint;
    do {
      // Set first point in the set as the initial candidate for the
      // next point on the convex hull.
      var endPoint = points[0];

      // Test the candidate point against all points in the set to find
      // the next convex hull point.
      for (final point in points) {
        // If the candidate point is the current last point on the convex
        // hull, update the candidate point to the current point and continue
        // checking against the remaining points.
        if (endPoint == pointOnHull) {
          endPoint = point;
          continue;
        }

        // Use the cross product of the vectors from the current convex hull
        // point to the candidate point and the test point to see if the winding
        // changes from CCW to CW. This indicates the current point is a better
        // candidate for a hull point. Update the candidate point.
        final r = endPoint.clone()..sub(pointOnHull);
        final v = point.clone()..sub(pointOnHull);
        final c = r.cross(v);
        if (c < 0.0) {
          endPoint = point;
        }

        // Collinearity check
        if (c == 0.0 && v.length2 > r.length2) {
          endPoint = point;
        }
      }

      // Set the end point candidate as the new current convex hull point.
      pointOnHull = endPoint;
      if (!hull.contains(pointOnHull)) {
        hull.add(pointOnHull);
      }
    } while (pointOnHull != hull.first);

    // Copy vertices.
    vertices.clear();
    vertices.addAll(hull);
    vertices.forEach((_) => normals.add(Vector2.zero()));

    final edge = Vector2.zero();

    // Compute normals. Ensure the edges have non-zero length.
    for (var i = 0; i < vertices.length; ++i) {
      final i1 = i;
      final i2 = (i + 1) % vertices.length;
      edge
        ..setFrom(vertices[i2])
        ..sub(vertices[i1]);

      assert(edge.length2 > settings.epsilon * settings.epsilon);
      edge.scaleOrthogonalInto(-1.0, normals[i]);
      normals[i].normalize();
    }

    // Compute the polygon centroid.
    computeCentroid(vertices, vertices.length);
  }

  /// Build vertices to represent an axis-aligned box.
  void setAsBoxXY(double halfWidth, double halfHeight) {
    vertices.clear();
    vertices.addAll([
      Vector2(-halfWidth, -halfHeight),
      Vector2(halfWidth, -halfHeight),
      Vector2(halfWidth, halfHeight),
      Vector2(-halfWidth, halfHeight),
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
  /// [center] and [angle] should be in local coordinates.
  void setAsBox(
    double halfWidth,
    double halfHeight,
    Vector2 center,
    double angle,
  ) {
    setAsBoxXY(halfWidth, halfHeight);
    centroid.setFrom(center);

    final xf = Transform.zero();
    xf.p.setFrom(center);
    xf.q.setAngle(angle);

    // Transform vertices and normals.
    for (var i = 0; i < vertices.length; ++i) {
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
  int get childCount {
    return 1;
  }

  @override
  bool testPoint(Transform xf, Vector2 p) {
    final xfq = xf.q;

    var tempX = p.x - xf.p.x;
    var tempY = p.y - xf.p.y;
    final pLocalX = xfq.cos * tempX + xfq.sin * tempY;
    final pLocalY = -xfq.sin * tempX + xfq.cos * tempY;

    for (var i = 0; i < vertices.length; ++i) {
      final vertex = vertices[i];
      final normal = normals[i];
      tempX = pLocalX - vertex.x;
      tempY = pLocalY - vertex.y;
      final dot = normal.x * tempX + normal.y * tempY;
      if (dot > 0.0) {
        return false;
      }
    }

    return true;
  }

  @override
  void computeAABB(AABB aabb, Transform xf, int childIndex) {
    final lower = aabb.lowerBound;
    final upper = aabb.upperBound;
    final v1 = vertices[0];
    final xfqc = xf.q.cos;
    final xfqs = xf.q.sin;
    final xfpx = xf.p.x;
    final xfpy = xf.p.y;
    lower.x = (xfqc * v1.x - xfqs * v1.y) + xfpx;
    lower.y = (xfqs * v1.x + xfqc * v1.y) + xfpy;
    upper.x = lower.x;
    upper.y = lower.y;

    for (var i = 1; i < vertices.length; ++i) {
      final v2 = vertices[i];
      // Vec2 v = Mul(xf, _vertices[i]);
      final vx = (xfqc * v2.x - xfqs * v2.y) + xfpx;
      final vy = (xfqs * v2.x + xfqc * v2.y) + xfpy;
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
    Transform xf,
    Vector2 p,
    int childIndex,
    Vector2 normalOut,
  ) {
    final xfqc = xf.q.cos;
    final xfqs = xf.q.sin;
    var tx = p.x - xf.p.x;
    var ty = p.y - xf.p.y;
    final pLocalx = xfqc * tx + xfqs * ty;
    final pLocaly = -xfqs * tx + xfqc * ty;

    var maxDistance = -double.maxFinite;
    var normalForMaxDistanceX = pLocalx;
    var normalForMaxDistanceY = pLocaly;

    for (var i = 0; i < vertices.length; ++i) {
      final vertex = vertices[i];
      final normal = normals[i];
      tx = pLocalx - vertex.x;
      ty = pLocaly - vertex.y;
      final dot = normal.x * tx + normal.y * ty;
      if (dot > maxDistance) {
        maxDistance = dot;
        normalForMaxDistanceX = normal.x;
        normalForMaxDistanceY = normal.y;
      }
    }

    double distance;
    if (maxDistance > 0) {
      var minDistanceX = normalForMaxDistanceX;
      var minDistanceY = normalForMaxDistanceY;
      var minDistance2 = maxDistance * maxDistance;
      for (var i = 0; i < vertices.length; ++i) {
        final vertex = vertices[i];
        final distanceVecX = pLocalx - vertex.x;
        final distanceVecY = pLocaly - vertex.y;
        final distance2 =
            distanceVecX * distanceVecX + distanceVecY * distanceVecY;
        if (minDistance2 > distance2) {
          minDistanceX = distanceVecX;
          minDistanceY = distanceVecY;
          minDistance2 = distance2;
        }
      }
      distance = sqrt(minDistance2);
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
    RayCastOutput output,
    RayCastInput input,
    Transform xf,
    int childIndex,
  ) {
    final xfqc = xf.q.cos;
    final xfqs = xf.q.sin;
    final xfp = xf.p;
    var tempX = input.p1.x - xfp.x;
    var tempY = input.p1.y - xfp.y;
    final p1x = xfqc * tempX + xfqs * tempY;
    final p1y = -xfqs * tempX + xfqc * tempY;

    tempX = input.p2.x - xfp.x;
    tempY = input.p2.y - xfp.y;
    final p2x = xfqc * tempX + xfqs * tempY;
    final p2y = -xfqs * tempX + xfqc * tempY;

    final dx = p2x - p1x;
    final dy = p2y - p1y;

    var lower = 0.0;
    var upper = input.maxFraction;

    var index = -1;

    for (var i = 0; i < vertices.length; ++i) {
      final normal = normals[i];
      final vertex = vertices[i];
      final tempX = vertex.x - p1x;
      final tempY = vertex.y - p1y;
      final numerator = normal.x * tempX + normal.y * tempY;
      final denominator = normal.x * dx + normal.y * dy;

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
      final normal = normals[index];
      final out = output.normal;
      out.x = xfqc * normal.x - xfqs * normal.y;
      out.y = xfqs * normal.x + xfqc * normal.y;
      return true;
    }
    return false;
  }

  void computeCentroid(List<Vector2> vs, int count) {
    assert(count >= 3);

    centroid.setZero();
    var area = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    final pRef = Vector2.zero();

    final e1 = Vector2.zero();
    final e2 = Vector2.zero();

    const inv3 = 1.0 / 3.0;

    for (var i = 0; i < count; ++i) {
      // Triangle vertices.
      final p1 = pRef;
      final p2 = vs[i];
      final p3 = i + 1 < count ? vs[i + 1] : vs[0];

      e1
        ..setFrom(p2)
        ..sub(p1);
      e2
        ..setFrom(p3)
        ..sub(p1);

      final D = e1.cross(e2);

      final triangleArea = 0.5 * D;
      area += triangleArea.abs();

      // Area weighted centroid
      e1
        ..setFrom(p1)
        ..add(p2)
        ..add(p3)
        ..scale(triangleArea * inv3);
      centroid.add(e1);
    }

    // Centroid
    assert(area > settings.epsilon);
    centroid.scale(1.0 / area);
  }

  @override
  void computeMass(MassData massData, double density) {
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

    final center = Vector2.zero();
    var area = 0.0;
    var I = 0.0;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    final s = Vector2.zero();
    // This code would put the reference point inside the polygon.
    for (var i = 0; i < vertices.length; ++i) {
      s.add(vertices[i]);
    }
    s.scale(1.0 / vertices.length.toDouble());

    const kInv3 = 1.0 / 3.0;

    final e1 = Vector2.zero();
    final e2 = Vector2.zero();

    for (var i = 0; i < vertices.length; ++i) {
      // Triangle vertices.
      e1
        ..setFrom(vertices[i])
        ..sub(s);
      e2
        ..setFrom(s)
        ..negate()
        ..add(i + 1 < vertices.length ? vertices[i + 1] : vertices[0]);

      final D = e1.cross(e2);

      final triangleArea = 0.5 * D;
      area += triangleArea.abs();

      // Area weighted centroid
      center.x += triangleArea * kInv3 * (e1.x + e2.x);
      center.y += triangleArea * kInv3 * (e1.y + e2.y);

      final ex1 = e1.x;
      final ey1 = e1.y;
      final ex2 = e2.x;
      final ey2 = e2.y;

      final intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
      final inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

      I += (0.25 * kInv3 * D) * (intx2 + inty2);
    }

    // Total mass
    massData.mass = density * area;

    // Center of mass
    assert(area > settings.epsilon);
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
    for (var i = 0; i < vertices.length; ++i) {
      final i1 = i;
      final i2 = i < vertices.length - 1 ? i1 + 1 : 0;
      final p = vertices[i1];
      final e = Vector2.copy(vertices[i2])..sub(p);

      for (var j = 0; j < vertices.length; ++j) {
        if (j == i1 || j == i2) {
          continue;
        }

        final v = Vector2.copy(vertices[j])..sub(p);
        final c = e.cross(v);
        if (c < 0.0) {
          return false;
        }
      }
    }

    return true;
  }

  /// Get the centroid and apply the supplied transform.
  Vector2 applyToCentroid(Transform xf) {
    return Transform.mulVec2(xf, centroid);
  }
}
