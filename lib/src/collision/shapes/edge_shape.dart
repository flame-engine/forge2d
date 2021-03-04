import 'dart:math';

import '../../../forge2d.dart';
import '../../settings.dart' as settings;

/// A line segment (edge) shape. These can be connected in chains or loops to other edge shapes. The
/// connectivity information is used to ensure correct contact normals.
class EdgeShape extends Shape {
  /// edge vertex 1
  final Vector2 vertex1 = Vector2.zero();

  /// edge vertex 2
  final Vector2 vertex2 = Vector2.zero();

  /// optional adjacent vertex 1. Used for smooth collision
  final Vector2 vertex0 = Vector2.zero();

  /// optional adjacent vertex 2. Used for smooth collision
  final Vector2 vertex3 = Vector2.zero();
  bool hasVertex0 = false;
  bool hasVertex3 = false;

  EdgeShape() : super(ShapeType.edge) {
    radius = settings.polygonRadius;
  }

  @override
  int getChildCount() {
    return 1;
  }

  void set(Vector2 v1, Vector2 v2) {
    vertex1.setFrom(v1);
    vertex2.setFrom(v2);
    hasVertex0 = hasVertex3 = false;
  }

  @override
  bool testPoint(Transform xf, Vector2 p) {
    return false;
  }

  @override
  double computeDistanceToOut(
    Transform xf,
    Vector2 p,
    int childIndex,
    Vector2 normalOut,
  ) {
    final xfqc = xf.q.c;
    final xfqs = xf.q.s;
    final xfpx = xf.p.x;
    final xfpy = xf.p.y;
    final v1x = (xfqc * vertex1.x - xfqs * vertex1.y) + xfpx;
    final v1y = (xfqs * vertex1.x + xfqc * vertex1.y) + xfpy;
    final v2x = (xfqc * vertex2.x - xfqs * vertex2.y) + xfpx;
    final v2y = (xfqs * vertex2.x + xfqc * vertex2.y) + xfpy;

    var dx = p.x - v1x;
    var dy = p.y - v1y;
    final sx = v2x - v1x;
    final sy = v2y - v1y;
    final ds = dx * sx + dy * sy;
    if (ds > 0) {
      final s2 = sx * sx + sy * sy;
      if (ds > s2) {
        dx = p.x - v2x;
        dy = p.y - v2y;
      } else {
        dx -= ds / s2 * sx;
        dy -= ds / s2 * sy;
      }
    }

    final d1 = sqrt(dx * dx + dy * dy);
    if (d1 > 0) {
      normalOut.x = 1 / d1 * dx;
      normalOut.y = 1 / d1 * dy;
    } else {
      normalOut.x = 0.0;
      normalOut.y = 0.0;
    }
    return d1;
  }

  @override
  bool raycast(
    RayCastOutput output,
    RayCastInput input,
    Transform xf,
    int childIndex,
  ) {
    final v1 = vertex1;
    final v2 = vertex2;
    final xfq = xf.q;
    final xfp = xf.p;

    // Put the ray into the edge's frame of reference.
    var tempX = input.p1.x - xfp.x;
    var tempY = input.p1.y - xfp.y;
    final p1x = xfq.c * tempX + xfq.s * tempY;
    final p1y = -xfq.s * tempX + xfq.c * tempY;

    tempX = input.p2.x - xfp.x;
    tempY = input.p2.y - xfp.y;
    final p2x = xfq.c * tempX + xfq.s * tempY;
    final p2y = -xfq.s * tempX + xfq.c * tempY;

    final dx = p2x - p1x;
    final dy = p2y - p1y;

    final normal = Vector2(v2.y - v1.y, v1.x - v2.x);
    normal.normalize();

    tempX = v1.x - p1x;
    tempY = v1.y - p1y;
    final numerator = normal.x * tempX + normal.y * tempY;
    final denominator = normal.x * dx + normal.y * dy;

    if (denominator == 0.0) {
      return false;
    }

    final t = numerator / denominator;
    if (t < 0.0 || 1.0 < t) {
      return false;
    }

    final qx = p1x + t * dx;
    final qy = p1y + t * dy;

    final rx = v2.x - v1.x;
    final ry = v2.y - v1.y;
    final rr = rx * rx + ry * ry;
    if (rr == 0.0) {
      return false;
    }
    tempX = qx - v1.x;
    tempY = qy - v1.y;
    final s = (tempX * rx + tempY * ry) / rr;
    if (s < 0.0 || 1.0 < s) {
      return false;
    }

    output.fraction = t;
    if (numerator > 0.0) {
      output.normal.x = -xfq.c * normal.x + xfq.s * normal.y;
      output.normal.y = -xfq.s * normal.x - xfq.c * normal.y;
    } else {
      output.normal.x = xfq.c * normal.x - xfq.s * normal.y;
      output.normal.y = xfq.s * normal.x + xfq.c * normal.y;
    }
    return true;
  }

  @override
  void computeAABB(AABB aabb, Transform xf, int childIndex) {
    final lowerBound = aabb.lowerBound;
    final upperBound = aabb.upperBound;
    final xfq = xf.q;

    final v1x = (xfq.c * vertex1.x - xfq.s * vertex1.y) + xf.p.x;
    final v1y = (xfq.s * vertex1.x + xfq.c * vertex1.y) + xf.p.y;
    final v2x = (xfq.c * vertex2.x - xfq.s * vertex2.y) + xf.p.x;
    final v2y = (xfq.s * vertex2.x + xfq.c * vertex2.y) + xf.p.y;

    lowerBound.x = v1x < v2x ? v1x : v2x;
    lowerBound.y = v1y < v2y ? v1y : v2y;
    upperBound.x = v1x > v2x ? v1x : v2x;
    upperBound.y = v1y > v2y ? v1y : v2y;

    lowerBound.x -= radius;
    lowerBound.y -= radius;
    upperBound.x += radius;
    upperBound.y += radius;
  }

  @override
  void computeMass(MassData massData, double density) {
    massData.mass = 0.0;
    massData.center
      ..setFrom(vertex1)
      ..add(vertex2)
      ..scale(0.5);
    massData.I = 0.0;
  }

  @override
  Shape clone() {
    final edge = EdgeShape();
    edge.radius = radius;
    edge.hasVertex0 = hasVertex0;
    edge.hasVertex3 = hasVertex3;
    edge.vertex0.setFrom(vertex0);
    edge.vertex1.setFrom(vertex1);
    edge.vertex2.setFrom(vertex2);
    edge.vertex3.setFrom(vertex3);
    return edge;
  }
}
