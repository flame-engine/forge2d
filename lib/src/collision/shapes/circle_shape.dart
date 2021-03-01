import 'dart:math';

import '../../../forge2d.dart';
import '../../settings.dart' as settings;

/// A circle shape.
class CircleShape extends Shape {
  final Vector2 position = Vector2.zero();

  CircleShape() : super(ShapeType.CIRCLE) {
    radius = 0.0;
  }

  @override
  Shape clone() {
    final CircleShape shape = CircleShape();
    shape.position.x = position.x;
    shape.position.y = position.y;
    shape.radius = radius;
    return shape;
  }

  @override
  int getChildCount() => 1;

  /// Get the supporting vertex index in the given direction.
  int getSupport(final Vector2 d) => 0;

  /// Get the supporting vertex in the given direction.
  Vector2 getSupportVertex(final Vector2 d) => position;

  /// Get the vertex count.
  int getVertexCount() => 1;

  /// Get a vertex by index.
  Vector2 getVertex(final int index) {
    assert(index == 0);
    return position;
  }

  @override
  bool testPoint(final Transform transform, final Vector2 point) {
    final Rot q = transform.q;
    final Vector2 tp = transform.p;
    final double centerX =
        -(q.c * position.x - q.s * position.y + tp.x - point.x);
    final double centerY =
        -(q.s * position.x + q.c * position.y + tp.y - point.y);

    return centerX * centerX + centerY * centerY <= radius * radius;
  }

  @override
  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    final Rot xfq = xf.q;
    final double centerX = xfq.c * p.x - xfq.s * p.y + xf.p.x;
    final double centerY = xfq.s * p.x + xfq.c * p.y + xf.p.y;
    final double dx = p.x - centerX;
    final double dy = p.y - centerY;
    final double d1 = sqrt(dx * dx + dy * dy);
    normalOut.x = dx * 1 / d1;
    normalOut.y = dy * 1 / d1;
    return d1 - radius;
  }

  // Collision Detection in Interactive 3D Environments by Gino van den Bergen
  // From Section 3.1.2
  // x = s + a * r
  // norm(x) = radius
  @override
  bool raycast(RayCastOutput output, RayCastInput input, Transform transform,
      int childIndex) {
    final Vector2 inputP1 = input.p1;
    final Vector2 inputP2 = input.p2;
    final Rot tq = transform.q;
    final Vector2 tp = transform.p;

    final double positionX = tq.c * position.x - tq.s * position.y + tp.x;
    final double positionY = tq.s * position.x + tq.c * position.y + tp.y;

    final double sx = inputP1.x - positionX;
    final double sy = inputP1.y - positionY;
    final double b = sx * sx + sy * sy - radius * radius;

    // Solve quadratic equation.
    final double rx = inputP2.x - inputP1.x;
    final double ry = inputP2.y - inputP1.y;
    final double c = sx * rx + sy * ry;
    final double rr = rx * rx + ry * ry;
    final double sigma = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < 0.0 || rr < settings.EPSILON) {
      return false;
    }

    // Find the point of intersection of the line with the circle.
    double a = -(c + sqrt(sigma));

    // Is the intersection point on the segment?
    if (0.0 <= a && a <= input.maxFraction * rr) {
      a /= rr;
      output.fraction = a;
      output.normal.x = rx * a + sx;
      output.normal.y = ry * a + sy;
      output.normal.normalize();
      return true;
    }

    return false;
  }

  @override
  void computeAABB(final AABB aabb, final Transform transform, int childIndex) {
    final Rot tq = transform.q;
    final Vector2 tp = transform.p;
    final double px = tq.c * position.x - tq.s * position.y + tp.x;
    final double py = tq.s * position.x + tq.c * position.y + tp.y;

    aabb.lowerBound.x = px - radius;
    aabb.lowerBound.y = py - radius;
    aabb.upperBound.x = px + radius;
    aabb.upperBound.y = py + radius;
  }

  @override
  void computeMass(final MassData massData, final double density) {
    massData.mass = density * pi * radius * radius;
    massData.center.x = position.x;
    massData.center.y = position.y;

    // inertia about the local origin
    massData.I = massData.mass *
        (0.5 * radius * radius +
            (position.x * position.x + position.y * position.y));
  }
}
