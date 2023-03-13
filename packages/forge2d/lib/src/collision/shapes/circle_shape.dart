import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// A circle shape.
class CircleShape extends Shape {
  final Vector2 position = Vector2.zero();

  CircleShape() : super(ShapeType.circle) {
    radius = 0.0;
  }

  @override
  Shape clone() {
    final shape = CircleShape();
    shape.position.x = position.x;
    shape.position.y = position.y;
    shape.radius = radius;
    return shape;
  }

  @override
  int get childCount => 1;

  /// Get the supporting vertex index in the given direction.
  int getSupport(Vector2 d) => 0;

  /// Get the supporting vertex in the given direction.
  Vector2 getSupportVertex(Vector2 d) => position;

  /// Get the vertex count.
  int getVertexCount() => 1;

  /// Get a vertex by index.
  Vector2 getVertex(int index) {
    assert(index == 0);
    return position;
  }

  @override
  bool testPoint(Transform transform, Vector2 point) {
    final q = transform.q;
    final tp = transform.p;
    final centerX = -(q.cos * position.x - q.sin * position.y + tp.x - point.x);
    final centerY = -(q.sin * position.x + q.cos * position.y + tp.y - point.y);

    return centerX * centerX + centerY * centerY <= radius * radius;
  }

  @override
  double computeDistanceToOut(
    Transform xf,
    Vector2 p,
    int childIndex,
    Vector2 normalOut,
  ) {
    final xfq = xf.q;
    final centerX = xfq.cos * p.x - xfq.sin * p.y + xf.p.x;
    final centerY = xfq.sin * p.x + xfq.cos * p.y + xf.p.y;
    final dx = p.x - centerX;
    final dy = p.y - centerY;
    final d1 = sqrt(dx * dx + dy * dy);
    normalOut.x = dx * 1 / d1;
    normalOut.y = dy * 1 / d1;
    return d1 - radius;
  }

  // Collision Detection in Interactive 3D Environments by Gino van den Bergen
  // From Section 3.1.2
  // x = s + a * r
  // norm(x) = radius
  @override
  bool raycast(
    RayCastOutput output,
    RayCastInput input,
    Transform transform,
    int childIndex,
  ) {
    final inputP1 = input.p1;
    final inputP2 = input.p2;
    final tq = transform.q;
    final tp = transform.p;

    final positionX = tq.cos * position.x - tq.sin * position.y + tp.x;
    final positionY = tq.sin * position.x + tq.cos * position.y + tp.y;

    final sx = inputP1.x - positionX;
    final sy = inputP1.y - positionY;
    final b = sx * sx + sy * sy - radius * radius;

    // Solve quadratic equation.
    final rx = inputP2.x - inputP1.x;
    final ry = inputP2.y - inputP1.y;
    final c = sx * rx + sy * ry;
    final rr = rx * rx + ry * ry;
    final sigma = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < 0.0 || rr < settings.epsilon) {
      return false;
    }

    // Find the point of intersection of the line with the circle.
    var a = -(c + sqrt(sigma));

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
  void computeAABB(AABB aabb, Transform transform, int childIndex) {
    final tq = transform.q;
    final tp = transform.p;
    final px = tq.cos * position.x - tq.sin * position.y + tp.x;
    final py = tq.sin * position.x + tq.cos * position.y + tp.y;

    aabb.lowerBound.x = px - radius;
    aabb.lowerBound.y = py - radius;
    aabb.upperBound.x = px + radius;
    aabb.upperBound.y = py + radius;
  }

  @override
  void computeMass(MassData massData, double density) {
    massData.mass = density * pi * radius * radius;
    massData.center.x = position.x;
    massData.center.y = position.y;

    // inertia about the local origin
    massData.I = massData.mass *
        (0.5 * radius * radius +
            (position.x * position.x + position.y * position.y));
  }
}
