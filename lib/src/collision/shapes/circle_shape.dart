part of box2d;

/// A circle shape.
class CircleShape extends Shape {
  final Vector2 position = Vector2.zero();

  CircleShape() : super(ShapeType.CIRCLE) {
    radius = 0.0;
  }

  Shape clone() {
    CircleShape shape = CircleShape();
    shape.position.x = position.x;
    shape.position.y = position.y;
    shape.radius = radius;
    return shape;
  }

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

  bool testPoint(final Transform transform, final Vector2 point) {
    final Rot q = transform.q;
    final Vector2 tp = transform.p;
    double centerX = -(q.c * position.x - q.s * position.y + tp.x - point.x);
    double centerY = -(q.s * position.x + q.c * position.y + tp.y - point.y);

    return centerX * centerX + centerY * centerY <= radius * radius;
  }

  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    final Rot xfq = xf.q;
    double centerX = xfq.c * p.x - xfq.s * p.y + xf.p.x;
    double centerY = xfq.s * p.x + xfq.c * p.y + xf.p.y;
    double dx = p.x - centerX;
    double dy = p.y - centerY;
    double d1 = Math.sqrt(dx * dx + dy * dy);
    normalOut.x = dx * 1 / d1;
    normalOut.y = dy * 1 / d1;
    return d1 - radius;
  }

  // Collision Detection in Interactive 3D Environments by Gino van den Bergen
  // From Section 3.1.2
  // x = s + a * r
  // norm(x) = radius
  bool raycast(RayCastOutput output, RayCastInput input, Transform transform,
      int childIndex) {
    final Vector2 inputp1 = input.p1;
    final Vector2 inputp2 = input.p2;
    final Rot tq = transform.q;
    final Vector2 tp = transform.p;

    // Rot.mulToOutUnsafe(transform.q, _p, position);
    // position.addLocal(transform.p);
    final double positionx = tq.c * position.x - tq.s * position.y + tp.x;
    final double positiony = tq.s * position.x + tq.c * position.y + tp.y;

    final double sx = inputp1.x - positionx;
    final double sy = inputp1.y - positiony;
    // final double b = Vec2.dot(s, s) - _radius * _radius;
    final double b = sx * sx + sy * sy - radius * radius;

    // Solve quadratic equation.
    final double rx = inputp2.x - inputp1.x;
    final double ry = inputp2.y - inputp1.y;
    // final double c = Vec2.dot(s, r);
    // final double rr = Vec2.dot(r, r);
    final double c = sx * rx + sy * ry;
    final double rr = rx * rx + ry * ry;
    final double sigma = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < 0.0 || rr < Settings.EPSILON) {
      return false;
    }

    // Find the point of intersection of the line with the circle.
    double a = -(c + Math.sqrt(sigma));

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

  void computeMass(final MassData massData, final double density) {
    massData.mass = density * Math.pi * radius * radius;
    massData.center.x = position.x;
    massData.center.y = position.y;

    // inertia about the local origin
    massData.I = massData.mass *
        (0.5 * radius * radius +
            (position.x * position.x + position.y * position.y));
  }
}
