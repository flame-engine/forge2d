part of forge2d;

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

  EdgeShape() : super(ShapeType.EDGE) {
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
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    final double xfqc = xf.q.c;
    final double xfqs = xf.q.s;
    final double xfpx = xf.p.x;
    final double xfpy = xf.p.y;
    final double v1x = (xfqc * vertex1.x - xfqs * vertex1.y) + xfpx;
    final double v1y = (xfqs * vertex1.x + xfqc * vertex1.y) + xfpy;
    final double v2x = (xfqc * vertex2.x - xfqs * vertex2.y) + xfpx;
    final double v2y = (xfqs * vertex2.x + xfqc * vertex2.y) + xfpy;

    double dx = p.x - v1x;
    double dy = p.y - v1y;
    final double sx = v2x - v1x;
    final double sy = v2y - v1y;
    final double ds = dx * sx + dy * sy;
    if (ds > 0) {
      final double s2 = sx * sx + sy * sy;
      if (ds > s2) {
        dx = p.x - v2x;
        dy = p.y - v2y;
      } else {
        dx -= ds / s2 * sx;
        dy -= ds / s2 * sy;
      }
    }

    final double d1 = math.sqrt(dx * dx + dy * dy);
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
      RayCastOutput output, RayCastInput input, Transform xf, int childIndex) {
    final Vector2 v1 = vertex1;
    final Vector2 v2 = vertex2;
    final Rot xfq = xf.q;
    final Vector2 xfp = xf.p;

    // Put the ray into the edge's frame of reference.
    double tempX = input.p1.x - xfp.x;
    double tempY = input.p1.y - xfp.y;
    final double p1x = xfq.c * tempX + xfq.s * tempY;
    final double p1y = -xfq.s * tempX + xfq.c * tempY;

    tempX = input.p2.x - xfp.x;
    tempY = input.p2.y - xfp.y;
    final double p2x = xfq.c * tempX + xfq.s * tempY;
    final double p2y = -xfq.s * tempX + xfq.c * tempY;

    final double dx = p2x - p1x;
    final double dy = p2y - p1y;

    final Vector2 normal = Vector2(v2.y - v1.y, v1.x - v2.x);
    normal.normalize();

    tempX = v1.x - p1x;
    tempY = v1.y - p1y;
    final double numerator = normal.x * tempX + normal.y * tempY;
    final double denominator = normal.x * dx + normal.y * dy;

    if (denominator == 0.0) {
      return false;
    }

    final double t = numerator / denominator;
    if (t < 0.0 || 1.0 < t) {
      return false;
    }

    final double qx = p1x + t * dx;
    final double qy = p1y + t * dy;

    final double rx = v2.x - v1.x;
    final double ry = v2.y - v1.y;
    final double rr = rx * rx + ry * ry;
    if (rr == 0.0) {
      return false;
    }
    tempX = qx - v1.x;
    tempY = qy - v1.y;
    final double s = (tempX * rx + tempY * ry) / rr;
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
    final Vector2 lowerBound = aabb.lowerBound;
    final Vector2 upperBound = aabb.upperBound;
    final Rot xfq = xf.q;

    final double v1x = (xfq.c * vertex1.x - xfq.s * vertex1.y) + xf.p.x;
    final double v1y = (xfq.s * vertex1.x + xfq.c * vertex1.y) + xf.p.y;
    final double v2x = (xfq.c * vertex2.x - xfq.s * vertex2.y) + xf.p.x;
    final double v2y = (xfq.s * vertex2.x + xfq.c * vertex2.y) + xf.p.y;

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
    final EdgeShape edge = EdgeShape();
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
