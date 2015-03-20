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
 * A circle shape.
 */
class CircleShape extends Shape {
  final Vector2 m_p = new Vector2.zero();

  CircleShape() : super(ShapeType.CIRCLE) {
    radius = 0.0;
  }

  Shape clone() {
    CircleShape shape = new CircleShape();
    shape.m_p.x = m_p.x;
    shape.m_p.y = m_p.y;
    shape.radius = radius;
    return shape;
  }

  int getChildCount() => 1;

  /**
   * Get the supporting vertex index in the given direction.
   * 
   * @param d
   * @return
   */
  int getSupport(final Vector2 d) => 0;

  /**
   * Get the supporting vertex in the given direction.
   * 
   * @param d
   * @return
   */
  Vector2 getSupportVertex(final Vector2 d) => m_p;

  /**
   * Get the vertex count.
   * 
   * @return
   */
  int getVertexCount() => 1;

  /**
   * Get a vertex by index.
   * 
   * @param index
   * @return
   */
  Vector2 getVertex(final int index) {
    assert(index == 0);
    return m_p;
  }

  bool testPoint(final Transform transform, final Vector2 p) {
    // Rot.mulToOutUnsafe(transform.q, m_p, center);
    // center.addLocal(transform.p);
    //
    // final Vec2 d = center.subLocal(p).negateLocal();
    // return Vec2.dot(d, d) <= m_radius * m_radius;
    final Rot q = transform.q;
    final Vector2 tp = transform.p;
    double centerx = -(q.c * m_p.x - q.s * m_p.y + tp.x - p.x);
    double centery = -(q.s * m_p.x + q.c * m_p.y + tp.y - p.y);

    return centerx * centerx + centery * centery <= radius * radius;
  }

  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    final Rot xfq = xf.q;
    double centerx = xfq.c * m_p.x - xfq.s * m_p.y + xf.p.x;
    double centery = xfq.s * m_p.x + xfq.c * m_p.y + xf.p.y;
    double dx = p.x - centerx;
    double dy = p.y - centery;
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

    // Rot.mulToOutUnsafe(transform.q, m_p, position);
    // position.addLocal(transform.p);
    final double positionx = tq.c * m_p.x - tq.s * m_p.y + tp.x;
    final double positiony = tq.s * m_p.x + tq.c * m_p.y + tp.y;

    final double sx = inputp1.x - positionx;
    final double sy = inputp1.y - positiony;
    // final double b = Vec2.dot(s, s) - m_radius * m_radius;
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
    final double px = tq.c * m_p.x - tq.s * m_p.y + tp.x;
    final double py = tq.s * m_p.x + tq.c * m_p.y + tp.y;

    aabb.lowerBound.x = px - radius;
    aabb.lowerBound.y = py - radius;
    aabb.upperBound.x = px + radius;
    aabb.upperBound.y = py + radius;
  }

  void computeMass(final MassData massData, final double density) {
    massData.mass = density * Settings.PI * radius * radius;
    massData.center.x = m_p.x;
    massData.center.y = m_p.y;

    // inertia about the local origin
    // massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_p, m_p));
    massData.I = massData.mass *
        (0.5 * radius * radius + (m_p.x * m_p.x + m_p.y * m_p.y));
  }
}
