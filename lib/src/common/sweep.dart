part of box2d.common;

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to
/// the body origin, which may not coincide with the center of mass. However, to support dynamics we
/// must interpolate the center of mass position.
class Sweep {
  /// Local center of mass position
  final Vector2 localCenter = Vector2.zero();

  /// Center world positions
  final Vector2 c0 = Vector2.zero(), c = Vector2.zero();

  /// World angles
  double a0 = 0.0, a = 0.0;

  /// Fraction of the current time step in the range [0,1] c0 and a0 are the positions at alpha0.
  double alpha0 = 0.0;

  String toString() {
    String s = "Sweep:\nlocalCenter: $localCenter\n";
    s += "c0: $c0, c: $c\n";
    s += "a0: $a0, a: $a\n";
    s += "alpha0: $alpha0";
    return s;
  }

  void normalize() {
    double d = MathUtils.TWOPI * (a0 / MathUtils.TWOPI).floor();
    a0 -= d;
    a -= d;
  }

  Sweep set(Sweep other) {
    localCenter.setFrom(other.localCenter);
    c0.setFrom(other.c0);
    c.setFrom(other.c);
    a0 = other.a0;
    a = other.a;
    alpha0 = other.alpha0;
    return this;
  }

  /// Get the interpolated transform at a specific time.
  ///
  /// @param xf the result is placed here - must not be null
  /// @param t the normalized time in [0,1].
  void getTransform(final Transform xf, final double beta) {
    assert(xf != null);
    // xf->p = (1.0f - beta) * c0 + beta * c;
    // float32 angle = (1.0f - beta) * a0 + beta * a;
    // xf->q.Set(angle);
    xf.p.x = (1.0 - beta) * c0.x + beta * c.x;
    xf.p.y = (1.0 - beta) * c0.y + beta * c.y;
    double angle = (1.0 - beta) * a0 + beta * a;
    xf.q.setAngle(angle);

    // Shift to origin
    // xf->p -= b2Mul(xf->q, localCenter);
    final Rot q = xf.q;
    xf.p.x -= q.c * localCenter.x - q.s * localCenter.y;
    xf.p.y -= q.s * localCenter.x + q.c * localCenter.y;
  }

  /// Advance the sweep forward, yielding a new initial state.
  ///
  /// @param alpha the new initial time.
  void advance(double alpha) {
    assert(alpha0 < 1.0);
    // float32 beta = (alpha - alpha0) / (1.0f - alpha0);
    // c0 += beta * (c - c0);
    // a0 += beta * (a - a0);
    // alpha0 = alpha;
    double beta = (alpha - alpha0) / (1.0 - alpha0);
    c0.x += beta * (c.x - c0.x);
    c0.y += beta * (c.y - c0.y);
    a0 += beta * (a - a0);
    alpha0 = alpha;
  }
}
