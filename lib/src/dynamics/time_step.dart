part of box2d;

/// This is an internal structure.
class TimeStep {
  /// time step
  double dt = 0.0;

  /// inverse time step (0 if dt == 0).
  double inv_dt = 0.0;

  /// dt * inv_dt0
  double dtRatio = 0.0;

  int velocityIterations = 0;

  int positionIterations = 0;

  bool warmStarting = false;
}
