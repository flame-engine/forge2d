part of box2d;

/// Mouse joint definition. This requires a world target point, tuning parameters, and the time step.
class MouseJointDef extends JointDef {
  /// The initial world target point. This is assumed to coincide with the body anchor initially.
  final Vector2 target = Vector2.zero();

  /// The maximum constraint force that can be exerted to move the candidate body. Usually you will
  /// express as some multiple of the weight (multiplier * mass * gravity).
  double maxForce = 0.0;

  /// The response speed.
  double frequencyHz = 5.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = .7;

  MouseJointDef() : super(JointType.MOUSE);
}
