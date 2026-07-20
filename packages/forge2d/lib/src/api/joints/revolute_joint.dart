import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [RevoluteJoint].
///
/// Mirrors the native `b2RevoluteJointDef`, with the native defaults.
class RevoluteJointDef extends JointDef {
  /// Creates a revolute joint definition.
  RevoluteJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? localAnchorA,
    Vector2? localAnchorB,
    this.referenceAngle = 0,
    this.targetAngle = 0,
    this.enableSpring = false,
    this.hertz = 0,
    this.dampingRatio = 0,
    this.enableLimit = false,
    this.lowerAngle = 0,
    this.upperAngle = 0,
    this.enableMotor = false,
    this.maxMotorTorque = 0,
    this.motorSpeed = 0,
    this.drawSize = 0.25,
    super.collideConnected,
    super.userData,
  }) : localAnchorA = localAnchorA ?? Vector2.zero(),
       localAnchorB = localAnchorB ?? Vector2.zero();

  /// The anchor point on the first body, in its local coordinates.
  Vector2 localAnchorA;

  /// The anchor point on the second body, in its local coordinates.
  Vector2 localAnchorB;

  /// The angle of body B relative to body A that counts as zero, in
  /// radians.
  double referenceAngle;

  /// The target angle for the spring, in radians.
  double targetAngle;

  /// Whether the spring is enabled.
  bool enableSpring;

  /// The spring stiffness in hertz.
  double hertz;

  /// The spring damping ratio, non-dimensional.
  double dampingRatio;

  /// Whether the angle limit is enabled.
  bool enableLimit;

  /// The lower angle limit relative to the reference angle, in radians.
  double lowerAngle;

  /// The upper angle limit relative to the reference angle, in radians.
  double upperAngle;

  /// Whether the motor is enabled.
  bool enableMotor;

  /// The maximum motor torque, in newton-meters.
  double maxMotorTorque;

  /// The motor speed, in radians per second.
  double motorSpeed;

  /// The debug-draw size of the joint, in meters.
  double drawSize;
}

/// Allows relative rotation of two bodies around a point, like a hinge or a
/// pin, with an optional limit, motor, and spring.
final class RevoluteJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const RevoluteJoint.internal(
    super.world,
    super.index1,
    super.worldAndGeneration,
  ) : super.internal();

  /// The current joint angle relative to the reference angle, in radians.
  double get angle =>
      rawBox2D.revoluteJointGetAngle(index1, worldAndGeneration);

  /// Whether the spring is enabled.
  bool get springEnabled =>
      rawBox2D.revoluteJointIsSpringEnabled(index1, worldAndGeneration);

  set springEnabled(bool value) => rawBox2D.revoluteJointEnableSpring(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The spring stiffness in hertz.
  double get springHertz =>
      rawBox2D.revoluteJointGetSpringHertz(index1, worldAndGeneration);

  set springHertz(double value) =>
      rawBox2D.revoluteJointSetSpringHertz(index1, worldAndGeneration, value);

  /// The spring damping ratio.
  double get springDampingRatio =>
      rawBox2D.revoluteJointGetSpringDampingRatio(index1, worldAndGeneration);

  set springDampingRatio(double value) => rawBox2D
      .revoluteJointSetSpringDampingRatio(index1, worldAndGeneration, value);

  /// The target angle for the spring, in radians.
  double get targetAngle =>
      rawBox2D.revoluteJointGetTargetAngle(index1, worldAndGeneration);

  set targetAngle(double value) =>
      rawBox2D.revoluteJointSetTargetAngle(index1, worldAndGeneration, value);

  /// Whether the angle limit is enabled.
  bool get limitEnabled =>
      rawBox2D.revoluteJointIsLimitEnabled(index1, worldAndGeneration);

  set limitEnabled(bool value) => rawBox2D.revoluteJointEnableLimit(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The lower angle limit, in radians.
  double get lowerLimit =>
      rawBox2D.revoluteJointGetLowerLimit(index1, worldAndGeneration);

  /// The upper angle limit, in radians.
  double get upperLimit =>
      rawBox2D.revoluteJointGetUpperLimit(index1, worldAndGeneration);

  /// Sets the angle limits, in radians.
  void setLimits({required double lower, required double upper}) =>
      rawBox2D.revoluteJointSetLimits(index1, worldAndGeneration, lower, upper);

  /// Whether the motor is enabled.
  bool get motorEnabled =>
      rawBox2D.revoluteJointIsMotorEnabled(index1, worldAndGeneration);

  set motorEnabled(bool value) => rawBox2D.revoluteJointEnableMotor(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The motor speed, in radians per second.
  double get motorSpeed =>
      rawBox2D.revoluteJointGetMotorSpeed(index1, worldAndGeneration);

  set motorSpeed(double value) =>
      rawBox2D.revoluteJointSetMotorSpeed(index1, worldAndGeneration, value);

  /// The maximum motor torque, in newton-meters.
  double get maxMotorTorque =>
      rawBox2D.revoluteJointGetMaxMotorTorque(index1, worldAndGeneration);

  set maxMotorTorque(double value) => rawBox2D.revoluteJointSetMaxMotorTorque(
    index1,
    worldAndGeneration,
    value,
  );

  /// The current motor torque, in newton-meters.
  double get motorTorque =>
      rawBox2D.revoluteJointGetMotorTorque(index1, worldAndGeneration);
}
