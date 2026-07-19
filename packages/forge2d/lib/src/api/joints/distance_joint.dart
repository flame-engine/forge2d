import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [DistanceJoint].
///
/// Mirrors the native `b2DistanceJointDef`, with the native defaults.
class DistanceJointDef extends JointDef {
  /// Creates a distance joint definition.
  DistanceJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? localAnchorA,
    Vector2? localAnchorB,
    this.length = 1,
    this.enableSpring = false,
    this.hertz = 0,
    this.dampingRatio = 0,
    this.enableLimit = false,
    this.minLength = 0,
    this.maxLength = huge,
    this.enableMotor = false,
    this.maxMotorForce = 0,
    this.motorSpeed = 0,
    super.collideConnected,
    super.userData,
  }) : localAnchorA = localAnchorA ?? Vector2.zero(),
       localAnchorB = localAnchorB ?? Vector2.zero();

  /// The native `B2_HUGE` constant, the default maximum length.
  static const double huge = 100000;

  /// The anchor point on the first body, in its local coordinates.
  Vector2 localAnchorA;

  /// The anchor point on the second body, in its local coordinates.
  Vector2 localAnchorB;

  /// The rest length of the joint, clamped to a stable minimum.
  double length;

  /// Whether the spring is enabled; when false the joint is rigid.
  bool enableSpring;

  /// The spring stiffness in hertz.
  double hertz;

  /// The spring damping ratio, non-dimensional.
  double dampingRatio;

  /// Whether the length limit is enabled; only used with the spring.
  bool enableLimit;

  /// The minimum length of the joint.
  double minLength;

  /// The maximum length of the joint.
  double maxLength;

  /// Whether the motor is enabled.
  bool enableMotor;

  /// The maximum motor force, in newtons.
  double maxMotorForce;

  /// The motor speed, in meters per second.
  double motorSpeed;
}

/// Keeps two points on two bodies at a fixed distance apart, optionally
/// softened into a spring with limits and a motor.
final class DistanceJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const DistanceJoint.internal(
    super.world,
    super.index1,
    super.worldAndGeneration,
  ) : super.internal();

  /// The rest length of the joint.
  double get length =>
      rawBox2D.distanceJointGetLength(index1, worldAndGeneration);

  set length(double value) =>
      rawBox2D.distanceJointSetLength(index1, worldAndGeneration, value);

  /// The current distance between the anchor points.
  double get currentLength =>
      rawBox2D.distanceJointGetCurrentLength(index1, worldAndGeneration);

  /// Whether the spring is enabled; when false the joint is rigid.
  bool get springEnabled =>
      rawBox2D.distanceJointIsSpringEnabled(index1, worldAndGeneration);

  set springEnabled(bool value) => rawBox2D.distanceJointEnableSpring(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The spring stiffness in hertz.
  double get springHertz =>
      rawBox2D.distanceJointGetSpringHertz(index1, worldAndGeneration);

  set springHertz(double value) =>
      rawBox2D.distanceJointSetSpringHertz(index1, worldAndGeneration, value);

  /// The spring damping ratio.
  double get springDampingRatio =>
      rawBox2D.distanceJointGetSpringDampingRatio(index1, worldAndGeneration);

  set springDampingRatio(double value) => rawBox2D
      .distanceJointSetSpringDampingRatio(index1, worldAndGeneration, value);

  /// Whether the length limit is enabled.
  bool get limitEnabled =>
      rawBox2D.distanceJointIsLimitEnabled(index1, worldAndGeneration);

  set limitEnabled(bool value) => rawBox2D.distanceJointEnableLimit(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The minimum length of the joint.
  double get minLength =>
      rawBox2D.distanceJointGetMinLength(index1, worldAndGeneration);

  /// The maximum length of the joint.
  double get maxLength =>
      rawBox2D.distanceJointGetMaxLength(index1, worldAndGeneration);

  /// Sets the length limit range.
  void setLengthRange({required double min, required double max}) => rawBox2D
      .distanceJointSetLengthRange(index1, worldAndGeneration, min, max);

  /// Whether the motor is enabled.
  bool get motorEnabled =>
      rawBox2D.distanceJointIsMotorEnabled(index1, worldAndGeneration);

  set motorEnabled(bool value) => rawBox2D.distanceJointEnableMotor(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The motor speed, in meters per second.
  double get motorSpeed =>
      rawBox2D.distanceJointGetMotorSpeed(index1, worldAndGeneration);

  set motorSpeed(double value) =>
      rawBox2D.distanceJointSetMotorSpeed(index1, worldAndGeneration, value);

  /// The maximum motor force, in newtons.
  double get maxMotorForce =>
      rawBox2D.distanceJointGetMaxMotorForce(index1, worldAndGeneration);

  set maxMotorForce(double value) =>
      rawBox2D.distanceJointSetMaxMotorForce(index1, worldAndGeneration, value);

  /// The current motor force, in newtons.
  double get motorForce =>
      rawBox2D.distanceJointGetMotorForce(index1, worldAndGeneration);
}
