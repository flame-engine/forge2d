import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [WheelJoint].
///
/// Mirrors the native `b2WheelJointDef`, with the native defaults.
class WheelJointDef extends JointDef {
  /// Creates a wheel joint definition.
  WheelJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? localAnchorA,
    Vector2? localAnchorB,
    Vector2? localAxisA,
    this.enableSpring = true,
    this.hertz = 1,
    this.dampingRatio = 0.7,
    this.enableLimit = false,
    this.lowerTranslation = 0,
    this.upperTranslation = 0,
    this.enableMotor = false,
    this.maxMotorTorque = 0,
    this.motorSpeed = 0,
    super.collideConnected,
    super.userData,
  }) : localAnchorA = localAnchorA ?? Vector2.zero(),
       localAnchorB = localAnchorB ?? Vector2.zero(),
       localAxisA = localAxisA ?? Vector2(0, 1);

  /// The anchor point on the first body, in its local coordinates.
  Vector2 localAnchorA;

  /// The anchor point on the second body, in its local coordinates.
  Vector2 localAnchorB;

  /// The suspension axis, in the first body's local coordinates.
  Vector2 localAxisA;

  /// Whether the suspension spring is enabled.
  bool enableSpring;

  /// The spring stiffness in hertz.
  double hertz;

  /// The spring damping ratio, non-dimensional.
  double dampingRatio;

  /// Whether the translation limit is enabled.
  bool enableLimit;

  /// The lower translation limit, in meters.
  double lowerTranslation;

  /// The upper translation limit, in meters.
  double upperTranslation;

  /// Whether the motor is enabled.
  bool enableMotor;

  /// The maximum motor torque, in newton-meters.
  double maxMotorTorque;

  /// The motor speed, in radians per second.
  double motorSpeed;
}

/// Provides wheel suspension: body B may rotate freely around its anchor
/// while translating along an axis on body A with a spring and optional
/// limit and motor.
final class WheelJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const WheelJoint.internal(super.world, super.index1, super.wg)
    : super.internal();

  /// Whether the suspension spring is enabled.
  bool get springEnabled => rawBox2D.wheelJointIsSpringEnabled(index1, wg);

  set springEnabled(bool value) =>
      rawBox2D.wheelJointEnableSpring(index1, wg, enabled: value);

  /// The spring stiffness in hertz.
  double get springHertz => rawBox2D.wheelJointGetSpringHertz(index1, wg);

  set springHertz(double value) =>
      rawBox2D.wheelJointSetSpringHertz(index1, wg, value);

  /// The spring damping ratio.
  double get springDampingRatio =>
      rawBox2D.wheelJointGetSpringDampingRatio(index1, wg);

  set springDampingRatio(double value) =>
      rawBox2D.wheelJointSetSpringDampingRatio(index1, wg, value);

  /// Whether the translation limit is enabled.
  bool get limitEnabled => rawBox2D.wheelJointIsLimitEnabled(index1, wg);

  set limitEnabled(bool value) =>
      rawBox2D.wheelJointEnableLimit(index1, wg, enabled: value);

  /// The lower translation limit, in meters.
  double get lowerLimit => rawBox2D.wheelJointGetLowerLimit(index1, wg);

  /// The upper translation limit, in meters.
  double get upperLimit => rawBox2D.wheelJointGetUpperLimit(index1, wg);

  /// Sets the translation limits, in meters.
  void setLimits({required double lower, required double upper}) =>
      rawBox2D.wheelJointSetLimits(index1, wg, lower, upper);

  /// Whether the motor is enabled.
  bool get motorEnabled => rawBox2D.wheelJointIsMotorEnabled(index1, wg);

  set motorEnabled(bool value) =>
      rawBox2D.wheelJointEnableMotor(index1, wg, enabled: value);

  /// The motor speed, in radians per second.
  double get motorSpeed => rawBox2D.wheelJointGetMotorSpeed(index1, wg);

  set motorSpeed(double value) =>
      rawBox2D.wheelJointSetMotorSpeed(index1, wg, value);

  /// The maximum motor torque, in newton-meters.
  double get maxMotorTorque => rawBox2D.wheelJointGetMaxMotorTorque(index1, wg);

  set maxMotorTorque(double value) =>
      rawBox2D.wheelJointSetMaxMotorTorque(index1, wg, value);

  /// The current motor torque, in newton-meters.
  double get motorTorque => rawBox2D.wheelJointGetMotorTorque(index1, wg);
}
