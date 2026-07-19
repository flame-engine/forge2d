import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [PrismaticJoint].
///
/// Mirrors the native `b2PrismaticJointDef`, with the native defaults.
class PrismaticJointDef extends JointDef {
  /// Creates a prismatic joint definition.
  PrismaticJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? localAnchorA,
    Vector2? localAnchorB,
    Vector2? localAxisA,
    this.referenceAngle = 0,
    this.targetTranslation = 0,
    this.enableSpring = false,
    this.hertz = 0,
    this.dampingRatio = 0,
    this.enableLimit = false,
    this.lowerTranslation = 0,
    this.upperTranslation = 0,
    this.enableMotor = false,
    this.maxMotorForce = 0,
    this.motorSpeed = 0,
    super.collideConnected,
    super.userData,
  }) : localAnchorA = localAnchorA ?? Vector2.zero(),
       localAnchorB = localAnchorB ?? Vector2.zero(),
       localAxisA = localAxisA ?? Vector2(1, 0);

  /// The anchor point on the first body, in its local coordinates.
  Vector2 localAnchorA;

  /// The anchor point on the second body, in its local coordinates.
  Vector2 localAnchorB;

  /// The translation axis, in the first body's local coordinates.
  Vector2 localAxisA;

  /// The angle of body B relative to body A that counts as zero, in
  /// radians.
  double referenceAngle;

  /// The target translation for the spring, in meters.
  double targetTranslation;

  /// Whether the spring is enabled.
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

  /// The maximum motor force, in newtons.
  double maxMotorForce;

  /// The motor speed, in meters per second.
  double motorSpeed;
}

/// Allows relative translation of two bodies along an axis while preventing
/// relative rotation, like an elevator or a piston.
final class PrismaticJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const PrismaticJoint.internal(
    super.world,
    super.index1,
    super.worldAndGeneration,
  ) : super.internal();

  /// Whether the spring is enabled.
  bool get springEnabled =>
      rawBox2D.prismaticJointIsSpringEnabled(index1, worldAndGeneration);

  set springEnabled(bool value) => rawBox2D.prismaticJointEnableSpring(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The spring stiffness in hertz.
  double get springHertz =>
      rawBox2D.prismaticJointGetSpringHertz(index1, worldAndGeneration);

  set springHertz(double value) =>
      rawBox2D.prismaticJointSetSpringHertz(index1, worldAndGeneration, value);

  /// The spring damping ratio.
  double get springDampingRatio =>
      rawBox2D.prismaticJointGetSpringDampingRatio(index1, worldAndGeneration);

  set springDampingRatio(double value) => rawBox2D
      .prismaticJointSetSpringDampingRatio(index1, worldAndGeneration, value);

  /// The target translation for the spring, in meters.
  double get targetTranslation =>
      rawBox2D.prismaticJointGetTargetTranslation(index1, worldAndGeneration);

  set targetTranslation(double value) => rawBox2D
      .prismaticJointSetTargetTranslation(index1, worldAndGeneration, value);

  /// Whether the translation limit is enabled.
  bool get limitEnabled =>
      rawBox2D.prismaticJointIsLimitEnabled(index1, worldAndGeneration);

  set limitEnabled(bool value) => rawBox2D.prismaticJointEnableLimit(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The lower translation limit, in meters.
  double get lowerLimit =>
      rawBox2D.prismaticJointGetLowerLimit(index1, worldAndGeneration);

  /// The upper translation limit, in meters.
  double get upperLimit =>
      rawBox2D.prismaticJointGetUpperLimit(index1, worldAndGeneration);

  /// Sets the translation limits, in meters.
  void setLimits({required double lower, required double upper}) => rawBox2D
      .prismaticJointSetLimits(index1, worldAndGeneration, lower, upper);

  /// Whether the motor is enabled.
  bool get motorEnabled =>
      rawBox2D.prismaticJointIsMotorEnabled(index1, worldAndGeneration);

  set motorEnabled(bool value) => rawBox2D.prismaticJointEnableMotor(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// The motor speed, in meters per second.
  double get motorSpeed =>
      rawBox2D.prismaticJointGetMotorSpeed(index1, worldAndGeneration);

  set motorSpeed(double value) =>
      rawBox2D.prismaticJointSetMotorSpeed(index1, worldAndGeneration, value);

  /// The maximum motor force, in newtons.
  double get maxMotorForce =>
      rawBox2D.prismaticJointGetMaxMotorForce(index1, worldAndGeneration);

  set maxMotorForce(double value) => rawBox2D.prismaticJointSetMaxMotorForce(
    index1,
    worldAndGeneration,
    value,
  );

  /// The current motor force, in newtons.
  double get motorForce =>
      rawBox2D.prismaticJointGetMotorForce(index1, worldAndGeneration);

  /// The current translation along the joint axis, in meters.
  double get translation =>
      rawBox2D.prismaticJointGetTranslation(index1, worldAndGeneration);

  /// The current speed along the joint axis, in meters per second.
  double get speed =>
      rawBox2D.prismaticJointGetSpeed(index1, worldAndGeneration);
}
