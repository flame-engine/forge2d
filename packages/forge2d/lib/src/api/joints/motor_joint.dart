import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [MotorJoint].
///
/// Mirrors the native `b2MotorJointDef`, with the native defaults.
class MotorJointDef extends JointDef {
  /// Creates a motor joint definition.
  MotorJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? linearOffset,
    this.angularOffset = 0,
    this.maxForce = 1,
    this.maxTorque = 1,
    this.correctionFactor = 0.3,
    super.collideConnected,
    super.userData,
  }) : linearOffset = linearOffset ?? Vector2.zero();

  /// The target position of body B, in the frame of body A.
  Vector2 linearOffset;

  /// The target angle of body B relative to body A, in radians.
  double angularOffset;

  /// The maximum force the motor can apply, in newtons.
  double maxForce;

  /// The maximum torque the motor can apply, in newton-meters.
  double maxTorque;

  /// The position correction factor, in `[0, 1]`.
  double correctionFactor;
}

/// Drives the relative transform between two bodies towards a target,
/// useful for moving platforms and kinematic-style control of dynamic
/// bodies.
final class MotorJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const MotorJoint.internal(super.world, super.index1, super.wg)
    : super.internal();

  /// The target position of body B, in the frame of body A.
  Vector2 get linearOffset {
    final (x, y) = rawBox2D.motorJointGetLinearOffset(index1, wg);
    return Vector2(x, y);
  }

  set linearOffset(Vector2 value) =>
      rawBox2D.motorJointSetLinearOffset(index1, wg, value.x, value.y);

  /// The target angle of body B relative to body A, in radians.
  double get angularOffset => rawBox2D.motorJointGetAngularOffset(index1, wg);

  set angularOffset(double value) =>
      rawBox2D.motorJointSetAngularOffset(index1, wg, value);

  /// The maximum force the motor can apply, in newtons.
  double get maxForce => rawBox2D.motorJointGetMaxForce(index1, wg);

  set maxForce(double value) =>
      rawBox2D.motorJointSetMaxForce(index1, wg, value);

  /// The maximum torque the motor can apply, in newton-meters.
  double get maxTorque => rawBox2D.motorJointGetMaxTorque(index1, wg);

  set maxTorque(double value) =>
      rawBox2D.motorJointSetMaxTorque(index1, wg, value);

  /// The position correction factor, in `[0, 1]`.
  double get correctionFactor =>
      rawBox2D.motorJointGetCorrectionFactor(index1, wg);

  set correctionFactor(double value) =>
      rawBox2D.motorJointSetCorrectionFactor(index1, wg, value);
}
