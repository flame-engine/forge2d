import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [WeldJoint].
///
/// Mirrors the native `b2WeldJointDef`, with the native defaults.
class WeldJointDef extends JointDef {
  /// Creates a weld joint definition.
  WeldJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? localAnchorA,
    Vector2? localAnchorB,
    this.referenceAngle = 0,
    this.linearHertz = 0,
    this.angularHertz = 0,
    this.linearDampingRatio = 0,
    this.angularDampingRatio = 0,
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

  /// The linear stiffness in hertz; zero for a rigid connection.
  double linearHertz;

  /// The angular stiffness in hertz; zero for a rigid connection.
  double angularHertz;

  /// The linear damping ratio, non-dimensional.
  double linearDampingRatio;

  /// The angular damping ratio, non-dimensional.
  double angularDampingRatio;
}

/// Rigidly attaches two bodies, with optional softness in the connection.
///
/// Note that soft body dynamics built out of weld joints will not be rigid
/// under load; consider merging shapes onto one body instead.
final class WeldJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const WeldJoint.internal(super.world, super.index1, super.worldAndGeneration)
    : super.internal();

  /// The linear stiffness in hertz; zero for a rigid connection.
  double get linearHertz =>
      rawBox2D.weldJointGetLinearHertz(index1, worldAndGeneration);

  set linearHertz(double value) =>
      rawBox2D.weldJointSetLinearHertz(index1, worldAndGeneration, value);

  /// The angular stiffness in hertz; zero for a rigid connection.
  double get angularHertz =>
      rawBox2D.weldJointGetAngularHertz(index1, worldAndGeneration);

  set angularHertz(double value) =>
      rawBox2D.weldJointSetAngularHertz(index1, worldAndGeneration, value);

  /// The linear damping ratio.
  double get linearDampingRatio =>
      rawBox2D.weldJointGetLinearDampingRatio(index1, worldAndGeneration);

  set linearDampingRatio(double value) => rawBox2D
      .weldJointSetLinearDampingRatio(index1, worldAndGeneration, value);

  /// The angular damping ratio.
  double get angularDampingRatio =>
      rawBox2D.weldJointGetAngularDampingRatio(index1, worldAndGeneration);

  set angularDampingRatio(double value) => rawBox2D
      .weldJointSetAngularDampingRatio(index1, worldAndGeneration, value);
}
