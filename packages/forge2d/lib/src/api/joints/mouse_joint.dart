import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The data needed to construct a [MouseJoint].
///
/// Mirrors the native `b2MouseJointDef`, with the native defaults.
class MouseJointDef extends JointDef {
  /// Creates a mouse joint definition.
  ///
  /// [bodyA] is a static reference body; [bodyB] is the body to drag.
  MouseJointDef({
    required super.bodyA,
    required super.bodyB,
    Vector2? target,
    this.hertz = 4,
    this.dampingRatio = 1,
    this.maxForce = 1,
    super.collideConnected,
    super.userData,
  }) : target = target ?? Vector2.zero();

  /// The initial target point in world coordinates.
  Vector2 target;

  /// The spring stiffness in hertz.
  double hertz;

  /// The spring damping ratio, non-dimensional.
  double dampingRatio;

  /// The maximum force the joint can apply, in newtons.
  ///
  /// Usually a multiple of the weight of the dragged body:
  /// `mass * gravity * factor`.
  double maxForce;
}

/// Pulls a point on a body towards a target point, designed for picking up
/// and dragging bodies with a pointer.
final class MouseJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const MouseJoint.internal(super.world, super.index1, super.worldAndGeneration)
    : super.internal();

  /// The target point in world coordinates.
  Vector2 get target {
    final (x, y) = rawBox2D.mouseJointGetTarget(index1, worldAndGeneration);
    return Vector2(x, y);
  }

  set target(Vector2 value) => rawBox2D.mouseJointSetTarget(
    index1,
    worldAndGeneration,
    value.x,
    value.y,
  );

  /// The spring stiffness in hertz.
  double get springHertz =>
      rawBox2D.mouseJointGetSpringHertz(index1, worldAndGeneration);

  set springHertz(double value) =>
      rawBox2D.mouseJointSetSpringHertz(index1, worldAndGeneration, value);

  /// The spring damping ratio.
  double get springDampingRatio =>
      rawBox2D.mouseJointGetSpringDampingRatio(index1, worldAndGeneration);

  set springDampingRatio(double value) => rawBox2D
      .mouseJointSetSpringDampingRatio(index1, worldAndGeneration, value);

  /// The maximum force the joint can apply, in newtons.
  double get maxForce =>
      rawBox2D.mouseJointGetMaxForce(index1, worldAndGeneration);

  set maxForce(double value) =>
      rawBox2D.mouseJointSetMaxForce(index1, worldAndGeneration, value);
}
