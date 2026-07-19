import 'package:forge2d/src/api/joints/joint.dart';
import 'package:meta/meta.dart';

/// The data needed to construct a [FilterJoint].
///
/// Mirrors the native `b2FilterJointDef`.
class FilterJointDef extends JointDef {
  /// Creates a filter joint definition.
  FilterJointDef({required super.bodyA, required super.bodyB, super.userData});
}

/// Disables collision between the two attached bodies, with no other
/// constraint.
final class FilterJoint extends Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const FilterJoint.internal(
    super.world,
    super.index1,
    super.worldAndGeneration,
  ) : super.internal();
}
