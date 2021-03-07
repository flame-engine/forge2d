import '../../../forge2d.dart';

/// Joint definitions are used to construct joints.
class JointDef {
  /// The local anchor point relative to body1's origin.
  final Vector2 localAnchorA = Vector2.zero();

  /// The local anchor point relative to body2's origin.
  final Vector2 localAnchorB = Vector2.zero();

  JointDef(this.type, [this.collideConnected = false]);

  /// The joint type is set automatically for concrete joint types.
  JointType type;

  /// Use this to attach application specific data to your joints.
  Object? userData;

  /// The first attached body.
  late Body bodyA;

  /// The second attached body.
  late Body bodyB;

  /// Set this flag to true if the attached bodies should collide.
  bool collideConnected = false;
}
