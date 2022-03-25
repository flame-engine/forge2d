import '../../../forge2d.dart';

/// {@template dynamics.joints.joint_def}
/// [JointDef]s are used to construct [Joint]s.
///
/// All [Joint]s are connected between two different [Body]ies.
/// One [Body] may be [BodyType.static].
///
/// [Joint]s between [BodyType.static] and/or [BodyType.kinematic] are allowed,
/// but have no effect and use some processing time.
///
/// Don't subclass [JointDef]. Instead, extend from an already defined
/// [JointDef] subclass.
/// {@endtemplate}
abstract class JointDef<A extends Body, B extends Body> {
  /// {@macro dynamics.joints.joint_def}
  JointDef([this.collideConnected = false]);

  /// The local anchor point relative to body1's origin.
  final Vector2 localAnchorA = Vector2.zero();

  /// The local anchor point relative to body2's origin.
  final Vector2 localAnchorB = Vector2.zero();

  /// Use this to attach application specific data to your joints.
  Object? userData;

  /// The first attached body.
  late A bodyA;

  /// The second attached body.
  late B bodyB;

  /// Set this flag to true if the attached bodies should collide.
  bool collideConnected = false;
}
