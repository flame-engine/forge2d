import '../../forge2d.dart';

/// Holds all the data needed to construct a [Body].
///
/// You can safely re-use body definitions.
///
/// [Shape]s are added through [Fixture]s to a [Body] after construction via
/// [Body.createFixture].
class BodyDef {
  BodyDef({
    this.type = BodyType.static,
    this.userData,
    Vector2? position,
    this.angle = 0.0,
    Vector2? linearVelocity,
    this.angularVelocity = 0.0,
    this.linearDamping = 0.0,
    this.angularDamping = 0.0,
    this.allowSleep = true,
    this.isAwake = true,
    this.fixedRotation = false,
    this.bullet = false,
    this.active = true,
    this.gravityOverride,
  })  : position = position ?? Vector2.zero(),
        linearVelocity = linearVelocity ?? Vector2.zero();

  /// The body type: static, kinematic, or dynamic.
  ///
  /// Note: A [BodyType.dynamic] body with zero mass, will have a mass of one.
  BodyType type;

  /// Use this to store application specific body data.
  Object? userData;

  /// The world position of the body.
  ///
  /// Avoid creating bodies at the origin since this can lead to many
  /// overlapping [Shape]s.
  Vector2 position;

  /// The world angle of the body in radians.
  double angle;

  /// The linear velocity of the body in world co-ordinates.
  Vector2 linearVelocity;

  /// The angular velocity of the body.
  double angularVelocity;

  /// Linear damping is use to reduce the linear velocity.
  ///
  /// The damping parameter can be larger than 1.0f but the damping effect
  /// becomes sensitive to the time step when the damping parameter is large.
  double linearDamping;

  /// Angular damping is use to reduce the angular velocity.
  ///
  /// The damping parameter can be larger than 1.0f but the damping effect
  /// becomes sensitive to the time step when the damping parameter is large.
  double angularDamping;

  /// Set this flag to false if this body should never fall asleep.
  ///
  /// Note: Not alllowing a body to sleep increases CPU usage.
  bool allowSleep;

  /// Is this body initially sleeping?
  bool isAwake;

  /// Should this body be prevented from rotating?
  ///
  /// Useful for characters.
  bool fixedRotation;

  /// Is this a fast moving body that should be prevented from tunneling through
  /// other moving bodies?
  ///
  /// Note: All bodies are prevented from tunneling through [BodyType.kinematic]
  /// and [BodyType.static] bodies. This setting is only considered on
  /// [BodyType.dynamic] bodies.
  ///
  /// Warning: You should use this flag sparingly since it increases processing
  /// time.
  bool bullet;

  /// Does this body start out active?
  bool active;

  /// Changes how the [World] treats the gravity for this body.
  ///
  /// Specifying a [gravityOverride] overrides the world's gravity. For example,
  /// if [World.gravity] is (0, -10), and a body has a [gravityOverride] of
  /// (0, 0) the body will behave as if the world does not have a gravity.
  ///
  /// If you wish to modify the gravity relative to the world, use
  /// [World.gravity] as part of the calculation.
  Vector2? gravityOverride;
}
