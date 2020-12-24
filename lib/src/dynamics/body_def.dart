part of forge2d;

/// A body definition holds all the data needed to construct a rigid body. You can safely re-use body
/// definitions. Shapes are added to a body after construction.
class BodyDef {
  /// The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
  /// mass is set to one.
  BodyType type = BodyType.STATIC;

  /// Use this to store application specific body data.
  Object userData;

  /// The world position of the body. Avoid creating bodies at the origin since this can lead to many
  /// overlapping shapes.
  Vector2 position = Vector2.zero();

  /// The world angle of the body in radians.
  double angle = 0.0;

  /// The linear velocity of the body in world co-ordinates.
  Vector2 linearVelocity = Vector2.zero();

  /// The angular velocity of the body.
  double angularVelocity = 0.0;

  /// Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  double linearDamping = 0.0;

  /// Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  double angularDamping = 0.0;

  /// Set this flag to false if this body should never fall asleep. Note that this increases CPU
  /// usage.
  bool allowSleep = true;

  /// Is this body initially sleeping?
  bool isAwake = true;

  /// Should this body be prevented from rotating? Useful for characters.
  bool fixedRotation = false;

  /// Is this a fast moving body that should be prevented from tunneling through other moving bodies?
  /// Note that all bodies are prevented from tunneling through kinematic and static bodies. This
  /// setting is only considered on dynamic bodies.
  ///
  /// @warning You should use this flag sparingly since it increases processing time.
  bool bullet = false;

  /// Does this body start out active?
  bool active = true;

  /// Experimental: scales the inertia tensor.
  double gravityScale = 1.0;
}
