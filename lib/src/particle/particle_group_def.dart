part of forge2d;

/// A particle group definition holds all the data needed to construct a particle group. You can
/// safely re-use these definitions.
// TODO: Remove this and only ues ParticleGroup directly
class ParticleGroupDef {
  /// The particle-behavior flags.
  int flags = 0;

  /// The group-construction flags.
  int groupFlags = 0;

  /// The world position of the group. Moves the group's shape a distance equal to the value of
  /// position.
  final Vector2 position = Vector2.zero();

  /// The world angle of the group in radians. Rotates the shape by an angle equal to the value of
  /// angle.
  double angle = 0.0;

  /// The linear velocity of the group's origin in world co-ordinates.
  final Vector2 linearVelocity = Vector2.zero();

  /// The angular velocity of the group.
  double angularVelocity = 0.0;

  /// The color of all particles in the group.
  ParticleColor color = ParticleColor.black();

  /// The strength of cohesion among the particles in a group with flag elasticParticle or
  /// springParticle.
  double strength = 1.0;

  /// Shape containing the particle group.
  Shape shape;

  /// If true, destroy the group automatically after its last particle has been destroyed.
  bool destroyAutomatically = true;

  /// Use this to store application-specific group data.
  Object userData;
}
