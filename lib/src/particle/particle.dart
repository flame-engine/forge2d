part of forge2d;

class Particle {
  /// Specifies the type of particle. A particle may be more than one type. Multiple types are
  /// chained by logical sums, for example: pd.flags = ParticleType.elasticParticle |
  /// ParticleType.viscousParticle.
  int flags = 0;

  /// The world position of the particle.
  final Vector2 position = Vector2.zero();

  /// The linear velocity of the particle in world co-ordinates.
  final Vector2 velocity = Vector2.zero();

  /// The group which the particle belongs to
  ParticleGroup group = ParticleGroup();

  // TODO: No idea what this is used for
  double accumulation = 0.0;

  /// TODO: The depth of the particle?
  double depth = 0.0;

  /// The color of the particle.
  ParticleColor color;

  /// Use this to store application-specific body data.
  Object userData;

  Particle clone() {
    return Particle()
      ..flags = flags
      ..position.setFrom(position)
      ..velocity.setFrom(velocity)
      ..group = group
      ..depth = depth
      ..color = color.clone()
      ..userData = userData;
  }
}
