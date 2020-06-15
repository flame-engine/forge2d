part of box2d;

class ParticleDef {
  /// Specifies the type of particle. A particle may be more than one type. Multiple types are
  /// chained by logical sums, for example: pd.flags = ParticleType.b2_elasticParticle |
  /// ParticleType.b2_viscousParticle.
  int flags = 0;

  /// The world position of the particle.
  final Vector2 position = Vector2.zero();

  /// The linear velocity of the particle in world co-ordinates.
  final Vector2 velocity = Vector2.zero();

  /// The color of the particle.
  ParticleColor color;

  /// Use this to store application-specific body data.
  Object userData;
}
