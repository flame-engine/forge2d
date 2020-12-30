part of forge2d;

class ParticleContact {
  /// The respective particles making contact.
  Particle particleA;
  Particle particleB;

  ParticleContact(this.particleA, this.particleB);

  /// The logical sum of the particle behaviors that have been set.
  int flags = 0;

  /// Weight of the contact. A value between 0.0f and 1.0f.
  double weight = 0.0;

  /// The normalized direction from A to B.
  final Vector2 normal = Vector2.zero();
}
