part of box2d;

class ParticleContact {
  /// Indices of the respective particles making contact.
  int indexA = 0;
  int indexB = 0;

  /// The logical sum of the particle behaviors that have been set.
  int flags = 0;

  /// Weight of the contact. A value between 0.0f and 1.0f.
  double weight = 0.0;

  /// The normalized direction from A to B.
  final Vector2 normal = Vector2.zero();
}
