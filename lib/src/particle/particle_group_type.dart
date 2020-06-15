part of box2d;

class ParticleGroupType {
  /// resists penetration
  static const int b2_solidParticleGroup = 1 << 0;

  /// keeps its shape
  static const int b2_rigidParticleGroup = 1 << 1;
}
