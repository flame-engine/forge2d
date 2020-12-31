part of forge2d;

abstract class ParticleDestructionListener {
  /// Called when any particle group is about to be destroyed.
  void sayGoodbyeParticleGroup(ParticleGroup group);

  /// Called when a particle is about to be destroyed.
  void sayGoodbyeParticle(Particle particle);
}
