part of box2d;

abstract class ParticleDestructionListener {
  /// Called when any particle group is about to be destroyed.
  void sayGoodbyeParticleGroup(ParticleGroup group);

  /// Called when a particle is about to be destroyed. The index can be used in conjunction with
  /// {@link World#getParticleUserDataBuffer} to determine which particle has been destroyed.
  void sayGoodbyeIndex(int index);
}
