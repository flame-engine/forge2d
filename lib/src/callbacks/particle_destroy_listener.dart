import '../../forge2d.dart';

abstract class ParticleDestroyListener {
  /// Called when any particle group is about to be destroyed.
  void onDestroyParticleGroup(ParticleGroup group);

  /// Called when a particle is about to be destroyed.
  void onDestroyParticle(Particle particle);
}
