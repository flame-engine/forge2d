/// The particle type. Can be combined with | operator. Zero means liquid.
class ParticleType {
  static const int waterParticle = 0;

  /// removed after next step
  static const int zombieParticle = 1 << 1;

  /// zero velocity
  static const int wallParticle = 1 << 2;

  /// with restitution from stretching
  static const int springParticle = 1 << 3;

  /// with restitution from deformation
  static const int elasticParticle = 1 << 4;

  /// with viscosity
  static const int viscousParticle = 1 << 5;

  /// without isotropic pressure
  static const int powderParticle = 1 << 6;

  /// with surface tension
  static const int tensileParticle = 1 << 7;

  /// mixing color between contacting particles
  static const int colorMixingParticle = 1 << 8;

  /// call the destroy listener on destruction
  static const int destroyListener = 1 << 9;
}
