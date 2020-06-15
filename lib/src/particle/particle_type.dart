part of box2d;

/// The particle type. Can be combined with | operator. Zero means liquid.
class ParticleType {
  static const int b2_waterParticle = 0;

  /// removed after next step
  static const int b2_zombieParticle = 1 << 1;

  /// zero velocity
  static const int b2_wallParticle = 1 << 2;

  /// with restitution from stretching
  static const int b2_springParticle = 1 << 3;

  /// with restitution from deformation
  static const int b2_elasticParticle = 1 << 4;

  /// with viscosity
  static const int b2_viscousParticle = 1 << 5;

  /// without isotropic pressure
  static const int b2_powderParticle = 1 << 6;

  /// with surface tension
  static const int b2_tensileParticle = 1 << 7;

  /// mixing color between contacting particles
  static const int b2_colorMixingParticle = 1 << 8;

  /// call b2DestructionListener on destruction
  static const int b2_destructionListener = 1 << 9;
}
