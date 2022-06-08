import 'package:forge2d/forge2d.dart';

class Particle {
  /// Specifies the type of particle. A particle may be more than one type.
  /// Multiple types are chained by logical sums, for example:
  /// pd.flags = ParticleType.elasticParticle | ParticleType.viscousParticle.
  int flags = 0;

  /// The ParticleSystem which the particle belongs to
  final ParticleSystem system;

  /// The world position of the particle.
  final Vector2 position = Vector2.zero();

  /// The linear velocity of the particle in world co-ordinates.
  final Vector2 velocity = Vector2.zero();

  /// The group which the particle belongs to
  late ParticleGroup group;

  // TODO(spydon): No idea what this is used for
  double accumulation = 0.0;

  final Vector2 accumulationVector = Vector2.zero();

  // TODO(spydon): What is the depth of the particle?
  double depth = 0.0;

  /// The color of the particle.
  Color3i color = Color3i.black();

  /// Use this to store application-specific body data.
  Object? userData;

  Particle(this.system, {ParticleGroup? group}) {
    this.group = group ?? ParticleGroup(system);
  }

  Particle clone() {
    return Particle(system, group: group)
      ..flags = flags
      ..position.setFrom(position)
      ..velocity.setFrom(velocity)
      ..depth = depth
      ..color = color.clone()
      ..userData = userData;
  }
}
