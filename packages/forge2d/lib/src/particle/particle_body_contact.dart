import 'package:forge2d/forge2d.dart';

class ParticleBodyContact {
  /// Index of the particle making contact.
  final Particle particle;

  /// The body making contact.
  final Body body;

  /// Weight of the contact. A value between 0.0f and 1.0f.
  double weight = 0.0;

  /// The normalized direction from the particle to the body.
  final Vector2 normal = Vector2.zero();

  /// The effective mass used in calculating force.
  double mass = 0.0;

  ParticleBodyContact(this.particle, this.body);
}
