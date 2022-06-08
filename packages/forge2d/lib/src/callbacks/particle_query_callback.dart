import 'package:forge2d/forge2d.dart';

/// Callback class for AABB queries. See [World.queryAABB].
abstract class ParticleQueryCallback {
  /// Called for each particle found in the query AABB.
  ///
  /// Return false to terminate the query.
  bool reportParticle(Particle particle);
}
