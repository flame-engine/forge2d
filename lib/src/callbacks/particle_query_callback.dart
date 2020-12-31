part of forge2d;

/// Callback class for AABB queries. See
/// {@link World#queryAABB(QueryCallback, org.jbox2d.collision.AABB)}.
abstract class ParticleQueryCallback {
  /// Called for each particle found in the query AABB.
  ///
  /// @return false to terminate the query.
  bool reportParticle(Particle particle);
}
