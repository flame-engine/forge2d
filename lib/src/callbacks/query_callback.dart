part of box2d;

/// Callback class for AABB queries.
/// See {@link World#queryAABB(QueryCallback, org.jbox2d.collision.AABB)}.
abstract class QueryCallback {
  /// Called for each fixture found in the query AABB.
  /// @param fixture
  /// @return false to terminate the query.
  bool reportFixture(Fixture fixture);
}
