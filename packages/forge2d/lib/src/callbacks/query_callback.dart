import 'package:forge2d/forge2d.dart';

/// Callback class for AABB queries.
/// See [World.queryAABB].
abstract class QueryCallback {
  /// Called for each fixture found in the query AABB.
  /// Return false to terminate the query.
  bool reportFixture(Fixture fixture);
}
