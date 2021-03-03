import '../../forge2d.dart';

/// Callback class for ray casts.
/// See {@link World#raycast(RayCastCallback, Vec2, Vec2)}
abstract class RayCastCallback {
  /// Called for each fixture found in the query. You control how the ray cast
  /// proceeds by returning a float:
  /// return -1: ignore this fixture and continue
  /// return 0: terminate the ray cast
  /// return fraction: clip the ray to this point
  /// return 1: don't clip the ray and continue
  /// @param fixture the fixture hit by the ray
  /// @param point the point of initial intersection
  /// @param normal the normal vector at the point of intersection
  /// @param fraction
  /// @return -1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue
  double reportFixture(
    Fixture fixture,
    Vector2 point,
    Vector2 normal,
    double fraction,
  );
}
