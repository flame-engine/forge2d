import 'package:forge2d/forge2d.dart';

/// Callback class for ray casts.
/// See [World.raycast].
abstract class RayCastCallback {
  /// Called for each fixture found in the query. You control how the ray cast
  /// proceeds by returning a float:
  /// return -1: ignore this fixture and continue
  /// return 0: terminate the ray cast
  /// return fraction: clip the ray to this point
  /// return 1: don't clip the ray and continue
  /// [fixture] is the fixture hit by the ray.
  /// [point] is the point of initial intersection.
  /// [normal] is the normal vector at the point of intersection.
  double reportFixture(
    Fixture fixture,
    Vector2 point,
    Vector2 normal,
    double fraction,
  );
}
