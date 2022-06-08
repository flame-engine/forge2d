import 'package:forge2d/forge2d.dart';

abstract class ParticleRaycastCallback {
  /// Called for each particle found in the query.
  /// See [RayCastCallback.reportFixture] for more info.
  double reportParticle(
    int index,
    Vector2 point,
    Vector2 normal,
    double fraction,
  );
}
