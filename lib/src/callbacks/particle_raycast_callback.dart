part of box2d;

abstract class ParticleRaycastCallback {
  /// Called for each particle found in the query. See
  /// {@link RayCastCallback#reportFixture(org.jbox2d.dynamics.Fixture, Vec2, Vec2, float)} for argument info.
  double reportParticle(
      int index, Vector2 point, Vector2 normal, double fraction);
}
