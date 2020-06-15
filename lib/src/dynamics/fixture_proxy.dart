part of box2d;

/// This proxy is used internally to connect fixtures to the broad-phase.
class FixtureProxy {
  final AABB aabb = AABB();
  Fixture fixture;
  int childIndex = 0;
  int proxyId = 0;
}
