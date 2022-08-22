import 'package:forge2d/forge2d.dart';

/// This proxy is used internally to connect fixtures to the broad-phase.
class FixtureProxy {
  FixtureProxy(this.fixture);
  final Fixture fixture;
  final AABB aabb = AABB();
  int childIndex = 0;
  int proxyId = 0;
}
