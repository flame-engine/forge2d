import '../../forge2d.dart';

/// This proxy is used internally to connect fixtures to the broad-phase.
class FixtureProxy {
  FixtureProxy(this._fixture);
  final Fixture _fixture;
  Fixture get fixture => _fixture;
  final AABB aabb = AABB();
  int childIndex = 0;
  int proxyId = 0;
}
