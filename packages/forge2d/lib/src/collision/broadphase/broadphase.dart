import 'package:forge2d/forge2d.dart';

abstract class BroadPhase {
  // TODO(spydon): remove the use of this
  static const int nullProxy = -1;

  /// Create a proxy with an initial AABB. Pairs are not reported until
  /// [updatePairs] is called.
  int createProxy(AABB aabb, Object userData);

  /// Destroy a proxy. It is up to the client to remove any pairs.
  void destroyProxy(int proxyId);

  /// Call MoveProxy as many times as you like, then when you are done call
  /// [updatePairs] to finalized the proxy pairs (for your time step).
  void moveProxy(int proxyId, AABB aabb, Vector2 displacement);

  void touchProxy(int proxyId);

  Object? getUserData(int proxyId);

  AABB fatAABB(int proxyId);

  bool testOverlap(int proxyIdA, int proxyIdB);

  /// Get the number of proxies.
  int get proxyCount;

  void drawTree(DebugDraw argDraw);

  /// Update the pairs. This results in pair callbacks. This can only add pairs.
  void updatePairs(PairCallback callback);

  /// Query an AABB for overlapping proxies. The callback class is called for
  /// each proxy that overlaps the supplied AABB.
  void query(TreeCallback callback, AABB aabb);

  /// Ray-cast against the proxies in the tree. This relies on the callback to
  /// perform a exact ray-cast in the case were the proxy contains a shape.
  /// The callback also performs the any collision filtering. This has
  /// performance roughly equal to k * log(n), where k is the number of
  /// collisions and n is the number of proxies in the tree.
  ///
  /// [input] is the ray-cast input data.
  /// The ray extends from p1 to p1 + maxFraction * (p2 - p1).
  /// [callback] is a callback class that is called for each proxy that is hit
  /// by the ray.
  void raycast(TreeRayCastCallback callback, RayCastInput input);

  /// Get the height of the embedded tree.
  int getTreeHeight();

  int getTreeBalance();

  double getTreeQuality();
}
