import 'dart:math';

import '../../../forge2d.dart';
import '../../../src/callbacks/debug_draw.dart';
import '../../../src/callbacks/pair_callback.dart';
import '../../../src/callbacks/tree_callback.dart';
import '../../../src/callbacks/tree_raycast_callback.dart';

/// The broad-phase is used for computing pairs and performing volume queries and ray casts. This
/// broad-phase does not persist pairs. Instead, this reports potentially new pairs. It is up to the
/// client to consume the new pairs and to track subsequent overlap.
class DefaultBroadPhaseBuffer implements TreeCallback, BroadPhase {
  final BroadPhaseStrategy _tree;

  final List<int> _moveBuffer = <int>[];
  final Set<Pair> _pairBuffer = <Pair>{};

  int _queryProxyId = BroadPhase.nullProxy;

  DefaultBroadPhaseBuffer(this._tree);

  @override
  int createProxy(final AABB aabb, Object userData) {
    final proxyId = _tree.createProxy(aabb, userData);
    _moveBuffer.add(proxyId);
    return proxyId;
  }

  @override
  void destroyProxy(int proxyId) {
    _moveBuffer.remove(proxyId);
    _tree.destroyProxy(proxyId);
  }

  @override
  void moveProxy(int proxyId, final AABB aabb, final Vector2 displacement) {
    final buffer = _tree.moveProxy(proxyId, aabb, displacement);
    if (buffer) {
      _moveBuffer.add(proxyId);
    }
  }

  @override
  void touchProxy(int proxyId) {
    _moveBuffer.add(proxyId);
  }

  @override
  Object getUserData(int proxyId) {
    return _tree.getUserData(proxyId);
  }

  @override
  AABB getFatAABB(int proxyId) {
    return _tree.getFatAABB(proxyId);
  }

  @override
  bool testOverlap(int proxyIdA, int proxyIdB) {
    final a = _tree.getFatAABB(proxyIdA);
    final b = _tree.getFatAABB(proxyIdB);
    if (b.lowerBound.x - a.upperBound.x > 0.0 ||
        b.lowerBound.y - a.upperBound.y > 0.0) {
      return false;
    }

    if (a.lowerBound.x - b.upperBound.x > 0.0 ||
        a.lowerBound.y - b.upperBound.y > 0.0) {
      return false;
    }

    return true;
  }

  @override
  int getProxyCount() {
    return _moveBuffer.length;
  }

  @override
  void drawTree(DebugDraw argDraw) {
    _tree.drawTree(argDraw);
  }

  @override
  void updatePairs(PairCallback callback) {
    // Reset pair buffer
    _pairBuffer.clear();

    // Perform tree queries for all moving proxies.
    for (final proxyId in _moveBuffer) {
      _queryProxyId = proxyId;
      if (proxyId == BroadPhase.nullProxy) {
        continue;
      }

      // We have to query the tree with the fat AABB so that
      // we don't fail to create a pair that may touch later.
      final fatAABB = _tree.getFatAABB(proxyId);

      // Query tree, create pairs and add them pair buffer.
      _tree.query(this, fatAABB);
    }

    // Reset move buffer
    _moveBuffer.clear();

    // Send the pairs back to the client.
    for (final pair in _pairBuffer) {
      final userDataA = _tree.getUserData(pair.proxyIdA);
      final userDataB = _tree.getUserData(pair.proxyIdB);

      callback.addPair(userDataA, userDataB);
    }
  }

  @override
  void query(final TreeCallback callback, final AABB aabb) {
    _tree.query(callback, aabb);
  }

  @override
  void raycast(final TreeRayCastCallback callback, final RayCastInput input) {
    _tree.raycast(callback, input);
  }

  @override
  int getTreeHeight() {
    return _tree.getHeight();
  }

  @override
  int getTreeBalance() {
    return _tree.getMaxBalance();
  }

  @override
  double getTreeQuality() {
    return _tree.getAreaRatio();
  }

  /// This is called from DynamicTree.query when we are gathering pairs.
  @override
  bool treeCallback(int proxyId) {
    // A proxy cannot form a pair with itself.
    if (proxyId == _queryProxyId) {
      return true;
    }

    _pairBuffer.add(
      Pair(
        min(proxyId, _queryProxyId),
        max(proxyId, _queryProxyId),
      ),
    );
    return true;
  }
}
