part of forge2d;

/// The broad-phase is used for computing pairs and performing volume queries and ray casts. This
/// broad-phase does not persist pairs. Instead, this reports potentially new pairs. It is up to the
/// client to consume the new pairs and to track subsequent overlap.
class DefaultBroadPhaseBuffer implements TreeCallback, BroadPhase {
  final BroadPhaseStrategy _tree;

  int _proxyCount = 0;

  List<int> _moveBuffer;
  int _moveCapacity = 16;
  int _moveCount = 0;

  List<Pair> _pairBuffer;
  int _pairCapacity = 16;
  int _pairCount = 0;

  int _queryProxyId = BroadPhase.NULL_PROXY;

  DefaultBroadPhaseBuffer(BroadPhaseStrategy strategy) : _tree = strategy {
    _pairBuffer = List<Pair>.generate(_pairCapacity, (_) => Pair());
    _moveBuffer = List<int>.filled(_moveCapacity, 0);
  }

  @override
  int createProxy(final AABB aabb, Object userData) {
    final int proxyId = _tree.createProxy(aabb, userData);
    ++_proxyCount;
    bufferMove(proxyId);
    return proxyId;
  }

  @override
  void destroyProxy(int proxyId) {
    unbufferMove(proxyId);
    --_proxyCount;
    _tree.destroyProxy(proxyId);
  }

  @override
  void moveProxy(int proxyId, final AABB aabb, final Vector2 displacement) {
    final bool buffer = _tree.moveProxy(proxyId, aabb, displacement);
    if (buffer) {
      bufferMove(proxyId);
    }
  }

  @override
  void touchProxy(int proxyId) {
    bufferMove(proxyId);
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
    final AABB a = _tree.getFatAABB(proxyIdA);
    final AABB b = _tree.getFatAABB(proxyIdB);
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
    return _proxyCount;
  }

  @override
  void drawTree(DebugDraw argDraw) {
    _tree.drawTree(argDraw);
  }

  @override
  void updatePairs(PairCallback callback) {
    // Reset pair buffer
    _pairCount = 0;

    // Perform tree queries for all moving proxies.
    for (int i = 0; i < _moveCount; ++i) {
      _queryProxyId = _moveBuffer[i];
      if (_queryProxyId == BroadPhase.NULL_PROXY) {
        continue;
      }

      // We have to query the tree with the fat AABB so that
      // we don't fail to create a pair that may touch later.
      final AABB fatAABB = _tree.getFatAABB(_queryProxyId);

      // Query tree, create pairs and add them pair buffer.
      _tree.query(this, fatAABB);
    }

    // Reset move buffer
    _moveCount = 0;

    // Sort the pair buffer to expose duplicates.
    buffer_utils.sort(_pairBuffer, 0, _pairCount);

    // Send the pairs back to the client.
    int i = 0;
    while (i < _pairCount) {
      final Pair primaryPair = _pairBuffer[i];
      final Object userDataA = _tree.getUserData(primaryPair.proxyIdA);
      final Object userDataB = _tree.getUserData(primaryPair.proxyIdB);

      callback.addPair(userDataA, userDataB);
      ++i;

      // Skip any duplicate pairs.
      while (i < _pairCount) {
        final Pair pair = _pairBuffer[i];
        if (pair.proxyIdA != primaryPair.proxyIdA ||
            pair.proxyIdB != primaryPair.proxyIdB) {
          break;
        }
        ++i;
      }
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

  void bufferMove(int proxyId) {
    if (_moveCount == _moveCapacity) {
      _moveBuffer = _moveBuffer + List.filled(_moveCapacity ~/ 2, 0);
      _moveCapacity = _moveBuffer.length;
    }

    _moveBuffer[_moveCount] = proxyId;
    ++_moveCount;
  }

  void unbufferMove(int proxyId) {
    for (int i = 0; i < _moveCount; i++) {
      if (_moveBuffer[i] == proxyId) {
        _moveBuffer[i] = BroadPhase.NULL_PROXY;
      }
    }
  }

  /// This is called from DynamicTree::query when we are gathering pairs.
  @override
  bool treeCallback(int proxyId) {
    // A proxy cannot form a pair with itself.
    if (proxyId == _queryProxyId) {
      return true;
    }

    // Grow the pair buffer as needed.
    if (_pairCount == _pairCapacity) {
      _pairBuffer =
          _pairBuffer + List<Pair>.generate(_pairCapacity, (_) => Pair());
      _pairCapacity = _pairBuffer.length;
    }

    if (proxyId < _queryProxyId) {
      _pairBuffer[_pairCount].proxyIdA = proxyId;
      _pairBuffer[_pairCount].proxyIdB = _queryProxyId;
    } else {
      _pairBuffer[_pairCount].proxyIdA = _queryProxyId;
      _pairBuffer[_pairCount].proxyIdB = proxyId;
    }

    ++_pairCount;
    return true;
  }
}
