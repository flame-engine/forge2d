/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

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
    _pairBuffer = new List<Pair>(_pairCapacity);
    for (int i = 0; i < _pairCapacity; i++) {
      _pairBuffer[i] = new Pair();
    }
    _moveBuffer = BufferUtils.allocClearIntList(_moveCapacity);
  }

  int createProxy(final AABB aabb, Object userData) {
    int proxyId = _tree.createProxy(aabb, userData);
    ++_proxyCount;
    bufferMove(proxyId);
    return proxyId;
  }

  void destroyProxy(int proxyId) {
    unbufferMove(proxyId);
    --_proxyCount;
    _tree.destroyProxy(proxyId);
  }

  void moveProxy(int proxyId, final AABB aabb, final Vector2 displacement) {
    bool buffer = _tree.moveProxy(proxyId, aabb, displacement);
    if (buffer) {
      bufferMove(proxyId);
    }
  }

  void touchProxy(int proxyId) {
    bufferMove(proxyId);
  }

  Object getUserData(int proxyId) {
    return _tree.getUserData(proxyId);
  }

  AABB getFatAABB(int proxyId) {
    return _tree.getFatAABB(proxyId);
  }

  bool testOverlap(int proxyIdA, int proxyIdB) {
    // return AABB.testOverlap(proxyA.aabb, proxyB.aabb);
    // return _tree.overlap(proxyIdA, proxyIdB);
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

  int getProxyCount() {
    return _proxyCount;
  }

  void drawTree(DebugDraw argDraw) {
    _tree.drawTree(argDraw);
  }

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
      // log.debug("quering aabb: "+_queryProxy.aabb);
      _tree.query(this, fatAABB);
    }
    // log.debug("Number of pairs found: "+_pairCount);

    // Reset move buffer
    _moveCount = 0;

    // Sort the pair buffer to expose duplicates.
    BufferUtils.sort(_pairBuffer, 0, _pairCount);

    // Send the pairs back to the client.
    int i = 0;
    while (i < _pairCount) {
      Pair primaryPair = _pairBuffer[i];
      Object userDataA = _tree.getUserData(primaryPair.proxyIdA);
      Object userDataB = _tree.getUserData(primaryPair.proxyIdB);

      // log.debug("returning pair: "+userDataA+", "+userDataB);
      callback.addPair(userDataA, userDataB);
      ++i;

      // Skip any duplicate pairs.
      while (i < _pairCount) {
        Pair pair = _pairBuffer[i];
        if (pair.proxyIdA != primaryPair.proxyIdA ||
            pair.proxyIdB != primaryPair.proxyIdB) {
          break;
        }
        ++i;
      }
    }
  }

  void query(final TreeCallback callback, final AABB aabb) {
    _tree.query(callback, aabb);
  }

  void raycast(final TreeRayCastCallback callback, final RayCastInput input) {
    _tree.raycast(callback, input);
  }

  int getTreeHeight() {
    return _tree.getHeight();
  }

  int getTreeBalance() {
    return _tree.getMaxBalance();
  }

  double getTreeQuality() {
    return _tree.getAreaRatio();
  }

  void bufferMove(int proxyId) {
    if (_moveCount == _moveCapacity) {
      List<int> old = _moveBuffer;
      _moveCapacity *= 2;
      _moveBuffer = new List<int>(_moveCapacity);
      BufferUtils.arraycopy(old, 0, _moveBuffer, 0, old.length);
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
  bool treeCallback(int proxyId) {
    // A proxy cannot form a pair with itself.
    if (proxyId == _queryProxyId) {
      return true;
    }

    // Grow the pair buffer as needed.
    if (_pairCount == _pairCapacity) {
      List<Pair> oldBuffer = _pairBuffer;
      _pairCapacity *= 2;
      _pairBuffer = new List<Pair>(_pairCapacity);
      BufferUtils.arraycopy(oldBuffer, 0, _pairBuffer, 0, oldBuffer.length);
      for (int i = oldBuffer.length; i < _pairCapacity; i++) {
        _pairBuffer[i] = new Pair();
      }
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
