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

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts. This
 * broad-phase does not persist pairs. Instead, this reports potentially new pairs. It is up to the
 * client to consume the new pairs and to track subsequent overlap.
 * 
 * @author Daniel Murphy
 */
class DefaultBroadPhaseBuffer implements TreeCallback, BroadPhase {
  final BroadPhaseStrategy _m_tree;

  int _m_proxyCount = 0;

  List<int> _m_moveBuffer;
  int _m_moveCapacity = 16;
  int _m_moveCount = 0;

  List<Pair> _m_pairBuffer;
  int _m_pairCapacity = 16;
  int _m_pairCount = 0;

  int _m_queryProxyId = BroadPhase.NULL_PROXY;

  DefaultBroadPhaseBuffer(BroadPhaseStrategy strategy) : _m_tree = strategy {
    _m_pairBuffer = new List<Pair>(_m_pairCapacity);
    for (int i = 0; i < _m_pairCapacity; i++) {
      _m_pairBuffer[i] = new Pair();
    }
    _m_moveBuffer = BufferUtils.allocClearIntList(_m_moveCapacity);
  }

  int createProxy(final AABB aabb, Object userData) {
    int proxyId = _m_tree.createProxy(aabb, userData);
    ++_m_proxyCount;
    bufferMove(proxyId);
    return proxyId;
  }

  void destroyProxy(int proxyId) {
    unbufferMove(proxyId);
    --_m_proxyCount;
    _m_tree.destroyProxy(proxyId);
  }

  void moveProxy(int proxyId, final AABB aabb, final Vec2 displacement) {
    bool buffer = _m_tree.moveProxy(proxyId, aabb, displacement);
    if (buffer) {
      bufferMove(proxyId);
    }
  }

  void touchProxy(int proxyId) {
    bufferMove(proxyId);
  }

  Object getUserData(int proxyId) {
    return _m_tree.getUserData(proxyId);
  }

  AABB getFatAABB(int proxyId) {
    return _m_tree.getFatAABB(proxyId);
  }

  bool testOverlap(int proxyIdA, int proxyIdB) {
    // return AABB.testOverlap(proxyA.aabb, proxyB.aabb);
    // return _m_tree.overlap(proxyIdA, proxyIdB);
    final AABB a = _m_tree.getFatAABB(proxyIdA);
    final AABB b = _m_tree.getFatAABB(proxyIdB);
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
    return _m_proxyCount;
  }

  void drawTree(DebugDraw argDraw) {
    _m_tree.drawTree(argDraw);
  }

  void updatePairs(PairCallback callback) {
    // Reset pair buffer
    _m_pairCount = 0;

    // Perform tree queries for all moving proxies.
    for (int i = 0; i < _m_moveCount; ++i) {
      _m_queryProxyId = _m_moveBuffer[i];
      if (_m_queryProxyId == BroadPhase.NULL_PROXY) {
        continue;
      }

      // We have to query the tree with the fat AABB so that
      // we don't fail to create a pair that may touch later.
      final AABB fatAABB = _m_tree.getFatAABB(_m_queryProxyId);

      // Query tree, create pairs and add them pair buffer.
      // log.debug("quering aabb: "+_m_queryProxy.aabb);
      _m_tree.query(this, fatAABB);
    }
    // log.debug("Number of pairs found: "+_m_pairCount);

    // Reset move buffer
    _m_moveCount = 0;

    // Sort the pair buffer to expose duplicates.
    BufferUtils.sort(_m_pairBuffer, 0, _m_pairCount);

    // Send the pairs back to the client.
    int i = 0;
    while (i < _m_pairCount) {
      Pair primaryPair = _m_pairBuffer[i];
      Object userDataA = _m_tree.getUserData(primaryPair.proxyIdA);
      Object userDataB = _m_tree.getUserData(primaryPair.proxyIdB);

      // log.debug("returning pair: "+userDataA+", "+userDataB);
      callback.addPair(userDataA, userDataB);
      ++i;

      // Skip any duplicate pairs.
      while (i < _m_pairCount) {
        Pair pair = _m_pairBuffer[i];
        if (pair.proxyIdA != primaryPair.proxyIdA ||
            pair.proxyIdB != primaryPair.proxyIdB) {
          break;
        }
        ++i;
      }
    }
  }

  void query(final TreeCallback callback, final AABB aabb) {
    _m_tree.query(callback, aabb);
  }

  void raycast(final TreeRayCastCallback callback, final RayCastInput input) {
    _m_tree.raycast(callback, input);
  }

  int getTreeHeight() {
    return _m_tree.getHeight();
  }

  int getTreeBalance() {
    return _m_tree.getMaxBalance();
  }

  double getTreeQuality() {
    return _m_tree.getAreaRatio();
  }

  void bufferMove(int proxyId) {
    if (_m_moveCount == _m_moveCapacity) {
      List<int> old = _m_moveBuffer;
      _m_moveCapacity *= 2;
      _m_moveBuffer = new List<int>(_m_moveCapacity);
      BufferUtils.arraycopy(old, 0, _m_moveBuffer, 0, old.length);
    }

    _m_moveBuffer[_m_moveCount] = proxyId;
    ++_m_moveCount;
  }

  void unbufferMove(int proxyId) {
    for (int i = 0; i < _m_moveCount; i++) {
      if (_m_moveBuffer[i] == proxyId) {
        _m_moveBuffer[i] = BroadPhase.NULL_PROXY;
      }
    }
  }

  /**
   * This is called from DynamicTree::query when we are gathering pairs.
   */
  bool treeCallback(int proxyId) {
    // A proxy cannot form a pair with itself.
    if (proxyId == _m_queryProxyId) {
      return true;
    }

    // Grow the pair buffer as needed.
    if (_m_pairCount == _m_pairCapacity) {
      List<Pair> oldBuffer = _m_pairBuffer;
      _m_pairCapacity *= 2;
      _m_pairBuffer = new List<Pair>(_m_pairCapacity);
      BufferUtils.arraycopy(oldBuffer, 0, _m_pairBuffer, 0, oldBuffer.length);
      for (int i = oldBuffer.length; i < _m_pairCapacity; i++) {
        _m_pairBuffer[i] = new Pair();
      }
    }

    if (proxyId < _m_queryProxyId) {
      _m_pairBuffer[_m_pairCount].proxyIdA = proxyId;
      _m_pairBuffer[_m_pairCount].proxyIdB = _m_queryProxyId;
    } else {
      _m_pairBuffer[_m_pairCount].proxyIdA = _m_queryProxyId;
      _m_pairBuffer[_m_pairCount].proxyIdB = proxyId;
    }

    ++_m_pairCount;
    return true;
  }
}
