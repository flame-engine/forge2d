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

class DynamicTreeFlatNodes implements BroadPhaseStrategy {
  static const int MAX_STACK_SIZE = 64;
  static const int NULL_NODE = -1;
  static const int INITIAL_BUFFER_LENGTH = 16;

  int m_root = NULL_NODE;
  List<AABB> m_aabb;
  List<Object> m_userData;
  List<int> m_parent;
  List<int> m_child1;
  List<int> m_child2;
  List<int> m_height;

  int _m_nodeCount = 0;
  int _m_nodeCapacity = 16;

  int _m_freeList;

  final List<Vec2> drawVecs = new List<Vec2>(4);

  DynamicTreeFlatNodes() {
    _expandBuffers(0, _m_nodeCapacity);

    for (int i = 0; i < drawVecs.length; i++) {
      drawVecs[i] = new Vec2.zero();
    }
  }

  static AABB allocAABB() => new AABB();
  static Object allocObject() => new Object();

  void _expandBuffers(int oldSize, int newSize) {
    m_aabb = BufferUtils.reallocateBufferWithAlloc(m_aabb, oldSize, newSize, allocAABB);
    m_userData = BufferUtils.reallocateBufferWithAlloc(m_userData, oldSize, newSize, allocObject);
    m_parent = BufferUtils.reallocateBufferInt(m_parent, oldSize, newSize);
    m_child1 = BufferUtils.reallocateBufferInt(m_child1, oldSize, newSize);
    m_child2 = BufferUtils.reallocateBufferInt(m_child2, oldSize, newSize);
    m_height = BufferUtils.reallocateBufferInt(m_height, oldSize, newSize);

    // Build a linked list for the free list.
    for (int i = oldSize; i < newSize; i++) {
      m_aabb[i] = new AABB();
      m_parent[i] = (i == newSize - 1) ? NULL_NODE : i + 1;
      m_height[i] = -1;
      m_child1[i] = -1;
      m_child2[i] = -1;
    }
    _m_freeList = oldSize;
  }

  int createProxy(final AABB aabb, Object userData) {
    final int node = _allocateNode();
    // Fatten the aabb
    final AABB nodeAABB = m_aabb[node];
    nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
    m_userData[node] = userData;

    _insertLeaf(node);

    return node;
  }

  void destroyProxy(int proxyId) {
    assert(0 <= proxyId && proxyId < _m_nodeCapacity);
    assert(m_child1[proxyId] == NULL_NODE);

    _removeLeaf(proxyId);
    _freeNode(proxyId);
  }

  bool moveProxy(int proxyId, final AABB aabb, Vec2 displacement) {
    assert(0 <= proxyId && proxyId < _m_nodeCapacity);
    final int node = proxyId;
    assert(m_child1[node] == NULL_NODE);

    final AABB nodeAABB = m_aabb[node];
    // if (nodeAABB.contains(aabb)) {
    if (nodeAABB.lowerBound.x <= aabb.lowerBound.x && nodeAABB.lowerBound.y <= aabb.lowerBound.y && aabb.upperBound.x <= nodeAABB.upperBound.x && aabb.upperBound.y <= nodeAABB.upperBound.y) {
      return false;
    }

    _removeLeaf(node);

    // Extend AABB
    final Vec2 lowerBound = nodeAABB.lowerBound;
    final Vec2 upperBound = nodeAABB.upperBound;
    lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    upperBound.y = aabb.upperBound.y + Settings.aabbExtension;

    // Predict AABB displacement.
    final double dx = displacement.x * Settings.aabbMultiplier;
    final double dy = displacement.y * Settings.aabbMultiplier;
    if (dx < 0.0) {
      lowerBound.x += dx;
    } else {
      upperBound.x += dx;
    }

    if (dy < 0.0) {
      lowerBound.y += dy;
    } else {
      upperBound.y += dy;
    }

    _insertLeaf(proxyId);
    return true;
  }

  Object getUserData(int proxyId) {
    assert(0 <= proxyId && proxyId < _m_nodeCount);
    return m_userData[proxyId];
  }

  AABB getFatAABB(int proxyId) {
    assert(0 <= proxyId && proxyId < _m_nodeCount);
    return m_aabb[proxyId];
  }

  List<int> _nodeStack = BufferUtils.allocClearIntList(20);
  int _nodeStackIndex = 0;

  void query(TreeCallback callback, AABB aabb) {
    _nodeStackIndex = 0;
    _nodeStack[_nodeStackIndex++] = m_root;

    while (_nodeStackIndex > 0) {
      int node = _nodeStack[--_nodeStackIndex];
      if (node == NULL_NODE) {
        continue;
      }

      if (AABB.testOverlap(m_aabb[node], aabb)) {
        int child1 = m_child1[node];
        if (child1 == NULL_NODE) {
          bool proceed = callback.treeCallback(node);
          if (!proceed) {
            return;
          }
        } else {
          if (_nodeStack.length - _nodeStackIndex - 2 <= 0) {
            _nodeStack = BufferUtils.reallocateBufferInt(_nodeStack, _nodeStack.length, _nodeStack.length * 2);
          }
          _nodeStack[_nodeStackIndex++] = child1;
          _nodeStack[_nodeStackIndex++] = m_child2[node];
        }
      }
    }
  }

  final Vec2 _r = new Vec2.zero();
  final AABB _aabb = new AABB();
  final RayCastInput _subInput = new RayCastInput();

  void raycast(TreeRayCastCallback callback, RayCastInput input) {
    final Vec2 p1 = input.p1;
    final Vec2 p2 = input.p2;
    double p1x = p1.x,
        p2x = p2.x,
        p1y = p1.y,
        p2y = p2.y;
    double vx, vy;
    double rx, ry;
    double absVx, absVy;
    double cx, cy;
    double hx, hy;
    double tempx, tempy;
    _r.x = p2x - p1x;
    _r.y = p2y - p1y;
    assert((_r.x * _r.x + _r.y * _r.y) > 0.0);
    _r.normalize();
    rx = _r.x;
    ry = _r.y;

    // v is perpendicular to the segment.
    vx = -1.0 * ry;
    vy = 1.0 * rx;
    absVx = vx.abs();
    absVy = vy.abs();

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    double maxFraction = input.maxFraction;

    // Build a bounding box for the segment.
    final AABB segAABB = _aabb;
    // Vec2 t = p1 + maxFraction * (p2 - p1);
    // before inline
    // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
    // Vec2.minToOut(p1, temp, segAABB.lowerBound);
    // Vec2.maxToOut(p1, temp, segAABB.upperBound);
    tempx = (p2x - p1x) * maxFraction + p1x;
    tempy = (p2y - p1y) * maxFraction + p1y;
    segAABB.lowerBound.x = p1x < tempx ? p1x : tempx;
    segAABB.lowerBound.y = p1y < tempy ? p1y : tempy;
    segAABB.upperBound.x = p1x > tempx ? p1x : tempx;
    segAABB.upperBound.y = p1y > tempy ? p1y : tempy;
    // end inline

    _nodeStackIndex = 0;
    _nodeStack[_nodeStackIndex++] = m_root;
    while (_nodeStackIndex > 0) {
      int node = _nodeStack[--_nodeStackIndex] = m_root;
      if (node == NULL_NODE) {
        continue;
      }

      final AABB nodeAABB = m_aabb[node];
      if (!AABB.testOverlap(nodeAABB, segAABB)) {
        continue;
      }

      // Separating axis for segment (Gino, p80).
      // |dot(v, p1 - c)| > dot(|v|, h)
      // node.aabb.getCenterToOut(c);
      // node.aabb.getExtentsToOut(h);
      cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5;
      cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5;
      hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5;
      hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5;
      tempx = p1x - cx;
      tempy = p1y - cy;
      double separation = (vx * tempx + vy * tempy).abs() - (absVx * hx + absVy * hy);
      if (separation > 0.0) {
        continue;
      }

      int child1 = m_child1[node];
      if (child1 == NULL_NODE) {
        _subInput.p1.x = p1x;
        _subInput.p1.y = p1y;
        _subInput.p2.x = p2x;
        _subInput.p2.y = p2y;
        _subInput.maxFraction = maxFraction;

        double value = callback.raycastCallback(_subInput, node);

        if (value == 0.0) {
          // The client has terminated the ray cast.
          return;
        }

        if (value > 0.0) {
          // Update segment bounding box.
          maxFraction = value;
          // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
          // Vec2.minToOut(p1, temp, segAABB.lowerBound);
          // Vec2.maxToOut(p1, temp, segAABB.upperBound);
          tempx = (p2x - p1x) * maxFraction + p1x;
          tempy = (p2y - p1y) * maxFraction + p1y;
          segAABB.lowerBound.x = p1x < tempx ? p1x : tempx;
          segAABB.lowerBound.y = p1y < tempy ? p1y : tempy;
          segAABB.upperBound.x = p1x > tempx ? p1x : tempx;
          segAABB.upperBound.y = p1y > tempy ? p1y : tempy;
        }
      } else {
        _nodeStack[_nodeStackIndex++] = child1;
        _nodeStack[_nodeStackIndex++] = m_child2[node];
      }
    }
  }

  int computeHeight() {
    return _computeHeight(m_root);
  }

  int _computeHeight(int node) {
    assert(0 <= node && node < _m_nodeCapacity);

    if (m_child1[node] == NULL_NODE) {
      return 0;
    }
    int height1 = _computeHeight(m_child1[node]);
    int height2 = _computeHeight(m_child2[node]);
    return 1 + Math.max(height1, height2);
  }

  /**
   * Validate this tree. For testing.
   */
  void validate() {
    _validateStructure(m_root);
    _validateMetrics(m_root);

    int freeCount = 0;
    int freeNode = _m_freeList;
    while (freeNode != NULL_NODE) {
      assert(0 <= freeNode && freeNode < _m_nodeCapacity);
      freeNode = m_parent[freeNode];
      ++freeCount;
    }

    assert(getHeight() == computeHeight());
    assert(_m_nodeCount + freeCount == _m_nodeCapacity);
  }

  int getHeight() {
    if (m_root == NULL_NODE) {
      return 0;
    }
    return m_height[m_root];
  }

  int getMaxBalance() {
    int maxBalance = 0;
    for (int i = 0; i < _m_nodeCapacity; ++i) {
      if (m_height[i] <= 1) {
        continue;
      }

      assert(m_child1[i] != NULL_NODE);

      int child1 = m_child1[i];
      int child2 = m_child2[i];
      int balance = (m_height[child2] - m_height[child1]).abs();
      maxBalance = Math.max(maxBalance, balance);
    }

    return maxBalance;
  }

  double getAreaRatio() {
    if (m_root == NULL_NODE) {
      return 0.0;
    }

    final int root = m_root;
    double rootArea = m_aabb[root].getPerimeter();

    double totalArea = 0.0;
    for (int i = 0; i < _m_nodeCapacity; ++i) {
      if (m_height[i] < 0) {
        // Free node in pool
        continue;
      }

      totalArea += m_aabb[i].getPerimeter();
    }

    return totalArea / rootArea;
  }

  // /**
  // * Build an optimal tree. Very expensive. For testing.
  // */
  // void rebuildBottomUp() {
  // int[] nodes = new int[_m_nodeCount];
  // int count = 0;
  //
  // // Build array of leaves. Free the rest.
  // for (int i = 0; i < _m_nodeCapacity; ++i) {
  // if (m_nodes[i].height < 0) {
  // // free node in pool
  // continue;
  // }
  //
  // DynamicTreeNode node = m_nodes[i];
  // if (node.isLeaf()) {
  // node.parent = null;
  // nodes[count] = i;
  // ++count;
  // } else {
  // freeNode(node);
  // }
  // }
  //
  // AABB b = new AABB();
  // while (count > 1) {
  // double minCost = Float.MAX_VALUE;
  // int iMin = -1, jMin = -1;
  // for (int i = 0; i < count; ++i) {
  // AABB aabbi = m_nodes[nodes[i]].aabb;
  //
  // for (int j = i + 1; j < count; ++j) {
  // AABB aabbj = m_nodes[nodes[j]].aabb;
  // b.combine(aabbi, aabbj);
  // double cost = b.getPerimeter();
  // if (cost < minCost) {
  // iMin = i;
  // jMin = j;
  // minCost = cost;
  // }
  // }
  // }
  //
  // int index1 = nodes[iMin];
  // int index2 = nodes[jMin];
  // DynamicTreeNode child1 = m_nodes[index1];
  // DynamicTreeNode child2 = m_nodes[index2];
  //
  // DynamicTreeNode parent = allocateNode();
  // parent.child1 = child1;
  // parent.child2 = child2;
  // parent.height = 1 + MathUtils.max(child1.height, child2.height);
  // parent.aabb.combine(child1.aabb, child2.aabb);
  // parent.parent = null;
  //
  // child1.parent = parent;
  // child2.parent = parent;
  //
  // nodes[jMin] = nodes[count - 1];
  // nodes[iMin] = parent.id;
  // --count;
  // }
  //
  // m_root = m_nodes[nodes[0]];
  //
  // validate();
  // }

  int _allocateNode() {
    if (_m_freeList == NULL_NODE) {
      assert(_m_nodeCount == _m_nodeCapacity);
      _m_nodeCapacity *= 2;
      _expandBuffers(_m_nodeCount, _m_nodeCapacity);
    }
    assert(_m_freeList != NULL_NODE);
    int node = _m_freeList;
    _m_freeList = m_parent[node];
    m_parent[node] = NULL_NODE;
    m_child1[node] = NULL_NODE;
    m_height[node] = 0;
    ++_m_nodeCount;
    return node;
  }

  /**
   * returns a node to the pool
   */
  void _freeNode(int node) {
    assert(node != NULL_NODE);
    assert(0 < _m_nodeCount);
    m_parent[node] = _m_freeList != NULL_NODE ? _m_freeList : NULL_NODE;
    m_height[node] = -1;
    _m_freeList = node;
    _m_nodeCount--;
  }

  final AABB _combinedAABB = new AABB();

  void _insertLeaf(int leaf) {
    if (m_root == NULL_NODE) {
      m_root = leaf;
      m_parent[m_root] = NULL_NODE;
      return;
    }

    // find the best sibling
    AABB leafAABB = m_aabb[leaf];
    int index = m_root;
    while (m_child1[index] != NULL_NODE) {
      final int node = index;
      int child1 = m_child1[node];
      int child2 = m_child2[node];
      final AABB nodeAABB = m_aabb[node];
      double area = nodeAABB.getPerimeter();

      _combinedAABB.combine2(nodeAABB, leafAABB);
      double combinedArea = _combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      double cost = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      double inheritanceCost = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      double cost1;
      AABB child1AABB = m_aabb[child1];
      if (m_child1[child1] == NULL_NODE) {
        _combinedAABB.combine2(leafAABB, child1AABB);
        cost1 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child1AABB);
        double oldArea = child1AABB.getPerimeter();
        double newArea = _combinedAABB.getPerimeter();
        cost1 = (newArea - oldArea) + inheritanceCost;
      }

      // Cost of descending into child2
      double cost2;
      AABB child2AABB = m_aabb[child2];
      if (m_child1[child2] == NULL_NODE) {
        _combinedAABB.combine2(leafAABB, child2AABB);
        cost2 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child2AABB);
        double oldArea = child2AABB.getPerimeter();
        double newArea = _combinedAABB.getPerimeter();
        cost2 = newArea - oldArea + inheritanceCost;
      }

      // Descend according to the minimum cost.
      if (cost < cost1 && cost < cost2) {
        break;
      }

      // Descend
      if (cost1 < cost2) {
        index = child1;
      } else {
        index = child2;
      }
    }

    int sibling = index;
    int oldParent = m_parent[sibling];
    final int newParent = _allocateNode();
    m_parent[newParent] = oldParent;
    m_userData[newParent] = null;
    m_aabb[newParent].combine2(leafAABB, m_aabb[sibling]);
    m_height[newParent] = m_height[sibling] + 1;

    if (oldParent != NULL_NODE) {
      // The sibling was not the root.
      if (m_child1[oldParent] == sibling) {
        m_child1[oldParent] = newParent;
      } else {
        m_child2[oldParent] = newParent;
      }

      m_child1[newParent] = sibling;
      m_child2[newParent] = leaf;
      m_parent[sibling] = newParent;
      m_parent[leaf] = newParent;
    } else {
      // The sibling was the root.
      m_child1[newParent] = sibling;
      m_child2[newParent] = leaf;
      m_parent[sibling] = newParent;
      m_parent[leaf] = newParent;
      m_root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    index = m_parent[leaf];
    while (index != NULL_NODE) {
      index = _balance(index);

      int child1 = m_child1[index];
      int child2 = m_child2[index];

      assert(child1 != NULL_NODE);
      assert(child2 != NULL_NODE);

      m_height[index] = 1 + Math.max(m_height[child1], m_height[child2]);
      m_aabb[index].combine2(m_aabb[child1], m_aabb[child2]);

      index = m_parent[index];
    }
    // validate();
  }

  void _removeLeaf(int leaf) {
    if (leaf == m_root) {
      m_root = NULL_NODE;
      return;
    }

    int parent = m_parent[leaf];
    int grandParent = m_parent[parent];
    int parentChild1 = m_child1[parent];
    int parentChild2 = m_child2[parent];
    int sibling;
    if (parentChild1 == leaf) {
      sibling = parentChild2;
    } else {
      sibling = parentChild1;
    }

    if (grandParent != NULL_NODE) {
      // Destroy parent and connect sibling to grandParent.
      if (m_child1[grandParent] == parent) {
        m_child1[grandParent] = sibling;
      } else {
        m_child2[grandParent] = sibling;
      }
      m_parent[sibling] = grandParent;
      _freeNode(parent);

      // Adjust ancestor bounds.
      int index = grandParent;
      while (index != NULL_NODE) {
        index = _balance(index);

        int child1 = m_child1[index];
        int child2 = m_child2[index];

        m_aabb[index].combine2(m_aabb[child1], m_aabb[child2]);
        m_height[index] = 1 + Math.max(m_height[child1], m_height[child2]);

        index = m_parent[index];
      }
    } else {
      m_root = sibling;
      m_parent[sibling] = NULL_NODE;
      _freeNode(parent);
    }

    // validate();
  }

  // Perform a left or right rotation if node A is imbalanced.
  // Returns the new root index.
  int _balance(int iA) {
    assert(iA != NULL_NODE);

    int A = iA;
    if (m_child1[A] == NULL_NODE || m_height[A] < 2) {
      return iA;
    }

    int iB = m_child1[A];
    int iC = m_child2[A];
    assert(0 <= iB && iB < _m_nodeCapacity);
    assert(0 <= iC && iC < _m_nodeCapacity);

    int B = iB;
    int C = iC;

    int balance = m_height[C] - m_height[B];

    // Rotate C up
    if (balance > 1) {
      int iF = m_child1[C];
      int iG = m_child2[C];
      int F = iF;
      int G = iG;
      // assert (F != null);
      // assert (G != null);
      assert(0 <= iF && iF < _m_nodeCapacity);
      assert(0 <= iG && iG < _m_nodeCapacity);

      // Swap A and C
      m_child1[C] = iA;
      int cParent = m_parent[C] = m_parent[A];
      m_parent[A] = iC;

      // A's old parent should point to C
      if (cParent != NULL_NODE) {
        if (m_child1[cParent] == iA) {
          m_child1[cParent] = iC;
        } else {
          assert(m_child2[cParent] == iA);
          m_child2[cParent] = iC;
        }
      } else {
        m_root = iC;
      }

      // Rotate
      if (m_height[F] > m_height[G]) {
        m_child2[C] = iF;
        m_child2[A] = iG;
        m_parent[G] = iA;
        m_aabb[A].combine2(m_aabb[B], m_aabb[G]);
        m_aabb[C].combine2(m_aabb[A], m_aabb[F]);

        m_height[A] = 1 + Math.max(m_height[B], m_height[G]);
        m_height[C] = 1 + Math.max(m_height[A], m_height[F]);
      } else {
        m_child2[C] = iG;
        m_child2[A] = iF;
        m_parent[F] = iA;
        m_aabb[A].combine2(m_aabb[B], m_aabb[F]);
        m_aabb[C].combine2(m_aabb[A], m_aabb[G]);

        m_height[A] = 1 + Math.max(m_height[B], m_height[F]);
        m_height[C] = 1 + Math.max(m_height[A], m_height[G]);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      int iD = m_child1[B];
      int iE = m_child2[B];
      int D = iD;
      int E = iE;
      assert(0 <= iD && iD < _m_nodeCapacity);
      assert(0 <= iE && iE < _m_nodeCapacity);

      // Swap A and B
      m_child1[B] = iA;
      int Bparent = m_parent[B] = m_parent[A];
      m_parent[A] = iB;

      // A's old parent should point to B
      if (Bparent != NULL_NODE) {
        if (m_child1[Bparent] == iA) {
          m_child1[Bparent] = iB;
        } else {
          assert(m_child2[Bparent] == iA);
          m_child2[Bparent] = iB;
        }
      } else {
        m_root = iB;
      }

      // Rotate
      if (m_height[D] > m_height[E]) {
        m_child2[B] = iD;
        m_child1[A] = iE;
        m_parent[E] = iA;
        m_aabb[A].combine2(m_aabb[C], m_aabb[E]);
        m_aabb[B].combine2(m_aabb[A], m_aabb[D]);

        m_height[A] = 1 + Math.max(m_height[C], m_height[E]);
        m_height[B] = 1 + Math.max(m_height[A], m_height[D]);
      } else {
        m_child2[B] = iE;
        m_child1[A] = iD;
        m_parent[D] = iA;
        m_aabb[A].combine2(m_aabb[C], m_aabb[D]);
        m_aabb[B].combine2(m_aabb[A], m_aabb[E]);

        m_height[A] = 1 + Math.max(m_height[C], m_height[D]);
        m_height[B] = 1 + Math.max(m_height[A], m_height[E]);
      }

      return iB;
    }

    return iA;
  }

  void _validateStructure(int node) {
    if (node == NULL_NODE) {
      return;
    }

    if (node == m_root) {
      assert(m_parent[node] == NULL_NODE);
    }

    int child1 = m_child1[node];
    int child2 = m_child2[node];

    if (child1 == NULL_NODE) {
      assert(child1 == NULL_NODE);
      assert(child2 == NULL_NODE);
      assert(m_height[node] == 0);
      return;
    }

    assert(child1 != NULL_NODE && 0 <= child1 && child1 < _m_nodeCapacity);
    assert(child2 != NULL_NODE && 0 <= child2 && child2 < _m_nodeCapacity);

    assert(m_parent[child1] == node);
    assert(m_parent[child2] == node);

    _validateStructure(child1);
    _validateStructure(child2);
  }

  void _validateMetrics(int node) {
    if (node == NULL_NODE) {
      return;
    }

    int child1 = m_child1[node];
    int child2 = m_child2[node];

    if (child1 == NULL_NODE) {
      assert(child1 == NULL_NODE);
      assert(child2 == NULL_NODE);
      assert(m_height[node] == 0);
      return;
    }

    assert(child1 != NULL_NODE && 0 <= child1 && child1 < _m_nodeCapacity);
    assert(child2 != child1 && 0 <= child2 && child2 < _m_nodeCapacity);

    int height1 = m_height[child1];
    int height2 = m_height[child2];
    int height;
    height = 1 + Math.max(height1, height2);
    assert(m_height[node] == height);

    AABB aabb = new AABB();
    aabb.combine2(m_aabb[child1], m_aabb[child2]);

    assert(aabb.lowerBound.equals(m_aabb[node].lowerBound));
    assert(aabb.upperBound.equals(m_aabb[node].upperBound));

    _validateMetrics(child1);
    _validateMetrics(child2);
  }

  void drawTree(DebugDraw argDraw) {
    if (m_root == NULL_NODE) {
      return;
    }
    int height = computeHeight();
    drawTreeX(argDraw, m_root, 0, height);
  }

  final Color3f _color = new Color3f.zero();
  final Vec2 _textVec = new Vec2.zero();

  void drawTreeX(DebugDraw argDraw, int node, int spot, int height) {
    AABB a = m_aabb[node];
    a.getVertices(drawVecs);

    _color.setRGB(1.0, (height - spot) * 1.0 / height, (height - spot) * 1.0 / height);
    argDraw.drawPolygon(drawVecs, 4, _color);

    argDraw.getViewportTranform().getWorldToScreen(a.upperBound, _textVec);
    argDraw.drawStringXY(_textVec.x, _textVec.y, "$node-${(spot + 1)}/$height", _color);

    int c1 = m_child1[node];
    int c2 = m_child2[node];
    if (c1 != NULL_NODE) {
      drawTreeX(argDraw, c1, spot + 1, height);
    }
    if (c2 != NULL_NODE) {
      drawTreeX(argDraw, c2, spot + 1, height);
    }
  }
}
