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

  int _root = NULL_NODE;
  List<AABB> _aabb;
  List<Object> _userData;
  List<int> _parent;
  List<int> _child1;
  List<int> _child2;
  List<int> _height;

  int _nodeCount = 0;
  int _nodeCapacity = 16;

  int _freeList;

  final List<Vector2> drawVecs = new List<Vector2>(4);

  DynamicTreeFlatNodes() {
    _expandBuffers(0, _nodeCapacity);

    for (int i = 0; i < drawVecs.length; i++) {
      drawVecs[i] = new Vector2.zero();
    }
  }

  static AABB allocAABB() => new AABB();
  static Object allocObject() => new Object();

  void _expandBuffers(int oldSize, int newSize) {
    _aabb = BufferUtils.reallocateBufferWithAlloc(
        _aabb, oldSize, newSize, allocAABB);
    _userData = BufferUtils.reallocateBufferWithAlloc(
        _userData, oldSize, newSize, allocObject);
    _parent = BufferUtils.reallocateBufferInt(_parent, oldSize, newSize);
    _child1 = BufferUtils.reallocateBufferInt(_child1, oldSize, newSize);
    _child2 = BufferUtils.reallocateBufferInt(_child2, oldSize, newSize);
    _height = BufferUtils.reallocateBufferInt(_height, oldSize, newSize);

    // Build a linked list for the free list.
    for (int i = oldSize; i < newSize; i++) {
      _aabb[i] = new AABB();
      _parent[i] = (i == newSize - 1) ? NULL_NODE : i + 1;
      _height[i] = -1;
      _child1[i] = -1;
      _child2[i] = -1;
    }
    _freeList = oldSize;
  }

  int createProxy(final AABB aabb, Object userData) {
    final int node = _allocateNode();
    // Fatten the aabb
    final AABB nodeAABB = _aabb[node];
    nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
    _userData[node] = userData;

    _insertLeaf(node);

    return node;
  }

  void destroyProxy(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    assert(_child1[proxyId] == NULL_NODE);

    _removeLeaf(proxyId);
    _freeNode(proxyId);
  }

  bool moveProxy(int proxyId, final AABB aabb, Vector2 displacement) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    final int node = proxyId;
    assert(_child1[node] == NULL_NODE);

    final AABB nodeAABB = _aabb[node];
    // if (nodeAABB.contains(aabb)) {
    if (nodeAABB.lowerBound.x <= aabb.lowerBound.x &&
        nodeAABB.lowerBound.y <= aabb.lowerBound.y &&
        aabb.upperBound.x <= nodeAABB.upperBound.x &&
        aabb.upperBound.y <= nodeAABB.upperBound.y) {
      return false;
    }

    _removeLeaf(node);

    // Extend AABB
    final Vector2 lowerBound = nodeAABB.lowerBound;
    final Vector2 upperBound = nodeAABB.upperBound;
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
    assert(0 <= proxyId && proxyId < _nodeCount);
    return _userData[proxyId];
  }

  AABB getFatAABB(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCount);
    return _aabb[proxyId];
  }

  List<int> _nodeStack = BufferUtils.allocClearIntList(20);
  int _nodeStackIndex = 0;

  void query(TreeCallback callback, AABB aabb) {
    _nodeStackIndex = 0;
    _nodeStack[_nodeStackIndex++] = _root;

    while (_nodeStackIndex > 0) {
      int node = _nodeStack[--_nodeStackIndex];
      if (node == NULL_NODE) {
        continue;
      }

      if (AABB.testOverlap(_aabb[node], aabb)) {
        int child1 = _child1[node];
        if (child1 == NULL_NODE) {
          bool proceed = callback.treeCallback(node);
          if (!proceed) {
            return;
          }
        } else {
          if (_nodeStack.length - _nodeStackIndex - 2 <= 0) {
            _nodeStack = BufferUtils.reallocateBufferInt(
                _nodeStack, _nodeStack.length, _nodeStack.length * 2);
          }
          _nodeStack[_nodeStackIndex++] = child1;
          _nodeStack[_nodeStackIndex++] = _child2[node];
        }
      }
    }
  }

  final Vector2 _r = new Vector2.zero();
  final AABB _aabbTemp = new AABB();
  final RayCastInput _subInput = new RayCastInput();

  void raycast(TreeRayCastCallback callback, RayCastInput input) {
    final Vector2 p1 = input.p1;
    final Vector2 p2 = input.p2;
    double p1x = p1.x, p2x = p2.x, p1y = p1.y, p2y = p2.y;
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
    final AABB segAABB = _aabbTemp;
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
    _nodeStack[_nodeStackIndex++] = _root;
    while (_nodeStackIndex > 0) {
      int node = _nodeStack[--_nodeStackIndex] = _root;
      if (node == NULL_NODE) {
        continue;
      }

      final AABB nodeAABB = _aabb[node];
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
      double separation =
          (vx * tempx + vy * tempy).abs() - (absVx * hx + absVy * hy);
      if (separation > 0.0) {
        continue;
      }

      int child1 = _child1[node];
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
        _nodeStack[_nodeStackIndex++] = _child2[node];
      }
    }
  }

  int computeHeight() {
    return _computeHeight(_root);
  }

  int _computeHeight(int node) {
    assert(0 <= node && node < _nodeCapacity);

    if (_child1[node] == NULL_NODE) {
      return 0;
    }
    int height1 = _computeHeight(_child1[node]);
    int height2 = _computeHeight(_child2[node]);
    return 1 + Math.max<int>(height1, height2);
  }

  /// Validate this tree. For testing.
  void validate() {
    _validateStructure(_root);
    _validateMetrics(_root);

    int freeCount = 0;
    int freeNode = _freeList;
    while (freeNode != NULL_NODE) {
      assert(0 <= freeNode && freeNode < _nodeCapacity);
      freeNode = _parent[freeNode];
      ++freeCount;
    }

    assert(getHeight() == computeHeight());
    assert(_nodeCount + freeCount == _nodeCapacity);
  }

  int getHeight() {
    if (_root == NULL_NODE) {
      return 0;
    }
    return _height[_root];
  }

  int getMaxBalance() {
    int maxBalance = 0;
    for (int i = 0; i < _nodeCapacity; ++i) {
      if (_height[i] <= 1) {
        continue;
      }

      assert(_child1[i] != NULL_NODE);

      int child1 = _child1[i];
      int child2 = _child2[i];
      int balance = (_height[child2] - _height[child1]).abs();
      maxBalance = Math.max(maxBalance, balance);
    }

    return maxBalance;
  }

  double getAreaRatio() {
    if (_root == NULL_NODE) {
      return 0.0;
    }

    final int root = _root;
    double rootArea = _aabb[root].getPerimeter();

    double totalArea = 0.0;
    for (int i = 0; i < _nodeCapacity; ++i) {
      if (_height[i] < 0) {
        // Free node in pool
        continue;
      }

      totalArea += _aabb[i].getPerimeter();
    }

    return totalArea / rootArea;
  }

  int _allocateNode() {
    if (_freeList == NULL_NODE) {
      assert(_nodeCount == _nodeCapacity);
      _nodeCapacity *= 2;
      _expandBuffers(_nodeCount, _nodeCapacity);
    }
    assert(_freeList != NULL_NODE);
    int node = _freeList;
    _freeList = _parent[node];
    _parent[node] = NULL_NODE;
    _child1[node] = NULL_NODE;
    _height[node] = 0;
    ++_nodeCount;
    return node;
  }

  /// returns a node to the pool
  void _freeNode(int node) {
    assert(node != NULL_NODE);
    assert(0 < _nodeCount);
    _parent[node] = _freeList != NULL_NODE ? _freeList : NULL_NODE;
    _height[node] = -1;
    _freeList = node;
    _nodeCount--;
  }

  final AABB _combinedAABB = new AABB();

  void _insertLeaf(int leaf) {
    if (_root == NULL_NODE) {
      _root = leaf;
      _parent[_root] = NULL_NODE;
      return;
    }

    // find the best sibling
    AABB leafAABB = _aabb[leaf];
    int index = _root;
    while (_child1[index] != NULL_NODE) {
      final int node = index;
      int child1 = _child1[node];
      int child2 = _child2[node];
      final AABB nodeAABB = _aabb[node];
      double area = nodeAABB.getPerimeter();

      _combinedAABB.combine2(nodeAABB, leafAABB);
      double combinedArea = _combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      double cost = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      double inheritanceCost = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      double cost1;
      AABB child1AABB = _aabb[child1];
      if (_child1[child1] == NULL_NODE) {
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
      AABB child2AABB = _aabb[child2];
      if (_child1[child2] == NULL_NODE) {
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
    int oldParent = _parent[sibling];
    final int newParent = _allocateNode();
    _parent[newParent] = oldParent;
    _userData[newParent] = null;
    _aabb[newParent].combine2(leafAABB, _aabb[sibling]);
    _height[newParent] = _height[sibling] + 1;

    if (oldParent != NULL_NODE) {
      // The sibling was not the root.
      if (_child1[oldParent] == sibling) {
        _child1[oldParent] = newParent;
      } else {
        _child2[oldParent] = newParent;
      }

      _child1[newParent] = sibling;
      _child2[newParent] = leaf;
      _parent[sibling] = newParent;
      _parent[leaf] = newParent;
    } else {
      // The sibling was the root.
      _child1[newParent] = sibling;
      _child2[newParent] = leaf;
      _parent[sibling] = newParent;
      _parent[leaf] = newParent;
      _root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    index = _parent[leaf];
    while (index != NULL_NODE) {
      index = _balance(index);

      int child1 = _child1[index];
      int child2 = _child2[index];

      assert(child1 != NULL_NODE);
      assert(child2 != NULL_NODE);

      _height[index] = 1 + Math.max<int>(_height[child1], _height[child2]);
      _aabb[index].combine2(_aabb[child1], _aabb[child2]);

      index = _parent[index];
    }
    // validate();
  }

  void _removeLeaf(int leaf) {
    if (leaf == _root) {
      _root = NULL_NODE;
      return;
    }

    int parent = _parent[leaf];
    int grandParent = _parent[parent];
    int parentChild1 = _child1[parent];
    int parentChild2 = _child2[parent];
    int sibling;
    if (parentChild1 == leaf) {
      sibling = parentChild2;
    } else {
      sibling = parentChild1;
    }

    if (grandParent != NULL_NODE) {
      // Destroy parent and connect sibling to grandParent.
      if (_child1[grandParent] == parent) {
        _child1[grandParent] = sibling;
      } else {
        _child2[grandParent] = sibling;
      }
      _parent[sibling] = grandParent;
      _freeNode(parent);

      // Adjust ancestor bounds.
      int index = grandParent;
      while (index != NULL_NODE) {
        index = _balance(index);

        int child1 = _child1[index];
        int child2 = _child2[index];

        _aabb[index].combine2(_aabb[child1], _aabb[child2]);
        _height[index] = 1 + Math.max<int>(_height[child1], _height[child2]);

        index = _parent[index];
      }
    } else {
      _root = sibling;
      _parent[sibling] = NULL_NODE;
      _freeNode(parent);
    }

    // validate();
  }

  // Perform a left or right rotation if node A is imbalanced.
  // Returns the new root index.
  int _balance(int iA) {
    assert(iA != NULL_NODE);

    int A = iA;
    if (_child1[A] == NULL_NODE || _height[A] < 2) {
      return iA;
    }

    int iB = _child1[A];
    int iC = _child2[A];
    assert(0 <= iB && iB < _nodeCapacity);
    assert(0 <= iC && iC < _nodeCapacity);

    int B = iB;
    int C = iC;

    int balance = _height[C] - _height[B];

    // Rotate C up
    if (balance > 1) {
      int iF = _child1[C];
      int iG = _child2[C];
      int F = iF;
      int G = iG;
      // assert (F != null);
      // assert (G != null);
      assert(0 <= iF && iF < _nodeCapacity);
      assert(0 <= iG && iG < _nodeCapacity);

      // Swap A and C
      _child1[C] = iA;
      int cParent = _parent[C] = _parent[A];
      _parent[A] = iC;

      // A's old parent should point to C
      if (cParent != NULL_NODE) {
        if (_child1[cParent] == iA) {
          _child1[cParent] = iC;
        } else {
          assert(_child2[cParent] == iA);
          _child2[cParent] = iC;
        }
      } else {
        _root = iC;
      }

      // Rotate
      if (_height[F] > _height[G]) {
        _child2[C] = iF;
        _child2[A] = iG;
        _parent[G] = iA;
        _aabb[A].combine2(_aabb[B], _aabb[G]);
        _aabb[C].combine2(_aabb[A], _aabb[F]);

        _height[A] = 1 + Math.max<int>(_height[B], _height[G]);
        _height[C] = 1 + Math.max<int>(_height[A], _height[F]);
      } else {
        _child2[C] = iG;
        _child2[A] = iF;
        _parent[F] = iA;
        _aabb[A].combine2(_aabb[B], _aabb[F]);
        _aabb[C].combine2(_aabb[A], _aabb[G]);

        _height[A] = 1 + Math.max<int>(_height[B], _height[F]);
        _height[C] = 1 + Math.max<int>(_height[A], _height[G]);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      int iD = _child1[B];
      int iE = _child2[B];
      int D = iD;
      int E = iE;
      assert(0 <= iD && iD < _nodeCapacity);
      assert(0 <= iE && iE < _nodeCapacity);

      // Swap A and B
      _child1[B] = iA;
      int Bparent = _parent[B] = _parent[A];
      _parent[A] = iB;

      // A's old parent should point to B
      if (Bparent != NULL_NODE) {
        if (_child1[Bparent] == iA) {
          _child1[Bparent] = iB;
        } else {
          assert(_child2[Bparent] == iA);
          _child2[Bparent] = iB;
        }
      } else {
        _root = iB;
      }

      // Rotate
      if (_height[D] > _height[E]) {
        _child2[B] = iD;
        _child1[A] = iE;
        _parent[E] = iA;
        _aabb[A].combine2(_aabb[C], _aabb[E]);
        _aabb[B].combine2(_aabb[A], _aabb[D]);

        _height[A] = 1 + Math.max<int>(_height[C], _height[E]);
        _height[B] = 1 + Math.max<int>(_height[A], _height[D]);
      } else {
        _child2[B] = iE;
        _child1[A] = iD;
        _parent[D] = iA;
        _aabb[A].combine2(_aabb[C], _aabb[D]);
        _aabb[B].combine2(_aabb[A], _aabb[E]);

        _height[A] = 1 + Math.max<int>(_height[C], _height[D]);
        _height[B] = 1 + Math.max<int>(_height[A], _height[E]);
      }

      return iB;
    }

    return iA;
  }

  void _validateStructure(int node) {
    if (node == NULL_NODE) {
      return;
    }

    if (node == _root) {
      assert(_parent[node] == NULL_NODE);
    }

    int child1 = _child1[node];
    int child2 = _child2[node];

    if (child1 == NULL_NODE) {
      assert(child1 == NULL_NODE);
      assert(child2 == NULL_NODE);
      assert(_height[node] == 0);
      return;
    }

    assert(child1 != NULL_NODE && 0 <= child1 && child1 < _nodeCapacity);
    assert(child2 != NULL_NODE && 0 <= child2 && child2 < _nodeCapacity);

    assert(_parent[child1] == node);
    assert(_parent[child2] == node);

    _validateStructure(child1);
    _validateStructure(child2);
  }

  void _validateMetrics(int node) {
    if (node == NULL_NODE) {
      return;
    }

    int child1 = _child1[node];
    int child2 = _child2[node];

    if (child1 == NULL_NODE) {
      assert(child1 == NULL_NODE);
      assert(child2 == NULL_NODE);
      assert(_height[node] == 0);
      return;
    }

    assert(child1 != NULL_NODE && 0 <= child1 && child1 < _nodeCapacity);
    assert(child2 != child1 && 0 <= child2 && child2 < _nodeCapacity);

    int height1 = _height[child1];
    int height2 = _height[child2];
    int height;
    height = 1 + Math.max<int>(height1, height2);
    assert(_height[node] == height);

    AABB aabb = new AABB();
    aabb.combine2(_aabb[child1], _aabb[child2]);

    assert(MathUtils.vector2Equals(aabb.lowerBound, _aabb[node].lowerBound));
    assert(MathUtils.vector2Equals(aabb.upperBound, _aabb[node].upperBound));

    _validateMetrics(child1);
    _validateMetrics(child2);
  }

  void drawTree(DebugDraw argDraw) {
    if (_root == NULL_NODE) {
      return;
    }
    int height = computeHeight();
    drawTreeX(argDraw, _root, 0, height);
  }

  final Color3i _color = new Color3i.zero();

  void drawTreeX(DebugDraw argDraw, int node, int spot, int height) {
    AABB a = _aabb[node];
    a.getVertices(drawVecs);

    _color.setFromRGBd(
        1.0, (height - spot) * 1.0 / height, (height - spot) * 1.0 / height);
    argDraw.drawPolygon(drawVecs, 4, _color);

    Vector2 textVec =
        argDraw.getViewportTranform().getWorldToScreen(a.upperBound);
    argDraw.drawStringXY(
        textVec.x, textVec.y, "$node-${(spot + 1)}/$height", _color);

    int c1 = _child1[node];
    int c2 = _child2[node];
    if (c1 != NULL_NODE) {
      drawTreeX(argDraw, c1, spot + 1, height);
    }
    if (c2 != NULL_NODE) {
      drawTreeX(argDraw, c2, spot + 1, height);
    }
  }
}
