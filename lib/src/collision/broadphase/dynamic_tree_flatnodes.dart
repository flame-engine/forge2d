part of forge2d;

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

  final List<Vector2> drawVecs = List<Vector2>(4);

  DynamicTreeFlatNodes() {
    _expandBuffers(0, _nodeCapacity);

    for (int i = 0; i < drawVecs.length; i++) {
      drawVecs[i] = Vector2.zero();
    }
  }

  static AABB allocAABB() => AABB();
  static Object allocObject() => Object();

  void _expandBuffers(int oldSize, int newSize) {
    _aabb = buffer_utils.reallocateBufferWithAlloc(
        _aabb, oldSize, newSize, allocAABB);
    _userData = buffer_utils.reallocateBufferWithAlloc(
        _userData, oldSize, newSize, allocObject);
    _parent = buffer_utils.reallocateBufferInt(_parent, oldSize, newSize);
    _child1 = buffer_utils.reallocateBufferInt(_child1, oldSize, newSize);
    _child2 = buffer_utils.reallocateBufferInt(_child2, oldSize, newSize);
    _height = buffer_utils.reallocateBufferInt(_height, oldSize, newSize);

    // Build a linked list for the free list.
    for (int i = oldSize; i < newSize; i++) {
      _aabb[i] = AABB();
      _parent[i] = (i == newSize - 1) ? NULL_NODE : i + 1;
      _height[i] = -1;
      _child1[i] = -1;
      _child2[i] = -1;
    }
    _freeList = oldSize;
  }

  @override
  int createProxy(final AABB aabb, Object userData) {
    final int node = _allocateNode();
    // Fatten the aabb
    final AABB nodeAABB = _aabb[node];
    nodeAABB.lowerBound.x = aabb.lowerBound.x - settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + settings.aabbExtension;
    _userData[node] = userData;

    _insertLeaf(node);

    return node;
  }

  @override
  void destroyProxy(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    assert(_child1[proxyId] == NULL_NODE);

    _removeLeaf(proxyId);
    _freeNode(proxyId);
  }

  @override
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
    lowerBound.x = aabb.lowerBound.x - settings.aabbExtension;
    lowerBound.y = aabb.lowerBound.y - settings.aabbExtension;
    upperBound.x = aabb.upperBound.x + settings.aabbExtension;
    upperBound.y = aabb.upperBound.y + settings.aabbExtension;

    // Predict AABB displacement.
    final double dx = displacement.x * settings.aabbMultiplier;
    final double dy = displacement.y * settings.aabbMultiplier;
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

  @override
  Object getUserData(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCount);
    return _userData[proxyId];
  }

  @override
  AABB getFatAABB(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCount);
    return _aabb[proxyId];
  }

  List<int> _nodeStack = buffer_utils.intList(20);
  int _nodeStackIndex = 0;

  @override
  void query(TreeCallback callback, AABB aabb) {
    _nodeStackIndex = 0;
    _nodeStack[_nodeStackIndex++] = _root;

    while (_nodeStackIndex > 0) {
      final int node = _nodeStack[--_nodeStackIndex];
      if (node == NULL_NODE) {
        continue;
      }

      if (AABB.testOverlap(_aabb[node], aabb)) {
        final int child1 = _child1[node];
        if (child1 == NULL_NODE) {
          final bool proceed = callback.treeCallback(node);
          if (!proceed) {
            return;
          }
        } else {
          if (_nodeStack.length - _nodeStackIndex - 2 <= 0) {
            _nodeStack = buffer_utils.reallocateBufferInt(
                _nodeStack, _nodeStack.length, _nodeStack.length * 2);
          }
          _nodeStack[_nodeStackIndex++] = child1;
          _nodeStack[_nodeStackIndex++] = _child2[node];
        }
      }
    }
  }

  final Vector2 _r = Vector2.zero();
  final AABB _aabbTemp = AABB();
  final RayCastInput _subInput = RayCastInput();

  @override
  void raycast(TreeRayCastCallback callback, RayCastInput input) {
    final Vector2 p1 = input.p1;
    final Vector2 p2 = input.p2;
    final double p1x = p1.x, p2x = p2.x, p1y = p1.y, p2y = p2.y;
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
    tempx = (p2x - p1x) * maxFraction + p1x;
    tempy = (p2y - p1y) * maxFraction + p1y;
    segAABB.lowerBound.x = p1x < tempx ? p1x : tempx;
    segAABB.lowerBound.y = p1y < tempy ? p1y : tempy;
    segAABB.upperBound.x = p1x > tempx ? p1x : tempx;
    segAABB.upperBound.y = p1y > tempy ? p1y : tempy;

    _nodeStackIndex = 0;
    _nodeStack[_nodeStackIndex++] = _root;
    while (_nodeStackIndex > 0) {
      final int node = _nodeStack[--_nodeStackIndex] = _root;
      if (node == NULL_NODE) {
        continue;
      }

      final AABB nodeAABB = _aabb[node];
      if (!AABB.testOverlap(nodeAABB, segAABB)) {
        continue;
      }

      // Separating axis for segment (Gino, p80).
      // |dot(v, p1 - c)| > dot(|v|, h)
      cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5;
      cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5;
      hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5;
      hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5;
      tempx = p1x - cx;
      tempy = p1y - cy;
      final double separation =
          (vx * tempx + vy * tempy).abs() - (absVx * hx + absVy * hy);
      if (separation > 0.0) {
        continue;
      }

      final int child1 = _child1[node];
      if (child1 == NULL_NODE) {
        _subInput.p1.x = p1x;
        _subInput.p1.y = p1y;
        _subInput.p2.x = p2x;
        _subInput.p2.y = p2y;
        _subInput.maxFraction = maxFraction;

        final double value = callback.raycastCallback(_subInput, node);

        if (value == 0.0) {
          // The client has terminated the ray cast.
          return;
        }

        if (value > 0.0) {
          // Update segment bounding box.
          maxFraction = value;
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

  @override
  int computeHeight() {
    return _computeHeight(_root);
  }

  int _computeHeight(int node) {
    assert(0 <= node && node < _nodeCapacity);

    if (_child1[node] == NULL_NODE) {
      return 0;
    }
    final int height1 = _computeHeight(_child1[node]);
    final int height2 = _computeHeight(_child2[node]);
    return 1 + math.max<int>(height1, height2);
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

  @override
  int getHeight() {
    if (_root == NULL_NODE) {
      return 0;
    }
    return _height[_root];
  }

  @override
  int getMaxBalance() {
    int maxBalance = 0;
    for (int i = 0; i < _nodeCapacity; ++i) {
      if (_height[i] <= 1) {
        continue;
      }

      assert(_child1[i] != NULL_NODE);

      final int child1 = _child1[i];
      final int child2 = _child2[i];
      final int balance = (_height[child2] - _height[child1]).abs();
      maxBalance = math.max(maxBalance, balance);
    }

    return maxBalance;
  }

  @override
  double getAreaRatio() {
    if (_root == NULL_NODE) {
      return 0.0;
    }

    final int root = _root;
    final double rootArea = _aabb[root].getPerimeter();

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
    final int node = _freeList;
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

  final AABB _combinedAABB = AABB();

  void _insertLeaf(int leaf) {
    if (_root == NULL_NODE) {
      _root = leaf;
      _parent[_root] = NULL_NODE;
      return;
    }

    // find the best sibling
    final AABB leafAABB = _aabb[leaf];
    int index = _root;
    while (_child1[index] != NULL_NODE) {
      final int node = index;
      final int child1 = _child1[node];
      final int child2 = _child2[node];
      final AABB nodeAABB = _aabb[node];
      final double area = nodeAABB.getPerimeter();

      _combinedAABB.combine2(nodeAABB, leafAABB);
      final double combinedArea = _combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      final double cost = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      final double inheritanceCost = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      double cost1;
      final AABB child1AABB = _aabb[child1];
      if (_child1[child1] == NULL_NODE) {
        _combinedAABB.combine2(leafAABB, child1AABB);
        cost1 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child1AABB);
        final double oldArea = child1AABB.getPerimeter();
        final double newArea = _combinedAABB.getPerimeter();
        cost1 = (newArea - oldArea) + inheritanceCost;
      }

      // Cost of descending into child2
      double cost2;
      final AABB child2AABB = _aabb[child2];
      if (_child1[child2] == NULL_NODE) {
        _combinedAABB.combine2(leafAABB, child2AABB);
        cost2 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child2AABB);
        final double oldArea = child2AABB.getPerimeter();
        final double newArea = _combinedAABB.getPerimeter();
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

    final int sibling = index;
    final int oldParent = _parent[sibling];
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

      final int child1 = _child1[index];
      final int child2 = _child2[index];

      assert(child1 != NULL_NODE);
      assert(child2 != NULL_NODE);

      _height[index] = 1 + math.max<int>(_height[child1], _height[child2]);
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

    final int parent = _parent[leaf];
    final int grandParent = _parent[parent];
    final int parentChild1 = _child1[parent];
    final int parentChild2 = _child2[parent];
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

        final int child1 = _child1[index];
        final int child2 = _child2[index];

        _aabb[index].combine2(_aabb[child1], _aabb[child2]);
        _height[index] = 1 + math.max<int>(_height[child1], _height[child2]);

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

    final int a = iA;
    if (_child1[a] == NULL_NODE || _height[a] < 2) {
      return iA;
    }

    final int iB = _child1[a];
    final int iC = _child2[a];
    assert(0 <= iB && iB < _nodeCapacity);
    assert(0 <= iC && iC < _nodeCapacity);

    final int b = iB;
    final int c = iC;

    final int balance = _height[c] - _height[b];

    // Rotate C up
    if (balance > 1) {
      final int iF = _child1[c];
      final int iG = _child2[c];
      final int f = iF;
      final int g = iG;
      assert(0 <= iF && iF < _nodeCapacity);
      assert(0 <= iG && iG < _nodeCapacity);

      // Swap A and C
      _child1[c] = iA;
      final int cParent = _parent[c] = _parent[a];
      _parent[a] = iC;

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
      if (_height[f] > _height[g]) {
        _child2[c] = iF;
        _child2[a] = iG;
        _parent[g] = iA;
        _aabb[a].combine2(_aabb[b], _aabb[g]);
        _aabb[c].combine2(_aabb[a], _aabb[f]);

        _height[a] = 1 + math.max<int>(_height[b], _height[g]);
        _height[c] = 1 + math.max<int>(_height[a], _height[f]);
      } else {
        _child2[c] = iG;
        _child2[a] = iF;
        _parent[f] = iA;
        _aabb[a].combine2(_aabb[b], _aabb[f]);
        _aabb[c].combine2(_aabb[a], _aabb[g]);

        _height[a] = 1 + math.max<int>(_height[b], _height[f]);
        _height[c] = 1 + math.max<int>(_height[a], _height[g]);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      final int iD = _child1[b];
      final int iE = _child2[b];
      final int d = iD;
      final int e = iE;
      assert(0 <= iD && iD < _nodeCapacity);
      assert(0 <= iE && iE < _nodeCapacity);

      // Swap A and B
      _child1[b] = iA;
      final int bParent = _parent[b] = _parent[a];
      _parent[a] = iB;

      // A's old parent should point to B
      if (bParent != NULL_NODE) {
        if (_child1[bParent] == iA) {
          _child1[bParent] = iB;
        } else {
          assert(_child2[bParent] == iA);
          _child2[bParent] = iB;
        }
      } else {
        _root = iB;
      }

      // Rotate
      if (_height[d] > _height[e]) {
        _child2[b] = iD;
        _child1[a] = iE;
        _parent[e] = iA;
        _aabb[a].combine2(_aabb[c], _aabb[e]);
        _aabb[b].combine2(_aabb[a], _aabb[d]);

        _height[a] = 1 + math.max<int>(_height[c], _height[e]);
        _height[b] = 1 + math.max<int>(_height[a], _height[d]);
      } else {
        _child2[b] = iE;
        _child1[a] = iD;
        _parent[d] = iA;
        _aabb[a].combine2(_aabb[c], _aabb[d]);
        _aabb[b].combine2(_aabb[a], _aabb[e]);

        _height[a] = 1 + math.max<int>(_height[c], _height[d]);
        _height[b] = 1 + math.max<int>(_height[a], _height[e]);
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

    final int child1 = _child1[node];
    final int child2 = _child2[node];

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

    final int child1 = _child1[node];
    final int child2 = _child2[node];

    if (child1 == NULL_NODE) {
      assert(child1 == NULL_NODE);
      assert(child2 == NULL_NODE);
      assert(_height[node] == 0);
      return;
    }

    assert(child1 != NULL_NODE && 0 <= child1 && child1 < _nodeCapacity);
    assert(child2 != child1 && 0 <= child2 && child2 < _nodeCapacity);

    final int height1 = _height[child1];
    final int height2 = _height[child2];
    final int height = 1 + math.max<int>(height1, height2);
    assert(_height[node] == height);

    final AABB aabb = AABB();
    aabb.combine2(_aabb[child1], _aabb[child2]);

    assert(aabb.lowerBound == _aabb[node].lowerBound);
    assert(aabb.upperBound == _aabb[node].upperBound);

    _validateMetrics(child1);
    _validateMetrics(child2);
  }

  @override
  void drawTree(DebugDraw argDraw) {
    if (_root == NULL_NODE) {
      return;
    }
    final int height = computeHeight();
    drawTreeX(argDraw, _root, 0, height);
  }

  final Color3i _color = Color3i.zero();

  void drawTreeX(DebugDraw argDraw, int node, int spot, int height) {
    final AABB a = _aabb[node];
    a.getVertices(drawVecs);

    _color.setFromRGBd(
        1.0, (height - spot) * 1.0 / height, (height - spot) * 1.0 / height);
    argDraw.drawPolygon(drawVecs, 4, _color);

    final Vector2 textVec =
        argDraw.getViewportTranform().getWorldToScreen(a.upperBound);
    argDraw.drawStringXY(
        textVec.x, textVec.y, "$node-${spot + 1}/$height", _color);

    final int c1 = _child1[node];
    final int c2 = _child2[node];
    if (c1 != NULL_NODE) {
      drawTreeX(argDraw, c1, spot + 1, height);
    }
    if (c2 != NULL_NODE) {
      drawTreeX(argDraw, c2, spot + 1, height);
    }
  }
}
