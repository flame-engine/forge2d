part of forge2d;

/// A dynamic tree arranges data in a binary tree to accelerate queries such as volume queries and
/// ray casts. Leafs are proxies with an AABB. In the tree we expand the proxy AABB by _fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client object to move by
/// small amounts without triggering a tree update.
class DynamicTree implements BroadPhaseStrategy {
  static const int MAX_STACK_SIZE = 64;
  static const int NULL_NODE = -1;

  DynamicTreeNode _root;
  List<DynamicTreeNode> _nodes = List<DynamicTreeNode>.generate(
    16,
    (i) => DynamicTreeNode(i),
  );
  int _nodeCount = 0;
  int _nodeCapacity = 16;

  int _freeList = 0;

  final List<Vector2> drawVecs = List<Vector2>.generate(
    4,
    (_) => Vector2.zero(),
  );
  List<DynamicTreeNode> nodeStack = List<DynamicTreeNode>.generate(
    20,
    (i) => DynamicTreeNode(i),
  );
  int nodeStackIndex = 0;

  DynamicTree() {
    // Build a linked list for the free list.
    for (int i = _nodeCapacity - 1; i >= 0; i--) {
      _nodes[i] = DynamicTreeNode(i);
      _nodes[i].parent = (i == _nodeCapacity - 1) ? null : _nodes[i + 1];
      _nodes[i].height = -1;
    }

    for (int i = 0; i < drawVecs.length; i++) {
      drawVecs[i] = Vector2.zero();
    }
  }

  @override
  int createProxy(final AABB aabb, Object userData) {
    assert(aabb.isValid());
    final DynamicTreeNode node = _allocateNode();
    final int proxyId = node.id;
    // Fatten the aabb
    final AABB nodeAABB = node.aabb;
    nodeAABB.lowerBound.x = aabb.lowerBound.x - settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + settings.aabbExtension;
    node.userData = userData;

    _insertLeaf(proxyId);

    return proxyId;
  }

  @override
  void destroyProxy(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    final DynamicTreeNode node = _nodes[proxyId];
    assert(node.child1 == null);

    _removeLeaf(node);
    _freeNode(node);
  }

  @override
  bool moveProxy(int proxyId, final AABB aabb, Vector2 displacement) {
    assert(aabb.isValid());
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    final DynamicTreeNode node = _nodes[proxyId];
    assert(node.child1 == null);

    final AABB nodeAABB = node.aabb;
    // if (nodeAABB.contains(aabb)) {
    if ((nodeAABB.lowerBound.x <= aabb.lowerBound.x) &&
        (nodeAABB.lowerBound.y <= aabb.lowerBound.y) &&
        (aabb.upperBound.x <= nodeAABB.upperBound.x) &&
        (aabb.upperBound.y <= nodeAABB.upperBound.y)) {
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
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    return _nodes[proxyId].userData;
  }

  @override
  AABB getFatAABB(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    return _nodes[proxyId].aabb;
  }

  @override
  void query(TreeCallback callback, AABB aabb) {
    assert(aabb.isValid());
    nodeStackIndex = 0;
    nodeStack[nodeStackIndex++] = _root;

    while (nodeStackIndex > 0) {
      final DynamicTreeNode node = nodeStack[--nodeStackIndex];
      if (node == null) {
        continue;
      }

      if (AABB.testOverlap(node.aabb, aabb)) {
        if (node.child1 == null) {
          final bool proceed = callback.treeCallback(node.id);
          if (!proceed) {
            return;
          }
        } else {
          if (nodeStack.length - nodeStackIndex - 2 <= 0) {
            final previousSize = nodeStack.length;
            nodeStack = nodeStack +
                List.generate(
                  previousSize,
                  (i) => DynamicTreeNode(previousSize + i),
                );
          }
          nodeStack[nodeStackIndex++] = node.child1;
          nodeStack[nodeStackIndex++] = node.child2;
        }
      }
    }
  }

  final Vector2 _r = Vector2.zero();
  final AABB _aabb = AABB();
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
    double tempX, tempY;
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
    tempX = (p2x - p1x) * maxFraction + p1x;
    tempY = (p2y - p1y) * maxFraction + p1y;
    segAABB.lowerBound.x = math.min(p1x, tempX);
    segAABB.lowerBound.y = math.min(p1y, tempY);
    segAABB.upperBound.x = math.max(p1x, tempX);
    segAABB.upperBound.y = math.max(p1y, tempY);

    nodeStackIndex = 0;
    nodeStack[nodeStackIndex++] = _root;
    while (nodeStackIndex > 0) {
      final DynamicTreeNode node = nodeStack[--nodeStackIndex];
      if (node == null) {
        continue;
      }

      final AABB nodeAABB = node.aabb;
      if (!AABB.testOverlap(nodeAABB, segAABB)) {
        continue;
      }

      // Separating axis for segment (Gino, p80).
      // |dot(v, p1 - c)| > dot(|v|, h)
      cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5;
      cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5;
      hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5;
      hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5;
      tempX = p1x - cx;
      tempY = p1y - cy;
      final double separation =
          (vx * tempX + vy * tempY).abs() - (absVx * hx + absVy * hy);
      if (separation > 0.0) {
        continue;
      }

      if (node.child1 == null) {
        _subInput.p1.x = p1x;
        _subInput.p1.y = p1y;
        _subInput.p2.x = p2x;
        _subInput.p2.y = p2y;
        _subInput.maxFraction = maxFraction;

        final double value = callback.raycastCallback(_subInput, node.id);

        if (value == 0.0) {
          // The client has terminated the ray cast.
          return;
        }

        if (value > 0.0) {
          // Update segment bounding box.
          maxFraction = value;
          tempX = (p2x - p1x) * maxFraction + p1x;
          tempY = (p2y - p1y) * maxFraction + p1y;
          segAABB.lowerBound.x = math.min(p1x, tempX);
          segAABB.lowerBound.y = math.min(p1y, tempY);
          segAABB.upperBound.x = math.max(p1x, tempX);
          segAABB.upperBound.y = math.max(p1y, tempY);
        }
      } else {
        if (nodeStack.length - nodeStackIndex - 2 <= 0) {
          final previousSize = nodeStack.length;
          nodeStack = nodeStack +
              List.generate(
                previousSize,
                (i) => DynamicTreeNode(previousSize + i),
              );
        }
        nodeStack[nodeStackIndex++] = node.child1;
        nodeStack[nodeStackIndex++] = node.child2;
      }
    }
  }

  @override
  int computeHeight() {
    return _computeHeight(_root);
  }

  int _computeHeight(DynamicTreeNode node) {
    assert(0 <= node.id && node.id < _nodeCapacity);

    if (node.child1 == null) {
      return 0;
    }
    final int height1 = _computeHeight(node.child1);
    final int height2 = _computeHeight(node.child2);
    return 1 + math.max<int>(height1, height2);
  }

  /// Validate this tree. For testing.
  void validate() {
    _validateStructure(_root);
    _validateMetrics(_root);

    int freeCount = 0;
    DynamicTreeNode freeNode =
        _freeList != NULL_NODE ? _nodes[_freeList] : null;
    while (freeNode != null) {
      assert(0 <= freeNode.id && freeNode.id < _nodeCapacity);
      assert(freeNode == _nodes[freeNode.id]);
      freeNode = freeNode.parent;
      ++freeCount;
    }

    assert(getHeight() == computeHeight());

    assert(_nodeCount + freeCount == _nodeCapacity);
  }

  @override
  int getHeight() {
    if (_root == null) {
      return 0;
    }
    return _root.height;
  }

  @override
  int getMaxBalance() {
    int maxBalance = 0;
    for (int i = 0; i < _nodeCapacity; ++i) {
      final DynamicTreeNode node = _nodes[i];
      if (node.height <= 1) {
        continue;
      }

      assert(node.child1 != null);

      final DynamicTreeNode child1 = node.child1;
      final DynamicTreeNode child2 = node.child2;
      final int balance = (child2.height - child1.height).abs();
      maxBalance = math.max(maxBalance, balance);
    }

    return maxBalance;
  }

  @override
  double getAreaRatio() {
    if (_root == null) {
      return 0.0;
    }

    final DynamicTreeNode root = _root;
    final double rootArea = root.aabb.getPerimeter();

    double totalArea = 0.0;
    for (int i = 0; i < _nodeCapacity; ++i) {
      final DynamicTreeNode node = _nodes[i];
      if (node.height < 0) {
        // Free node in pool
        continue;
      }

      totalArea += node.aabb.getPerimeter();
    }

    return totalArea / rootArea;
  }

  /// Build an optimal tree. Very expensive. For testing.
  void rebuildBottomUp() {
    final List<int> nodes = List<int>.filled(_nodeCount, 0);
    int count = 0;

    // Build array of leaves. Free the rest.
    for (int i = 0; i < _nodeCapacity; ++i) {
      if (_nodes[i].height < 0) {
        // free node in pool
        continue;
      }

      final DynamicTreeNode node = _nodes[i];
      if (node.child1 == null) {
        node.parent = null;
        nodes[count] = i;
        ++count;
      } else {
        _freeNode(node);
      }
    }

    final AABB b = AABB();
    while (count > 1) {
      double minCost = double.maxFinite;
      int iMin = -1, jMin = -1;
      for (int i = 0; i < count; ++i) {
        final AABB aabbi = _nodes[nodes[i]].aabb;

        for (int j = i + 1; j < count; ++j) {
          final AABB aabbj = _nodes[nodes[j]].aabb;
          b.combine2(aabbi, aabbj);
          final double cost = b.getPerimeter();
          if (cost < minCost) {
            iMin = i;
            jMin = j;
            minCost = cost;
          }
        }
      }

      final int index1 = nodes[iMin];
      final int index2 = nodes[jMin];
      final DynamicTreeNode child1 = _nodes[index1];
      final DynamicTreeNode child2 = _nodes[index2];

      final DynamicTreeNode parent = _allocateNode();
      parent.child1 = child1;
      parent.child2 = child2;
      parent.height = 1 + math.max<int>(child1.height, child2.height);
      parent.aabb.combine2(child1.aabb, child2.aabb);
      parent.parent = null;

      child1.parent = parent;
      child2.parent = parent;

      nodes[jMin] = nodes[count - 1];
      nodes[iMin] = parent.id;
      --count;
    }

    _root = _nodes[nodes[0]];

    validate();
  }

  DynamicTreeNode _allocateNode() {
    if (_freeList == NULL_NODE) {
      assert(_nodeCount == _nodeCapacity);

      _nodes = _nodes +
          List<DynamicTreeNode>.generate(
              _nodeCapacity, (i) => DynamicTreeNode(_nodeCapacity + i));
      _nodeCapacity = _nodes.length;

      // Build a linked list for the free list.
      for (int i = _nodeCapacity - 1; i >= _nodeCount; i--) {
        _nodes[i].parent = (i == _nodeCapacity - 1) ? null : _nodes[i + 1];
        _nodes[i].height = -1;
      }
      _freeList = _nodeCount;
    }
    final int nodeId = _freeList;
    final DynamicTreeNode treeNode = _nodes[nodeId];
    _freeList = treeNode.parent != null ? treeNode.parent.id : NULL_NODE;

    treeNode.parent = null;
    treeNode.child1 = null;
    treeNode.child2 = null;
    treeNode.height = 0;
    treeNode.userData = null;
    ++_nodeCount;
    return treeNode;
  }

  /// returns a node to the pool
  void _freeNode(DynamicTreeNode node) {
    assert(node != null);
    assert(0 < _nodeCount);
    node.parent = _freeList != NULL_NODE ? _nodes[_freeList] : null;
    node.height = -1;
    _freeList = node.id;
    _nodeCount--;
  }

  final AABB _combinedAABB = AABB();

  void _insertLeaf(int leafIndex) {
    final DynamicTreeNode leaf = _nodes[leafIndex];
    if (_root == null) {
      _root = leaf;
      _root.parent = null;
      return;
    }

    // find the best sibling
    final AABB leafAABB = leaf.aabb;
    DynamicTreeNode index = _root;
    while (index.child1 != null) {
      final DynamicTreeNode node = index;
      final DynamicTreeNode child1 = node.child1;
      final DynamicTreeNode child2 = node.child2;

      final double area = node.aabb.getPerimeter();

      _combinedAABB.combine2(node.aabb, leafAABB);
      final double combinedArea = _combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      final double cost = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      final double inheritanceCost = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      double cost1;
      if (child1.child1 == null) {
        _combinedAABB.combine2(leafAABB, child1.aabb);
        cost1 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child1.aabb);
        final double oldArea = child1.aabb.getPerimeter();
        final double newArea = _combinedAABB.getPerimeter();
        cost1 = (newArea - oldArea) + inheritanceCost;
      }

      // Cost of descending into child2
      double cost2;
      if (child2.child1 == null) {
        _combinedAABB.combine2(leafAABB, child2.aabb);
        cost2 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child2.aabb);
        final double oldArea = child2.aabb.getPerimeter();
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

    final DynamicTreeNode sibling = index;
    final DynamicTreeNode oldParent = _nodes[sibling.id].parent;
    final DynamicTreeNode newParent = _allocateNode();
    newParent.parent = oldParent;
    newParent.userData = null;
    newParent.aabb.combine2(leafAABB, sibling.aabb);
    newParent.height = sibling.height + 1;

    if (oldParent != null) {
      // The sibling was not the root.
      if (oldParent.child1 == sibling) {
        oldParent.child1 = newParent;
      } else {
        oldParent.child2 = newParent;
      }

      newParent.child1 = sibling;
      newParent.child2 = leaf;
      sibling.parent = newParent;
      leaf.parent = newParent;
    } else {
      // The sibling was the root.
      newParent.child1 = sibling;
      newParent.child2 = leaf;
      sibling.parent = newParent;
      leaf.parent = newParent;
      _root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    index = leaf.parent;
    while (index != null) {
      index = _balance(index);

      final DynamicTreeNode child1 = index.child1;
      final DynamicTreeNode child2 = index.child2;

      assert(child1 != null);
      assert(child2 != null);

      index.height = 1 + math.max<int>(child1.height, child2.height);
      index.aabb.combine2(child1.aabb, child2.aabb);

      index = index.parent;
    }
    // validate();
  }

  void _removeLeaf(DynamicTreeNode leaf) {
    if (leaf == _root) {
      _root = null;
      return;
    }

    final DynamicTreeNode parent = leaf.parent;
    final DynamicTreeNode grandParent = parent.parent;
    DynamicTreeNode sibling;
    if (parent.child1 == leaf) {
      sibling = parent.child2;
    } else {
      sibling = parent.child1;
    }

    if (grandParent != null) {
      // Destroy parent and connect sibling to grandParent.
      if (grandParent.child1 == parent) {
        grandParent.child1 = sibling;
      } else {
        grandParent.child2 = sibling;
      }
      sibling.parent = grandParent;
      _freeNode(parent);

      // Adjust ancestor bounds.
      DynamicTreeNode index = grandParent;
      while (index != null) {
        index = _balance(index);

        final DynamicTreeNode child1 = index.child1;
        final DynamicTreeNode child2 = index.child2;

        index.aabb.combine2(child1.aabb, child2.aabb);
        index.height = 1 + math.max<int>(child1.height, child2.height);

        index = index.parent;
      }
    } else {
      _root = sibling;
      sibling.parent = null;
      _freeNode(parent);
    }

    // validate();
  }

  // Perform a left or right rotation if node A is imbalanced.
  // Returns the new root index.
  DynamicTreeNode _balance(DynamicTreeNode iA) {
    assert(iA != null);

    final DynamicTreeNode a = iA;
    if (a.child1 == null || a.height < 2) {
      return iA;
    }

    final DynamicTreeNode iB = a.child1;
    final DynamicTreeNode iC = a.child2;
    assert(0 <= iB.id && iB.id < _nodeCapacity);
    assert(0 <= iC.id && iC.id < _nodeCapacity);

    final DynamicTreeNode b = iB;
    final DynamicTreeNode c = iC;

    final int balance = c.height - b.height;

    // Rotate C up
    if (balance > 1) {
      final DynamicTreeNode iF = c.child1;
      final DynamicTreeNode iG = c.child2;
      final DynamicTreeNode f = iF;
      final DynamicTreeNode g = iG;
      assert(f != null);
      assert(g != null);
      assert(0 <= iF.id && iF.id < _nodeCapacity);
      assert(0 <= iG.id && iG.id < _nodeCapacity);

      // Swap A and C
      c.child1 = iA;
      c.parent = a.parent;
      a.parent = iC;

      // A's old parent should point to C
      if (c.parent != null) {
        if (c.parent.child1 == iA) {
          c.parent.child1 = iC;
        } else {
          assert(c.parent.child2 == iA);
          c.parent.child2 = iC;
        }
      } else {
        _root = iC;
      }

      // Rotate
      if (f.height > g.height) {
        c.child2 = iF;
        a.child2 = iG;
        g.parent = iA;
        a.aabb.combine2(b.aabb, g.aabb);
        c.aabb.combine2(a.aabb, f.aabb);

        a.height = 1 + math.max<int>(b.height, g.height);
        c.height = 1 + math.max<int>(a.height, f.height);
      } else {
        c.child2 = iG;
        a.child2 = iF;
        f.parent = iA;
        a.aabb.combine2(b.aabb, f.aabb);
        c.aabb.combine2(a.aabb, g.aabb);

        a.height = 1 + math.max<int>(b.height, f.height);
        c.height = 1 + math.max<int>(a.height, g.height);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      final DynamicTreeNode iD = b.child1;
      final DynamicTreeNode iE = b.child2;
      final DynamicTreeNode d = iD;
      final DynamicTreeNode e = iE;
      assert(0 <= iD.id && iD.id < _nodeCapacity);
      assert(0 <= iE.id && iE.id < _nodeCapacity);

      // Swap A and B
      b.child1 = iA;
      b.parent = a.parent;
      a.parent = iB;

      // A's old parent should point to B
      if (b.parent != null) {
        if (b.parent.child1 == iA) {
          b.parent.child1 = iB;
        } else {
          assert(b.parent.child2 == iA);
          b.parent.child2 = iB;
        }
      } else {
        _root = iB;
      }

      // Rotate
      if (d.height > e.height) {
        b.child2 = iD;
        a.child1 = iE;
        e.parent = iA;
        a.aabb.combine2(c.aabb, e.aabb);
        b.aabb.combine2(a.aabb, d.aabb);

        a.height = 1 + math.max<int>(c.height, e.height);
        b.height = 1 + math.max<int>(a.height, d.height);
      } else {
        b.child2 = iE;
        a.child1 = iD;
        d.parent = iA;
        a.aabb.combine2(c.aabb, d.aabb);
        b.aabb.combine2(a.aabb, e.aabb);

        a.height = 1 + math.max<int>(c.height, d.height);
        b.height = 1 + math.max<int>(a.height, e.height);
      }

      return iB;
    }

    return iA;
  }

  void _validateStructure(DynamicTreeNode node) {
    if (node == null) {
      return;
    }
    assert(node == _nodes[node.id]);

    if (node == _root) {
      assert(node.parent == null);
    }

    final DynamicTreeNode child1 = node.child1;
    final DynamicTreeNode child2 = node.child2;

    if (node.child1 == null) {
      assert(child1 == null);
      assert(child2 == null);
      assert(node.height == 0);
      return;
    }

    assert(child1 != null && 0 <= child1.id && child1.id < _nodeCapacity);
    assert(child2 != null && 0 <= child2.id && child2.id < _nodeCapacity);

    assert(child1.parent == node);
    assert(child2.parent == node);

    _validateStructure(child1);
    _validateStructure(child2);
  }

  void _validateMetrics(DynamicTreeNode node) {
    if (node == null) {
      return;
    }

    final DynamicTreeNode child1 = node.child1;
    final DynamicTreeNode child2 = node.child2;

    if (node.child1 == null) {
      assert(child1 == null);
      assert(child2 == null);
      assert(node.height == 0);
      return;
    }

    assert(child1 != null && 0 <= child1.id && child1.id < _nodeCapacity);
    assert(child2 != null && 0 <= child2.id && child2.id < _nodeCapacity);

    final int height1 = child1.height;
    final int height2 = child2.height;
    int height;
    height = 1 + math.max<int>(height1, height2);
    assert(node.height == height);

    final AABB aabb = AABB();
    aabb.combine2(child1.aabb, child2.aabb);

    assert(aabb.lowerBound == node.aabb.lowerBound);
    assert(aabb.upperBound == node.aabb.upperBound);

    _validateMetrics(child1);
    _validateMetrics(child2);
  }

  @override
  void drawTree(DebugDraw argDraw) {
    if (_root == null) {
      return;
    }
    final int height = computeHeight();
    drawTreeX(argDraw, _root, 0, height);
  }

  final Color3i _color = Color3i.zero();

  void drawTreeX(
    DebugDraw argDraw,
    DynamicTreeNode node,
    int spot,
    int height,
  ) {
    node.aabb.getVertices(drawVecs);

    _color.setFromRGBd(
      1.0,
      (height - spot) * 1.0 / height,
      (height - spot) * 1.0 / height,
    );
    argDraw.drawPolygon(drawVecs, _color);

    final Vector2 textVec =
        argDraw.viewportTransform.getWorldToScreen(node.aabb.upperBound);
    argDraw.drawStringXY(
      textVec.x,
      textVec.y,
      "$node.id-${spot + 1}/$height",
      _color,
    );

    if (node.child1 != null) {
      drawTreeX(argDraw, node.child1, spot + 1, height);
    }
    if (node.child2 != null) {
      drawTreeX(argDraw, node.child2, spot + 1, height);
    }
  }
}
