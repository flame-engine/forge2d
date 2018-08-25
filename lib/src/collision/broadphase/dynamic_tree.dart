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
 * A dynamic tree arranges data in a binary tree to accelerate queries such as volume queries and
 * ray casts. Leafs are proxies with an AABB. In the tree we expand the proxy AABB by _fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client object to move by
 * small amounts without triggering a tree update.
 * 
 * @author daniel
 */
class DynamicTree implements BroadPhaseStrategy {
  static const int MAX_STACK_SIZE = 64;
  static const int NULL_NODE = -1;

  DynamicTreeNode _root;
  List<DynamicTreeNode> _nodes = new List<DynamicTreeNode>(16);
  int _nodeCount = 0;
  int _nodeCapacity = 16;

  int _freeList = 0;

  final List<Vector2> drawVecs = new List<Vector2>(4);
  List<DynamicTreeNode> nodeStack = new List<DynamicTreeNode>(20);
  int nodeStackIndex = 0;

  DynamicTree() {
    // Build a linked list for the free list.
    for (int i = _nodeCapacity - 1; i >= 0; i--) {
      _nodes[i] = new DynamicTreeNode(i);
      _nodes[i].parent = (i == _nodeCapacity - 1) ? null : _nodes[i + 1];
      _nodes[i].height = -1;
    }

    for (int i = 0; i < drawVecs.length; i++) {
      drawVecs[i] = new Vector2.zero();
    }
  }

  int createProxy(final AABB aabb, Object userData) {
    assert(aabb.isValid());
    final DynamicTreeNode node = _allocateNode();
    int proxyId = node.id;
    // Fatten the aabb
    final AABB nodeAABB = node.aabb;
    nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
    nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
    nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
    nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
    node.userData = userData;

    _insertLeaf(proxyId);

    return proxyId;
  }

  void destroyProxy(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    DynamicTreeNode node = _nodes[proxyId];
    assert(node.child1 == null);

    _removeLeaf(node);
    _freeNode(node);
  }

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
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    return _nodes[proxyId].userData;
  }

  AABB getFatAABB(int proxyId) {
    assert(0 <= proxyId && proxyId < _nodeCapacity);
    return _nodes[proxyId].aabb;
  }

  void query(TreeCallback callback, AABB aabb) {
    assert(aabb.isValid());
    nodeStackIndex = 0;
    nodeStack[nodeStackIndex++] = _root;

    while (nodeStackIndex > 0) {
      DynamicTreeNode node = nodeStack[--nodeStackIndex];
      if (node == null) {
        continue;
      }

      if (AABB.testOverlap(node.aabb, aabb)) {
        if (node.child1 == null) {
          bool proceed = callback.treeCallback(node.id);
          if (!proceed) {
            return;
          }
        } else {
          if (nodeStack.length - nodeStackIndex - 2 <= 0) {
            List<DynamicTreeNode> newBuffer =
                new List<DynamicTreeNode>(nodeStack.length * 2);
            BufferUtils.arraycopy(nodeStack, 0, newBuffer, 0, nodeStack.length);
            nodeStack = newBuffer;
          }
          nodeStack[nodeStackIndex++] = node.child1;
          nodeStack[nodeStackIndex++] = node.child2;
        }
      }
    }
  }

  final Vector2 _r = new Vector2.zero();
  final AABB _aabb = new AABB();
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

      if (node.child1 == null) {
        _subInput.p1.x = p1x;
        _subInput.p1.y = p1y;
        _subInput.p2.x = p2x;
        _subInput.p2.y = p2y;
        _subInput.maxFraction = maxFraction;

        double value = callback.raycastCallback(_subInput, node.id);

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
        if (nodeStack.length - nodeStackIndex - 2 <= 0) {
          List<DynamicTreeNode> newBuffer =
              new List<DynamicTreeNode>(nodeStack.length * 2);
          BufferUtils.arraycopy(nodeStack, 0, newBuffer, 0, nodeStack.length);
          nodeStack = newBuffer;
        }
        nodeStack[nodeStackIndex++] = node.child1;
        nodeStack[nodeStackIndex++] = node.child2;
      }
    }
  }

  int computeHeight() {
    return _computeHeight(_root);
  }

  int _computeHeight(DynamicTreeNode node) {
    assert(0 <= node.id && node.id < _nodeCapacity);

    if (node.child1 == null) {
      return 0;
    }
    int height1 = _computeHeight(node.child1);
    int height2 = _computeHeight(node.child2);
    return 1 + Math.max(height1, height2);
  }

  /**
   * Validate this tree. For testing.
   */
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

  int getHeight() {
    if (_root == null) {
      return 0;
    }
    return _root.height;
  }

  int getMaxBalance() {
    int maxBalance = 0;
    for (int i = 0; i < _nodeCapacity; ++i) {
      final DynamicTreeNode node = _nodes[i];
      if (node.height <= 1) {
        continue;
      }

      assert((node.child1 == null) == false);

      DynamicTreeNode child1 = node.child1;
      DynamicTreeNode child2 = node.child2;
      int balance = (child2.height - child1.height).abs();
      maxBalance = Math.max(maxBalance, balance);
    }

    return maxBalance;
  }

  double getAreaRatio() {
    if (_root == null) {
      return 0.0;
    }

    final DynamicTreeNode root = _root;
    double rootArea = root.aabb.getPerimeter();

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

  /**
   * Build an optimal tree. Very expensive. For testing.
   */
  void rebuildBottomUp() {
    List<int> nodes = BufferUtils.allocClearIntList(_nodeCount);
    int count = 0;

    // Build array of leaves. Free the rest.
    for (int i = 0; i < _nodeCapacity; ++i) {
      if (_nodes[i].height < 0) {
        // free node in pool
        continue;
      }

      DynamicTreeNode node = _nodes[i];
      if (node.child1 == null) {
        node.parent = null;
        nodes[count] = i;
        ++count;
      } else {
        _freeNode(node);
      }
    }

    AABB b = new AABB();
    while (count > 1) {
      double minCost = double.maxFinite;
      int iMin = -1, jMin = -1;
      for (int i = 0; i < count; ++i) {
        AABB aabbi = _nodes[nodes[i]].aabb;

        for (int j = i + 1; j < count; ++j) {
          AABB aabbj = _nodes[nodes[j]].aabb;
          b.combine2(aabbi, aabbj);
          double cost = b.getPerimeter();
          if (cost < minCost) {
            iMin = i;
            jMin = j;
            minCost = cost;
          }
        }
      }

      int index1 = nodes[iMin];
      int index2 = nodes[jMin];
      DynamicTreeNode child1 = _nodes[index1];
      DynamicTreeNode child2 = _nodes[index2];

      DynamicTreeNode parent = _allocateNode();
      parent.child1 = child1;
      parent.child2 = child2;
      parent.height = 1 + Math.max(child1.height, child2.height);
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

      List<DynamicTreeNode> old = _nodes;
      _nodeCapacity *= 2;
      _nodes = new List<DynamicTreeNode>(_nodeCapacity);
      BufferUtils.arraycopy(old, 0, _nodes, 0, old.length);

      // Build a linked list for the free list.
      for (int i = _nodeCapacity - 1; i >= _nodeCount; i--) {
        _nodes[i] = new DynamicTreeNode(i);
        _nodes[i].parent = (i == _nodeCapacity - 1) ? null : _nodes[i + 1];
        _nodes[i].height = -1;
      }
      _freeList = _nodeCount;
    }
    int nodeId = _freeList;
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

  /**
   * returns a node to the pool
   */
  void _freeNode(DynamicTreeNode node) {
    assert(node != null);
    assert(0 < _nodeCount);
    node.parent = _freeList != NULL_NODE ? _nodes[_freeList] : null;
    node.height = -1;
    _freeList = node.id;
    _nodeCount--;
  }

  final AABB _combinedAABB = new AABB();

  void _insertLeaf(int leaf_index) {
    DynamicTreeNode leaf = _nodes[leaf_index];
    if (_root == null) {
      _root = leaf;
      _root.parent = null;
      return;
    }

    // find the best sibling
    AABB leafAABB = leaf.aabb;
    DynamicTreeNode index = _root;
    while (index.child1 != null) {
      final DynamicTreeNode node = index;
      DynamicTreeNode child1 = node.child1;
      DynamicTreeNode child2 = node.child2;

      double area = node.aabb.getPerimeter();

      _combinedAABB.combine2(node.aabb, leafAABB);
      double combinedArea = _combinedAABB.getPerimeter();

      // Cost of creating a new parent for this node and the new leaf
      double cost = 2.0 * combinedArea;

      // Minimum cost of pushing the leaf further down the tree
      double inheritanceCost = 2.0 * (combinedArea - area);

      // Cost of descending into child1
      double cost1;
      if (child1.child1 == null) {
        _combinedAABB.combine2(leafAABB, child1.aabb);
        cost1 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child1.aabb);
        double oldArea = child1.aabb.getPerimeter();
        double newArea = _combinedAABB.getPerimeter();
        cost1 = (newArea - oldArea) + inheritanceCost;
      }

      // Cost of descending into child2
      double cost2;
      if (child2.child1 == null) {
        _combinedAABB.combine2(leafAABB, child2.aabb);
        cost2 = _combinedAABB.getPerimeter() + inheritanceCost;
      } else {
        _combinedAABB.combine2(leafAABB, child2.aabb);
        double oldArea = child2.aabb.getPerimeter();
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

    DynamicTreeNode sibling = index;
    DynamicTreeNode oldParent = _nodes[sibling.id].parent;
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

      DynamicTreeNode child1 = index.child1;
      DynamicTreeNode child2 = index.child2;

      assert(child1 != null);
      assert(child2 != null);

      index.height = 1 + Math.max(child1.height, child2.height);
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

    DynamicTreeNode parent = leaf.parent;
    DynamicTreeNode grandParent = parent.parent;
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

        DynamicTreeNode child1 = index.child1;
        DynamicTreeNode child2 = index.child2;

        index.aabb.combine2(child1.aabb, child2.aabb);
        index.height = 1 + Math.max(child1.height, child2.height);

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

    DynamicTreeNode A = iA;
    if (A.child1 == null || A.height < 2) {
      return iA;
    }

    DynamicTreeNode iB = A.child1;
    DynamicTreeNode iC = A.child2;
    assert(0 <= iB.id && iB.id < _nodeCapacity);
    assert(0 <= iC.id && iC.id < _nodeCapacity);

    DynamicTreeNode B = iB;
    DynamicTreeNode C = iC;

    int balance = C.height - B.height;

    // Rotate C up
    if (balance > 1) {
      DynamicTreeNode iF = C.child1;
      DynamicTreeNode iG = C.child2;
      DynamicTreeNode F = iF;
      DynamicTreeNode G = iG;
      assert(F != null);
      assert(G != null);
      assert(0 <= iF.id && iF.id < _nodeCapacity);
      assert(0 <= iG.id && iG.id < _nodeCapacity);

      // Swap A and C
      C.child1 = iA;
      C.parent = A.parent;
      A.parent = iC;

      // A's old parent should point to C
      if (C.parent != null) {
        if (C.parent.child1 == iA) {
          C.parent.child1 = iC;
        } else {
          assert(C.parent.child2 == iA);
          C.parent.child2 = iC;
        }
      } else {
        _root = iC;
      }

      // Rotate
      if (F.height > G.height) {
        C.child2 = iF;
        A.child2 = iG;
        G.parent = iA;
        A.aabb.combine2(B.aabb, G.aabb);
        C.aabb.combine2(A.aabb, F.aabb);

        A.height = 1 + Math.max(B.height, G.height);
        C.height = 1 + Math.max(A.height, F.height);
      } else {
        C.child2 = iG;
        A.child2 = iF;
        F.parent = iA;
        A.aabb.combine2(B.aabb, F.aabb);
        C.aabb.combine2(A.aabb, G.aabb);

        A.height = 1 + Math.max(B.height, F.height);
        C.height = 1 + Math.max(A.height, G.height);
      }

      return iC;
    }

    // Rotate B up
    if (balance < -1) {
      DynamicTreeNode iD = B.child1;
      DynamicTreeNode iE = B.child2;
      DynamicTreeNode D = iD;
      DynamicTreeNode E = iE;
      assert(0 <= iD.id && iD.id < _nodeCapacity);
      assert(0 <= iE.id && iE.id < _nodeCapacity);

      // Swap A and B
      B.child1 = iA;
      B.parent = A.parent;
      A.parent = iB;

      // A's old parent should point to B
      if (B.parent != null) {
        if (B.parent.child1 == iA) {
          B.parent.child1 = iB;
        } else {
          assert(B.parent.child2 == iA);
          B.parent.child2 = iB;
        }
      } else {
        _root = iB;
      }

      // Rotate
      if (D.height > E.height) {
        B.child2 = iD;
        A.child1 = iE;
        E.parent = iA;
        A.aabb.combine2(C.aabb, E.aabb);
        B.aabb.combine2(A.aabb, D.aabb);

        A.height = 1 + Math.max(C.height, E.height);
        B.height = 1 + Math.max(A.height, D.height);
      } else {
        B.child2 = iE;
        A.child1 = iD;
        D.parent = iA;
        A.aabb.combine2(C.aabb, D.aabb);
        B.aabb.combine2(A.aabb, E.aabb);

        A.height = 1 + Math.max(C.height, D.height);
        B.height = 1 + Math.max(A.height, E.height);
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

    DynamicTreeNode child1 = node.child1;
    DynamicTreeNode child2 = node.child2;

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

    DynamicTreeNode child1 = node.child1;
    DynamicTreeNode child2 = node.child2;

    if (node.child1 == null) {
      assert(child1 == null);
      assert(child2 == null);
      assert(node.height == 0);
      return;
    }

    assert(child1 != null && 0 <= child1.id && child1.id < _nodeCapacity);
    assert(child2 != null && 0 <= child2.id && child2.id < _nodeCapacity);

    int height1 = child1.height;
    int height2 = child2.height;
    int height;
    height = 1 + Math.max(height1, height2);
    assert(node.height == height);

    AABB aabb = new AABB();
    aabb.combine2(child1.aabb, child2.aabb);

    assert(MathUtils.vector2Equals(aabb.lowerBound, node.aabb.lowerBound));
    assert(MathUtils.vector2Equals(aabb.upperBound, node.aabb.upperBound));

    _validateMetrics(child1);
    _validateMetrics(child2);
  }

  void drawTree(DebugDraw argDraw) {
    if (_root == null) {
      return;
    }
    int height = computeHeight();
    drawTreeX(argDraw, _root, 0, height);
  }

  final Color3i _color = new Color3i.zero();
  final Vector2 _textVec = new Vector2.zero();

  void drawTreeX(
      DebugDraw argDraw, DynamicTreeNode node, int spot, int height) {
    node.aabb.getVertices(drawVecs);

    _color.setFromRGBd(
        1.0, (height - spot) * 1.0 / height, (height - spot) * 1.0 / height);
    argDraw.drawPolygon(drawVecs, 4, _color);

    argDraw
        .getViewportTranform()
        .getWorldToScreen(node.aabb.upperBound, _textVec);
    argDraw.drawStringXY(
        _textVec.x, _textVec.y, "$node.id-${(spot + 1)}/$height", _color);

    if (node.child1 != null) {
      drawTreeX(argDraw, node.child1, spot + 1, height);
    }
    if (node.child2 != null) {
      drawTreeX(argDraw, node.child2, spot + 1, height);
    }
  }
}
