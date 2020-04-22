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

/// A chain shape is a free form sequence of line segments. The chain has two-sided collision, so you
/// can use inside and outside collision. Therefore, you may use any winding order. Connectivity
/// information is used to create smooth collisions. WARNING: The chain will not collide properly if
/// there are self-intersections.
class ChainShape extends Shape {
  List<Vector2> _vertices;
  int _count = 0;
  final Vector2 _prevVertex = Vector2.zero(), _nextVertex = Vector2.zero();
  bool _hasPrevVertex = false, _hasNextVertex = false;

  final EdgeShape _pool0 = EdgeShape();

  ChainShape() : super(ShapeType.CHAIN) {
    radius = Settings.polygonRadius;
  }

  void clear() {
    _vertices = null;
    _count = 0;
  }

  int getChildCount() {
    return _count - 1;
  }

  /// Get a child edge.
  void getChildEdge(EdgeShape edge, int index) {
    assert(0 <= index && index < _count - 1);
    edge.radius = radius;

    final Vector2 v0 = _vertices[index + 0];
    final Vector2 v1 = _vertices[index + 1];
    edge.vertex1.x = v0.x;
    edge.vertex1.y = v0.y;
    edge.vertex2.x = v1.x;
    edge.vertex2.y = v1.y;

    if (index > 0) {
      Vector2 v = _vertices[index - 1];
      edge.vertex0.x = v.x;
      edge.vertex0.y = v.y;
      edge.hasVertex0 = true;
    } else {
      edge.vertex0.x = _prevVertex.x;
      edge.vertex0.y = _prevVertex.y;
      edge.hasVertex0 = _hasPrevVertex;
    }

    if (index < _count - 2) {
      Vector2 v = _vertices[index + 2];
      edge.vertex3.x = v.x;
      edge.vertex3.y = v.y;
      edge.hasVertex3 = true;
    } else {
      edge.vertex3.x = _nextVertex.x;
      edge.vertex3.y = _nextVertex.y;
      edge.hasVertex3 = _hasNextVertex;
    }
  }

  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut) {
    final EdgeShape edge = _pool0;
    getChildEdge(edge, childIndex);
    return edge.computeDistanceToOut(xf, p, 0, normalOut);
  }

  bool testPoint(Transform xf, Vector2 p) {
    return false;
  }

  bool raycast(
      RayCastOutput output, RayCastInput input, Transform xf, int childIndex) {
    assert(childIndex < _count);

    final EdgeShape edgeShape = _pool0;

    int i1 = childIndex;
    int i2 = childIndex + 1;
    if (i2 == _count) {
      i2 = 0;
    }
    Vector2 v = _vertices[i1];
    edgeShape.vertex1.x = v.x;
    edgeShape.vertex1.y = v.y;
    Vector2 v1 = _vertices[i2];
    edgeShape.vertex2.x = v1.x;
    edgeShape.vertex2.y = v1.y;

    return edgeShape.raycast(output, input, xf, 0);
  }

  void computeAABB(AABB aabb, Transform xf, int childIndex) {
    assert(childIndex < _count);
    final Vector2 lower = aabb.lowerBound;
    final Vector2 upper = aabb.upperBound;

    int i1 = childIndex;
    int i2 = childIndex + 1;
    if (i2 == _count) {
      i2 = 0;
    }

    final Vector2 vi1 = _vertices[i1];
    final Vector2 vi2 = _vertices[i2];
    final Rot xfq = xf.q;
    final Vector2 xfp = xf.p;
    double v1x = (xfq.c * vi1.x - xfq.s * vi1.y) + xfp.x;
    double v1y = (xfq.s * vi1.x + xfq.c * vi1.y) + xfp.y;
    double v2x = (xfq.c * vi2.x - xfq.s * vi2.y) + xfp.x;
    double v2y = (xfq.s * vi2.x + xfq.c * vi2.y) + xfp.y;

    lower.x = v1x < v2x ? v1x : v2x;
    lower.y = v1y < v2y ? v1y : v2y;
    upper.x = v1x > v2x ? v1x : v2x;
    upper.y = v1y > v2y ? v1y : v2y;
  }

  void computeMass(MassData massData, double density) {
    massData.mass = 0.0;
    massData.center.setZero();
    massData.I = 0.0;
  }

  Shape clone() {
    ChainShape clone = ChainShape();
    clone.createChain(_vertices, _count);
    clone._prevVertex.setFrom(_prevVertex);
    clone._nextVertex.setFrom(_nextVertex);
    clone._hasPrevVertex = _hasPrevVertex;
    clone._hasNextVertex = _hasNextVertex;
    return clone;
  }

  /// Returns the vertex at the given position (index).
  ///
  /// @param index the index of the vertex 0 <= index < getVertexCount( )
  /// @param vertex output vertex object, must be initialized
  Vector2 getVertex(int index) {
    assert(index >= 0 && index < _vertices.length);
    return _vertices[index].clone();
  }

  int getVertexCount() {
    return _vertices.length;
  }

  /// Create a loop. This automatically adjusts connectivity.
  ///
  /// @param vertices an array of vertices, these are copied
  /// @param count the vertex count
  void createLoop(final List<Vector2> vertices, int count) {
    assert(_vertices == null && _count == 0);
    assert(count >= 3);
    _count = count + 1;
    _vertices = List<Vector2>(_count);
    for (int i = 1; i < count; i++) {
      Vector2 v1 = vertices[i - 1];
      Vector2 v2 = vertices[i];
      // If the code crashes here, it means your vertices are too close together.
      if (MathUtils.distanceSquared(v1, v2) <
          Settings.linearSlop * Settings.linearSlop) {
        throw "Vertices of chain shape are too close together";
      }
    }
    for (int i = 0; i < count; i++) {
      _vertices[i] = Vector2.copy(vertices[i]);
    }
    _vertices[count] = Vector2.copy(_vertices[0]);
    _prevVertex.setFrom(_vertices[_count - 2]);
    _nextVertex.setFrom(_vertices[1]);
    _hasPrevVertex = true;
    _hasNextVertex = true;
  }

  /// Create a chain with isolated end vertices.
  ///
  /// @param vertices an array of vertices, these are copied
  /// @param count the vertex count
  void createChain(final List<Vector2> vertices, int count) {
    assert(_vertices == null && _count == 0);
    assert(count >= 2);
    _count = count;
    _vertices = List<Vector2>(_count);
    for (int i = 1; i < _count; i++) {
      Vector2 v1 = vertices[i - 1];
      Vector2 v2 = vertices[i];
      // If the code crashes here, it means your vertices are too close together.
      if (MathUtils.distanceSquared(v1, v2) <
          Settings.linearSlop * Settings.linearSlop) {
        throw "Vertices of chain shape are too close together";
      }
    }
    for (int i = 0; i < _count; i++) {
      _vertices[i] = Vector2.copy(vertices[i]);
    }
    _hasPrevVertex = false;
    _hasNextVertex = false;

    _prevVertex.setZero();
    _nextVertex.setZero();
  }

  /// Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops.
  void setPrevVertex(final Vector2 prevVertex) {
    _prevVertex.setFrom(prevVertex);
    _hasPrevVertex = true;
  }

  /// Establish connectivity to a vertex that follows the last vertex. Don't call this for loops.
  void setNextVertex(final Vector2 nextVertex) {
    _nextVertex.setFrom(nextVertex);
    _hasNextVertex = true;
  }
}
