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

/***
 * A chain shape is a free form sequence of line segments. The chain has two-sided collision, so you
 * can use inside and outside collision. Therefore, you may use any winding order. Connectivity
 * information is used to create smooth collisions. WARNING: The chain will not collide properly if
 * there are self-intersections.
 */

class ChainShape extends Shape {
  List<Vec2> m_vertices;
  int m_count = 0;
  final Vec2 m_prevVertex = new Vec2.zero(),
      m_nextVertex = new Vec2.zero();
  bool m_hasPrevVertex = false,
      m_hasNextVertex = false;

  final EdgeShape _pool0 = new EdgeShape();

  ChainShape() : super(ShapeType.CHAIN) {
    radius = Settings.polygonRadius;
  }

  void clear() {
    m_vertices = null;
    m_count = 0;
  }

  int getChildCount() {
    return m_count - 1;
  }

  /**
   * Get a child edge.
   */
  void getChildEdge(EdgeShape edge, int index) {
    assert(0 <= index && index < m_count - 1);
    edge.radius = radius;

    final Vec2 v0 = m_vertices[index + 0];
    final Vec2 v1 = m_vertices[index + 1];
    edge.m_vertex1.x = v0.x;
    edge.m_vertex1.y = v0.y;
    edge.m_vertex2.x = v1.x;
    edge.m_vertex2.y = v1.y;

    if (index > 0) {
      Vec2 v = m_vertices[index - 1];
      edge.m_vertex0.x = v.x;
      edge.m_vertex0.y = v.y;
      edge.m_hasVertex0 = true;
    } else {
      edge.m_vertex0.x = m_prevVertex.x;
      edge.m_vertex0.y = m_prevVertex.y;
      edge.m_hasVertex0 = m_hasPrevVertex;
    }

    if (index < m_count - 2) {
      Vec2 v = m_vertices[index + 2];
      edge.m_vertex3.x = v.x;
      edge.m_vertex3.y = v.y;
      edge.m_hasVertex3 = true;
    } else {
      edge.m_vertex3.x = m_nextVertex.x;
      edge.m_vertex3.y = m_nextVertex.y;
      edge.m_hasVertex3 = m_hasNextVertex;
    }
  }

  double computeDistanceToOut(
      Transform xf, Vec2 p, int childIndex, Vec2 normalOut) {
    final EdgeShape edge = _pool0;
    getChildEdge(edge, childIndex);
    return edge.computeDistanceToOut(xf, p, 0, normalOut);
  }

  bool testPoint(Transform xf, Vec2 p) {
    return false;
  }

  bool raycast(
      RayCastOutput output, RayCastInput input, Transform xf, int childIndex) {
    assert(childIndex < m_count);

    final EdgeShape edgeShape = _pool0;

    int i1 = childIndex;
    int i2 = childIndex + 1;
    if (i2 == m_count) {
      i2 = 0;
    }
    Vec2 v = m_vertices[i1];
    edgeShape.m_vertex1.x = v.x;
    edgeShape.m_vertex1.y = v.y;
    Vec2 v1 = m_vertices[i2];
    edgeShape.m_vertex2.x = v1.x;
    edgeShape.m_vertex2.y = v1.y;

    return edgeShape.raycast(output, input, xf, 0);
  }

  void computeAABB(AABB aabb, Transform xf, int childIndex) {
    assert(childIndex < m_count);
    final Vec2 lower = aabb.lowerBound;
    final Vec2 upper = aabb.upperBound;

    int i1 = childIndex;
    int i2 = childIndex + 1;
    if (i2 == m_count) {
      i2 = 0;
    }

    final Vec2 vi1 = m_vertices[i1];
    final Vec2 vi2 = m_vertices[i2];
    final Rot xfq = xf.q;
    final Vec2 xfp = xf.p;
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
    ChainShape clone = new ChainShape();
    clone.createChain(m_vertices, m_count);
    clone.m_prevVertex.set(m_prevVertex);
    clone.m_nextVertex.set(m_nextVertex);
    clone.m_hasPrevVertex = m_hasPrevVertex;
    clone.m_hasNextVertex = m_hasNextVertex;
    return clone;
  }

  /**
   * Create a loop. This automatically adjusts connectivity.
   * 
   * @param vertices an array of vertices, these are copied
   * @param count the vertex count
   */
  void createLoop(final List<Vec2> vertices, int count) {
    assert(m_vertices == null && m_count == 0);
    assert(count >= 3);
    m_count = count + 1;
    m_vertices = new List<Vec2>(m_count);
    for (int i = 1; i < count; i++) {
      Vec2 v1 = vertices[i - 1];
      Vec2 v2 = vertices[i];
      // If the code crashes here, it means your vertices are too close together.
      if (MathUtils.distanceSquared(v1, v2) <
          Settings.linearSlop * Settings.linearSlop) {
        throw "Vertices of chain shape are too close together";
      }
    }
    for (int i = 0; i < count; i++) {
      m_vertices[i] = new Vec2.copy(vertices[i]);
    }
    m_vertices[count] = new Vec2.copy(m_vertices[0]);
    m_prevVertex.set(m_vertices[m_count - 2]);
    m_nextVertex.set(m_vertices[1]);
    m_hasPrevVertex = true;
    m_hasNextVertex = true;
  }

  /**
   * Create a chain with isolated end vertices.
   * 
   * @param vertices an array of vertices, these are copied
   * @param count the vertex count
   */
  void createChain(final List<Vec2> vertices, int count) {
    assert(m_vertices == null && m_count == 0);
    assert(count >= 2);
    m_count = count;
    m_vertices = new List<Vec2>(m_count);
    for (int i = 1; i < m_count; i++) {
      Vec2 v1 = vertices[i - 1];
      Vec2 v2 = vertices[i];
      // If the code crashes here, it means your vertices are too close together.
      if (MathUtils.distanceSquared(v1, v2) <
          Settings.linearSlop * Settings.linearSlop) {
        throw "Vertices of chain shape are too close together";
      }
    }
    for (int i = 0; i < m_count; i++) {
      m_vertices[i] = new Vec2.copy(vertices[i]);
    }
    m_hasPrevVertex = false;
    m_hasNextVertex = false;

    m_prevVertex.setZero();
    m_nextVertex.setZero();
  }

  /**
   * Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops.
   * 
   * @param prevVertex
   */
  void setPrevVertex(final Vec2 prevVertex) {
    m_prevVertex.set(prevVertex);
    m_hasPrevVertex = true;
  }

  /**
   * Establish connectivity to a vertex that follows the last vertex. Don't call this for loops.
   * 
   * @param nextVertex
   */
  void setNextVertex(final Vec2 nextVertex) {
    m_nextVertex.set(nextVertex);
    m_hasNextVertex = true;
  }
}
