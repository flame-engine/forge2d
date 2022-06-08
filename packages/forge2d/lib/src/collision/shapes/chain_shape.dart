import '../../../forge2d.dart';
import '../../settings.dart' as settings;

/// A chain shape is a free form sequence of line segments. The chain has
/// two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order. Connectivity information is used
/// to create smooth collisions. WARNING: The chain will not collide properly if
/// there are self-intersections.
class ChainShape extends Shape {
  /// Do not modify directly
  final List<Vector2> vertices = <Vector2>[];
  int get vertexCount => vertices.length;
  // TODO(spydon): Rely on order of list instead
  final Vector2 _prevVertex = Vector2.zero();
  final Vector2 _nextVertex = Vector2.zero();
  bool _hasPrevVertex = false;
  bool _hasNextVertex = false;

  ChainShape() : super(ShapeType.chain) {
    radius = settings.polygonRadius;
  }

  void clear() {
    vertices.clear();
  }

  @override
  int get childCount {
    return vertexCount - 1;
  }

  /// Get a child edge.
  EdgeShape childEdge(int index) {
    assert(0 <= index && index < vertexCount - 1);
    final edge = EdgeShape();

    edge.radius = radius;
    final v0 = vertices[index + 0];
    final v1 = vertices[index + 1];
    edge.vertex1.x = v0.x;
    edge.vertex1.y = v0.y;
    edge.vertex2.x = v1.x;
    edge.vertex2.y = v1.y;

    if (index > 0) {
      final v = vertices[index - 1];
      edge.vertex0.x = v.x;
      edge.vertex0.y = v.y;
      edge.hasVertex0 = true;
    } else {
      edge.vertex0.x = _prevVertex.x;
      edge.vertex0.y = _prevVertex.y;
      edge.hasVertex0 = _hasPrevVertex;
    }

    if (index < vertexCount - 2) {
      final v = vertices[index + 2];
      edge.vertex3.x = v.x;
      edge.vertex3.y = v.y;
      edge.hasVertex3 = true;
    } else {
      edge.vertex3.x = _nextVertex.x;
      edge.vertex3.y = _nextVertex.y;
      edge.hasVertex3 = _hasNextVertex;
    }
    return edge;
  }

  @override
  double computeDistanceToOut(
    Transform xf,
    Vector2 p,
    int childIndex,
    Vector2 normalOut,
  ) {
    final edge = childEdge(childIndex);
    return edge.computeDistanceToOut(xf, p, 0, normalOut);
  }

  @override
  bool testPoint(Transform xf, Vector2 p) {
    return false;
  }

  @override
  bool raycast(
    RayCastOutput output,
    RayCastInput input,
    Transform xf,
    int childIndex,
  ) {
    assert(childIndex < vertexCount);

    final edgeShape = EdgeShape();

    final i1 = childIndex;
    var i2 = childIndex + 1;
    if (i2 == vertexCount) {
      i2 = 0;
    }
    final v = vertices[i1];
    edgeShape.vertex1.x = v.x;
    edgeShape.vertex1.y = v.y;
    final v1 = vertices[i2];
    edgeShape.vertex2.x = v1.x;
    edgeShape.vertex2.y = v1.y;

    return edgeShape.raycast(output, input, xf, 0);
  }

  @override
  void computeAABB(AABB aabb, Transform xf, int childIndex) {
    assert(childIndex < vertexCount);
    final lower = aabb.lowerBound;
    final upper = aabb.upperBound;

    final i1 = childIndex;
    var i2 = childIndex + 1;
    if (i2 == vertexCount) {
      i2 = 0;
    }

    final vi1 = vertices[i1];
    final vi2 = vertices[i2];
    final xfq = xf.q;
    final xfp = xf.p;
    final v1x = (xfq.cos * vi1.x - xfq.sin * vi1.y) + xfp.x;
    final v1y = (xfq.sin * vi1.x + xfq.cos * vi1.y) + xfp.y;
    final v2x = (xfq.cos * vi2.x - xfq.sin * vi2.y) + xfp.x;
    final v2y = (xfq.sin * vi2.x + xfq.cos * vi2.y) + xfp.y;

    lower.x = v1x < v2x ? v1x : v2x;
    lower.y = v1y < v2y ? v1y : v2y;
    upper.x = v1x > v2x ? v1x : v2x;
    upper.y = v1y > v2y ? v1y : v2y;
  }

  @override
  void computeMass(MassData massData, double density) {
    massData.mass = 0.0;
    massData.center.setZero();
    massData.I = 0.0;
  }

  @override
  Shape clone() {
    final clone = ChainShape();
    clone.createChain(vertices);
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
  Vector2 vertex(int index) {
    assert(index >= 0 && index < vertices.length);
    return vertices[index].clone();
  }

  /// Create a loop. This automatically adjusts connectivity.
  ///
  /// @param vertices an array of vertices, these are copied
  void createLoop(final List<Vector2> vertices) {
    assert(vertexCount == 0);
    assert(
      vertices.length >= 3,
      "A loop can't be created with less than 3 vertices",
    );
    this.vertices.addAll(vertices.map((Vector2 v) => v.clone()));
    _validateDistances(this.vertices);
    this.vertices.add(this.vertices[0].clone());
    prevVertex = this.vertices[vertexCount - 2];
    nextVertex = this.vertices[1];
    _hasPrevVertex = true;
    _hasNextVertex = true;
  }

  /// Create a chain with isolated end vertices.
  ///
  /// @param vertices an array of vertices, these are copied
  void createChain(final List<Vector2> vertices) {
    assert(vertexCount == 0);
    assert(vertices.length >= 2);
    this.vertices.addAll(vertices.map((Vector2 v) => v.clone()));
    _validateDistances(this.vertices);
    _prevVertex.setZero();
    _nextVertex.setZero();
    _hasPrevVertex = false;
    _hasNextVertex = false;
  }

  /// Establish connectivity to a vertex that precedes the first vertex.
  /// Don't call this for loops.
  set prevVertex(Vector2 prevVertex) {
    _prevVertex.setFrom(prevVertex);
    _hasPrevVertex = true;
  }

  /// Establish connectivity to a vertex that follows the last vertex.
  /// Don't call this for loops.
  set nextVertex(Vector2 nextVertex) {
    _nextVertex.setFrom(nextVertex);
    _hasNextVertex = true;
  }

  void _validateDistances(final List<Vector2> vertices) {
    for (var i = 1; i < vertices.length; i++) {
      final v1 = vertices[i - 1];
      final v2 = vertices[i];
      // If the code crashes here, it means your vertices are too close
      // together.
      if (v1.distanceToSquared(v2) <
          settings.linearSlop * settings.linearSlop) {
        throw 'Vertices of chain shape are too close together';
      }
    }
  }
}
