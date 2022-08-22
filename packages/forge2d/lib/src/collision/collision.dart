import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// Java-specific class for returning edge results
class _EdgeResults {
  double separation = 0.0;
  int edgeIndex = 0;
}

/// Used for computing contact manifolds.
class ClipVertex {
  final Vector2 v = Vector2.zero();
  final ContactID id = ContactID();

  void set(final ClipVertex cv) {
    final v1 = cv.v;
    v.x = v1.x;
    v.y = v1.y;
    final c = cv.id;
    id.indexA = c.indexA;
    id.indexB = c.indexB;
    id.typeA = c.typeA;
    id.typeB = c.typeB;
  }
}

/// This is used for determining the state of contact points.
enum PointState {
  /// point does not exist
  nullState,

  /// point was added in the update
  addState,

  /// point persisted across the update
  persistState,

  /// point was removed in the update
  removeState,
}

/// This structure is used to keep track of the best separating axis.

enum EPAxisType { unknown, edgeA, edgeB }

class EPAxis {
  EPAxisType type = EPAxisType.unknown;
  int index = 0;
  double separation = 0.0;
}

/// This holds polygon B expressed in frame A.
class TempPolygon {
  final List<Vector2> vertices = List<Vector2>.generate(
    settings.maxPolygonVertices,
    (_) => Vector2.zero(),
  );
  final List<Vector2> normals = List<Vector2>.generate(
    settings.maxPolygonVertices,
    (_) => Vector2.zero(),
  );
  int count = 0;
}

/// Reference face used for clipping
class _ReferenceFace {
  int i1 = 0, i2 = 0;
  final Vector2 v1 = Vector2.zero();
  final Vector2 v2 = Vector2.zero();
  final Vector2 normal = Vector2.zero();

  final Vector2 sideNormal1 = Vector2.zero();
  double sideOffset1 = 0.0;

  final Vector2 sideNormal2 = Vector2.zero();
  double sideOffset2 = 0.0;
}

/// Functions used for computing contact points, distance queries, and TOI
/// queries. Collision methods are non-static for pooling speed, retrieve a
/// collision object from the SingletonPool.
/// Should not be finalructed.
class Collision {
  static const int nullFeature = 0x3FFFFFFF; // Integer.MAX_VALUE;

  final DistanceInput _input = DistanceInput();
  final SimplexCache _cache = SimplexCache();
  final DistanceOutput _output = DistanceOutput();

  /// Determine if two generic shapes overlap.
  bool testOverlap(
    Shape shapeA,
    int indexA,
    Shape shapeB,
    int indexB,
    Transform xfA,
    Transform xfB,
  ) {
    _input.proxyA.set(shapeA, indexA);
    _input.proxyB.set(shapeB, indexB);
    _input.transformA.set(xfA);
    _input.transformB.set(xfB);
    _input.useRadii = true;

    _cache.count = 0;

    World.distance.compute(_output, _cache, _input);
    // djm note: anything significant about 10.0f?
    return _output.distance < 10.0 * settings.epsilon;
  }

  /// Compute the point states given two manifolds. The states pertain to the
  /// transition from manifold1 to manifold2. So state1 is either persist or
  /// remove while state2 is either add or persist.
  static void computePointStates(
    final List<PointState> state1,
    final List<PointState> state2,
    final Manifold manifold1,
    final Manifold manifold2,
  ) {
    for (var i = 0; i < settings.maxManifoldPoints; i++) {
      state1[i] = PointState.nullState;
      state2[i] = PointState.nullState;
    }

    // Detect persists and removes.
    for (var i = 0; i < manifold1.pointCount; i++) {
      final id = manifold1.points[i].id;

      state1[i] = PointState.removeState;

      for (var j = 0; j < manifold2.pointCount; j++) {
        if (manifold2.points[j].id.isEqual(id)) {
          state1[i] = PointState.persistState;
          break;
        }
      }
    }

    // Detect persists and adds
    for (var i = 0; i < manifold2.pointCount; i++) {
      final id = manifold2.points[i].id;

      state2[i] = PointState.addState;

      for (var j = 0; j < manifold1.pointCount; j++) {
        if (manifold1.points[j].id.isEqual(id)) {
          state2[i] = PointState.persistState;
          break;
        }
      }
    }
  }

  /// Clipping for contact manifolds. Sutherland-Hodgman clipping.
  static int clipSegmentToLine(
    final List<ClipVertex> vOut,
    final List<ClipVertex> vIn,
    final Vector2 normal,
    double offset,
    int vertexIndexA,
  ) {
    // Start with no _output points
    var numOut = 0;
    final vIn0 = vIn[0];
    final vIn1 = vIn[1];
    final vIn0v = vIn0.v;
    final vIn1v = vIn1.v;

    // Calculate the distance of end points to the line
    final distance0 = normal.dot(vIn0v) - offset;
    final distance1 = normal.dot(vIn1v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0.0) {
      vOut[numOut++].set(vIn0);
    }
    if (distance1 <= 0.0) {
      vOut[numOut++].set(vIn1);
    }

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0) {
      // Find intersection point of edge and plane
      final interp = distance0 / (distance0 - distance1);

      final vOutNO = vOut[numOut];
      // vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
      vOutNO.v.x = vIn0v.x + interp * (vIn1v.x - vIn0v.x);
      vOutNO.v.y = vIn0v.y + interp * (vIn1v.y - vIn0v.y);

      // VertexA is hitting edgeB.
      vOutNO.id.indexA = vertexIndexA & 0xFF;
      vOutNO.id.indexB = vIn0.id.indexB;
      vOutNO.id.typeA = ContactIDType.vertex.index & 0xFF;
      vOutNO.id.typeB = ContactIDType.face.index & 0xFF;
      ++numOut;
    }

    return numOut;
  }

  // #### COLLISION STUFF (not from collision.h or collision.cpp) ####

  // djm pooling
  final Vector2 _d = Vector2.zero();

  /// Compute the collision manifold between two circles.
  void collideCircles(
    Manifold manifold,
    final CircleShape circle1,
    final Transform xfA,
    final CircleShape circle2,
    final Transform xfB,
  ) {
    manifold.pointCount = 0;

    // after inline:
    final circle1p = circle1.position;
    final circle2p = circle2.position;
    final pAx = (xfA.q.cos * circle1p.x - xfA.q.sin * circle1p.y) + xfA.p.x;
    final pAy = (xfA.q.sin * circle1p.x + xfA.q.cos * circle1p.y) + xfA.p.y;
    final pBx = (xfB.q.cos * circle2p.x - xfB.q.sin * circle2p.y) + xfB.p.x;
    final pBy = (xfB.q.sin * circle2p.x + xfB.q.cos * circle2p.y) + xfB.p.y;
    final dx = pBx - pAx;
    final dy = pBy - pAy;
    final distSqr = dx * dx + dy * dy;
    // end inline

    final radius = circle1.radius + circle2.radius;
    if (distSqr > radius * radius) {
      return;
    }

    manifold.type = ManifoldType.circles;
    manifold.localPoint.setFrom(circle1p);
    manifold.localNormal.setZero();
    manifold.pointCount = 1;

    manifold.points[0].localPoint.setFrom(circle2p);
    manifold.points[0].id.zero();
  }

  // djm pooling, and from above

  /// Compute the collision manifold between a polygon and a circle.
  void collidePolygonAndCircle(
    Manifold manifold,
    final PolygonShape polygon,
    final Transform xfA,
    final CircleShape circle,
    final Transform xfB,
  ) {
    manifold.pointCount = 0;
    // Vec2 v = circle.p;

    // Compute circle position in the frame of the polygon.
    // before inline:
    // Transform.mulToOutUnsafe(xfB, circle.p, c);
    // Transform.mulTransToOut(xfA, c, cLocal);
    // final double cLocalx = cLocal.x;
    // final double cLocaly = cLocal.y;

    // after inline:
    final circlep = circle.position;
    final xfBq = xfB.q;
    final xfAq = xfA.q;
    final cx = (xfBq.cos * circlep.x - xfBq.sin * circlep.y) + xfB.p.x;
    final cy = (xfBq.sin * circlep.x + xfBq.cos * circlep.y) + xfB.p.y;
    final px = cx - xfA.p.x;
    final py = cy - xfA.p.y;
    final cLocalX = xfAq.cos * px + xfAq.sin * py;
    final cLocalY = -xfAq.sin * px + xfAq.cos * py;
    // end inline

    // Find the min separating edge.
    var normalIndex = 0;
    var separation = -double.maxFinite;
    final radius = polygon.radius + circle.radius;
    double s;
    final vertices = polygon.vertices;
    final normals = polygon.normals;

    for (var i = 0; i < vertices.length; i++) {
      // before inline
      // temp.set(cLocal).subLocal(vertices[i]);
      // double s = Vec2.dot(normals[i], temp);
      // after inline
      final vertex = vertices[i];
      final tempX = cLocalX - vertex.x;
      final tempY = cLocalY - vertex.y;
      s = normals[i].x * tempX + normals[i].y * tempY;

      if (s > radius) {
        // early out
        return;
      }

      if (s > separation) {
        separation = s;
        normalIndex = i;
      }
    }

    // Vertices that subtend the incident face.
    final vertIndex1 = normalIndex;
    final vertIndex2 = vertIndex1 + 1 < vertices.length ? vertIndex1 + 1 : 0;
    final v1 = vertices[vertIndex1];
    final v2 = vertices[vertIndex2];

    // If the center is inside the polygon ...
    if (separation < settings.epsilon) {
      manifold.pointCount = 1;
      manifold.type = ManifoldType.faceA;

      final normal = normals[normalIndex];
      manifold.localNormal.x = normal.x;
      manifold.localNormal.y = normal.y;
      manifold.localPoint.x = (v1.x + v2.x) * .5;
      manifold.localPoint.y = (v1.y + v2.y) * .5;
      final mpoint = manifold.points[0];
      mpoint.localPoint.x = circlep.x;
      mpoint.localPoint.y = circlep.y;
      mpoint.id.zero();
      // end inline

      return;
    }

    // Compute barycentric coordinates
    final tempX = cLocalX - v1.x;
    final tempY = cLocalY - v1.y;
    final temp2X = v2.x - v1.x;
    final temp2Y = v2.y - v1.y;
    final u1 = tempX * temp2X + tempY * temp2Y;

    final temp3X = cLocalX - v2.x;
    final temp3Y = cLocalY - v2.y;
    final temp4X = v1.x - v2.x;
    final temp4Y = v1.y - v2.y;
    final u2 = temp3X * temp4X + temp3Y * temp4Y;

    if (u1 <= 0.0) {
      final dx = cLocalX - v1.x;
      final dy = cLocalY - v1.y;
      if (dx * dx + dy * dy > radius * radius) {
        return;
      }

      manifold.pointCount = 1;
      manifold.type = ManifoldType.faceA;
      manifold.localNormal.x = cLocalX - v1.x;
      manifold.localNormal.y = cLocalY - v1.y;
      manifold.localNormal.normalize();
      manifold.localPoint.setFrom(v1);
      manifold.points[0].localPoint.setFrom(circlep);
      manifold.points[0].id.zero();
    } else if (u2 <= 0.0) {
      final dx = cLocalX - v2.x;
      final dy = cLocalY - v2.y;
      if (dx * dx + dy * dy > radius * radius) {
        return;
      }

      manifold.pointCount = 1;
      manifold.type = ManifoldType.faceA;
      manifold.localNormal.x = cLocalX - v2.x;
      manifold.localNormal.y = cLocalY - v2.y;
      manifold.localNormal.normalize();
      manifold.localPoint.setFrom(v2);
      manifold.points[0].localPoint.setFrom(circlep);
      manifold.points[0].id.zero();
    } else {
      final fcx = (v1.x + v2.x) * .5;
      final fcy = (v1.y + v2.y) * .5;

      final tx = cLocalX - fcx;
      final ty = cLocalY - fcy;
      final normal = normals[vertIndex1];
      separation = tx * normal.x + ty * normal.y;
      if (separation > radius) {
        return;
      }

      manifold.pointCount = 1;
      manifold.type = ManifoldType.faceA;
      manifold.localNormal.setFrom(normals[vertIndex1]);
      manifold.localPoint.x = fcx; // (faceCenter)
      manifold.localPoint.y = fcy;
      manifold.points[0].localPoint.setFrom(circlep);
      manifold.points[0].id.zero();
    }
  }

  final Vector2 _temp = Vector2.zero();
  final Transform _xf = Transform.zero();
  final Vector2 _n = Vector2.zero();
  final Vector2 _v1 = Vector2.zero();

  /// Find the max separation between poly1 and poly2 using edge normals from
  /// poly1.
  void findMaxSeparation(
    _EdgeResults results,
    final PolygonShape poly1,
    final Transform xf1,
    final PolygonShape poly2,
    final Transform xf2,
  ) {
    final count1 = poly1.vertices.length;
    final count2 = poly2.vertices.length;
    final n1s = poly1.normals;
    final v1s = poly1.vertices;
    final v2s = poly2.vertices;

    _xf.set(Transform.mulTrans(xf2, xf1));
    final xfq = _xf.q;

    var bestIndex = 0;
    var maxSeparation = -double.maxFinite;
    for (var i = 0; i < count1; i++) {
      // Get poly1 normal in frame2.
      _n.setFrom(Rot.mulVec2(xfq, n1s[i]));
      _v1.setFrom(Transform.mulVec2(_xf, v1s[i]));

      // Find deepest point for normal i.
      var si = double.maxFinite;
      for (var j = 0; j < count2; ++j) {
        final v2sj = v2s[j];
        final sij = _n.x * (v2sj.x - _v1.x) + _n.y * (v2sj.y - _v1.y);
        if (sij < si) {
          si = sij;
        }
      }

      if (si > maxSeparation) {
        maxSeparation = si;
        bestIndex = i;
      }
    }

    results.edgeIndex = bestIndex;
    results.separation = maxSeparation;
  }

  void findIncidentEdge(
    final List<ClipVertex> c,
    final PolygonShape poly1,
    final Transform xf1,
    int edge1,
    final PolygonShape poly2,
    final Transform xf2,
  ) {
    final count1 = poly1.vertices.length;
    final normals1 = poly1.normals;

    final count2 = poly2.vertices.length;
    final vertices2 = poly2.vertices;
    final normals2 = poly2.normals;

    assert(0 <= edge1 && edge1 < count1);

    final c0 = c[0];
    final c1 = c[1];
    final xf1q = xf1.q;
    final xf2q = xf2.q;

    // Get the normal of the reference edge in poly2's frame.
    final v = normals1[edge1];
    final tempx = xf1q.cos * v.x - xf1q.sin * v.y;
    final tempy = xf1q.sin * v.x + xf1q.cos * v.y;
    final normal1x = xf2q.cos * tempx + xf2q.sin * tempy;
    final normal1y = -xf2q.sin * tempx + xf2q.cos * tempy;

    // Find the incident edge on poly2.
    var index = 0;
    var minDot = double.maxFinite;
    for (var i = 0; i < count2; ++i) {
      final b = normals2[i];
      final dot = normal1x * b.x + normal1y * b.y;
      if (dot < minDot) {
        minDot = dot;
        index = i;
      }
    }

    // Build the clip vertices for the incident edge.
    final i1 = index;
    final i2 = i1 + 1 < count2 ? i1 + 1 : 0;

    final v1 = vertices2[i1];
    final out = c0.v;
    out.x = (xf2q.cos * v1.x - xf2q.sin * v1.y) + xf2.p.x;
    out.y = (xf2q.sin * v1.x + xf2q.cos * v1.y) + xf2.p.y;
    c0.id.indexA = edge1 & 0xFF;
    c0.id.indexB = i1 & 0xFF;
    c0.id.typeA = ContactIDType.face.index & 0xFF;
    c0.id.typeB = ContactIDType.vertex.index & 0xFF;

    final v2 = vertices2[i2];
    final out1 = c1.v;
    out1.x = (xf2q.cos * v2.x - xf2q.sin * v2.y) + xf2.p.x;
    out1.y = (xf2q.sin * v2.x + xf2q.cos * v2.y) + xf2.p.y;
    c1.id.indexA = edge1 & 0xFF;
    c1.id.indexB = i2 & 0xFF;
    c1.id.typeA = ContactIDType.face.index & 0xFF;
    c1.id.typeB = ContactIDType.vertex.index & 0xFF;
  }

  final _EdgeResults _results1 = _EdgeResults();
  final _EdgeResults results2 = _EdgeResults();
  final Vector2 _localTangent = Vector2.zero();
  final Vector2 _localNormal = Vector2.zero();
  final Vector2 _planePoint = Vector2.zero();
  final Vector2 _tangent = Vector2.zero();
  final Vector2 _v11 = Vector2.zero();
  final Vector2 _v12 = Vector2.zero();
  final List<ClipVertex> _incidentEdge = [ClipVertex(), ClipVertex()];
  final List<ClipVertex> _clipPoints1 = [ClipVertex(), ClipVertex()];
  final List<ClipVertex> _clipPoints2 = [ClipVertex(), ClipVertex()];

  /// Compute the collision manifold between two polygons.
  void collidePolygons(
    Manifold manifold,
    final PolygonShape polyA,
    final Transform xfA,
    final PolygonShape polyB,
    final Transform xfB,
  ) {
    // - Find edge normal of max separation on A - return if separating axis is
    // found
    // - Find edge normal of max separation on B - return if separation axis is
    // found
    // - Choose reference edge as min(minA, minB)
    // - Find incident edge
    // - Clip

    // The normal points from 1 to 2

    manifold.pointCount = 0;
    final totalRadius = polyA.radius + polyB.radius;

    findMaxSeparation(_results1, polyA, xfA, polyB, xfB);
    if (_results1.separation > totalRadius) {
      return;
    }

    findMaxSeparation(results2, polyB, xfB, polyA, xfA);
    if (results2.separation > totalRadius) {
      return;
    }

    PolygonShape poly1; // reference polygon
    PolygonShape poly2; // incident polygon
    Transform xf1, xf2;
    int edge1; // reference edge
    bool flip;
    final kTol = 0.1 * settings.linearSlop;

    if (results2.separation > _results1.separation + kTol) {
      poly1 = polyB;
      poly2 = polyA;
      xf1 = xfB;
      xf2 = xfA;
      edge1 = results2.edgeIndex;
      manifold.type = ManifoldType.faceB;
      flip = true;
    } else {
      poly1 = polyA;
      poly2 = polyB;
      xf1 = xfA;
      xf2 = xfB;
      edge1 = _results1.edgeIndex;
      manifold.type = ManifoldType.faceA;
      flip = false;
    }
    final xf1q = xf1.q;

    findIncidentEdge(_incidentEdge, poly1, xf1, edge1, poly2, xf2);

    final count1 = poly1.vertices.length;
    final vertices1 = poly1.vertices;

    final iv1 = edge1;
    final iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;
    _v11.setFrom(vertices1[iv1]);
    _v12.setFrom(vertices1[iv2]);
    _localTangent.x = _v12.x - _v11.x;
    _localTangent.y = _v12.y - _v11.y;
    _localTangent.normalize();

    _localNormal.x = 1.0 * _localTangent.y;
    _localNormal.y = -1.0 * _localTangent.x;

    _planePoint.x = (_v11.x + _v12.x) * .5;
    _planePoint.y = (_v11.y + _v12.y) * .5;

    _tangent.x = xf1q.cos * _localTangent.x - xf1q.sin * _localTangent.y;
    _tangent.y = xf1q.sin * _localTangent.x + xf1q.cos * _localTangent.y;

    final normalx = 1.0 * _tangent.y;
    final normaly = -1.0 * _tangent.x;

    _v11.setFrom(Transform.mulVec2(xf1, _v11));
    _v12.setFrom(Transform.mulVec2(xf1, _v12));

    // Face offset
    final frontOffset = normalx * _v11.x + normaly * _v11.y;

    // Side offsets, extended by polytope skin thickness.
    final sideOffset1 =
        -(_tangent.x * _v11.x + _tangent.y * _v11.y) + totalRadius;
    final sideOffset2 = _tangent.x * _v12.x + _tangent.y * _v12.y + totalRadius;

    _tangent.negate();
    // Clip to box side 1
    if (clipSegmentToLine(
          _clipPoints1,
          _incidentEdge,
          _tangent,
          sideOffset1,
          iv1,
        ) <
        2) {
      return;
    }

    _tangent.negate();
    // Clip to negative box side 1
    if (clipSegmentToLine(
          _clipPoints2,
          _clipPoints1,
          _tangent,
          sideOffset2,
          iv2,
        ) <
        2) {
      return;
    }

    // Now _clipPoints2 contains the clipped points.
    manifold.localNormal.setFrom(_localNormal);
    manifold.localPoint.setFrom(_planePoint);

    var pointCount = 0;
    for (var i = 0; i < settings.maxManifoldPoints; ++i) {
      final separation = normalx * _clipPoints2[i].v.x +
          normaly * _clipPoints2[i].v.y -
          frontOffset;

      if (separation <= totalRadius) {
        final cp = manifold.points[pointCount];
        final out = cp.localPoint;
        final px = _clipPoints2[i].v.x - xf2.p.x;
        final py = _clipPoints2[i].v.y - xf2.p.y;
        out.x = xf2.q.cos * px + xf2.q.sin * py;
        out.y = -xf2.q.sin * px + xf2.q.cos * py;
        cp.id.set(_clipPoints2[i].id);
        if (flip) {
          // Swap features
          cp.id.flip();
        }
        ++pointCount;
      }
    }

    manifold.pointCount = pointCount;
  }

  final Vector2 _q = Vector2.zero();
  final Vector2 _e = Vector2.zero();
  final ContactID _cf = ContactID();
  final Vector2 _e1 = Vector2.zero();
  final Vector2 _p = Vector2.zero();

  // Compute contact points for edge versus circle.
  // This accounts for edge connectivity.
  void collideEdgeAndCircle(
    Manifold manifold,
    final EdgeShape edgeA,
    final Transform xfA,
    final CircleShape circleB,
    final Transform xfB,
  ) {
    manifold.pointCount = 0;

    // Compute circle in frame of edge
    // Vec2 Q = MulT(xfA, Mul(xfB, circleB.p));
    _temp.setFrom(Transform.mulVec2(xfB, circleB.position));
    _q.setFrom(Transform.mulTransVec2(xfA, _temp));

    final a = edgeA.vertex1;
    final b = edgeA.vertex2;
    _e
      ..setFrom(b)
      ..sub(a);

    // Barycentric coordinates
    final u = _e.dot(
      _temp
        ..setFrom(b)
        ..sub(_q),
    );
    final v = _e.dot(
      _temp
        ..setFrom(_q)
        ..sub(a),
    );

    final radius = edgeA.radius + circleB.radius;

    _cf.indexB = 0;
    _cf.typeB = ContactIDType.vertex.index & 0xFF;

    // Region A
    if (v <= 0.0) {
      final P = a;
      _d
        ..setFrom(_q)
        ..sub(P);
      final dd = _d.dot(_d);
      if (dd > radius * radius) {
        return;
      }

      // Is there an edge connected to A?
      if (edgeA.hasVertex0) {
        final a1 = edgeA.vertex0;
        final b1 = a;
        _e1
          ..setFrom(b1)
          ..sub(a1);
        final u1 = _e1.dot(
          _temp
            ..setFrom(b1)
            ..sub(_q),
        );

        // Is the circle in Region AB of the previous edge?
        if (u1 > 0.0) {
          return;
        }
      }

      _cf.indexA = 0;
      _cf.typeA = ContactIDType.vertex.index & 0xFF;
      manifold.pointCount = 1;
      manifold.type = ManifoldType.circles;
      manifold.localNormal.setZero();
      manifold.localPoint.setFrom(P);
      // manifold.points[0].id.key = 0;
      manifold.points[0].id.set(_cf);
      manifold.points[0].localPoint.setFrom(circleB.position);
      return;
    }

    // Region B
    if (u <= 0.0) {
      final p = b;
      _d
        ..setFrom(_q)
        ..sub(p);
      final dd = _d.dot(_d);
      if (dd > radius * radius) {
        return;
      }

      // Is there an edge connected to B?
      if (edgeA.hasVertex3) {
        final b2 = edgeA.vertex3;
        final a2 = b;
        final e2 = _e1;
        e2
          ..setFrom(b2)
          ..sub(a2);
        final v2 = e2.dot(
          _temp
            ..setFrom(_q)
            ..sub(a2),
        );

        // Is the circle in Region AB of the next edge?
        if (v2 > 0.0) {
          return;
        }
      }

      _cf.indexA = 1;
      _cf.typeA = ContactIDType.vertex.index & 0xFF;
      manifold.pointCount = 1;
      manifold.type = ManifoldType.circles;
      manifold.localNormal.setZero();
      manifold.localPoint.setFrom(p);
      manifold.points[0].id.set(_cf);
      manifold.points[0].localPoint.setFrom(circleB.position);
      return;
    }

    // Region AB
    final den = _e.dot(_e);
    assert(den > 0.0);

    // Vec2 P = (1.0f / den) * (u * A + v * B);
    _p
      ..setFrom(a)
      ..scale(u)
      ..add(
        _temp
          ..setFrom(b)
          ..scale(v),
      );
    _p.scale(1.0 / den);
    _d
      ..setFrom(_q)
      ..sub(_p);
    final dd = _d.dot(_d);
    if (dd > radius * radius) {
      return;
    }

    _n.x = -_e.y;
    _n.y = _e.x;
    if (_n.dot(
          _temp
            ..setFrom(_q)
            ..sub(a),
        ) <
        0.0) {
      _n.setValues(-_n.x, -_n.y);
    }
    _n.normalize();

    _cf.indexA = 0;
    _cf.typeA = ContactIDType.face.index & 0xFF;
    manifold.pointCount = 1;
    manifold.type = ManifoldType.faceA;
    manifold.localNormal.setFrom(_n);
    manifold.localPoint.setFrom(a);
    // manifold.points[0].id.key = 0;
    manifold.points[0].id.set(_cf);
    manifold.points[0].localPoint.setFrom(circleB.position);
  }

  final EdgePolygonCollider _collider = EdgePolygonCollider();

  void collideEdgeAndPolygon(
    Manifold manifold,
    final EdgeShape edgeA,
    final Transform xfA,
    final PolygonShape polygonB,
    final Transform xfB,
  ) {
    _collider.collide(manifold, edgeA, xfA, polygonB, xfB);
  }
}

enum VertexType { isolated, concave, convex }

/// This class collides an edge and a polygon, taking into account edge
/// adjacency.
class EdgePolygonCollider {
  final TempPolygon polygonB = TempPolygon();

  final Transform xf = Transform.zero();
  final Vector2 centroidB = Vector2.zero();
  Vector2 v0 = Vector2.zero();
  Vector2 v1 = Vector2.zero();
  Vector2 v2 = Vector2.zero();
  Vector2 v3 = Vector2.zero();
  final Vector2 normal0 = Vector2.zero();
  final Vector2 normal1 = Vector2.zero();
  final Vector2 normal2 = Vector2.zero();
  final Vector2 normal = Vector2.zero();

  VertexType type1 = VertexType.isolated, type2 = VertexType.isolated;

  final Vector2 lowerLimit = Vector2.zero();
  final Vector2 upperLimit = Vector2.zero();
  double radius = 0.0;
  bool front = false;

  final Vector2 _edge1 = Vector2.zero();
  final Vector2 _temp = Vector2.zero();
  final Vector2 _edge0 = Vector2.zero();
  final Vector2 _edge2 = Vector2.zero();
  final List<ClipVertex> _incidentEdge = [ClipVertex(), ClipVertex()];
  final List<ClipVertex> _clipPoints1 = [ClipVertex(), ClipVertex()];
  final List<ClipVertex> _clipPoints2 = [ClipVertex(), ClipVertex()];
  final _ReferenceFace _rf = _ReferenceFace();
  final EPAxis _edgeAxis = EPAxis();
  final EPAxis _polygonAxis = EPAxis();

  void collide(
    Manifold manifold,
    final EdgeShape edgeA,
    final Transform xfA,
    final PolygonShape polygonB2,
    final Transform xfB,
  ) {
    xf.set(Transform.mulTrans(xfA, xfB));
    centroidB.setFrom(Transform.mulVec2(xf, polygonB2.centroid));

    v0 = edgeA.vertex0;
    v1 = edgeA.vertex1;
    v2 = edgeA.vertex2;
    v3 = edgeA.vertex3;

    final hasVertex0 = edgeA.hasVertex0;
    final hasVertex3 = edgeA.hasVertex3;

    _edge1
      ..setFrom(v2)
      ..sub(v1);
    _edge1.normalize();
    normal1.setValues(_edge1.y, -_edge1.x);
    final offset1 = normal1.dot(
      _temp
        ..setFrom(centroidB)
        ..sub(v1),
    );
    var offset0 = 0.0;
    var offset2 = 0.0;
    var convex1 = false, convex2 = false;

    // Is there a preceding edge?
    if (hasVertex0) {
      _edge0
        ..setFrom(v1)
        ..sub(v0);
      _edge0.normalize();
      normal0.setValues(_edge0.y, -_edge0.x);
      convex1 = _edge0.cross(_edge1) >= 0.0;
      offset0 = normal0.dot(
        _temp
          ..setFrom(centroidB)
          ..sub(v0),
      );
    }

    // Is there a following edge?
    if (hasVertex3) {
      _edge2
        ..setFrom(v3)
        ..sub(v2);
      _edge2.normalize();
      normal2.setValues(_edge2.y, -_edge2.x);
      convex2 = _edge1.cross(_edge2) > 0.0;
      offset2 = normal2.dot(
        _temp
          ..setFrom(centroidB)
          ..sub(v2),
      );
    }

    // Determine front or back collision. Determine collision normal limits.
    if (hasVertex0 && hasVertex3) {
      if (convex1 && convex2) {
        front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = normal0.x;
          lowerLimit.y = normal0.y;
          upperLimit.x = normal2.x;
          upperLimit.y = normal2.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = -normal1.x;
          lowerLimit.y = -normal1.y;
          upperLimit.x = -normal1.x;
          upperLimit.y = -normal1.y;
        }
      } else if (convex1) {
        front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = normal0.x;
          lowerLimit.y = normal0.y;
          upperLimit.x = normal1.x;
          upperLimit.y = normal1.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = -normal2.x;
          lowerLimit.y = -normal2.y;
          upperLimit.x = -normal1.x;
          upperLimit.y = -normal1.y;
        }
      } else if (convex2) {
        front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = normal1.x;
          lowerLimit.y = normal1.y;
          upperLimit.x = normal2.x;
          upperLimit.y = normal2.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = -normal1.x;
          lowerLimit.y = -normal1.y;
          upperLimit.x = -normal0.x;
          upperLimit.y = -normal0.y;
        }
      } else {
        front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = normal1.x;
          lowerLimit.y = normal1.y;
          upperLimit.x = normal1.x;
          upperLimit.y = normal1.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = -normal2.x;
          lowerLimit.y = -normal2.y;
          upperLimit.x = -normal0.x;
          upperLimit.y = -normal0.y;
        }
      }
    } else if (hasVertex0) {
      if (convex1) {
        front = offset0 >= 0.0 || offset1 >= 0.0;
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = normal0.x;
          lowerLimit.y = normal0.y;
          upperLimit.x = -normal1.x;
          upperLimit.y = -normal1.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = normal1.x;
          lowerLimit.y = normal1.y;
          upperLimit.x = -normal1.x;
          upperLimit.y = -normal1.y;
        }
      } else {
        front = offset0 >= 0.0 && offset1 >= 0.0;
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = normal1.x;
          lowerLimit.y = normal1.y;
          upperLimit.x = -normal1.x;
          upperLimit.y = -normal1.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = normal1.x;
          lowerLimit.y = normal1.y;
          upperLimit.x = -normal0.x;
          upperLimit.y = -normal0.y;
        }
      }
    } else if (hasVertex3) {
      if (convex2) {
        front = offset1 >= 0.0 || offset2 >= 0.0;
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = -normal1.x;
          lowerLimit.y = -normal1.y;
          upperLimit.x = normal2.x;
          upperLimit.y = normal2.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = -normal1.x;
          lowerLimit.y = -normal1.y;
          upperLimit.x = normal1.x;
          upperLimit.y = normal1.y;
        }
      } else {
        front = offset1 >= 0.0 && offset2 >= 0.0;
        if (front) {
          normal.x = normal1.x;
          normal.y = normal1.y;
          lowerLimit.x = -normal1.x;
          lowerLimit.y = -normal1.y;
          upperLimit.x = normal1.x;
          upperLimit.y = normal1.y;
        } else {
          normal.x = -normal1.x;
          normal.y = -normal1.y;
          lowerLimit.x = -normal2.x;
          lowerLimit.y = -normal2.y;
          upperLimit.x = normal1.x;
          upperLimit.y = normal1.y;
        }
      }
    } else {
      front = offset1 >= 0.0;
      if (front) {
        normal.x = normal1.x;
        normal.y = normal1.y;
        lowerLimit.x = -normal1.x;
        lowerLimit.y = -normal1.y;
        upperLimit.x = -normal1.x;
        upperLimit.y = -normal1.y;
      } else {
        normal.x = -normal1.x;
        normal.y = -normal1.y;
        lowerLimit.x = normal1.x;
        lowerLimit.y = normal1.y;
        upperLimit.x = normal1.x;
        upperLimit.y = normal1.y;
      }
    }

    // Get polygonB in frameA
    polygonB.count = polygonB2.vertices.length;
    for (var i = 0; i < polygonB2.vertices.length; ++i) {
      polygonB.vertices[i]
          .setFrom(Transform.mulVec2(xf, polygonB2.vertices[i]));
      polygonB.normals[i].setFrom(Rot.mulVec2(xf.q, polygonB2.normals[i]));
    }

    radius = 2.0 * settings.polygonRadius;

    manifold.pointCount = 0;

    computeEdgeSeparation(_edgeAxis);

    // If no valid normal can be found than this edge should not collide.
    if (_edgeAxis.type == EPAxisType.unknown) {
      return;
    }

    if (_edgeAxis.separation > radius) {
      return;
    }

    computePolygonSeparation(_polygonAxis);
    if (_polygonAxis.type != EPAxisType.unknown &&
        _polygonAxis.separation > radius) {
      return;
    }

    // Use hysteresis for jitter reduction.
    const relativeTol = 0.98;
    const kAbsoluteTol = 0.001;

    EPAxis primaryAxis;
    if (_polygonAxis.type == EPAxisType.unknown) {
      primaryAxis = _edgeAxis;
    } else if (_polygonAxis.separation >
        relativeTol * _edgeAxis.separation + kAbsoluteTol) {
      primaryAxis = _polygonAxis;
    } else {
      primaryAxis = _edgeAxis;
    }

    final ie0 = _incidentEdge[0];
    final ie1 = _incidentEdge[1];

    if (primaryAxis.type == EPAxisType.edgeA) {
      manifold.type = ManifoldType.faceA;

      // Search for the polygon normal that is most anti-parallel to the edge
      // normal.
      var bestIndex = 0;
      var bestValue = normal.dot(polygonB.normals[0]);
      for (var i = 1; i < polygonB.count; ++i) {
        final value = normal.dot(polygonB.normals[i]);
        if (value < bestValue) {
          bestValue = value;
          bestIndex = i;
        }
      }

      final i1 = bestIndex;
      final i2 = i1 + 1 < polygonB.count ? i1 + 1 : 0;

      ie0.v.setFrom(polygonB.vertices[i1]);
      ie0.id.indexA = 0;
      ie0.id.indexB = i1 & 0xFF;
      ie0.id.typeA = ContactIDType.face.index & 0xFF;
      ie0.id.typeB = ContactIDType.vertex.index & 0xFF;

      ie1.v.setFrom(polygonB.vertices[i2]);
      ie1.id.indexA = 0;
      ie1.id.indexB = i2 & 0xFF;
      ie1.id.typeA = ContactIDType.face.index & 0xFF;
      ie1.id.typeB = ContactIDType.vertex.index & 0xFF;

      if (front) {
        _rf.i1 = 0;
        _rf.i2 = 1;
        _rf.v1.setFrom(v1);
        _rf.v2.setFrom(v2);
        _rf.normal.setFrom(normal1);
      } else {
        _rf.i1 = 1;
        _rf.i2 = 0;
        _rf.v1.setFrom(v2);
        _rf.v2.setFrom(v1);
        _rf.normal
          ..setFrom(normal1)
          ..negate();
      }
    } else {
      manifold.type = ManifoldType.faceB;

      ie0.v.setFrom(v1);
      ie0.id.indexA = 0;
      ie0.id.indexB = primaryAxis.index & 0xFF;
      ie0.id.typeA = ContactIDType.vertex.index & 0xFF;
      ie0.id.typeB = ContactIDType.face.index & 0xFF;

      ie1.v.setFrom(v2);
      ie1.id.indexA = 0;
      ie1.id.indexB = primaryAxis.index & 0xFF;
      ie1.id.typeA = ContactIDType.vertex.index & 0xFF;
      ie1.id.typeB = ContactIDType.face.index & 0xFF;

      _rf.i1 = primaryAxis.index;
      _rf.i2 = _rf.i1 + 1 < polygonB.count ? _rf.i1 + 1 : 0;
      _rf.v1.setFrom(polygonB.vertices[_rf.i1]);
      _rf.v2.setFrom(polygonB.vertices[_rf.i2]);
      _rf.normal.setFrom(polygonB.normals[_rf.i1]);
    }

    _rf.sideNormal1.setValues(_rf.normal.y, -_rf.normal.x);
    _rf.sideNormal2
      ..setFrom(_rf.sideNormal1)
      ..negate();
    _rf.sideOffset1 = _rf.sideNormal1.dot(_rf.v1);
    _rf.sideOffset2 = _rf.sideNormal2.dot(_rf.v2);

    // Clip to box side 1
    if (Collision.clipSegmentToLine(
          _clipPoints1,
          _incidentEdge,
          _rf.sideNormal1,
          _rf.sideOffset1,
          _rf.i1,
        ) <
        settings.maxManifoldPoints) {
      return;
    }

    // Clip to negative box side 1
    if (Collision.clipSegmentToLine(
          _clipPoints2,
          _clipPoints1,
          _rf.sideNormal2,
          _rf.sideOffset2,
          _rf.i2,
        ) <
        settings.maxManifoldPoints) {
      return;
    }

    // Now _clipPoints2 contains the clipped points.
    if (primaryAxis.type == EPAxisType.edgeA) {
      manifold.localNormal.setFrom(_rf.normal);
      manifold.localPoint.setFrom(_rf.v1);
    } else {
      manifold.localNormal.setFrom(polygonB2.normals[_rf.i1]);
      manifold.localPoint.setFrom(polygonB2.vertices[_rf.i1]);
    }

    var pointCount = 0;
    for (var i = 0; i < settings.maxManifoldPoints; ++i) {
      double separation;

      separation = _rf.normal.dot(
        _temp
          ..setFrom(_clipPoints2[i].v)
          ..sub(_rf.v1),
      );

      if (separation <= radius) {
        final cp = manifold.points[pointCount];

        if (primaryAxis.type == EPAxisType.edgeA) {
          cp.localPoint.setFrom(Transform.mulTransVec2(xf, _clipPoints2[i].v));
          cp.id.set(_clipPoints2[i].id);
        } else {
          cp.localPoint.setFrom(_clipPoints2[i].v);
          cp.id.typeA = _clipPoints2[i].id.typeB;
          cp.id.typeB = _clipPoints2[i].id.typeA;
          cp.id.indexA = _clipPoints2[i].id.indexB;
          cp.id.indexB = _clipPoints2[i].id.indexA;
        }

        ++pointCount;
      }
    }

    manifold.pointCount = pointCount;
  }

  void computeEdgeSeparation(EPAxis axis) {
    axis.type = EPAxisType.edgeA;
    axis.index = front ? 0 : 1;
    axis.separation = double.maxFinite;
    final nx = normal.x;
    final ny = normal.y;

    for (var i = 0; i < polygonB.count; ++i) {
      final v = polygonB.vertices[i];
      final tempX = v.x - v1.x;
      final tempY = v.y - v1.y;
      final s = nx * tempX + ny * tempY;
      if (s < axis.separation) {
        axis.separation = s;
      }
    }
  }

  final Vector2 _perp = Vector2.zero();
  final Vector2 _n = Vector2.zero();

  void computePolygonSeparation(EPAxis axis) {
    axis.type = EPAxisType.unknown;
    axis.index = -1;
    axis.separation = -double.maxFinite;

    _perp.x = -normal.y;
    _perp.y = normal.x;

    for (var i = 0; i < polygonB.count; ++i) {
      final normalB = polygonB.normals[i];
      final vB = polygonB.vertices[i];
      _n.x = -normalB.x;
      _n.y = -normalB.y;

      // double s1 = Vec2.dot(n, temp.set(vB).subLocal(v1));
      // double s2 = Vec2.dot(n, temp.set(vB).subLocal(v2));
      var tempX = vB.x - v1.x;
      var tempY = vB.y - v1.y;
      final s1 = _n.x * tempX + _n.y * tempY;
      tempX = vB.x - v2.x;
      tempY = vB.y - v2.y;
      final s2 = _n.x * tempX + _n.y * tempY;
      final s = min(s1, s2);

      if (s > radius) {
        // No collision
        axis.type = EPAxisType.edgeB;
        axis.index = i;
        axis.separation = s;
        return;
      }

      // Adjacency
      if (_n.x * _perp.x + _n.y * _perp.y >= 0.0) {
        if ((_temp
                  ..setFrom(_n)
                  ..sub(upperLimit))
                .dot(normal) <
            -settings.angularSlop) {
          continue;
        }
      } else {
        if ((_temp
                  ..setFrom(_n)
                  ..sub(lowerLimit))
                .dot(normal) <
            -settings.angularSlop) {
          continue;
        }
      }

      if (s > axis.separation) {
        axis.type = EPAxisType.edgeB;
        axis.index = i;
        axis.separation = s;
      }
    }
  }
}
