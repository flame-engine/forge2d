import 'dart:typed_data';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// This is used to compute the current state of a contact manifold.
class WorldManifold {
  /// World vector pointing from A to B
  final Vector2 normal = Vector2.zero();

  /// World contact point (point of intersection)
  final List<Vector2> points = List<Vector2>.generate(
    settings.maxManifoldPoints,
    (_) => Vector2.zero(),
  );

  /// A negative value indicates overlap, in meters.
  final Float64List separations = Float64List(settings.maxManifoldPoints);

  WorldManifold();

  final Vector2 _pool3 = Vector2.zero();
  final Vector2 _pool4 = Vector2.zero();

  void initialize(
    final Manifold manifold,
    final Transform xfA,
    double radiusA,
    final Transform xfB,
    double radiusB,
  ) {
    if (manifold.pointCount == 0) {
      return;
    }

    switch (manifold.type) {
      case ManifoldType.circles:
        {
          final pointA = _pool3;
          final pointB = _pool4;

          normal.x = 1.0;
          normal.y = 0.0;
          final v = manifold.localPoint;
          pointA.x = (xfA.q.cos * v.x - xfA.q.sin * v.y) + xfA.p.x;
          pointA.y = (xfA.q.sin * v.x + xfA.q.cos * v.y) + xfA.p.y;
          final mp0p = manifold.points[0].localPoint;
          pointB.x = (xfB.q.cos * mp0p.x - xfB.q.sin * mp0p.y) + xfB.p.x;
          pointB.y = (xfB.q.sin * mp0p.x + xfB.q.cos * mp0p.y) + xfB.p.y;

          if (pointA.distanceToSquared(pointB) >
              settings.epsilon * settings.epsilon) {
            normal.x = pointB.x - pointA.x;
            normal.y = pointB.y - pointA.y;
            normal.normalize();
          }

          final cAx = normal.x * radiusA + pointA.x;
          final cAy = normal.y * radiusA + pointA.y;

          final cBx = -normal.x * radiusB + pointB.x;
          final cBy = -normal.y * radiusB + pointB.y;

          points[0].x = (cAx + cBx) * .5;
          points[0].y = (cAy + cBy) * .5;
          separations[0] = (cBx - cAx) * normal.x + (cBy - cAy) * normal.y;
        }
        break;
      case ManifoldType.faceA:
        {
          final planePoint = _pool3;

          normal.setFrom(Rot.mulVec2(xfA.q, manifold.localNormal));
          planePoint.setFrom(Transform.mulVec2(xfA, manifold.localPoint));

          final clipPoint = _pool4;

          for (var i = 0; i < manifold.pointCount; i++) {
            clipPoint
                .setFrom(Transform.mulVec2(xfB, manifold.points[i].localPoint));

            final scalar = radiusA -
                ((clipPoint.x - planePoint.x) * normal.x +
                    (clipPoint.y - planePoint.y) * normal.y);

            final cAx = normal.x * scalar + clipPoint.x;
            final cAy = normal.y * scalar + clipPoint.y;

            final cBx = -normal.x * radiusB + clipPoint.x;
            final cBy = -normal.y * radiusB + clipPoint.y;

            points[i].x = (cAx + cBx) * .5;
            points[i].y = (cAy + cBy) * .5;
            separations[i] = (cBx - cAx) * normal.x + (cBy - cAy) * normal.y;
          }
        }
        break;
      case ManifoldType.faceB:
        final planePoint = _pool3;
        normal.setFrom(Rot.mulVec2(xfB.q, manifold.localNormal));
        planePoint.setFrom(Transform.mulVec2(xfB, manifold.localPoint));

        final clipPoint = _pool4;

        for (var i = 0; i < manifold.pointCount; i++) {
          clipPoint
              .setFrom(Transform.mulVec2(xfA, manifold.points[i].localPoint));

          final scalar = radiusB -
              ((clipPoint.x - planePoint.x) * normal.x +
                  (clipPoint.y - planePoint.y) * normal.y);

          final cBx = normal.x * scalar + clipPoint.x;
          final cBy = normal.y * scalar + clipPoint.y;

          final cAx = -normal.x * radiusA + clipPoint.x;
          final cAy = -normal.y * radiusA + clipPoint.y;

          points[i].x = (cAx + cBx) * .5;
          points[i].y = (cAy + cBy) * .5;
          separations[i] = (cAx - cBx) * normal.x + (cAy - cBy) * normal.y;
        }
        // Ensure normal points from A to B.
        normal.x = -normal.x;
        normal.y = -normal.y;
        break;
    }
  }
}
