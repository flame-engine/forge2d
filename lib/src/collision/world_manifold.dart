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

/// This is used to compute the current state of a contact manifold.
class WorldManifold {
  /// World vector pointing from A to B
  final Vector2 normal = Vector2.zero();

  /// World contact point (point of intersection)
  final List<Vector2> points = List<Vector2>(Settings.maxManifoldPoints);

  /// A negative value indicates overlap, in meters.
  final Float64List separations = Float64List(Settings.maxManifoldPoints);

  WorldManifold() {
    for (int i = 0; i < Settings.maxManifoldPoints; i++) {
      points[i] = Vector2.zero();
    }
  }

  final Vector2 _pool3 = Vector2.zero();
  final Vector2 _pool4 = Vector2.zero();

  void initialize(final Manifold manifold, final Transform xfA, double radiusA,
      final Transform xfB, double radiusB) {
    if (manifold.pointCount == 0) {
      return;
    }

    switch (manifold.type) {
      case ManifoldType.CIRCLES:
        {
          final Vector2 pointA = _pool3;
          final Vector2 pointB = _pool4;

          normal.x = 1.0;
          normal.y = 0.0;
          Vector2 v = manifold.localPoint;
          // Transform.mulToOutUnsafe(xfA, manifold.localPoint, pointA);
          // Transform.mulToOutUnsafe(xfB, manifold.points[0].localPoint, pointB);
          pointA.x = (xfA.q.c * v.x - xfA.q.s * v.y) + xfA.p.x;
          pointA.y = (xfA.q.s * v.x + xfA.q.c * v.y) + xfA.p.y;
          Vector2 mp0p = manifold.points[0].localPoint;
          pointB.x = (xfB.q.c * mp0p.x - xfB.q.s * mp0p.y) + xfB.p.x;
          pointB.y = (xfB.q.s * mp0p.x + xfB.q.c * mp0p.y) + xfB.p.y;

          if (MathUtils.distanceSquared(pointA, pointB) >
              Settings.EPSILON * Settings.EPSILON) {
            normal.x = pointB.x - pointA.x;
            normal.y = pointB.y - pointA.y;
            normal.normalize();
          }

          final double cAx = normal.x * radiusA + pointA.x;
          final double cAy = normal.y * radiusA + pointA.y;

          final double cBx = -normal.x * radiusB + pointB.x;
          final double cBy = -normal.y * radiusB + pointB.y;

          points[0].x = (cAx + cBx) * .5;
          points[0].y = (cAy + cBy) * .5;
          separations[0] = (cBx - cAx) * normal.x + (cBy - cAy) * normal.y;
        }
        break;
      case ManifoldType.FACE_A:
        {
          final Vector2 planePoint = _pool3;

          Rot.mulToOutUnsafe(xfA.q, manifold.localNormal, normal);
          Transform.mulToOutVec2(xfA, manifold.localPoint, planePoint);

          final Vector2 clipPoint = _pool4;

          for (int i = 0; i < manifold.pointCount; i++) {
            // b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
            // b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint,
            // normal)) * normal;
            // b2Vec2 cB = clipPoint - radiusB * normal;
            // points[i] = 0.5f * (cA + cB);
            Transform.mulToOutVec2(
                xfB, manifold.points[i].localPoint, clipPoint);
            // use cA as temporary for now
            // cA.set(clipPoint).subLocal(planePoint);
            // double scalar = radiusA - Vec2.dot(cA, normal);
            // cA.set(normal).mulLocal(scalar).addLocal(clipPoint);
            // cB.set(normal).mulLocal(radiusB).subLocal(clipPoint).negateLocal();
            // points[i].set(cA).addLocal(cB).mulLocal(0.5f);

            final double scalar = radiusA -
                ((clipPoint.x - planePoint.x) * normal.x +
                    (clipPoint.y - planePoint.y) * normal.y);

            final double cAx = normal.x * scalar + clipPoint.x;
            final double cAy = normal.y * scalar + clipPoint.y;

            final double cBx = -normal.x * radiusB + clipPoint.x;
            final double cBy = -normal.y * radiusB + clipPoint.y;

            points[i].x = (cAx + cBx) * .5;
            points[i].y = (cAy + cBy) * .5;
            separations[i] = (cBx - cAx) * normal.x + (cBy - cAy) * normal.y;
          }
        }
        break;
      case ManifoldType.FACE_B:
        final Vector2 planePoint = _pool3;
        Rot.mulToOutUnsafe(xfB.q, manifold.localNormal, normal);
        Transform.mulToOutVec2(xfB, manifold.localPoint, planePoint);

        // final Mat22 R = xfB.q;
        // normal.x = R.ex.x * manifold.localNormal.x + R.ey.x * manifold.localNormal.y;
        // normal.y = R.ex.y * manifold.localNormal.x + R.ey.y * manifold.localNormal.y;
        // final Vec2 v = manifold.localPoint;
        // planePoint.x = xfB.p.x + xfB.q.ex.x * v.x + xfB.q.ey.x * v.y;
        // planePoint.y = xfB.p.y + xfB.q.ex.y * v.x + xfB.q.ey.y * v.y;

        final Vector2 clipPoint = _pool4;

        for (int i = 0; i < manifold.pointCount; i++) {
          // b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
          // b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint,
          // normal)) * normal;
          // b2Vec2 cA = clipPoint - radiusA * normal;
          // points[i] = 0.5f * (cA + cB);

          Transform.mulToOutVec2(xfA, manifold.points[i].localPoint, clipPoint);
          // cB.set(clipPoint).subLocal(planePoint);
          // double scalar = radiusB - Vec2.dot(cB, normal);
          // cB.set(normal).mulLocal(scalar).addLocal(clipPoint);
          // cA.set(normal).mulLocal(radiusA).subLocal(clipPoint).negateLocal();
          // points[i].set(cA).addLocal(cB).mulLocal(0.5f);

          // points[i] = 0.5f * (cA + cB);

          //
          // clipPoint.x = xfA.p.x + xfA.q.ex.x * manifold.points[i].localPoint.x + xfA.q.ey.x *
          // manifold.points[i].localPoint.y;
          // clipPoint.y = xfA.p.y + xfA.q.ex.y * manifold.points[i].localPoint.x + xfA.q.ey.y *
          // manifold.points[i].localPoint.y;

          final double scalar = radiusB -
              ((clipPoint.x - planePoint.x) * normal.x +
                  (clipPoint.y - planePoint.y) * normal.y);

          final double cBx = normal.x * scalar + clipPoint.x;
          final double cBy = normal.y * scalar + clipPoint.y;

          final double cAx = -normal.x * radiusA + clipPoint.x;
          final double cAy = -normal.y * radiusA + clipPoint.y;

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
