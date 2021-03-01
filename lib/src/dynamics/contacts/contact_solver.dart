import 'dart:math';

import '../../../forge2d.dart';
import '../../settings.dart' as settings;

class ContactSolverDef {
  TimeStep step;
  List<Contact> contacts;
  List<Position> positions;
  List<Velocity> velocities;
}

class ContactSolver {
  static const bool DEBUG_SOLVER = false;
  static const double k_errorTol = 1e-3;

  /// For each solver, this is the initial number of constraints in the array, which expands as
  /// needed.
  static const int INITIAL_NUM_CONSTRAINTS = 256;

  /// Ensure a reasonable condition number. for the block solver
  static const double kMaxConditionNumber = 100.0;

  TimeStep _step;
  List<Position> _positions;
  List<Velocity> _velocities;
  List<Contact> _contacts;

  void init(ContactSolverDef def) {
    _step = def.step;
    _positions = def.positions;
    _velocities = def.velocities;
    _contacts = def.contacts;

    for (var contact in _contacts) {
      final fixtureA = contact.fixtureA;
      final fixtureB = contact.fixtureB;
      final shapeA = fixtureA.shape;
      final shapeB = fixtureB.shape;
      final radiusA = shapeA.radius;
      final radiusB = shapeB.radius;
      final bodyA = fixtureA.body;
      final bodyB = fixtureB.body;
      final manifold = contact.manifold;

      final pointCount = manifold.pointCount;
      assert(pointCount > 0);

      final velocityConstraint =
          contact.velocityConstraint
            ..friction = contact.friction
            ..restitution = contact.restitution
            ..tangentSpeed = contact.tangentSpeed
            ..indexA = bodyA.islandIndex
            ..indexB = bodyB.islandIndex
            ..invMassA = bodyA.inverseMass
            ..invMassB = bodyB.inverseMass
            ..invIA = bodyA.inverseInertia
            ..invIB = bodyB.inverseInertia
            ..contactIndex = _contacts.indexOf(contact)
            ..pointCount = pointCount
            ..K.setZero()
            ..normalMass.setZero();

      final positionConstraint =
          contact.positionConstraint
            ..indexA = bodyA.islandIndex
            ..indexB = bodyB.islandIndex
            ..invMassA = bodyA.inverseMass
            ..invMassB = bodyB.inverseMass
            ..localCenterA.setFrom(bodyA.sweep.localCenter)
            ..localCenterB.setFrom(bodyB.sweep.localCenter)
            ..invIA = bodyA.inverseInertia
            ..invIB = bodyB.inverseInertia
            ..localNormal.setFrom(manifold.localNormal)
            ..localPoint.setFrom(manifold.localPoint)
            ..pointCount = pointCount
            ..radiusA = radiusA
            ..radiusB = radiusB
            ..type = manifold.type;

      for (var j = 0; j < pointCount; j++) {
        final cp = manifold.points[j];
        final vcp = velocityConstraint.points[j];

        if (_step.warmStarting) {
          vcp.normalImpulse = _step.dtRatio * cp.normalImpulse;
          vcp.tangentImpulse = _step.dtRatio * cp.tangentImpulse;
        } else {
          vcp.normalImpulse = 0.0;
          vcp.tangentImpulse = 0.0;
        }

        vcp.rA.setZero();
        vcp.rB.setZero();
        vcp.normalMass = 0.0;
        vcp.tangentMass = 0.0;
        vcp.velocityBias = 0.0;
        positionConstraint.localPoints[j].x = cp.localPoint.x;
        positionConstraint.localPoints[j].y = cp.localPoint.y;
      }
    }
  }

  void warmStart() {
    // Warm start.
    for (var contact in _contacts) {
      final velocityConstraint =
          contact.velocityConstraint;

      final indexA = velocityConstraint.indexA;
      final indexB = velocityConstraint.indexB;
      final mA = velocityConstraint.invMassA;
      final iA = velocityConstraint.invIA;
      final mB = velocityConstraint.invMassB;
      final iB = velocityConstraint.invIB;
      final pointCount = velocityConstraint.pointCount;

      final vA = _velocities[indexA].v;
      var wA = _velocities[indexA].w;
      final vB = _velocities[indexB].v;
      var wB = _velocities[indexB].w;

      final normal = velocityConstraint.normal;
      final tangentX = 1.0 * normal.y;
      final tangentY = -1.0 * normal.x;

      for (var j = 0; j < pointCount; ++j) {
        final vcp = velocityConstraint.points[j];
        final pX =
            tangentX * vcp.tangentImpulse + normal.x * vcp.normalImpulse;
        final pY =
            tangentY * vcp.tangentImpulse + normal.y * vcp.normalImpulse;

        wA -= iA * (vcp.rA.x * pY - vcp.rA.y * pX);
        vA.x -= pX * mA;
        vA.y -= pY * mA;
        wB += iB * (vcp.rB.x * pY - vcp.rB.y * pX);
        vB.x += pX * mB;
        vB.y += pY * mB;
      }
      _velocities[indexA].w = wA;
      _velocities[indexB].w = wB;
    }
  }

  // djm pooling, and from above
  // TODO(srdjan): make them private.
  final Transform xfA = Transform.zero();
  final Transform xfB = Transform.zero();
  final WorldManifold worldManifold = WorldManifold();

  void initializeVelocityConstraints() {
    // Warm start.
    for (var contact in _contacts) {
      final velocityConstraint =
          contact.velocityConstraint;
      final positionConstraint =
          contact.positionConstraint;

      final radiusA = positionConstraint.radiusA;
      final radiusB = positionConstraint.radiusB;
      final manifold =
          _contacts[velocityConstraint.contactIndex].manifold;

      final indexA = velocityConstraint.indexA;
      final indexB = velocityConstraint.indexB;

      final mA = velocityConstraint.invMassA;
      final mB = velocityConstraint.invMassB;
      final iA = velocityConstraint.invIA;
      final iB = velocityConstraint.invIB;
      final localCenterA = positionConstraint.localCenterA;
      final localCenterB = positionConstraint.localCenterB;

      final cA = _positions[indexA].c;
      final aA = _positions[indexA].a;
      final vA = _velocities[indexA].v;
      final wA = _velocities[indexA].w;

      final cB = _positions[indexB].c;
      final aB = _positions[indexB].a;
      final vB = _velocities[indexB].v;
      final wB = _velocities[indexB].w;

      assert(manifold.pointCount > 0);

      final xfAq = xfA.q;
      final xfBq = xfB.q;
      xfAq.setAngle(aA);
      xfBq.setAngle(aB);
      xfA.p.x = cA.x - (xfAq.c * localCenterA.x - xfAq.s * localCenterA.y);
      xfA.p.y = cA.y - (xfAq.s * localCenterA.x + xfAq.c * localCenterA.y);
      xfB.p.x = cB.x - (xfBq.c * localCenterB.x - xfBq.s * localCenterB.y);
      xfB.p.y = cB.y - (xfBq.s * localCenterB.x + xfBq.c * localCenterB.y);

      worldManifold.initialize(manifold, xfA, radiusA, xfB, radiusB);

      final vcNormal = velocityConstraint.normal;
      vcNormal.x = worldManifold.normal.x;
      vcNormal.y = worldManifold.normal.y;

      final pointCount = velocityConstraint.pointCount;
      for (var j = 0; j < pointCount; ++j) {
        final vcp = velocityConstraint.points[j];
        final wmPj = worldManifold.points[j];
        final vcprA = vcp.rA;
        final vcprB = vcp.rB;
        vcprA.x = wmPj.x - cA.x;
        vcprA.y = wmPj.y - cA.y;
        vcprB.x = wmPj.x - cB.x;
        vcprB.y = wmPj.y - cB.y;

        final rnA = vcprA.x * vcNormal.y - vcprA.y * vcNormal.x;
        final rnB = vcprB.x * vcNormal.y - vcprB.y * vcNormal.x;

        final kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

        final tangentX = 1.0 * vcNormal.y;
        final tangentY = -1.0 * vcNormal.x;

        final rtA = vcprA.x * tangentY - vcprA.y * tangentX;
        final rtB = vcprB.x * tangentY - vcprB.y * tangentX;

        final kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

        vcp.tangentMass = kTangent > 0.0 ? 1.0 / kTangent : 0.0;

        // Setup a velocity bias for restitution.
        vcp.velocityBias = 0.0;
        final tempX = vB.x + -wB * vcprB.y - vA.x - (-wA * vcprA.y);
        final tempY = vB.y + wB * vcprB.x - vA.y - (wA * vcprA.x);
        final vRel = vcNormal.x * tempX + vcNormal.y * tempY;
        if (vRel < -settings.velocityThreshold) {
          vcp.velocityBias = -velocityConstraint.restitution * vRel;
        }
      }

      // If we have two points, then prepare the block solver.
      if (velocityConstraint.pointCount == 2) {
        final vcp1 = velocityConstraint.points[0];
        final vcp2 = velocityConstraint.points[1];
        final rn1A = vcp1.rA.x * vcNormal.y - vcp1.rA.y * vcNormal.x;
        final rn1B = vcp1.rB.x * vcNormal.y - vcp1.rB.y * vcNormal.x;
        final rn2A = vcp2.rA.x * vcNormal.y - vcp2.rA.y * vcNormal.x;
        final rn2B = vcp2.rB.x * vcNormal.y - vcp2.rB.y * vcNormal.x;

        final k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
        final k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
        final k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
        if (k11 * k11 < kMaxConditionNumber * (k11 * k22 - k12 * k12)) {
          // K is safe to invert.
          velocityConstraint.K.setValues(k11, k12, k12, k22);
          velocityConstraint.normalMass.setFrom(velocityConstraint.K);
          velocityConstraint.normalMass.invert();
        } else {
          // The constraints are redundant, just use one.
          // TODO_ERIN use deepest?
          velocityConstraint.pointCount = 1;
        }
      }
    }
  }

  void solveVelocityConstraints() {
    for (var contact in _contacts) {
      final vc = contact.velocityConstraint;

      final indexA = vc.indexA;
      final indexB = vc.indexB;

      final mA = vc.invMassA;
      final mB = vc.invMassB;
      final iA = vc.invIA;
      final iB = vc.invIB;
      final pointCount = vc.pointCount;

      final vA = _velocities[indexA].v;
      var wA = _velocities[indexA].w;
      final vB = _velocities[indexB].v;
      var wB = _velocities[indexB].w;

      final normal = vc.normal;
      final normalX = normal.x;
      final normalY = normal.y;
      final tangentX = 1.0 * vc.normal.y;
      final tangentY = -1.0 * vc.normal.x;
      final friction = vc.friction;

      assert(pointCount == 1 || pointCount == 2);

      // Solve tangent constraints
      for (var j = 0; j < pointCount; ++j) {
        final vcp = vc.points[j];
        final a = vcp.rA;
        final dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * a.y;
        final dvy = wB * vcp.rB.x + vB.y - vA.y - wA * a.x;

        // Compute tangent force
        final vt = dvx * tangentX + dvy * tangentY - vc.tangentSpeed;
        var lambda = vcp.tangentMass * (-vt);

        // Clamp the accumulated force
        final maxFriction = (friction * vcp.normalImpulse).abs();
        final newImpulse = (vcp.tangentImpulse + lambda)
            .clamp(-maxFriction, maxFriction)
            .toDouble();
        lambda = newImpulse - vcp.tangentImpulse;
        vcp.tangentImpulse = newImpulse;

        // Apply contact impulse
        // Vec2 P = lambda * tangent;

        final pX = tangentX * lambda;
        final pY = tangentY * lambda;

        // vA -= invMassA * P;
        vA.x -= pX * mA;
        vA.y -= pY * mA;
        wA -= iA * (vcp.rA.x * pY - vcp.rA.y * pX);

        // vB += invMassB * P;
        vB.x += pX * mB;
        vB.y += pY * mB;
        wB += iB * (vcp.rB.x * pY - vcp.rB.y * pX);
      }

      // Solve normal constraints
      if (vc.pointCount == 1) {
        final vcp = vc.points[0];

        // Relative velocity at contact
        // Vec2 dv = vB + Cross(wB, vcp.rB) - vA - Cross(wA, vcp.rA);

        final dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * vcp.rA.y;
        final dvy = wB * vcp.rB.x + vB.y - vA.y - wA * vcp.rA.x;

        // Compute normal impulse
        final vn = dvx * normalX + dvy * normalY;
        var lambda = -vcp.normalMass * (vn - vcp.velocityBias);

        // Clamp the accumulated impulse
        final a = vcp.normalImpulse + lambda;
        final newImpulse = a > 0.0 ? a : 0.0;
        lambda = newImpulse - vcp.normalImpulse;
        vcp.normalImpulse = newImpulse;

        // Apply contact impulse
        final pX = normalX * lambda;
        final pY = normalY * lambda;

        vA.x -= pX * mA;
        vA.y -= pY * mA;
        wA -= iA * (vcp.rA.x * pY - vcp.rA.y * pX);

        // vB += invMassB * P;
        vB.x += pX * mB;
        vB.y += pY * mB;
        wB += iB * (vcp.rB.x * pY - vcp.rB.y * pX);
      } else {
        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on
        // Box2D_Lite).
        // Build the mini LCP for this contact patch
        //
        // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
        //
        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        // b = vn_0 - velocityBias
        //
        // The system is solved using the "Total enumeration method" (s. Murty). The complementary
        // constraint vn_i * x_i
        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D
        // contact problem the cases
        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be
        // tested. The first valid
        // solution that satisfies the problem is chosen.
        //
        // In order to account of the accumulated impulse 'a' (because of the iterative nature of
        // the solver which only requires
        // that the accumulated impulse is clamped and not the incremental impulse) we change the
        // impulse variable (x_i).
        //
        // Substitute:
        //
        // x = a + d
        //
        // a := old total impulse
        // x := new total impulse
        // d := incremental impulse
        //
        // For the current iteration we extend the formula for the incremental impulse
        // to compute the new total impulse:
        //
        // vn = A * d + b
        // = A * (x - a) + b
        // = A * x + b - A * a
        // = A * x + b'
        // b' = b - A * a;

        final cp1 = vc.points[0];
        final cp2 = vc.points[1];
        final cp1rA = cp1.rA;
        final cp1rB = cp1.rB;
        final cp2rA = cp2.rA;
        final cp2rB = cp2.rB;
        final ax = cp1.normalImpulse;
        final ay = cp2.normalImpulse;

        assert(ax >= 0.0 && ay >= 0.0);
        // Relative velocity at contact
        // Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
        final dv1x = -wB * cp1rB.y + vB.x - vA.x + wA * cp1rA.y;
        final dv1y = wB * cp1rB.x + vB.y - vA.y - wA * cp1rA.x;

        // Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
        final dv2x = -wB * cp2rB.y + vB.x - vA.x + wA * cp2rA.y;
        final dv2y = wB * cp2rB.x + vB.y - vA.y - wA * cp2rA.x;

        // Compute normal velocity
        var vn1 = dv1x * normalX + dv1y * normalY;
        var vn2 = dv2x * normalX + dv2y * normalY;

        var bx = vn1 - cp1.velocityBias;
        var by = vn2 - cp2.velocityBias;

        // Compute b'
        final r = vc.K;
        bx -= r.entry(0, 0) * ax + r.entry(0, 1) * ay;
        by -= r.entry(1, 0) * ax + r.entry(1, 1) * ay;

        // final double k_errorTol = 1e-3f;
        // B2_NOT_USED(k_errorTol);
        while (true) {
          //
          // Case 1: vn = 0
          //
          // 0 = A * x' + b'
          //
          // Solve for x':
          //
          // x' = - inv(A) * b'
          //
          // Vec2 x = - Mul(c.normalMass, b);
          final r1 = vc.normalMass;
          var xx = r1.entry(0, 0) * bx + r1.entry(0, 1) * by;
          var xy = r1.entry(1, 0) * bx + r1.entry(1, 1) * by;
          xx *= -1;
          xy *= -1;

          if (xx >= 0.0 && xy >= 0.0) {
            // Get the incremental impulse
            final dx = xx - ax;
            final dy = xy - ay;

            // Apply incremental impulse
            final p1x = dx * normalX;
            final p1y = dx * normalY;
            final p2x = dy * normalX;
            final p2y = dy * normalY;

            /*
             * vA -= invMassA * (P1 + P2); wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (p1x + p2x);
            vA.y -= mA * (p1y + p2y);
            vB.x += mB * (p1x + p2x);
            vB.y += mB * (p1y + p2y);

            wA -= iA *
                (cp1rA.x * p1y -
                    cp1rA.y * p1x +
                    (cp2rA.x * p2y - cp2rA.y * p2x));
            wB += iB *
                (cp1rB.x * p1y -
                    cp1rB.y * p1x +
                    (cp2rB.x * p2y - cp2rB.y * p2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
             * Cross(wA, cp1.rA); dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
             *
             * // Compute normal velocity vn1 = Dot(dv1, normal); vn2 = Dot(dv2, normal);
             *
             * assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); assert(Abs(vn2 - cp2.velocityBias)
             * < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              final dv1 = vB + _crossDoubleVector2(wB, cp1rB)
                ..sub(vA)
                ..sub(_crossDoubleVector2(wA, cp1rA));
              final dv2 = vB + _crossDoubleVector2(wB, cp2rB)
                ..sub(vA)
                ..sub(_crossDoubleVector2(wA, cp2rA));
              // Compute normal velocity
              vn1 = dv1.dot(normal);
              vn2 = dv2.dot(normal);

              assert((vn1 - cp1.velocityBias).abs() < k_errorTol);
              assert((vn2 - cp2.velocityBias).abs() < k_errorTol);
            }
            break;
          }

          //
          // Case 2: vn1 = 0 and x2 = 0
          //
          // 0 = a11 * x1' + a12 * 0 + b1'
          // vn2 = a21 * x1' + a22 * 0 + '
          //
          xx = -cp1.normalMass * bx;
          xy = 0.0;
          vn1 = 0.0;
          vn2 = vc.K.entry(1, 0) * xx + by;

          if (xx >= 0.0 && vn2 >= 0.0) {
            // Get the incremental impulse
            final dx = xx - ax;
            final dy = xy - ay;

            // Apply incremental impulse
            final p1x = normalX * dx;
            final p1y = normalY * dx;
            final p2x = normalX * dy;
            final p2y = normalY * dy;

            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (p1x + p2x);
            vA.y -= mA * (p1y + p2y);
            vB.x += mB * (p1x + p2x);
            vB.y += mB * (p1y + p2y);

            wA -= iA *
                (cp1rA.x * p1y -
                    cp1rA.y * p1x +
                    (cp2rA.x * p2y - cp2rA.y * p2x));
            wB += iB *
                (cp1rB.x * p1y -
                    cp1rB.y * p1x +
                    (cp2rB.x * p2y - cp2rB.y * p2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
             * Cross(wA, cp1.rA);
             *
             * // Compute normal velocity vn1 = Dot(dv1, normal);
             *
             * assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              final dv1 = vB + _crossDoubleVector2(wB, cp1rB)
                ..sub(vA)
                ..sub(_crossDoubleVector2(wA, cp1rA));
              // Compute normal velocity
              vn1 = dv1.dot(normal);

              assert((vn1 - cp1.velocityBias).abs() < k_errorTol);
            }
            break;
          }

          //
          // Case 3: wB = 0 and x1 = 0
          //
          // vn1 = a11 * 0 + a12 * x2' + b1'
          // 0 = a21 * 0 + a22 * x2' + '
          //
          xx = 0.0;
          xy = -cp2.normalMass * by;
          vn1 = vc.K.entry(0, 1) * xy + bx;
          vn2 = 0.0;

          if (xy >= 0.0 && vn1 >= 0.0) {
            // Resubstitute for the incremental impulse
            final dx = xx - ax;
            final dy = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            final p1x = normalX * dx;
            final p1y = normalY * dx;
            final p2x = normalX * dy;
            final p2y = normalY * dy;

            vA.x -= mA * (p1x + p2x);
            vA.y -= mA * (p1y + p2y);
            vB.x += mB * (p1x + p2x);
            vB.y += mB * (p1y + p2y);

            wA -= iA *
                (cp1rA.x * p1y -
                    cp1rA.y * p1x +
                    (cp2rA.x * p2y - cp2rA.y * p2x));
            wB += iB *
                (cp1rB.x * p1y -
                    cp1rB.y * p1x +
                    (cp2rB.x * p2y - cp2rB.y * p2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            /*
             * #if B2_DEBUG_SOLVER == 1 // Postconditions dv2 = vB + Cross(wB, cp2.rB) - vA -
             * Cross(wA, cp2.rA);
             *
             * // Compute normal velocity vn2 = Dot(dv2, normal);
             *
             * assert(Abs(vn2 - cp2.velocityBias) < k_errorTol); #endif
             */
            if (DEBUG_SOLVER) {
              // Postconditions
              final dv2 = vB + _crossDoubleVector2(wB, cp2rB)
                ..sub(vA)
                ..sub(_crossDoubleVector2(wA, cp2rA));
              // Compute normal velocity
              vn2 = dv2.dot(normal);

              assert((vn2 - cp2.velocityBias).abs() < k_errorTol);
            }
            break;
          }

          //
          // Case 4: x1 = 0 and x2 = 0
          //
          // vn1 = b1
          // vn2 = ;
          xx = 0.0;
          xy = 0.0;
          vn1 = bx;
          vn2 = by;

          if (vn1 >= 0.0 && vn2 >= 0.0) {
            // Resubstitute for the incremental impulse
            final dx = xx - ax;
            final dy = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            final p1x = normalX * dx;
            final p1y = normalY * dx;
            final p2x = normalX * dy;
            final p2y = normalY * dy;

            vA.x -= mA * (p1x + p2x);
            vA.y -= mA * (p1y + p2y);
            vB.x += mB * (p1x + p2x);
            vB.y += mB * (p1y + p2y);

            wA -= iA *
                (cp1rA.x * p1y -
                    cp1rA.y * p1x +
                    (cp2rA.x * p2y - cp2rA.y * p2x));
            wB += iB *
                (cp1rB.x * p1y -
                    cp1rB.y * p1x +
                    (cp2rB.x * p2y - cp2rB.y * p2x));

            // Accumulate
            cp1.normalImpulse = xx;
            cp2.normalImpulse = xy;

            break;
          }

          // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
          break;
        }
      }

      _velocities[indexA].w = wA;
      _velocities[indexB].w = wB;
    }
  }

  void storeImpulses() {
    for (var contact in _contacts) {
      final vc = contact.velocityConstraint;
      final manifold = _contacts[vc.contactIndex].manifold;

      for (var j = 0; j < vc.pointCount; j++) {
        manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
        manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
      }
    }
  }

  final PositionSolverManifold _pSolver = PositionSolverManifold();

  /// Sequential solver.
  bool solvePositionConstraints() {
    var minSeparation = 0.0;

    for (var contact in _contacts) {
      final pc = contact.positionConstraint;

      final indexA = pc.indexA;
      final indexB = pc.indexB;

      final mA = pc.invMassA;
      final iA = pc.invIA;
      final localCenterA = pc.localCenterA;
      final localCenterAx = localCenterA.x;
      final localCenterAy = localCenterA.y;
      final mB = pc.invMassB;
      final iB = pc.invIB;
      final localCenterB = pc.localCenterB;
      final localCenterBx = localCenterB.x;
      final localCenterBy = localCenterB.y;
      final pointCount = pc.pointCount;

      final cA = _positions[indexA].c;
      var aA = _positions[indexA].a;
      final cB = _positions[indexB].c;
      var aB = _positions[indexB].a;

      // Solve normal constraints
      for (var j = 0; j < pointCount; ++j) {
        final xfAq = xfA.q;
        final xfBq = xfB.q;
        xfAq.setAngle(aA);
        xfBq.setAngle(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        final psm = _pSolver;
        psm.initialize(pc, xfA, xfB, j);
        final normal = psm.normal;
        final point = psm.point;
        final separation = psm.separation;

        final rAx = point.x - cA.x;
        final rAy = point.y - cA.y;
        final rBx = point.x - cB.x;
        final rBy = point.y - cB.y;

        // Track max constraint error.
        minSeparation = min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        final C = (settings.baumgarte * (separation + settings.linearSlop))
            .clamp(-settings.maxLinearCorrection, 0.0);

        // Compute the effective mass.
        final rnA = rAx * normal.y - rAy * normal.x;
        final rnB = rBx * normal.y - rBy * normal.x;
        final K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        final impulse = K > 0.0 ? -C / K : 0.0;

        final pX = normal.x * impulse;
        final pY = normal.y * impulse;

        cA.x -= pX * mA;
        cA.y -= pY * mA;
        aA -= iA * (rAx * pY - rAy * pX);

        cB.x += pX * mB;
        cB.y += pY * mB;
        aB += iB * (rBx * pY - rBy * pX);
      }

      _positions[indexA].a = aA;
      _positions[indexB].a = aB;
    }

    // We can't expect minSeparation >= -linearSlop because we don't
    // push the separation above -linearSlop.
    return minSeparation >= -3.0 * settings.linearSlop;
  }

  // Sequential position solver for position constraints.
  bool solveTOIPositionConstraints(int toiIndexA, int toiIndexB) {
    var minSeparation = 0.0;

    for (var contact in _contacts) {
      final pc = contact.positionConstraint;

      final indexA = pc.indexA;
      final indexB = pc.indexB;
      final localCenterA = pc.localCenterA;
      final localCenterB = pc.localCenterB;
      final localCenterAx = localCenterA.x;
      final localCenterAy = localCenterA.y;
      final localCenterBx = localCenterB.x;
      final localCenterBy = localCenterB.y;
      final pointCount = pc.pointCount;

      var mA = 0.0;
      var iA = 0.0;
      if (indexA == toiIndexA || indexA == toiIndexB) {
        mA = pc.invMassA;
        iA = pc.invIA;
      }

      var mB = 0.0;
      var iB = 0.0;
      if (indexB == toiIndexA || indexB == toiIndexB) {
        mB = pc.invMassB;
        iB = pc.invIB;
      }

      final cA = _positions[indexA].c;
      var aA = _positions[indexA].a;

      final cB = _positions[indexB].c;
      var aB = _positions[indexB].a;

      // Solve normal constraints
      for (var j = 0; j < pointCount; ++j) {
        final xfAq = xfA.q;
        final xfBq = xfB.q;
        xfAq.setAngle(aA);
        xfBq.setAngle(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        final psm = _pSolver;
        psm.initialize(pc, xfA, xfB, j);
        final normal = psm.normal;

        final point = psm.point;
        final separation = psm.separation;

        final rAx = point.x - cA.x;
        final rAy = point.y - cA.y;
        final rBx = point.x - cB.x;
        final rBy = point.y - cB.y;

        // Track max constraint error.
        minSeparation = min(minSeparation, separation);

        // Prevent large corrections and allow slop.

        final C = (settings.baumgarte * (separation + settings.linearSlop))
            .clamp(-settings.maxLinearCorrection, 0.0);

        // Compute the effective mass.
        final rnA = rAx * normal.y - rAy * normal.x;
        final rnB = rBx * normal.y - rBy * normal.x;
        final k = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        final impulse = k > 0.0 ? -C / k : 0.0;

        final pX = normal.x * impulse;
        final pY = normal.y * impulse;

        cA.x -= pX * mA;
        cA.y -= pY * mA;
        aA -= iA * (rAx * pY - rAy * pX);

        cB.x += pX * mB;
        cB.y += pY * mB;
        aB += iB * (rBx * pY - rBy * pX);
      }

      _positions[indexA].a = aA;
      _positions[indexB].a = aB;
    }

    // We can't expect minSeparation >= -_linearSlop because we don't
    // push the separation above -_linearSlop.
    return minSeparation >= -1.5 * settings.linearSlop;
  }

  Vector2 _crossDoubleVector2(double s, Vector2 a) {
    return Vector2(-s * a.y, s * a.x);
  }
}

class PositionSolverManifold {
  final Vector2 normal = Vector2.zero();
  final Vector2 point = Vector2.zero();
  double separation = 0.0;

  void initialize(
      ContactPositionConstraint pc, Transform xfA, Transform xfB, int index) {
    assert(pc.pointCount > 0);

    final xfAq = xfA.q;
    final xfBq = xfB.q;
    final pcLocalPointsI = pc.localPoints[index];
    switch (pc.type) {
      case ManifoldType.CIRCLES:
        final pLocalPoint = pc.localPoint;
        final pLocalPoints0 = pc.localPoints[0];
        final pointAx =
            (xfAq.c * pLocalPoint.x - xfAq.s * pLocalPoint.y) + xfA.p.x;
        final pointAy =
            (xfAq.s * pLocalPoint.x + xfAq.c * pLocalPoint.y) + xfA.p.y;
        final pointBx =
            (xfBq.c * pLocalPoints0.x - xfBq.s * pLocalPoints0.y) + xfB.p.x;
        final pointBy =
            (xfBq.s * pLocalPoints0.x + xfBq.c * pLocalPoints0.y) + xfB.p.y;
        normal.x = pointBx - pointAx;
        normal.y = pointBy - pointAy;
        normal.normalize();

        point.x = (pointAx + pointBx) * .5;
        point.y = (pointAy + pointBy) * .5;
        final tempX = pointBx - pointAx;
        final tempY = pointBy - pointAy;
        separation =
            tempX * normal.x + tempY * normal.y - pc.radiusA - pc.radiusB;
        break;

      case ManifoldType.FACE_A:
        final pcLocalNormal = pc.localNormal;
        final pcLocalPoint = pc.localPoint;
        normal.x = xfAq.c * pcLocalNormal.x - xfAq.s * pcLocalNormal.y;
        normal.y = xfAq.s * pcLocalNormal.x + xfAq.c * pcLocalNormal.y;
        final planePointX =
            (xfAq.c * pcLocalPoint.x - xfAq.s * pcLocalPoint.y) + xfA.p.x;
        final planePointY =
            (xfAq.s * pcLocalPoint.x + xfAq.c * pcLocalPoint.y) + xfA.p.y;

        final clipPointX =
            (xfBq.c * pcLocalPointsI.x - xfBq.s * pcLocalPointsI.y) + xfB.p.x;
        final clipPointY =
            (xfBq.s * pcLocalPointsI.x + xfBq.c * pcLocalPointsI.y) + xfB.p.y;
        final tempX = clipPointX - planePointX;
        final tempY = clipPointY - planePointY;
        separation =
            tempX * normal.x + tempY * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointX;
        point.y = clipPointY;
        break;

      case ManifoldType.FACE_B:
        final pcLocalNormal = pc.localNormal;
        final pcLocalPoint = pc.localPoint;
        normal.x = xfBq.c * pcLocalNormal.x - xfBq.s * pcLocalNormal.y;
        normal.y = xfBq.s * pcLocalNormal.x + xfBq.c * pcLocalNormal.y;
        final planePointX =
            (xfBq.c * pcLocalPoint.x - xfBq.s * pcLocalPoint.y) + xfB.p.x;
        final planePointY =
            (xfBq.s * pcLocalPoint.x + xfBq.c * pcLocalPoint.y) + xfB.p.y;

        final clipPointX =
            (xfAq.c * pcLocalPointsI.x - xfAq.s * pcLocalPointsI.y) + xfA.p.x;
        final clipPointY =
            (xfAq.s * pcLocalPointsI.x + xfAq.c * pcLocalPointsI.y) + xfA.p.y;
        final tempX = clipPointX - planePointX;
        final tempY = clipPointY - planePointY;
        separation =
            tempX * normal.x + tempY * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointX;
        point.y = clipPointY;
        normal.x *= -1;
        normal.y *= -1;
        break;
    }
  }
}
