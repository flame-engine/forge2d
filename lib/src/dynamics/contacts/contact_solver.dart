part of box2d;

class ContactSolverDef {
  TimeStep step;
  List<Contact> contacts;
  int count = 0;
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
  static final double k_maxConditionNumber = 100.0;

  TimeStep _step;
  List<Position> _positions;
  List<Velocity> _velocities;
  List<ContactPositionConstraint> _positionConstraints;
  List<ContactVelocityConstraint> _velocityConstraints;
  List<Contact> _contacts;
  int _count = 0;

  ContactSolver() {
    _positionConstraints =
        List<ContactPositionConstraint>(INITIAL_NUM_CONSTRAINTS);
    _velocityConstraints =
        List<ContactVelocityConstraint>(INITIAL_NUM_CONSTRAINTS);
    for (int i = 0; i < INITIAL_NUM_CONSTRAINTS; i++) {
      _positionConstraints[i] = ContactPositionConstraint();
      _velocityConstraints[i] = ContactVelocityConstraint();
    }
  }

  void init(ContactSolverDef def) {
    _step = def.step;
    _count = def.count;

    if (_positionConstraints.length < _count) {
      List<ContactPositionConstraint> old = _positionConstraints;
      _positionConstraints =
          List<ContactPositionConstraint>(Math.max(old.length * 2, _count));
      BufferUtils.arrayCopy(old, 0, _positionConstraints, 0, old.length);
      for (int i = old.length; i < _positionConstraints.length; i++) {
        _positionConstraints[i] = ContactPositionConstraint();
      }
    }

    if (_velocityConstraints.length < _count) {
      List<ContactVelocityConstraint> old = _velocityConstraints;
      _velocityConstraints =
          List<ContactVelocityConstraint>(Math.max(old.length * 2, _count));
      BufferUtils.arrayCopy(old, 0, _velocityConstraints, 0, old.length);
      for (int i = old.length; i < _velocityConstraints.length; i++) {
        _velocityConstraints[i] = ContactVelocityConstraint();
      }
    }

    _positions = def.positions;
    _velocities = def.velocities;
    _contacts = def.contacts;

    for (int i = 0; i < _count; ++i) {
      final Contact contact = _contacts[i];

      final Fixture fixtureA = contact._fixtureA;
      final Fixture fixtureB = contact._fixtureB;
      final Shape shapeA = fixtureA.getShape();
      final Shape shapeB = fixtureB.getShape();
      final double radiusA = shapeA.radius;
      final double radiusB = shapeB.radius;
      final Body bodyA = fixtureA.getBody();
      final Body bodyB = fixtureB.getBody();
      final Manifold manifold = contact._manifold;

      int pointCount = manifold.pointCount;
      assert(pointCount > 0);

      ContactVelocityConstraint vc = _velocityConstraints[i];
      vc.friction = contact._friction;
      vc.restitution = contact._restitution;
      vc.tangentSpeed = contact._tangentSpeed;
      vc.indexA = bodyA._islandIndex;
      vc.indexB = bodyB._islandIndex;
      vc.invMassA = bodyA._invMass;
      vc.invMassB = bodyB._invMass;
      vc.invIA = bodyA._invI;
      vc.invIB = bodyB._invI;
      vc.contactIndex = i;
      vc.pointCount = pointCount;
      vc.K.setZero();
      vc.normalMass.setZero();

      ContactPositionConstraint pc = _positionConstraints[i];
      pc.indexA = bodyA._islandIndex;
      pc.indexB = bodyB._islandIndex;
      pc.invMassA = bodyA._invMass;
      pc.invMassB = bodyB._invMass;
      pc.localCenterA.setFrom(bodyA._sweep.localCenter);
      pc.localCenterB.setFrom(bodyB._sweep.localCenter);
      pc.invIA = bodyA._invI;
      pc.invIB = bodyB._invI;
      pc.localNormal.setFrom(manifold.localNormal);
      pc.localPoint.setFrom(manifold.localPoint);
      pc.pointCount = pointCount;
      pc.radiusA = radiusA;
      pc.radiusB = radiusB;
      pc.type = manifold.type;

      for (int j = 0; j < pointCount; j++) {
        ManifoldPoint cp = manifold.points[j];
        VelocityConstraintPoint vcp = vc.points[j];

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
        pc.localPoints[j].x = cp.localPoint.x;
        pc.localPoints[j].y = cp.localPoint.y;
      }
    }
  }

  void warmStart() {
    // Warm start.
    for (int i = 0; i < _count; ++i) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];

      int indexA = vc.indexA;
      int indexB = vc.indexB;
      double mA = vc.invMassA;
      double iA = vc.invIA;
      double mB = vc.invMassB;
      double iB = vc.invIB;
      int pointCount = vc.pointCount;

      Vector2 vA = _velocities[indexA].v;
      double wA = _velocities[indexA].w;
      Vector2 vB = _velocities[indexB].v;
      double wB = _velocities[indexB].w;

      Vector2 normal = vc.normal;
      double tangentX = 1.0 * normal.y;
      double tangentY = -1.0 * normal.x;

      for (int j = 0; j < pointCount; ++j) {
        VelocityConstraintPoint vcp = vc.points[j];
        double Px =
            tangentX * vcp.tangentImpulse + normal.x * vcp.normalImpulse;
        double Py =
            tangentY * vcp.tangentImpulse + normal.y * vcp.normalImpulse;

        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
        vB.x += Px * mB;
        vB.y += Py * mB;
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
    for (int i = 0; i < _count; ++i) {
      ContactVelocityConstraint vc = _velocityConstraints[i];
      ContactPositionConstraint pc = _positionConstraints[i];

      double radiusA = pc.radiusA;
      double radiusB = pc.radiusB;
      Manifold manifold = _contacts[vc.contactIndex]._manifold;

      int indexA = vc.indexA;
      int indexB = vc.indexB;

      double mA = vc.invMassA;
      double mB = vc.invMassB;
      double iA = vc.invIA;
      double iB = vc.invIB;
      Vector2 localCenterA = pc.localCenterA;
      Vector2 localCenterB = pc.localCenterB;

      Vector2 cA = _positions[indexA].c;
      double aA = _positions[indexA].a;
      Vector2 vA = _velocities[indexA].v;
      double wA = _velocities[indexA].w;

      Vector2 cB = _positions[indexB].c;
      double aB = _positions[indexB].a;
      Vector2 vB = _velocities[indexB].v;
      double wB = _velocities[indexB].w;

      assert(manifold.pointCount > 0);

      final Rot xfAq = xfA.q;
      final Rot xfBq = xfB.q;
      xfAq.setAngle(aA);
      xfBq.setAngle(aB);
      xfA.p.x = cA.x - (xfAq.c * localCenterA.x - xfAq.s * localCenterA.y);
      xfA.p.y = cA.y - (xfAq.s * localCenterA.x + xfAq.c * localCenterA.y);
      xfB.p.x = cB.x - (xfBq.c * localCenterB.x - xfBq.s * localCenterB.y);
      xfB.p.y = cB.y - (xfBq.s * localCenterB.x + xfBq.c * localCenterB.y);

      worldManifold.initialize(manifold, xfA, radiusA, xfB, radiusB);

      final Vector2 vcNormal = vc.normal;
      vcNormal.x = worldManifold.normal.x;
      vcNormal.y = worldManifold.normal.y;

      int pointCount = vc.pointCount;
      for (int j = 0; j < pointCount; ++j) {
        VelocityConstraintPoint vcp = vc.points[j];
        Vector2 wmPj = worldManifold.points[j];
        final Vector2 vcprA = vcp.rA;
        final Vector2 vcprB = vcp.rB;
        vcprA.x = wmPj.x - cA.x;
        vcprA.y = wmPj.y - cA.y;
        vcprB.x = wmPj.x - cB.x;
        vcprB.y = wmPj.y - cB.y;

        double rnA = vcprA.x * vcNormal.y - vcprA.y * vcNormal.x;
        double rnB = vcprB.x * vcNormal.y - vcprB.y * vcNormal.x;

        double kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

        double tangentX = 1.0 * vcNormal.y;
        double tangentY = -1.0 * vcNormal.x;

        double rtA = vcprA.x * tangentY - vcprA.y * tangentX;
        double rtB = vcprB.x * tangentY - vcprB.y * tangentX;

        double kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

        vcp.tangentMass = kTangent > 0.0 ? 1.0 / kTangent : 0.0;

        // Setup a velocity bias for restitution.
        vcp.velocityBias = 0.0;
        double tempx = vB.x + -wB * vcprB.y - vA.x - (-wA * vcprA.y);
        double tempy = vB.y + wB * vcprB.x - vA.y - (wA * vcprA.x);
        double vRel = vcNormal.x * tempx + vcNormal.y * tempy;
        if (vRel < -Settings.velocityThreshold) {
          vcp.velocityBias = -vc.restitution * vRel;
        }
      }

      // If we have two points, then prepare the block solver.
      if (vc.pointCount == 2) {
        VelocityConstraintPoint vcp1 = vc.points[0];
        VelocityConstraintPoint vcp2 = vc.points[1];
        double rn1A = vcp1.rA.x * vcNormal.y - vcp1.rA.y * vcNormal.x;
        double rn1B = vcp1.rB.x * vcNormal.y - vcp1.rB.y * vcNormal.x;
        double rn2A = vcp2.rA.x * vcNormal.y - vcp2.rA.y * vcNormal.x;
        double rn2B = vcp2.rB.x * vcNormal.y - vcp2.rB.y * vcNormal.x;

        double k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
        double k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
        double k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
        if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
          // K is safe to invert.
          vc.K.setValues(k11, k12, k12, k22);
          vc.normalMass.setFrom(vc.K);
          vc.normalMass.invert();
        } else {
          // The constraints are redundant, just use one.
          // TODO_ERIN use deepest?
          vc.pointCount = 1;
        }
      }
    }
  }

  void solveVelocityConstraints() {
    for (int i = 0; i < _count; ++i) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];

      int indexA = vc.indexA;
      int indexB = vc.indexB;

      double mA = vc.invMassA;
      double mB = vc.invMassB;
      double iA = vc.invIA;
      double iB = vc.invIB;
      int pointCount = vc.pointCount;

      Vector2 vA = _velocities[indexA].v;
      double wA = _velocities[indexA].w;
      Vector2 vB = _velocities[indexB].v;
      double wB = _velocities[indexB].w;

      Vector2 normal = vc.normal;
      final double normalX = normal.x;
      final double normalY = normal.y;
      double tangentX = 1.0 * vc.normal.y;
      double tangentY = -1.0 * vc.normal.x;
      final double friction = vc.friction;

      assert(pointCount == 1 || pointCount == 2);

      // Solve tangent constraints
      for (int j = 0; j < pointCount; ++j) {
        final VelocityConstraintPoint vcp = vc.points[j];
        final Vector2 a = vcp.rA;
        double dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * a.y;
        double dvy = wB * vcp.rB.x + vB.y - vA.y - wA * a.x;

        // Compute tangent force
        final double vt = dvx * tangentX + dvy * tangentY - vc.tangentSpeed;
        double lambda = vcp.tangentMass * (-vt);

        // Clamp the accumulated force
        final double maxFriction = friction * vcp.normalImpulse;
        final double newImpulse = MathUtils.clampDouble(
            vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
        lambda = newImpulse - vcp.tangentImpulse;
        vcp.tangentImpulse = newImpulse;

        // Apply contact impulse
        // Vec2 P = lambda * tangent;

        final double Px = tangentX * lambda;
        final double Py = tangentY * lambda;

        // vA -= invMassA * P;
        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);

        // vB += invMassB * P;
        vB.x += Px * mB;
        vB.y += Py * mB;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
      }

      // Solve normal constraints
      if (vc.pointCount == 1) {
        final VelocityConstraintPoint vcp = vc.points[0];

        // Relative velocity at contact
        // Vec2 dv = vB + Cross(wB, vcp.rB) - vA - Cross(wA, vcp.rA);

        double dvx = -wB * vcp.rB.y + vB.x - vA.x + wA * vcp.rA.y;
        double dvy = wB * vcp.rB.x + vB.y - vA.y - wA * vcp.rA.x;

        // Compute normal impulse
        final double vn = dvx * normalX + dvy * normalY;
        double lambda = -vcp.normalMass * (vn - vcp.velocityBias);

        // Clamp the accumulated impulse
        double a = vcp.normalImpulse + lambda;
        final double newImpulse = (a > 0.0 ? a : 0.0);
        lambda = newImpulse - vcp.normalImpulse;
        vcp.normalImpulse = newImpulse;

        // Apply contact impulse
        double Px = normalX * lambda;
        double Py = normalY * lambda;

        vA.x -= Px * mA;
        vA.y -= Py * mA;
        wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);

        // vB += invMassB * P;
        vB.x += Px * mB;
        vB.y += Py * mB;
        wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);
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

        final VelocityConstraintPoint cp1 = vc.points[0];
        final VelocityConstraintPoint cp2 = vc.points[1];
        final Vector2 cp1rA = cp1.rA;
        final Vector2 cp1rB = cp1.rB;
        final Vector2 cp2rA = cp2.rA;
        final Vector2 cp2rB = cp2.rB;
        double ax = cp1.normalImpulse;
        double ay = cp2.normalImpulse;

        assert(ax >= 0.0 && ay >= 0.0);
        // Relative velocity at contact
        // Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
        double dv1x = -wB * cp1rB.y + vB.x - vA.x + wA * cp1rA.y;
        double dv1y = wB * cp1rB.x + vB.y - vA.y - wA * cp1rA.x;

        // Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
        double dv2x = -wB * cp2rB.y + vB.x - vA.x + wA * cp2rA.y;
        double dv2y = wB * cp2rB.x + vB.y - vA.y - wA * cp2rA.x;

        // Compute normal velocity
        double vn1 = dv1x * normalX + dv1y * normalY;
        double vn2 = dv2x * normalX + dv2y * normalY;

        double bx = vn1 - cp1.velocityBias;
        double by = vn2 - cp2.velocityBias;

        // Compute b'
        Matrix2 R = vc.K;
        bx -= R.entry(0, 0) * ax + R.entry(0, 1) * ay;
        by -= R.entry(1, 0) * ax + R.entry(1, 1) * ay;

        // final double k_errorTol = 1e-3f;
        // B2_NOT_USED(k_errorTol);
        for (;;) {
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
          Matrix2 R1 = vc.normalMass;
          double xx = R1.entry(0, 0) * bx + R1.entry(0, 1) * by;
          double xy = R1.entry(1, 0) * bx + R1.entry(1, 1) * by;
          xx *= -1;
          xy *= -1;

          if (xx >= 0.0 && xy >= 0.0) {
            // Get the incremental impulse
            double dx = xx - ax;
            double dy = xy - ay;

            // Apply incremental impulse
            double P1x = dx * normalX;
            double P1y = dx * normalY;
            double P2x = dy * normalX;
            double P2y = dy * normalY;

            /*
             * vA -= invMassA * (P1 + P2); wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

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
              Vector2 dv1 = vB + MathUtils.crossDblVec2(wB, cp1rB)
                ..sub(vA)
                ..sub(MathUtils.crossDblVec2(wA, cp1rA));
              Vector2 dv2 = vB + MathUtils.crossDblVec2(wB, cp2rB)
                ..sub(vA)
                ..sub(MathUtils.crossDblVec2(wA, cp2rA));
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
            double dx = xx - ax;
            double dy = xy - ay;

            // Apply incremental impulse
            double P1x = normalX * dx;
            double P1y = normalY * dx;
            double P2x = normalX * dy;
            double P2y = normalY * dy;

            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

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
              Vector2 dv1 = vB + MathUtils.crossDblVec2(wB, cp1rB)
                ..sub(vA)
                ..sub(MathUtils.crossDblVec2(wA, cp1rA));
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
            double dx = xx - ax;
            double dy = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            double P1x = normalX * dx;
            double P1y = normalY * dx;
            double P2x = normalX * dy;
            double P2y = normalY * dy;

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

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
              Vector2 dv2 = vB + MathUtils.crossDblVec2(wB, cp2rB)
                ..sub(vA)
                ..sub(MathUtils.crossDblVec2(wA, cp2rA));
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
            double dx = xx - ax;
            double dy = xy - ay;

            // Apply incremental impulse
            /*
             * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
             * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
             *
             * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
             */

            double P1x = normalX * dx;
            double P1y = normalY * dx;
            double P2x = normalX * dy;
            double P2y = normalY * dy;

            vA.x -= mA * (P1x + P2x);
            vA.y -= mA * (P1y + P2y);
            vB.x += mB * (P1x + P2x);
            vB.y += mB * (P1y + P2y);

            wA -= iA *
                (cp1rA.x * P1y -
                    cp1rA.y * P1x +
                    (cp2rA.x * P2y - cp2rA.y * P2x));
            wB += iB *
                (cp1rB.x * P1y -
                    cp1rB.y * P1x +
                    (cp2rB.x * P2y - cp2rB.y * P2x));

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
    for (int i = 0; i < _count; i++) {
      final ContactVelocityConstraint vc = _velocityConstraints[i];
      final Manifold manifold = _contacts[vc.contactIndex]._manifold;

      for (int j = 0; j < vc.pointCount; j++) {
        manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
        manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
      }
    }
  }

  final PositionSolverManifold _pSolver = PositionSolverManifold();

  /// Sequential solver.
  bool solvePositionConstraints() {
    double minSeparation = 0.0;

    for (int i = 0; i < _count; ++i) {
      ContactPositionConstraint pc = _positionConstraints[i];

      int indexA = pc.indexA;
      int indexB = pc.indexB;

      double mA = pc.invMassA;
      double iA = pc.invIA;
      Vector2 localCenterA = pc.localCenterA;
      final double localCenterAx = localCenterA.x;
      final double localCenterAy = localCenterA.y;
      double mB = pc.invMassB;
      double iB = pc.invIB;
      Vector2 localCenterB = pc.localCenterB;
      final double localCenterBx = localCenterB.x;
      final double localCenterBy = localCenterB.y;
      int pointCount = pc.pointCount;

      Vector2 cA = _positions[indexA].c;
      double aA = _positions[indexA].a;
      Vector2 cB = _positions[indexB].c;
      double aB = _positions[indexB].a;

      // Solve normal constraints
      for (int j = 0; j < pointCount; ++j) {
        final Rot xfAq = xfA.q;
        final Rot xfBq = xfB.q;
        xfAq.setAngle(aA);
        xfBq.setAngle(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        final PositionSolverManifold psm = _pSolver;
        psm.initialize(pc, xfA, xfB, j);
        final Vector2 normal = psm.normal;
        final Vector2 point = psm.point;
        final double separation = psm.separation;

        double rAx = point.x - cA.x;
        double rAy = point.y - cA.y;
        double rBx = point.x - cB.x;
        double rBy = point.y - cB.y;

        // Track max constraint error.
        minSeparation = Math.min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        final double C = MathUtils.clampDouble(
            Settings.baumgarte * (separation + Settings.linearSlop),
            -Settings.maxLinearCorrection,
            0.0);

        // Compute the effective mass.
        final double rnA = rAx * normal.y - rAy * normal.x;
        final double rnB = rBx * normal.y - rBy * normal.x;
        final double K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        final double impulse = K > 0.0 ? -C / K : 0.0;

        double Px = normal.x * impulse;
        double Py = normal.y * impulse;

        cA.x -= Px * mA;
        cA.y -= Py * mA;
        aA -= iA * (rAx * Py - rAy * Px);

        cB.x += Px * mB;
        cB.y += Py * mB;
        aB += iB * (rBx * Py - rBy * Px);
      }

      _positions[indexA].a = aA;
      _positions[indexB].a = aB;
    }

    // We can't expect minSeparation >= -linearSlop because we don't
    // push the separation above -linearSlop.
    return minSeparation >= -3.0 * Settings.linearSlop;
  }

  // Sequential position solver for position constraints.
  bool solveTOIPositionConstraints(int toiIndexA, int toiIndexB) {
    double minSeparation = 0.0;

    for (int i = 0; i < _count; ++i) {
      ContactPositionConstraint pc = _positionConstraints[i];

      int indexA = pc.indexA;
      int indexB = pc.indexB;
      Vector2 localCenterA = pc.localCenterA;
      Vector2 localCenterB = pc.localCenterB;
      final double localCenterAx = localCenterA.x;
      final double localCenterAy = localCenterA.y;
      final double localCenterBx = localCenterB.x;
      final double localCenterBy = localCenterB.y;
      int pointCount = pc.pointCount;

      double mA = 0.0;
      double iA = 0.0;
      if (indexA == toiIndexA || indexA == toiIndexB) {
        mA = pc.invMassA;
        iA = pc.invIA;
      }

      double mB = 0.0;
      double iB = 0.0;
      if (indexB == toiIndexA || indexB == toiIndexB) {
        mB = pc.invMassB;
        iB = pc.invIB;
      }

      Vector2 cA = _positions[indexA].c;
      double aA = _positions[indexA].a;

      Vector2 cB = _positions[indexB].c;
      double aB = _positions[indexB].a;

      // Solve normal constraints
      for (int j = 0; j < pointCount; ++j) {
        final Rot xfAq = xfA.q;
        final Rot xfBq = xfB.q;
        xfAq.setAngle(aA);
        xfBq.setAngle(aB);
        xfA.p.x = cA.x - xfAq.c * localCenterAx + xfAq.s * localCenterAy;
        xfA.p.y = cA.y - xfAq.s * localCenterAx - xfAq.c * localCenterAy;
        xfB.p.x = cB.x - xfBq.c * localCenterBx + xfBq.s * localCenterBy;
        xfB.p.y = cB.y - xfBq.s * localCenterBx - xfBq.c * localCenterBy;

        final PositionSolverManifold psm = _pSolver;
        psm.initialize(pc, xfA, xfB, j);
        Vector2 normal = psm.normal;

        Vector2 point = psm.point;
        double separation = psm.separation;

        double rAx = point.x - cA.x;
        double rAy = point.y - cA.y;
        double rBx = point.x - cB.x;
        double rBy = point.y - cB.y;

        // Track max constraint error.
        minSeparation = Math.min(minSeparation, separation);

        // Prevent large corrections and allow slop.
        double C = MathUtils.clampDouble(
            Settings.toiBaugarte * (separation + Settings.linearSlop),
            -Settings.maxLinearCorrection,
            0.0);

        // Compute the effective mass.
        double rnA = rAx * normal.y - rAy * normal.x;
        double rnB = rBx * normal.y - rBy * normal.x;
        double K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

        // Compute normal impulse
        double impulse = K > 0.0 ? -C / K : 0.0;

        double Px = normal.x * impulse;
        double Py = normal.y * impulse;

        cA.x -= Px * mA;
        cA.y -= Py * mA;
        aA -= iA * (rAx * Py - rAy * Px);

        cB.x += Px * mB;
        cB.y += Py * mB;
        aB += iB * (rBx * Py - rBy * Px);
      }

      _positions[indexA].a = aA;
      _positions[indexB].a = aB;
    }

    // We can't expect minSeparation >= -_linearSlop because we don't
    // push the separation above -_linearSlop.
    return minSeparation >= -1.5 * Settings.linearSlop;
  }
}

class PositionSolverManifold {
  final Vector2 normal = Vector2.zero();
  final Vector2 point = Vector2.zero();
  double separation = 0.0;

  void initialize(
      ContactPositionConstraint pc, Transform xfA, Transform xfB, int index) {
    assert(pc.pointCount > 0);

    final Rot xfAq = xfA.q;
    final Rot xfBq = xfB.q;
    final Vector2 pcLocalPointsI = pc.localPoints[index];
    switch (pc.type) {
      case ManifoldType.CIRCLES:
        final Vector2 plocalPoint = pc.localPoint;
        final Vector2 pLocalPoints0 = pc.localPoints[0];
        final double pointAx =
            (xfAq.c * plocalPoint.x - xfAq.s * plocalPoint.y) + xfA.p.x;
        final double pointAy =
            (xfAq.s * plocalPoint.x + xfAq.c * plocalPoint.y) + xfA.p.y;
        final double pointBx =
            (xfBq.c * pLocalPoints0.x - xfBq.s * pLocalPoints0.y) + xfB.p.x;
        final double pointBy =
            (xfBq.s * pLocalPoints0.x + xfBq.c * pLocalPoints0.y) + xfB.p.y;
        normal.x = pointBx - pointAx;
        normal.y = pointBy - pointAy;
        normal.normalize();

        point.x = (pointAx + pointBx) * .5;
        point.y = (pointAy + pointBy) * .5;
        final double tempx = pointBx - pointAx;
        final double tempy = pointBy - pointAy;
        separation =
            tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        break;

      case ManifoldType.FACE_A:
        final Vector2 pcLocalNormal = pc.localNormal;
        final Vector2 pcLocalPoint = pc.localPoint;
        normal.x = xfAq.c * pcLocalNormal.x - xfAq.s * pcLocalNormal.y;
        normal.y = xfAq.s * pcLocalNormal.x + xfAq.c * pcLocalNormal.y;
        final double planePointx =
            (xfAq.c * pcLocalPoint.x - xfAq.s * pcLocalPoint.y) + xfA.p.x;
        final double planePointy =
            (xfAq.s * pcLocalPoint.x + xfAq.c * pcLocalPoint.y) + xfA.p.y;

        final double clipPointx =
            (xfBq.c * pcLocalPointsI.x - xfBq.s * pcLocalPointsI.y) + xfB.p.x;
        final double clipPointy =
            (xfBq.s * pcLocalPointsI.x + xfBq.c * pcLocalPointsI.y) + xfB.p.y;
        final double tempx = clipPointx - planePointx;
        final double tempy = clipPointy - planePointy;
        separation =
            tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointx;
        point.y = clipPointy;
        break;

      case ManifoldType.FACE_B:
        final Vector2 pcLocalNormal = pc.localNormal;
        final Vector2 pcLocalPoint = pc.localPoint;
        normal.x = xfBq.c * pcLocalNormal.x - xfBq.s * pcLocalNormal.y;
        normal.y = xfBq.s * pcLocalNormal.x + xfBq.c * pcLocalNormal.y;
        final double planePointx =
            (xfBq.c * pcLocalPoint.x - xfBq.s * pcLocalPoint.y) + xfB.p.x;
        final double planePointy =
            (xfBq.s * pcLocalPoint.x + xfBq.c * pcLocalPoint.y) + xfB.p.y;

        final double clipPointx =
            (xfAq.c * pcLocalPointsI.x - xfAq.s * pcLocalPointsI.y) + xfA.p.x;
        final double clipPointy =
            (xfAq.s * pcLocalPointsI.x + xfAq.c * pcLocalPointsI.y) + xfA.p.y;
        final double tempx = clipPointx - planePointx;
        final double tempy = clipPointy - planePointy;
        separation =
            tempx * normal.x + tempy * normal.y - pc.radiusA - pc.radiusB;
        point.x = clipPointx;
        point.y = clipPointy;
        normal.x *= -1;
        normal.y *= -1;
        break;
    }
  }
}
