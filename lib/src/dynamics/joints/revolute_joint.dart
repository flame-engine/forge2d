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
//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Motor constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A revolute joint constrains two bodies to share a common point while they are free to rotate
 * about the point. The relative rotation about the shared point is the joint angle. You can limit
 * the relative rotation with a joint limit that specifies a lower and upper angle. You can use a
 * motor to drive the relative rotation about the shared point. A maximum motor torque is provided
 * so that infinite forces are not generated.
 * 
 * @author Daniel Murphy
 */
class RevoluteJoint extends Joint {

  // Solver shared
  final Vec2 m_localAnchorA = new Vec2.zero();
  final Vec2 m_localAnchorB = new Vec2.zero();
  // TODO(srdjan): Mark fields below private.
  final Vec3 m_impulse = new Vec3.zero();
  double m_motorImpulse = 0.0;

  bool m_enableMotor = false;
  double m_maxMotorTorque = 0.0;
  double m_motorSpeed = 0.0;

  bool m_enableLimit = false;
  double m_referenceAngle = 0.0;
  double m_lowerAngle = 0.0;
  double m_upperAngle = 0.0;

  // Solver temp
  int m_indexA = 0;
  int m_indexB = 0;
  final Vec2 m_rA = new Vec2.zero();
  final Vec2 m_rB = new Vec2.zero();
  final Vec2 m_localCenterA = new Vec2.zero();
  final Vec2 m_localCenterB = new Vec2.zero();
  double m_invMassA = 0.0;
  double m_invMassB = 0.0;
  double m_invIA = 0.0;
  double m_invIB = 0.0;
  final Mat33 m_mass =
      new Mat33.zero(); // effective mass for point-to-point constraint.
  double m_motorMass =
      0.0; // effective mass for motor/limit angular constraint.
  LimitState m_limitState = LimitState.INACTIVE;

  RevoluteJoint(IWorldPool argWorld, RevoluteJointDef def)
      : super(argWorld, def) {
    m_localAnchorA.set(def.localAnchorA);
    m_localAnchorB.set(def.localAnchorB);
    m_referenceAngle = def.referenceAngle;

    m_lowerAngle = def.lowerAngle;
    m_upperAngle = def.upperAngle;
    m_maxMotorTorque = def.maxMotorTorque;
    m_motorSpeed = def.motorSpeed;
    m_enableLimit = def.enableLimit;
    m_enableMotor = def.enableMotor;
  }

  void initVelocityConstraints(final SolverData data) {
    m_indexA = m_bodyA.m_islandIndex;
    m_indexB = m_bodyB.m_islandIndex;
    m_localCenterA.set(m_bodyA.m_sweep.localCenter);
    m_localCenterB.set(m_bodyB.m_sweep.localCenter);
    m_invMassA = m_bodyA.m_invMass;
    m_invMassB = m_bodyB.m_invMass;
    m_invIA = m_bodyA.m_invI;
    m_invIB = m_bodyB.m_invI;

    // Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vec2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;

    // Vec2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;
    Vec2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), m_rA);
    Rot.mulToOutUnsafe(
        qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), m_rB);

    // J = [-I -r1_skew I r2_skew]
    // [ 0 -1 0 1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
    // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
    // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    bool fixedRotation = (iA + iB == 0.0);

    m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
    m_mass.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
    m_mass.ez.x = -m_rA.y * iA - m_rB.y * iB;
    m_mass.ex.y = m_mass.ey.x;
    m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
    m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
    m_mass.ex.z = m_mass.ez.x;
    m_mass.ey.z = m_mass.ez.y;
    m_mass.ez.z = iA + iB;

    m_motorMass = iA + iB;
    if (m_motorMass > 0.0) {
      m_motorMass = 1.0 / m_motorMass;
    }

    if (m_enableMotor == false || fixedRotation) {
      m_motorImpulse = 0.0;
    }

    if (m_enableLimit && fixedRotation == false) {
      double jointAngle = aB - aA - m_referenceAngle;
      if ((m_upperAngle - m_lowerAngle).abs() < 2.0 * Settings.angularSlop) {
        m_limitState = LimitState.EQUAL;
      } else if (jointAngle <= m_lowerAngle) {
        if (m_limitState != LimitState.AT_LOWER) {
          m_impulse.z = 0.0;
        }
        m_limitState = LimitState.AT_LOWER;
      } else if (jointAngle >= m_upperAngle) {
        if (m_limitState != LimitState.AT_UPPER) {
          m_impulse.z = 0.0;
        }
        m_limitState = LimitState.AT_UPPER;
      } else {
        m_limitState = LimitState.INACTIVE;
        m_impulse.z = 0.0;
      }
    } else {
      m_limitState = LimitState.INACTIVE;
    }

    if (data.step.warmStarting) {
      final Vec2 P = pool.popVec2();
      // Scale impulses to support a variable time step.
      m_impulse.x *= data.step.dtRatio;
      m_impulse.y *= data.step.dtRatio;
      m_motorImpulse *= data.step.dtRatio;

      P.x = m_impulse.x;
      P.y = m_impulse.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.cross(m_rA, P) + m_motorImpulse + m_impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.cross(m_rB, P) + m_motorImpulse + m_impulse.z);
      pool.pushVec2(1);
    } else {
      m_impulse.setZero();
      m_motorImpulse = 0.0;
    }
    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushRot(2);
  }

  void solveVelocityConstraints(final SolverData data) {
    Vec2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;
    Vec2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    bool fixedRotation = (iA + iB == 0.0);

    // Solve motor constraint.
    if (m_enableMotor &&
        m_limitState != LimitState.EQUAL &&
        fixedRotation == false) {
      double Cdot = wB - wA - m_motorSpeed;
      double impulse = -m_motorMass * Cdot;
      double oldImpulse = m_motorImpulse;
      double maxImpulse = data.step.dt * m_maxMotorTorque;
      m_motorImpulse = MathUtils.clampDouble(
          m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }
    final Vec2 temp = pool.popVec2();

    // Solve limit constraint.
    if (m_enableLimit &&
        m_limitState != LimitState.INACTIVE &&
        fixedRotation == false) {
      final Vec2 Cdot1 = pool.popVec2();
      final Vec3 Cdot = pool.popVec3();

      // Solve point-to-point constraint
      Vec2.crossToOutUnsafeDblVec2(wA, m_rA, temp);
      Vec2.crossToOutUnsafeDblVec2(wB, m_rB, Cdot1);
      Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);
      double Cdot2 = wB - wA;
      Cdot.setXYZ(Cdot1.x, Cdot1.y, Cdot2);

      Vec3 impulse = pool.popVec3();
      m_mass.solve33ToOut(Cdot, impulse);
      impulse.negateLocal();

      if (m_limitState == LimitState.EQUAL) {
        m_impulse.addLocal(impulse);
      } else if (m_limitState == LimitState.AT_LOWER) {
        double newImpulse = m_impulse.z + impulse.z;
        if (newImpulse < 0.0) {
          final Vec2 rhs = pool.popVec2();
          rhs
              .setXY(m_mass.ez.x, m_mass.ez.y)
              .mulLocal(m_impulse.z)
              .subLocal(Cdot1);
          m_mass.solve22ToOut(rhs, temp);
          impulse.x = temp.x;
          impulse.y = temp.y;
          impulse.z = -m_impulse.z;
          m_impulse.x += temp.x;
          m_impulse.y += temp.y;
          m_impulse.z = 0.0;
          pool.pushVec2(1);
        } else {
          m_impulse.addLocal(impulse);
        }
      } else if (m_limitState == LimitState.AT_UPPER) {
        double newImpulse = m_impulse.z + impulse.z;
        if (newImpulse > 0.0) {
          final Vec2 rhs = pool.popVec2();
          rhs
              .setXY(m_mass.ez.x, m_mass.ez.y)
              .mulLocal(m_impulse.z)
              .subLocal(Cdot1);
          m_mass.solve22ToOut(rhs, temp);
          impulse.x = temp.x;
          impulse.y = temp.y;
          impulse.z = -m_impulse.z;
          m_impulse.x += temp.x;
          m_impulse.y += temp.y;
          m_impulse.z = 0.0;
          pool.pushVec2(1);
        } else {
          m_impulse.addLocal(impulse);
        }
      }
      final Vec2 P = pool.popVec2();

      P.setXY(impulse.x, impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.cross(m_rA, P) + impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.cross(m_rB, P) + impulse.z);

      pool.pushVec2(2);
      pool.pushVec3(2);
    } else {

      // Solve point-to-point constraint
      Vec2 Cdot = pool.popVec2();
      Vec2 impulse = pool.popVec2();

      Vec2.crossToOutUnsafeDblVec2(wA, m_rA, temp);
      Vec2.crossToOutUnsafeDblVec2(wB, m_rB, Cdot);
      Cdot.addLocal(vB).subLocal(vA).subLocal(temp);
      m_mass.solve22ToOut(Cdot.negateLocal(), impulse); // just leave negated

      m_impulse.x += impulse.x;
      m_impulse.y += impulse.y;

      vA.x -= mA * impulse.x;
      vA.y -= mA * impulse.y;
      wA -= iA * Vec2.cross(m_rA, impulse);

      vB.x += mB * impulse.x;
      vB.y += mB * impulse.y;
      wB += iB * Vec2.cross(m_rB, impulse);

      pool.pushVec2(2);
    }

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
  }

  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vec2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    double angularError = 0.0;
    double positionError = 0.0;

    bool fixedRotation = (m_invIA + m_invIB == 0.0);

    // Solve angular limit constraint.
    if (m_enableLimit &&
        m_limitState != LimitState.INACTIVE &&
        fixedRotation == false) {
      double angle = aB - aA - m_referenceAngle;
      double limitImpulse = 0.0;

      if (m_limitState == LimitState.EQUAL) {
        // Prevent large angular corrections
        double C = MathUtils.clampDouble(angle - m_lowerAngle,
            -Settings.maxAngularCorrection, Settings.maxAngularCorrection);
        limitImpulse = -m_motorMass * C;
        angularError = C.abs();
      } else if (m_limitState == LimitState.AT_LOWER) {
        double C = angle - m_lowerAngle;
        angularError = -C;

        // Prevent large angular corrections and allow some slop.
        C = MathUtils.clampDouble(
            C + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0);
        limitImpulse = -m_motorMass * C;
      } else if (m_limitState == LimitState.AT_UPPER) {
        double C = angle - m_upperAngle;
        angularError = C;

        // Prevent large angular corrections and allow some slop.
        C = MathUtils.clampDouble(
            C - Settings.angularSlop, 0.0, Settings.maxAngularCorrection);
        limitImpulse = -m_motorMass * C;
      }

      aA -= m_invIA * limitImpulse;
      aB += m_invIB * limitImpulse;
    }
    // Solve point-to-point constraint.
    {
      qA.setAngle(aA);
      qB.setAngle(aB);

      final Vec2 rA = pool.popVec2();
      final Vec2 rB = pool.popVec2();
      final Vec2 C = pool.popVec2();
      final Vec2 impulse = pool.popVec2();

      Rot.mulToOutUnsafe(
          qA, C.set(m_localAnchorA).subLocal(m_localCenterA), rA);
      Rot.mulToOutUnsafe(
          qB, C.set(m_localAnchorB).subLocal(m_localCenterB), rB);
      C.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);
      positionError = C.length();

      double mA = m_invMassA,
          mB = m_invMassB;
      double iA = m_invIA,
          iB = m_invIB;

      final Mat22 K = pool.popMat22();
      K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
      K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
      K.ey.x = K.ex.y;
      K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
      K.solveToOut(C, impulse);
      impulse.negateLocal();

      cA.x -= mA * impulse.x;
      cA.y -= mA * impulse.y;
      aA -= iA * Vec2.cross(rA, impulse);

      cB.x += mB * impulse.x;
      cB.y += mB * impulse.y;
      aB += iB * Vec2.cross(rB, impulse);

      pool.pushVec2(4);
      pool.pushMat22(1);
    }
    // data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushRot(2);

    return positionError <= Settings.linearSlop &&
        angularError <= Settings.angularSlop;
  }

  Vec2 getLocalAnchorA() {
    return m_localAnchorA;
  }

  Vec2 getLocalAnchorB() {
    return m_localAnchorB;
  }

  double getReferenceAngle() {
    return m_referenceAngle;
  }

  void getAnchorA(Vec2 argOut) {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  void getAnchorB(Vec2 argOut) {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vec2 argOut) {
    argOut.setXY(m_impulse.x, m_impulse.y).mulLocal(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * m_impulse.z;
  }

  double getJointAngle() {
    final Body b1 = m_bodyA;
    final Body b2 = m_bodyB;
    return b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
  }

  double getJointSpeed() {
    final Body b1 = m_bodyA;
    final Body b2 = m_bodyB;
    return b2.m_angularVelocity - b1.m_angularVelocity;
  }

  bool isMotorEnabled() {
    return m_enableMotor;
  }

  void enableMotor(bool flag) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_enableMotor = flag;
  }

  double getMotorTorque(double inv_dt) {
    return m_motorImpulse * inv_dt;
  }

  void setMotorSpeed(final double speed) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_motorSpeed = speed;
  }

  void setMaxMotorTorque(final double torque) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_maxMotorTorque = torque;
  }

  double getMotorSpeed() {
    return m_motorSpeed;
  }

  double getMaxMotorTorque() {
    return m_maxMotorTorque;
  }

  bool isLimitEnabled() {
    return m_enableLimit;
  }

  void enableLimit(final bool flag) {
    if (flag != m_enableLimit) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_enableLimit = flag;
      m_impulse.z = 0.0;
    }
  }

  double getLowerLimit() {
    return m_lowerAngle;
  }

  double getUpperLimit() {
    return m_upperAngle;
  }

  void setLimits(final double lower, final double upper) {
    assert(lower <= upper);
    if (lower != m_lowerAngle || upper != m_upperAngle) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_impulse.z = 0.0;
      m_lowerAngle = lower;
      m_upperAngle = upper;
    }
  }
}
