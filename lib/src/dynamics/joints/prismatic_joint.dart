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

//Linear constraint (point-to-line)
//d = p2 - p1 = x2 + r2 - x1 - r1
//C = dot(perp, d)
//Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//   = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
//J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
//Angular constraint
//C = a2 - a1 + a_initial
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//
//K = J * invM * JT
//
//J = [-a -s1 a s2]
//  [0  -1  0  1]
//a = perp
//s1 = cross(d + r1, a) = cross(p2 - x1, a)
//s2 = cross(r2, a) = cross(p2 - x2, a)

//Motor/Limit linear constraint
//C = dot(ax1, d)
//Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
//J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

//Block Solver
//We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
//when the mass has poor distribution (leading to large torques about the joint anchor points).
//
//The Jacobian has 3 rows:
//J = [-uT -s1 uT s2] // linear
//  [0   -1   0  1] // angular
//  [-vT -a1 vT a2] // limit
//
//u = perp
//v = axis
//s1 = cross(d + r1, u), s2 = cross(r2, u)
//a1 = cross(d + r1, v), a2 = cross(r2, v)

//M * (v2 - v1) = JT * df
//J * v2 = bias
//
//v2 = v1 + invM * JT * df
//J * (v1 + invM * JT * df) = bias
//K * df = bias - J * v1 = -Cdot
//K = J * invM * JT
//Cdot = J * v1 - bias
//
//Now solve for f2.
//df = f2 - f1
//K * (f2 - f1) = -Cdot
//f2 = invK * (-Cdot) + f1
//
//Clamp accumulated limit impulse.
//lower: f2(3) = max(f2(3), 0)
//upper: f2(3) = min(f2(3), 0)
//
//Solve for correct f2(1:2)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                    = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
//f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
//Now compute impulse to be applied:
//df = f2 - f1

/**
 * A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in
 * bodyA. Relative rotation is prevented. You can use a joint limit to restrict the range of motion
 * and a joint motor to drive the motion or to model joint friction.
 *
 * @author Daniel
 */
class PrismaticJoint extends Joint {

  // Solver shared
  final Vector2 m_localAnchorA;
  final Vector2 m_localAnchorB;
  final Vector2 m_localXAxisA;
  final Vector2 m_localYAxisA;
  double m_referenceAngle;

  // TODO(srdjan): Make fields below private.
  final Vec3 m_impulse = new Vec3.zero();
  double m_motorImpulse = 0.0;
  double m_lowerTranslation = 0.0;
  double m_upperTranslation = 0.0;
  double m_maxMotorForce = 0.0;
  double m_motorSpeed = 0.0;
  bool m_enableLimit = false;
  bool m_enableMotor = false;
  LimitState m_limitState;

  // Solver temp
  int m_indexA = 0;
  int m_indexB = 0;
  final Vector2 m_localCenterA = new Vector2.zero();
  final Vector2 m_localCenterB = new Vector2.zero();
  double m_invMassA = 0.0;
  double m_invMassB = 0.0;
  double m_invIA = 0.0;
  double m_invIB = 0.0;
  final Vector2 m_axis = new Vector2.zero();
  final Vector2 m_perp = new Vector2.zero();
  double m_s1 = 0.0,
      m_s2 = 0.0;
  double m_a1 = 0.0,
      m_a2 = 0.0;
  final Mat33 m_K = new Mat33.zero();
  double m_motorMass =
      0.0; // effective mass for motor/limit translational constraint.

  PrismaticJoint(IWorldPool argWorld, PrismaticJointDef def)
      : m_localAnchorA = new Vector2.copy(def.localAnchorA),
        m_localAnchorB = new Vector2.copy(def.localAnchorB),
        m_localXAxisA = new Vector2.copy(def.localAxisA)..normalize(),
        m_localYAxisA = new Vector2.zero(),
        super(argWorld, def) {
    Vector2.crossToOutUnsafeDblVec2(1.0, m_localXAxisA, m_localYAxisA);
    m_referenceAngle = def.referenceAngle;

    m_lowerTranslation = def.lowerTranslation;
    m_upperTranslation = def.upperTranslation;
    m_maxMotorForce = def.maxMotorForce;
    m_motorSpeed = def.motorSpeed;
    m_enableLimit = def.enableLimit;
    m_enableMotor = def.enableMotor;
    m_limitState = LimitState.INACTIVE;
  }

  Vector2 getLocalAnchorA() {
    return m_localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return m_localAnchorB;
  }

  void getAnchorA(Vector2 argOut) {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vector2 argOut) {
    Vector2 temp = pool.popVec2();
    temp.set(m_axis).mul(m_motorImpulse + m_impulse.z);
    argOut.set(m_perp).mul(m_impulse.x).add(temp).mul(inv_dt);
    pool.pushVec2(1);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * m_impulse.y;
  }

  /**
   * Get the current joint translation, usually in meters.
   */
  double getJointSpeed() {
    Body bA = m_bodyA;
    Body bB = m_bodyB;

    Vector2 temp = pool.popVec2();
    Vector2 rA = pool.popVec2();
    Vector2 rB = pool.popVec2();
    Vector2 p1 = pool.popVec2();
    Vector2 p2 = pool.popVec2();
    Vector2 d = pool.popVec2();
    Vector2 axis = pool.popVec2();
    Vector2 temp2 = pool.popVec2();
    Vector2 temp3 = pool.popVec2();

    temp.set(m_localAnchorA).sub(bA.m_sweep.localCenter);
    Rot.mulToOutUnsafe(bA.m_xf.q, temp, rA);

    temp.set(m_localAnchorB).sub(bB.m_sweep.localCenter);
    Rot.mulToOutUnsafe(bB.m_xf.q, temp, rB);

    p1.set(bA.m_sweep.c).add(rA);
    p2.set(bB.m_sweep.c).add(rB);

    d.set(p2).sub(p1);
    Rot.mulToOutUnsafe(bA.m_xf.q, m_localXAxisA, axis);

    Vector2 vA = bA._linearVelocity;
    Vector2 vB = bB._linearVelocity;
    double wA = bA._angularVelocity;
    double wB = bB._angularVelocity;

    Vector2.crossToOutUnsafeDblVec2(wA, axis, temp);
    Vector2.crossToOutUnsafeDblVec2(wB, rB, temp2);
    Vector2.crossToOutUnsafeDblVec2(wA, rA, temp3);

    temp2.add(vB).sub(vA).sub(temp3);
    double speed = Vector2.dot(d, temp) + Vector2.dot(axis, temp2);

    pool.pushVec2(9);

    return speed;
  }

  double getJointTranslation() {
    Vector2 pA = pool.popVec2(),
        pB = pool.popVec2(),
        axis = pool.popVec2();
    m_bodyA.getWorldPointToOut(m_localAnchorA, pA);
    m_bodyB.getWorldPointToOut(m_localAnchorB, pB);
    m_bodyA.getWorldVectorToOutUnsafe(m_localXAxisA, axis);
    pB.sub(pA);
    double translation = Vector2.dot(pB, axis);
    pool.pushVec2(3);
    return translation;
  }

  /**
   * Is the joint limit enabled?
   *
   * @return
   */
  bool isLimitEnabled() {
    return m_enableLimit;
  }

  /**
   * Enable/disable the joint limit.
   *
   * @param flag
   */
  void enableLimit(bool flag) {
    if (flag != m_enableLimit) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_enableLimit = flag;
      m_impulse.z = 0.0;
    }
  }

  /**
   * Get the lower joint limit, usually in meters.
   *
   * @return
   */
  double getLowerLimit() {
    return m_lowerTranslation;
  }

  /**
   * Get the upper joint limit, usually in meters.
   *
   * @return
   */
  double getUpperLimit() {
    return m_upperTranslation;
  }

  /**
   * Set the joint limits, usually in meters.
   *
   * @param lower
   * @param upper
   */
  void setLimits(double lower, double upper) {
    assert(lower <= upper);
    if (lower != m_lowerTranslation || upper != m_upperTranslation) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      m_lowerTranslation = lower;
      m_upperTranslation = upper;
      m_impulse.z = 0.0;
    }
  }

  /**
   * Is the joint motor enabled?
   *
   * @return
   */
  bool isMotorEnabled() {
    return m_enableMotor;
  }

  /**
   * Enable/disable the joint motor.
   *
   * @param flag
   */
  void enableMotor(bool flag) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_enableMotor = flag;
  }

  /**
   * Set the motor speed, usually in meters per second.
   *
   * @param speed
   */
  void setMotorSpeed(double speed) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_motorSpeed = speed;
  }

  /**
   * Get the motor speed, usually in meters per second.
   *
   * @return
   */
  double getMotorSpeed() {
    return m_motorSpeed;
  }

  /**
   * Set the maximum motor force, usually in N.
   *
   * @param force
   */
  void setMaxMotorForce(double force) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_maxMotorForce = force;
  }

  /**
   * Get the current motor force, usually in N.
   *
   * @param inv_dt
   * @return
   */
  double getMotorForce(double inv_dt) {
    return m_motorImpulse * inv_dt;
  }

  double getMaxMotorForce() {
    return m_maxMotorForce;
  }

  double getReferenceAngle() {
    return m_referenceAngle;
  }

  Vector2 getLocalAxisA() {
    return m_localXAxisA;
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

    Vector2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vector2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;

    Vector2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;
    Vector2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 d = pool.popVec2();
    final Vector2 temp = pool.popVec2();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(qA, d.set(m_localAnchorA).sub(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, d.set(m_localAnchorB).sub(m_localCenterB), rB);
    d.set(cB).sub(cA).add(rB).sub(rA);

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    // Compute motor Jacobian and effective mass.
    {
      Rot.mulToOutUnsafe(qA, m_localXAxisA, m_axis);
      temp.set(d).add(rA);
      m_a1 = Vector2.cross(temp, m_axis);
      m_a2 = Vector2.cross(rB, m_axis);

      m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
      if (m_motorMass > 0.0) {
        m_motorMass = 1.0 / m_motorMass;
      }
    }

    // Prismatic constraint.
    {
      Rot.mulToOutUnsafe(qA, m_localYAxisA, m_perp);

      temp.set(d).add(rA);
      m_s1 = Vector2.cross(temp, m_perp);
      m_s2 = Vector2.cross(rB, m_perp);

      double k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
      double k12 = iA * m_s1 + iB * m_s2;
      double k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        // For bodies with fixed rotation.
        k22 = 1.0;
      }
      double k23 = iA * m_a1 + iB * m_a2;
      double k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

      m_K.ex.setXYZ(k11, k12, k13);
      m_K.ey.setXYZ(k12, k22, k23);
      m_K.ez.setXYZ(k13, k23, k33);
    }

    // Compute motor and limit terms.
    if (m_enableLimit) {
      double jointTranslation = Vector2.dot(m_axis, d);
      if ((m_upperTranslation - m_lowerTranslation).abs() <
          2.0 * Settings.linearSlop) {
        m_limitState = LimitState.EQUAL;
      } else if (jointTranslation <= m_lowerTranslation) {
        if (m_limitState != LimitState.AT_LOWER) {
          m_limitState = LimitState.AT_LOWER;
          m_impulse.z = 0.0;
        }
      } else if (jointTranslation >= m_upperTranslation) {
        if (m_limitState != LimitState.AT_UPPER) {
          m_limitState = LimitState.AT_UPPER;
          m_impulse.z = 0.0;
        }
      } else {
        m_limitState = LimitState.INACTIVE;
        m_impulse.z = 0.0;
      }
    } else {
      m_limitState = LimitState.INACTIVE;
      m_impulse.z = 0.0;
    }

    if (m_enableMotor == false) {
      m_motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      m_impulse.mulLocal(data.step.dtRatio);
      m_motorImpulse *= data.step.dtRatio;

      final Vector2 P = pool.popVec2();
      temp.set(m_axis).mul(m_motorImpulse + m_impulse.z);
      P.set(m_perp).mul(m_impulse.x).add(temp);

      double LA = m_impulse.x * m_s1 +
          m_impulse.y +
          (m_motorImpulse + m_impulse.z) * m_a1;
      double LB = m_impulse.x * m_s2 +
          m_impulse.y +
          (m_motorImpulse + m_impulse.z) * m_a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(1);
    } else {
      m_impulse.setZero();
      m_motorImpulse = 0.0;
    }

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushRot(2);
    pool.pushVec2(4);
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;
    Vector2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    final Vector2 temp = pool.popVec2();

    // Solve linear motor constraint.
    if (m_enableMotor && m_limitState != LimitState.EQUAL) {
      temp.set(vB).sub(vA);
      double Cdot = Vector2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;
      double impulse = m_motorMass * (m_motorSpeed - Cdot);
      double oldImpulse = m_motorImpulse;
      double maxImpulse = data.step.dt * m_maxMotorForce;
      m_motorImpulse = MathUtils.clampDouble(
          m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_motorImpulse - oldImpulse;

      final Vector2 P = pool.popVec2();
      P.set(m_axis).mul(impulse);
      double LA = impulse * m_a1;
      double LB = impulse * m_a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(1);
    }

    final Vector2 Cdot1 = pool.popVec2();
    temp.set(vB).sub(vA);
    Cdot1.x = Vector2.dot(m_perp, temp) + m_s2 * wB - m_s1 * wA;
    Cdot1.y = wB - wA;
    // System.out.println(Cdot1);

    if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
      // Solve prismatic and limit constraint in block form.
      double Cdot2;
      temp.set(vB).sub(vA);
      Cdot2 = Vector2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;

      final Vec3 Cdot = pool.popVec3();
      Cdot.setXYZ(Cdot1.x, Cdot1.y, Cdot2);

      final Vec3 f1 = pool.popVec3();
      final Vec3 df = pool.popVec3();

      f1.set(m_impulse);
      m_K.solve33ToOut(Cdot.negateLocal(), df);
      // Cdot.negateLocal(); not used anymore
      m_impulse.addLocal(df);

      if (m_limitState == LimitState.AT_LOWER) {
        m_impulse.z = Math.max(m_impulse.z, 0.0);
      } else if (m_limitState == LimitState.AT_UPPER) {
        m_impulse.z = Math.min(m_impulse.z, 0.0);
      }

      // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
      // f1(1:2)
      final Vector2 b = pool.popVec2();
      final Vector2 f2r = pool.popVec2();

      temp.setValues(m_K.ez.x, m_K.ez.y).mul(m_impulse.z - f1.z);
      b.set(Cdot1).negate().sub(temp);

      m_K.solve22ToOut(b, f2r);
      f2r.addValues(f1.x, f1.y);
      m_impulse.x = f2r.x;
      m_impulse.y = f2r.y;

      df.set(m_impulse).subLocal(f1);

      final Vector2 P = pool.popVec2();
      temp.set(m_axis).mul(df.z);
      P.set(m_perp).mul(df.x).add(temp);

      double LA = df.x * m_s1 + df.y + df.z * m_a1;
      double LB = df.x * m_s2 + df.y + df.z * m_a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(3);
      pool.pushVec3(3);
    } else {
      // Limit is inactive, just solve the prismatic constraint in block form.
      final Vector2 df = pool.popVec2();
      m_K.solve22ToOut(Cdot1.negate(), df);
      Cdot1.negate();

      m_impulse.x += df.x;
      m_impulse.y += df.y;

      final Vector2 P = pool.popVec2();
      P.set(m_perp).mul(df.x);
      double LA = df.x * m_s1 + df.y;
      double LB = df.x * m_s2 + df.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;

      pool.pushVec2(2);
    }

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(2);
  }

  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 d = pool.popVec2();
    final Vector2 axis = pool.popVec2();
    final Vector2 perp = pool.popVec2();
    final Vector2 temp = pool.popVec2();
    final Vector2 C1 = pool.popVec2();

    final Vec3 impulse = pool.popVec3();

    Vector2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vector2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    // Compute fresh Jacobians
    Rot.mulToOutUnsafe(
        qA, temp.set(m_localAnchorA).sub(m_localCenterA), rA);
    Rot.mulToOutUnsafe(
        qB, temp.set(m_localAnchorB).sub(m_localCenterB), rB);
    d.set(cB).add(rB).sub(cA).sub(rA);

    Rot.mulToOutUnsafe(qA, m_localXAxisA, axis);
    double a1 = Vector2.cross(temp.set(d).add(rA), axis);
    double a2 = Vector2.cross(rB, axis);
    Rot.mulToOutUnsafe(qA, m_localYAxisA, perp);

    double s1 = Vector2.cross(temp.set(d).add(rA), perp);
    double s2 = Vector2.cross(rB, perp);

    C1.x = Vector2.dot(perp, d);
    C1.y = aB - aA - m_referenceAngle;

    double linearError = C1.x.abs();
    double angularError = C1.y.abs();

    bool active = false;
    double C2 = 0.0;
    if (m_enableLimit) {
      double translation = Vector2.dot(axis, d);
      if ((m_upperTranslation - m_lowerTranslation).abs() <
          2.0 * Settings.linearSlop) {
        // Prevent large angular corrections
        C2 = MathUtils.clampDouble(translation, -Settings.maxLinearCorrection,
            Settings.maxLinearCorrection);
        linearError = Math.max(linearError, translation.abs());
        active = true;
      } else if (translation <= m_lowerTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 = MathUtils.clampDouble(
            translation - m_lowerTranslation + Settings.linearSlop,
            -Settings.maxLinearCorrection, 0.0);
        linearError = Math.max(linearError, m_lowerTranslation - translation);
        active = true;
      } else if (translation >= m_upperTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 = MathUtils.clampDouble(
            translation - m_upperTranslation - Settings.linearSlop, 0.0,
            Settings.maxLinearCorrection);
        linearError = Math.max(linearError, translation - m_upperTranslation);
        active = true;
      }
    }

    if (active) {
      double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      double k12 = iA * s1 + iB * s2;
      double k13 = iA * s1 * a1 + iB * s2 * a2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        // For fixed rotation
        k22 = 1.0;
      }
      double k23 = iA * a1 + iB * a2;
      double k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

      final Mat33 K = pool.popMat33();
      K.ex.setXYZ(k11, k12, k13);
      K.ey.setXYZ(k12, k22, k23);
      K.ez.setXYZ(k13, k23, k33);

      final Vec3 C = pool.popVec3();
      C.x = C1.x;
      C.y = C1.y;
      C.z = C2;

      K.solve33ToOut(C.negateLocal(), impulse);
      pool.pushVec3(1);
      pool.pushMat33(1);
    } else {
      double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      double k12 = iA * s1 + iB * s2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        k22 = 1.0;
      }

      final Mat22 K = pool.popMat22();
      K.ex.setValues(k11, k12);
      K.ey.setValues(k12, k22);

      // temp is impulse1
      K.solveToOut(C1.negate(), temp);
      C1.negate();

      impulse.x = temp.x;
      impulse.y = temp.y;
      impulse.z = 0.0;

      pool.pushMat22(1);
    }

    double Px = impulse.x * perp.x + impulse.z * axis.x;
    double Py = impulse.x * perp.y + impulse.z * axis.y;
    double LA = impulse.x * s1 + impulse.y + impulse.z * a1;
    double LB = impulse.x * s2 + impulse.y + impulse.z * a2;

    cA.x -= mA * Px;
    cA.y -= mA * Py;
    aA -= iA * LA;
    cB.x += mB * Px;
    cB.y += mB * Py;
    aB += iB * LB;

    // data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushVec2(7);
    pool.pushVec3(1);
    pool.pushRot(2);

    return linearError <= Settings.linearSlop &&
        angularError <= Settings.angularSlop;
  }
}
