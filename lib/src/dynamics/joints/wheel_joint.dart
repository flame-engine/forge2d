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
//d = pB - pA = xB + rB - xA - rA
//C = dot(ay, d)
//Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//   = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
//J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

//Spring linear constraint
//C = dot(ax, d)
//Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
//J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

//Motor rotational constraint
//Cdot = wB - wA
//J = [0 0 -1 0 0 1]

/**
 * A wheel joint. This joint provides two degrees of freedom: translation along an axis fixed in
 * bodyA and rotation in the plane. You can use a joint limit to restrict the range of motion and a
 * joint motor to drive the rotation or to model rotational friction. This joint is designed for
 * vehicle suspensions.
 * 
 * @author Daniel Murphy
 */
class WheelJoint extends Joint {
  // TODO(srdjan): make fields private.
  double m_frequencyHz = 0.0;
  double m_dampingRatio = 0.0;

  // Solver shared
  final Vec2 m_localAnchorA = new Vec2.zero();
  final Vec2 m_localAnchorB = new Vec2.zero();
  final Vec2 m_localXAxisA = new Vec2.zero();
  final Vec2 m_localYAxisA = new Vec2.zero();

  double m_impulse = 0.0;
  double m_motorImpulse = 0.0;
  double m_springImpulse = 0.0;

  double m_maxMotorTorque = 0.0;
  double m_motorSpeed = 0.0;
  bool m_enableMotor = false;

  // Solver temp
  int m_indexA = 0;
  int m_indexB = 0;
  final Vec2 m_localCenterA = new Vec2.zero();
  final Vec2 m_localCenterB = new Vec2.zero();
  double m_invMassA = 0.0;
  double m_invMassB = 0.0;
  double m_invIA = 0.0;
  double m_invIB = 0.0;

  final Vec2 m_ax = new Vec2.zero();
  final Vec2 m_ay = new Vec2.zero();
  double m_sAx = 0.0,
      m_sBx = 0.0;
  double m_sAy = 0.0,
      m_sBy = 0.0;

  double m_mass = 0.0;
  double m_motorMass = 0.0;
  double m_springMass = 0.0;

  double m_bias = 0.0;
  double m_gamma = 0.0;

  WheelJoint(IWorldPool argPool, WheelJointDef def) : super(argPool, def) {
    m_localAnchorA.set(def.localAnchorA);
    m_localAnchorB.set(def.localAnchorB);
    m_localXAxisA.set(def.localAxisA);
    Vec2.crossToOutUnsafeDblVec2(1.0, m_localXAxisA, m_localYAxisA);

    m_motorMass = 0.0;
    m_motorImpulse = 0.0;

    m_maxMotorTorque = def.maxMotorTorque;
    m_motorSpeed = def.motorSpeed;
    m_enableMotor = def.enableMotor;

    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;
  }

  Vec2 getLocalAnchorA() {
    return m_localAnchorA;
  }

  Vec2 getLocalAnchorB() {
    return m_localAnchorB;
  }

  void getAnchorA(Vec2 argOut) {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  void getAnchorB(Vec2 argOut) {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vec2 argOut) {
    final Vec2 temp = pool.popVec2();
    temp.set(m_ay).mulLocal(m_impulse);
    argOut.set(m_ax).mulLocal(m_springImpulse).addLocal(temp).mulLocal(inv_dt);
    pool.pushVec2(1);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * m_motorImpulse;
  }

  double getJointTranslation() {
    Body b1 = m_bodyA;
    Body b2 = m_bodyB;

    Vec2 p1 = pool.popVec2();
    Vec2 p2 = pool.popVec2();
    Vec2 axis = pool.popVec2();
    b1.getWorldPointToOut(m_localAnchorA, p1);
    b2.getWorldPointToOut(m_localAnchorA, p2);
    p2.subLocal(p1);
    b1.getWorldVectorToOut(m_localXAxisA, axis);

    double translation = Vec2.dot(p2, axis);
    pool.pushVec2(3);
    return translation;
  }

  /** For serialization */
  Vec2 getLocalAxisA() {
    return m_localXAxisA;
  }

  double getJointSpeed() {
    return m_bodyA._angularVelocity - m_bodyB._angularVelocity;
  }

  bool isMotorEnabled() {
    return m_enableMotor;
  }

  void enableMotor(bool flag) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_enableMotor = flag;
  }

  void setMotorSpeed(double speed) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_motorSpeed = speed;
  }

  double getMotorSpeed() {
    return m_motorSpeed;
  }

  double getMaxMotorTorque() {
    return m_maxMotorTorque;
  }

  void setMaxMotorTorque(double torque) {
    m_bodyA.setAwake(true);
    m_bodyB.setAwake(true);
    m_maxMotorTorque = torque;
  }

  double getMotorTorque(double inv_dt) {
    return m_motorImpulse * inv_dt;
  }

  void setSpringFrequencyHz(double hz) {
    m_frequencyHz = hz;
  }

  double getSpringFrequencyHz() {
    return m_frequencyHz;
  }

  void setSpringDampingRatio(double ratio) {
    m_dampingRatio = ratio;
  }

  double getSpringDampingRatio() {
    return m_dampingRatio;
  }

  // pooling
  // TODO(srdjan): Make fields private.
  final Vec2 rA = new Vec2.zero();
  final Vec2 rB = new Vec2.zero();
  final Vec2 d = new Vec2.zero();

  void initVelocityConstraints(SolverData data) {
    m_indexA = m_bodyA.m_islandIndex;
    m_indexB = m_bodyB.m_islandIndex;
    m_localCenterA.set(m_bodyA.m_sweep.localCenter);
    m_localCenterB.set(m_bodyB.m_sweep.localCenter);
    m_invMassA = m_bodyA.m_invMass;
    m_invMassB = m_bodyB.m_invMass;
    m_invIA = m_bodyA.m_invI;
    m_invIB = m_bodyB.m_invI;

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vec2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;

    Vec2 cB = data.positions[m_indexB].c;
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
        qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(
        qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB);
    d.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);

    // Point to line constraint
    {
      Rot.mulToOut(qA, m_localYAxisA, m_ay);
      m_sAy = Vec2.cross(temp.set(d).addLocal(rA), m_ay);
      m_sBy = Vec2.cross(rB, m_ay);

      m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

      if (m_mass > 0.0) {
        m_mass = 1.0 / m_mass;
      }
    }

    // Spring constraint
    m_springMass = 0.0;
    m_bias = 0.0;
    m_gamma = 0.0;
    if (m_frequencyHz > 0.0) {
      Rot.mulToOut(qA, m_localXAxisA, m_ax);
      m_sAx = Vec2.cross(temp.set(d).addLocal(rA), m_ax);
      m_sBx = Vec2.cross(rB, m_ax);

      double invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

      if (invMass > 0.0) {
        m_springMass = 1.0 / invMass;

        double C = Vec2.dot(d, m_ax);

        // Frequency
        double omega = 2.0 * Math.PI * m_frequencyHz;

        // Damping coefficient
        double dd = 2.0 * m_springMass * m_dampingRatio * omega;

        // Spring stiffness
        double k = m_springMass * omega * omega;

        // magic formulas
        double h = data.step.dt;
        m_gamma = h * (dd + h * k);
        if (m_gamma > 0.0) {
          m_gamma = 1.0 / m_gamma;
        }

        m_bias = C * h * k * m_gamma;

        m_springMass = invMass + m_gamma;
        if (m_springMass > 0.0) {
          m_springMass = 1.0 / m_springMass;
        }
      }
    } else {
      m_springImpulse = 0.0;
    }

    // Rotational motor
    if (m_enableMotor) {
      m_motorMass = iA + iB;
      if (m_motorMass > 0.0) {
        m_motorMass = 1.0 / m_motorMass;
      }
    } else {
      m_motorMass = 0.0;
      m_motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      final Vec2 P = pool.popVec2();
      // Account for variable time step.
      m_impulse *= data.step.dtRatio;
      m_springImpulse *= data.step.dtRatio;
      m_motorImpulse *= data.step.dtRatio;

      P.x = m_impulse * m_ay.x + m_springImpulse * m_ax.x;
      P.y = m_impulse * m_ay.y + m_springImpulse * m_ax.y;
      double LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
      double LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

      vA.x -= m_invMassA * P.x;
      vA.y -= m_invMassA * P.y;
      wA -= m_invIA * LA;

      vB.x += m_invMassB * P.x;
      vB.y += m_invMassB * P.y;
      wB += m_invIB * LB;
      pool.pushVec2(1);
    } else {
      m_impulse = 0.0;
      m_springImpulse = 0.0;
      m_motorImpulse = 0.0;
    }
    pool.pushRot(2);
    pool.pushVec2(1);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  void solveVelocityConstraints(SolverData data) {
    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    Vec2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;
    Vec2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    final Vec2 temp = pool.popVec2();
    final Vec2 P = pool.popVec2();

    // Solve spring constraint
    {
      double Cdot =
          Vec2.dot(m_ax, temp.set(vB).subLocal(vA)) + m_sBx * wB - m_sAx * wA;
      double impulse =
          -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
      m_springImpulse += impulse;

      P.x = impulse * m_ax.x;
      P.y = impulse * m_ax.y;
      double LA = impulse * m_sAx;
      double LB = impulse * m_sBx;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }

    // Solve rotational motor constraint
    {
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

    // Solve point to line constraint
    {
      double Cdot =
          Vec2.dot(m_ay, temp.set(vB).subLocal(vA)) + m_sBy * wB - m_sAy * wA;
      double impulse = -m_mass * Cdot;
      m_impulse += impulse;

      P.x = impulse * m_ay.x;
      P.y = impulse * m_ay.y;
      double LA = impulse * m_sAy;
      double LB = impulse * m_sBy;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }
    pool.pushVec2(2);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  bool solvePositionConstraints(SolverData data) {
    Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vec2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    Rot.mulToOut(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOut(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB);
    d.set(cB).subLocal(cA).addLocal(rB).subLocal(rA);

    Vec2 ay = pool.popVec2();
    Rot.mulToOut(qA, m_localYAxisA, ay);

    double sAy = Vec2.cross(temp.set(d).addLocal(rA), ay);
    double sBy = Vec2.cross(rB, ay);

    double C = Vec2.dot(d, ay);

    double k = m_invMassA +
        m_invMassB +
        m_invIA * m_sAy * m_sAy +
        m_invIB * m_sBy * m_sBy;

    double impulse;
    if (k != 0.0) {
      impulse = -C / k;
    } else {
      impulse = 0.0;
    }

    final Vec2 P = pool.popVec2();
    P.x = impulse * ay.x;
    P.y = impulse * ay.y;
    double LA = impulse * sAy;
    double LB = impulse * sBy;

    cA.x -= m_invMassA * P.x;
    cA.y -= m_invMassA * P.y;
    aA -= m_invIA * LA;
    cB.x += m_invMassB * P.x;
    cB.y += m_invMassB * P.y;
    aB += m_invIB * LB;

    pool.pushVec2(3);
    pool.pushRot(2);
    // data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;

    return C.abs() <= Settings.linearSlop;
  }
}
