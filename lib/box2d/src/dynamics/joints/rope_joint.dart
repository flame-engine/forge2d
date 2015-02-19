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

/**
 * A rope joint enforces a maximum distance between two points on two bodies. It has no other
 * effect. Warning: if you attempt to change the maximum length during the simulation you will get
 * some non-physical behavior. A model that would allow you to dynamically modify the length would
 * have some sponginess, so I chose not to implement it that way. See DistanceJoint if you want to
 * dynamically control length.
 * 
 * @author Daniel Murphy
 */
class RopeJoint extends Joint {
  // Solver shared
  // TODO(srdjan): make fields private.
  final Vec2 m_localAnchorA = new Vec2.zero();
  final Vec2 m_localAnchorB = new Vec2.zero();
  double m_maxLength = 0.0;
  double m_length = 0.0;
  double m_impulse = 0.0;

  // Solver temp
  int m_indexA = 0;
  int m_indexB = 0;
  final Vec2 m_u = new Vec2.zero();
  final Vec2 m_rA = new Vec2.zero();
  final Vec2 m_rB = new Vec2.zero();
  final Vec2 m_localCenterA = new Vec2.zero();
  final Vec2 m_localCenterB = new Vec2.zero();
  double m_invMassA = 0.0;
  double m_invMassB = 0.0;
  double m_invIA = 0.0;
  double m_invIB = 0.0;
  double m_mass = 0.0;
  LimitState m_state = LimitState.INACTIVE;

  RopeJoint(IWorldPool worldPool, RopeJointDef def)
      : super(worldPool, def) {
    m_localAnchorA.set(def.localAnchorA);
    m_localAnchorB.set(def.localAnchorB);

    m_maxLength = def.maxLength;
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
    Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), m_rA);
    Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), m_rB);

    m_u.set(cB).addLocal(m_rB).subLocal(cA).subLocal(m_rA);

    m_length = m_u.length();

    double C = m_length - m_maxLength;
    if (C > 0.0) {
      m_state = LimitState.AT_UPPER;
    } else {
      m_state = LimitState.INACTIVE;
    }

    if (m_length > Settings.linearSlop) {
      m_u.mulLocal(1.0 / m_length);
    } else {
      m_u.setZero();
      m_mass = 0.0;
      m_impulse = 0.0;
      return;
    }

    // Compute effective mass.
    double crA = Vec2.cross(m_rA, m_u);
    double crB = Vec2.cross(m_rB, m_u);
    double invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

    m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      m_impulse *= data.step.dtRatio;

      double Px = m_impulse * m_u.x;
      double Py = m_impulse * m_u.y;
      vA.x -= m_invMassA * Px;
      vA.y -= m_invMassA * Py;
      wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);

      vB.x += m_invMassB * Px;
      vB.y += m_invMassB * Py;
      wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);
    } else {
      m_impulse = 0.0;
    }

    pool.pushRot(2);
    pool.pushVec2(1);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vec2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;
    Vec2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    // Cdot = dot(u, v + cross(w, r))
    Vec2 vpA = pool.popVec2();
    Vec2 vpB = pool.popVec2();
    Vec2 temp = pool.popVec2();

    Vec2.crossToOutUnsafeDblVec2(wA, m_rA, vpA);
    vpA.addLocal(vA);
    Vec2.crossToOutUnsafeDblVec2(wB, m_rB, vpB);
    vpB.addLocal(vB);

    double C = m_length - m_maxLength;
    double Cdot = Vec2.dot(m_u, temp.set(vpB).subLocal(vpA));

    // Predictive constraint.
    if (C < 0.0) {
      Cdot += data.step.inv_dt * C;
    }

    double impulse = -m_mass * Cdot;
    double oldImpulse = m_impulse;
    m_impulse = Math.min(0.0, m_impulse + impulse);
    impulse = m_impulse - oldImpulse;

    double Px = impulse * m_u.x;
    double Py = impulse * m_u.y;
    vA.x -= m_invMassA * Px;
    vA.y -= m_invMassA * Py;
    wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px);
    vB.x += m_invMassB * Px;
    vB.y += m_invMassB * Py;
    wB += m_invIB * (m_rB.x * Py - m_rB.y * Px);

    pool.pushVec2(3);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vec2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 u = pool.popVec2();
    final Vec2 rA = pool.popVec2();
    final Vec2 rB = pool.popVec2();
    final Vec2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB);
    u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);

    double length = u.normalize();
    double C = length - m_maxLength;

    C = MathUtils.clampDouble(C, 0.0, Settings.maxLinearCorrection);

    double impulse = -m_mass * C;
    double Px = impulse * u.x;
    double Py = impulse * u.y;

    cA.x -= m_invMassA * Px;
    cA.y -= m_invMassA * Py;
    aA -= m_invIA * (rA.x * Py - rA.y * Px);
    cB.x += m_invMassB * Px;
    cB.y += m_invMassB * Py;
    aB += m_invIB * (rB.x * Py - rB.y * Px);

    pool.pushRot(2);
    pool.pushVec2(4);

    // data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;

    return length - m_maxLength < Settings.linearSlop;
  }

  void getAnchorA(Vec2 argOut) {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }


  void getAnchorB(Vec2 argOut) {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vec2 argOut) {
    argOut.set(m_u).mulLocal(inv_dt).mulLocal(m_impulse);
  }


  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  Vec2 getLocalAnchorA() {
    return m_localAnchorA;
  }

  Vec2 getLocalAnchorB() {
    return m_localAnchorB;
  }

  double getMaxLength() {
    return m_maxLength;
  }

  void setMaxLength(double maxLength) {
    this.m_maxLength = maxLength;
  }

  LimitState getLimitState() {
    return m_state;
  }
}
