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

//Angle constraint
//C = angle2 - angle1 - referenceAngle
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2


/**
 * A weld joint essentially glues two bodies together. A weld joint may distort somewhat because the
 * island constraint solver is approximate.
 * 
 * @author Daniel Murphy
 */
class WeldJoint extends Joint {
  // TODO(srdjan): Make fields private.
  double m_frequencyHz = 0.0;
  double m_dampingRatio = 0.0;
  double m_bias = 0.0;

  // Solver shared
  final Vec2 m_localAnchorA;
  final Vec2 m_localAnchorB;
  double m_referenceAngle = 0.0;
  double m_gamma = 0.0;
  final Vec3 m_impulse;


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
  final Mat33 m_mass = new Mat33.zero();

  WeldJoint(IWorldPool argWorld, WeldJointDef def)
      : super(argWorld, def),
        m_localAnchorA = new Vec2.copy(def.localAnchorA),
        m_localAnchorB = new Vec2.copy(def.localAnchorB),
        m_impulse = new Vec3.zero() {
    m_referenceAngle = def.referenceAngle;
    m_frequencyHz = def.frequencyHz;
    m_dampingRatio = def.dampingRatio;
  }

  double getReferenceAngle() {
    return m_referenceAngle;
  }

  Vec2 getLocalAnchorA() {
    return m_localAnchorA;
  }

  Vec2 getLocalAnchorB() {
    return m_localAnchorB;
  }

  double getFrequency() {
    return m_frequencyHz;
  }

  void setFrequency(double frequencyHz) {
    this.m_frequencyHz = frequencyHz;
  }

  double getDampingRatio() {
    return m_dampingRatio;
  }

  void setDampingRatio(double dampingRatio) {
    this.m_dampingRatio = dampingRatio;
  }


  void getAnchorA(Vec2 argOut) {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }


  void getAnchorB(Vec2 argOut) {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }


  void getReactionForce(double inv_dt, Vec2 argOut) {
    argOut.setXY(m_impulse.x, m_impulse.y);
    argOut.mulLocal(inv_dt);
  }


  double getReactionTorque(double inv_dt) {
    return inv_dt * m_impulse.z;
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
    Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), m_rA);
    Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), m_rB);

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

    final Mat33 K = pool.popMat33();

    K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
    K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
    K.ez.x = -m_rA.y * iA - m_rB.y * iB;
    K.ex.y = K.ey.x;
    K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
    K.ez.y = m_rA.x * iA + m_rB.x * iB;
    K.ex.z = K.ez.x;
    K.ey.z = K.ez.y;
    K.ez.z = iA + iB;

    if (m_frequencyHz > 0.0) {
      K.getInverse22(m_mass);

      double invM = iA + iB;
      double m = invM > 0.0 ? 1.0 / invM : 0.0;

      double C = aB - aA - m_referenceAngle;

      // Frequency
      double omega = 2.0 * Math.PI * m_frequencyHz;

      // Damping coefficient
      double d = 2.0 * m * m_dampingRatio * omega;

      // Spring stiffness
      double k = m * omega * omega;

      // magic formulas
      double h = data.step.dt;
      m_gamma = h * (d + h * k);
      m_gamma = m_gamma != 0.0 ? 1.0 / m_gamma : 0.0;
      m_bias = C * h * k * m_gamma;

      invM += m_gamma;
      m_mass.ez.z = invM != 0.0 ? 1.0 / invM : 0.0;
    } else {
      K.getSymInverse33(m_mass);
      m_gamma = 0.0;
      m_bias = 0.0;
    }

    if (data.step.warmStarting) {
      final Vec2 P = pool.popVec2();
      // Scale impulses to support a variable time step.
      m_impulse.mulLocal(data.step.dtRatio);

      P.setXY(m_impulse.x, m_impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.cross(m_rA, P) + m_impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.cross(m_rB, P) + m_impulse.z);
      pool.pushVec2(1);
    } else {
      m_impulse.setZero();
    }

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushRot(2);
    pool.pushMat33(1);
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

    final Vec2 Cdot1 = pool.popVec2();
    final Vec2 P = pool.popVec2();
    final Vec2 temp = pool.popVec2();
    if (m_frequencyHz > 0.0) {
      double Cdot2 = wB - wA;

      double impulse2 = -m_mass.ez.z * (Cdot2 + m_bias + m_gamma * m_impulse.z);
      m_impulse.z += impulse2;

      wA -= iA * impulse2;
      wB += iB * impulse2;

      Vec2.crossToOutUnsafeDblVec2(wB, m_rB, Cdot1);
      Vec2.crossToOutUnsafeDblVec2(wA, m_rA, temp);
      Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);

      final Vec2 impulse1 = P;
      Mat33.mul22ToOutUnsafe(m_mass, Cdot1, impulse1);
      impulse1.negateLocal();

      m_impulse.x += impulse1.x;
      m_impulse.y += impulse1.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * Vec2.cross(m_rA, P);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * Vec2.cross(m_rB, P);
    } else {
      Vec2.crossToOutUnsafeDblVec2(wA, m_rA, temp);
      Vec2.crossToOutUnsafeDblVec2(wB, m_rB, Cdot1);
      Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);
      double Cdot2 = wB - wA;

      final Vec3 Cdot = pool.popVec3();
      Cdot.setXYZ(Cdot1.x, Cdot1.y, Cdot2);

      final Vec3 impulse = pool.popVec3();
      Mat33.mulToOutUnsafe(m_mass, Cdot, impulse);
      impulse.negateLocal();
      m_impulse.addLocal(impulse);

      P.setXY(impulse.x, impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (Vec2.cross(m_rA, P) + impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (Vec2.cross(m_rB, P) + impulse.z);

      pool.pushVec3(2);
    }

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(3);
  }

  bool solvePositionConstraints(final SolverData data) {
    Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vec2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 temp = pool.popVec2();
    final Vec2 rA = pool.popVec2();
    final Vec2 rB = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    double mA = m_invMassA,
        mB = m_invMassB;
    double iA = m_invIA,
        iB = m_invIB;

    Rot.mulToOutUnsafe(qA, temp.set(m_localAnchorA).subLocal(m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.set(m_localAnchorB).subLocal(m_localCenterB), rB);
    double positionError, angularError;

    final Mat33 K = pool.popMat33();
    final Vec2 C1 = pool.popVec2();
    final Vec2 P = pool.popVec2();

    K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
    K.ez.x = -rA.y * iA - rB.y * iB;
    K.ex.y = K.ey.x;
    K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
    K.ez.y = rA.x * iA + rB.x * iB;
    K.ex.z = K.ez.x;
    K.ey.z = K.ez.y;
    K.ez.z = iA + iB;
    if (m_frequencyHz > 0.0) {
      C1.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);

      positionError = C1.length();
      angularError = 0.0;

      K.solve22ToOut(C1, P);
      P.negateLocal();

      cA.x -= mA * P.x;
      cA.y -= mA * P.y;
      aA -= iA * Vec2.cross(rA, P);

      cB.x += mB * P.x;
      cB.y += mB * P.y;
      aB += iB * Vec2.cross(rB, P);
    } else {
      C1.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);
      double C2 = aB - aA - m_referenceAngle;

      positionError = C1.length();
      angularError = C2.abs();

      final Vec3 C = pool.popVec3();
      final Vec3 impulse = pool.popVec3();
      C.setXYZ(C1.x, C1.y, C2);

      K.solve33ToOut(C, impulse);
      impulse.negateLocal();
      P.setXY(impulse.x, impulse.y);

      cA.x -= mA * P.x;
      cA.y -= mA * P.y;
      aA -= iA * (Vec2.cross(rA, P) + impulse.z);

      cB.x += mB * P.x;
      cB.y += mB * P.y;
      aB += iB * (Vec2.cross(rB, P) + impulse.z);
      pool.pushVec3(2);
    }

//    data.positions[m_indexA].c.set(cA);
    data.positions[m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
    data.positions[m_indexB].a = aB;

    pool.pushVec2(5);
    pool.pushRot(2);
    pool.pushMat33(1);

    return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
  }
}
