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

class FrictionJoint extends Joint {

  // TODO(srdjan): Make all m_xxx fields private.
  final Vector2 m_localAnchorA;
  final Vector2 m_localAnchorB;

  // Solver shared
  final Vector2 m_linearImpulse;
  double m_angularImpulse = 0.0;
  double m_maxForce = 0.0;
  double m_maxTorque = 0.0;

  // Solver temp
  int m_indexA = 0;
  int m_indexB = 0;
  final Vector2 m_rA = new Vector2.zero();
  final Vector2 m_rB = new Vector2.zero();
  final Vector2 m_localCenterA = new Vector2.zero();
  final Vector2 m_localCenterB = new Vector2.zero();
  double m_invMassA = 0.0;
  double m_invMassB = 0.0;
  double m_invIA = 0.0;
  double m_invIB = 0.0;
  final Matrix2 m_linearMass = new Matrix2.zero();
  double m_angularMass = 0.0;

  FrictionJoint(IWorldPool argWorldPool, FrictionJointDef def)
      : m_localAnchorA = new Vector2.copy(def.localAnchorA),
        m_localAnchorB = new Vector2.copy(def.localAnchorB),
        m_linearImpulse = new Vector2.zero(),
        super(argWorldPool, def) {
    m_maxForce = def.maxForce;
    m_maxTorque = def.maxTorque;
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
    argOut.setFrom(m_linearImpulse).scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * m_angularImpulse;
  }

  void setMaxForce(double force) {
    assert(force >= 0.0);
    m_maxForce = force;
  }

  double getMaxForce() {
    return m_maxForce;
  }

  void setMaxTorque(double torque) {
    assert(torque >= 0.0);
    m_maxTorque = torque;
  }

  double getMaxTorque() {
    return m_maxTorque;
  }

  /**
   * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
   */

  void initVelocityConstraints(final SolverData data) {
    m_indexA = m_bodyA.islandIndex;
    m_indexB = m_bodyB.islandIndex;
    m_localCenterA.setFrom(m_bodyA.sweep.localCenter);
    m_localCenterB.setFrom(m_bodyB.sweep.localCenter);
    m_invMassA = m_bodyA.invMass;
    m_invMassB = m_bodyB.invMass;
    m_invIA = m_bodyA.invI;
    m_invIB = m_bodyB.invI;

    double aA = data.positions[m_indexA].a;
    Vector2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;

    double aB = data.positions[m_indexB].a;
    Vector2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    final Vector2 temp = pool.popVec2();
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective mass matrix.
    Rot.mulToOutUnsafe(
        qA, temp.setFrom(m_localAnchorA).sub(m_localCenterA), m_rA);
    Rot.mulToOutUnsafe(
        qB, temp.setFrom(m_localAnchorB).sub(m_localCenterB), m_rB);

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

    final Matrix2 K = pool.popMat22();
    double a11 = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
    double a21 = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
    double a12 = a21;
    double a22 = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

    K.setValues(a11, a12, a21, a22);
    m_linearMass.setFrom(K);
    m_linearMass.invert();

    m_angularMass = iA + iB;
    if (m_angularMass > 0.0) {
      m_angularMass = 1.0 / m_angularMass;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      m_linearImpulse.scale(data.step.dtRatio);
      m_angularImpulse *= data.step.dtRatio;

      final Vector2 P = pool.popVec2();
      P.setFrom(m_linearImpulse);

      temp.setFrom(P).scale(mA);
      vA.sub(temp);
      wA -= iA * (m_rA.cross(P) + m_angularImpulse);

      temp.setFrom(P).scale(mB);
      vB.add(temp);
      wB += iB * (m_rB.cross(P) + m_angularImpulse);

      pool.pushVec2(1);
    } else {
      m_linearImpulse.setZero();
      m_angularImpulse = 0.0;
    }
//    data.velocities[m_indexA].v.set(vA);
    if (data.velocities[m_indexA].w != wA) {
      assert(data.velocities[m_indexA].w != wA);
    }
    data.velocities[m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushRot(2);
    pool.pushVec2(1);
    pool.pushMat22(1);
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

    double h = data.step.dt;

    // Solve angular friction
    {
      double Cdot = wB - wA;
      double impulse = -m_angularMass * Cdot;

      double oldImpulse = m_angularImpulse;
      double maxImpulse = h * m_maxTorque;
      m_angularImpulse = MathUtils.clampDouble(
          m_angularImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = m_angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve linear friction
    {
      final Vector2 Cdot = pool.popVec2();
      final Vector2 temp = pool.popVec2();

      m_rA.scaleOrthogonalInto(wA, temp);
      m_rB.scaleOrthogonalInto(wB, Cdot);
      Cdot.add(vB).sub(vA).sub(temp);

      final Vector2 impulse = pool.popVec2();
      m_linearMass.transformed(Cdot, impulse);
      impulse.negate();

      final Vector2 oldImpulse = pool.popVec2();
      oldImpulse.setFrom(m_linearImpulse);
      m_linearImpulse.add(impulse);

      double maxImpulse = h * m_maxForce;

      if (m_linearImpulse.length2 > maxImpulse * maxImpulse) {
        m_linearImpulse.normalize();
        m_linearImpulse.scale(maxImpulse);
      }

      impulse.setFrom(m_linearImpulse).sub(oldImpulse);

      temp.setFrom(impulse).scale(mA);
      vA.sub(temp);
      wA -= iA * m_rA.cross(impulse);

      temp.setFrom(impulse).scale(mB);
      vB.add(temp);
      wB += iB * m_rB.cross(impulse);
    }

//    data.velocities[m_indexA].v.set(vA);
    if (data.velocities[m_indexA].w != wA) {
      assert(data.velocities[m_indexA].w != wA);
    }
    data.velocities[m_indexA].w = wA;

//    data.velocities[m_indexB].v.set(vB);
    data.velocities[m_indexB].w = wB;

    pool.pushVec2(4);
  }

  bool solvePositionConstraints(final SolverData data) {
    return true;
  }
}
