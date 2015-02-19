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
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Angle constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A motor joint is used to control the relative motion between two bodies. A typical usage is to
 * control the movement of a dynamic body with respect to the ground.
 * 
 * @author dmurph
 */
class MotorJoint extends Joint {

  // Solver shared
  final Vec2 _m_linearOffset = new Vec2.zero();
  double _m_angularOffset = 0.0;
  final Vec2 _m_linearImpulse = new Vec2.zero();
  double _m_angularImpulse = 0.0;
  double _m_maxForce = 0.0;
  double _m_maxTorque = 0.0;
  double _m_correctionFactor = 0.0;

  // Solver temp
  int _m_indexA = 0;
  int _m_indexB = 0;
  final Vec2 _m_rA = new Vec2.zero();
  final Vec2 _m_rB = new Vec2.zero();
  final Vec2 _m_localCenterA = new Vec2.zero();
  final Vec2 _m_localCenterB = new Vec2.zero();
  final Vec2 _m_linearError = new Vec2.zero();
  double _m_angularError = 0.0;
  double _m_invMassA = 0.0;
  double _m_invMassB = 0.0;
  double _m_invIA = 0.0;
  double _m_invIB = 0.0;
  final Mat22 _m_linearMass = new Mat22.zero();
  double _m_angularMass = 0.0;

  MotorJoint(IWorldPool pool, MotorJointDef def)
      : super(pool, def) {
    _m_linearOffset.set(def.linearOffset);
    _m_angularOffset = def.angularOffset;

    _m_angularImpulse = 0.0;

    _m_maxForce = def.maxForce;
    _m_maxTorque = def.maxTorque;
    _m_correctionFactor = def.correctionFactor;
  }

  void getAnchorA(Vec2 out) {
    out.set(m_bodyA.getPosition());
  }

  void getAnchorB(Vec2 out) {
    out.set(m_bodyB.getPosition());
  }

  void getReactionForce(double inv_dt, Vec2 out) {
    out.set(_m_linearImpulse).mulLocal(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return _m_angularImpulse * inv_dt;
  }

  double getCorrectionFactor() {
    return _m_correctionFactor;
  }

  void setCorrectionFactor(double correctionFactor) {
    this._m_correctionFactor = correctionFactor;
  }

  /**
   * Set the target linear offset, in frame A, in meters.
   */
  void setLinearOffset(Vec2 linearOffset) {
    if (linearOffset.x != _m_linearOffset.x || linearOffset.y != _m_linearOffset.y) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      _m_linearOffset.set(linearOffset);
    }
  }

  /**
   * Get the target linear offset, in frame A, in meters.
   */
  void getLinearOffsetOut(Vec2 out) {
    out.set(_m_linearOffset);
  }

  /**
   * Get the target linear offset, in frame A, in meters. Do not modify.
   */
  Vec2 getLinearOffset() {
    return _m_linearOffset;
  }

  /**
   * Set the target angular offset, in radians.
   * 
   * @param angularOffset
   */
  void setAngularOffset(double angularOffset) {
    if (angularOffset != _m_angularOffset) {
      m_bodyA.setAwake(true);
      m_bodyB.setAwake(true);
      _m_angularOffset = angularOffset;
    }
  }

  double getAngularOffset() {
    return _m_angularOffset;
  }

  /**
   * Set the maximum friction force in N.
   * 
   * @param force
   */
  void setMaxForce(double force) {
    assert(force >= 0.0);
    _m_maxForce = force;
  }

  /**
   * Get the maximum friction force in N.
   */
  double getMaxForce() {
    return _m_maxForce;
  }

  /**
   * Set the maximum friction torque in N*m.
   */
  void setMaxTorque(double torque) {
    assert(torque >= 0.0);
    _m_maxTorque = torque;
  }

  /**
   * Get the maximum friction torque in N*m.
   */
  double getMaxTorque() {
    return _m_maxTorque;
  }

  void initVelocityConstraints(SolverData data) {
    _m_indexA = m_bodyA.m_islandIndex;
    _m_indexB = m_bodyB.m_islandIndex;
    _m_localCenterA.set(m_bodyA.m_sweep.localCenter);
    _m_localCenterB.set(m_bodyB.m_sweep.localCenter);
    _m_invMassA = m_bodyA.m_invMass;
    _m_invMassB = m_bodyB.m_invMass;
    _m_invIA = m_bodyA.m_invI;
    _m_invIB = m_bodyB.m_invI;

    final Vec2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    final Vec2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;

    final Vec2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;
    final Vec2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 temp = pool.popVec2();
    Mat22 K = pool.popMat22();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective mass matrix.
    // m_rA = b2Mul(qA, -m_localCenterA);
    // m_rB = b2Mul(qB, -m_localCenterB);
    _m_rA.x = qA.c * -_m_localCenterA.x - qA.s * -_m_localCenterA.y;
    _m_rA.y = qA.s * -_m_localCenterA.x + qA.c * -_m_localCenterA.y;
    _m_rB.x = qB.c * -_m_localCenterB.x - qB.s * -_m_localCenterB.y;
    _m_rB.y = qB.s * -_m_localCenterB.x + qB.c * -_m_localCenterB.y;

    // J = [-I -r1_skew I r2_skew]
    // [ 0 -1 0 1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
    // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
    // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
    double mA = _m_invMassA,
        mB = _m_invMassB;
    double iA = _m_invIA,
        iB = _m_invIB;

    K.ex.x = mA + mB + iA * _m_rA.y * _m_rA.y + iB * _m_rB.y * _m_rB.y;
    K.ex.y = -iA * _m_rA.x * _m_rA.y - iB * _m_rB.x * _m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = mA + mB + iA * _m_rA.x * _m_rA.x + iB * _m_rB.x * _m_rB.x;

    K.invertToOut(_m_linearMass);

    _m_angularMass = iA + iB;
    if (_m_angularMass > 0.0) {
      _m_angularMass = 1.0 / _m_angularMass;
    }

    // m_linearError = cB + m_rB - cA - m_rA - b2Mul(qA, m_linearOffset);
    Rot.mulToOutUnsafe(qA, _m_linearOffset, temp);
    _m_linearError.x = cB.x + _m_rB.x - cA.x - _m_rA.x - temp.x;
    _m_linearError.y = cB.y + _m_rB.y - cA.y - _m_rA.y - temp.y;
    _m_angularError = aB - aA - _m_angularOffset;

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      _m_linearImpulse.x *= data.step.dtRatio;
      _m_linearImpulse.y *= data.step.dtRatio;
      _m_angularImpulse *= data.step.dtRatio;

      final Vec2 P = _m_linearImpulse;
      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (_m_rA.x * P.y - _m_rA.y * P.x + _m_angularImpulse);
      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (_m_rB.x * P.y - _m_rB.y * P.x + _m_angularImpulse);
    } else {
      _m_linearImpulse.setZero();
      _m_angularImpulse = 0.0;
    }

    pool.pushVec2(1);
    pool.pushMat22(1);
    pool.pushRot(2);

    // data.velocities[m_indexA].v = vA;
    data.velocities[_m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[_m_indexB].w = wB;
  }

  void solveVelocityConstraints(SolverData data) {
    final Vec2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;
    final Vec2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    double mA = _m_invMassA,
        mB = _m_invMassB;
    double iA = _m_invIA,
        iB = _m_invIB;

    double h = data.step.dt;
    double inv_h = data.step.inv_dt;

    final Vec2 temp = pool.popVec2();

    // Solve angular friction
    {
      double Cdot = wB - wA + inv_h * _m_correctionFactor * _m_angularError;
      double impulse = -_m_angularMass * Cdot;

      double oldImpulse = _m_angularImpulse;
      double maxImpulse = h * _m_maxTorque;
      _m_angularImpulse = MathUtils.clampDouble(_m_angularImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = _m_angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    final Vec2 Cdot = pool.popVec2();

    // Solve linear friction
    {
      // Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA) + inv_h * m_correctionFactor *
      // m_linearError;
      Cdot.x = vB.x + -wB * _m_rB.y - vA.x - -wA * _m_rA.y + inv_h * _m_correctionFactor * _m_linearError.x;
      Cdot.y = vB.y + wB * _m_rB.x - vA.y - wA * _m_rA.x + inv_h * _m_correctionFactor * _m_linearError.y;

      final Vec2 impulse = temp;
      Mat22.mulToOutUnsafeVec2_(_m_linearMass, Cdot, impulse);
      impulse.negateLocal();
      final Vec2 oldImpulse = pool.popVec2();
      oldImpulse.set(_m_linearImpulse);
      _m_linearImpulse.addLocal(impulse);

      double maxImpulse = h * _m_maxForce;

      if (_m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
        _m_linearImpulse.normalize();
        _m_linearImpulse.mulLocal(maxImpulse);
      }

      impulse.x = _m_linearImpulse.x - oldImpulse.x;
      impulse.y = _m_linearImpulse.y - oldImpulse.y;

      vA.x -= mA * impulse.x;
      vA.y -= mA * impulse.y;
      wA -= iA * (_m_rA.x * impulse.y - _m_rA.y * impulse.x);

      vB.x += mB * impulse.x;
      vB.y += mB * impulse.y;
      wB += iB * (_m_rB.x * impulse.y - _m_rB.y * impulse.x);
    }

    pool.pushVec2(3);

    // data.velocities[m_indexA].v.set(vA);
    data.velocities[_m_indexA].w = wA;
    // data.velocities[m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;
  }

  bool solvePositionConstraints(SolverData data) {
    return true;
  }
}
