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
 * A mouse joint is used to make a point on a body track a specified world point. This a soft
 * constraint with a maximum force. This allows the constraint to stretch and without applying huge
 * forces. NOTE: this joint is not documented in the manual because it was developed to be used in
 * the testbed. If you want to learn how to use the mouse joint, look at the testbed.
 *
 * @author Daniel
 */
class MouseJoint extends Joint {
  final Vector2 _m_localAnchorB = new Vector2.zero();
  final Vector2 _m_targetA = new Vector2.zero();
  double _m_frequencyHz = 0.0;
  double _m_dampingRatio = 0.0;
  double _m_beta = 0.0;

  // Solver shared
  final Vector2 _m_impulse = new Vector2.zero();
  double _m_maxForce = 0.0;
  double _m_gamma = 0.0;

  // Solver temp
  int _m_indexB = 0;
  final Vector2 _m_rB = new Vector2.zero();
  final Vector2 _m_localCenterB = new Vector2.zero();
  double _m_invMassB = 0.0;
  double _m_invIB = 0.0;
  final Mat22 _m_mass = new Mat22.zero();
  final Vector2 _m_C = new Vector2.zero();

  MouseJoint(IWorldPool argWorld, MouseJointDef def) : super(argWorld, def) {
    assert(def.target.isValid());
    assert(def.maxForce >= 0);
    assert(def.frequencyHz >= 0);
    assert(def.dampingRatio >= 0);

    _m_targetA.set(def.target);
    Transform.mulTransToOutUnsafeVec2(
        m_bodyB.getTransform(), _m_targetA, _m_localAnchorB);

    _m_maxForce = def.maxForce;
    _m_impulse.setZero();

    _m_frequencyHz = def.frequencyHz;
    _m_dampingRatio = def.dampingRatio;
  }

  void getAnchorA(Vector2 argOut) {
    argOut.set(_m_targetA);
  }

  void getAnchorB(Vector2 argOut) {
    m_bodyB.getWorldPointToOut(_m_localAnchorB, argOut);
  }

  void getReactionForce(double invDt, Vector2 argOut) {
    argOut.set(_m_impulse).mul(invDt);
  }

  double getReactionTorque(double invDt) {
    return invDt * 0.0;
  }

  void setTarget(Vector2 target) {
    if (m_bodyB.isAwake() == false) {
      m_bodyB.setAwake(true);
    }
    _m_targetA.set(target);
  }

  Vector2 getTarget() {
    return _m_targetA;
  }

  // / set/get the maximum force in Newtons.
  void setMaxForce(double force) {
    _m_maxForce = force;
  }

  double getMaxForce() {
    return _m_maxForce;
  }

  // / set/get the frequency in Hertz.
  void setFrequency(double hz) {
    _m_frequencyHz = hz;
  }

  double getFrequency() {
    return _m_frequencyHz;
  }

  // / set/get the damping ratio (dimensionless).
  void setDampingRatio(double ratio) {
    _m_dampingRatio = ratio;
  }

  double getDampingRatio() {
    return _m_dampingRatio;
  }

  void initVelocityConstraints(final SolverData data) {
    _m_indexB = m_bodyB.m_islandIndex;
    _m_localCenterB.set(m_bodyB.m_sweep.localCenter);
    _m_invMassB = m_bodyB.m_invMass;
    _m_invIB = m_bodyB.m_invI;

    Vector2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;
    Vector2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Rot qB = pool.popRot();

    qB.setAngle(aB);

    double mass = m_bodyB.mass;

    // Frequency
    double omega = 2.0 * Math.PI * _m_frequencyHz;

    // Damping coefficient
    double d = 2.0 * mass * _m_dampingRatio * omega;

    // Spring stiffness
    double k = mass * (omega * omega);

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    double h = data.step.dt;
    assert(d + h * k > Settings.EPSILON);
    _m_gamma = h * (d + h * k);
    if (_m_gamma != 0.0) {
      _m_gamma = 1.0 / _m_gamma;
    }
    _m_beta = h * k * _m_gamma;

    Vector2 temp = pool.popVec2();

    // Compute the effective mass matrix.
    Rot.mulToOutUnsafe(
        qB, temp.set(_m_localAnchorB).sub(_m_localCenterB), _m_rB);

    // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
    final Mat22 K = pool.popMat22();
    K.ex.x = _m_invMassB + _m_invIB * _m_rB.y * _m_rB.y + _m_gamma;
    K.ex.y = -_m_invIB * _m_rB.x * _m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = _m_invMassB + _m_invIB * _m_rB.x * _m_rB.x + _m_gamma;

    K.invertToOut(_m_mass);

    _m_C.set(cB).add(_m_rB).sub(_m_targetA);
    _m_C.mul(_m_beta);

    // Cheat with some damping
    wB *= 0.98;

    if (data.step.warmStarting) {
      _m_impulse.mul(data.step.dtRatio);
      vB.x += _m_invMassB * _m_impulse.x;
      vB.y += _m_invMassB * _m_impulse.y;
      wB += _m_invIB * Vector2.cross(_m_rB, _m_impulse);
    } else {
      _m_impulse.setZero();
    }

//    data.velocities[m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushMat22(1);
    pool.pushRot(1);
  }

  bool solvePositionConstraints(final SolverData data) {
    return true;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    // Cdot = v + cross(w, r)
    final Vector2 Cdot = pool.popVec2();
    Vector2.crossToOutUnsafeDblVec2(wB, _m_rB, Cdot);
    Cdot.add(vB);

    final Vector2 impulse = pool.popVec2();
    final Vector2 temp = pool.popVec2();

    temp
        .set(_m_impulse)
        .mul(_m_gamma)
        .add(_m_C)
        .add(Cdot)
        .negate();
    Mat22.mulToOutUnsafeVec2_(_m_mass, temp, impulse);

    Vector2 oldImpulse = temp;
    oldImpulse.set(_m_impulse);
    _m_impulse.add(impulse);
    double maxImpulse = data.step.dt * _m_maxForce;
    if (_m_impulse.lengthSquared() > maxImpulse * maxImpulse) {
      _m_impulse.mul(maxImpulse / _m_impulse.length);
    }
    impulse.set(_m_impulse).sub(oldImpulse);

    vB.x += _m_invMassB * impulse.x;
    vB.y += _m_invMassB * impulse.y;
    wB += _m_invIB * Vector2.cross(_m_rB, impulse);

//    data.velocities[m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;

    pool.pushVec2(3);
  }
}
