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

//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/**
 * A distance joint constrains two points on two bodies to remain at a fixed distance from each
 * other. You can view this as a massless, rigid rod.
 */

class DistanceJoint extends Joint {
  double _m_frequencyHz = 0.0;
  double _m_dampingRatio = 0.0;
  double _m_bias = 0.0;

  // Solver shared
  final Vector2 _m_localAnchorA;
  final Vector2 _m_localAnchorB;
  double _m_gamma = 0.0;
  double _m_impulse = 0.0;
  double _m_length = 0.0;

  // Solver temp
  int _m_indexA = 0;
  int _m_indexB = 0;
  final Vector2 _m_u = new Vector2.zero();
  final Vector2 _m_rA = new Vector2.zero();
  final Vector2 _m_rB = new Vector2.zero();
  final Vector2 _m_localCenterA = new Vector2.zero();
  final Vector2 _m_localCenterB = new Vector2.zero();
  double _m_invMassA = 0.0;
  double _m_invMassB = 0.0;
  double _m_invIA = 0.0;
  double _m_invIB = 0.0;
  double _m_mass = 0.0;

  DistanceJoint(IWorldPool argWorld, final DistanceJointDef def)
      : _m_localAnchorA = def.localAnchorA.clone(),
        _m_localAnchorB = def.localAnchorB.clone(),
        super(argWorld, def) {
    _m_length = def.length;
    _m_frequencyHz = def.frequencyHz;
    _m_dampingRatio = def.dampingRatio;
  }

  void setFrequency(double hz) {
    _m_frequencyHz = hz;
  }

  double getFrequency() {
    return _m_frequencyHz;
  }

  double getLength() {
    return _m_length;
  }

  void setLength(double argLength) {
    _m_length = argLength;
  }

  void setDampingRatio(double damp) {
    _m_dampingRatio = damp;
  }

  double getDampingRatio() {
    return _m_dampingRatio;
  }

  void getAnchorA(Vector2 argOut) {
    m_bodyA.getWorldPointToOut(_m_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    m_bodyB.getWorldPointToOut(_m_localAnchorB, argOut);
  }

  Vector2 getLocalAnchorA() {
    return _m_localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return _m_localAnchorB;
  }

  /**
   * Get the reaction force given the inverse time step. Unit is N.
   */

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut.x = _m_impulse * _m_u.x * inv_dt;
    argOut.y = _m_impulse * _m_u.y * inv_dt;
  }

  /**
   * Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a
   * distance joint.
   */

  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  void initVelocityConstraints(final SolverData data) {
    _m_indexA = m_bodyA.m_islandIndex;
    _m_indexB = m_bodyB.m_islandIndex;
    _m_localCenterA.set(m_bodyA.m_sweep.localCenter);
    _m_localCenterB.set(m_bodyB.m_sweep.localCenter);
    _m_invMassA = m_bodyA.m_invMass;
    _m_invMassB = m_bodyB.m_invMass;
    _m_invIA = m_bodyA.m_invI;
    _m_invIB = m_bodyB.m_invI;

    Vector2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    Vector2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;

    Vector2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;
    Vector2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // use _m_u as temporary variable
    Rot.mulToOutUnsafe(
        qA, _m_u.set(_m_localAnchorA).sub(_m_localCenterA), _m_rA);
    Rot.mulToOutUnsafe(
        qB, _m_u.set(_m_localAnchorB).sub(_m_localCenterB), _m_rB);
    _m_u.set(cB).add(_m_rB).sub(cA).sub(_m_rA);

    pool.pushRot(2);

    // Handle singularity.
    double length = _m_u.length;
    if (length > Settings.linearSlop) {
      _m_u.x *= 1.0 / length;
      _m_u.y *= 1.0 / length;
    } else {
      _m_u.setValues(0.0, 0.0);
    }

    double crAu = Vector2.cross(_m_rA, _m_u);
    double crBu = Vector2.cross(_m_rB, _m_u);
    double invMass = _m_invMassA +
        _m_invIA * crAu * crAu +
        _m_invMassB +
        _m_invIB * crBu * crBu;

    // Compute the effective mass matrix.
    _m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (_m_frequencyHz > 0.0) {
      double C = length - _m_length;

      // Frequency
      double omega = 2.0 * Math.PI * _m_frequencyHz;

      // Damping coefficient
      double d = 2.0 * _m_mass * _m_dampingRatio * omega;

      // Spring stiffness
      double k = _m_mass * omega * omega;

      // magic formulas
      double h = data.step.dt;
      _m_gamma = h * (d + h * k);
      _m_gamma = _m_gamma != 0.0 ? 1.0 / _m_gamma : 0.0;
      _m_bias = C * h * k * _m_gamma;

      invMass += _m_gamma;
      _m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    } else {
      _m_gamma = 0.0;
      _m_bias = 0.0;
    }
    if (data.step.warmStarting) {

      // Scale the impulse to support a variable time step.
      _m_impulse *= data.step.dtRatio;

      Vector2 P = pool.popVec2();
      P.set(_m_u).mul(_m_impulse);

      vA.x -= _m_invMassA * P.x;
      vA.y -= _m_invMassA * P.y;
      wA -= _m_invIA * Vector2.cross(_m_rA, P);

      vB.x += _m_invMassB * P.x;
      vB.y += _m_invMassB * P.y;
      wB += _m_invIB * Vector2.cross(_m_rB, P);

      pool.pushVec2(1);
    } else {
      _m_impulse = 0.0;
    }
//    data.velocities[_m_indexA].v.set(vA);
    data.velocities[_m_indexA].w = wA;
//    data.velocities[_m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;
    Vector2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Vector2 vpA = pool.popVec2();
    final Vector2 vpB = pool.popVec2();

    // Cdot = dot(u, v + cross(w, r))
    Vector2.crossToOutUnsafeDblVec2(wA, _m_rA, vpA);
    vpA.add(vA);
    Vector2.crossToOutUnsafeDblVec2(wB, _m_rB, vpB);
    vpB.add(vB);
    double Cdot = Vector2.dot(_m_u, vpB.sub(vpA));

    double impulse = -_m_mass * (Cdot + _m_bias + _m_gamma * _m_impulse);
    _m_impulse += impulse;

    double Px = impulse * _m_u.x;
    double Py = impulse * _m_u.y;

    vA.x -= _m_invMassA * Px;
    vA.y -= _m_invMassA * Py;
    wA -= _m_invIA * (_m_rA.x * Py - _m_rA.y * Px);
    vB.x += _m_invMassB * Px;
    vB.y += _m_invMassB * Py;
    wB += _m_invIB * (_m_rB.x * Py - _m_rB.y * Px);

//    data.velocities[_m_indexA].v.set(vA);
    data.velocities[_m_indexA].w = wA;
//    data.velocities[_m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;

    pool.pushVec2(2);
  }

  bool solvePositionConstraints(final SolverData data) {
    if (_m_frequencyHz > 0.0) {
      return true;
    }
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 u = pool.popVec2();

    Vector2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    Vector2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    Rot.mulToOutUnsafe(
        qA, u.set(_m_localAnchorA).sub(_m_localCenterA), rA);
    Rot.mulToOutUnsafe(
        qB, u.set(_m_localAnchorB).sub(_m_localCenterB), rB);
    u.set(cB).add(rB).sub(cA).sub(rA);

    double length = u.normalizeLength();
    double C = length - _m_length;
    C = MathUtils.clampDouble(
        C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);

    double impulse = -_m_mass * C;
    double Px = impulse * u.x;
    double Py = impulse * u.y;

    cA.x -= _m_invMassA * Px;
    cA.y -= _m_invMassA * Py;
    aA -= _m_invIA * (rA.x * Py - rA.y * Px);
    cB.x += _m_invMassB * Px;
    cB.y += _m_invMassB * Py;
    aB += _m_invIB * (rB.x * Py - rB.y * Px);

//    data.positions[_m_indexA].c.set(cA);
    data.positions[_m_indexA].a = aA;
//    data.positions[_m_indexB].c.set(cB);
    data.positions[_m_indexB].a = aB;

    pool.pushVec2(3);
    pool.pushRot(2);

    return C.abs() < Settings.linearSlop;
  }
}
