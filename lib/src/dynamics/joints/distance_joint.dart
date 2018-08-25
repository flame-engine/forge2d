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
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;
  double _bias = 0.0;

  // Solver shared
  final Vector2 _localAnchorA;
  final Vector2 _localAnchorB;
  double _gamma = 0.0;
  double _impulse = 0.0;
  double _length = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _u = new Vector2.zero();
  final Vector2 _rA = new Vector2.zero();
  final Vector2 _rB = new Vector2.zero();
  final Vector2 _localCenterA = new Vector2.zero();
  final Vector2 _localCenterB = new Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  double _mass = 0.0;

  DistanceJoint(IWorldPool argWorld, final DistanceJointDef def)
      : _localAnchorA = def.localAnchorA.clone(),
        _localAnchorB = def.localAnchorB.clone(),
        super(argWorld, def) {
    _length = def.length;
    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
  }

  void getAnchorA(Vector2 argOut) {
    _bodyA.getWorldPointToOut(_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    _bodyB.getWorldPointToOut(_localAnchorB, argOut);
  }

  /**
   * Get the reaction force given the inverse time step. Unit is N.
   */

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut.x = _impulse * _u.x * inv_dt;
    argOut.y = _impulse * _u.y * inv_dt;
  }

  /**
   * Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a
   * distance joint.
   */

  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // use _u as temporary variable
    Rot.mulToOutUnsafe(
        qA,
        _u
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        _rA);
    Rot.mulToOutUnsafe(
        qB,
        _u
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        _rB);
    _u
      ..setFrom(cB)
      ..add(_rB)
      ..sub(cA)
      ..sub(_rA);

    pool.pushRot(2);

    // Handle singularity.
    double length = _u.length;
    if (length > Settings.linearSlop) {
      _u.x *= 1.0 / length;
      _u.y *= 1.0 / length;
    } else {
      _u.setValues(0.0, 0.0);
    }

    double crAu = _rA.cross(_u);
    double crBu = _rB.cross(_u);
    double invMass =
        _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

    // Compute the effective mass matrix.
    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (_frequencyHz > 0.0) {
      double C = length - _length;

      // Frequency
      double omega = 2.0 * Math.pi * _frequencyHz;

      // Damping coefficient
      double d = 2.0 * _mass * _dampingRatio * omega;

      // Spring stiffness
      double k = _mass * omega * omega;

      // magic formulas
      double h = data.step.dt;
      _gamma = h * (d + h * k);
      _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
      _bias = C * h * k * _gamma;

      invMass += _gamma;
      _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    } else {
      _gamma = 0.0;
      _bias = 0.0;
    }
    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      Vector2 P = pool.popVec2();
      P
        ..setFrom(_u)
        ..scale(_impulse);

      vA.x -= _invMassA * P.x;
      vA.y -= _invMassA * P.y;
      wA -= _invIA * _rA.cross(P);

      vB.x += _invMassB * P.x;
      vB.y += _invMassB * P.y;
      wB += _invIB * _rB.cross(P);

      pool.pushVec2(1);
    } else {
      _impulse = 0.0;
    }
//    data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
//    data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 vpA = pool.popVec2();
    final Vector2 vpB = pool.popVec2();

    // Cdot = dot(u, v + cross(w, r))
    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);
    double Cdot = _u.dot(vpB..sub(vpA));

    double impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
    _impulse += impulse;

    double Px = impulse * _u.x;
    double Py = impulse * _u.y;

    vA.x -= _invMassA * Px;
    vA.y -= _invMassA * Py;
    wA -= _invIA * (_rA.x * Py - _rA.y * Px);
    vB.x += _invMassB * Px;
    vB.y += _invMassB * Py;
    wB += _invIB * (_rB.x * Py - _rB.y * Px);

//    data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
//    data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushVec2(2);
  }

  bool solvePositionConstraints(final SolverData data) {
    if (_frequencyHz > 0.0) {
      return true;
    }
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 u = pool.popVec2();

    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    Rot.mulToOutUnsafe(
        qA,
        u
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        rA);
    Rot.mulToOutUnsafe(
        qB,
        u
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        rB);
    u
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    double length = u.normalize();
    double C = length - _length;
    C = MathUtils.clampDouble(
        C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);

    double impulse = -_mass * C;
    double Px = impulse * u.x;
    double Py = impulse * u.y;

    cA.x -= _invMassA * Px;
    cA.y -= _invMassA * Py;
    aA -= _invIA * (rA.x * Py - rA.y * Px);
    cB.x += _invMassB * Px;
    cB.y += _invMassB * Py;
    aB += _invIB * (rB.x * Py - rB.y * Px);

//    data.positions[_indexA].c.set(cA);
    data.positions[_indexA].a = aA;
//    data.positions[_indexB].c.set(cB);
    data.positions[_indexB].a = aB;

    pool.pushVec2(3);
    pool.pushRot(2);

    return C.abs() < Settings.linearSlop;
  }
}
