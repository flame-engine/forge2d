/// *****************************************************************************
/// Copyright (c) 2015, Daniel Murphy, Google
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
///  * Redistributions of source code must retain the above copyright notice,
///    this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// *****************************************************************************

part of box2d;

/// A rope joint enforces a maximum distance between two points on two bodies. It has no other
/// effect. Warning: if you attempt to change the maximum length during the simulation you will get
/// some non-physical behavior. A model that would allow you to dynamically modify the length would
/// have some sponginess, so I chose not to implement it that way. See DistanceJoint if you want to
/// dynamically control length.
class RopeJoint extends Joint {
  // Solver shared
  final Vector2 _localAnchorA = Vector2.zero();
  final Vector2 _localAnchorB = Vector2.zero();
  double _maxLength = 0.0;
  double _length = 0.0;
  double _impulse = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _u = Vector2.zero();
  final Vector2 _rA = Vector2.zero();
  final Vector2 _rB = Vector2.zero();
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  double _mass = 0.0;
  LimitState _state = LimitState.INACTIVE;

  RopeJoint(IWorldPool worldPool, RopeJointDef def) : super(worldPool, def) {
    _localAnchorA.setFrom(def.localAnchorA);
    _localAnchorB.setFrom(def.localAnchorB);

    _maxLength = def.maxLength;
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
    final Vector2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA,
        temp
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        _rA);
    Rot.mulToOutUnsafe(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        _rB);

    _u
      ..setFrom(cB)
      ..add(_rB)
      ..sub(cA)
      ..sub(_rA);

    _length = _u.length;

    double C = _length - _maxLength;
    if (C > 0.0) {
      _state = LimitState.AT_UPPER;
    } else {
      _state = LimitState.INACTIVE;
    }

    if (_length > Settings.linearSlop) {
      _u.scale(1.0 / _length);
    } else {
      _u.setZero();
      _mass = 0.0;
      _impulse = 0.0;
      pool.pushRot(2);
      pool.pushVec2(1);
      return;
    }

    // Compute effective mass.
    double crA = _rA.cross(_u);
    double crB = _rB.cross(_u);
    double invMass =
        _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      double Px = _impulse * _u.x;
      double Py = _impulse * _u.y;
      vA.x -= _invMassA * Px;
      vA.y -= _invMassA * Py;
      wA -= _invIA * (_rA.x * Py - _rA.y * Px);

      vB.x += _invMassB * Px;
      vB.y += _invMassB * Py;
      wB += _invIB * (_rB.x * Py - _rB.y * Px);
    } else {
      _impulse = 0.0;
    }

    pool.pushRot(2);
    pool.pushVec2(1);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    // Cdot = dot(u, v + cross(w, r))
    Vector2 vpA = pool.popVec2();
    Vector2 vpB = pool.popVec2();
    Vector2 temp = pool.popVec2();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    double C = _length - _maxLength;
    double Cdot = _u.dot(temp
      ..setFrom(vpB)
      ..sub(vpA));

    // Predictive constraint.
    if (C < 0.0) {
      Cdot += data.step.inv_dt * C;
    }

    double impulse = -_mass * Cdot;
    double oldImpulse = _impulse;
    _impulse = Math.min(0.0, _impulse + impulse);
    impulse = _impulse - oldImpulse;

    double Px = impulse * _u.x;
    double Py = impulse * _u.y;
    vA.x -= _invMassA * Px;
    vA.y -= _invMassA * Py;
    wA -= _invIA * (_rA.x * Py - _rA.y * Px);
    vB.x += _invMassB * Px;
    vB.y += _invMassB * Py;
    wB += _invIB * (_rB.x * Py - _rB.y * Px);

    pool.pushVec2(3);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 u = pool.popVec2();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA,
        temp
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        rA);
    Rot.mulToOutUnsafe(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        rB);
    u
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    double length = u.normalize();
    double C = length - _maxLength;

    C = MathUtils.clampDouble(C, 0.0, Settings.maxLinearCorrection);

    double impulse = -_mass * C;
    double Px = impulse * u.x;
    double Py = impulse * u.y;

    cA.x -= _invMassA * Px;
    cA.y -= _invMassA * Py;
    aA -= _invIA * (rA.x * Py - rA.y * Px);
    cB.x += _invMassB * Px;
    cB.y += _invMassB * Py;
    aB += _invIB * (rB.x * Py - rB.y * Px);

    pool.pushRot(2);
    pool.pushVec2(4);

    // data.positions[_indexA].c = cA;
    data.positions[_indexA].a = aA;
    // data.positions[_indexB].c = cB;
    data.positions[_indexB].a = aB;

    return length - _maxLength < Settings.linearSlop;
  }

  void getAnchorA(Vector2 argOut) {
    _bodyA.getWorldPointToOut(_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    _bodyB.getWorldPointToOut(_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut
      ..setFrom(_u)
      ..scale(inv_dt)
      ..scale(_impulse);
  }

  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  Vector2 getLocalAnchorA() {
    return _localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return _localAnchorB;
  }

  double getMaxLength() {
    return _maxLength;
  }

  void setMaxLength(double maxLength) {
    this._maxLength = maxLength;
  }

  LimitState getLimitState() {
    return _state;
  }
}
