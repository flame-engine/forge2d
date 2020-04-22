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

class FrictionJoint extends Joint {
  final Vector2 _localAnchorA;
  final Vector2 _localAnchorB;

  // Solver shared
  final Vector2 _linearImpulse;
  double _angularImpulse = 0.0;
  double _maxForce = 0.0;
  double _maxTorque = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _rA = new Vector2.zero();
  final Vector2 _rB = new Vector2.zero();
  final Vector2 _localCenterA = new Vector2.zero();
  final Vector2 _localCenterB = new Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  final Matrix2 _linearMass = new Matrix2.zero();
  double _angularMass = 0.0;

  FrictionJoint(IWorldPool argWorldPool, FrictionJointDef def)
      : _localAnchorA = new Vector2.copy(def.localAnchorA),
        _localAnchorB = new Vector2.copy(def.localAnchorB),
        _linearImpulse = new Vector2.zero(),
        super(argWorldPool, def) {
    _maxForce = def.maxForce;
    _maxTorque = def.maxTorque;
  }

  Vector2 getLocalAnchorA() {
    return _localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return _localAnchorB;
  }

  void getAnchorA(Vector2 argOut) {
    _bodyA.getWorldPointToOut(_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    _bodyB.getWorldPointToOut(_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut
      ..setFrom(_linearImpulse)
      ..scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * _angularImpulse;
  }

  void setMaxForce(double force) {
    assert(force >= 0.0);
    _maxForce = force;
  }

  double getMaxForce() {
    return _maxForce;
  }

  void setMaxTorque(double torque) {
    assert(torque >= 0.0);
    _maxTorque = torque;
  }

  double getMaxTorque() {
    return _maxTorque;
  }

  /// @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)

  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

    double aA = data.positions[_indexA].a;
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 temp = pool.popVec2();
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective mass matrix.
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

    // J = [-I -r1_skew I r2_skew]
    // [ 0 -1 0 1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
    // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
    // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    final Matrix2 K = pool.popMat22();
    double a11 = mA + mB + iA * _rA.y * _rA.y + iB * _rB.y * _rB.y;
    double a21 = -iA * _rA.x * _rA.y - iB * _rB.x * _rB.y;
    double a12 = a21;
    double a22 = mA + mB + iA * _rA.x * _rA.x + iB * _rB.x * _rB.x;

    K.setValues(a11, a12, a21, a22);
    _linearMass.setFrom(K);
    _linearMass.invert();

    _angularMass = iA + iB;
    if (_angularMass > 0.0) {
      _angularMass = 1.0 / _angularMass;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      _linearImpulse.scale(data.step.dtRatio);
      _angularImpulse *= data.step.dtRatio;

      final Vector2 P = pool.popVec2();
      P.setFrom(_linearImpulse);

      temp
        ..setFrom(P)
        ..scale(mA);
      vA.sub(temp);
      wA -= iA * (_rA.cross(P) + _angularImpulse);

      temp
        ..setFrom(P)
        ..scale(mB);
      vB.add(temp);
      wB += iB * (_rB.cross(P) + _angularImpulse);

      pool.pushVec2(1);
    } else {
      _linearImpulse.setZero();
      _angularImpulse = 0.0;
    }
//    data.velocities[_indexA].v.set(vA);
    if (data.velocities[_indexA].w != wA) {
      assert(data.velocities[_indexA].w != wA);
    }
    data.velocities[_indexA].w = wA;
//    data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushRot(2);
    pool.pushVec2(1);
    pool.pushMat22(1);
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    double h = data.step.dt;

    // Solve angular friction
    {
      double Cdot = wB - wA;
      double impulse = -_angularMass * Cdot;

      double oldImpulse = _angularImpulse;
      double maxImpulse = h * _maxTorque;
      _angularImpulse = MathUtils.clampDouble(
          _angularImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = _angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve linear friction
    {
      final Vector2 Cdot = pool.popVec2();
      final Vector2 temp = pool.popVec2();

      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, Cdot);
      Cdot
        ..add(vB)
        ..sub(vA)
        ..sub(temp);

      final Vector2 impulse = pool.popVec2();
      _linearMass.transformed(Cdot, impulse);
      impulse.negate();

      final Vector2 oldImpulse = pool.popVec2();
      oldImpulse.setFrom(_linearImpulse);
      _linearImpulse.add(impulse);

      double maxImpulse = h * _maxForce;

      if (_linearImpulse.length2 > maxImpulse * maxImpulse) {
        _linearImpulse.normalize();
        _linearImpulse.scale(maxImpulse);
      }

      impulse
        ..setFrom(_linearImpulse)
        ..sub(oldImpulse);

      temp
        ..setFrom(impulse)
        ..scale(mA);
      vA.sub(temp);
      wA -= iA * _rA.cross(impulse);

      temp
        ..setFrom(impulse)
        ..scale(mB);
      vB.add(temp);
      wB += iB * _rB.cross(impulse);
    }

//    data.velocities[_indexA].v.set(vA);
    if (data.velocities[_indexA].w != wA) {
      assert(data.velocities[_indexA].w != wA);
    }
    data.velocities[_indexA].w = wA;

//    data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushVec2(4);
  }

  bool solvePositionConstraints(final SolverData data) {
    return true;
  }
}
