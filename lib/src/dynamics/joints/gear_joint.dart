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

//Gear Joint:
//C0 = (coordinate1 + ratio * coordinate2)_initial
//C = (coordinate1 + ratio * coordinate2) - C0 = 0
//J = [J1 ratio * J2]
//K = J * invM * JT
//= J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
//Revolute:
//coordinate = rotation
//Cdot = angularVelocity
//J = [0 0 1]
//K = J * invM * JT = invI
//
//Prismatic:
//coordinate = dot(p - pg, ug)
//Cdot = dot(v + cross(w, r), ug)
//J = [ug cross(r, ug)]
//K = J * invM * JT = invMass + invI * cross(r, ug)^2

/// A gear joint is used to connect two joints together. Either joint can be a revolute or prismatic
/// joint. You specify a gear ratio to bind the motions together: coordinate1 + ratio * coordinate2 =
/// constant The ratio can be negative or positive. If one joint is a revolute joint and the other
/// joint is a prismatic joint, then the ratio will have units of length or units of 1/length.
///
/// @warning The revolute and prismatic joints must be attached to fixed bodies (which must be body1
///          on those joints).
/// @warning You have to manually destroy the gear joint if joint1 or joint2 is destroyed.
class GearJoint extends Joint {
  final Joint _joint1;
  final Joint _joint2;

  final JointType _typeA;
  final JointType _typeB;

  // Body A is connected to body C
  // Body B is connected to body D
  final Body _bodyC;
  final Body _bodyD;

  // Solver shared
  final Vector2 _localAnchorA = Vector2.zero();
  final Vector2 _localAnchorB = Vector2.zero();
  final Vector2 _localAnchorC = Vector2.zero();
  final Vector2 _localAnchorD = Vector2.zero();

  final Vector2 _localAxisC = Vector2.zero();
  final Vector2 _localAxisD = Vector2.zero();

  double _referenceAngleA = 0.0;
  double _referenceAngleB = 0.0;

  double _constant = 0.0;
  double _ratio = 0.0;

  double _impulse = 0.0;

  // Solver temp
  int _indexA = 0, _indexB = 0, _indexC = 0, _indexD = 0;
  final Vector2 _lcA = Vector2.zero(),
      _lcB = Vector2.zero(),
      _lcC = Vector2.zero(),
      _lcD = Vector2.zero();
  double _mA = 0.0, _mB = 0.0, _mC = 0.0, _mD = 0.0;
  double _iA = 0.0, _iB = 0.0, _iC = 0.0, _iD = 0.0;
  final Vector2 _JvAC = Vector2.zero(), _JvBD = Vector2.zero();
  double _JwA = 0.0, _JwB = 0.0, _JwC = 0.0, _JwD = 0.0;
  double _mass = 0.0;

  GearJoint(IWorldPool argWorldPool, GearJointDef def)
      : _joint1 = def.joint1,
        _joint2 = def.joint2,
        _typeA = def.joint1.getType(),
        _typeB = def.joint2.getType(),
        _bodyC = def.joint1.getBodyA(),
        _bodyD = def.joint2.getBodyA(),
        super(argWorldPool, def) {
    assert(_typeA == JointType.REVOLUTE || _typeA == JointType.PRISMATIC);
    assert(_typeB == JointType.REVOLUTE || _typeB == JointType.PRISMATIC);

    double coordinateA, coordinateB;

    // TODO_ERIN there might be some problem with the joint edges in Joint.
    _bodyA = _joint1.getBodyB();

    // Get geometry of joint1
    Transform xfA = _bodyA._transform;
    double aA = _bodyA._sweep.a;
    Transform xfC = _bodyC._transform;
    double aC = _bodyC._sweep.a;

    if (_typeA == JointType.REVOLUTE) {
      final revolute = def.joint1 as RevoluteJoint;
      _localAnchorC.setFrom(revolute._localAnchorA);
      _localAnchorA.setFrom(revolute._localAnchorB);
      _referenceAngleA = revolute._referenceAngle;
      _localAxisC.setZero();

      coordinateA = aA - aC - _referenceAngleA;
    } else {
      Vector2 pA = pool.popVec2();
      Vector2 temp = pool.popVec2();
      final prismatic = def.joint1 as PrismaticJoint;
      _localAnchorC.setFrom(prismatic._localAnchorA);
      _localAnchorA.setFrom(prismatic._localAnchorB);
      _referenceAngleA = prismatic._referenceAngle;
      _localAxisC.setFrom(prismatic._localXAxisA);

      Vector2 pC = _localAnchorC;
      Rot.mulToOutUnsafe(xfA.q, _localAnchorA, temp);
      temp
        ..add(xfA.p)
        ..sub(xfC.p);
      Rot.mulTransUnsafeVec2(xfC.q, temp, pA);
      coordinateA = (pA..sub(pC)).dot(_localAxisC);
      pool.pushVec2(2);
    }

    _bodyB = _joint2.getBodyB();

    // Get geometry of joint2
    Transform xfB = _bodyB._transform;
    double aB = _bodyB._sweep.a;
    Transform xfD = _bodyD._transform;
    double aD = _bodyD._sweep.a;

    if (_typeB == JointType.REVOLUTE) {
      final revolute = def.joint2 as RevoluteJoint;
      _localAnchorD.setFrom(revolute._localAnchorA);
      _localAnchorB.setFrom(revolute._localAnchorB);
      _referenceAngleB = revolute._referenceAngle;
      _localAxisD.setZero();

      coordinateB = aB - aD - _referenceAngleB;
    } else {
      Vector2 pB = pool.popVec2();
      Vector2 temp = pool.popVec2();
      final prismatic = def.joint2 as PrismaticJoint;
      _localAnchorD.setFrom(prismatic._localAnchorA);
      _localAnchorB.setFrom(prismatic._localAnchorB);
      _referenceAngleB = prismatic._referenceAngle;
      _localAxisD.setFrom(prismatic._localXAxisA);

      Vector2 pD = _localAnchorD;
      Rot.mulToOutUnsafe(xfB.q, _localAnchorB, temp);
      temp
        ..add(xfB.p)
        ..sub(xfD.p);
      Rot.mulTransUnsafeVec2(xfD.q, temp, pB);
      coordinateB = (pB..sub(pD)).dot(_localAxisD);
      pool.pushVec2(2);
    }

    _ratio = def.ratio;

    _constant = coordinateA + _ratio * coordinateB;

    _impulse = 0.0;
  }

  void getAnchorA(Vector2 argOut) {
    _bodyA.getWorldPointToOut(_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    _bodyB.getWorldPointToOut(_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut
      ..setFrom(_JvAC)
      ..scale(_impulse);
    argOut.scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    double L = _impulse * _JwA;
    return inv_dt * L;
  }

  void setRatio(double argRatio) {
    _ratio = argRatio;
  }

  double getRatio() {
    return _ratio;
  }

  void initVelocityConstraints(SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
    _indexC = _bodyC._islandIndex;
    _indexD = _bodyD._islandIndex;
    _lcA.setFrom(_bodyA._sweep.localCenter);
    _lcB.setFrom(_bodyB._sweep.localCenter);
    _lcC.setFrom(_bodyC._sweep.localCenter);
    _lcD.setFrom(_bodyD._sweep.localCenter);
    _mA = _bodyA._invMass;
    _mB = _bodyB._invMass;
    _mC = _bodyC._invMass;
    _mD = _bodyD._invMass;
    _iA = _bodyA._invI;
    _iB = _bodyB._invI;
    _iC = _bodyC._invI;
    _iD = _bodyD._invI;

    // Vec2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    // Vec2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    // Vec2 cC = data.positions[_indexC].c;
    double aC = data.positions[_indexC].a;
    Vector2 vC = data.velocities[_indexC].v;
    double wC = data.velocities[_indexC].w;

    // Vec2 cD = data.positions[_indexD].c;
    double aD = data.positions[_indexD].a;
    Vector2 vD = data.velocities[_indexD].v;
    double wD = data.velocities[_indexD].w;

    Rot qA = pool.popRot(),
        qB = pool.popRot(),
        qC = pool.popRot(),
        qD = pool.popRot();
    qA.setAngle(aA);
    qB.setAngle(aB);
    qC.setAngle(aC);
    qD.setAngle(aD);

    _mass = 0.0;

    Vector2 temp = pool.popVec2();

    if (_typeA == JointType.REVOLUTE) {
      _JvAC.setZero();
      _JwA = 1.0;
      _JwC = 1.0;
      _mass += _iA + _iC;
    } else {
      Vector2 rC = pool.popVec2();
      Vector2 rA = pool.popVec2();
      Rot.mulToOutUnsafe(qC, _localAxisC, _JvAC);
      Rot.mulToOutUnsafe(
          qC,
          temp
            ..setFrom(_localAnchorC)
            ..sub(_lcC),
          rC);
      Rot.mulToOutUnsafe(
          qA,
          temp
            ..setFrom(_localAnchorA)
            ..sub(_lcA),
          rA);
      _JwC = rC.cross(_JvAC);
      _JwA = rA.cross(_JvAC);
      _mass += _mC + _mA + _iC * _JwC * _JwC + _iA * _JwA * _JwA;
      pool.pushVec2(2);
    }

    if (_typeB == JointType.REVOLUTE) {
      _JvBD.setZero();
      _JwB = _ratio;
      _JwD = _ratio;
      _mass += _ratio * _ratio * (_iB + _iD);
    } else {
      Vector2 u = pool.popVec2();
      Vector2 rD = pool.popVec2();
      Vector2 rB = pool.popVec2();
      Rot.mulToOutUnsafe(qD, _localAxisD, u);
      Rot.mulToOutUnsafe(
          qD,
          temp
            ..setFrom(_localAnchorD)
            ..sub(_lcD),
          rD);
      Rot.mulToOutUnsafe(
          qB,
          temp
            ..setFrom(_localAnchorB)
            ..sub(_lcB),
          rB);
      _JvBD
        ..setFrom(u)
        ..scale(_ratio);
      _JwD = _ratio * rD.cross(u);
      _JwB = _ratio * rB.cross(u);
      _mass +=
          _ratio * _ratio * (_mD + _mB) + _iD * _JwD * _JwD + _iB * _JwB * _JwB;
      pool.pushVec2(3);
    }

    // Compute effective mass.
    _mass = _mass > 0.0 ? 1.0 / _mass : 0.0;

    if (data.step.warmStarting) {
      vA.x += (_mA * _impulse) * _JvAC.x;
      vA.y += (_mA * _impulse) * _JvAC.y;
      wA += _iA * _impulse * _JwA;

      vB.x += (_mB * _impulse) * _JvBD.x;
      vB.y += (_mB * _impulse) * _JvBD.y;
      wB += _iB * _impulse * _JwB;

      vC.x -= (_mC * _impulse) * _JvAC.x;
      vC.y -= (_mC * _impulse) * _JvAC.y;
      wC -= _iC * _impulse * _JwC;

      vD.x -= (_mD * _impulse) * _JvBD.x;
      vD.y -= (_mD * _impulse) * _JvBD.y;
      wD -= _iD * _impulse * _JwD;
    } else {
      _impulse = 0.0;
    }
    pool.pushVec2(1);
    pool.pushRot(4);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
    // data.velocities[_indexC].v = vC;
    data.velocities[_indexC].w = wC;
    // data.velocities[_indexD].v = vD;
    data.velocities[_indexD].w = wD;
  }

  void solveVelocityConstraints(SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;
    Vector2 vC = data.velocities[_indexC].v;
    double wC = data.velocities[_indexC].w;
    Vector2 vD = data.velocities[_indexD].v;
    double wD = data.velocities[_indexD].w;

    Vector2 temp1 = pool.popVec2();
    Vector2 temp2 = pool.popVec2();
    double Cdot = _JvAC.dot(temp1
          ..setFrom(vA)
          ..sub(vC)) +
        _JvBD.dot(temp2
          ..setFrom(vB)
          ..sub(vD));
    Cdot += (_JwA * wA - _JwC * wC) + (_JwB * wB - _JwD * wD);
    pool.pushVec2(2);

    double impulse = -_mass * Cdot;
    _impulse += impulse;

    vA.x += (_mA * impulse) * _JvAC.x;
    vA.y += (_mA * impulse) * _JvAC.y;
    wA += _iA * impulse * _JwA;

    vB.x += (_mB * impulse) * _JvBD.x;
    vB.y += (_mB * impulse) * _JvBD.y;
    wB += _iB * impulse * _JwB;

    vC.x -= (_mC * impulse) * _JvAC.x;
    vC.y -= (_mC * impulse) * _JvAC.y;
    wC -= _iC * impulse * _JwC;

    vD.x -= (_mD * impulse) * _JvBD.x;
    vD.y -= (_mD * impulse) * _JvBD.y;
    wD -= _iD * impulse * _JwD;

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
    // data.velocities[_indexC].v = vC;
    data.velocities[_indexC].w = wC;
    // data.velocities[_indexD].v = vD;
    data.velocities[_indexD].w = wD;
  }

  Joint getJoint1() {
    return _joint1;
  }

  Joint getJoint2() {
    return _joint2;
  }

  bool solvePositionConstraints(SolverData data) {
    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    Vector2 cC = data.positions[_indexC].c;
    double aC = data.positions[_indexC].a;
    Vector2 cD = data.positions[_indexD].c;
    double aD = data.positions[_indexD].a;

    Rot qA = pool.popRot(),
        qB = pool.popRot(),
        qC = pool.popRot(),
        qD = pool.popRot();
    qA.setAngle(aA);
    qB.setAngle(aB);
    qC.setAngle(aC);
    qD.setAngle(aD);

    double linearError = 0.0;

    double coordinateA, coordinateB;

    Vector2 temp = pool.popVec2();
    Vector2 JvAC = pool.popVec2();
    Vector2 JvBD = pool.popVec2();
    double JwA, JwB, JwC, JwD;
    double mass = 0.0;

    if (_typeA == JointType.REVOLUTE) {
      JvAC.setZero();
      JwA = 1.0;
      JwC = 1.0;
      mass += _iA + _iC;

      coordinateA = aA - aC - _referenceAngleA;
    } else {
      Vector2 rC = pool.popVec2();
      Vector2 rA = pool.popVec2();
      Vector2 pC = pool.popVec2();
      Vector2 pA = pool.popVec2();
      Rot.mulToOutUnsafe(qC, _localAxisC, JvAC);
      Rot.mulToOutUnsafe(
          qC,
          temp
            ..setFrom(_localAnchorC)
            ..sub(_lcC),
          rC);
      Rot.mulToOutUnsafe(
          qA,
          temp
            ..setFrom(_localAnchorA)
            ..sub(_lcA),
          rA);
      JwC = rC.cross(JvAC);
      JwA = rA.cross(JvAC);
      mass += _mC + _mA + _iC * JwC * JwC + _iA * JwA * JwA;

      pC
        ..setFrom(_localAnchorC)
        ..sub(_lcC);
      Rot.mulTransUnsafeVec2(
          qC,
          temp
            ..setFrom(rA)
            ..add(cA)
            ..sub(cC),
          pA);
      coordinateA = (pA..sub(pC)).dot(_localAxisC);
      pool.pushVec2(4);
    }

    if (_typeB == JointType.REVOLUTE) {
      JvBD.setZero();
      JwB = _ratio;
      JwD = _ratio;
      mass += _ratio * _ratio * (_iB + _iD);

      coordinateB = aB - aD - _referenceAngleB;
    } else {
      Vector2 u = pool.popVec2();
      Vector2 rD = pool.popVec2();
      Vector2 rB = pool.popVec2();
      Vector2 pD = pool.popVec2();
      Vector2 pB = pool.popVec2();
      Rot.mulToOutUnsafe(qD, _localAxisD, u);
      Rot.mulToOutUnsafe(
          qD,
          temp
            ..setFrom(_localAnchorD)
            ..sub(_lcD),
          rD);
      Rot.mulToOutUnsafe(
          qB,
          temp
            ..setFrom(_localAnchorB)
            ..sub(_lcB),
          rB);
      JvBD
        ..setFrom(u)
        ..scale(_ratio);
      JwD = rD.cross(u);
      JwB = rB.cross(u);
      mass += _ratio * _ratio * (_mD + _mB) + _iD * JwD * JwD + _iB * JwB * JwB;

      pD
        ..setFrom(_localAnchorD)
        ..sub(_lcD);
      Rot.mulTransUnsafeVec2(
          qD,
          temp
            ..setFrom(rB)
            ..add(cB)
            ..sub(cD),
          pB);
      coordinateB = (pB..sub(pD)).dot(_localAxisD);
      pool.pushVec2(5);
    }

    double C = (coordinateA + _ratio * coordinateB) - _constant;

    double impulse = 0.0;
    if (mass > 0.0) {
      impulse = -C / mass;
    }
    pool.pushVec2(3);
    pool.pushRot(4);

    cA.x += (_mA * impulse) * JvAC.x;
    cA.y += (_mA * impulse) * JvAC.y;
    aA += _iA * impulse * JwA;

    cB.x += (_mB * impulse) * JvBD.x;
    cB.y += (_mB * impulse) * JvBD.y;
    aB += _iB * impulse * JwB;

    cC.x -= (_mC * impulse) * JvAC.x;
    cC.y -= (_mC * impulse) * JvAC.y;
    aC -= _iC * impulse * JwC;

    cD.x -= (_mD * impulse) * JvBD.x;
    cD.y -= (_mD * impulse) * JvBD.y;
    aD -= _iD * impulse * JwD;

    // data.positions[_indexA].c = cA;
    data.positions[_indexA].a = aA;
    // data.positions[_indexB].c = cB;
    data.positions[_indexB].a = aB;
    // data.positions[_indexC].c = cC;
    data.positions[_indexC].a = aC;
    // data.positions[_indexD].c = cD;
    data.positions[_indexD].a = aD;

    // TODO_ERIN not implemented
    return linearError < Settings.linearSlop;
  }
}
