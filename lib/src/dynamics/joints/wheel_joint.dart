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

//Linear constraint (point-to-line)
//d = pB - pA = xB + rB - xA - rA
//C = dot(ay, d)
//Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//   = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
//J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

//Spring linear constraint
//C = dot(ax, d)
//Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
//J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

//Motor rotational constraint
//Cdot = wB - wA
//J = [0 0 -1 0 0 1]

/**
 * A wheel joint. This joint provides two degrees of freedom: translation along an axis fixed in
 * bodyA and rotation in the plane. You can use a joint limit to restrict the range of motion and a
 * joint motor to drive the rotation or to model rotational friction. This joint is designed for
 * vehicle suspensions.
 *
 * @author Daniel Murphy
 */
class WheelJoint extends Joint {
  // TODO(srdjan): make fields private.
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;

  // Solver shared
  final Vector2 _localAnchorA = new Vector2.zero();
  final Vector2 _localAnchorB = new Vector2.zero();
  final Vector2 _localXAxisA = new Vector2.zero();
  final Vector2 _localYAxisA = new Vector2.zero();

  double _impulse = 0.0;
  double _motorImpulse = 0.0;
  double _springImpulse = 0.0;

  double _maxMotorTorque = 0.0;
  double _motorSpeed = 0.0;
  bool _enableMotor = false;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _localCenterA = new Vector2.zero();
  final Vector2 _localCenterB = new Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;

  final Vector2 _ax = new Vector2.zero();
  final Vector2 _ay = new Vector2.zero();
  double _sAx = 0.0, _sBx = 0.0;
  double _sAy = 0.0, _sBy = 0.0;

  double _mass = 0.0;
  double _motorMass = 0.0;
  double _springMass = 0.0;

  double _bias = 0.0;
  double _gamma = 0.0;

  WheelJoint(IWorldPool argPool, WheelJointDef def) : super(argPool, def) {
    _localAnchorA.setFrom(def.localAnchorA);
    _localAnchorB.setFrom(def.localAnchorB);
    _localXAxisA.setFrom(def.localAxisA);
    _localXAxisA.scaleOrthogonalInto(1.0, _localYAxisA);

    _motorMass = 0.0;
    _motorImpulse = 0.0;

    _maxMotorTorque = def.maxMotorTorque;
    _motorSpeed = def.motorSpeed;
    _enableMotor = def.enableMotor;

    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
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
    final Vector2 temp = pool.popVec2();
    temp
      ..setFrom(_ay)
      ..scale(_impulse);
    argOut
      ..setFrom(_ax)
      ..scale(_springImpulse)
      ..add(temp)
      ..scale(inv_dt);
    pool.pushVec2(1);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * _motorImpulse;
  }

  double getJointTranslation() {
    Body b1 = _bodyA;
    Body b2 = _bodyB;

    Vector2 p1 = pool.popVec2();
    Vector2 p2 = pool.popVec2();
    Vector2 axis = pool.popVec2();
    b1.getWorldPointToOut(_localAnchorA, p1);
    b2.getWorldPointToOut(_localAnchorA, p2);
    p2.sub(p1);
    b1.getWorldVectorToOut(_localXAxisA, axis);

    double translation = p2.dot(axis);
    pool.pushVec2(3);
    return translation;
  }

  /** For serialization */
  Vector2 getLocalAxisA() {
    return _localXAxisA;
  }

  double getJointSpeed() {
    return _bodyA._angularVelocity - _bodyB._angularVelocity;
  }

  bool isMotorEnabled() {
    return _enableMotor;
  }

  void enableMotor(bool flag) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _enableMotor = flag;
  }

  void setMotorSpeed(double speed) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _motorSpeed = speed;
  }

  double getMotorSpeed() {
    return _motorSpeed;
  }

  double getMaxMotorTorque() {
    return _maxMotorTorque;
  }

  void setMaxMotorTorque(double torque) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _maxMotorTorque = torque;
  }

  double getMotorTorque(double inv_dt) {
    return _motorImpulse * inv_dt;
  }

  // pooling
  // TODO(srdjan): Make fields private.
  final Vector2 rA = new Vector2.zero();
  final Vector2 rB = new Vector2.zero();
  final Vector2 d = new Vector2.zero();

  void initVelocityConstraints(SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

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
        rA);
    Rot.mulToOutUnsafe(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        rB);
    d
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    // Point to line constraint
    {
      Rot.mulToOut(qA, _localYAxisA, _ay);
      _sAy = (temp
            ..setFrom(d)
            ..add(rA))
          .cross(_ay);
      _sBy = rB.cross(_ay);

      _mass = mA + mB + iA * _sAy * _sAy + iB * _sBy * _sBy;

      if (_mass > 0.0) {
        _mass = 1.0 / _mass;
      }
    }

    // Spring constraint
    _springMass = 0.0;
    _bias = 0.0;
    _gamma = 0.0;
    if (_frequencyHz > 0.0) {
      Rot.mulToOut(qA, _localXAxisA, _ax);
      _sAx = (temp
            ..setFrom(d)
            ..add(rA))
          .cross(_ax);
      _sBx = rB.cross(_ax);

      double invMass = mA + mB + iA * _sAx * _sAx + iB * _sBx * _sBx;

      if (invMass > 0.0) {
        _springMass = 1.0 / invMass;

        double C = d.dot(_ax);

        // Frequency
        double omega = 2.0 * Math.pi * _frequencyHz;

        // Damping coefficient
        double dd = 2.0 * _springMass * _dampingRatio * omega;

        // Spring stiffness
        double k = _springMass * omega * omega;

        // magic formulas
        double h = data.step.dt;
        _gamma = h * (dd + h * k);
        if (_gamma > 0.0) {
          _gamma = 1.0 / _gamma;
        }

        _bias = C * h * k * _gamma;

        _springMass = invMass + _gamma;
        if (_springMass > 0.0) {
          _springMass = 1.0 / _springMass;
        }
      }
    } else {
      _springImpulse = 0.0;
    }

    // Rotational motor
    if (_enableMotor) {
      _motorMass = iA + iB;
      if (_motorMass > 0.0) {
        _motorMass = 1.0 / _motorMass;
      }
    } else {
      _motorMass = 0.0;
      _motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      final Vector2 P = pool.popVec2();
      // Account for variable time step.
      _impulse *= data.step.dtRatio;
      _springImpulse *= data.step.dtRatio;
      _motorImpulse *= data.step.dtRatio;

      P.x = _impulse * _ay.x + _springImpulse * _ax.x;
      P.y = _impulse * _ay.y + _springImpulse * _ax.y;
      double LA = _impulse * _sAy + _springImpulse * _sAx + _motorImpulse;
      double LB = _impulse * _sBy + _springImpulse * _sBx + _motorImpulse;

      vA.x -= _invMassA * P.x;
      vA.y -= _invMassA * P.y;
      wA -= _invIA * LA;

      vB.x += _invMassB * P.x;
      vB.y += _invMassB * P.y;
      wB += _invIB * LB;
      pool.pushVec2(1);
    } else {
      _impulse = 0.0;
      _springImpulse = 0.0;
      _motorImpulse = 0.0;
    }
    pool.pushRot(2);
    pool.pushVec2(1);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  void solveVelocityConstraints(SolverData data) {
    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 temp = pool.popVec2();
    final Vector2 P = pool.popVec2();

    // Solve spring constraint
    {
      double Cdot = _ax.dot(temp
            ..setFrom(vB)
            ..sub(vA)) +
          _sBx * wB -
          _sAx * wA;
      double impulse = -_springMass * (Cdot + _bias + _gamma * _springImpulse);
      _springImpulse += impulse;

      P.x = impulse * _ax.x;
      P.y = impulse * _ax.y;
      double LA = impulse * _sAx;
      double LB = impulse * _sBx;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }

    // Solve rotational motor constraint
    {
      double Cdot = wB - wA - _motorSpeed;
      double impulse = -_motorMass * Cdot;

      double oldImpulse = _motorImpulse;
      double maxImpulse = data.step.dt * _maxMotorTorque;
      _motorImpulse = MathUtils.clampDouble(
          _motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = _motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve point to line constraint
    {
      double Cdot = _ay.dot(temp
            ..setFrom(vB)
            ..sub(vA)) +
          _sBy * wB -
          _sAy * wA;
      double impulse = -_mass * Cdot;
      _impulse += impulse;

      P.x = impulse * _ay.x;
      P.y = impulse * _ay.y;
      double LA = impulse * _sAy;
      double LB = impulse * _sBy;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }
    pool.pushVec2(2);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(SolverData data) {
    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    Rot.mulToOut(
        qA,
        temp
          ..setFrom(_localAnchorA)
          ..sub(_localCenterA),
        rA);
    Rot.mulToOut(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        rB);
    d
      ..setFrom(cB)
      ..sub(cA)
      ..add(rB)
      ..sub(rA);

    Vector2 ay = pool.popVec2();
    Rot.mulToOut(qA, _localYAxisA, ay);

    double sAy = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(ay);
    double sBy = rB.cross(ay);

    double C = d.dot(ay);

    double k =
        _invMassA + _invMassB + _invIA * _sAy * _sAy + _invIB * _sBy * _sBy;

    double impulse;
    if (k != 0.0) {
      impulse = -C / k;
    } else {
      impulse = 0.0;
    }

    final Vector2 P = pool.popVec2();
    P.x = impulse * ay.x;
    P.y = impulse * ay.y;
    double LA = impulse * sAy;
    double LB = impulse * sBy;

    cA.x -= _invMassA * P.x;
    cA.y -= _invMassA * P.y;
    aA -= _invIA * LA;
    cB.x += _invMassB * P.x;
    cB.y += _invMassB * P.y;
    aB += _invIB * LB;

    pool.pushVec2(3);
    pool.pushRot(2);
    // data.positions[_indexA].c = cA;
    data.positions[_indexA].a = aA;
    // data.positions[_indexB].c = cB;
    data.positions[_indexB].a = aB;

    return C.abs() <= Settings.linearSlop;
  }
}
