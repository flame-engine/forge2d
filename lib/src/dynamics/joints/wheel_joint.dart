part of forge2d;

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

/// A wheel joint. This joint provides two degrees of freedom: translation along an axis fixed in
/// bodyA and rotation in the plane. You can use a joint limit to restrict the range of motion and a
/// joint motor to drive the rotation or to model rotational friction. This joint is designed for
/// vehicle suspensions.
class WheelJoint extends Joint {
  // TODO(srdjan): make fields private.
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;

  // Solver shared
  @override
  final Vector2 localAnchorA = Vector2.zero();
  @override
  final Vector2 localAnchorB = Vector2.zero();
  final Vector2 _localXAxisA = Vector2.zero();
  final Vector2 _localYAxisA = Vector2.zero();

  double _impulse = 0.0;
  double _motorImpulse = 0.0;
  double _springImpulse = 0.0;

  double _maxMotorTorque = 0.0;
  double _motorSpeed = 0.0;
  bool _enableMotor = false;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;

  final Vector2 _ax = Vector2.zero();
  final Vector2 _ay = Vector2.zero();
  double _sAx = 0.0, _sBx = 0.0;
  double _sAy = 0.0, _sBy = 0.0;

  double _mass = 0.0;
  double _motorMass = 0.0;
  double _springMass = 0.0;

  double _bias = 0.0;
  double _gamma = 0.0;

  WheelJoint(WheelJointDef def) : super(def) {
    localAnchorA.setFrom(def.localAnchorA);
    localAnchorB.setFrom(def.localAnchorB);
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

  @override
  Vector2 getReactionForce(double invDt) {
    final Vector2 temp = Vector2.zero();
    temp
      ..setFrom(_ay)
      ..scale(_impulse);
    final Vector2 result = Vector2.copy(_ax)
      ..setFrom(_ax)
      ..scale(_springImpulse)
      ..add(temp)
      ..scale(invDt);
    return result;
  }

  @override
  double getReactionTorque(double invDt) {
    return invDt * _motorImpulse;
  }

  double getJointTranslation() {
    final Body b1 = bodyA;
    final Body b2 = bodyB;

    final Vector2 p1 = Vector2.zero();
    final Vector2 p2 = Vector2.zero();
    final Vector2 axis = Vector2.zero();
    p1.setFrom(b1.getWorldPoint(localAnchorA));
    p2.setFrom(b2.getWorldPoint(localAnchorA));
    p2.sub(p1);
    axis.setFrom(b1.getWorldVector(_localXAxisA));

    final double translation = p2.dot(axis);
    return translation;
  }

  /// For serialization
  Vector2 getLocalAxisA() {
    return _localXAxisA;
  }

  double getJointSpeed() {
    return bodyA._angularVelocity - bodyB._angularVelocity;
  }

  bool isMotorEnabled() {
    return _enableMotor;
  }

  void enableMotor(bool flag) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _enableMotor = flag;
  }

  void setMotorSpeed(double speed) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _motorSpeed = speed;
  }

  double getMotorSpeed() {
    return _motorSpeed;
  }

  double getMaxMotorTorque() {
    return _maxMotorTorque;
  }

  void setMaxMotorTorque(double torque) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _maxMotorTorque = torque;
  }

  double getMotorTorque(double invDt) {
    return _motorImpulse * invDt;
  }

  // pooling
  // TODO(srdjan): Make fields private.
  final Vector2 rA = Vector2.zero();
  final Vector2 rB = Vector2.zero();
  final Vector2 d = Vector2.zero();

  @override
  void initVelocityConstraints(SolverData data) {
    _indexA = bodyA.islandIndex;
    _indexB = bodyB.islandIndex;
    _localCenterA.setFrom(bodyA._sweep.localCenter);
    _localCenterB.setFrom(bodyB._sweep.localCenter);
    _invMassA = bodyA._invMass;
    _invMassB = bodyB._invMass;
    _invIA = bodyA.inverseInertia;
    _invIB = bodyB.inverseInertia;

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final Vector2 cA = data.positions[_indexA].c;
    final double aA = data.positions[_indexA].a;
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    final Vector2 cB = data.positions[_indexB].c;
    final double aB = data.positions[_indexB].a;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 temp = Vector2.zero();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    temp
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    rB.setFrom(Rot.mulVec2(qB, temp));

    d
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    // Point to line constraint
    {
      _ay.setFrom(Rot.mulVec2(qA, _localYAxisA));
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
      _ax.setFrom(Rot.mulVec2(qA, _localXAxisA));
      _sAx = (temp
            ..setFrom(d)
            ..add(rA))
          .cross(_ax);
      _sBx = rB.cross(_ax);

      final double invMass = mA + mB + iA * _sAx * _sAx + iB * _sBx * _sBx;

      if (invMass > 0.0) {
        _springMass = 1.0 / invMass;

        final double c = d.dot(_ax);

        // Frequency
        final double omega = 2.0 * math.pi * _frequencyHz;

        // Damping coefficient
        final double dd = 2.0 * _springMass * _dampingRatio * omega;

        // Spring stiffness
        final double k = _springMass * omega * omega;

        // magic formulas
        final double dt = data.step.dt;
        _gamma = dt * (dd + dt * k);
        if (_gamma > 0.0) {
          _gamma = 1.0 / _gamma;
        }

        _bias = c * dt * k * _gamma;

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
      final Vector2 p = Vector2.zero();
      // Account for variable time step.
      _impulse *= data.step.dtRatio;
      _springImpulse *= data.step.dtRatio;
      _motorImpulse *= data.step.dtRatio;

      p.x = _impulse * _ay.x + _springImpulse * _ax.x;
      p.y = _impulse * _ay.y + _springImpulse * _ax.y;
      final double lA = _impulse * _sAy + _springImpulse * _sAx + _motorImpulse;
      final double lB = _impulse * _sBy + _springImpulse * _sBx + _motorImpulse;

      vA.x -= _invMassA * p.x;
      vA.y -= _invMassA * p.y;
      wA -= _invIA * lA;

      vB.x += _invMassB * p.x;
      vB.y += _invMassB * p.y;
      wB += _invIB * lB;
    } else {
      _impulse = 0.0;
      _springImpulse = 0.0;
      _motorImpulse = 0.0;
    }

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  @override
  void solveVelocityConstraints(SolverData data) {
    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 temp = Vector2.zero();
    final Vector2 p = Vector2.zero();

    // Solve spring constraint
    {
      final double cDot = _ax.dot(temp
            ..setFrom(vB)
            ..sub(vA)) +
          _sBx * wB -
          _sAx * wA;
      final double impulse =
          -_springMass * (cDot + _bias + _gamma * _springImpulse);
      _springImpulse += impulse;

      p.x = impulse * _ax.x;
      p.y = impulse * _ax.y;
      final double lA = impulse * _sAx;
      final double lB = impulse * _sBx;

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * lA;

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * lB;
    }

    // Solve rotational motor constraint
    {
      final double cDot = wB - wA - _motorSpeed;
      double impulse = -_motorMass * cDot;

      final double oldImpulse = _motorImpulse;
      final double maxImpulse = data.step.dt * _maxMotorTorque;
      _motorImpulse =
          (_motorImpulse + impulse).clamp(-maxImpulse, maxImpulse).toDouble();
      impulse = _motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve point to line constraint
    {
      final double cDot = _ay.dot(temp
            ..setFrom(vB)
            ..sub(vA)) +
          _sBy * wB -
          _sAy * wA;
      final double impulse = -_mass * cDot;
      _impulse += impulse;

      p.x = impulse * _ay.x;
      p.y = impulse * _ay.y;
      final double lA = impulse * _sAy;
      final double lB = impulse * _sBy;

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * lA;

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * lB;
    }

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(SolverData data) {
    final Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    final Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 temp = Vector2.zero();

    qA.setAngle(aA);
    qB.setAngle(aB);

    temp
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    rB.setFrom(Rot.mulVec2(qB, temp));

    d
      ..setFrom(cB)
      ..sub(cA)
      ..add(rB)
      ..sub(rA);

    final Vector2 ay = Vector2.copy(Rot.mulVec2(qA, _localYAxisA));
    final double sAy = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(ay);
    final double sBy = rB.cross(ay);

    final double c = d.dot(ay);

    final double k =
        _invMassA + _invMassB + _invIA * _sAy * _sAy + _invIB * _sBy * _sBy;

    final double impulse = k != 0.0 ? -c / k : 0.0;
    final Vector2 p = Vector2.zero();
    p.x = impulse * ay.x;
    p.y = impulse * ay.y;
    final double lA = impulse * sAy;
    final double lB = impulse * sBy;

    cA.x -= _invMassA * p.x;
    cA.y -= _invMassA * p.y;
    aA -= _invIA * lA;
    cB.x += _invMassB * p.x;
    cB.y += _invMassB * p.y;
    aB += _invIB * lB;

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return c.abs() <= settings.linearSlop;
  }
}
