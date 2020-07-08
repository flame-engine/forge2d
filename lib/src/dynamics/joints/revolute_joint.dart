part of box2d;
//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Motor constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/// A revolute joint constrains two bodies to share a common point while they are free to rotate
/// about the point. The relative rotation about the shared point is the joint angle. You can limit
/// the relative rotation with a joint limit that specifies a lower and upper angle. You can use a
/// motor to drive the relative rotation about the shared point. A maximum motor torque is provided
/// so that infinite forces are not generated.
class RevoluteJoint extends Joint {
  // Solver shared
  final Vector2 localAnchorA = Vector2.zero();
  final Vector2 localAnchorB = Vector2.zero();
  final Vector3 _impulse = Vector3.zero();
  double _motorImpulse = 0.0;

  bool _enableMotor = false;
  double _maxMotorTorque = 0.0;
  double _motorSpeed = 0.0;

  bool _enableLimit = false;
  double _referenceAngle = 0.0;
  double _lowerAngle = 0.0;
  double _upperAngle = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _rA = Vector2.zero();
  final Vector2 _rB = Vector2.zero();
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  final Matrix3 _mass =
      Matrix3.zero(); // effective mass for point-to-point constraint.
  double _motorMass = 0.0; // effective mass for motor/limit angular constraint.
  LimitState _limitState = LimitState.INACTIVE;

  RevoluteJoint(RevoluteJointDef def) : super(def) {
    localAnchorA.setFrom(def.localAnchorA);
    localAnchorB.setFrom(def.localAnchorB);
    _referenceAngle = def.referenceAngle;

    _lowerAngle = def.lowerAngle;
    _upperAngle = def.upperAngle;
    _maxMotorTorque = def.maxMotorTorque;
    _motorSpeed = def.motorSpeed;
    _enableLimit = def.enableLimit;
    _enableMotor = def.enableMotor;
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

    // Vec2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    // Vec2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
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
    _rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    _rB.setFrom(Rot.mulVec2(qB, temp));

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    bool fixedRotation = (iA + iB == 0.0);

    double ex_x = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
    double ey_x = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
    double ez_x = -_rA.y * iA - _rB.y * iB;
    double ex_y = _mass.entry(0, 1);
    double ey_y = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
    double ez_y = _rA.x * iA + _rB.x * iB;
    double ex_z = _mass.entry(0, 2);
    double ey_z = _mass.entry(1, 2);
    double ez_z = iA + iB;

    _mass.setValues(ex_x, ex_y, ex_z, ey_x, ey_y, ey_z, ez_x, ez_y, ez_z);

    _motorMass = iA + iB;
    if (_motorMass > 0.0) {
      _motorMass = 1.0 / _motorMass;
    }

    if (_enableMotor == false || fixedRotation) {
      _motorImpulse = 0.0;
    }

    if (_enableLimit && fixedRotation == false) {
      double jointAngle = aB - aA - _referenceAngle;
      if ((_upperAngle - _lowerAngle).abs() < 2.0 * Settings.angularSlop) {
        _limitState = LimitState.EQUAL;
      } else if (jointAngle <= _lowerAngle) {
        if (_limitState != LimitState.AT_LOWER) {
          _impulse.z = 0.0;
        }
        _limitState = LimitState.AT_LOWER;
      } else if (jointAngle >= _upperAngle) {
        if (_limitState != LimitState.AT_UPPER) {
          _impulse.z = 0.0;
        }
        _limitState = LimitState.AT_UPPER;
      } else {
        _limitState = LimitState.INACTIVE;
        _impulse.z = 0.0;
      }
    } else {
      _limitState = LimitState.INACTIVE;
    }

    if (data.step.warmStarting) {
      final Vector2 P = Vector2.zero();
      // Scale impulses to support a variable time step.
      _impulse.x *= data.step.dtRatio;
      _impulse.y *= data.step.dtRatio;
      _motorImpulse *= data.step.dtRatio;

      P.x = _impulse.x;
      P.y = _impulse.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (_rA.cross(P) + _motorImpulse + _impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (_rB.cross(P) + _motorImpulse + _impulse.z);
    } else {
      _impulse.setZero();
      _motorImpulse = 0.0;
    }
    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    bool fixedRotation = (iA + iB == 0.0);

    // Solve motor constraint.
    if (_enableMotor &&
        _limitState != LimitState.EQUAL &&
        fixedRotation == false) {
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
    final Vector2 temp = Vector2.zero();

    // Solve limit constraint.
    if (_enableLimit &&
        _limitState != LimitState.INACTIVE &&
        fixedRotation == false) {
      final Vector2 Cdot1 = Vector2.zero();
      final Vector3 Cdot = Vector3.zero();

      // Solve point-to-point constraint
      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, Cdot1);
      Cdot1
        ..add(vB)
        ..sub(vA)
        ..sub(temp);
      double Cdot2 = wB - wA;
      Cdot.setValues(Cdot1.x, Cdot1.y, Cdot2);

      Vector3 impulse = Vector3.zero();
      Matrix3.solve(_mass, impulse, Cdot);
      impulse.negate();

      if (_limitState == LimitState.EQUAL) {
        _impulse.add(impulse);
      } else if (_limitState == LimitState.AT_LOWER) {
        double newImpulse = _impulse.z + impulse.z;
        if (newImpulse < 0.0) {
          final Vector2 rhs = Vector2.zero();
          rhs
            ..setValues(_mass.entry(0, 2), _mass.entry(1, 2))
            ..scale(_impulse.z)
            ..sub(Cdot1);
          Matrix3.solve2(_mass, temp, rhs);
          impulse.x = temp.x;
          impulse.y = temp.y;
          impulse.z = -_impulse.z;
          _impulse.x += temp.x;
          _impulse.y += temp.y;
          _impulse.z = 0.0;
        } else {
          _impulse.add(impulse);
        }
      } else if (_limitState == LimitState.AT_UPPER) {
        double newImpulse = _impulse.z + impulse.z;
        if (newImpulse > 0.0) {
          final Vector2 rhs = Vector2.zero();
          rhs
            ..setValues(_mass.entry(0, 2), _mass.entry(1, 2))
            ..scale(_impulse.z)
            ..sub(Cdot1);
          Matrix3.solve2(_mass, temp, rhs);
          impulse.x = temp.x;
          impulse.y = temp.y;
          impulse.z = -_impulse.z;
          _impulse.x += temp.x;
          _impulse.y += temp.y;
          _impulse.z = 0.0;
        } else {
          _impulse.add(impulse);
        }
      }
      final Vector2 P = Vector2.zero();

      P.setValues(impulse.x, impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (_rA.cross(P) + impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (_rB.cross(P) + impulse.z);
    } else {
      // Solve point-to-point constraint
      Vector2 Cdot = Vector2.zero();
      Vector2 impulse = Vector2.zero();

      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, Cdot);
      Cdot
        ..add(vB)
        ..sub(vA)
        ..sub(temp);
      Matrix3.solve2(_mass, impulse, Cdot..negate());

      _impulse.x += impulse.x;
      _impulse.y += impulse.y;

      vA.x -= mA * impulse.x;
      vA.y -= mA * impulse.y;
      wA -= iA * _rA.cross(impulse);

      vB.x += mB * impulse.x;
      vB.y += mB * impulse.y;
      wB += iB * _rB.cross(impulse);
    }

    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = Rot();
    final Rot qB = Rot();
    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    double angularError = 0.0;
    double positionError = 0.0;

    bool fixedRotation = (_invIA + _invIB == 0.0);

    // Solve angular limit constraint.
    if (_enableLimit &&
        _limitState != LimitState.INACTIVE &&
        fixedRotation == false) {
      double angle = aB - aA - _referenceAngle;
      double limitImpulse = 0.0;

      if (_limitState == LimitState.EQUAL) {
        // Prevent large angular corrections
        double C = MathUtils.clampDouble(angle - _lowerAngle,
            -Settings.maxAngularCorrection, Settings.maxAngularCorrection);
        limitImpulse = -_motorMass * C;
        angularError = C.abs();
      } else if (_limitState == LimitState.AT_LOWER) {
        double C = angle - _lowerAngle;
        angularError = -C;

        // Prevent large angular corrections and allow some slop.
        C = MathUtils.clampDouble(
            C + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0);
        limitImpulse = -_motorMass * C;
      } else if (_limitState == LimitState.AT_UPPER) {
        double C = angle - _upperAngle;
        angularError = C;

        // Prevent large angular corrections and allow some slop.
        C = MathUtils.clampDouble(
            C - Settings.angularSlop, 0.0, Settings.maxAngularCorrection);
        limitImpulse = -_motorMass * C;
      }

      aA -= _invIA * limitImpulse;
      aB += _invIB * limitImpulse;
    }
    // Solve point-to-point constraint.
    {
      qA.setAngle(aA);
      qB.setAngle(aB);

      final Vector2 rA = Vector2.zero();
      final Vector2 rB = Vector2.zero();
      final Vector2 temp = Vector2.zero();
      final Vector2 impulse = Vector2.zero();

      temp
        ..setFrom(localAnchorA)
        ..sub(_localCenterA);
      rA.setFrom(Rot.mulVec2(qA, temp));
      temp
        ..setFrom(localAnchorB)
        ..sub(_localCenterB);
      rB.setFrom(Rot.mulVec2(qB, temp));

      temp
        ..setFrom(cB)
        ..add(rB)
        ..sub(cA)
        ..sub(rA);
      positionError = temp.length;

      double mA = _invMassA, mB = _invMassB;
      double iA = _invIA, iB = _invIB;

      final Matrix2 K = Matrix2.zero();
      double a11 = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
      double a21 = -iA * rA.x * rA.y - iB * rB.x * rB.y;
      double a12 = a21;
      double a22 = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

      K.setValues(a11, a21, a12, a22);
      Matrix2.solve(K, impulse, temp);
      impulse.negate();

      cA.x -= mA * impulse.x;
      cA.y -= mA * impulse.y;
      aA -= iA * rA.cross(impulse);

      cB.x += mB * impulse.x;
      cB.y += mB * impulse.y;
      aB += iB * rB.cross(impulse);
    }
    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return positionError <= Settings.linearSlop &&
        angularError <= Settings.angularSlop;
  }

  double getReferenceAngle() {
    return _referenceAngle;
  }

  @override
  Vector2 getReactionForce(double inv_dt) {
    return Vector2(_impulse.x, _impulse.y)..scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * _impulse.z;
  }

  double getJointAngle() {
    final Body b1 = _bodyA;
    final Body b2 = _bodyB;
    return b2._sweep.a - b1._sweep.a - _referenceAngle;
  }

  double getJointSpeed() {
    final Body b1 = _bodyA;
    final Body b2 = _bodyB;
    return b2._angularVelocity - b1._angularVelocity;
  }

  bool isMotorEnabled() {
    return _enableMotor;
  }

  void enableMotor(bool flag) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _enableMotor = flag;
  }

  double getMotorTorque(double inv_dt) {
    return _motorImpulse * inv_dt;
  }

  void setMotorSpeed(final double speed) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _motorSpeed = speed;
  }

  void setMaxMotorTorque(final double torque) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _maxMotorTorque = torque;
  }

  double getMotorSpeed() {
    return _motorSpeed;
  }

  double getMaxMotorTorque() {
    return _maxMotorTorque;
  }

  bool isLimitEnabled() {
    return _enableLimit;
  }

  void enableLimit(final bool flag) {
    if (flag != _enableLimit) {
      _bodyA.setAwake(true);
      _bodyB.setAwake(true);
      _enableLimit = flag;
      _impulse.z = 0.0;
    }
  }

  double getLowerLimit() {
    return _lowerAngle;
  }

  double getUpperLimit() {
    return _upperAngle;
  }

  void setLimits(final double lower, final double upper) {
    assert(lower <= upper);
    if (lower != _lowerAngle || upper != _upperAngle) {
      _bodyA.setAwake(true);
      _bodyB.setAwake(true);
      _impulse.z = 0.0;
      _lowerAngle = lower;
      _upperAngle = upper;
    }
  }
}
