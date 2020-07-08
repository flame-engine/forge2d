part of box2d;

//Linear constraint (point-to-line)
//d = p2 - p1 = x2 + r2 - x1 - r1
//C = dot(perp, d)
//Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//   = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
//J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
//Angular constraint
//C = a2 - a1 + a_initial
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//
//K = J * invM * JT
//
//J = [-a -s1 a s2]
//  [0  -1  0  1]
//a = perp
//s1 = cross(d + r1, a) = cross(p2 - x1, a)
//s2 = cross(r2, a) = cross(p2 - x2, a)

//Motor/Limit linear constraint
//C = dot(ax1, d)
//Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
//J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

//Block Solver
//We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
//when the mass has poor distribution (leading to large torques about the joint anchor points).
//
//The Jacobian has 3 rows:
//J = [-uT -s1 uT s2] // linear
//  [0   -1   0  1] // angular
//  [-vT -a1 vT a2] // limit
//
//u = perp
//v = axis
//s1 = cross(d + r1, u), s2 = cross(r2, u)
//a1 = cross(d + r1, v), a2 = cross(r2, v)

//M * (v2 - v1) = JT * df
//J * v2 = bias
//
//v2 = v1 + invM * JT * df
//J * (v1 + invM * JT * df) = bias
//K * df = bias - J * v1 = -Cdot
//K = J * invM * JT
//Cdot = J * v1 - bias
//
//Now solve for f2.
//df = f2 - f1
//K * (f2 - f1) = -Cdot
//f2 = invK * (-Cdot) + f1
//
//Clamp accumulated limit impulse.
//lower: f2(3) = max(f2(3), 0)
//upper: f2(3) = min(f2(3), 0)
//
//Solve for correct f2(1:2)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                    = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
//K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
//f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
//Now compute impulse to be applied:
//df = f2 - f1

/// A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in
/// bodyA. Relative rotation is prevented. You can use a joint limit to restrict the range of motion
/// and a joint motor to drive the motion or to model joint friction.
class PrismaticJoint extends Joint {
  // Solver shared
  final Vector2 localAnchorA;
  final Vector2 localAnchorB;
  final Vector2 _localXAxisA;
  final Vector2 _localYAxisA;
  double _referenceAngle;

  // TODO(srdjan): Make fields below private.
  final Vector3 _impulse = Vector3.zero();
  double _motorImpulse = 0.0;
  double _lowerTranslation = 0.0;
  double _upperTranslation = 0.0;
  double _maxMotorForce = 0.0;
  double _motorSpeed = 0.0;
  bool _enableLimit = false;
  bool _enableMotor = false;
  LimitState _limitState;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  final Vector2 _axis = Vector2.zero();
  final Vector2 _perp = Vector2.zero();
  double _s1 = 0.0, _s2 = 0.0;
  double _a1 = 0.0, _a2 = 0.0;
  final Matrix3 _K = Matrix3.zero();
  double _motorMass =
      0.0; // effective mass for motor/limit translational constraint.

  PrismaticJoint(PrismaticJointDef def)
      : localAnchorA = Vector2.copy(def.localAnchorA),
        localAnchorB = Vector2.copy(def.localAnchorB),
        _localXAxisA = Vector2.copy(def.localAxisA)..normalize(),
        _localYAxisA = Vector2.zero(),
        super(def) {
    _localXAxisA.scaleOrthogonalInto(1.0, _localYAxisA);
    _referenceAngle = def.referenceAngle;

    _lowerTranslation = def.lowerTranslation;
    _upperTranslation = def.upperTranslation;
    _maxMotorForce = def.maxMotorForce;
    _motorSpeed = def.motorSpeed;
    _enableLimit = def.enableLimit;
    _enableMotor = def.enableMotor;
    _limitState = LimitState.INACTIVE;
  }

  @override
  Vector2 getReactionForce(double inv_dt) {
    Vector2 temp = Vector2.zero();
    temp
      ..setFrom(_axis)
      ..scale(_motorImpulse + _impulse.z);
    Vector2 out = Vector2.copy(_perp)
      ..scale(_impulse.x)
      ..add(temp)
      ..scale(inv_dt);
    return out;
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * _impulse.y;
  }

  /// Get the current joint translation, usually in meters.
  double getJointSpeed() {
    Body bA = _bodyA;
    Body bB = _bodyB;

    Vector2 temp = Vector2.zero();
    Vector2 rA = Vector2.zero();
    Vector2 rB = Vector2.zero();
    Vector2 p1 = Vector2.zero();
    Vector2 p2 = Vector2.zero();
    Vector2 d = Vector2.zero();
    Vector2 axis = Vector2.zero();
    Vector2 temp2 = Vector2.zero();
    Vector2 temp3 = Vector2.zero();

    temp
      ..setFrom(localAnchorA)
      ..sub(bA._sweep.localCenter);
    rA.setFrom(Rot.mulVec2(bA._transform.q, temp));

    temp
      ..setFrom(localAnchorB)
      ..sub(bB._sweep.localCenter);
    rB.setFrom(Rot.mulVec2(bB._transform.q, temp));

    p1
      ..setFrom(bA._sweep.c)
      ..add(rA);
    p2
      ..setFrom(bB._sweep.c)
      ..add(rB);

    d
      ..setFrom(p2)
      ..sub(p1);
    axis.setFrom(Rot.mulVec2(bA._transform.q, _localXAxisA));

    Vector2 vA = bA._linearVelocity;
    Vector2 vB = bB._linearVelocity;
    double wA = bA._angularVelocity;
    double wB = bB._angularVelocity;

    axis.scaleOrthogonalInto(wA, temp);
    rB.scaleOrthogonalInto(wB, temp2);
    rA.scaleOrthogonalInto(wA, temp3);

    temp2
      ..add(vB)
      ..sub(vA)
      ..sub(temp3);
    double speed = d.dot(temp) + axis.dot(temp2);

    return speed;
  }

  double getJointTranslation() {
    Vector2 pA = Vector2.zero(), pB = Vector2.zero(), axis = Vector2.zero();
    pA.setFrom(_bodyA.getWorldPoint(localAnchorA));
    pB.setFrom(_bodyB.getWorldPoint(localAnchorB));
    axis.setFrom(_bodyA.getWorldVector(_localXAxisA));
    pB.sub(pA);
    double translation = pB.dot(axis);
    return translation;
  }

  /// Is the joint limit enabled?
  ///
  /// @return
  bool isLimitEnabled() {
    return _enableLimit;
  }

  /// Enable/disable the joint limit.
  ///
  /// @param flag
  void enableLimit(bool flag) {
    if (flag != _enableLimit) {
      _bodyA.setAwake(true);
      _bodyB.setAwake(true);
      _enableLimit = flag;
      _impulse.z = 0.0;
    }
  }

  /// Get the lower joint limit, usually in meters.
  ///
  /// @return
  double getLowerLimit() {
    return _lowerTranslation;
  }

  /// Get the upper joint limit, usually in meters.
  ///
  /// @return
  double getUpperLimit() {
    return _upperTranslation;
  }

  /// Set the joint limits, usually in meters.
  ///
  /// @param lower
  /// @param upper
  void setLimits(double lower, double upper) {
    assert(lower <= upper);
    if (lower != _lowerTranslation || upper != _upperTranslation) {
      _bodyA.setAwake(true);
      _bodyB.setAwake(true);
      _lowerTranslation = lower;
      _upperTranslation = upper;
      _impulse.z = 0.0;
    }
  }

  /// Is the joint motor enabled?
  ///
  /// @return
  bool isMotorEnabled() {
    return _enableMotor;
  }

  /// Enable/disable the joint motor.
  ///
  /// @param flag
  void enableMotor(bool flag) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _enableMotor = flag;
  }

  /// Set the motor speed, usually in meters per second.
  ///
  /// @param speed
  void setMotorSpeed(double speed) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _motorSpeed = speed;
  }

  /// Get the motor speed, usually in meters per second.
  ///
  /// @return
  double getMotorSpeed() {
    return _motorSpeed;
  }

  /// Set the maximum motor force, usually in N.
  ///
  /// @param force
  void setMaxMotorForce(double force) {
    _bodyA.setAwake(true);
    _bodyB.setAwake(true);
    _maxMotorForce = force;
  }

  /// Get the current motor force, usually in N.
  ///
  /// @param inv_dt
  /// @return
  double getMotorForce(double inv_dt) {
    return _motorImpulse * inv_dt;
  }

  double getMaxMotorForce() {
    return _maxMotorForce;
  }

  double getReferenceAngle() {
    return _referenceAngle;
  }

  Vector2 getLocalAxisA() {
    return _localXAxisA;
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

    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 d = Vector2.zero();
    final Vector2 temp = Vector2.zero();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    d
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    rA.setFrom(Rot.mulVec2(qA, d));
    d
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    rB.setFrom(Rot.mulVec2(qB, d));
    d
      ..setFrom(cB)
      ..sub(cA)
      ..add(rB)
      ..sub(rA);

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    // Compute motor Jacobian and effective mass.
    {
      _axis.setFrom(Rot.mulVec2(qA, _localXAxisA));
      temp
        ..setFrom(d)
        ..add(rA);
      _a1 = temp.cross(_axis);
      _a2 = rB.cross(_axis);

      _motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
      if (_motorMass > 0.0) {
        _motorMass = 1.0 / _motorMass;
      }
    }

    // Prismatic constraint.
    {
      _perp.setFrom(Rot.mulVec2(qA, _localYAxisA));

      temp
        ..setFrom(d)
        ..add(rA);
      _s1 = temp.cross(_perp);
      _s2 = rB.cross(_perp);

      double k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
      double k12 = iA * _s1 + iB * _s2;
      double k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        // For bodies with fixed rotation.
        k22 = 1.0;
      }
      double k23 = iA * _a1 + iB * _a2;
      double k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

      _K.setValues(k11, k12, k13, k12, k22, k23, k13, k23, k33);
    }

    // Compute motor and limit terms.
    if (_enableLimit) {
      double jointTranslation = _axis.dot(d);
      if ((_upperTranslation - _lowerTranslation).abs() <
          2.0 * Settings.linearSlop) {
        _limitState = LimitState.EQUAL;
      } else if (jointTranslation <= _lowerTranslation) {
        if (_limitState != LimitState.AT_LOWER) {
          _limitState = LimitState.AT_LOWER;
          _impulse.z = 0.0;
        }
      } else if (jointTranslation >= _upperTranslation) {
        if (_limitState != LimitState.AT_UPPER) {
          _limitState = LimitState.AT_UPPER;
          _impulse.z = 0.0;
        }
      } else {
        _limitState = LimitState.INACTIVE;
        _impulse.z = 0.0;
      }
    } else {
      _limitState = LimitState.INACTIVE;
      _impulse.z = 0.0;
    }

    if (_enableMotor == false) {
      _motorImpulse = 0.0;
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      _impulse.scale(data.step.dtRatio);
      _motorImpulse *= data.step.dtRatio;

      final Vector2 P = Vector2.zero();
      temp
        ..setFrom(_axis)
        ..scale(_motorImpulse + _impulse.z);
      P
        ..setFrom(_perp)
        ..scale(_impulse.x)
        ..add(temp);

      double LA =
          _impulse.x * _s1 + _impulse.y + (_motorImpulse + _impulse.z) * _a1;
      double LB =
          _impulse.x * _s2 + _impulse.y + (_motorImpulse + _impulse.z) * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
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

    final Vector2 temp = Vector2.zero();

    // Solve linear motor constraint.
    if (_enableMotor && _limitState != LimitState.EQUAL) {
      temp
        ..setFrom(vB)
        ..sub(vA);
      double Cdot = _axis.dot(temp) + _a2 * wB - _a1 * wA;
      double impulse = _motorMass * (_motorSpeed - Cdot);
      double oldImpulse = _motorImpulse;
      double maxImpulse = data.step.dt * _maxMotorForce;
      _motorImpulse = MathUtils.clampDouble(
          _motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = _motorImpulse - oldImpulse;

      final Vector2 P = Vector2.zero();
      P
        ..setFrom(_axis)
        ..scale(impulse);
      double LA = impulse * _a1;
      double LB = impulse * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }

    final Vector2 Cdot1 = Vector2.zero();
    temp
      ..setFrom(vB)
      ..sub(vA);
    Cdot1.x = _perp.dot(temp) + _s2 * wB - _s1 * wA;
    Cdot1.y = wB - wA;

    if (_enableLimit && _limitState != LimitState.INACTIVE) {
      // Solve prismatic and limit constraint in block form.
      double Cdot2;
      temp
        ..setFrom(vB)
        ..sub(vA);
      Cdot2 = _axis.dot(temp) + _a2 * wB - _a1 * wA;

      final Vector3 Cdot = Vector3.zero();
      Cdot.setValues(Cdot1.x, Cdot1.y, Cdot2);

      final Vector3 f1 = Vector3.zero();
      final Vector3 df = Vector3.zero();

      f1.setFrom(_impulse);
      Matrix3.solve(_K, df, Cdot..negate());

      // Cdot.negateLocal(); not used anymore
      _impulse.add(df);

      if (_limitState == LimitState.AT_LOWER) {
        _impulse.z = Math.max(_impulse.z, 0.0);
      } else if (_limitState == LimitState.AT_UPPER) {
        _impulse.z = Math.min(_impulse.z, 0.0);
      }

      // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
      // f1(1:2)
      final Vector2 b = Vector2.zero();
      final Vector2 f2r = Vector2.zero();

      temp
        ..setValues(_K.entry(0, 2), _K.entry(1, 2))
        ..scale(_impulse.z - f1.z);
      b
        ..setFrom(Cdot1)
        ..negate()
        ..sub(temp);

      Matrix3.solve2(_K, f2r, b);
      f2r.add(Vector2(f1.x, f1.y));
      _impulse.x = f2r.x;
      _impulse.y = f2r.y;

      df
        ..setFrom(_impulse)
        ..sub(f1);

      final Vector2 P = Vector2.zero();
      temp
        ..setFrom(_axis)
        ..scale(df.z);
      P
        ..setFrom(_perp)
        ..scale(df.x)
        ..add(temp);

      double LA = df.x * _s1 + df.y + df.z * _a1;
      double LB = df.x * _s2 + df.y + df.z * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    } else {
      // Limit is inactive, just solve the prismatic constraint in block form.
      final Vector2 df = Vector2.zero();
      Matrix3.solve2(_K, df, Cdot1..negate());
      Cdot1.negate();

      _impulse.x += df.x;
      _impulse.y += df.y;

      final Vector2 P = Vector2.zero();
      P
        ..setFrom(_perp)
        ..scale(df.x);
      double LA = df.x * _s1 + df.y;
      double LB = df.x * _s2 + df.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * LA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * LB;
    }

    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();
    final Vector2 d = Vector2.zero();
    final Vector2 axis = Vector2.zero();
    final Vector2 perp = Vector2.zero();
    final Vector2 temp = Vector2.zero();
    final Vector2 C1 = Vector2.zero();

    final Vector3 impulse = Vector3.zero();

    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    // Compute fresh Jacobians
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

    axis.setFrom(Rot.mulVec2(qA, _localXAxisA));
    double a1 = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(axis);
    double a2 = rB.cross(axis);
    perp.setFrom(Rot.mulVec2(qA, _localYAxisA));

    double s1 = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(perp);
    double s2 = rB.cross(perp);

    C1.x = perp.dot(d);
    C1.y = aB - aA - _referenceAngle;

    double linearError = C1.x.abs();
    double angularError = C1.y.abs();

    bool active = false;
    double C2 = 0.0;
    if (_enableLimit) {
      double translation = axis.dot(d);
      if ((_upperTranslation - _lowerTranslation).abs() <
          2.0 * Settings.linearSlop) {
        // Prevent large angular corrections
        C2 = MathUtils.clampDouble(translation, -Settings.maxLinearCorrection,
            Settings.maxLinearCorrection);
        linearError = Math.max(linearError, translation.abs());
        active = true;
      } else if (translation <= _lowerTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 = MathUtils.clampDouble(
            translation - _lowerTranslation + Settings.linearSlop,
            -Settings.maxLinearCorrection,
            0.0);
        linearError = Math.max(linearError, _lowerTranslation - translation);
        active = true;
      } else if (translation >= _upperTranslation) {
        // Prevent large linear corrections and allow some slop.
        C2 = MathUtils.clampDouble(
            translation - _upperTranslation - Settings.linearSlop,
            0.0,
            Settings.maxLinearCorrection);
        linearError = Math.max(linearError, translation - _upperTranslation);
        active = true;
      }
    }

    if (active) {
      double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      double k12 = iA * s1 + iB * s2;
      double k13 = iA * s1 * a1 + iB * s2 * a2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        // For fixed rotation
        k22 = 1.0;
      }
      double k23 = iA * a1 + iB * a2;
      double k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

      final Matrix3 K = Matrix3.zero();
      K.setValues(k11, k12, k13, k12, k22, k23, k13, k23, k33);

      final Vector3 C = Vector3.zero();
      C.x = C1.x;
      C.y = C1.y;
      C.z = C2;

      Matrix3.solve(K, impulse, C..negate());
    } else {
      double k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      double k12 = iA * s1 + iB * s2;
      double k22 = iA + iB;
      if (k22 == 0.0) {
        k22 = 1.0;
      }

      final Matrix2 K = Matrix2.zero();

      K.setValues(k11, k12, k12, k22);
      // temp is impulse1
      Matrix2.solve(K, temp, C1..negate());
      C1.negate();

      impulse.x = temp.x;
      impulse.y = temp.y;
      impulse.z = 0.0;
    }

    double Px = impulse.x * perp.x + impulse.z * axis.x;
    double Py = impulse.x * perp.y + impulse.z * axis.y;
    double LA = impulse.x * s1 + impulse.y + impulse.z * a1;
    double LB = impulse.x * s2 + impulse.y + impulse.z * a2;

    cA.x -= mA * Px;
    cA.y -= mA * Py;
    aA -= iA * LA;
    cB.x += mB * Px;
    cB.y += mB * Py;
    aB += iB * LB;

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return linearError <= Settings.linearSlop &&
        angularError <= Settings.angularSlop;
  }
}
