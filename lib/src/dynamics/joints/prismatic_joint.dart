import 'dart:math';

import '../../../forge2d.dart';
import '../../settings.dart' as settings;

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
  @override
  final Vector2 localAnchorA;
  @override
  final Vector2 localAnchorB;
  final Vector2 localXAxisA;
  final Vector2 _localYAxisA;
  double _referenceAngle;
  double get referenceAngle => _referenceAngle;

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
  final Matrix3 _k = Matrix3.zero();
  double _motorMass =
      0.0; // effective mass for motor/limit translational constraint.

  PrismaticJoint(PrismaticJointDef def)
      : localAnchorA = Vector2.copy(def.localAnchorA),
        localAnchorB = Vector2.copy(def.localAnchorB),
        localXAxisA = Vector2.copy(def.localAxisA)..normalize(),
        _localYAxisA = Vector2.zero(),
        super(def) {
    localXAxisA.scaleOrthogonalInto(1.0, _localYAxisA);
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
  Vector2 getReactionForce(double invDt) {
    final temp = Vector2.zero();
    temp
      ..setFrom(_axis)
      ..scale(_motorImpulse + _impulse.z);
    final out = Vector2.copy(_perp)
      ..scale(_impulse.x)
      ..add(temp)
      ..scale(invDt);
    return out;
  }

  @override
  double getReactionTorque(double invDt) {
    return invDt * _impulse.y;
  }

  /// Get the current joint translation, usually in meters.
  double getJointSpeed() {
    final bA = bodyA;
    final bB = bodyB;

    final temp = Vector2.zero();
    final rA = Vector2.zero();
    final rB = Vector2.zero();
    final p1 = Vector2.zero();
    final p2 = Vector2.zero();
    final d = Vector2.zero();
    final axis = Vector2.zero();
    final temp2 = Vector2.zero();
    final temp3 = Vector2.zero();

    temp
      ..setFrom(localAnchorA)
      ..sub(bA.sweep.localCenter);
    rA.setFrom(Rot.mulVec2(bA.transform.q, temp));

    temp
      ..setFrom(localAnchorB)
      ..sub(bB.sweep.localCenter);
    rB.setFrom(Rot.mulVec2(bB.transform.q, temp));

    p1
      ..setFrom(bA.sweep.c)
      ..add(rA);
    p2
      ..setFrom(bB.sweep.c)
      ..add(rB);

    d
      ..setFrom(p2)
      ..sub(p1);
    axis.setFrom(Rot.mulVec2(bA.transform.q, localXAxisA));

    final vA = bA.linearVelocity;
    final vB = bB.linearVelocity;
    final wA = bA.angularVelocity;
    final wB = bB.angularVelocity;

    axis.scaleOrthogonalInto(wA, temp);
    rB.scaleOrthogonalInto(wB, temp2);
    rA.scaleOrthogonalInto(wA, temp3);

    temp2
      ..add(vB)
      ..sub(vA)
      ..sub(temp3);
    final speed = d.dot(temp) + axis.dot(temp2);

    return speed;
  }

  double getJointTranslation() {
    final Vector2 pA = Vector2.zero(),
        pB = Vector2.zero(),
        axis = Vector2.zero();
    pA.setFrom(bodyA.getWorldPoint(localAnchorA));
    pB.setFrom(bodyB.getWorldPoint(localAnchorB));
    axis.setFrom(bodyA.getWorldVector(localXAxisA));
    pB.sub(pA);
    final translation = pB.dot(axis);
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
      bodyA.setAwake(true);
      bodyB.setAwake(true);
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
      bodyA.setAwake(true);
      bodyB.setAwake(true);
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
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _enableMotor = flag;
  }

  /// Set the motor speed, usually in meters per second.
  ///
  /// @param speed
  void setMotorSpeed(double speed) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
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
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _maxMotorForce = force;
  }

  /// Get the current motor force, usually in N.
  ///
  /// @param invDt
  /// @return
  double getMotorForce(double invDt) {
    return _motorImpulse * invDt;
  }

  double getMaxMotorForce() {
    return _maxMotorForce;
  }

  double getReferenceAngle() {
    return _referenceAngle;
  }

  Vector2 getLocalAxisA() {
    return localXAxisA;
  }

  @override
  void initVelocityConstraints(final SolverData data) {
    _indexA = bodyA.islandIndex;
    _indexB = bodyB.islandIndex;
    _localCenterA.setFrom(bodyA.sweep.localCenter);
    _localCenterB.setFrom(bodyB.sweep.localCenter);
    _invMassA = bodyA.inverseMass;
    _invMassB = bodyB.inverseMass;
    _invIA = bodyA.inverseInertia;
    _invIB = bodyB.inverseInertia;

    final cA = data.positions[_indexA].c;
    final aA = data.positions[_indexA].a;
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;

    final cB = data.positions[_indexB].c;
    final aB = data.positions[_indexB].a;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final qA = Rot();
    final qB = Rot();
    final d = Vector2.zero();
    final temp = Vector2.zero();
    final rA = Vector2.zero();
    final rB = Vector2.zero();

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

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    // Compute motor Jacobian and effective mass.
    {
      _axis.setFrom(Rot.mulVec2(qA, localXAxisA));
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

      final k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
      final k12 = iA * _s1 + iB * _s2;
      final k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
      var k22 = iA + iB;
      if (k22 == 0.0) {
        // For bodies with fixed rotation.
        k22 = 1.0;
      }
      final k23 = iA * _a1 + iB * _a2;
      final k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

      _k.setValues(k11, k12, k13, k12, k22, k23, k13, k23, k33);
    }

    // Compute motor and limit terms.
    if (_enableLimit) {
      final jointTranslation = _axis.dot(d);
      if ((_upperTranslation - _lowerTranslation).abs() <
          2.0 * settings.linearSlop) {
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

      final p = Vector2.zero();
      temp
        ..setFrom(_axis)
        ..scale(_motorImpulse + _impulse.z);
      p
        ..setFrom(_perp)
        ..scale(_impulse.x)
        ..add(temp);

      final lA =
          _impulse.x * _s1 + _impulse.y + (_motorImpulse + _impulse.z) * _a1;
      final lB =
          _impulse.x * _s2 + _impulse.y + (_motorImpulse + _impulse.z) * _a2;

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * lA;

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * lB;
    } else {
      _impulse.setZero();
      _motorImpulse = 0.0;
    }

    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;
  }

  @override
  void solveVelocityConstraints(final SolverData data) {
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final temp = Vector2.zero();

    // Solve linear motor constraint.
    if (_enableMotor && _limitState != LimitState.EQUAL) {
      temp
        ..setFrom(vB)
        ..sub(vA);
      final cDot = _axis.dot(temp) + _a2 * wB - _a1 * wA;
      var impulse = _motorMass * (_motorSpeed - cDot);
      final oldImpulse = _motorImpulse;
      final maxImpulse = data.step.dt * _maxMotorForce;
      _motorImpulse =
          (_motorImpulse + impulse).clamp(-maxImpulse, maxImpulse).toDouble();
      impulse = _motorImpulse - oldImpulse;

      final P = Vector2.zero();
      P
        ..setFrom(_axis)
        ..scale(impulse);
      final lA = impulse * _a1;
      final lB = impulse * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * lA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * lB;
    }

    final cDot1 = Vector2.zero();
    temp
      ..setFrom(vB)
      ..sub(vA);
    cDot1.x = _perp.dot(temp) + _s2 * wB - _s1 * wA;
    cDot1.y = wB - wA;

    if (_enableLimit && _limitState != LimitState.INACTIVE) {
      // Solve prismatic and limit constraint in block form.
      final cDot2 = _axis.dot(vB - vA) + _a2 * wB - _a1 * wA;
      final cDot = Vector3(cDot1.x, cDot1.y, cDot2);
      final f1 = Vector3.zero();
      final df = Vector3.zero();

      f1.setFrom(_impulse);
      Matrix3.solve(_k, df, cDot..negate());

      // Cdot.negateLocal(); not used anymore
      _impulse.add(df);

      if (_limitState == LimitState.AT_LOWER) {
        _impulse.z = max(_impulse.z, 0.0);
      } else if (_limitState == LimitState.AT_UPPER) {
        _impulse.z = min(_impulse.z, 0.0);
      }

      // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
      // f1(1:2)
      final b = Vector2.zero();
      final f2r = Vector2.zero();

      temp
        ..setValues(_k.entry(0, 2), _k.entry(1, 2))
        ..scale(_impulse.z - f1.z);
      b
        ..setFrom(cDot1)
        ..negate()
        ..sub(temp);

      Matrix3.solve2(_k, f2r, b);
      f2r.add(Vector2(f1.x, f1.y));
      _impulse.x = f2r.x;
      _impulse.y = f2r.y;

      df
        ..setFrom(_impulse)
        ..sub(f1);

      final P = Vector2.zero();
      temp
        ..setFrom(_axis)
        ..scale(df.z);
      P
        ..setFrom(_perp)
        ..scale(df.x)
        ..add(temp);

      final lA = df.x * _s1 + df.y + df.z * _a1;
      final lB = df.x * _s2 + df.y + df.z * _a2;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * lA;

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * lB;
    } else {
      // Limit is inactive, just solve the prismatic constraint in block form.
      final df = Vector2.zero();
      Matrix3.solve2(_k, df, cDot1..negate());
      cDot1.negate();

      _impulse.x += df.x;
      _impulse.y += df.y;

      final p = Vector2.zero();
      p
        ..setFrom(_perp)
        ..scale(df.x);
      final lA = df.x * _s1 + df.y;
      final lB = df.x * _s2 + df.y;

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * lA;

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * lB;
    }

    // data.velocities[_indexA].v.set(vA);
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(final SolverData data) {
    final qA = Rot();
    final qB = Rot();
    final rA = Vector2.zero();
    final rB = Vector2.zero();
    final d = Vector2.zero();
    final axis = Vector2.zero();
    final perp = Vector2.zero();
    final temp = Vector2.zero();
    final c1 = Vector2.zero();

    final impulse = Vector3.zero();

    final cA = data.positions[_indexA].c;
    var aA = data.positions[_indexA].a;
    final cB = data.positions[_indexB].c;
    var aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

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

    axis.setFrom(Rot.mulVec2(qA, localXAxisA));
    final a1 = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(axis);
    final a2 = rB.cross(axis);
    perp.setFrom(Rot.mulVec2(qA, _localYAxisA));

    final s1 = (temp
          ..setFrom(d)
          ..add(rA))
        .cross(perp);
    final s2 = rB.cross(perp);

    c1.x = perp.dot(d);
    c1.y = aB - aA - _referenceAngle;

    var linearError = c1.x.abs();
    final angularError = c1.y.abs();

    var active = false;
    var c2 = 0.0;
    if (_enableLimit) {
      final translation = axis.dot(d);
      if ((_upperTranslation - _lowerTranslation).abs() <
          2.0 * settings.linearSlop) {
        // Prevent large angular corrections
        c2 = translation
            .clamp(-settings.maxLinearCorrection, settings.maxLinearCorrection)
            .toDouble();
        linearError = max(linearError, translation.abs());
        active = true;
      } else if (translation <= _lowerTranslation) {
        // Prevent large linear corrections and allow some slop.
        c2 = (translation - _lowerTranslation + settings.linearSlop)
            .clamp(-settings.maxLinearCorrection, 0.0)
            .toDouble();
        linearError = max(linearError, _lowerTranslation - translation);
        active = true;
      } else if (translation >= _upperTranslation) {
        // Prevent large linear corrections and allow some slop.
        c2 = (translation - _upperTranslation - settings.linearSlop)
            .clamp(0.0, settings.maxLinearCorrection)
            .toDouble();
        linearError = max(linearError, translation - _upperTranslation);
        active = true;
      }
    }

    if (active) {
      final k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      final k12 = iA * s1 + iB * s2;
      final k13 = iA * s1 * a1 + iB * s2 * a2;
      var k22 = iA + iB;
      if (k22 == 0.0) {
        // For fixed rotation
        k22 = 1.0;
      }
      final k23 = iA * a1 + iB * a2;
      final k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

      final k = Matrix3.zero();
      k.setValues(k11, k12, k13, k12, k22, k23, k13, k23, k33);

      final c = Vector3.zero();
      c.x = c1.x;
      c.y = c1.y;
      c.z = c2;

      Matrix3.solve(k, impulse, c..negate());
    } else {
      final k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
      final k12 = iA * s1 + iB * s2;
      final k22 = iA + iB == 0.0 ? 1.0 : iA + iB;

      final k = Matrix2.zero();

      k.setValues(k11, k12, k12, k22);
      // temp is impulse1
      Matrix2.solve(k, temp, c1..negate());
      c1.negate();

      impulse.x = temp.x;
      impulse.y = temp.y;
      impulse.z = 0.0;
    }

    final pX = impulse.x * perp.x + impulse.z * axis.x;
    final pY = impulse.x * perp.y + impulse.z * axis.y;
    final lA = impulse.x * s1 + impulse.y + impulse.z * a1;
    final lB = impulse.x * s2 + impulse.y + impulse.z * a2;

    cA.x -= mA * pX;
    cA.y -= mA * pY;
    aA -= iA * lA;
    cB.x += mB * pX;
    cB.y += mB * pY;
    aB += iB * lB;

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return linearError <= settings.linearSlop &&
        angularError <= settings.angularSlop;
  }
}
