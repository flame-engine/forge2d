// TODO(any): Rewrite the setters instead of ignoring this lint.
// ignore_for_file: avoid_positional_boolean_parameters

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//    = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with a joint
/// limit that specifies a lower and upper angle. You can use a motor to drive
/// the relative rotation about the shared point. A maximum motor torque is
/// provided so that infinite forces are not generated.
class RevoluteJoint extends Joint {
  // Solver shared
  @override
  final Vector2 localAnchorA = Vector2.zero();
  @override
  final Vector2 localAnchorB = Vector2.zero();
  final Vector3 _impulse = Vector3.zero();
  double _motorImpulse = 0.0;

  bool _enableMotor = false;
  bool get motorEnabled => _enableMotor;
  double _maxMotorTorque = 0.0;
  double get maxMotorTorque => _maxMotorTorque;
  double _motorSpeed = 0.0;
  double get motorSpeed => _motorSpeed;

  bool _enableLimit = false;
  bool get limitEnabled => _enableLimit;
  double _referenceAngle = 0.0;
  double get referenceAngle => _referenceAngle;
  double _lowerAngle = 0.0;
  double _upperAngle = 0.0;
  double get lowerLimit => _lowerAngle;
  double get upperLimit => _upperAngle;

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
  LimitState _limitState = LimitState.inactive;

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

  @override
  void initVelocityConstraints(SolverData data) {
    _indexA = bodyA.islandIndex;
    _indexB = bodyB.islandIndex;
    _localCenterA.setFrom(bodyA.sweep.localCenter);
    _localCenterB.setFrom(bodyB.sweep.localCenter);
    _invMassA = bodyA.inverseMass;
    _invMassB = bodyB.inverseMass;
    _invIA = bodyA.inverseInertia;
    _invIB = bodyB.inverseInertia;

    // Vec2 cA = data.positions[_indexA].c;
    final aA = data.positions[_indexA].a;
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;

    // Vec2 cB = data.positions[_indexB].c;
    final aB = data.positions[_indexB].a;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;
    final qA = Rot();
    final qB = Rot();
    final temp = Vector2.zero();

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

    final mA = _invMassA;
    final mB = _invMassB;
    final iA = _invIA;
    final iB = _invIB;

    final fixedRotation = iA + iB == 0.0;

    final exX = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
    final eyX = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
    final ezX = -_rA.y * iA - _rB.y * iB;
    final exY = _mass.entry(0, 1);
    final eyY = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
    final ezY = _rA.x * iA + _rB.x * iB;
    final exZ = _mass.entry(0, 2);
    final eyZ = _mass.entry(1, 2);
    final ezZ = iA + iB;

    _mass.setValues(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);

    _motorMass = iA + iB;
    if (_motorMass > 0.0) {
      _motorMass = 1.0 / _motorMass;
    }

    if (_enableMotor == false || fixedRotation) {
      _motorImpulse = 0.0;
    }

    if (_enableLimit && fixedRotation == false) {
      final jointAngle = aB - aA - _referenceAngle;
      if ((_upperAngle - _lowerAngle).abs() < 2.0 * settings.angularSlop) {
        _limitState = LimitState.equal;
      } else if (jointAngle <= _lowerAngle) {
        if (_limitState != LimitState.atLower) {
          _impulse.z = 0.0;
        }
        _limitState = LimitState.atLower;
      } else if (jointAngle >= _upperAngle) {
        if (_limitState != LimitState.atUpper) {
          _impulse.z = 0.0;
        }
        _limitState = LimitState.atUpper;
      } else {
        _limitState = LimitState.inactive;
        _impulse.z = 0.0;
      }
    } else {
      _limitState = LimitState.inactive;
    }

    if (data.step.warmStarting) {
      final P = Vector2.zero();
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

  @override
  void solveVelocityConstraints(SolverData data) {
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final mA = _invMassA;
    final mB = _invMassB;
    final iA = _invIA;
    final iB = _invIB;

    final fixedRotation = iA + iB == 0.0;

    // Solve motor constraint.
    if (_enableMotor &&
        _limitState != LimitState.equal &&
        fixedRotation == false) {
      final cDot = wB - wA - _motorSpeed;
      var impulse = -_motorMass * cDot;
      final oldImpulse = _motorImpulse;
      final maxImpulse = data.step.dt * _maxMotorTorque;
      _motorImpulse = (_motorImpulse + impulse).clamp(-maxImpulse, maxImpulse);
      impulse = _motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }
    final temp = Vector2.zero();

    // Solve limit constraint.
    if (_enableLimit &&
        _limitState != LimitState.inactive &&
        fixedRotation == false) {
      final cDot1 = Vector2.zero();
      final cDot = Vector3.zero();

      // Solve point-to-point constraint
      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, cDot1);
      cDot1
        ..add(vB)
        ..sub(vA)
        ..sub(temp);
      final cDot2 = wB - wA;
      cDot.setValues(cDot1.x, cDot1.y, cDot2);

      final impulse = Vector3.zero();
      Matrix3.solve(_mass, impulse, cDot);
      impulse.negate();

      if (_limitState == LimitState.equal) {
        _impulse.add(impulse);
      } else if (_limitState == LimitState.atLower) {
        final newImpulse = _impulse.z + impulse.z;
        if (newImpulse < 0.0) {
          final rhs = Vector2.zero();
          rhs
            ..setValues(_mass.entry(0, 2), _mass.entry(1, 2))
            ..scale(_impulse.z)
            ..sub(cDot1);
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
      } else if (_limitState == LimitState.atUpper) {
        final newImpulse = _impulse.z + impulse.z;
        if (newImpulse > 0.0) {
          final rhs = Vector2.zero();
          rhs
            ..setValues(_mass.entry(0, 2), _mass.entry(1, 2))
            ..scale(_impulse.z)
            ..sub(cDot1);
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
      final p = Vector2.zero();

      p.setValues(impulse.x, impulse.y);

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * (_rA.cross(p) + impulse.z);

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * (_rB.cross(p) + impulse.z);
    } else {
      // Solve point-to-point constraint
      final cDot = Vector2.zero();
      final impulse = Vector2.zero();

      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, cDot);
      cDot
        ..add(vB)
        ..sub(vA)
        ..sub(temp);
      Matrix3.solve2(_mass, impulse, cDot..negate());

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

  @override
  bool solvePositionConstraints(SolverData data) {
    final qA = Rot();
    final qB = Rot();
    final cA = data.positions[_indexA].c;
    var aA = data.positions[_indexA].a;
    final cB = data.positions[_indexB].c;
    var aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    var angularError = 0.0;
    var positionError = 0.0;

    final fixedRotation = _invIA + _invIB == 0.0;

    // Solve angular limit constraint.
    if (_enableLimit &&
        _limitState != LimitState.inactive &&
        fixedRotation == false) {
      final angle = aB - aA - _referenceAngle;
      var limitImpulse = 0.0;

      if (_limitState == LimitState.equal) {
        // Prevent large angular corrections
        final c = (angle - _lowerAngle).clamp(
          -settings.maxAngularCorrection,
          settings.maxAngularCorrection,
        );
        limitImpulse = -_motorMass * c;
        angularError = c.abs();
      } else if (_limitState == LimitState.atLower) {
        var C = angle - _lowerAngle;
        angularError = -C;

        // Prevent large angular corrections and allow some slop.
        C = (C + settings.angularSlop).clamp(
          -settings.maxAngularCorrection,
          0.0,
        );
        limitImpulse = -_motorMass * C;
      } else if (_limitState == LimitState.atUpper) {
        var C = angle - _upperAngle;
        angularError = C;

        // Prevent large angular corrections and allow some slop.
        C = (C - settings.angularSlop).clamp(
          0.0,
          settings.maxAngularCorrection,
        );
        limitImpulse = -_motorMass * C;
      }

      aA -= _invIA * limitImpulse;
      aB += _invIB * limitImpulse;
    }
    // Solve point-to-point constraint.
    {
      qA.setAngle(aA);
      qB.setAngle(aB);

      final rA = Vector2.zero();
      final rB = Vector2.zero();
      final temp = Vector2.zero();
      final impulse = Vector2.zero();

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

      final mA = _invMassA;
      final mB = _invMassB;
      final iA = _invIA;
      final iB = _invIB;

      final K = Matrix2.zero();
      final a11 = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
      final a21 = -iA * rA.x * rA.y - iB * rB.x * rB.y;
      final a12 = a21;
      final a22 = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

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

    return positionError <= settings.linearSlop &&
        angularError <= settings.angularSlop;
  }

  @override
  Vector2 reactionForce(double invDt) {
    return Vector2(_impulse.x, _impulse.y)..scale(invDt);
  }

  @override
  double reactionTorque(double invDt) {
    return invDt * _impulse.z;
  }

  double jointAngle() {
    final b1 = bodyA;
    final b2 = bodyB;
    return b2.sweep.a - b1.sweep.a - referenceAngle;
  }

  double jointSpeed() {
    final b1 = bodyA;
    final b2 = bodyB;
    return b2.angularVelocity - b1.angularVelocity;
  }

  void enableMotor(bool flag) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _enableMotor = flag;
  }

  set motorSpeed(double speed) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _motorSpeed = speed;
  }

  double motorTorque(double invDt) => _motorImpulse * invDt;

  void setMaxMotorTorque(double torque) {
    bodyA.setAwake(true);
    bodyB.setAwake(true);
    _maxMotorTorque = torque;
  }

  void enableLimit(bool flag) {
    if (flag != _enableLimit) {
      bodyA.setAwake(true);
      bodyB.setAwake(true);
      _enableLimit = flag;
      _impulse.z = 0.0;
    }
  }

  void setLimits(double lower, double upper) {
    assert(lower <= upper);
    if (lower != _lowerAngle || upper != _upperAngle) {
      bodyA.setAwake(true);
      bodyB.setAwake(true);
      _impulse.z = 0.0;
      _lowerAngle = lower;
      _upperAngle = upper;
    }
  }

  @override
  void render(DebugDraw debugDraw) {
    super.render(debugDraw);

    final xf1 = bodyA.transform;
    final xf2 = bodyB.transform;
    final x1 = xf1.p;
    final x2 = xf2.p;
    final p1 = anchorA;
    final p2 = anchorB;

    debugDraw.drawSegment(x1, p1, renderColor);
    debugDraw.drawSegment(p1, p2, renderColor);
    debugDraw.drawSegment(x2, p2, renderColor);
  }
}
