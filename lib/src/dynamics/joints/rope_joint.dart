import 'dart:math';

import '../../../forge2d.dart';
import '../../settings.dart' as settings;

/// A rope joint enforces a maximum distance between two points on two bodies. It has no other
/// effect. Warning: if you attempt to change the maximum length during the simulation you will get
/// some non-physical behavior. A model that would allow you to dynamically modify the length would
/// have some sponginess, so I chose not to implement it that way. See DistanceJoint if you want to
/// dynamically control length.
class RopeJoint extends Joint {
  // Solver shared
  @override
  final Vector2 localAnchorA = Vector2.zero();
  @override
  final Vector2 localAnchorB = Vector2.zero();
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

  RopeJoint(RopeJointDef def) : super(def) {
    localAnchorA.setFrom(def.localAnchorA);
    localAnchorB.setFrom(def.localAnchorB);

    _maxLength = def.maxLength;
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
    _rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    _rB.setFrom(Rot.mulVec2(qB, temp));

    _u
      ..setFrom(cB)
      ..add(_rB)
      ..sub(cA)
      ..sub(_rA);

    _length = _u.length;

    final double c = _length - _maxLength;
    if (c > 0.0) {
      _state = LimitState.AT_UPPER;
    } else {
      _state = LimitState.INACTIVE;
    }

    if (_length > settings.linearSlop) {
      _u.scale(1.0 / _length);
    } else {
      _u.setZero();
      _mass = 0.0;
      _impulse = 0.0;
      return;
    }

    // Compute effective mass.
    final double crA = _rA.cross(_u);
    final double crB = _rB.cross(_u);
    final double invMass =
        _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      final double pX = _impulse * _u.x;
      final double pY = _impulse * _u.y;
      vA.x -= _invMassA * pX;
      vA.y -= _invMassA * pY;
      wA -= _invIA * (_rA.x * pY - _rA.y * pX);

      vB.x += _invMassB * pX;
      vB.y += _invMassB * pY;
      wB += _invIB * (_rB.x * pY - _rB.y * pX);
    } else {
      _impulse = 0.0;
    }

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  @override
  void solveVelocityConstraints(final SolverData data) {
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 vpA = Vector2.zero();
    final Vector2 vpB = Vector2.zero();
    final Vector2 temp = Vector2.zero();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    final double c = _length - _maxLength;
    double cDot = _u.dot(temp
      ..setFrom(vpB)
      ..sub(vpA));

    // Predictive constraint.
    if (c < 0.0) {
      cDot += data.step.invDt * c;
    }

    double impulse = -_mass * cDot;
    final double oldImpulse = _impulse;
    _impulse = min<double>(0.0, _impulse + impulse);
    impulse = _impulse - oldImpulse;

    final double pX = impulse * _u.x;
    final double pY = impulse * _u.y;
    vA.x -= _invMassA * pX;
    vA.y -= _invMassA * pY;
    wA -= _invIA * (_rA.x * pY - _rA.y * pX);
    vB.x += _invMassB * pX;
    vB.y += _invMassB * pY;
    wB += _invIB * (_rB.x * pY - _rB.y * pX);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(final SolverData data) {
    final Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    final Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 u = Vector2.zero();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();
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

    u
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    final double length = u.normalize();
    double c = length - _maxLength;

    c = c.clamp(0.0, settings.maxLinearCorrection).toDouble();

    final double impulse = -_mass * c;
    final double pX = impulse * u.x;
    final double pY = impulse * u.y;

    cA.x -= _invMassA * pX;
    cA.y -= _invMassA * pY;
    aA -= _invIA * (rA.x * pY - rA.y * pX);
    cB.x += _invMassB * pX;
    cB.y += _invMassB * pY;
    aB += _invIB * (rB.x * pY - rB.y * pX);

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return length - _maxLength < settings.linearSlop;
  }

  @override
  Vector2 getReactionForce(double invDt) {
    return Vector2.copy(_u)..scale(invDt)..scale(_impulse);
  }

  @override
  double getReactionTorque(double invDt) {
    return 0.0;
  }

  // TODO: remove these getters and setters
  double getMaxLength() {
    return _maxLength;
  }

  void setMaxLength(double maxLength) {
    _maxLength = maxLength;
  }

  LimitState getLimitState() {
    return _state;
  }
}
