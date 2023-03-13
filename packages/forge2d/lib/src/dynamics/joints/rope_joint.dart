import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// A rope joint enforces a maximum distance between two points on two bodies.
/// It has no other effect.
/// Warning: if you attempt to change the maximum length during the simulation
/// you will get some non-physical behavior. A model that would allow you to
/// dynamically modify the length would have some sponginess, so I chose not to
/// implement it that way. See DistanceJoint if you want to dynamically control
/// length.
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
  LimitState _state = LimitState.inactive;

  RopeJoint(RopeJointDef def) : super(def) {
    localAnchorA.setFrom(def.localAnchorA);
    localAnchorB.setFrom(def.localAnchorB);

    _maxLength = def.maxLength;
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

    _u
      ..setFrom(cB)
      ..add(_rB)
      ..sub(cA)
      ..sub(_rA);

    _length = _u.length;

    final c = _length - _maxLength;
    if (c > 0.0) {
      _state = LimitState.atUpper;
    } else {
      _state = LimitState.inactive;
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
    final crA = _rA.cross(_u);
    final crB = _rB.cross(_u);
    final invMass =
        _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      final pX = _impulse * _u.x;
      final pY = _impulse * _u.y;
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
  void solveVelocityConstraints(SolverData data) {
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final vpA = Vector2.zero();
    final vpB = Vector2.zero();
    final temp = Vector2.zero();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    final c = _length - _maxLength;
    var cDot = _u.dot(
      temp
        ..setFrom(vpB)
        ..sub(vpA),
    );

    // Predictive constraint.
    if (c < 0.0) {
      cDot += data.step.invDt * c;
    }

    var impulse = -_mass * cDot;
    final oldImpulse = _impulse;
    _impulse = min<double>(0.0, _impulse + impulse);
    impulse = _impulse - oldImpulse;

    final pX = impulse * _u.x;
    final pY = impulse * _u.y;
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
  bool solvePositionConstraints(SolverData data) {
    final cA = data.positions[_indexA].c;
    var aA = data.positions[_indexA].a;
    final cB = data.positions[_indexB].c;
    var aB = data.positions[_indexB].a;

    final qA = Rot();
    final qB = Rot();
    final u = Vector2.zero();
    final rA = Vector2.zero();
    final rB = Vector2.zero();
    final temp = Vector2.zero();

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

    final length = u.normalize();
    var c = length - _maxLength;

    c = c.clamp(0.0, settings.maxLinearCorrection);

    final impulse = -_mass * c;
    final pX = impulse * u.x;
    final pY = impulse * u.y;

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
  Vector2 reactionForce(double invDt) {
    return Vector2.copy(_u)
      ..scale(invDt)
      ..scale(_impulse);
  }

  @override
  double reactionTorque(double invDt) {
    return 0.0;
  }

  // TODO(spydon): Remove these old style getters and setters.
  double getMaxLength() {
    return _maxLength;
  }

  void setMaxLength(double maxLength) {
    _maxLength = maxLength;
  }

  LimitState getLimitState() {
    return _state;
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
