import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// A distance joint constrains two points on two bodies to remain at a fixed
/// distance from each other. You can view this as a massless, rigid rod.
class DistanceJoint extends Joint {
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;
  double _bias = 0.0;

  // Solver shared
  double _gamma = 0.0;
  double _impulse = 0.0;
  double _length = 0.0;

  set length(double value) {
    _length = value;
  }

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

  DistanceJoint(DistanceJointDef def) : super(def) {
    _length = def.length;
    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
  }

  /// Get the reaction force given the inverse time step. Unit is N.
  @override
  Vector2 reactionForce(double invDt) {
    return Vector2(
      _impulse * _u.x * invDt,
      _impulse * _u.y * invDt,
    );
  }

  /// Get the reaction torque given the inverse time step. Unit is N*m. This is
  /// always zero for a distance joint.
  @override
  double reactionTorque(double invDt) => 0.0;

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

    qA.setAngle(aA);
    qB.setAngle(aB);

    // use _u as temporary variable
    _u
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    _rA.setFrom(Rot.mulVec2(qA, _u));
    _u
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    _rB.setFrom(Rot.mulVec2(qB, _u));
    _u
      ..setFrom(cB)
      ..add(_rB)
      ..sub(cA)
      ..sub(_rA);

    // Handle singularity.
    final length = _u.length;
    if (length > settings.linearSlop) {
      _u.x *= 1.0 / length;
      _u.y *= 1.0 / length;
    } else {
      _u.setValues(0.0, 0.0);
    }

    final crAu = _rA.cross(_u);
    final crBu = _rB.cross(_u);
    var invMass =
        _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

    // Compute the effective mass matrix.
    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (_frequencyHz > 0.0) {
      final c = length - _length;

      // Frequency
      final omega = 2.0 * pi * _frequencyHz;

      // Damping coefficient
      final d = 2.0 * _mass * _dampingRatio * omega;

      // Spring stiffness
      final k = _mass * omega * omega;

      // magic formulas
      final dt = data.step.dt;
      _gamma = dt * (d + dt * k);
      _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
      _bias = c * dt * k * _gamma;

      invMass += _gamma;
      _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    } else {
      _gamma = 0.0;
      _bias = 0.0;
    }
    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      final p = Vector2.copy(_u)..scale(_impulse);

      vA.x -= _invMassA * p.x;
      vA.y -= _invMassA * p.y;
      wA -= _invIA * _rA.cross(p);

      vB.x += _invMassB * p.x;
      vB.y += _invMassB * p.y;
      wB += _invIB * _rB.cross(p);
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

    // Cdot = dot(u, v + cross(w, r))
    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);
    final cDot = _u.dot(vpB..sub(vpA));

    final impulse = -_mass * (cDot + _bias + _gamma * _impulse);
    _impulse += impulse;

    final pX = impulse * _u.x;
    final pY = impulse * _u.y;

    vA.x -= _invMassA * pX;
    vA.y -= _invMassA * pY;
    wA -= _invIA * (_rA.x * pY - _rA.y * pX);
    vB.x += _invMassB * pX;
    vB.y += _invMassB * pY;
    wB += _invIB * (_rB.x * pY - _rB.y * pX);

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(SolverData data) {
    if (_frequencyHz > 0.0) {
      return true;
    }
    final qA = Rot();
    final qB = Rot();
    final rA = Vector2.zero();
    final rB = Vector2.zero();
    final u = Vector2.zero();

    final cA = data.positions[_indexA].c;
    var aA = data.positions[_indexA].a;
    final cB = data.positions[_indexB].c;
    var aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    u
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    rA.setFrom(Rot.mulVec2(qA, u));
    u
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    rB.setFrom(Rot.mulVec2(qB, u));
    u
      ..setFrom(cB)
      ..add(rB)
      ..sub(cA)
      ..sub(rA);

    final length = u.normalize();
    final C = (length - _length).clamp(
      -settings.maxLinearCorrection,
      settings.maxLinearCorrection,
    );

    final impulse = -_mass * C;
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

    return C.abs() < settings.linearSlop;
  }

  @override
  void render(DebugDraw debugDraw) {
    super.render(debugDraw);

    final p1 = anchorA;
    final p2 = anchorB;

    debugDraw.drawSegment(p1, p2, renderColor);
  }
}
