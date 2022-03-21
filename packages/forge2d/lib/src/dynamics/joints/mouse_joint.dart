import 'dart:math';

import '../../../forge2d.dart';
import '../../settings.dart' as settings;

/// A mouse joint is used to make a point on a body track a specified world point. This a soft
/// constraint with a maximum force. This allows the constraint to stretch and without applying huge
/// forces. NOTE: this joint is not documented in the manual because it was developed to be used in
/// the testbed. If you want to learn how to use the mouse joint, look at the testbed.
class MouseJoint extends Joint {
  final Vector2 _targetA = Vector2.zero();
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;
  double _beta = 0.0;

  // Solver shared
  final Vector2 _impulse = Vector2.zero();
  double _maxForce = 0.0;
  double _gamma = 0.0;

  // Solver temp
  int _indexB = 0;
  final Vector2 _rB = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassB = 0.0;
  double _invIB = 0.0;
  final Matrix2 _mass = Matrix2.zero();
  final Vector2 _c = Vector2.zero();

  MouseJoint(MouseJointDef def)
      : assert(!def.target.isInfinite && !def.target.isNaN),
        assert(def.maxForce >= 0),
        assert(def.frequencyHz >= 0),
        assert(def.dampingRatio >= 0),
        super(def) {
    _targetA.setFrom(def.target);
    localAnchorB.setFrom(Transform.mulTransVec2(bodyB.transform, _targetA));

    _maxForce = def.maxForce;
    _impulse.setZero();

    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
  }

  @override
  Vector2 get anchorA {
    return Vector2.copy(_targetA);
  }

  @override
  Vector2 reactionForce(double invDt) {
    return Vector2.copy(_impulse)..scale(invDt);
  }

  @override
  double reactionTorque(double invDt) {
    return invDt * 0.0;
  }

  void setTarget(Vector2 target) {
    if (bodyB.isAwake == false) {
      bodyB.setAwake(true);
    }
    _targetA.setFrom(target);
  }

  Vector2 getTarget() {
    return _targetA;
  }

  @override
  void initVelocityConstraints(final SolverData data) {
    _indexB = bodyB.islandIndex;
    _localCenterB.setFrom(bodyB.sweep.localCenter);
    _invMassB = bodyB.inverseMass;
    _invIB = bodyB.inverseInertia;

    final cB = data.positions[_indexB].c;
    final aB = data.positions[_indexB].a;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final qB = Rot();

    qB.setAngle(aB);

    final mass = bodyB.mass;

    // Frequency
    final omega = 2.0 * pi * _frequencyHz;

    // Damping coefficient
    final d = 2.0 * mass * _dampingRatio * omega;

    final springStiffness = mass * (omega * omega);

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    final dt = data.step.dt;
    assert(d + dt * springStiffness > settings.epsilon);
    _gamma = dt * (d + dt * springStiffness);
    if (_gamma != 0.0) {
      _gamma = 1.0 / _gamma;
    }
    _beta = dt * springStiffness * _gamma;

    // Compute the effective mass matrix.
    final effectiveMassMatrix = Vector2.copy(localAnchorB)..sub(_localCenterB);

    _rB.setFrom(Rot.mulVec2(qB, effectiveMassMatrix));

    // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
    final k = Matrix2.zero();
    final a11 = _invMassB + _invIB * _rB.y * _rB.y + _gamma;
    final a21 = -_invIB * _rB.x * _rB.y;
    final a12 = a21;
    final a22 = _invMassB + _invIB * _rB.x * _rB.x + _gamma;

    k.setValues(a11, a21, a12, a22);
    _mass.setFrom(k);
    _mass.invert();

    _c
      ..setFrom(cB)
      ..add(_rB)
      ..sub(_targetA);
    _c.scale(_beta);

    // Cheat with some damping
    wB *= 0.98;

    if (data.step.warmStarting) {
      _impulse.scale(data.step.dtRatio);
      vB.x += _invMassB * _impulse.x;
      vB.y += _invMassB * _impulse.y;
      wB += _invIB * _rB.cross(_impulse);
    } else {
      _impulse.setZero();
    }

    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(final SolverData data) {
    return true;
  }

  @override
  void solveVelocityConstraints(final SolverData data) {
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    // Cdot = v + cross(w, r)
    final cDot = Vector2.zero();
    _rB.scaleOrthogonalInto(wB, cDot);
    cDot.add(vB);

    final impulse = Vector2.zero();
    final temp = Vector2.zero();

    temp
      ..setFrom(_impulse)
      ..scale(_gamma)
      ..add(_c)
      ..add(cDot)
      ..negate();
    _mass.transformed(temp, impulse);

    final oldImpulse = temp;
    oldImpulse.setFrom(_impulse);
    _impulse.add(impulse);
    final maxImpulse = data.step.dt * _maxForce;
    if (_impulse.length2 > maxImpulse * maxImpulse) {
      _impulse.scale(maxImpulse / _impulse.length);
    }
    impulse
      ..setFrom(_impulse)
      ..sub(oldImpulse);

    vB.x += _invMassB * impulse.x;
    vB.y += _invMassB * impulse.y;
    wB += _invIB * _rB.cross(impulse);

    data.velocities[_indexB].w = wB;
  }
}
