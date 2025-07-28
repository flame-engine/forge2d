import 'dart:math';

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//    = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

/// A weld joint essentially glues two bodies together. A weld joint may distort
/// somewhat because the island constraint solver is approximate.
class WeldJoint extends Joint {
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;
  double _bias = 0.0;

  // Solver shared
  @override
  final Vector2 localAnchorA;
  @override
  final Vector2 localAnchorB;
  double _referenceAngle = 0.0;
  double _gamma = 0.0;
  final Vector3 _impulse;

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
  final Matrix3 _mass = Matrix3.zero();

  WeldJoint(WeldJointDef def)
    : localAnchorA = Vector2.copy(def.localAnchorA),
      localAnchorB = Vector2.copy(def.localAnchorB),
      _impulse = Vector3.zero(),
      super(def) {
    _referenceAngle = def.referenceAngle;
    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
  }

  double getReferenceAngle() {
    return _referenceAngle;
  }

  @override
  Vector2 reactionForce(double invDt) {
    return Vector2(_impulse.x, _impulse.y)..scale(invDt);
  }

  @override
  double reactionTorque(double invDt) {
    return invDt * _impulse.z;
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

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    final temp = Vector2.copy(localAnchorA)..sub(_localCenterA);
    _rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    _rB.setFrom(Rot.mulVec2(qB, temp));

    // J = [-I -r1_skew I r2_skew]
    // [ 0 -1 0 1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
    // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
    // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

    final mA = _invMassA;
    final mB = _invMassB;
    final iA = _invIA;
    final iB = _invIB;

    final K = Matrix3.zero();

    final exX = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
    final eyX = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
    final ezX = -_rA.y * iA - _rB.y * iB;
    final exY = K.entry(0, 1);
    final eyY = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
    final ezY = _rA.x * iA + _rB.x * iB;
    final exZ = K.entry(0, 2);
    final eyZ = K.entry(1, 2);
    final ezZ = iA + iB;

    K.setValues(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);

    if (_frequencyHz > 0.0) {
      _mass.setFrom(_matrix3GetInverse22(K));

      var invM = iA + iB;
      final m = invM > 0.0 ? 1.0 / invM : 0.0;

      final c = aB - aA - _referenceAngle;

      // Frequency
      final omega = 2.0 * pi * _frequencyHz;

      // Damping coefficient
      final d = 2.0 * m * _dampingRatio * omega;

      // Spring stiffness
      final k = m * omega * omega;

      // magic formulas
      final dt = data.step.dt;
      _gamma = dt * (d + dt * k);
      _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
      _bias = c * dt * k * _gamma;

      invM += _gamma;
      _mass.setEntry(2, 2, invM != 0.0 ? 1.0 / invM : 0.0);
    } else {
      _mass.setFrom(_matrix3GetSymInverse33(K, _mass));
      _gamma = 0.0;
      _bias = 0.0;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      _impulse.scale(data.step.dtRatio);

      final P = Vector2(_impulse.x, _impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (_rA.cross(P) + _impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (_rB.cross(P) + _impulse.z);
    } else {
      _impulse.setZero();
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

    final mA = _invMassA;
    final mB = _invMassB;
    final iA = _invIA;
    final iB = _invIB;

    final cDot1 = Vector2.zero();
    final p = Vector2.zero();
    final temp = Vector2.zero();
    if (_frequencyHz > 0.0) {
      final cDot2 = wB - wA;

      final impulse2 =
          -_mass.entry(2, 2) * (cDot2 + _bias + _gamma * _impulse.z);
      _impulse.z += impulse2;

      wA -= iA * impulse2;
      wB += iB * impulse2;

      _rB.scaleOrthogonalInto(wB, cDot1);
      _rA.scaleOrthogonalInto(wA, temp);
      cDot1
        ..add(vB)
        ..sub(vA)
        ..sub(temp);

      final impulse1 = Vector2(
        _mass.entry(1, 0) * cDot1.x + _mass.entry(1, 1) * cDot1.y,
        _mass.entry(0, 0) * cDot1.x + _mass.entry(0, 1) * cDot1.y,
      )..negate();

      _impulse.x += impulse1.x;
      _impulse.y += impulse1.y;

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * _rA.cross(p);

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * _rB.cross(p);
    } else {
      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, cDot1);
      cDot1
        ..add(vB)
        ..sub(vA)
        ..sub(temp);
      final cDot2 = wB - wA;

      final impulse = Vector3(cDot1.x, cDot1.y, cDot2)
        ..applyMatrix3(_mass)
        ..negate();
      _impulse.add(impulse);

      p.setValues(impulse.x, impulse.y);

      vA.x -= mA * p.x;
      vA.y -= mA * p.y;
      wA -= iA * (_rA.cross(p) + impulse.z);

      vB.x += mB * p.x;
      vB.y += mB * p.y;
      wB += iB * (_rB.cross(p) + impulse.z);
    }

    data.velocities[_indexA].w = wA;
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

    qA.setAngle(aA);
    qB.setAngle(aB);

    final mA = _invMassA;
    final mB = _invMassB;
    final iA = _invIA;
    final iB = _invIB;

    final temp = Vector2.copy(localAnchorA)..sub(_localCenterA);
    final rA = Vector2.copy(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    final rB = Vector2.copy(Rot.mulVec2(qB, temp));
    double positionError;
    double angularError;

    final k = Matrix3.zero();
    final c1 = Vector2.zero();
    final p = Vector2.zero();

    final exX = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    final eyX = -rA.y * rA.x * iA - rB.y * rB.x * iB;
    final ezX = -rA.y * iA - rB.y * iB;
    final exY = k.entry(0, 1);
    final eyY = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
    final ezY = rA.x * iA + rB.x * iB;
    final exZ = k.entry(0, 2);
    final eyZ = k.entry(1, 2);
    final ezZ = iA + iB;

    k.setValues(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);

    if (_frequencyHz > 0.0) {
      c1
        ..setFrom(cB)
        ..add(rB)
        ..sub(cA)
        ..sub(rA);

      positionError = c1.length;
      angularError = 0.0;

      Matrix3.solve2(k, p, c1);
      p.negate();

      cA.x -= mA * p.x;
      cA.y -= mA * p.y;
      aA -= iA * rA.cross(p);

      cB.x += mB * p.x;
      cB.y += mB * p.y;
      aB += iB * rB.cross(p);
    } else {
      c1
        ..setFrom(cB)
        ..add(rB)
        ..sub(cA)
        ..sub(rA);
      final c2 = aB - aA - _referenceAngle;

      positionError = c1.length;
      angularError = c2.abs();

      final C = Vector3(c1.x, c1.y, c2);
      final impulse = Vector3.zero();

      Matrix3.solve(k, impulse, C);
      impulse.negate();
      p.setValues(impulse.x, impulse.y);

      cA.x -= mA * p.x;
      cA.y -= mA * p.y;
      aA -= iA * (rA.cross(p) + impulse.z);

      cB.x += mB * p.x;
      cB.y += mB * p.y;
      aB += iB * (rB.cross(p) + impulse.z);
    }

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return positionError <= settings.linearSlop &&
        angularError <= settings.angularSlop;
  }

  Matrix3 _matrix3GetInverse22(Matrix3 m) {
    final a = m.entry(1, 0);
    final b = m.entry(0, 1);
    final c = m.entry(1, 0);
    final d = m.entry(1, 1);
    var det = a * d - b * c;
    if (det != 0.0) {
      det = 1.0 / det;
    }

    final exX = det * d;
    final eyX = -det * b;
    const ezX = 0.0;
    final exY = -det * c;
    final eyY = det * a;
    const ezY = 0.0;
    const exZ = 0.0;
    const eyZ = 0.0;
    const ezZ = 0.0;
    return Matrix3(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);
  }

  /// Returns the zero matrix if singular.
  Matrix3 _matrix3GetSymInverse33(Matrix3 m, Matrix3 m2) {
    final bx = m.entry(1, 1) * m.entry(2, 2) - m.entry(2, 1) * m.entry(1, 2);
    final by = m.entry(2, 1) * m.entry(0, 2) - m.entry(0, 1) * m.entry(2, 2);
    final bz = m.entry(0, 1) * m.entry(1, 2) - m.entry(1, 1) * m.entry(0, 2);
    var det = m.entry(0, 0) * bx + m.entry(1, 0) * by + m.entry(2, 0) * bz;
    if (det != 0.0) {
      det = 1.0 / det;
    }

    final a11 = m.entry(0, 0);
    final a12 = m.entry(0, 1);
    final a13 = m.entry(0, 2);
    final a22 = m.entry(1, 1);
    final a23 = m.entry(1, 2);
    final a33 = m.entry(2, 2);

    final exX = det * (a22 * a33 - a23 * a23);
    final exY = det * (a13 * a23 - a12 * a33);
    final exZ = det * (a12 * a23 - a13 * a22);

    final eyX = m2.entry(1, 0);
    final eyY = det * (a11 * a33 - a13 * a13);
    final eyZ = det * (a13 * a12 - a11 * a23);

    final ezX = m2.entry(2, 0);
    final ezY = m2.entry(2, 1);
    final ezZ = det * (a11 * a22 - a12 * a12);
    return Matrix3(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);
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
