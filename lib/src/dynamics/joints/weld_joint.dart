part of forge2d;

//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Angle constraint
//C = angle2 - angle1 - referenceAngle
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/// A weld joint essentially glues two bodies together. A weld joint may distort somewhat because the
/// island constraint solver is approximate.
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
  Vector2 getReactionForce(double invDt) {
    return Vector2(_impulse.x, _impulse.y)..scale(invDt);
  }

  @override
  double getReactionTorque(double invDt) {
    return invDt * _impulse.z;
  }

  @override
  void initVelocityConstraints(final SolverData data) {
    _indexA = bodyA.islandIndex;
    _indexB = bodyB.islandIndex;
    _localCenterA.setFrom(bodyA._sweep.localCenter);
    _localCenterB.setFrom(bodyB._sweep.localCenter);
    _invMassA = bodyA._invMass;
    _invMassB = bodyB._invMass;
    _invIA = bodyA.inverseInertia;
    _invIB = bodyB.inverseInertia;

    // Vec2 cA = data.positions[_indexA].c;
    final double aA = data.positions[_indexA].a;
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    // Vec2 cB = data.positions[_indexB].c;
    final double aB = data.positions[_indexB].a;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Rot qA = Rot();
    final Rot qB = Rot();

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

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final Matrix3 K = Matrix3.zero();

    final double exX = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
    final double eyX = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
    final double ezX = -_rA.y * iA - _rB.y * iB;
    final double exY = K.entry(0, 1);
    final double eyY = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
    final double ezY = _rA.x * iA + _rB.x * iB;
    final double exZ = K.entry(0, 2);
    final double eyZ = K.entry(1, 2);
    final double ezZ = iA + iB;

    K.setValues(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);

    if (_frequencyHz > 0.0) {
      _mass.setFrom(_matrix3GetInverse22(K));

      double invM = iA + iB;
      final double m = invM > 0.0 ? 1.0 / invM : 0.0;

      final double c = aB - aA - _referenceAngle;

      // Frequency
      final double omega = 2.0 * math.pi * _frequencyHz;

      // Damping coefficient
      final double d = 2.0 * m * _dampingRatio * omega;

      // Spring stiffness
      final double k = m * omega * omega;

      // magic formulas
      final double dt = data.step.dt;
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

      final Vector2 P = Vector2(_impulse.x, _impulse.y);

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
  void solveVelocityConstraints(final SolverData data) {
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final Vector2 cDot1 = Vector2.zero();
    final Vector2 p = Vector2.zero();
    final Vector2 temp = Vector2.zero();
    if (_frequencyHz > 0.0) {
      final double cDot2 = wB - wA;

      final double impulse2 =
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

      final Vector2 impulse1 = Vector2(
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
      final double cDot2 = wB - wA;

      final Vector3 impulse = Vector3(cDot1.x, cDot1.y, cDot2)
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
  bool solvePositionConstraints(final SolverData data) {
    final Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    final Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    final Rot qA = Rot();
    final Rot qB = Rot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    final double mA = _invMassA, mB = _invMassB;
    final double iA = _invIA, iB = _invIB;

    final Vector2 temp = Vector2.copy(localAnchorA)..sub(_localCenterA);
    final Vector2 rA = Vector2.copy(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    final Vector2 rB = Vector2.copy(Rot.mulVec2(qB, temp));
    double positionError, angularError;

    final Matrix3 k = Matrix3.zero();
    final Vector2 c1 = Vector2.zero();
    final Vector2 p = Vector2.zero();

    final double exX = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    final double eyX = -rA.y * rA.x * iA - rB.y * rB.x * iB;
    final double ezX = -rA.y * iA - rB.y * iB;
    final double exY = k.entry(0, 1);
    final double eyY = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
    final double ezY = rA.x * iA + rB.x * iB;
    final double exZ = k.entry(0, 2);
    final double eyZ = k.entry(1, 2);
    final double ezZ = iA + iB;

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
      final double c2 = aB - aA - _referenceAngle;

      positionError = c1.length;
      angularError = c2.abs();

      final Vector3 C = Vector3(c1.x, c1.y, c2);
      final Vector3 impulse = Vector3.zero();

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
    final double a = m.entry(1, 0);
    final double b = m.entry(0, 1);
    final double c = m.entry(1, 0);
    final double d = m.entry(1, 1);
    double det = a * d - b * c;
    if (det != 0.0) {
      det = 1.0 / det;
    }

    final double exX = det * d;
    final double eyX = -det * b;
    const double ezX = 0.0;
    final double exY = -det * c;
    final double eyY = det * a;
    const double ezY = 0.0;
    const double exZ = 0.0;
    const double eyZ = 0.0;
    const double ezZ = 0.0;
    return Matrix3(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);
  }

  /// Returns the zero matrix if singular.
  Matrix3 _matrix3GetSymInverse33(Matrix3 m, Matrix3 m2) {
    final double bx =
        m.entry(1, 1) * m.entry(2, 2) - m.entry(2, 1) * m.entry(1, 2);
    final double by =
        m.entry(2, 1) * m.entry(0, 2) - m.entry(0, 1) * m.entry(2, 2);
    final double bz =
        m.entry(0, 1) * m.entry(1, 2) - m.entry(1, 1) * m.entry(0, 2);
    double det = m.entry(0, 0) * bx + m.entry(1, 0) * by + m.entry(2, 0) * bz;
    if (det != 0.0) {
      det = 1.0 / det;
    }

    final double a11 = m.entry(0, 0), a12 = m.entry(0, 1), a13 = m.entry(0, 2);
    final double a22 = m.entry(1, 1), a23 = m.entry(1, 2);
    final double a33 = m.entry(2, 2);

    final double exX = det * (a22 * a33 - a23 * a23);
    final double exY = det * (a13 * a23 - a12 * a33);
    final double exZ = det * (a12 * a23 - a13 * a22);

    final double eyX = m2.entry(1, 0);
    final double eyY = det * (a11 * a33 - a13 * a13);
    final double eyZ = det * (a13 * a12 - a11 * a23);

    final double ezX = m2.entry(2, 0);
    final double ezY = m2.entry(2, 1);
    final double ezZ = det * (a11 * a22 - a12 * a12);
    return Matrix3(exX, exY, exZ, eyX, eyY, eyZ, ezX, ezY, ezZ);
  }
}
