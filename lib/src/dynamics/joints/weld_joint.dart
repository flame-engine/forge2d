part of box2d;

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
  final Vector2 localAnchorA;
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

  Vector2 getReactionForce(double inv_dt) {
    return Vector2(_impulse.x, _impulse.y)..scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * _impulse.z;
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

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    final Matrix3 K = Matrix3.zero();

    double ex_x = mA + mB + _rA.y * _rA.y * iA + _rB.y * _rB.y * iB;
    double ey_x = -_rA.y * _rA.x * iA - _rB.y * _rB.x * iB;
    double ez_x = -_rA.y * iA - _rB.y * iB;
    double ex_y = K.entry(0, 1);
    double ey_y = mA + mB + _rA.x * _rA.x * iA + _rB.x * _rB.x * iB;
    double ez_y = _rA.x * iA + _rB.x * iB;
    double ex_z = K.entry(0, 2);
    double ey_z = K.entry(1, 2);
    double ez_z = iA + iB;

    K.setValues(ex_x, ex_y, ex_z, ey_x, ey_y, ey_z, ez_x, ez_y, ez_z);

    if (_frequencyHz > 0.0) {
      MathUtils.matrix3GetInverse22(K, _mass);

      double invM = iA + iB;
      double m = invM > 0.0 ? 1.0 / invM : 0.0;

      double C = aB - aA - _referenceAngle;

      // Frequency
      double omega = 2.0 * Math.pi * _frequencyHz;

      // Damping coefficient
      double d = 2.0 * m * _dampingRatio * omega;

      // Spring stiffness
      double k = m * omega * omega;

      // magic formulas
      double h = data.step.dt;
      _gamma = h * (d + h * k);
      _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
      _bias = C * h * k * _gamma;

      invM += _gamma;
      _mass.setEntry(2, 2, (invM != 0.0 ? 1.0 / invM : 0.0));
    } else {
      MathUtils.matrix3GetSymInverse33(K, _mass);
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

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    final Vector2 Cdot1 = Vector2.zero();
    final Vector2 P = Vector2.zero();
    final Vector2 temp = Vector2.zero();
    if (_frequencyHz > 0.0) {
      double Cdot2 = wB - wA;

      double impulse2 =
          -_mass.entry(2, 2) * (Cdot2 + _bias + _gamma * _impulse.z);
      _impulse.z += impulse2;

      wA -= iA * impulse2;
      wB += iB * impulse2;

      _rB.scaleOrthogonalInto(wB, Cdot1);
      _rA.scaleOrthogonalInto(wA, temp);
      Cdot1
        ..add(vB)
        ..sub(vA)
        ..sub(temp);

      P.setFrom(MathUtils.matrix3Mul22(_mass, Cdot1)..negate());
      final Vector2 impulse1 = P;

      _impulse.x += impulse1.x;
      _impulse.y += impulse1.y;

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * _rA.cross(P);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * _rB.cross(P);
    } else {
      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, Cdot1);
      Cdot1
        ..add(vB)
        ..sub(vA)
        ..sub(temp);
      double Cdot2 = wB - wA;

      final Vector3 Cdot = Vector3(Cdot1.x, Cdot1.y, Cdot2);
      final Vector3 impulse = MathUtils.matrix3Mul(_mass, Cdot)..negate();

      _impulse.add(impulse);

      P.setValues(impulse.x, impulse.y);

      vA.x -= mA * P.x;
      vA.y -= mA * P.y;
      wA -= iA * (_rA.cross(P) + impulse.z);

      vB.x += mB * P.x;
      vB.y += mB * P.y;
      wB += iB * (_rB.cross(P) + impulse.z);
    }

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    final Rot qA = Rot();
    final Rot qB = Rot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    final Vector2 temp = Vector2.copy(localAnchorA)..sub(_localCenterA);
    final Vector2 rA = Vector2.copy(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    final Vector2 rB = Vector2.copy(Rot.mulVec2(qB, temp));
    double positionError, angularError;

    final Matrix3 K = Matrix3.zero();
    final Vector2 C1 = Vector2.zero();
    final Vector2 P = Vector2.zero();

    double ex_x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
    double ey_x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
    double ez_x = -rA.y * iA - rB.y * iB;
    double ex_y = K.entry(0, 1);
    double ey_y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
    double ez_y = rA.x * iA + rB.x * iB;
    double ex_z = K.entry(0, 2);
    double ey_z = K.entry(1, 2);
    double ez_z = iA + iB;

    K.setValues(ex_x, ex_y, ex_z, ey_x, ey_y, ey_z, ez_x, ez_y, ez_z);

    if (_frequencyHz > 0.0) {
      C1
        ..setFrom(cB)
        ..add(rB)
        ..sub(cA)
        ..sub(rA);

      positionError = C1.length;
      angularError = 0.0;

      Matrix3.solve2(K, P, C1);
      P.negate();

      cA.x -= mA * P.x;
      cA.y -= mA * P.y;
      aA -= iA * rA.cross(P);

      cB.x += mB * P.x;
      cB.y += mB * P.y;
      aB += iB * rB.cross(P);
    } else {
      C1
        ..setFrom(cB)
        ..add(rB)
        ..sub(cA)
        ..sub(rA);
      double C2 = aB - aA - _referenceAngle;

      positionError = C1.length;
      angularError = C2.abs();

      final Vector3 C = Vector3(C1.x, C1.y, C2);
      final Vector3 impulse = Vector3.zero();

      Matrix3.solve(K, impulse, C);
      impulse.negate();
      P.setValues(impulse.x, impulse.y);

      cA.x -= mA * P.x;
      cA.y -= mA * P.y;
      aA -= iA * (rA.cross(P) + impulse.z);

      cB.x += mB * P.x;
      cB.y += mB * P.y;
      aB += iB * (rB.cross(P) + impulse.z);
    }

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return positionError <= Settings.linearSlop &&
        angularError <= Settings.angularSlop;
  }
}
