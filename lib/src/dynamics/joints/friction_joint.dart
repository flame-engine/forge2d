part of box2d;

class FrictionJoint extends Joint {
  // Solver shared
  final Vector2 _linearImpulse;
  double _angularImpulse = 0.0;
  double _maxForce = 0.0;
  double _maxTorque = 0.0;

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
  final Matrix2 _linearMass = Matrix2.zero();
  double _angularMass = 0.0;

  FrictionJoint(FrictionJointDef def)
      : _linearImpulse = Vector2.zero(),
        super(def) {
    _maxForce = def.maxForce;
    _maxTorque = def.maxTorque;
  }

  /// Get the reaction force given the inverse time step. Unit is N.
  @override
  Vector2 getReactionForce(double inv_dt) {
    return Vector2.copy(_linearImpulse)..scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return inv_dt * _angularImpulse;
  }

  void setMaxForce(double force) {
    assert(force >= 0.0);
    _maxForce = force;
  }

  double getMaxForce() {
    return _maxForce;
  }

  void setMaxTorque(double torque) {
    assert(torque >= 0.0);
    _maxTorque = torque;
  }

  double getMaxTorque() {
    return _maxTorque;
  }

  /// @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)

  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

    double aA = data.positions[_indexA].a;
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 temp = Vector2.zero();
    final Rot qA = Rot();
    final Rot qB = Rot();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective mass matrix.
    temp
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    _rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    _rB.setFrom(Rot.mulVec2(qB, temp));

    double mA = _invMassA, mB = _invMassB;
    double iA = _invIA, iB = _invIB;

    final Matrix2 K = Matrix2.zero();
    double a11 = mA + mB + iA * _rA.y * _rA.y + iB * _rB.y * _rB.y;
    double a21 = -iA * _rA.x * _rA.y - iB * _rB.x * _rB.y;
    double a12 = a21;
    double a22 = mA + mB + iA * _rA.x * _rA.x + iB * _rB.x * _rB.x;

    K.setValues(a11, a12, a21, a22);
    _linearMass.setFrom(K);
    _linearMass.invert();

    _angularMass = iA + iB;
    if (_angularMass > 0.0) {
      _angularMass = 1.0 / _angularMass;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      _linearImpulse.scale(data.step.dtRatio);
      _angularImpulse *= data.step.dtRatio;

      final Vector2 P = Vector2.zero();
      P.setFrom(_linearImpulse);

      temp
        ..setFrom(P)
        ..scale(mA);
      vA.sub(temp);
      wA -= iA * (_rA.cross(P) + _angularImpulse);

      temp
        ..setFrom(P)
        ..scale(mB);
      vB.add(temp);
      wB += iB * (_rB.cross(P) + _angularImpulse);
    } else {
      _linearImpulse.setZero();
      _angularImpulse = 0.0;
    }
    if (data.velocities[_indexA].w != wA) {
      assert(data.velocities[_indexA].w != wA);
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

    double h = data.step.dt;

    // Solve angular friction
    {
      double Cdot = wB - wA;
      double impulse = -_angularMass * Cdot;

      double oldImpulse = _angularImpulse;
      double maxImpulse = h * _maxTorque;
      _angularImpulse = MathUtils.clampDouble(
          _angularImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = _angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve linear friction
    {
      final Vector2 Cdot = Vector2.zero();
      final Vector2 temp = Vector2.zero();

      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, Cdot);
      Cdot
        ..add(vB)
        ..sub(vA)
        ..sub(temp);

      final Vector2 impulse = Vector2.zero();
      _linearMass.transformed(Cdot, impulse);
      impulse.negate();

      final Vector2 oldImpulse = Vector2.zero();
      oldImpulse.setFrom(_linearImpulse);
      _linearImpulse.add(impulse);

      double maxImpulse = h * _maxForce;

      if (_linearImpulse.length2 > maxImpulse * maxImpulse) {
        _linearImpulse.normalize();
        _linearImpulse.scale(maxImpulse);
      }

      impulse
        ..setFrom(_linearImpulse)
        ..sub(oldImpulse);

      temp
        ..setFrom(impulse)
        ..scale(mA);
      vA.sub(temp);
      wA -= iA * _rA.cross(impulse);

      temp
        ..setFrom(impulse)
        ..scale(mB);
      vB.add(temp);
      wB += iB * _rB.cross(impulse);
    }

    if (data.velocities[_indexA].w != wA) {
      assert(data.velocities[_indexA].w != wA);
    }
    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    return true;
  }
}
