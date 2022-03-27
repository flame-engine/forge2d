import '../../../forge2d.dart';

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
  Vector2 reactionForce(double invDt) {
    return Vector2.copy(_linearImpulse)..scale(invDt);
  }

  @override
  double reactionTorque(double invDt) {
    return invDt * _angularImpulse;
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

    final aA = data.positions[_indexA].a;
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;

    final aB = data.positions[_indexB].a;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final temp = Vector2.zero();
    final qA = Rot();
    final qB = Rot();

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

    final mA = _invMassA, mB = _invMassB;
    final iA = _invIA, iB = _invIB;

    final K = Matrix2.zero();
    final a11 = mA + mB + iA * _rA.y * _rA.y + iB * _rB.y * _rB.y;
    final a21 = -iA * _rA.x * _rA.y - iB * _rB.x * _rB.y;
    final a12 = a21;
    final a22 = mA + mB + iA * _rA.x * _rA.x + iB * _rB.x * _rB.x;

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

      final P = Vector2.zero();
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

  @override
  void solveVelocityConstraints(final SolverData data) {
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final mA = _invMassA, mB = _invMassB;
    final iA = _invIA, iB = _invIB;

    final dt = data.step.dt;

    // Solve angular friction
    {
      final cDot = wB - wA;
      var impulse = -_angularMass * cDot;

      final oldImpulse = _angularImpulse;
      final maxImpulse = dt * _maxTorque;
      _angularImpulse =
          (_angularImpulse + impulse).clamp(-maxImpulse, maxImpulse).toDouble();
      impulse = _angularImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve linear friction
    {
      final cDot = Vector2.zero();
      final temp = Vector2.zero();

      _rA.scaleOrthogonalInto(wA, temp);
      _rB.scaleOrthogonalInto(wB, cDot);
      cDot
        ..add(vB)
        ..sub(vA)
        ..sub(temp);

      final impulse = Vector2.zero();
      _linearMass.transformed(cDot, impulse);
      impulse.negate();

      final oldImpulse = Vector2.zero();
      oldImpulse.setFrom(_linearImpulse);
      _linearImpulse.add(impulse);

      final maxImpulse = dt * _maxForce;

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

  @override
  bool solvePositionConstraints(final SolverData data) {
    return true;
  }

  @override
  void render(DebugDraw debugDraw) {
    super.render(debugDraw);

    final xf1 = bodyA.transform;
    final xf2 = bodyB.transform;
    final x1 = xf1.p;
    final x2 = xf2.p;

    debugDraw.drawSegment(x1, x2, renderColor);
  }
}
