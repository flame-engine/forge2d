part of forge2d;

/// A distance joint constrains two points on two bodies to remain at a fixed distance from each
/// other. You can view this as a massless, rigid rod.

class DistanceJoint extends Joint {
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;
  double _bias = 0.0;

  // Solver shared
  double _gamma = 0.0;
  double _impulse = 0.0;
  double _length = 0.0;

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

  DistanceJoint(final DistanceJointDef def) : super(def) {
    _length = def.length;
    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
  }

  /// Get the reaction force given the inverse time step. Unit is N.
  @override
  Vector2 getReactionForce(double invDt) {
    return Vector2(
      _impulse * _u.x * invDt,
      _impulse * _u.y * invDt,
    );
  }

  /// Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a
  /// distance joint.
  @override
  double getReactionTorque(double invDt) => 0.0;

  @override
  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA.islandIndex;
    _indexB = _bodyB.islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

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
    final double length = _u.length;
    if (length > settings.linearSlop) {
      _u.x *= 1.0 / length;
      _u.y *= 1.0 / length;
    } else {
      _u.setValues(0.0, 0.0);
    }

    final double crAu = _rA.cross(_u);
    final double crBu = _rB.cross(_u);
    double invMass =
        _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

    // Compute the effective mass matrix.
    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (_frequencyHz > 0.0) {
      final double c = length - _length;

      // Frequency
      final double omega = 2.0 * math.pi * _frequencyHz;

      // Damping coefficient
      final double d = 2.0 * _mass * _dampingRatio * omega;

      // Spring stiffness
      final double k = _mass * omega * omega;

      // magic formulas
      final double dt = data.step.dt;
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

      final Vector2 p = Vector2.copy(_u)..scale(_impulse);

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
  void solveVelocityConstraints(final SolverData data) {
    final Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    final Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 vpA = Vector2.zero();
    final Vector2 vpB = Vector2.zero();

    // Cdot = dot(u, v + cross(w, r))
    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);
    final double cDot = _u.dot(vpB..sub(vpA));

    final double impulse = -_mass * (cDot + _bias + _gamma * _impulse);
    _impulse += impulse;

    final double pX = impulse * _u.x;
    final double pY = impulse * _u.y;

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
  bool solvePositionConstraints(final SolverData data) {
    if (_frequencyHz > 0.0) {
      return true;
    }
    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();
    final Vector2 u = Vector2.zero();

    final Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    final Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

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

    final double length = u.normalize();
    final C = (length - _length)
        .clamp(-settings.maxLinearCorrection, settings.maxLinearCorrection);

    final double impulse = -_mass * C;
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

    return C.abs() < settings.linearSlop;
  }
}
