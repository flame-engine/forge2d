part of box2d;

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
  Vector2 getReactionForce(double inv_dt) {
    return Vector2(
      _impulse * _u.x * inv_dt,
      _impulse * _u.y * inv_dt,
    );
  }

  /// Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a
  /// distance joint.
  @override
  double getReactionTorque(double inv_dt) => 0.0;

  void initVelocityConstraints(final SolverData data) {
    _indexA = _bodyA._islandIndex;
    _indexB = _bodyB._islandIndex;
    _localCenterA.setFrom(_bodyA._sweep.localCenter);
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassA = _bodyA._invMass;
    _invMassB = _bodyB._invMass;
    _invIA = _bodyA._invI;
    _invIB = _bodyB._invI;

    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;

    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
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
    double length = _u.length;
    if (length > Settings.linearSlop) {
      _u.x *= 1.0 / length;
      _u.y *= 1.0 / length;
    } else {
      _u.setValues(0.0, 0.0);
    }

    double crAu = _rA.cross(_u);
    double crBu = _rB.cross(_u);
    double invMass =
        _invMassA + _invIA * crAu * crAu + _invMassB + _invIB * crBu * crBu;

    // Compute the effective mass matrix.
    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (_frequencyHz > 0.0) {
      double C = length - _length;

      // Frequency
      double omega = 2.0 * Math.pi * _frequencyHz;

      // Damping coefficient
      double d = 2.0 * _mass * _dampingRatio * omega;

      // Spring stiffness
      double k = _mass * omega * omega;

      // magic formulas
      double h = data.step.dt;
      _gamma = h * (d + h * k);
      _gamma = _gamma != 0.0 ? 1.0 / _gamma : 0.0;
      _bias = C * h * k * _gamma;

      invMass += _gamma;
      _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;
    } else {
      _gamma = 0.0;
      _bias = 0.0;
    }
    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      Vector2 P = Vector2.zero();
      P
        ..setFrom(_u)
        ..scale(_impulse);

      vA.x -= _invMassA * P.x;
      vA.y -= _invMassA * P.y;
      wA -= _invIA * _rA.cross(P);

      vB.x += _invMassB * P.x;
      vB.y += _invMassB * P.y;
      wB += _invIB * _rB.cross(P);
    } else {
      _impulse = 0.0;
    }
    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_indexA].v;
    double wA = data.velocities[_indexA].w;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Vector2 vpA = Vector2.zero();
    final Vector2 vpB = Vector2.zero();

    // Cdot = dot(u, v + cross(w, r))
    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);
    double Cdot = _u.dot(vpB..sub(vpA));

    double impulse = -_mass * (Cdot + _bias + _gamma * _impulse);
    _impulse += impulse;

    double Px = impulse * _u.x;
    double Py = impulse * _u.y;

    vA.x -= _invMassA * Px;
    vA.y -= _invMassA * Py;
    wA -= _invIA * (_rA.x * Py - _rA.y * Px);
    vB.x += _invMassB * Px;
    vB.y += _invMassB * Py;
    wB += _invIB * (_rB.x * Py - _rB.y * Px);

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    if (_frequencyHz > 0.0) {
      return true;
    }
    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();
    final Vector2 u = Vector2.zero();

    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
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

    double length = u.normalize();
    double C = length - _length;
    C = MathUtils.clampDouble(
        C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);

    double impulse = -_mass * C;
    double Px = impulse * u.x;
    double Py = impulse * u.y;

    cA.x -= _invMassA * Px;
    cA.y -= _invMassA * Py;
    aA -= _invIA * (rA.x * Py - rA.y * Px);
    cB.x += _invMassB * Px;
    cB.y += _invMassB * Py;
    aB += _invIB * (rB.x * Py - rB.y * Px);

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return C.abs() < Settings.linearSlop;
  }
}
