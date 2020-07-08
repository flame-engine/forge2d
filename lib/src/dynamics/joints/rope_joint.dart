part of box2d;

/// A rope joint enforces a maximum distance between two points on two bodies. It has no other
/// effect. Warning: if you attempt to change the maximum length during the simulation you will get
/// some non-physical behavior. A model that would allow you to dynamically modify the length would
/// have some sponginess, so I chose not to implement it that way. See DistanceJoint if you want to
/// dynamically control length.
class RopeJoint extends Joint {
  // Solver shared
  final Vector2 localAnchorA = Vector2.zero();
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
  LimitState _state = LimitState.INACTIVE;

  RopeJoint(RopeJointDef def) : super(def) {
    localAnchorA.setFrom(def.localAnchorA);
    localAnchorB.setFrom(def.localAnchorB);

    _maxLength = def.maxLength;
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
    final Vector2 temp = Vector2.zero();

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

    double C = _length - _maxLength;
    if (C > 0.0) {
      _state = LimitState.AT_UPPER;
    } else {
      _state = LimitState.INACTIVE;
    }

    if (_length > Settings.linearSlop) {
      _u.scale(1.0 / _length);
    } else {
      _u.setZero();
      _mass = 0.0;
      _impulse = 0.0;
      return;
    }

    // Compute effective mass.
    double crA = _rA.cross(_u);
    double crB = _rB.cross(_u);
    double invMass =
        _invMassA + _invIA * crA * crA + _invMassB + _invIB * crB * crB;

    _mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      _impulse *= data.step.dtRatio;

      double Px = _impulse * _u.x;
      double Py = _impulse * _u.y;
      vA.x -= _invMassA * Px;
      vA.y -= _invMassA * Py;
      wA -= _invIA * (_rA.x * Py - _rA.y * Px);

      vB.x += _invMassB * Px;
      vB.y += _invMassB * Py;
      wB += _invIB * (_rB.x * Py - _rB.y * Px);
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

    Vector2 vpA = Vector2.zero();
    Vector2 vpB = Vector2.zero();
    Vector2 temp = Vector2.zero();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    double C = _length - _maxLength;
    double Cdot = _u.dot(temp
      ..setFrom(vpB)
      ..sub(vpA));

    // Predictive constraint.
    if (C < 0.0) {
      Cdot += data.step.inv_dt * C;
    }

    double impulse = -_mass * Cdot;
    double oldImpulse = _impulse;
    _impulse = Math.min<double>(0.0, _impulse + impulse);
    impulse = _impulse - oldImpulse;

    double Px = impulse * _u.x;
    double Py = impulse * _u.y;
    vA.x -= _invMassA * Px;
    vA.y -= _invMassA * Py;
    wA -= _invIA * (_rA.x * Py - _rA.y * Px);
    vB.x += _invMassB * Px;
    vB.y += _invMassB * Py;
    wB += _invIB * (_rB.x * Py - _rB.y * Px);

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 u = Vector2.zero();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();
    final Vector2 temp = Vector2.zero();

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

    double length = u.normalize();
    double c = length - _maxLength;

    c = MathUtils.clampDouble(c, 0.0, Settings.maxLinearCorrection);

    double impulse = -_mass * c;
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

    return length - _maxLength < Settings.linearSlop;
  }

  Vector2 getReactionForce(double inv_dt) {
    return Vector2.copy(_u)..scale(inv_dt)..scale(_impulse);
  }

  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  double getMaxLength() {
    return _maxLength;
  }

  void setMaxLength(double maxLength) {
    this._maxLength = maxLength;
  }

  LimitState getLimitState() {
    return _state;
  }
}
