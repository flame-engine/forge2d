part of box2d;

/// The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a
/// ratio such that: length1 + ratio * length2 <= constant Yes, the force transmitted is scaled by
/// the ratio. Warning: the pulley joint can get a bit squirrelly by itself. They often work better
/// when combined with prismatic joints. You should also cover the the anchor points with static
/// shapes to prevent one side from going to zero length.
class PulleyJoint extends Joint {
  static const double MIN_PULLEY_LENGTH = 2.0;

  final Vector2 _groundAnchorA = Vector2.zero();
  final Vector2 _groundAnchorB = Vector2.zero();
  double _lengthA = 0.0;
  double _lengthB = 0.0;

  // Solver shared
  double _constant = 0.0;
  double _ratio = 0.0;
  double _impulse = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  final Vector2 _uA = Vector2.zero();
  final Vector2 _uB = Vector2.zero();
  final Vector2 _rA = Vector2.zero();
  final Vector2 _rB = Vector2.zero();
  final Vector2 _localCenterA = Vector2.zero();
  final Vector2 _localCenterB = Vector2.zero();
  double _invMassA = 0.0;
  double _invMassB = 0.0;
  double _invIA = 0.0;
  double _invIB = 0.0;
  double _mass = 0.0;

  PulleyJoint(PulleyJointDef def) : super(def) {
    _groundAnchorA.setFrom(def.groundAnchorA);
    _groundAnchorB.setFrom(def.groundAnchorB);
    localAnchorA.setFrom(def.localAnchorA);
    localAnchorB.setFrom(def.localAnchorB);

    assert(def.ratio != 0.0);
    _ratio = def.ratio;

    _lengthA = def.lengthA;
    _lengthB = def.lengthB;

    _constant = def.lengthA + _ratio * def.lengthB;
    _impulse = 0.0;
  }

  double getLengthA() {
    return _lengthA;
  }

  double getLengthB() {
    return _lengthB;
  }

  double getCurrentLengthA() {
    final Vector2 p = Vector2.zero();
    p.setFrom(_bodyA.getWorldPoint(localAnchorA));
    p.sub(_groundAnchorA);
    double length = p.length;
    return length;
  }

  double getCurrentLengthB() {
    final Vector2 p = Vector2.zero();
    p.setFrom(_bodyB.getWorldPoint(localAnchorB));
    p.sub(_groundAnchorB);
    double length = p.length;
    return length;
  }

  @override
  Vector2 getReactionForce(double inv_dt) {
    return Vector2.copy(_uB)..scale(_impulse)..scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  Vector2 getGroundAnchorA() {
    return _groundAnchorA;
  }

  Vector2 getGroundAnchorB() {
    return _groundAnchorB;
  }

  double getRatio() {
    return _ratio;
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

    _uA
      ..setFrom(cA)
      ..add(_rA)
      ..sub(_groundAnchorA);
    _uB
      ..setFrom(cB)
      ..add(_rB)
      ..sub(_groundAnchorB);

    double lengthA = _uA.length;
    double lengthB = _uB.length;

    if (lengthA > 10.0 * Settings.linearSlop) {
      _uA.scale(1.0 / lengthA);
    } else {
      _uA.setZero();
    }

    if (lengthB > 10.0 * Settings.linearSlop) {
      _uB.scale(1.0 / lengthB);
    } else {
      _uB.setZero();
    }

    // Compute effective mass.
    double ruA = _rA.cross(_uA);
    double ruB = _rB.cross(_uB);

    double mA = _invMassA + _invIA * ruA * ruA;
    double mB = _invMassB + _invIB * ruB * ruB;

    _mass = mA + _ratio * _ratio * mB;

    if (_mass > 0.0) {
      _mass = 1.0 / _mass;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support variable time steps.
      _impulse *= data.step.dtRatio;

      // Warm starting.
      final Vector2 PA = Vector2.zero();
      final Vector2 PB = Vector2.zero();

      PA
        ..setFrom(_uA)
        ..scale(-_impulse);
      PB
        ..setFrom(_uB)
        ..scale(-_ratio * _impulse);

      vA.x += _invMassA * PA.x;
      vA.y += _invMassA * PA.y;
      wA += _invIA * _rA.cross(PA);
      vB.x += _invMassB * PB.x;
      vB.y += _invMassB * PB.y;
      wB += _invIB * _rB.cross(PB);
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
    final Vector2 PA = Vector2.zero();
    final Vector2 PB = Vector2.zero();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    double Cdot = -_uA.dot(vpA) - _ratio * _uB.dot(vpB);
    double impulse = -_mass * Cdot;
    _impulse += impulse;

    PA
      ..setFrom(_uA)
      ..scale(-impulse);
    PB
      ..setFrom(_uB)
      ..scale(-_ratio * impulse);
    vA.x += _invMassA * PA.x;
    vA.y += _invMassA * PA.y;
    wA += _invIA * _rA.cross(PA);
    vB.x += _invMassB * PB.x;
    vB.y += _invMassB * PB.y;
    wB += _invIB * _rB.cross(PB);

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = Rot();
    final Rot qB = Rot();
    final Vector2 rA = Vector2.zero();
    final Vector2 rB = Vector2.zero();
    final Vector2 uA = Vector2.zero();
    final Vector2 uB = Vector2.zero();
    final Vector2 temp = Vector2.zero();
    final Vector2 PA = Vector2.zero();
    final Vector2 PB = Vector2.zero();

    Vector2 cA = data.positions[_indexA].c;
    double aA = data.positions[_indexA].a;
    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);
    temp
      ..setFrom(localAnchorA)
      ..sub(_localCenterA);
    rA.setFrom(Rot.mulVec2(qA, temp));
    temp
      ..setFrom(localAnchorB)
      ..sub(_localCenterB);
    rB.setFrom(Rot.mulVec2(qB, temp));

    uA
      ..setFrom(cA)
      ..add(rA)
      ..sub(_groundAnchorA);
    uB
      ..setFrom(cB)
      ..add(rB)
      ..sub(_groundAnchorB);

    double lengthA = uA.length;
    double lengthB = uB.length;

    if (lengthA > 10.0 * Settings.linearSlop) {
      uA.scale(1.0 / lengthA);
    } else {
      uA.setZero();
    }

    if (lengthB > 10.0 * Settings.linearSlop) {
      uB.scale(1.0 / lengthB);
    } else {
      uB.setZero();
    }

    // Compute effective mass.
    double ruA = rA.cross(uA);
    double ruB = rB.cross(uB);

    double mA = _invMassA + _invIA * ruA * ruA;
    double mB = _invMassB + _invIB * ruB * ruB;

    double mass = mA + _ratio * _ratio * mB;

    if (mass > 0.0) {
      mass = 1.0 / mass;
    }

    double C = _constant - lengthA - _ratio * lengthB;
    double linearError = C.abs();

    double impulse = -mass * C;

    PA
      ..setFrom(uA)
      ..scale(-impulse);
    PB
      ..setFrom(uB)
      ..scale(-_ratio * impulse);

    cA.x += _invMassA * PA.x;
    cA.y += _invMassA * PA.y;
    aA += _invIA * rA.cross(PA);
    cB.x += _invMassB * PB.x;
    cB.y += _invMassB * PB.y;
    aB += _invIB * rB.cross(PB);

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return linearError < Settings.linearSlop;
  }
}
