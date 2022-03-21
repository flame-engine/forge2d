import '../../../forge2d.dart';
import '../../settings.dart' as settings;

/// The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a
/// ratio such that: length1 + ratio * length2 <= constant Yes, the force transmitted is scaled by
/// the ratio. Warning: the pulley joint can get a bit squirrelly by itself. They often work better
/// when combined with prismatic joints. You should also cover the the anchor points with static
/// shapes to prevent one side from going to zero length.
class PulleyJoint extends Joint {
  static const double minPulleyLength = 2.0;

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
    final p = Vector2.zero();
    p.setFrom(bodyA.worldPoint(localAnchorA));
    p.sub(_groundAnchorA);
    return p.length;
  }

  double getCurrentLengthB() {
    final p = Vector2.zero();
    p.setFrom(bodyB.worldPoint(localAnchorB));
    p.sub(_groundAnchorB);
    return p.length;
  }

  @override
  Vector2 reactionForce(double invDt) {
    return Vector2.copy(_uB)
      ..scale(_impulse)
      ..scale(invDt);
  }

  @override
  double reactionTorque(double invDt) {
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

    final cA = data.positions[_indexA].c;
    final aA = data.positions[_indexA].a;
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;

    final cB = data.positions[_indexB].c;
    final aB = data.positions[_indexB].a;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    final qA = Rot();
    final qB = Rot();
    final temp = Vector2.zero();

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

    final lengthA = _uA.length;
    final lengthB = _uB.length;

    if (lengthA > 10.0 * settings.linearSlop) {
      _uA.scale(1.0 / lengthA);
    } else {
      _uA.setZero();
    }

    if (lengthB > 10.0 * settings.linearSlop) {
      _uB.scale(1.0 / lengthB);
    } else {
      _uB.setZero();
    }

    // Compute effective mass.
    final ruA = _rA.cross(_uA);
    final ruB = _rB.cross(_uB);

    final mA = _invMassA + _invIA * ruA * ruA;
    final mB = _invMassB + _invIB * ruB * ruB;

    _mass = mA + _ratio * _ratio * mB;

    if (_mass > 0.0) {
      _mass = 1.0 / _mass;
    }

    if (data.step.warmStarting) {
      // Scale impulses to support variable time steps.
      _impulse *= data.step.dtRatio;

      // Warm starting.
      final pA = Vector2.zero();
      final pB = Vector2.zero();

      pA
        ..setFrom(_uA)
        ..scale(-_impulse);
      pB
        ..setFrom(_uB)
        ..scale(-_ratio * _impulse);

      vA.x += _invMassA * pA.x;
      vA.y += _invMassA * pA.y;
      wA += _invIA * _rA.cross(pA);
      vB.x += _invMassB * pB.x;
      vB.y += _invMassB * pB.y;
      wB += _invIB * _rB.cross(pB);
    } else {
      _impulse = 0.0;
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

    final vpA = Vector2.zero();
    final vpB = Vector2.zero();
    final pA = Vector2.zero();
    final pB = Vector2.zero();

    _rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    final cDot = -_uA.dot(vpA) - _ratio * _uB.dot(vpB);
    final impulse = -_mass * cDot;
    _impulse += impulse;

    pA
      ..setFrom(_uA)
      ..scale(-impulse);
    pB
      ..setFrom(_uB)
      ..scale(-_ratio * impulse);
    vA.x += _invMassA * pA.x;
    vA.y += _invMassA * pA.y;
    wA += _invIA * _rA.cross(pA);
    vB.x += _invMassB * pB.x;
    vB.y += _invMassB * pB.y;
    wB += _invIB * _rB.cross(pB);

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
  }

  @override
  bool solvePositionConstraints(final SolverData data) {
    final qA = Rot();
    final qB = Rot();
    final rA = Vector2.zero();
    final rB = Vector2.zero();
    final uA = Vector2.zero();
    final uB = Vector2.zero();
    final temp = Vector2.zero();
    final pA = Vector2.zero();
    final pB = Vector2.zero();

    final cA = data.positions[_indexA].c;
    var aA = data.positions[_indexA].a;
    final cB = data.positions[_indexB].c;
    var aB = data.positions[_indexB].a;

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

    final lengthA = uA.length;
    final lengthB = uB.length;

    if (lengthA > 10.0 * settings.linearSlop) {
      uA.scale(1.0 / lengthA);
    } else {
      uA.setZero();
    }

    if (lengthB > 10.0 * settings.linearSlop) {
      uB.scale(1.0 / lengthB);
    } else {
      uB.setZero();
    }

    // Compute effective mass.
    final ruA = rA.cross(uA);
    final ruB = rB.cross(uB);

    final mA = _invMassA + _invIA * ruA * ruA;
    final mB = _invMassB + _invIB * ruB * ruB;

    var mass = mA + _ratio * _ratio * mB;

    if (mass > 0.0) {
      mass = 1.0 / mass;
    }

    final c = _constant - lengthA - _ratio * lengthB;
    final linearError = c.abs();

    final impulse = -mass * c;

    pA
      ..setFrom(uA)
      ..scale(-impulse);
    pB
      ..setFrom(uB)
      ..scale(-_ratio * impulse);

    cA.x += _invMassA * pA.x;
    cA.y += _invMassA * pA.y;
    aA += _invIA * rA.cross(pA);
    cB.x += _invMassB * pB.x;
    cB.y += _invMassB * pB.y;
    aB += _invIB * rB.cross(pB);

    data.positions[_indexA].a = aA;
    data.positions[_indexB].a = aB;

    return linearError < settings.linearSlop;
  }
}
