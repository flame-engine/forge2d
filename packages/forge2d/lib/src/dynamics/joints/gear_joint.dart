import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

//Gear Joint:
//C0 = (coordinate1 + ratio * coordinate2)_initial
//C = (coordinate1 + ratio * coordinate2) - C0 = 0
//J = [J1 ratio * J2]
//K = J * invM * JT
//= J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
//Revolute:
//coordinate = rotation
//Cdot = angularVelocity
//J = [0 0 1]
//K = J * invM * JT = invI
//
//Prismatic:
//coordinate = dot(p - pg, ug)
//Cdot = dot(v + cross(w, r), ug)
//J = [ug cross(r, ug)]
//K = J * invM * JT = invMass + invI * cross(r, ug)^2

/// A gear joint is used to connect two joints together. Either joint can be a
/// revolute or prismatic joint. You specify a gear ratio to bind the motion
/// together: coordinate1 + ratio * coordinate2 =
/// constant The ratio can be negative or positive. If one joint is a revolute
/// joint and the other joint is a prismatic joint, then the ratio will have
/// units of length or units of 1/length.
///
/// Warning: The revolute and prismatic joints must be attached to fixed bodies
///          (which must be body1 on those joints).
/// Warning: You have to manually destroy the gear joint if joint1 or joint2 is
///          destroyed.
class GearJoint extends Joint {
  final Joint _joint1;
  final Joint _joint2;

  // Body A is connected to body C
  // Body B is connected to body D
  final Body _bodyC;
  final Body _bodyD;

  // Solver shared
  final Vector2 _localAnchorC = Vector2.zero();
  final Vector2 _localAnchorD = Vector2.zero();

  final Vector2 _localAxisC = Vector2.zero();
  final Vector2 _localAxisD = Vector2.zero();

  double _referenceAngleA = 0.0;
  double _referenceAngleB = 0.0;

  double _constant = 0.0;
  double ratio = 0.0;

  double _impulse = 0.0;

  // Solver temp
  int _indexA = 0;
  int _indexB = 0;
  int _indexC = 0;
  int _indexD = 0;
  final Vector2 _lcA = Vector2.zero();
  final Vector2 _lcB = Vector2.zero();
  final Vector2 _lcC = Vector2.zero();
  final Vector2 _lcD = Vector2.zero();
  double _mA = 0.0;
  double _mB = 0.0;
  double _mC = 0.0;
  double _mD = 0.0;
  double _iA = 0.0;
  double _iB = 0.0;
  double _iC = 0.0;
  double _iD = 0.0;
  final Vector2 _jvAC = Vector2.zero();
  final Vector2 _jvBD = Vector2.zero();
  double _jwA = 0.0;
  double _jwB = 0.0;
  double _jwC = 0.0;
  double _jwD = 0.0;
  double _mass = 0.0;

  GearJoint(GearJointDef def)
      : _joint1 = def.joint1,
        _joint2 = def.joint2,
        _bodyC = def.joint1.bodyA,
        _bodyD = def.joint2.bodyA,
        super(def) {
    assert(
      (_joint1 is RevoluteJoint || _joint1 is PrismaticJoint) &&
          (_joint2 is RevoluteJoint || _joint2 is PrismaticJoint),
      'A GearJoint can only be joint with RevoluteJoint and/or PrismaticJoint.',
    );

    double coordinateA;
    double coordinateB;

    // TODO_ERIN there might be some problem with the joint edges in Joint.
    bodyA = _joint1.bodyB;

    // Get geometry of joint1
    final xfA = bodyA.transform;
    final aA = bodyA.sweep.a;
    final xfC = _bodyC.transform;
    final aC = _bodyC.sweep.a;

    if (_joint1 is RevoluteJoint) {
      final revolute = def.joint1 as RevoluteJoint;
      _localAnchorC.setFrom(revolute.localAnchorA);
      localAnchorA.setFrom(revolute.localAnchorB);
      _referenceAngleA = revolute.referenceAngle;
      _localAxisC.setZero();

      coordinateA = aA - aC - _referenceAngleA;
    } else {
      final pA = Vector2.zero();
      final temp = Vector2.zero();
      final prismatic = def.joint1 as PrismaticJoint;
      _localAnchorC.setFrom(prismatic.localAnchorA);
      localAnchorA.setFrom(prismatic.localAnchorB);
      _referenceAngleA = prismatic.referenceAngle;
      _localAxisC.setFrom(prismatic.localXAxisA);

      final pC = _localAnchorC;
      temp
        ..setFrom(Rot.mulVec2(xfA.q, localAnchorA))
        ..add(xfA.p)
        ..sub(xfC.p);
      pA.setFrom(Rot.mulTransVec2(xfC.q, temp));
      coordinateA = (pA..sub(pC)).dot(_localAxisC);
    }

    bodyB = _joint2.bodyB;

    // Get geometry of joint2
    final xfB = bodyB.transform;
    final aB = bodyB.sweep.a;
    final xfD = _bodyD.transform;
    final aD = _bodyD.sweep.a;

    if (_joint2 is RevoluteJoint) {
      final revolute = def.joint2 as RevoluteJoint;
      _localAnchorD.setFrom(revolute.localAnchorA);
      localAnchorB.setFrom(revolute.localAnchorB);
      _referenceAngleB = revolute.referenceAngle;
      _localAxisD.setZero();

      coordinateB = aB - aD - _referenceAngleB;
    } else {
      final pB = Vector2.zero();
      final temp = Vector2.zero();
      final prismatic = def.joint2 as PrismaticJoint;
      _localAnchorD.setFrom(prismatic.localAnchorA);
      localAnchorB.setFrom(prismatic.localAnchorB);
      _referenceAngleB = prismatic.referenceAngle;
      _localAxisD.setFrom(prismatic.localXAxisA);

      final pD = _localAnchorD;
      temp
        ..setFrom(Rot.mulVec2(xfB.q, localAnchorB))
        ..add(xfB.p)
        ..sub(xfD.p);
      pB.setFrom(Rot.mulTransVec2(xfD.q, temp));
      coordinateB = (pB..sub(pD)).dot(_localAxisD);
    }

    ratio = def.ratio;
    _constant = coordinateA + ratio * coordinateB;
    _impulse = 0.0;
  }

  /// Get the reaction force given the inverse time step. Unit is N.
  @override
  Vector2 reactionForce(double invDt) {
    return Vector2.copy(_jvAC)
      ..scale(_impulse)
      ..scale(invDt);
  }

  @override
  double reactionTorque(double invDt) {
    return invDt * _impulse * _jwA;
  }

  @override
  void initVelocityConstraints(SolverData data) {
    _indexA = bodyA.islandIndex;
    _indexB = bodyB.islandIndex;
    _indexC = _bodyC.islandIndex;
    _indexD = _bodyD.islandIndex;
    _lcA.setFrom(bodyA.sweep.localCenter);
    _lcB.setFrom(bodyB.sweep.localCenter);
    _lcC.setFrom(_bodyC.sweep.localCenter);
    _lcD.setFrom(_bodyD.sweep.localCenter);
    _mA = bodyA.inverseMass;
    _mB = bodyB.inverseMass;
    _mC = _bodyC.inverseMass;
    _mD = _bodyD.inverseMass;
    _iA = bodyA.inverseInertia;
    _iB = bodyB.inverseInertia;
    _iC = _bodyC.inverseInertia;
    _iD = _bodyD.inverseInertia;

    // Vec2 cA = data.positions[_indexA].c;
    final aA = data.positions[_indexA].a;
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;

    // Vec2 cB = data.positions[_indexB].c;
    final aB = data.positions[_indexB].a;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;

    // Vec2 cC = data.positions[_indexC].c;
    final aC = data.positions[_indexC].a;
    final vC = data.velocities[_indexC].v;
    var wC = data.velocities[_indexC].w;

    // Vec2 cD = data.positions[_indexD].c;
    final aD = data.positions[_indexD].a;
    final vD = data.velocities[_indexD].v;
    var wD = data.velocities[_indexD].w;

    final qA = Rot();
    final qB = Rot();
    final qC = Rot();
    final qD = Rot();
    qA.setAngle(aA);
    qB.setAngle(aB);
    qC.setAngle(aC);
    qD.setAngle(aD);

    _mass = 0.0;

    final temp = Vector2.zero();

    if (_joint1 is RevoluteJoint) {
      _jvAC.setZero();
      _jwA = 1.0;
      _jwC = 1.0;
      _mass += _iA + _iC;
    } else {
      final rC = Vector2.zero();
      final rA = Vector2.zero();
      _jvAC.setFrom(Rot.mulVec2(qC, _localAxisC));
      temp
        ..setFrom(_localAnchorC)
        ..sub(_lcC);
      rC.setFrom(Rot.mulVec2(qC, temp));
      temp
        ..setFrom(localAnchorA)
        ..sub(_lcA);
      rA.setFrom(Rot.mulVec2(qA, temp));
      _jwC = rC.cross(_jvAC);
      _jwA = rA.cross(_jvAC);
      _mass += _mC + _mA + _iC * _jwC * _jwC + _iA * _jwA * _jwA;
    }

    if (_joint2 is RevoluteJoint) {
      _jvBD.setZero();
      _jwB = ratio;
      _jwD = ratio;
      _mass += ratio * ratio * (_iB + _iD);
    } else {
      final u = Vector2.zero();
      final rD = Vector2.zero();
      final rB = Vector2.zero();
      u.setFrom(Rot.mulVec2(qD, _localAxisD));
      temp
        ..setFrom(_localAnchorD)
        ..sub(_lcD);
      rD.setFrom(Rot.mulVec2(qD, temp));
      temp
        ..setFrom(localAnchorB)
        ..sub(_lcB);
      rB.setFrom(Rot.mulVec2(qB, temp));
      _jvBD
        ..setFrom(u)
        ..scale(ratio);
      _jwD = ratio * rD.cross(u);
      _jwB = ratio * rB.cross(u);
      _mass +=
          ratio * ratio * (_mD + _mB) + _iD * _jwD * _jwD + _iB * _jwB * _jwB;
    }

    // Compute effective mass.
    _mass = _mass > 0.0 ? 1.0 / _mass : 0.0;

    if (data.step.warmStarting) {
      vA.x += (_mA * _impulse) * _jvAC.x;
      vA.y += (_mA * _impulse) * _jvAC.y;
      wA += _iA * _impulse * _jwA;

      vB.x += (_mB * _impulse) * _jvBD.x;
      vB.y += (_mB * _impulse) * _jvBD.y;
      wB += _iB * _impulse * _jwB;

      vC.x -= (_mC * _impulse) * _jvAC.x;
      vC.y -= (_mC * _impulse) * _jvAC.y;
      wC -= _iC * _impulse * _jwC;

      vD.x -= (_mD * _impulse) * _jvBD.x;
      vD.y -= (_mD * _impulse) * _jvBD.y;
      wD -= _iD * _impulse * _jwD;
    } else {
      _impulse = 0.0;
    }

    data.velocities[_indexA].w = wA;
    data.velocities[_indexB].w = wB;
    data.velocities[_indexC].w = wC;
    data.velocities[_indexD].w = wD;
  }

  @override
  void solveVelocityConstraints(SolverData data) {
    final vA = data.velocities[_indexA].v;
    var wA = data.velocities[_indexA].w;
    final vB = data.velocities[_indexB].v;
    var wB = data.velocities[_indexB].w;
    final vC = data.velocities[_indexC].v;
    var wC = data.velocities[_indexC].w;
    final vD = data.velocities[_indexD].v;
    var wD = data.velocities[_indexD].w;

    final temp1 = Vector2.zero();
    final temp2 = Vector2.zero();
    var cDot = _jvAC.dot(
          temp1
            ..setFrom(vA)
            ..sub(vC),
        ) +
        _jvBD.dot(
          temp2
            ..setFrom(vB)
            ..sub(vD),
        );
    cDot += (_jwA * wA - _jwC * wC) + (_jwB * wB - _jwD * wD);

    final impulse = -_mass * cDot;
    _impulse += impulse;

    vA.x += (_mA * impulse) * _jvAC.x;
    vA.y += (_mA * impulse) * _jvAC.y;
    wA += _iA * impulse * _jwA;

    vB.x += (_mB * impulse) * _jvBD.x;
    vB.y += (_mB * impulse) * _jvBD.y;
    wB += _iB * impulse * _jwB;

    vC.x -= (_mC * impulse) * _jvAC.x;
    vC.y -= (_mC * impulse) * _jvAC.y;
    wC -= _iC * impulse * _jwC;

    vD.x -= (_mD * impulse) * _jvBD.x;
    vD.y -= (_mD * impulse) * _jvBD.y;
    wD -= _iD * impulse * _jwD;

    // data.velocities[_indexA].v = vA;
    data.velocities[_indexA].w = wA;
    // data.velocities[_indexB].v = vB;
    data.velocities[_indexB].w = wB;
    // data.velocities[_indexC].v = vC;
    data.velocities[_indexC].w = wC;
    // data.velocities[_indexD].v = vD;
    data.velocities[_indexD].w = wD;
  }

  Joint getJoint1() {
    return _joint1;
  }

  Joint getJoint2() {
    return _joint2;
  }

  @override
  bool solvePositionConstraints(SolverData data) {
    final cA = data.positions[_indexA].c;
    var aA = data.positions[_indexA].a;
    final cB = data.positions[_indexB].c;
    var aB = data.positions[_indexB].a;
    final cC = data.positions[_indexC].c;
    var aC = data.positions[_indexC].a;
    final cD = data.positions[_indexD].c;
    var aD = data.positions[_indexD].a;

    final qA = Rot();
    final qB = Rot();
    final qC = Rot();
    final qD = Rot();
    qA.setAngle(aA);
    qB.setAngle(aB);
    qC.setAngle(aC);
    qD.setAngle(aD);

    // TODO(spydon): Is this really needed?
    const linearError = 0.0;

    double coordinateA;
    double coordinateB;

    final temp = Vector2.zero();
    final jvBC = Vector2.zero();
    final jvBD = Vector2.zero();
    double jwA;
    double jwB;
    double jwC;
    double jwD;
    var mass = 0.0;

    if (_joint1 is RevoluteJoint) {
      jvBC.setZero();
      jwA = 1.0;
      jwC = 1.0;
      mass += _iA + _iC;

      coordinateA = aA - aC - _referenceAngleA;
    } else {
      final rC = Vector2.zero();
      final rA = Vector2.zero();
      final pC = Vector2.zero();
      final pA = Vector2.zero();
      jvBC.setFrom(Rot.mulVec2(qC, _localAxisC));
      temp
        ..setFrom(_localAnchorC)
        ..sub(_lcC);
      rC.setFrom(Rot.mulVec2(qC, temp));
      temp
        ..setFrom(localAnchorA)
        ..sub(_lcA);
      rA.setFrom(Rot.mulVec2(qA, temp));
      jwC = rC.cross(jvBC);
      jwA = rA.cross(jvBC);
      mass += _mC + _mA + _iC * jwC * jwC + _iA * jwA * jwA;

      pC
        ..setFrom(_localAnchorC)
        ..sub(_lcC);
      temp
        ..setFrom(rA)
        ..add(cA)
        ..sub(cC);
      pA.setFrom(Rot.mulTransVec2(qC, temp));
      coordinateA = (pA..sub(pC)).dot(_localAxisC);
    }

    if (_joint2 is RevoluteJoint) {
      jvBD.setZero();
      jwB = ratio;
      jwD = ratio;
      mass += ratio * ratio * (_iB + _iD);

      coordinateB = aB - aD - _referenceAngleB;
    } else {
      final u = Vector2.zero();
      final rD = Vector2.zero();
      final rB = Vector2.zero();
      final pD = Vector2.zero();
      final pB = Vector2.zero();
      u.setFrom(Rot.mulVec2(qD, _localAxisD));
      temp
        ..setFrom(_localAnchorD)
        ..sub(_lcD);
      rD.setFrom(Rot.mulVec2(qD, temp));
      temp
        ..setFrom(localAnchorB)
        ..sub(_lcB);
      rB.setFrom(Rot.mulVec2(qB, temp));
      jvBD
        ..setFrom(u)
        ..scale(ratio);
      jwD = rD.cross(u);
      jwB = rB.cross(u);
      mass += ratio * ratio * (_mD + _mB) + _iD * jwD * jwD + _iB * jwB * jwB;

      pD
        ..setFrom(_localAnchorD)
        ..sub(_lcD);
      temp
        ..setFrom(rB)
        ..add(cB)
        ..sub(cD);
      pB.setFrom(Rot.mulTransVec2(qD, pB));
      coordinateB = (pB..sub(pD)).dot(_localAxisD);
    }

    final c = (coordinateA + ratio * coordinateB) - _constant;

    var impulse = 0.0;
    if (mass > 0.0) {
      impulse = -c / mass;
    }

    cA.x += (_mA * impulse) * jvBC.x;
    cA.y += (_mA * impulse) * jvBC.y;
    aA += _iA * impulse * jwA;

    cB.x += (_mB * impulse) * jvBD.x;
    cB.y += (_mB * impulse) * jvBD.y;
    aB += _iB * impulse * jwB;

    cC.x -= (_mC * impulse) * jvBC.x;
    cC.y -= (_mC * impulse) * jvBC.y;
    aC -= _iC * impulse * jwC;

    cD.x -= (_mD * impulse) * jvBD.x;
    cD.y -= (_mD * impulse) * jvBD.y;
    aD -= _iD * impulse * jwD;

    // data.positions[_indexA].c = cA;
    data.positions[_indexA].a = aA;
    // data.positions[_indexB].c = cB;
    data.positions[_indexB].a = aB;
    // data.positions[_indexC].c = cC;
    data.positions[_indexC].a = aC;
    // data.positions[_indexD].c = cD;
    data.positions[_indexD].a = aD;

    // TODO_ERIN not implemented
    return linearError < settings.linearSlop;
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
