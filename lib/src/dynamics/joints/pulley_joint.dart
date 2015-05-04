/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

/**
 * The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a
 * ratio such that: length1 + ratio * length2 <= constant Yes, the force transmitted is scaled by
 * the ratio. Warning: the pulley joint can get a bit squirrelly by itself. They often work better
 * when combined with prismatic joints. You should also cover the the anchor points with static
 * shapes to prevent one side from going to zero length.
 * 
 * @author Daniel Murphy
 */
class PulleyJoint extends Joint {
  static const double MIN_PULLEY_LENGTH = 2.0;

  final Vector2 _m_groundAnchorA = new Vector2.zero();
  final Vector2 _m_groundAnchorB = new Vector2.zero();
  double _m_lengthA = 0.0;
  double _m_lengthB = 0.0;

  // Solver shared
  final Vector2 _m_localAnchorA = new Vector2.zero();
  final Vector2 _m_localAnchorB = new Vector2.zero();
  double _m_constant = 0.0;
  double _m_ratio = 0.0;
  double _m_impulse = 0.0;

  // Solver temp
  int _m_indexA = 0;
  int _m_indexB = 0;
  final Vector2 _m_uA = new Vector2.zero();
  final Vector2 _m_uB = new Vector2.zero();
  final Vector2 _m_rA = new Vector2.zero();
  final Vector2 _m_rB = new Vector2.zero();
  final Vector2 _m_localCenterA = new Vector2.zero();
  final Vector2 _m_localCenterB = new Vector2.zero();
  double _m_invMassA = 0.0;
  double _m_invMassB = 0.0;
  double _m_invIA = 0.0;
  double _m_invIB = 0.0;
  double _m_mass = 0.0;

  PulleyJoint(IWorldPool argWorldPool, PulleyJointDef def)
      : super(argWorldPool, def) {
    _m_groundAnchorA.setFrom(def.groundAnchorA);
    _m_groundAnchorB.setFrom(def.groundAnchorB);
    _m_localAnchorA.setFrom(def.localAnchorA);
    _m_localAnchorB.setFrom(def.localAnchorB);

    assert(def.ratio != 0.0);
    _m_ratio = def.ratio;

    _m_lengthA = def.lengthA;
    _m_lengthB = def.lengthB;

    _m_constant = def.lengthA + _m_ratio * def.lengthB;
    _m_impulse = 0.0;
  }

  double getLengthA() {
    return _m_lengthA;
  }

  double getLengthB() {
    return _m_lengthB;
  }

  double getCurrentLengthA() {
    final Vector2 p = pool.popVec2();
    m_bodyA.getWorldPointToOut(_m_localAnchorA, p);
    p.sub(_m_groundAnchorA);
    double length = p.length;
    pool.pushVec2(1);
    return length;
  }

  double getCurrentLengthB() {
    final Vector2 p = pool.popVec2();
    m_bodyB.getWorldPointToOut(_m_localAnchorB, p);
    p.sub(_m_groundAnchorB);
    double length = p.length;
    pool.pushVec2(1);
    return length;
  }

  Vector2 getLocalAnchorA() {
    return _m_localAnchorA;
  }

  Vector2 getLocalAnchorB() {
    return _m_localAnchorB;
  }

  void getAnchorA(Vector2 argOut) {
    m_bodyA.getWorldPointToOut(_m_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    m_bodyB.getWorldPointToOut(_m_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut.setFrom(_m_uB).scale(_m_impulse).scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  Vector2 getGroundAnchorA() {
    return _m_groundAnchorA;
  }

  Vector2 getGroundAnchorB() {
    return _m_groundAnchorB;
  }

  double getLength1() {
    final Vector2 p = pool.popVec2();
    m_bodyA.getWorldPointToOut(_m_localAnchorA, p);
    p.sub(_m_groundAnchorA);

    double len = p.length;
    pool.pushVec2(1);
    return len;
  }

  double getLength2() {
    final Vector2 p = pool.popVec2();
    m_bodyB.getWorldPointToOut(_m_localAnchorB, p);
    p.sub(_m_groundAnchorB);

    double len = p.length;
    pool.pushVec2(1);
    return len;
  }

  double getRatio() {
    return _m_ratio;
  }

  void initVelocityConstraints(final SolverData data) {
    _m_indexA = m_bodyA.islandIndex;
    _m_indexB = m_bodyB.islandIndex;
    _m_localCenterA.setFrom(m_bodyA.sweep.localCenter);
    _m_localCenterB.setFrom(m_bodyB.sweep.localCenter);
    _m_invMassA = m_bodyA.invMass;
    _m_invMassB = m_bodyB.invMass;
    _m_invIA = m_bodyA.invI;
    _m_invIB = m_bodyB.invI;

    Vector2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    Vector2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;

    Vector2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;
    Vector2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(
        qA, temp.setFrom(_m_localAnchorA).sub(_m_localCenterA), _m_rA);
    Rot.mulToOutUnsafe(
        qB, temp.setFrom(_m_localAnchorB).sub(_m_localCenterB), _m_rB);

    _m_uA.setFrom(cA).add(_m_rA).sub(_m_groundAnchorA);
    _m_uB.setFrom(cB).add(_m_rB).sub(_m_groundAnchorB);

    double lengthA = _m_uA.length;
    double lengthB = _m_uB.length;

    if (lengthA > 10.0 * Settings.linearSlop) {
      _m_uA.scale(1.0 / lengthA);
    } else {
      _m_uA.setZero();
    }

    if (lengthB > 10.0 * Settings.linearSlop) {
      _m_uB.scale(1.0 / lengthB);
    } else {
      _m_uB.setZero();
    }

    // Compute effective mass.
    double ruA = _m_rA.cross(_m_uA);
    double ruB = _m_rB.cross(_m_uB);

    double mA = _m_invMassA + _m_invIA * ruA * ruA;
    double mB = _m_invMassB + _m_invIB * ruB * ruB;

    _m_mass = mA + _m_ratio * _m_ratio * mB;

    if (_m_mass > 0.0) {
      _m_mass = 1.0 / _m_mass;
    }

    if (data.step.warmStarting) {

      // Scale impulses to support variable time steps.
      _m_impulse *= data.step.dtRatio;

      // Warm starting.
      final Vector2 PA = pool.popVec2();
      final Vector2 PB = pool.popVec2();

      PA.setFrom(_m_uA).scale(-_m_impulse);
      PB.setFrom(_m_uB).scale(-_m_ratio * _m_impulse);

      vA.x += _m_invMassA * PA.x;
      vA.y += _m_invMassA * PA.y;
      wA += _m_invIA * _m_rA.cross(PA);
      vB.x += _m_invMassB * PB.x;
      vB.y += _m_invMassB * PB.y;
      wB += _m_invIB * _m_rB.cross(PB);

      pool.pushVec2(2);
    } else {
      _m_impulse = 0.0;
    }
//    data.velocities[m_indexA].v.set(vA);
    data.velocities[_m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushRot(2);
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;
    Vector2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Vector2 vpA = pool.popVec2();
    final Vector2 vpB = pool.popVec2();
    final Vector2 PA = pool.popVec2();
    final Vector2 PB = pool.popVec2();

    _m_rA.scaleOrthogonalInto(wA, vpA);
    vpA.add(vA);
    _m_rB.scaleOrthogonalInto(wB, vpB);
    vpB.add(vB);

    double Cdot = -_m_uA.dot(vpA) - _m_ratio * _m_uB.dot(vpB);
    double impulse = -_m_mass * Cdot;
    _m_impulse += impulse;

    PA.setFrom(_m_uA).scale(-impulse);
    PB.setFrom(_m_uB).scale(-_m_ratio * impulse);
    vA.x += _m_invMassA * PA.x;
    vA.y += _m_invMassA * PA.y;
    wA += _m_invIA * _m_rA.cross(PA);
    vB.x += _m_invMassB * PB.x;
    vB.y += _m_invMassB * PB.y;
    wB += _m_invIB * _m_rB.cross(PB);

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[_m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;

    pool.pushVec2(4);
  }

  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vector2 rA = pool.popVec2();
    final Vector2 rB = pool.popVec2();
    final Vector2 uA = pool.popVec2();
    final Vector2 uB = pool.popVec2();
    final Vector2 temp = pool.popVec2();
    final Vector2 PA = pool.popVec2();
    final Vector2 PB = pool.popVec2();

    Vector2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    Vector2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    Rot.mulToOutUnsafe(
        qA, temp.setFrom(_m_localAnchorA).sub(_m_localCenterA), rA);
    Rot.mulToOutUnsafe(
        qB, temp.setFrom(_m_localAnchorB).sub(_m_localCenterB), rB);

    uA.setFrom(cA).add(rA).sub(_m_groundAnchorA);
    uB.setFrom(cB).add(rB).sub(_m_groundAnchorB);

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

    double mA = _m_invMassA + _m_invIA * ruA * ruA;
    double mB = _m_invMassB + _m_invIB * ruB * ruB;

    double mass = mA + _m_ratio * _m_ratio * mB;

    if (mass > 0.0) {
      mass = 1.0 / mass;
    }

    double C = _m_constant - lengthA - _m_ratio * lengthB;
    double linearError = C.abs();

    double impulse = -mass * C;

    PA.setFrom(uA).scale(-impulse);
    PB.setFrom(uB).scale(-_m_ratio * impulse);

    cA.x += _m_invMassA * PA.x;
    cA.y += _m_invMassA * PA.y;
    aA += _m_invIA * rA.cross(PA);
    cB.x += _m_invMassB * PB.x;
    cB.y += _m_invMassB * PB.y;
    aB += _m_invIB * rB.cross(PB);

//    data.positions[m_indexA].c.set(cA);
    data.positions[_m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
    data.positions[_m_indexB].a = aB;

    pool.pushRot(2);
    pool.pushVec2(7);

    return linearError < Settings.linearSlop;
  }
}
