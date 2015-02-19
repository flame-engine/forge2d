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

  final Vec2 _m_groundAnchorA = new Vec2.zero();
  final Vec2 _m_groundAnchorB = new Vec2.zero();
  double _m_lengthA = 0.0;
  double _m_lengthB = 0.0;

  // Solver shared
  final Vec2 _m_localAnchorA = new Vec2.zero();
  final Vec2 _m_localAnchorB = new Vec2.zero();
  double _m_constant = 0.0;
  double _m_ratio = 0.0;
  double _m_impulse = 0.0;

  // Solver temp
  int _m_indexA = 0;
  int _m_indexB = 0;
  final Vec2 _m_uA = new Vec2.zero();
  final Vec2 _m_uB = new Vec2.zero();
  final Vec2 _m_rA = new Vec2.zero();
  final Vec2 _m_rB = new Vec2.zero();
  final Vec2 _m_localCenterA = new Vec2.zero();
  final Vec2 _m_localCenterB = new Vec2.zero();
  double _m_invMassA = 0.0;
  double _m_invMassB = 0.0;
  double _m_invIA = 0.0;
  double _m_invIB = 0.0;
  double _m_mass = 0.0;

  PulleyJoint(IWorldPool argWorldPool, PulleyJointDef def)
      : super(argWorldPool, def) {
    _m_groundAnchorA.set(def.groundAnchorA);
    _m_groundAnchorB.set(def.groundAnchorB);
    _m_localAnchorA.set(def.localAnchorA);
    _m_localAnchorB.set(def.localAnchorB);

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
    final Vec2 p = pool.popVec2();
    m_bodyA.getWorldPointToOut(_m_localAnchorA, p);
    p.subLocal(_m_groundAnchorA);
    double length = p.length();
    pool.pushVec2(1);
    return length;
  }

  double getCurrentLengthB() {
    final Vec2 p = pool.popVec2();
    m_bodyB.getWorldPointToOut(_m_localAnchorB, p);
    p.subLocal(_m_groundAnchorB);
    double length = p.length();
    pool.pushVec2(1);
    return length;
  }


  Vec2 getLocalAnchorA() {
    return _m_localAnchorA;
  }

  Vec2 getLocalAnchorB() {
    return _m_localAnchorB;
  }



  void getAnchorA(Vec2 argOut) {
    m_bodyA.getWorldPointToOut(_m_localAnchorA, argOut);
  }


  void getAnchorB(Vec2 argOut) {
    m_bodyB.getWorldPointToOut(_m_localAnchorB, argOut);
  }


  void getReactionForce(double inv_dt, Vec2 argOut) {
    argOut.set(_m_uB).mulLocal(_m_impulse).mulLocal(inv_dt);
  }


  double getReactionTorque(double inv_dt) {
    return 0.0;
  }

  Vec2 getGroundAnchorA() {
    return _m_groundAnchorA;
  }

  Vec2 getGroundAnchorB() {
    return _m_groundAnchorB;
  }

  double getLength1() {
    final Vec2 p = pool.popVec2();
    m_bodyA.getWorldPointToOut(_m_localAnchorA, p);
    p.subLocal(_m_groundAnchorA);

    double len = p.length();
    pool.pushVec2(1);
    return len;
  }

  double getLength2() {
    final Vec2 p = pool.popVec2();
    m_bodyB.getWorldPointToOut(_m_localAnchorB, p);
    p.subLocal(_m_groundAnchorB);

    double len = p.length();
    pool.pushVec2(1);
    return len;
  }

  double getRatio() {
    return _m_ratio;
  }


  void initVelocityConstraints(final SolverData data) {
    _m_indexA = m_bodyA.m_islandIndex;
    _m_indexB = m_bodyB.m_islandIndex;
    _m_localCenterA.set(m_bodyA.m_sweep.localCenter);
    _m_localCenterB.set(m_bodyB.m_sweep.localCenter);
    _m_invMassA = m_bodyA.m_invMass;
    _m_invMassB = m_bodyB.m_invMass;
    _m_invIA = m_bodyA.m_invI;
    _m_invIB = m_bodyB.m_invI;

    Vec2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    Vec2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;

    Vec2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;
    Vec2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 temp = pool.popVec2();

    qA.setAngle(aA);
    qB.setAngle(aB);

    // Compute the effective masses.
    Rot.mulToOutUnsafe(qA, temp.set(_m_localAnchorA).subLocal(_m_localCenterA), _m_rA);
    Rot.mulToOutUnsafe(qB, temp.set(_m_localAnchorB).subLocal(_m_localCenterB), _m_rB);

    _m_uA.set(cA).addLocal(_m_rA).subLocal(_m_groundAnchorA);
    _m_uB.set(cB).addLocal(_m_rB).subLocal(_m_groundAnchorB);

    double lengthA = _m_uA.length();
    double lengthB = _m_uB.length();

    if (lengthA > 10.0 * Settings.linearSlop) {
      _m_uA.mulLocal(1.0 / lengthA);
    } else {
      _m_uA.setZero();
    }

    if (lengthB > 10.0 * Settings.linearSlop) {
      _m_uB.mulLocal(1.0 / lengthB);
    } else {
      _m_uB.setZero();
    }

    // Compute effective mass.
    double ruA = Vec2.cross(_m_rA, _m_uA);
    double ruB = Vec2.cross(_m_rB, _m_uB);

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
      final Vec2 PA = pool.popVec2();
      final Vec2 PB = pool.popVec2();

      PA.set(_m_uA).mulLocal(-_m_impulse);
      PB.set(_m_uB).mulLocal(-_m_ratio * _m_impulse);

      vA.x += _m_invMassA * PA.x;
      vA.y += _m_invMassA * PA.y;
      wA += _m_invIA * Vec2.cross(_m_rA, PA);
      vB.x += _m_invMassB * PB.x;
      vB.y += _m_invMassB * PB.y;
      wB += _m_invIB * Vec2.cross(_m_rB, PB);

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
    Vec2 vA = data.velocities[_m_indexA].v;
    double wA = data.velocities[_m_indexA].w;
    Vec2 vB = data.velocities[_m_indexB].v;
    double wB = data.velocities[_m_indexB].w;

    final Vec2 vpA = pool.popVec2();
    final Vec2 vpB = pool.popVec2();
    final Vec2 PA = pool.popVec2();
    final Vec2 PB = pool.popVec2();

    Vec2.crossToOutUnsafeDblVec2(wA, _m_rA, vpA);
    vpA.addLocal(vA);
    Vec2.crossToOutUnsafeDblVec2(wB, _m_rB, vpB);
    vpB.addLocal(vB);

    double Cdot = -Vec2.dot(_m_uA, vpA) - _m_ratio * Vec2.dot(_m_uB, vpB);
    double impulse = -_m_mass * Cdot;
    _m_impulse += impulse;

    PA.set(_m_uA).mulLocal(-impulse);
    PB.set(_m_uB).mulLocal(-_m_ratio * impulse);
    vA.x += _m_invMassA * PA.x;
    vA.y += _m_invMassA * PA.y;
    wA += _m_invIA * Vec2.cross(_m_rA, PA);
    vB.x += _m_invMassB * PB.x;
    vB.y += _m_invMassB * PB.y;
    wB += _m_invIB * Vec2.cross(_m_rB, PB);

//    data.velocities[m_indexA].v.set(vA);
    data.velocities[_m_indexA].w = wA;
//    data.velocities[m_indexB].v.set(vB);
    data.velocities[_m_indexB].w = wB;

    pool.pushVec2(4);
  }


  bool solvePositionConstraints(final SolverData data) {
    final Rot qA = pool.popRot();
    final Rot qB = pool.popRot();
    final Vec2 rA = pool.popVec2();
    final Vec2 rB = pool.popVec2();
    final Vec2 uA = pool.popVec2();
    final Vec2 uB = pool.popVec2();
    final Vec2 temp = pool.popVec2();
    final Vec2 PA = pool.popVec2();
    final Vec2 PB = pool.popVec2();

    Vec2 cA = data.positions[_m_indexA].c;
    double aA = data.positions[_m_indexA].a;
    Vec2 cB = data.positions[_m_indexB].c;
    double aB = data.positions[_m_indexB].a;

    qA.setAngle(aA);
    qB.setAngle(aB);

    Rot.mulToOutUnsafe(qA, temp.set(_m_localAnchorA).subLocal(_m_localCenterA), rA);
    Rot.mulToOutUnsafe(qB, temp.set(_m_localAnchorB).subLocal(_m_localCenterB), rB);

    uA.set(cA).addLocal(rA).subLocal(_m_groundAnchorA);
    uB.set(cB).addLocal(rB).subLocal(_m_groundAnchorB);

    double lengthA = uA.length();
    double lengthB = uB.length();

    if (lengthA > 10.0 * Settings.linearSlop) {
      uA.mulLocal(1.0 / lengthA);
    } else {
      uA.setZero();
    }

    if (lengthB > 10.0 * Settings.linearSlop) {
      uB.mulLocal(1.0 / lengthB);
    } else {
      uB.setZero();
    }

    // Compute effective mass.
    double ruA = Vec2.cross(rA, uA);
    double ruB = Vec2.cross(rB, uB);

    double mA = _m_invMassA + _m_invIA * ruA * ruA;
    double mB = _m_invMassB + _m_invIB * ruB * ruB;

    double mass = mA + _m_ratio * _m_ratio * mB;

    if (mass > 0.0) {
      mass = 1.0 / mass;
    }

    double C = _m_constant - lengthA - _m_ratio * lengthB;
    double linearError = C.abs();

    double impulse = -mass * C;

    PA.set(uA).mulLocal(-impulse);
    PB.set(uB).mulLocal(-_m_ratio * impulse);

    cA.x += _m_invMassA * PA.x;
    cA.y += _m_invMassA * PA.y;
    aA += _m_invIA * Vec2.cross(rA, PA);
    cB.x += _m_invMassB * PB.x;
    cB.y += _m_invMassB * PB.y;
    aB += _m_invIB * Vec2.cross(rB, PB);

//    data.positions[m_indexA].c.set(cA);
    data.positions[_m_indexA].a = aA;
//    data.positions[m_indexB].c.set(cB);
    data.positions[_m_indexB].a = aB;

    pool.pushRot(2);
    pool.pushVec2(7);

    return linearError < Settings.linearSlop;
  }
}
