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

/**
 * A gear joint is used to connect two joints together. Either joint can be a revolute or prismatic
 * joint. You specify a gear ratio to bind the motions together: coordinate1 + ratio * coordinate2 =
 * constant The ratio can be negative or positive. If one joint is a revolute joint and the other
 * joint is a prismatic joint, then the ratio will have units of length or units of 1/length.
 *
 * @warning The revolute and prismatic joints must be attached to fixed bodies (which must be body1
 *          on those joints).
 * @warning You have to manually destroy the gear joint if joint1 or joint2 is destroyed.
 * @author Daniel Murphy
 */

class GearJoint extends Joint {

  // TODO(srdjan): Mark all fields m_xxx as private.
  final Joint m_joint1;
  final Joint m_joint2;

  final JointType m_typeA;
  final JointType m_typeB;

  // Body A is connected to body C
  // Body B is connected to body D
  final Body m_bodyC;
  final Body m_bodyD;

  // Solver shared
  final Vector2 m_localAnchorA = new Vector2.zero();
  final Vector2 m_localAnchorB = new Vector2.zero();
  final Vector2 m_localAnchorC = new Vector2.zero();
  final Vector2 m_localAnchorD = new Vector2.zero();

  final Vector2 m_localAxisC = new Vector2.zero();
  final Vector2 m_localAxisD = new Vector2.zero();

  double m_referenceAngleA = 0.0;
  double m_referenceAngleB = 0.0;

  double m_constant = 0.0;
  double m_ratio = 0.0;

  double m_impulse = 0.0;

  // Solver temp
  int m_indexA = 0,
      m_indexB = 0,
      m_indexC = 0,
      m_indexD = 0;
  final Vector2 m_lcA = new Vector2.zero(),
      m_lcB = new Vector2.zero(),
      m_lcC = new Vector2.zero(),
      m_lcD = new Vector2.zero();
  double m_mA = 0.0,
      m_mB = 0.0,
      m_mC = 0.0,
      m_mD = 0.0;
  double m_iA = 0.0,
      m_iB = 0.0,
      m_iC = 0.0,
      m_iD = 0.0;
  final Vector2 m_JvAC = new Vector2.zero(),
      m_JvBD = new Vector2.zero();
  double m_JwA = 0.0,
      m_JwB = 0.0,
      m_JwC = 0.0,
      m_JwD = 0.0;
  double m_mass = 0.0;

  GearJoint(IWorldPool argWorldPool, GearJointDef def)
      : m_joint1 = def.joint1,
        m_joint2 = def.joint2,
        m_typeA = def.joint1.getType(),
        m_typeB = def.joint2.getType(),
        m_bodyC = def.joint1.getBodyA(),
        m_bodyD = def.joint2.getBodyA(),
        super(argWorldPool, def) {
    assert(m_typeA == JointType.REVOLUTE || m_typeA == JointType.PRISMATIC);
    assert(m_typeB == JointType.REVOLUTE || m_typeB == JointType.PRISMATIC);

    double coordinateA, coordinateB;

    // TODO_ERIN there might be some problem with the joint edges in Joint.
    m_bodyA = m_joint1.getBodyB();

    // Get geometry of joint1
    Transform xfA = m_bodyA.transform;
    double aA = m_bodyA.sweep.a;
    Transform xfC = m_bodyC.transform;
    double aC = m_bodyC.sweep.a;

    if (m_typeA == JointType.REVOLUTE) {
      RevoluteJoint revolute = def.joint1;
      m_localAnchorC.setFrom(revolute.m_localAnchorA);
      m_localAnchorA.setFrom(revolute.m_localAnchorB);
      m_referenceAngleA = revolute.m_referenceAngle;
      m_localAxisC.setZero();

      coordinateA = aA - aC - m_referenceAngleA;
    } else {
      Vector2 pA = pool.popVec2();
      Vector2 temp = pool.popVec2();
      PrismaticJoint prismatic = def.joint1;
      m_localAnchorC.setFrom(prismatic.m_localAnchorA);
      m_localAnchorA.setFrom(prismatic.m_localAnchorB);
      m_referenceAngleA = prismatic.m_referenceAngle;
      m_localAxisC.setFrom(prismatic.m_localXAxisA);

      Vector2 pC = m_localAnchorC;
      Rot.mulToOutUnsafe(xfA.q, m_localAnchorA, temp);
      temp.add(xfA.p).sub(xfC.p);
      Rot.mulTransUnsafeVec2(xfC.q, temp, pA);
      coordinateA = pA.sub(pC).dot(m_localAxisC);
      pool.pushVec2(2);
    }

    m_bodyB = m_joint2.getBodyB();

    // Get geometry of joint2
    Transform xfB = m_bodyB.transform;
    double aB = m_bodyB.sweep.a;
    Transform xfD = m_bodyD.transform;
    double aD = m_bodyD.sweep.a;

    if (m_typeB == JointType.REVOLUTE) {
      RevoluteJoint revolute = def.joint2;
      m_localAnchorD.setFrom(revolute.m_localAnchorA);
      m_localAnchorB.setFrom(revolute.m_localAnchorB);
      m_referenceAngleB = revolute.m_referenceAngle;
      m_localAxisD.setZero();

      coordinateB = aB - aD - m_referenceAngleB;
    } else {
      Vector2 pB = pool.popVec2();
      Vector2 temp = pool.popVec2();
      PrismaticJoint prismatic = def.joint2;
      m_localAnchorD.setFrom(prismatic.m_localAnchorA);
      m_localAnchorB.setFrom(prismatic.m_localAnchorB);
      m_referenceAngleB = prismatic.m_referenceAngle;
      m_localAxisD.setFrom(prismatic.m_localXAxisA);

      Vector2 pD = m_localAnchorD;
      Rot.mulToOutUnsafe(xfB.q, m_localAnchorB, temp);
      temp.add(xfB.p).sub(xfD.p);
      Rot.mulTransUnsafeVec2(xfD.q, temp, pB);
      coordinateB = pB.sub(pD).dot(m_localAxisD);
      pool.pushVec2(2);
    }

    m_ratio = def.ratio;

    m_constant = coordinateA + m_ratio * coordinateB;

    m_impulse = 0.0;
  }

  void getAnchorA(Vector2 argOut) {
    m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
  }

  void getAnchorB(Vector2 argOut) {
    m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
  }

  void getReactionForce(double inv_dt, Vector2 argOut) {
    argOut.setFrom(m_JvAC).scale(m_impulse);
    argOut.scale(inv_dt);
  }

  double getReactionTorque(double inv_dt) {
    double L = m_impulse * m_JwA;
    return inv_dt * L;
  }

  void setRatio(double argRatio) {
    m_ratio = argRatio;
  }

  double getRatio() {
    return m_ratio;
  }

  void initVelocityConstraints(SolverData data) {
    m_indexA = m_bodyA.islandIndex;
    m_indexB = m_bodyB.islandIndex;
    m_indexC = m_bodyC.islandIndex;
    m_indexD = m_bodyD.islandIndex;
    m_lcA.setFrom(m_bodyA.sweep.localCenter);
    m_lcB.setFrom(m_bodyB.sweep.localCenter);
    m_lcC.setFrom(m_bodyC.sweep.localCenter);
    m_lcD.setFrom(m_bodyD.sweep.localCenter);
    m_mA = m_bodyA.invMass;
    m_mB = m_bodyB.invMass;
    m_mC = m_bodyC.invMass;
    m_mD = m_bodyD.invMass;
    m_iA = m_bodyA.invI;
    m_iB = m_bodyB.invI;
    m_iC = m_bodyC.invI;
    m_iD = m_bodyD.invI;

    // Vec2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vector2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;

    // Vec2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;
    Vector2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;

    // Vec2 cC = data.positions[m_indexC].c;
    double aC = data.positions[m_indexC].a;
    Vector2 vC = data.velocities[m_indexC].v;
    double wC = data.velocities[m_indexC].w;

    // Vec2 cD = data.positions[m_indexD].c;
    double aD = data.positions[m_indexD].a;
    Vector2 vD = data.velocities[m_indexD].v;
    double wD = data.velocities[m_indexD].w;

    Rot qA = pool.popRot(),
        qB = pool.popRot(),
        qC = pool.popRot(),
        qD = pool.popRot();
    qA.setAngle(aA);
    qB.setAngle(aB);
    qC.setAngle(aC);
    qD.setAngle(aD);

    m_mass = 0.0;

    Vector2 temp = pool.popVec2();

    if (m_typeA == JointType.REVOLUTE) {
      m_JvAC.setZero();
      m_JwA = 1.0;
      m_JwC = 1.0;
      m_mass += m_iA + m_iC;
    } else {
      Vector2 rC = pool.popVec2();
      Vector2 rA = pool.popVec2();
      Rot.mulToOutUnsafe(qC, m_localAxisC, m_JvAC);
      Rot.mulToOutUnsafe(qC, temp.setFrom(m_localAnchorC).sub(m_lcC), rC);
      Rot.mulToOutUnsafe(qA, temp.setFrom(m_localAnchorA).sub(m_lcA), rA);
      m_JwC = rC.cross(m_JvAC);
      m_JwA = rA.cross(m_JvAC);
      m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
      pool.pushVec2(2);
    }

    if (m_typeB == JointType.REVOLUTE) {
      m_JvBD.setZero();
      m_JwB = m_ratio;
      m_JwD = m_ratio;
      m_mass += m_ratio * m_ratio * (m_iB + m_iD);
    } else {
      Vector2 u = pool.popVec2();
      Vector2 rD = pool.popVec2();
      Vector2 rB = pool.popVec2();
      Rot.mulToOutUnsafe(qD, m_localAxisD, u);
      Rot.mulToOutUnsafe(qD, temp.setFrom(m_localAnchorD).sub(m_lcD), rD);
      Rot.mulToOutUnsafe(qB, temp.setFrom(m_localAnchorB).sub(m_lcB), rB);
      m_JvBD.setFrom(u).scale(m_ratio);
      m_JwD = m_ratio * rD.cross(u);
      m_JwB = m_ratio * rB.cross(u);
      m_mass += m_ratio * m_ratio * (m_mD + m_mB) +
          m_iD * m_JwD * m_JwD +
          m_iB * m_JwB * m_JwB;
      pool.pushVec2(3);
    }

    // Compute effective mass.
    m_mass = m_mass > 0.0 ? 1.0 / m_mass : 0.0;

    if (data.step.warmStarting) {
      vA.x += (m_mA * m_impulse) * m_JvAC.x;
      vA.y += (m_mA * m_impulse) * m_JvAC.y;
      wA += m_iA * m_impulse * m_JwA;

      vB.x += (m_mB * m_impulse) * m_JvBD.x;
      vB.y += (m_mB * m_impulse) * m_JvBD.y;
      wB += m_iB * m_impulse * m_JwB;

      vC.x -= (m_mC * m_impulse) * m_JvAC.x;
      vC.y -= (m_mC * m_impulse) * m_JvAC.y;
      wC -= m_iC * m_impulse * m_JwC;

      vD.x -= (m_mD * m_impulse) * m_JvBD.x;
      vD.y -= (m_mD * m_impulse) * m_JvBD.y;
      wD -= m_iD * m_impulse * m_JwD;
    } else {
      m_impulse = 0.0;
    }
    pool.pushVec2(1);
    pool.pushRot(4);

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
    // data.velocities[m_indexC].v = vC;
    data.velocities[m_indexC].w = wC;
    // data.velocities[m_indexD].v = vD;
    data.velocities[m_indexD].w = wD;
  }

  void solveVelocityConstraints(SolverData data) {
    Vector2 vA = data.velocities[m_indexA].v;
    double wA = data.velocities[m_indexA].w;
    Vector2 vB = data.velocities[m_indexB].v;
    double wB = data.velocities[m_indexB].w;
    Vector2 vC = data.velocities[m_indexC].v;
    double wC = data.velocities[m_indexC].w;
    Vector2 vD = data.velocities[m_indexD].v;
    double wD = data.velocities[m_indexD].w;

    Vector2 temp1 = pool.popVec2();
    Vector2 temp2 = pool.popVec2();
    double Cdot = m_JvAC.dot(temp1.setFrom(vA).sub(vC)) +
        m_JvBD.dot(temp2.setFrom(vB).sub(vD));
    Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);
    pool.pushVec2(2);

    double impulse = -m_mass * Cdot;
    m_impulse += impulse;

    vA.x += (m_mA * impulse) * m_JvAC.x;
    vA.y += (m_mA * impulse) * m_JvAC.y;
    wA += m_iA * impulse * m_JwA;

    vB.x += (m_mB * impulse) * m_JvBD.x;
    vB.y += (m_mB * impulse) * m_JvBD.y;
    wB += m_iB * impulse * m_JwB;

    vC.x -= (m_mC * impulse) * m_JvAC.x;
    vC.y -= (m_mC * impulse) * m_JvAC.y;
    wC -= m_iC * impulse * m_JwC;

    vD.x -= (m_mD * impulse) * m_JvBD.x;
    vD.y -= (m_mD * impulse) * m_JvBD.y;
    wD -= m_iD * impulse * m_JwD;

    // data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    // data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
    // data.velocities[m_indexC].v = vC;
    data.velocities[m_indexC].w = wC;
    // data.velocities[m_indexD].v = vD;
    data.velocities[m_indexD].w = wD;
  }

  Joint getJoint1() {
    return m_joint1;
  }

  Joint getJoint2() {
    return m_joint2;
  }

  bool solvePositionConstraints(SolverData data) {
    Vector2 cA = data.positions[m_indexA].c;
    double aA = data.positions[m_indexA].a;
    Vector2 cB = data.positions[m_indexB].c;
    double aB = data.positions[m_indexB].a;
    Vector2 cC = data.positions[m_indexC].c;
    double aC = data.positions[m_indexC].a;
    Vector2 cD = data.positions[m_indexD].c;
    double aD = data.positions[m_indexD].a;

    Rot qA = pool.popRot(),
        qB = pool.popRot(),
        qC = pool.popRot(),
        qD = pool.popRot();
    qA.setAngle(aA);
    qB.setAngle(aB);
    qC.setAngle(aC);
    qD.setAngle(aD);

    double linearError = 0.0;

    double coordinateA, coordinateB;

    Vector2 temp = pool.popVec2();
    Vector2 JvAC = pool.popVec2();
    Vector2 JvBD = pool.popVec2();
    double JwA, JwB, JwC, JwD;
    double mass = 0.0;

    if (m_typeA == JointType.REVOLUTE) {
      JvAC.setZero();
      JwA = 1.0;
      JwC = 1.0;
      mass += m_iA + m_iC;

      coordinateA = aA - aC - m_referenceAngleA;
    } else {
      Vector2 rC = pool.popVec2();
      Vector2 rA = pool.popVec2();
      Vector2 pC = pool.popVec2();
      Vector2 pA = pool.popVec2();
      Rot.mulToOutUnsafe(qC, m_localAxisC, JvAC);
      Rot.mulToOutUnsafe(qC, temp.setFrom(m_localAnchorC).sub(m_lcC), rC);
      Rot.mulToOutUnsafe(qA, temp.setFrom(m_localAnchorA).sub(m_lcA), rA);
      JwC = rC.cross(JvAC);
      JwA = rA.cross(JvAC);
      mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

      pC.setFrom(m_localAnchorC).sub(m_lcC);
      Rot.mulTransUnsafeVec2(qC, temp.setFrom(rA).add(cA).sub(cC), pA);
      coordinateA = pA.sub(pC).dot(m_localAxisC);
      pool.pushVec2(4);
    }

    if (m_typeB == JointType.REVOLUTE) {
      JvBD.setZero();
      JwB = m_ratio;
      JwD = m_ratio;
      mass += m_ratio * m_ratio * (m_iB + m_iD);

      coordinateB = aB - aD - m_referenceAngleB;
    } else {
      Vector2 u = pool.popVec2();
      Vector2 rD = pool.popVec2();
      Vector2 rB = pool.popVec2();
      Vector2 pD = pool.popVec2();
      Vector2 pB = pool.popVec2();
      Rot.mulToOutUnsafe(qD, m_localAxisD, u);
      Rot.mulToOutUnsafe(qD, temp.setFrom(m_localAnchorD).sub(m_lcD), rD);
      Rot.mulToOutUnsafe(qB, temp.setFrom(m_localAnchorB).sub(m_lcB), rB);
      JvBD.setFrom(u).scale(m_ratio);
      JwD = rD.cross(u);
      JwB = rB.cross(u);
      mass += m_ratio * m_ratio * (m_mD + m_mB) +
          m_iD * JwD * JwD +
          m_iB * JwB * JwB;

      pD.setFrom(m_localAnchorD).sub(m_lcD);
      Rot.mulTransUnsafeVec2(qD, temp.setFrom(rB).add(cB).sub(cD), pB);
      coordinateB = pB.sub(pD).dot(m_localAxisD);
      pool.pushVec2(5);
    }

    double C = (coordinateA + m_ratio * coordinateB) - m_constant;

    double impulse = 0.0;
    if (mass > 0.0) {
      impulse = -C / mass;
    }
    pool.pushVec2(3);
    pool.pushRot(4);

    cA.x += (m_mA * impulse) * JvAC.x;
    cA.y += (m_mA * impulse) * JvAC.y;
    aA += m_iA * impulse * JwA;

    cB.x += (m_mB * impulse) * JvBD.x;
    cB.y += (m_mB * impulse) * JvBD.y;
    aB += m_iB * impulse * JwB;

    cC.x -= (m_mC * impulse) * JvAC.x;
    cC.y -= (m_mC * impulse) * JvAC.y;
    aC -= m_iC * impulse * JwC;

    cD.x -= (m_mD * impulse) * JvBD.x;
    cD.y -= (m_mD * impulse) * JvBD.y;
    aD -= m_iD * impulse * JwD;

    // data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    // data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;
    // data.positions[m_indexC].c = cC;
    data.positions[m_indexC].a = aC;
    // data.positions[m_indexD].c = cD;
    data.positions[m_indexD].a = aD;

    // TODO_ERIN not implemented
    return linearError < Settings.linearSlop;
  }
}
