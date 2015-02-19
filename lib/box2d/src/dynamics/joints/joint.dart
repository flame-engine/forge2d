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
 * The base joint class. Joints are used to constrain two bodies together in various fashions. Some
 * joints also feature limits and motors.
 * 
 * @author Daniel Murphy
 */
abstract class Joint {

  static Joint create(World world, JointDef def) {
    // Joint joint = null;
    switch (def.type) {
      case JointType.MOUSE:
        return new MouseJoint(world.getPool(), def);
      case JointType.DISTANCE:
        return new DistanceJoint(world.getPool(), def);
      case JointType.PRISMATIC:
        return new PrismaticJoint(world.getPool(), def);
      case JointType.REVOLUTE:
        return new RevoluteJoint(world.getPool(), def);
      case JointType.WELD:
        return new WeldJoint(world.getPool(), def);
      case JointType.FRICTION:
        return new FrictionJoint(world.getPool(), def);
      case JointType.WHEEL:
        return new WheelJoint(world.getPool(), def);
      case JointType.GEAR:
        return new GearJoint(world.getPool(), def);
      case JointType.PULLEY:
        return new PulleyJoint(world.getPool(), def);
      case JointType.CONSTANT_VOLUME:
        return new ConstantVolumeJoint(world, def);
      case JointType.ROPE:
        return new RopeJoint(world.getPool(), def);
      case JointType.MOTOR:
        return new MotorJoint(world.getPool(), def);
      case JointType.UNKNOWN:
      default:
        return null;
    }
  }

  static void destroy(Joint joint) {
    joint.destructor();
  }

  final JointType _m_type;
  Joint m_prev;
  Joint m_next;
  JointEdge m_edgeA;
  JointEdge m_edgeB;
  Body m_bodyA;
  Body m_bodyB;

  bool m_islandFlag = false;
  bool _m_collideConnected = false;

  Object m_userData;

  IWorldPool pool;

  // Cache here per time step to reduce cache misses.
  // final Vec2 m_localCenterA, m_localCenterB;
  // double m_invMassA, m_invIA;
  // double m_invMassB, m_invIB;

  Joint(IWorldPool worldPool, JointDef def) : _m_type = def.type {
    assert(def.bodyA != def.bodyB);

    pool = worldPool;
    m_prev = null;
    m_next = null;
    m_bodyA = def.bodyA;
    m_bodyB = def.bodyB;
    _m_collideConnected = def.collideConnected;
    m_islandFlag = false;
    m_userData = def.userData;

    m_edgeA = new JointEdge();
    m_edgeA.joint = null;
    m_edgeA.other = null;
    m_edgeA.prev = null;
    m_edgeA.next = null;

    m_edgeB = new JointEdge();
    m_edgeB.joint = null;
    m_edgeB.other = null;
    m_edgeB.prev = null;
    m_edgeB.next = null;

    // m_localCenterA = new Vec2();
    // m_localCenterB = new Vec2();
  }

  /**
   * get the type of the concrete joint.
   * 
   * @return
   */
  JointType getType() {
    return _m_type;
  }

  /**
   * get the first body attached to this joint.
   */
  Body getBodyA() {
    return m_bodyA;
  }

  /**
   * get the second body attached to this joint.
   * 
   * @return
   */
  Body getBodyB() {
    return m_bodyB;
  }

  /**
   * get the anchor point on bodyA in world coordinates.
   * 
   * @return
   */
  void getAnchorA(Vec2 out);

  /**
   * get the anchor point on bodyB in world coordinates.
   * 
   * @return
   */
  void getAnchorB(Vec2 out);

  /**
   * get the reaction force on body2 at the joint anchor in Newtons.
   * 
   * @param inv_dt
   * @return
   */
  void getReactionForce(double inv_dt, Vec2 out);

  /**
   * get the reaction torque on body2 in N*m.
   * 
   * @param inv_dt
   * @return
   */
  double getReactionTorque(double inv_dt);

  /**
   * get the next joint the world joint list.
   */
  Joint getNext() {
    return m_next;
  }

  /**
   * get the user data pointer.
   */
  Object getUserData() {
    return m_userData;
  }

  /**
   * Set the user data pointer.
   */
  void setUserData(Object data) {
    m_userData = data;
  }

  /**
   * Get collide connected. Note: modifying the collide connect flag won't work correctly because
   * the flag is only checked when fixture AABBs begin to overlap.
   */
  bool getCollideConnected() {
    return _m_collideConnected;
  }

  /**
   * Short-cut function to determine if either body is inactive.
   * 
   * @return
   */
  bool isActive() {
    return m_bodyA.isActive() && m_bodyB.isActive();
  }

  /** Internal */
  void initVelocityConstraints(SolverData data);

  /** Internal */
  void solveVelocityConstraints(SolverData data);

  /**
   * This returns true if the position errors are within tolerance. Internal.
   */
  bool solvePositionConstraints(SolverData data);

  /**
   * Override to handle destruction of joint
   */
  void destructor() {}
}
