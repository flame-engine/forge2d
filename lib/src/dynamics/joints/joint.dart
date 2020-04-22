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

/// The base joint class. Joints are used to constrain two bodies together in various fashions. Some
/// joints also feature limits and motors.
abstract class Joint {
  static Joint create(World world, JointDef def) {
    switch (def.type) {
      case JointType.MOUSE:
        return new MouseJoint(world.getPool(), def as MouseJointDef);
      case JointType.DISTANCE:
        return new DistanceJoint(world.getPool(), def as DistanceJointDef);
      case JointType.PRISMATIC:
        return new PrismaticJoint(world.getPool(), def as PrismaticJointDef);
      case JointType.REVOLUTE:
        return new RevoluteJoint(world.getPool(), def as RevoluteJointDef);
      case JointType.WELD:
        return new WeldJoint(world.getPool(), def as WeldJointDef);
      case JointType.FRICTION:
        return new FrictionJoint(world.getPool(), def as FrictionJointDef);
      case JointType.WHEEL:
        return new WheelJoint(world.getPool(), def as WheelJointDef);
      case JointType.GEAR:
        return new GearJoint(world.getPool(), def as GearJointDef);
      case JointType.PULLEY:
        return new PulleyJoint(world.getPool(), def as PulleyJointDef);
      case JointType.CONSTANT_VOLUME:
        return new ConstantVolumeJoint(world, def as ConstantVolumeJointDef);
      case JointType.ROPE:
        return new RopeJoint(world.getPool(), def as RopeJointDef);
      case JointType.MOTOR:
        return new MotorJoint(world.getPool(), def as MotorJointDef);
      case JointType.UNKNOWN:
      default:
        return null;
    }
  }

  static void destroy(Joint joint) {
    joint.destructor();
  }

  final JointType _type;
  Joint _prev;
  Joint _next;
  JointEdge _edgeA;
  JointEdge _edgeB;
  Body _bodyA;
  Body _bodyB;

  bool _islandFlag = false;
  bool _collideConnected = false;

  IWorldPool pool;

  Joint(IWorldPool worldPool, JointDef def) : _type = def.type {
    assert(def.bodyA != def.bodyB);

    pool = worldPool;
    _prev = null;
    _next = null;
    _bodyA = def.bodyA;
    _bodyB = def.bodyB;
    _collideConnected = def.collideConnected;
    _islandFlag = false;

    _edgeA = new JointEdge();
    _edgeA.joint = null;
    _edgeA.other = null;
    _edgeA.prev = null;
    _edgeA.next = null;

    _edgeB = new JointEdge();
    _edgeB.joint = null;
    _edgeB.other = null;
    _edgeB.prev = null;
    _edgeB.next = null;
  }

  /// get the type of the concrete joint.
  ///
  /// @return
  JointType getType() {
    return _type;
  }

  /// get the first body attached to this joint.
  Body getBodyA() {
    return _bodyA;
  }

  /// get the second body attached to this joint.
  ///
  /// @return
  Body getBodyB() {
    return _bodyB;
  }

  /// get the anchor point on bodyA in world coordinates.
  ///
  /// @return
  void getAnchorA(Vector2 out);

  /// get the anchor point on bodyB in world coordinates.
  ///
  /// @return
  void getAnchorB(Vector2 out);

  /// get the reaction force on body2 at the joint anchor in Newtons.
  ///
  /// @param inv_dt
  /// @return
  void getReactionForce(double inv_dt, Vector2 out);

  /// get the reaction torque on body2 in N*m.
  ///
  /// @param inv_dt
  /// @return
  double getReactionTorque(double inv_dt);

  /// get the next joint the world joint list.
  Joint getNext() {
    return _next;
  }

  /// Get collide connected. Note: modifying the collide connect flag won't work correctly because
  /// the flag is only checked when fixture AABBs begin to overlap.
  bool getCollideConnected() {
    return _collideConnected;
  }

  /// Short-cut function to determine if either body is inactive.
  ///
  /// @return
  bool isActive() {
    return _bodyA.isActive() && _bodyB.isActive();
  }

  /// Internal
  void initVelocityConstraints(SolverData data);

  /// Internal
  void solveVelocityConstraints(SolverData data);

  /// This returns true if the position errors are within tolerance. Internal.
  bool solvePositionConstraints(SolverData data);

  /// Override to handle destruction of joint
  void destructor() {}
}
