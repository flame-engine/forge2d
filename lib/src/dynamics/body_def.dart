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

/// A body definition holds all the data needed to construct a rigid body. You can safely re-use body
/// definitions. Shapes are added to a body after construction.
class BodyDef {
  /// The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
  /// mass is set to one.
  BodyType type = BodyType.STATIC;

  /// Use this to store application specific body data.
  Object userData;

  /// The world position of the body. Avoid creating bodies at the origin since this can lead to many
  /// overlapping shapes.
  Vector2 position = new Vector2.zero();

  /// The world angle of the body in radians.
  double angle = 0.0;

  /// The linear velocity of the body in world co-ordinates.
  Vector2 linearVelocity = new Vector2.zero();

  /// The angular velocity of the body.
  double angularVelocity = 0.0;

  /// Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  double linearDamping = 0.0;

  /// Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  double angularDamping = 0.0;

  /// Set this flag to false if this body should never fall asleep. Note that this increases CPU
  /// usage.
  bool allowSleep = true;

  /// Is this body initially sleeping?
  bool awake = true;

  /// Should this body be prevented from rotating? Useful for characters.
  bool fixedRotation = false;

  /// Is this a fast moving body that should be prevented from tunneling through other moving bodies?
  /// Note that all bodies are prevented from tunneling through kinematic and static bodies. This
  /// setting is only considered on dynamic bodies.
  ///
  /// @warning You should use this flag sparingly since it increases processing time.
  bool bullet = false;

  /// Does this body start out active?
  bool active = true;

  /// Experimental: scales the inertia tensor.
  double gravityScale = 1.0;

  /// The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
  /// mass is set to one.
  BodyType getType() {
    return type;
  }

  /// The body type: static, kinematic, or dynamic. Note: if a dynamic body would have zero mass, the
  /// mass is set to one.
  void setType(BodyType type) {
    this.type = type;
  }

  /// Use this to store application specific body data.
  Object getUserData() {
    return userData;
  }

  /// Use this to store application specific body data.
  void setUserData(Object userData) {
    this.userData = userData;
  }

  /// The world position of the body. Avoid creating bodies at the origin since this can lead to many
  /// overlapping shapes.
  Vector2 getPosition() {
    return position;
  }

  /// The world position of the body. Avoid creating bodies at the origin since this can lead to many
  /// overlapping shapes.
  void setPosition(Vector2 position) {
    this.position = position;
  }

  /// The world angle of the body in radians.
  double getAngle() {
    return angle;
  }

  /// The world angle of the body in radians.
  void setAngle(double angle) {
    this.angle = angle;
  }

  /// The linear velocity of the body in world co-ordinates.
  Vector2 getLinearVelocity() {
    return linearVelocity;
  }

  /// The linear velocity of the body in world co-ordinates.
  void setLinearVelocity(Vector2 linearVelocity) {
    this.linearVelocity = linearVelocity;
  }

  /// The angular velocity of the body.
  double getAngularVelocity() {
    return angularVelocity;
  }

  /// The angular velocity of the body.
  void setAngularVelocity(double angularVelocity) {
    this.angularVelocity = angularVelocity;
  }

  /// Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  double getLinearDamping() {
    return linearDamping;
  }

  /// Linear damping is use to reduce the linear velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  void setLinearDamping(double linearDamping) {
    this.linearDamping = linearDamping;
  }

  /// Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  double getAngularDamping() {
    return angularDamping;
  }

  /// Angular damping is use to reduce the angular velocity. The damping parameter can be larger than
  /// 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  void setAngularDamping(double angularDamping) {
    this.angularDamping = angularDamping;
  }

  /// Set this flag to false if this body should never fall asleep. Note that this increases CPU
  /// usage.
  bool isAllowSleep() {
    return allowSleep;
  }

  /// Set this flag to false if this body should never fall asleep. Note that this increases CPU
  /// usage.
  void setAllowSleep(bool allowSleep) {
    this.allowSleep = allowSleep;
  }

  /// Is this body initially sleeping?
  bool isAwake() {
    return awake;
  }

  /// Is this body initially sleeping?
  void setAwake(bool awake) {
    this.awake = awake;
  }

  /// Should this body be prevented from rotating? Useful for characters.
  bool isFixedRotation() {
    return fixedRotation;
  }

  /// Should this body be prevented from rotating? Useful for characters.
  void setFixedRotation(bool fixedRotation) {
    this.fixedRotation = fixedRotation;
  }

  /// Is this a fast moving body that should be prevented from tunneling through other moving bodies?
  /// Note that all bodies are prevented from tunneling through kinematic and static bodies. This
  /// setting is only considered on dynamic bodies.
  ///
  /// @warning You should use this flag sparingly since it increases processing time.
  bool isBullet() {
    return bullet;
  }

  /// Is this a fast moving body that should be prevented from tunneling through other moving bodies?
  /// Note that all bodies are prevented from tunneling through kinematic and static bodies. This
  /// setting is only considered on dynamic bodies.
  ///
  /// @warning You should use this flag sparingly since it increases processing time.
  void setBullet(bool bullet) {
    this.bullet = bullet;
  }

  /// Does this body start out active?
  bool isActive() {
    return active;
  }

  /// Does this body start out active?
  void setActive(bool active) {
    this.active = active;
  }

  /// Experimental: scales the inertia tensor.
  double getGravityScale() {
    return gravityScale;
  }

  /// Experimental: scales the inertia tensor.
  void setGravityScale(double gravityScale) {
    this.gravityScale = gravityScale;
  }
}
