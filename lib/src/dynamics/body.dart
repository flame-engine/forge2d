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
 * A rigid body. These are created via World.createBody.
 *
 * @author Daniel Murphy
 */
class Body {
  static const int ISLAND_FLAG = 0x0001;
  static const int AWAKE_FLAG = 0x0002;
  static const int AUTO_SLEEP_FLAG = 0x0004;
  static const int BULLET_FLAG = 0x0008;
  static const int FIXED_ROTATION_FLAG = 0x0010;
  static const int ACTIVE_FLAG = 0x0020;
  static const int TOI_FLAG = 0x0040;

  BodyType _bodyType = BodyType.STATIC;

  int flags = 0;

  int islandIndex = 0;

  /**
   * The body origin transform.
   */
  final Transform transform = new Transform.zero();
  /**
   * The previous transform for particle simulation
   */
  final Transform xf0 = new Transform.zero();

  /**
   * The swept motion for CCD
   */
  final Sweep sweep = new Sweep();

  /// the linear velocity of the center of mass
  final Vector2 _linearVelocity = new Vector2.zero();
  double _angularVelocity = 0.0;

  final Vector2 force = new Vector2.zero();
  double torque = 0.0;

  final World world;
  Body prev;
  Body next;

  Fixture fixtureList;
  int fixtureCount = 0;

  JointEdge jointList;
  ContactEdge contactList;

  double _mass = 0.0,
      invMass = 0.0;

  // Rotational inertia about the center of mass.
  double I = 0.0,
      invI = 0.0;

  double linearDamping = 0.0;
  double angularDamping = 0.0;
  double gravityScale = 0.0;

  double sleepTime = 0.0;

  /// Use this to store your application specific data.
  Object userData;

  Body(final BodyDef bd, this.world) {
    assert(MathUtils.vector2IsValid(bd.position));
    assert(MathUtils.vector2IsValid(bd.linearVelocity));
    assert(bd.gravityScale >= 0.0);
    assert(bd.angularDamping >= 0.0);
    assert(bd.linearDamping >= 0.0);

    flags = 0;

    if (bd.bullet) {
      flags |= BULLET_FLAG;
    }
    if (bd.fixedRotation) {
      flags |= FIXED_ROTATION_FLAG;
    }
    if (bd.allowSleep) {
      flags |= AUTO_SLEEP_FLAG;
    }
    if (bd.awake) {
      flags |= AWAKE_FLAG;
    }
    if (bd.active) {
      flags |= ACTIVE_FLAG;
    }

    transform.p.setFrom(bd.position);
    transform.q.setAngle(bd.angle);

    sweep.localCenter.setZero();
    sweep.c0.setFrom(transform.p);
    sweep.c.setFrom(transform.p);
    sweep.a0 = bd.angle;
    sweep.a = bd.angle;
    sweep.alpha0 = 0.0;

    jointList = null;
    contactList = null;
    prev = null;
    next = null;

    _linearVelocity.setFrom(bd.linearVelocity);
    _angularVelocity = bd.angularVelocity;

    linearDamping = bd.linearDamping;
    angularDamping = bd.angularDamping;
    gravityScale = bd.gravityScale;

    force.setZero();

    sleepTime = 0.0;

    _bodyType = bd.type;

    if (_bodyType == BodyType.DYNAMIC) {
      _mass = 1.0;
      invMass = 1.0;
    } else {
      _mass = 0.0;
      invMass = 0.0;
    }

    I = 0.0;
    invI = 0.0;

    userData = bd.userData;

    fixtureList = null;
    fixtureCount = 0;
  }

  /**
   * Creates a fixture and attach it to this body. Use this function if you need to set some fixture
   * parameters, like friction. Otherwise you can create the fixture directly from a shape. If the
   * density is non-zero, this function automatically updates the mass of the body. Contacts are not
   * created until the next time step.
   *
   * @param def the fixture definition.
   * @warning This function is locked during callbacks.
   */
  Fixture createFixtureFromFixtureDef(FixtureDef def) {
    assert(world.isLocked() == false);

    if (world.isLocked() == true) {
      return null;
    }

    Fixture fixture = new Fixture();
    fixture.create(this, def);

    if ((flags & ACTIVE_FLAG) == ACTIVE_FLAG) {
      BroadPhase broadPhase = world.m_contactManager.broadPhase;
      fixture.createProxies(broadPhase, transform);
    }

    fixture.m_next = fixtureList;
    fixtureList = fixture;
    ++fixtureCount;

    fixture.m_body = this;

    // Adjust mass properties if needed.
    if (fixture.m_density > 0.0) {
      resetMassData();
    }

    // Let the world know we have a new fixture. This will cause new contacts
    // to be created at the beginning of the next time step.
    world.m_flags |= World.NEW_FIXTURE;

    return fixture;
  }

  final FixtureDef _fixDef = new FixtureDef();

  /**
   * Creates a fixture from a shape and attach it to this body. This is a convenience function. Use
   * FixtureDef if you need to set parameters like friction, restitution, user data, or filtering.
   * If the density is non-zero, this function automatically updates the mass of the body.
   *
   * @param shape the shape to be cloned.
   * @param density the shape density (set to zero for static bodies).
   * @warning This function is locked during callbacks.
   */
  Fixture createFixtureFromShape(Shape shape, [double density = 0.0]) {
    _fixDef.shape = shape;
    _fixDef.density = density;

    return createFixtureFromFixtureDef(_fixDef);
  }

  /**
   * Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts
   * associated with this fixture. This will automatically adjust the mass of the body if the body
   * is dynamic and the fixture has positive density. All fixtures attached to a body are implicitly
   * destroyed when the body is destroyed.
   *
   * @param fixture the fixture to be removed.
   * @warning This function is locked during callbacks.
   */
  void destroyFixture(Fixture fixture) {
    assert(world.isLocked() == false);
    if (world.isLocked() == true) {
      return;
    }

    assert(fixture.m_body == this);

    // Remove the fixture from this body's singly linked list.
    assert(fixtureCount > 0);
    Fixture node = fixtureList;
    Fixture last = null; // java change
    bool found = false;
    while (node != null) {
      if (node == fixture) {
        node = fixture.m_next;
        found = true;
        break;
      }
      last = node;
      node = node.m_next;
    }

    // You tried to remove a shape that is not attached to this body.
    assert(found);

    // java change, remove it from the list
    if (last == null) {
      fixtureList = fixture.m_next;
    } else {
      last.m_next = fixture.m_next;
    }

    // Destroy any contacts associated with the fixture.
    ContactEdge edge = contactList;
    while (edge != null) {
      Contact c = edge.contact;
      edge = edge.next;

      Fixture fixtureA = c.fixtureA;
      Fixture fixtureB = c.fixtureB;

      if (fixture == fixtureA || fixture == fixtureB) {
        // This destroys the contact and removes it from
        // this body's contact list.
        world.m_contactManager.destroy(c);
      }
    }

    if ((flags & ACTIVE_FLAG) == ACTIVE_FLAG) {
      BroadPhase broadPhase = world.m_contactManager.broadPhase;
      fixture.destroyProxies(broadPhase);
    }

    fixture.destroy();
    fixture.m_body = null;
    fixture.m_next = null;
    fixture = null;

    --fixtureCount;

    // Reset the mass data.
    resetMassData();
  }

  /**
   * Set the position of the body's origin and rotation. This breaks any contacts and wakes the
   * other bodies. Manipulating a body's transform may cause non-physical behavior. Note: contacts
   * are updated on the next call to World.step().
   *
   * @param position the world position of the body's local origin.
   * @param angle the world rotation in radians.
   */
  void setTransform(Vector2 position, double angle) {
    assert(world.isLocked() == false);
    if (world.isLocked() == true) {
      return;
    }

    transform.q.setAngle(angle);
    transform.p.setFrom(position);

    // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
    Transform.mulToOutUnsafeVec2(transform, sweep.localCenter, sweep.c);
    sweep.a = angle;

    sweep.c0.setFrom(sweep.c);
    sweep.a0 = sweep.a;

    BroadPhase broadPhase = world.m_contactManager.broadPhase;
    for (Fixture f = fixtureList; f != null; f = f.m_next) {
      f.synchronize(broadPhase, transform, transform);
    }
  }

  /**
   * Get the world body origin position. Do not modify.
   *
   * @return the world position of the body's origin.
   */
  Vector2 get position => transform.p;

  /**
   * Get the angle in radians.
   *
   * @return the current world rotation angle in radians.
   */
  double getAngle() {
    return sweep.a;
  }

  /**
   * Get the world position of the center of mass. Do not modify.
   */
  Vector2 get worldCenter => sweep.c;

  /**
   * Get the local position of the center of mass. Do not modify.
   */
  Vector2 getLocalCenter() {
    return sweep.localCenter;
  }

  /**
   * Set the linear velocity of the center of mass.
   *
   * @param v the new linear velocity of the center of mass.
   */
  void set linearVelocity(Vector2 v) {
    if (_bodyType == BodyType.STATIC) {
      return;
    }

    if (v.dot(v) > 0.0) {
      setAwake(true);
    }

    _linearVelocity.setFrom(v);
  }

  /**
   * Get the linear velocity of the center of mass. Do not modify, instead use
   * {@link #setLinearVelocity(Vec2)}.
   *
   * @return the linear velocity of the center of mass.
   */
  Vector2 get linearVelocity => _linearVelocity;

  /**
   * Set the angular velocity.
   *
   * @param omega the new angular velocity in radians/second.
   */
  void set angularVelocity(double w) {
    if (_bodyType == BodyType.STATIC) {
      return;
    }

    if (w * w > 0.0) {
      setAwake(true);
    }

    _angularVelocity = w;
  }

  /**
   * Get the angular velocity.
   *
   * @return the angular velocity in radians/second.
   */
  double get angularVelocity {
    return _angularVelocity;
  }

  /**
   * Apply a force at a world point. If the force is not applied at the center of mass, it will
   * generate a torque and affect the angular velocity. This wakes up the body.
   *
   * @param force the world force vector, usually in Newtons (N).
   * @param point the world position of the point of application.
   */
  void applyForce(Vector2 force, Vector2 point) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }

    // m_force.addLocal(force);
    // Vec2 temp = tltemp.get();
    // temp.set(point).subLocal(m_sweep.c);
    // m_torque += Vec2.cross(temp, force);

    force.x += force.x;
    force.y += force.y;

    torque +=
        (point.x - sweep.c.x) * force.y - (point.y - sweep.c.y) * force.x;
  }

  /**
   * Apply a force to the center of mass. This wakes up the body.
   *
   * @param force the world force vector, usually in Newtons (N).
   */
  void applyForceToCenter(Vector2 force) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }

    force.x += force.x;
    force.y += force.y;
  }

  /**
   * Apply a torque. This affects the angular velocity without affecting the linear velocity of the
   * center of mass. This wakes up the body.
   *
   * @param torque about the z-axis (out of the screen), usually in N-m.
   */
  void applyTorque(double torque) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }

    torque += torque;
  }

  /**
   * Apply an impulse at a point. This immediately modifies the velocity. It also modifies the
   * angular velocity if the point of application is not at the center of mass. This wakes up the
   * body if 'wake' is set to true. If the body is sleeping and 'wake' is false, then there is no
   * effect.
   *
   * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
   * @param point the world position of the point of application.
   * @param wake also wake up the body
   */
  void applyLinearImpulse(Vector2 impulse, Vector2 point, bool wake) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (!isAwake()) {
      if (wake) {
        setAwake(true);
      } else {
        return;
      }
    }

    _linearVelocity.x += impulse.x * invMass;
    _linearVelocity.y += impulse.y * invMass;

    _angularVelocity += invI *
        ((point.x - sweep.c.x) * impulse.y -
            (point.y - sweep.c.y) * impulse.x);
  }

  /**
   * Apply an angular impulse.
   *
   * @param impulse the angular impulse in units of kg*m*m/s
   */
  void applyAngularImpulse(double impulse) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }
    _angularVelocity += invI * impulse;
  }

  /**
   * Get the total mass of the body.
   *
   * @return the mass, usually in kilograms (kg).
   */
  double get mass => _mass;

  /**
   * Get the central rotational inertia of the body.
   *
   * @return the rotational inertia, usually in kg-m^2.
   */
  double getInertia() {
    return I +
        _mass *
            (sweep.localCenter.x * sweep.localCenter.x +
                sweep.localCenter.y * sweep.localCenter.y);
  }

  /**
   * Get the mass data of the body. The rotational inertia is relative to the center of mass.
   *
   * @return a struct containing the mass, inertia and center of the body.
   */
  void getMassData(MassData data) {
    // data.mass = m_mass;
    // data.I = m_I + m_mass * Vec2.dot(m_sweep.localCenter, m_sweep.localCenter);
    // data.center.set(m_sweep.localCenter);

    data.mass = _mass;
    data.I = I +
        _mass *
            (sweep.localCenter.x * sweep.localCenter.x +
                sweep.localCenter.y * sweep.localCenter.y);
    data.center.x = sweep.localCenter.x;
    data.center.y = sweep.localCenter.y;
  }

  /**
   * Set the mass properties to override the mass properties of the fixtures. Note that this changes
   * the center of mass position. Note that creating or destroying fixtures can also alter the mass.
   * This function has no effect if the body isn't dynamic.
   *
   * @param massData the mass properties.
   */
  void setMassData(MassData massData) {
    // TODO_ERIN adjust linear velocity and torque to account for movement of center.
    assert(world.isLocked() == false);
    if (world.isLocked() == true) {
      return;
    }

    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    invMass = 0.0;
    I = 0.0;
    invI = 0.0;

    _mass = massData.mass;
    if (_mass <= 0.0) {
      _mass = 1.0;
    }

    invMass = 1.0 / _mass;

    if (massData.I > 0.0 && (flags & FIXED_ROTATION_FLAG) == 0.0) {
      I = massData.I - _mass * massData.center.dot(massData.center);
      assert(I > 0.0);
      invI = 1.0 / I;
    }

    final Vector2 oldCenter = world.getPool().popVec2();
    // Move center of mass.
    oldCenter.setFrom(sweep.c);
    sweep.localCenter.setFrom(massData.center);
    // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
    Transform.mulToOutUnsafeVec2(transform, sweep.localCenter, sweep.c0);
    sweep.c.setFrom(sweep.c0);

    // Update center of mass velocity.
    // m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
    final Vector2 temp = world.getPool().popVec2();
    temp.setFrom(sweep.c).sub(oldCenter);
    temp.scaleOrthogonalInto(_angularVelocity, temp);
    _linearVelocity.add(temp);

    world.getPool().pushVec2(2);
  }

  final MassData _pmd = new MassData();

  /**
   * This resets the mass properties to the sum of the mass properties of the fixtures. This
   * normally does not need to be called unless you called setMassData to override the mass and you
   * later want to reset the mass.
   */
  void resetMassData() {
    // Compute mass data from shapes. Each shape has its own density.
    _mass = 0.0;
    invMass = 0.0;
    I = 0.0;
    invI = 0.0;
    sweep.localCenter.setZero();

    // Static and kinematic bodies have zero mass.
    if (_bodyType == BodyType.STATIC || _bodyType == BodyType.KINEMATIC) {
      // m_sweep.c0 = m_sweep.c = m_xf.position;
      sweep.c0.setFrom(transform.p);
      sweep.c.setFrom(transform.p);
      sweep.a0 = sweep.a;
      return;
    }

    assert(_bodyType == BodyType.DYNAMIC);

    // Accumulate mass over all fixtures.
    final Vector2 localCenter = world.getPool().popVec2();
    localCenter.setZero();
    final Vector2 temp = world.getPool().popVec2();
    final MassData massData = _pmd;
    for (Fixture f = fixtureList; f != null; f = f.m_next) {
      if (f.m_density == 0.0) {
        continue;
      }
      f.getMassData(massData);
      _mass += massData.mass;
      // center += massData.mass * massData.center;
      temp.setFrom(massData.center).scale(massData.mass);
      localCenter.add(temp);
      I += massData.I;
    }

    // Compute center of mass.
    if (_mass > 0.0) {
      invMass = 1.0 / _mass;
      localCenter.scale(invMass);
    } else {
      // Force all dynamic bodies to have a positive mass.
      _mass = 1.0;
      invMass = 1.0;
    }

    if (I > 0.0 && (flags & FIXED_ROTATION_FLAG) == 0.0) {
      // Center the inertia about the center of mass.
      I -= _mass * localCenter.dot(localCenter);
      assert(I > 0.0);
      invI = 1.0 / I;
    } else {
      I = 0.0;
      invI = 0.0;
    }

    Vector2 oldCenter = world.getPool().popVec2();
    // Move center of mass.
    oldCenter.setFrom(sweep.c);
    sweep.localCenter.setFrom(localCenter);
    // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
    Transform.mulToOutUnsafeVec2(transform, sweep.localCenter, sweep.c0);
    sweep.c.setFrom(sweep.c0);

    // Update center of mass velocity.
    // m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
    temp.setFrom(sweep.c).sub(oldCenter);

    final Vector2 temp2 = oldCenter;
    temp.scaleOrthogonalInto(_angularVelocity, temp2);
    _linearVelocity.add(temp2);

    world.getPool().pushVec2(3);
  }

  /**
   * Get the world coordinates of a point given the local coordinates.
   *
   * @param localPoint a point on the body measured relative the the body's origin.
   * @return the same point expressed in world coordinates.
   */
  Vector2 getWorldPoint(Vector2 localPoint) {
    Vector2 v = new Vector2.zero();
    getWorldPointToOut(localPoint, v);
    return v;
  }

  void getWorldPointToOut(Vector2 localPoint, Vector2 out) {
    Transform.mulToOutVec2(transform, localPoint, out);
  }

  /**
   * Get the world coordinates of a vector given the local coordinates.
   *
   * @param localVector a vector fixed in the body.
   * @return the same vector expressed in world coordinates.
   */
  Vector2 getWorldVector(Vector2 localVector) {
    Vector2 out = new Vector2.zero();
    getWorldVectorToOut(localVector, out);
    return out;
  }

  void getWorldVectorToOut(Vector2 localVector, Vector2 out) {
    Rot.mulToOut(transform.q, localVector, out);
  }

  void getWorldVectorToOutUnsafe(Vector2 localVector, Vector2 out) {
    Rot.mulToOutUnsafe(transform.q, localVector, out);
  }

  /**
   * Gets a local point relative to the body's origin given a world point.
   *
   * @param a point in world coordinates.
   * @return the corresponding local point relative to the body's origin.
   */
  Vector2 getLocalPoint(Vector2 worldPoint) {
    Vector2 out = new Vector2.zero();
    getLocalPointToOut(worldPoint, out);
    return out;
  }

  void getLocalPointToOut(Vector2 worldPoint, Vector2 out) {
    Transform.mulTransToOutVec2(transform, worldPoint, out);
  }

  /**
   * Gets a local vector given a world vector.
   *
   * @param a vector in world coordinates.
   * @return the corresponding local vector.
   */
  Vector2 getLocalVector(Vector2 worldVector) {
    Vector2 out = new Vector2.zero();
    getLocalVectorToOut(worldVector, out);
    return out;
  }

  void getLocalVectorToOut(Vector2 worldVector, Vector2 out) {
    Rot.mulTransVec2(transform.q, worldVector, out);
  }

  void getLocalVectorToOutUnsafe(Vector2 worldVector, Vector2 out) {
    Rot.mulTransUnsafeVec2(transform.q, worldVector, out);
  }

  /**
   * Get the world linear velocity of a world point attached to this body.
   *
   * @param a point in world coordinates.
   * @return the world velocity of a point.
   */
  Vector2 getLinearVelocityFromWorldPoint(Vector2 worldPoint) {
    Vector2 out = new Vector2.zero();
    getLinearVelocityFromWorldPointToOut(worldPoint, out);
    return out;
  }

  void getLinearVelocityFromWorldPointToOut(Vector2 worldPoint, Vector2 out) {
    final double tempX = worldPoint.x - sweep.c.x;
    final double tempY = worldPoint.y - sweep.c.y;
    out.x = -_angularVelocity * tempY + _linearVelocity.x;
    out.y = _angularVelocity * tempX + _linearVelocity.y;
  }

  /**
   * Get the world velocity of a local point.
   *
   * @param a point in local coordinates.
   * @return the world velocity of a point.
   */
  Vector2 getLinearVelocityFromLocalPoint(Vector2 localPoint) {
    Vector2 out = new Vector2.zero();
    getLinearVelocityFromLocalPointToOut(localPoint, out);
    return out;
  }

  void getLinearVelocityFromLocalPointToOut(Vector2 localPoint, Vector2 out) {
    getWorldPointToOut(localPoint, out);
    getLinearVelocityFromWorldPointToOut(out, out);
  }

  BodyType getType() {
    return _bodyType;
  }

  /**
   * Set the type of this body. This may alter the mass and velocity.
   *
   * @param type
   */
  void setType(BodyType type) {
    assert(world.isLocked() == false);
    if (world.isLocked() == true) {
      return;
    }

    if (_bodyType == type) {
      return;
    }

    _bodyType = type;

    resetMassData();

    if (_bodyType == BodyType.STATIC) {
      _linearVelocity.setZero();
      _angularVelocity = 0.0;
      sweep.a0 = sweep.a;
      sweep.c0.setFrom(sweep.c);
      synchronizeFixtures();
    }

    setAwake(true);

    force.setZero();
    torque = 0.0;

    // Delete the attached contacts.
    ContactEdge ce = contactList;
    while (ce != null) {
      ContactEdge ce0 = ce;
      ce = ce.next;
      world.m_contactManager.destroy(ce0.contact);
    }
    contactList = null;

    // Touch the proxies so that new contacts will be created (when appropriate)
    BroadPhase broadPhase = world.m_contactManager.broadPhase;
    for (Fixture f = fixtureList; f != null; f = f.m_next) {
      int proxyCount = f.m_proxyCount;
      for (int i = 0; i < proxyCount; ++i) {
        broadPhase.touchProxy(f.m_proxies[i].proxyId);
      }
    }
  }

  /** Is this body treated like a bullet for continuous collision detection? */
  bool isBullet() {
    return (flags & BULLET_FLAG) == BULLET_FLAG;
  }

  /** Should this body be treated like a bullet for continuous collision detection? */
  void setBullet(bool flag) {
    if (flag) {
      flags |= BULLET_FLAG;
    } else {
      flags &= ~BULLET_FLAG;
    }
  }

  /**
   * You can disable sleeping on this body. If you disable sleeping, the body will be woken.
   *
   * @param flag
   */
  void setSleepingAllowed(bool flag) {
    if (flag) {
      flags |= AUTO_SLEEP_FLAG;
    } else {
      flags &= ~AUTO_SLEEP_FLAG;
      setAwake(true);
    }
  }

  /**
   * Is this body allowed to sleep
   *
   * @return
   */
  bool isSleepingAllowed() {
    return (flags & AUTO_SLEEP_FLAG) == AUTO_SLEEP_FLAG;
  }

  /**
   * Set the sleep state of the body. A sleeping body has very low CPU cost.
   *
   * @param flag set to true to put body to sleep, false to wake it.
   * @param flag
   */
  void setAwake(bool flag) {
    if (flag) {
      if ((flags & AWAKE_FLAG) == 0) {
        flags |= AWAKE_FLAG;
        sleepTime = 0.0;
      }
    } else {
      flags &= ~AWAKE_FLAG;
      sleepTime = 0.0;
      _linearVelocity.setZero();
      _angularVelocity = 0.0;
      force.setZero();
      torque = 0.0;
    }
  }

  /**
   * Get the sleeping state of this body.
   *
   * @return true if the body is awake.
   */
  bool isAwake() {
    return (flags & AWAKE_FLAG) == AWAKE_FLAG;
  }

  /**
   * Set the active state of the body. An inactive body is not simulated and cannot be collided with
   * or woken up. If you pass a flag of true, all fixtures will be added to the broad-phase. If you
   * pass a flag of false, all fixtures will be removed from the broad-phase and all contacts will
   * be destroyed. Fixtures and joints are otherwise unaffected. You may continue to create/destroy
   * fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive
   * and will not participate in collisions, ray-casts, or queries. Joints connected to an inactive
   * body are implicitly inactive. An inactive body is still owned by a World object and remains in
   * the body list.
   *
   * @param flag
   */
  void setActive(bool flag) {
    assert(world.isLocked() == false);

    if (flag == isActive()) {
      return;
    }

    if (flag) {
      flags |= ACTIVE_FLAG;

      // Create all proxies.
      BroadPhase broadPhase = world.m_contactManager.broadPhase;
      for (Fixture f = fixtureList; f != null; f = f.m_next) {
        f.createProxies(broadPhase, transform);
      }

      // Contacts are created the next time step.
    } else {
      flags &= ~ACTIVE_FLAG;

      // Destroy all proxies.
      BroadPhase broadPhase = world.m_contactManager.broadPhase;
      for (Fixture f = fixtureList; f != null; f = f.m_next) {
        f.destroyProxies(broadPhase);
      }

      // Destroy the attached contacts.
      ContactEdge ce = contactList;
      while (ce != null) {
        ContactEdge ce0 = ce;
        ce = ce.next;
        world.m_contactManager.destroy(ce0.contact);
      }
      contactList = null;
    }
  }

  /**
   * Get the active state of the body.
   *
   * @return
   */
  bool isActive() {
    return (flags & ACTIVE_FLAG) == ACTIVE_FLAG;
  }

  /**
   * Set this body to have fixed rotation. This causes the mass to be reset.
   *
   * @param flag
   */
  void setFixedRotation(bool flag) {
    if (flag) {
      flags |= FIXED_ROTATION_FLAG;
    } else {
      flags &= ~FIXED_ROTATION_FLAG;
    }

    resetMassData();
  }

  /**
   * Does this body have fixed rotation?
   *
   * @return
   */
  bool isFixedRotation() {
    return (flags & FIXED_ROTATION_FLAG) == FIXED_ROTATION_FLAG;
  }

  /** Get the list of all fixtures attached to this body. */
  Fixture getFixtureList() {
    return fixtureList;
  }

  /** Get the list of all joints attached to this body. */
  JointEdge getJointList() {
    return jointList;
  }

  /**
   * Get the list of all contacts attached to this body.
   *
   * @warning this list changes during the time step and you may miss some collisions if you don't
   *          use ContactListener.
   */
  ContactEdge getContactList() {
    return contactList;
  }

  /** Get the next body in the world's body list. */
  Body getNext() {
    return next;
  }

  // djm pooling
  final Transform _pxf = new Transform.zero();

  void synchronizeFixtures() {
    final Transform xf1 = _pxf;
    // xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);

    // xf1.q.set(m_sweep.a0);
    // Rot.mulToOutUnsafe(xf1.q, m_sweep.localCenter, xf1.p);
    // xf1.p.mulLocal(-1).addLocal(m_sweep.c0);
    // inlined:
    xf1.q.s = Math.sin(sweep.a0);
    xf1.q.c = Math.cos(sweep.a0);
    xf1.p.x = sweep.c0.x -
        xf1.q.c * sweep.localCenter.x +
        xf1.q.s * sweep.localCenter.y;
    xf1.p.y = sweep.c0.y -
        xf1.q.s * sweep.localCenter.x -
        xf1.q.c * sweep.localCenter.y;
    // end inline

    for (Fixture f = fixtureList; f != null; f = f.m_next) {
      f.synchronize(world.m_contactManager.broadPhase, xf1, transform);
    }
  }

  void synchronizeTransform() {
    // m_xf.q.set(m_sweep.a);
    //
    // // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
    // Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
    // m_xf.p.mulLocal(-1).addLocal(m_sweep.c);
    //
    transform.q.s = Math.sin(sweep.a);
    transform.q.c = Math.cos(sweep.a);
    Rot q = transform.q;
    Vector2 v = sweep.localCenter;
    transform.p.x = sweep.c.x - q.c * v.x + q.s * v.y;
    transform.p.y = sweep.c.y - q.s * v.x - q.c * v.y;
  }

  /**
   * This is used to prevent connected bodies from colliding. It may lie, depending on the
   * collideConnected flag.
   *
   * @param other
   * @return
   */
  bool shouldCollide(Body other) {
    // At least one body should be dynamic.
    if (_bodyType != BodyType.DYNAMIC && other._bodyType != BodyType.DYNAMIC) {
      return false;
    }

    // Does a joint prevent collision?
    for (JointEdge jn = jointList; jn != null; jn = jn.next) {
      if (jn.other == other) {
        if (jn.joint.getCollideConnected() == false) {
          return false;
        }
      }
    }

    return true;
  }

  void advance(double t) {
    // Advance to the new safe time. This doesn't sync the broad-phase.
    sweep.advance(t);
    sweep.c.setFrom(sweep.c0);
    sweep.a = sweep.a0;
    transform.q.setAngle(sweep.a);
    // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
    Rot.mulToOutUnsafe(transform.q, sweep.localCenter, transform.p);
    transform.p.scale(-1.0).add(sweep.c);
  }

  String toString() {
    return "Body[pos: ${position} linVel: ${linearVelocity} angVel: ${angularVelocity}]";
  }
}
