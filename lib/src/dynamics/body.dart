part of forge2d;

/// A rigid body. These are created via World.createBody.
class Body {
  static const int ISLAND_FLAG = 0x0001;
  static const int AWAKE_FLAG = 0x0002;
  static const int AUTO_SLEEP_FLAG = 0x0004;
  static const int BULLET_FLAG = 0x0008;
  static const int FIXED_ROTATION_FLAG = 0x0010;
  static const int ACTIVE_FLAG = 0x0020;
  static const int TOI_FLAG = 0x0040;

  BodyType _bodyType = BodyType.STATIC;

  int _flags = 0;

  int islandIndex = 0;

  /// The body origin transform.
  final Transform _transform = Transform.zero();

  /// The previous transform for particle simulation
  final Transform _xf0 = Transform.zero();

  /// The swept motion for CCD
  final Sweep _sweep = Sweep();

  /// The linear velocity of the center of mass. Do not modify directly, instead
  /// use setLinearVelocity, applyLinearImpulse or applyForce
  final Vector2 linearVelocity = Vector2.zero();
  double _angularVelocity = 0.0;

  final Vector2 _force = Vector2.zero();
  double _torque = 0.0;

  final World world;

  final List<Fixture> fixtures = [];
  final List<Joint> joints = [];
  final List<Contact> contacts = [];

  double _mass = 0.0, _invMass = 0.0;

  // Rotational inertia about the center of mass.
  double inertia = 0.0, inverseInertia = 0.0;

  double linearDamping = 0.0;
  double angularDamping = 0.0;
  double _gravityScale = 0.0;

  double _sleepTime = 0.0;

  /// Use this to store your application specific data.
  Object userData;

  Body(final BodyDef bd, this.world)
      : assert(!bd.position.isInfinite && !bd.position.isNaN),
        assert(!bd.linearVelocity.isInfinite && !bd.linearVelocity.isNaN),
        assert(bd.gravityScale >= 0.0),
        assert(bd.angularDamping >= 0.0),
        assert(bd.linearDamping >= 0.0) {
    _flags = 0;

    if (bd.bullet) {
      _flags |= BULLET_FLAG;
    }
    if (bd.fixedRotation) {
      _flags |= FIXED_ROTATION_FLAG;
    }
    if (bd.allowSleep) {
      _flags |= AUTO_SLEEP_FLAG;
    }
    if (bd.awake) {
      _flags |= AWAKE_FLAG;
    }
    if (bd.active) {
      _flags |= ACTIVE_FLAG;
    }

    _transform.p.setFrom(bd.position);
    _transform.q.setAngle(bd.angle);

    _sweep.localCenter.setZero();
    _sweep.c0.setFrom(_transform.p);
    _sweep.c.setFrom(_transform.p);
    _sweep.a0 = bd.angle;
    _sweep.a = bd.angle;
    _sweep.alpha0 = 0.0;

    linearVelocity.setFrom(bd.linearVelocity);
    _angularVelocity = bd.angularVelocity;

    linearDamping = bd.linearDamping;
    angularDamping = bd.angularDamping;
    _gravityScale = bd.gravityScale;

    _force.setZero();

    _sleepTime = 0.0;

    _bodyType = bd.type;

    if (_bodyType == BodyType.DYNAMIC) {
      _mass = 1.0;
      _invMass = 1.0;
    } else {
      _mass = 0.0;
      _invMass = 0.0;
    }

    inertia = 0.0;
    inverseInertia = 0.0;

    userData = bd.userData;
  }

  /// Creates a fixture and attach it to this body. Use this function if you need to set some fixture
  /// parameters, like friction. Otherwise you can create the fixture directly from a shape. If the
  /// density is non-zero, this function automatically updates the mass of the body. Contacts are not
  /// created until the next time step.
  ///
  /// @param def the fixture definition.
  /// @warning This function is locked during callbacks.
  Fixture createFixture(FixtureDef def) {
    assert(world.isLocked() == false);

    final Fixture fixture = Fixture();
    fixture.create(this, def);

    if ((_flags & ACTIVE_FLAG) == ACTIVE_FLAG) {
      final BroadPhase broadPhase = world._contactManager.broadPhase;
      fixture.createProxies(broadPhase, _transform);
    }

    fixture.body = this;
    fixtures.add(fixture);

    // Adjust mass properties if needed.
    if (fixture._density > 0.0) {
      resetMassData();
    }

    // Let the world know we have a new fixture. This will cause new contacts
    // to be created at the beginning of the next time step.
    world._flags |= World.NEW_FIXTURE;

    return fixture;
  }

  final FixtureDef _fixDef = FixtureDef();

  /// Creates a fixture from a shape and attach it to this body. This is a convenience function. Use
  /// FixtureDef if you need to set parameters like friction, restitution, user data, or filtering.
  /// If the density is non-zero, this function automatically updates the mass of the body.
  ///
  /// @param shape the shape to be cloned.
  /// @param density the shape density (set to zero for static bodies).
  /// @warning This function is locked during callbacks.
  Fixture createFixtureFromShape(Shape shape, [double density = 0.0]) {
    _fixDef.shape = shape;
    _fixDef.density = density;

    return createFixture(_fixDef);
  }

  /// Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts
  /// associated with this fixture. This will automatically adjust the mass of the body if the body
  /// is dynamic and the fixture has positive density. All fixtures attached to a body are implicitly
  /// destroyed when the body is destroyed.
  ///
  /// @param fixture the fixture to be removed.
  /// @warning This function is locked during callbacks.
  void destroyFixture(Fixture fixture) {
    assert(world.isLocked() == false);
    assert(fixture.body == this);

    // Remove the fixture from this body's singly linked list.
    assert(fixtures.isNotEmpty);
    final removed = fixtures.remove(fixture);

    // You tried to remove a shape that is not attached to this body.
    assert(
      removed,
      "You tried to remove a fixture that is not attached to this body",
    );

    // Destroy any contacts associated with the fixture.
    for (Contact contact in contacts) {
      if (fixture == contact.fixtureA || fixture == contact.fixtureB) {
        // This destroys the contact and removes it from
        // this body's contact list.
        world._contactManager.destroy(contact);
      }
    }

    if ((_flags & ACTIVE_FLAG) == ACTIVE_FLAG) {
      final BroadPhase broadPhase = world._contactManager.broadPhase;
      fixture.destroyProxies(broadPhase);
    }

    // Reset the mass data.
    resetMassData();
  }

  /// Set the position of the body's origin and rotation. This breaks any contacts and wakes the
  /// other bodies. Manipulating a body's transform may cause non-physical behavior. Note: contacts
  /// are updated on the next call to World.step().
  ///
  /// @param position the world position of the body's local origin.
  /// @param angle the world rotation in radians.
  void setTransform(Vector2 position, double angle) {
    assert(world.isLocked() == false);
    _transform.q.setAngle(angle);
    _transform.p.setFrom(position);

    _sweep.c.setFrom(Transform.mulVec2(_transform, _sweep.localCenter));
    _sweep.a = angle;

    _sweep.c0.setFrom(_sweep.c);
    _sweep.a0 = _sweep.a;

    final BroadPhase broadPhase = world._contactManager.broadPhase;
    for (Fixture f in fixtures) {
      f.synchronize(broadPhase, _transform, _transform);
    }
  }

  /// Get the world body origin position. Do not modify.
  ///
  /// @return the world position of the body's origin.
  Vector2 get position => _transform.p;

  /// Get the angle in radians.
  ///
  /// @return the current world rotation angle in radians.
  double getAngle() => _sweep.a;

  /// Get the world position of the center of mass. Do not modify.
  Vector2 get worldCenter => _sweep.c;

  /// Get the local position of the center of mass. Do not modify.
  Vector2 getLocalCenter() => _sweep.localCenter;

  /// Set the linear velocity of the center of mass.
  ///
  /// @param v the new linear velocity of the center of mass.
  set linearVelocity(Vector2 v) {
    if (_bodyType == BodyType.STATIC) {
      return;
    }

    if (v.dot(v) > 0.0) {
      setAwake(true);
    }

    linearVelocity.setFrom(v);
  }

  /// Set the angular velocity.
  ///
  /// @param omega the new angular velocity in radians/second.
  set angularVelocity(double w) {
    if (_bodyType == BodyType.STATIC) {
      return;
    }

    if (w * w > 0.0) {
      setAwake(true);
    }

    _angularVelocity = w;
  }

  /// Get the angular velocity.
  ///
  /// @return the angular velocity in radians/second.
  double get angularVelocity => _angularVelocity;

  /// Apply a force at a world point. If the force is not applied at the center of mass, it will
  /// generate a torque and affect the angular velocity. This wakes up the body.
  ///
  /// @param force the world force vector, usually in Newtons (N).
  /// @param point the world position of the point of application (default: center of mass)
  void applyForce(Vector2 force, {Vector2 point}) {
    point ??= worldCenter;
    _applyForceToCenter(force);
    _torque +=
        (point.x - _sweep.c.x) * force.y - (point.y - _sweep.c.y) * force.x;
  }

  /// Apply a force to the center of mass. This wakes up the body.
  ///
  /// @param force the world force vector, usually in Newtons (N).
  void _applyForceToCenter(Vector2 force) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }

    _force.x += force.x;
    _force.y += force.y;
  }

  /// Apply a torque. This affects the angular velocity without affecting the linear velocity of the
  /// center of mass. This wakes up the body.
  ///
  /// @param torque about the z-axis (out of the screen), usually in N-m.
  void applyTorque(double torque) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }

    _torque += torque;
  }

  /// Apply an impulse at a point. This immediately modifies the velocity. It also modifies the
  /// angular velocity if the point of application is not at the center of mass. This wakes up the
  /// body if 'wake' is set to true. If the body is sleeping and 'wake' is false, then there is no
  /// effect.
  ///
  /// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
  /// @param point the world position of the point of application (default: center of mass)
  /// @param wake also wake up the body (default: true)
  void applyLinearImpulse(Vector2 impulse, {Vector2 point, bool wake = true}) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }
    point ??= worldCenter;

    if (!isAwake()) {
      if (wake) {
        setAwake(true);
      } else {
        return;
      }
    }

    linearVelocity += impulse * _invMass;
    _angularVelocity += inverseInertia *
        ((point.x - _sweep.c.x) * impulse.y -
            (point.y - _sweep.c.y) * impulse.x);
  }

  /// Apply an angular impulse.
  ///
  /// @param impulse the angular impulse in units of kg*m*m/s
  void applyAngularImpulse(double impulse) {
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    if (isAwake() == false) {
      setAwake(true);
    }
    _angularVelocity += inverseInertia * impulse;
  }

  /// Get the total mass of the body.
  ///
  /// @return the mass, usually in kilograms (kg).
  double get mass => _mass;

  /// Get the central rotational inertia of the body.
  ///
  /// @return the rotational inertia, usually in kg-m^2.
  double getInertia() {
    return inertia +
        _mass *
            (_sweep.localCenter.x * _sweep.localCenter.x +
                _sweep.localCenter.y * _sweep.localCenter.y);
  }

  /// Get the mass data of the body. The rotational inertia is relative to the center of mass.
  ///
  /// @return a struct containing the mass, inertia and center of the body.
  MassData getMassData() {
    return MassData()
      ..mass = _mass
      ..I = inertia + getInertia()
      ..center.x = _sweep.localCenter.x
      ..center.y = _sweep.localCenter.y;
  }

  /// Set the mass properties to override the mass properties of the fixtures. Note that this changes
  /// the center of mass position. Note that creating or destroying fixtures can also alter the mass.
  /// This function has no effect if the body isn't dynamic.
  ///
  /// @param massData the mass properties.
  void setMassData(MassData massData) {
    // TODO_ERIN adjust linear velocity and torque to account for movement of center.
    assert(world.isLocked() == false);
    if (_bodyType != BodyType.DYNAMIC) {
      return;
    }

    _invMass = 0.0;
    inertia = 0.0;
    inverseInertia = 0.0;

    _mass = massData.mass;
    if (_mass <= 0.0) {
      _mass = 1.0;
    }

    _invMass = 1.0 / _mass;

    if (massData.I > 0.0 && (_flags & FIXED_ROTATION_FLAG) == 0.0) {
      inertia = massData.I - _mass * massData.center.dot(massData.center);
      assert(inertia > 0.0);
      inverseInertia = 1.0 / inertia;
    }

    // Move center of mass.
    final Vector2 oldCenter = Vector2.copy(_sweep.c);
    _sweep.localCenter.setFrom(massData.center);
    _sweep.c0.setFrom(Transform.mulVec2(_transform, _sweep.localCenter));
    _sweep.c.setFrom(_sweep.c0);

    // Update center of mass velocity.
    final Vector2 temp = Vector2.copy(_sweep.c)..sub(oldCenter);
    temp.scaleOrthogonalInto(_angularVelocity, temp);
    linearVelocity.add(temp);
  }

  final MassData _pmd = MassData();

  /// This resets the mass properties to the sum of the mass properties of the fixtures. This
  /// normally does not need to be called unless you called setMassData to override the mass and you
  /// later want to reset the mass.
  void resetMassData() {
    // Compute mass data from shapes. Each shape has its own density.
    _mass = 0.0;
    _invMass = 0.0;
    inertia = 0.0;
    inverseInertia = 0.0;
    _sweep.localCenter.setZero();

    // Static and kinematic bodies have zero mass.
    if (_bodyType == BodyType.STATIC || _bodyType == BodyType.KINEMATIC) {
      _sweep.c0.setFrom(_transform.p);
      _sweep.c.setFrom(_transform.p);
      _sweep.a0 = _sweep.a;
      return;
    }

    assert(_bodyType == BodyType.DYNAMIC);

    // Accumulate mass over all fixtures.
    final Vector2 localCenter = Vector2.zero();
    final Vector2 temp = Vector2.zero();
    final MassData massData = _pmd;
    for (Fixture f in fixtures) {
      if (f._density == 0.0) {
        continue;
      }
      f.getMassData(massData);
      _mass += massData.mass;
      (temp..setFrom(massData.center)).scale(massData.mass);
      localCenter.add(temp);
      inertia += massData.I;
    }

    // Compute center of mass.
    if (_mass > 0.0) {
      _invMass = 1.0 / _mass;
      localCenter.scale(_invMass);
    } else {
      // Force all dynamic bodies to have a positive mass.
      _mass = 1.0;
      _invMass = 1.0;
    }

    if (inertia > 0.0 && (_flags & FIXED_ROTATION_FLAG) == 0.0) {
      // Center the inertia about the center of mass.
      inertia -= _mass * localCenter.dot(localCenter);
      assert(inertia > 0.0);
      inverseInertia = 1.0 / inertia;
    } else {
      inertia = 0.0;
      inverseInertia = 0.0;
    }

    // Move center of mass.
    final Vector2 oldCenter = Vector2.copy(_sweep.c);
    _sweep.localCenter.setFrom(localCenter);
    _sweep.c0.setFrom(Transform.mulVec2(_transform, _sweep.localCenter));
    _sweep.c.setFrom(_sweep.c0);

    // Update center of mass velocity.
    (temp..setFrom(_sweep.c)).sub(oldCenter);

    final Vector2 temp2 = oldCenter;
    temp.scaleOrthogonalInto(_angularVelocity, temp2);
    linearVelocity.add(temp2);
  }

  /// Get the world coordinates of a point given the local coordinates.
  ///
  /// @param localPoint a point on the body measured relative the the body's origin.
  /// @return the same point expressed in world coordinates.
  Vector2 getWorldPoint(Vector2 localPoint) {
    return Transform.mulVec2(_transform, localPoint);
  }

  /// Get the world coordinates of a vector given the local coordinates.
  ///
  /// @param localVector a vector fixed in the body.
  /// @return the same vector expressed in world coordinates.
  Vector2 getWorldVector(Vector2 localVector) {
    return Rot.mulVec2(_transform.q, localVector);
  }

  /// Gets a local point relative to the body's origin given a world point.
  ///
  /// @param a point in world coordinates.
  /// @return the corresponding local point relative to the body's origin.
  Vector2 getLocalPoint(Vector2 worldPoint) {
    return Transform.mulTransVec2(_transform, worldPoint);
  }

  /// Gets a local vector given a world vector.
  ///
  /// @param a vector in world coordinates.
  /// @return the corresponding local vector.
  Vector2 getLocalVector(Vector2 worldVector) {
    return Rot.mulTransVec2(_transform.q, worldVector);
  }

  /// Get the world linear velocity of a world point attached to this body.
  ///
  /// @param a point in world coordinates.
  /// @return the world velocity of a point.
  Vector2 getLinearVelocityFromWorldPoint(Vector2 worldPoint) {
    return Vector2(
      -_angularVelocity * (worldPoint.y - _sweep.c.y) + linearVelocity.x,
      _angularVelocity * (worldPoint.x - _sweep.c.x) + linearVelocity.y,
    );
  }

  /// Get the world velocity of a local point.
  ///
  /// @param a point in local coordinates.
  /// @return the world velocity of a point.
  Vector2 getLinearVelocityFromLocalPoint(Vector2 localPoint) {
    return getLinearVelocityFromWorldPoint(getWorldPoint(localPoint));
  }

  BodyType getType() {
    return _bodyType;
  }

  /// Set the type of this body. This may alter the mass and velocity.
  ///
  /// @param type
  void setType(BodyType type) {
    assert(world.isLocked() == false);
    if (_bodyType == type) {
      return;
    }

    _bodyType = type;

    resetMassData();

    if (_bodyType == BodyType.STATIC) {
      linearVelocity.setZero();
      _angularVelocity = 0.0;
      _sweep.a0 = _sweep.a;
      _sweep.c0.setFrom(_sweep.c);
      synchronizeFixtures();
    }

    setAwake(true);

    _force.setZero();
    _torque = 0.0;

    // Delete the attached contacts.
    contacts.forEach(world._contactManager.destroy);
    contacts.clear();

    // Touch the proxies so that new contacts will be created (when appropriate)
    final BroadPhase broadPhase = world._contactManager.broadPhase;
    for (Fixture f in fixtures) {
      final int proxyCount = f._proxyCount;
      for (int i = 0; i < proxyCount; ++i) {
        broadPhase.touchProxy(f._proxies[i].proxyId);
      }
    }
  }

  /// Is this body treated like a bullet for continuous collision detection?
  bool isBullet() {
    return (_flags & BULLET_FLAG) == BULLET_FLAG;
  }

  /// Should this body be treated like a bullet for continuous collision detection?
  void setBullet(bool flag) {
    if (flag) {
      _flags |= BULLET_FLAG;
    } else {
      _flags &= ~BULLET_FLAG;
    }
  }

  /// You can disable sleeping on this body. If you disable sleeping, the body will be woken.
  ///
  /// @param flag
  void setSleepingAllowed(bool flag) {
    if (flag) {
      _flags |= AUTO_SLEEP_FLAG;
    } else {
      _flags &= ~AUTO_SLEEP_FLAG;
      setAwake(true);
    }
  }

  /// Is this body allowed to sleep
  ///
  /// @return
  bool isSleepingAllowed() {
    return (_flags & AUTO_SLEEP_FLAG) == AUTO_SLEEP_FLAG;
  }

  /// Set the sleep state of the body. A sleeping body has very low CPU cost.
  ///
  /// @param flag set to true to put body to sleep, false to wake it.
  /// @param flag
  void setAwake(bool flag) {
    if (flag) {
      if ((_flags & AWAKE_FLAG) == 0) {
        _flags |= AWAKE_FLAG;
        _sleepTime = 0.0;
      }
    } else {
      _flags &= ~AWAKE_FLAG;
      _sleepTime = 0.0;
      linearVelocity.setZero();
      _angularVelocity = 0.0;
      _force.setZero();
      _torque = 0.0;
    }
  }

  /// Get the sleeping state of this body.
  ///
  /// @return true if the body is awake.
  bool isAwake() {
    return (_flags & AWAKE_FLAG) == AWAKE_FLAG;
  }

  /// Set the active state of the body. An inactive body is not simulated and cannot be collided with
  /// or woken up. If you pass a flag of true, all fixtures will be added to the broad-phase. If you
  /// pass a flag of false, all fixtures will be removed from the broad-phase and all contacts will
  /// be destroyed. Fixtures and joints are otherwise unaffected. You may continue to create/destroy
  /// fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive
  /// and will not participate in collisions, ray-casts, or queries. Joints connected to an inactive
  /// body are implicitly inactive. An inactive body is still owned by a World object and remains in
  /// the body list.
  void setActive(bool flag) {
    assert(world.isLocked() == false);

    if (flag == isActive()) {
      return;
    }

    if (flag) {
      _flags |= ACTIVE_FLAG;

      // Create all proxies.
      final BroadPhase broadPhase = world._contactManager.broadPhase;
      for (Fixture f in fixtures) {
        f.createProxies(broadPhase, _transform);
      }

      // Contacts are created the next time step.
    } else {
      _flags &= ~ACTIVE_FLAG;

      // Destroy all proxies.
      final BroadPhase broadPhase = world._contactManager.broadPhase;
      for (Fixture f in fixtures) {
        f.destroyProxies(broadPhase);
      }

      // Destroy the attached contacts.
      contacts.forEach(world._contactManager.destroy);
      contacts.clear();
    }
  }

  /// Get the active state of the body.
  ///
  /// @return
  bool isActive() {
    return (_flags & ACTIVE_FLAG) == ACTIVE_FLAG;
  }

  /// Set this body to have fixed rotation. This causes the mass to be reset.
  ///
  /// @param flag
  void setFixedRotation(bool flag) {
    if (flag) {
      _flags |= FIXED_ROTATION_FLAG;
    } else {
      _flags &= ~FIXED_ROTATION_FLAG;
    }

    resetMassData();
  }

  /// Does this body have fixed rotation?
  ///
  /// @return
  bool isFixedRotation() {
    return (_flags & FIXED_ROTATION_FLAG) == FIXED_ROTATION_FLAG;
  }

  // djm pooling
  final Transform _pxf = Transform.zero();

  void synchronizeFixtures() {
    final Transform xf1 = _pxf;
    xf1.q.s = math.sin(_sweep.a0);
    xf1.q.c = math.cos(_sweep.a0);
    xf1.p.x = _sweep.c0.x -
        xf1.q.c * _sweep.localCenter.x +
        xf1.q.s * _sweep.localCenter.y;
    xf1.p.y = _sweep.c0.y -
        xf1.q.s * _sweep.localCenter.x -
        xf1.q.c * _sweep.localCenter.y;

    for (Fixture f in fixtures) {
      f.synchronize(world._contactManager.broadPhase, xf1, _transform);
    }
  }

  void synchronizeTransform() {
    _transform.q.s = math.sin(_sweep.a);
    _transform.q.c = math.cos(_sweep.a);
    final Rot q = _transform.q;
    final Vector2 v = _sweep.localCenter;
    _transform.p.x = _sweep.c.x - q.c * v.x + q.s * v.y;
    _transform.p.y = _sweep.c.y - q.s * v.x - q.c * v.y;
  }

  /// This is used to prevent connected bodies from colliding. It may lie, depending on the
  /// collideConnected flag.
  ///
  /// @param other
  /// @return
  bool shouldCollide(Body other) {
    // At least one body should be dynamic.
    if (_bodyType != BodyType.DYNAMIC && other._bodyType != BodyType.DYNAMIC) {
      return false;
    }

    // Does a joint prevent collision?
    for (Joint joint in joints) {
      if (joint.containsBody(other) && !joint.getCollideConnected()) {
        return false;
      }
    }

    return true;
  }

  void advance(double t) {
    // Advance to the new safe time. This doesn't sync the broad-phase.
    _sweep.advance(t);
    _sweep.c.setFrom(_sweep.c0);
    _sweep.a = _sweep.a0;
    _transform.q.setAngle(_sweep.a);
    _transform.p.setFrom(Rot.mulVec2(_transform.q, _sweep.localCenter));
    (_transform.p..scale(-1.0)).add(_sweep.c);
  }

  @override
  String toString() {
    return "Body[pos: $position linVel: $linearVelocity angVel: $angularVelocity]";
  }
}
