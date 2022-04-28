import 'dart:math';

import '../../forge2d.dart';

/// A rigid body. These are created via World.createBody.
class Body {
  static const int islandFlag = 0x0001;
  static const int awakeFlag = 0x0002;
  static const int autoSleepFlag = 0x0004;
  static const int bulletFlag = 0x0008;
  static const int fixedRotationFlag = 0x0010;
  static const int activeFlag = 0x0020;
  static const int toiFlag = 0x0040;

  BodyType _bodyType = BodyType.static;
  BodyType get bodyType => _bodyType;

  int flags = 0;

  int islandIndex = 0;

  /// The body origin transform.
  final Transform transform = Transform.zero();

  /// The previous transform for particle simulation
  final Transform previousTransform = Transform.zero();

  /// The swept motion for CCD
  final Sweep sweep = Sweep();

  /// The linear velocity of the center of mass. Do not modify directly, instead
  /// use applyLinearImpulse or applyForce.
  final Vector2 linearVelocity = Vector2.zero();

  /// The angular velocity in radians/second.
  double _angularVelocity = 0.0;

  double get angularVelocity => _angularVelocity;

  /// Do not modify directly
  final Vector2 force = Vector2.zero();
  double _torque = 0.0;
  double get torque => _torque;

  final World world;

  final List<Fixture> fixtures = [];
  final List<Joint> joints = [];
  final List<Contact> contacts = [];

  double _mass = 0.0;
  double _inverseMass = 0.0;
  double get inverseMass => _inverseMass;

  // Rotational inertia about the center of mass.
  double inertia = 0.0, inverseInertia = 0.0;

  double linearDamping = 0.0;
  double angularDamping = 0.0;

  /// {@macro dynamics.body_def.gravity_override}
  Vector2? gravityOverride;

  /// {@macro dynamics.body_def.gravity_scale}
  Vector2? gravityScale;

  double sleepTime = 0.0;

  /// Use this to store your application specific data.
  Object? userData;

  Body(final BodyDef bd, this.world)
      : assert(!bd.position.isInfinite && !bd.position.isNaN),
        assert(!bd.linearVelocity.isInfinite && !bd.linearVelocity.isNaN),
        assert(bd.angularDamping >= 0.0),
        assert(bd.linearDamping >= 0.0) {
    flags = 0;

    if (bd.bullet) {
      flags |= bulletFlag;
    }
    if (bd.fixedRotation) {
      flags |= fixedRotationFlag;
    }
    if (bd.allowSleep) {
      flags |= autoSleepFlag;
    }
    if (bd.isAwake) {
      flags |= awakeFlag;
    }
    if (bd.active) {
      flags |= activeFlag;
    }

    transform.p.setFrom(bd.position);
    transform.q.setAngle(bd.angle);

    sweep.localCenter.setZero();
    sweep.c0.setFrom(transform.p);
    sweep.c.setFrom(transform.p);
    sweep.a0 = bd.angle;
    sweep.a = bd.angle;
    sweep.alpha0 = 0.0;

    linearVelocity.setFrom(bd.linearVelocity);
    _angularVelocity = bd.angularVelocity;

    linearDamping = bd.linearDamping;
    angularDamping = bd.angularDamping;
    gravityOverride = bd.gravityOverride;
    gravityScale = bd.gravityScale;

    force.setZero();

    sleepTime = 0.0;

    _bodyType = bd.type;

    if (_bodyType == BodyType.dynamic) {
      _mass = 1.0;
      _inverseMass = 1.0;
    } else {
      _mass = 0.0;
      _inverseMass = 0.0;
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
    assert(!world.isLocked);

    final fixture = Fixture(this, def);

    if ((flags & activeFlag) == activeFlag) {
      final broadPhase = world.contactManager.broadPhase;
      fixture.createProxies(broadPhase, transform);
    }

    fixtures.add(fixture);

    // Adjust mass properties if needed.
    if (fixture.density > 0.0) {
      resetMassData();
    }

    // Let the world know we have a new fixture. This will cause new contacts
    // to be created at the beginning of the next time step.
    world.flags |= World.newFixture;

    return fixture;
  }

  /// Creates a fixture from a shape and attach it to this body. This is a convenience function. Use
  /// FixtureDef if you need to set parameters like friction, restitution, user data, or filtering.
  /// If the density is non-zero, this function automatically updates the mass of the body.
  ///
  /// @param shape the shape to be cloned.
  /// @param density the shape density (set to zero for static bodies).
  /// @warning This function is locked during callbacks.
  Fixture createFixtureFromShape(
    Shape shape, {
    double density = 0.0,
    double friction = 0.0,
    double restitution = 0.0,
  }) {
    return createFixture(
      FixtureDef(shape)
        ..density = density
        ..friction = friction
        ..restitution = restitution,
    );
  }

  /// Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts
  /// associated with this fixture. This will automatically adjust the mass of the body if the body
  /// is dynamic and the fixture has positive density. All fixtures attached to a body are implicitly
  /// destroyed when the body is destroyed.
  ///
  /// @param fixture the fixture to be removed.
  /// @warning This function is locked during callbacks.
  void destroyFixture(Fixture fixture) {
    assert(!world.isLocked);
    assert(fixture.body == this);

    // Remove the fixture from this body's singly linked list.
    assert(fixtures.isNotEmpty);
    final removed = fixtures.remove(fixture);

    // You tried to remove a shape that is not attached to this body.
    assert(
      removed,
      'You tried to remove a fixture that is not attached to this body',
    );

    // Destroy any contacts associated with the fixture.
    var i = 0;
    while (i < contacts.length) {
      final contact = contacts[i];
      if (fixture == contact.fixtureA || fixture == contact.fixtureB) {
        // This destroys the contact and removes it from
        // this body's contact list.
        world.contactManager.destroy(contact);
      } else {
        /// Increase index only if contact was not deleted and need move to next one.
        /// If contact was deleted, then index should not be increased.
        i++;
      }
    }

    if ((flags & activeFlag) == activeFlag) {
      final broadPhase = world.contactManager.broadPhase;
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
    assert(!world.isLocked);
    transform.q.setAngle(angle);
    transform.p.setFrom(position);

    sweep.c.setFrom(Transform.mulVec2(transform, sweep.localCenter));
    sweep.a = angle;

    sweep.c0.setFrom(sweep.c);
    sweep.a0 = sweep.a;

    final broadPhase = world.contactManager.broadPhase;
    for (final f in fixtures) {
      f.synchronize(broadPhase, transform, transform);
    }
  }

  /// Get the world body origin position. Do not modify.
  ///
  /// @return the world position of the body's origin.
  Vector2 get position => transform.p;

  /// Get the angle in radians.
  ///
  /// @return the current world rotation angle in radians.
  double get angle => sweep.a;

  /// Get the world position of the center of mass. Do not modify.
  Vector2 get worldCenter => sweep.c;

  /// Get the local position of the center of mass. Do not modify.
  Vector2 getLocalCenter() => sweep.localCenter;

  /// Set the linear velocity of the center of mass.
  ///
  /// @param v the new linear velocity of the center of mass.
  set linearVelocity(Vector2 v) {
    if (_bodyType == BodyType.static) {
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
    if (_bodyType == BodyType.static) {
      return;
    }

    if (w * w > 0.0) {
      setAwake(true);
    }

    _angularVelocity = w;
  }

  /// Apply a force at a world point. If the force is not applied at the center of mass, it will
  /// generate a torque and affect the angular velocity. This wakes up the body.
  ///
  /// @param force the world force vector, usually in Newtons (N).
  /// @param point the world position of the point of application (default: center of mass)
  void applyForce(Vector2 force, {Vector2? point}) {
    point ??= worldCenter;
    _applyForceToCenter(force);
    _torque +=
        (point.x - sweep.c.x) * force.y - (point.y - sweep.c.y) * force.x;
  }

  /// Apply a force to the center of mass. This wakes up the body.
  ///
  /// @param force the world force vector, usually in Newtons (N).
  void _applyForceToCenter(Vector2 force) {
    if (_bodyType != BodyType.dynamic) {
      return;
    }

    if (isAwake == false) {
      setAwake(true);
    }

    this.force.x += force.x;
    this.force.y += force.y;
  }

  /// Apply a torque. This affects the angular velocity without affecting the linear velocity of the
  /// center of mass. This wakes up the body.
  ///
  /// @param torque about the z-axis (out of the screen), usually in N-m.
  void applyTorque(double torque) {
    if (_bodyType != BodyType.dynamic) {
      return;
    }

    if (isAwake == false) {
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
  void applyLinearImpulse(Vector2 impulse, {Vector2? point, bool wake = true}) {
    if (_bodyType != BodyType.dynamic) {
      return;
    }
    point ??= worldCenter;

    if (!isAwake) {
      if (wake) {
        setAwake(true);
      } else {
        return;
      }
    }

    linearVelocity += impulse * _inverseMass;
    _angularVelocity += inverseInertia *
        ((point.x - sweep.c.x) * impulse.y - (point.y - sweep.c.y) * impulse.x);
  }

  /// Apply an angular impulse.
  ///
  /// @param impulse the angular impulse in units of kg*m*m/s
  void applyAngularImpulse(double impulse) {
    if (_bodyType != BodyType.dynamic) {
      return;
    }

    if (isAwake == false) {
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
            (sweep.localCenter.x * sweep.localCenter.x +
                sweep.localCenter.y * sweep.localCenter.y);
  }

  /// Get the mass data of the body. The rotational inertia is relative to the center of mass.
  ///
  /// @return a struct containing the mass, inertia and center of the body.
  MassData getMassData() {
    return MassData()
      ..mass = _mass
      ..I = inertia + getInertia()
      ..center.x = sweep.localCenter.x
      ..center.y = sweep.localCenter.y;
  }

  /// Set the mass properties to override the mass properties of the fixtures. Note that this changes
  /// the center of mass position. Note that creating or destroying fixtures can also alter the mass.
  /// This function has no effect if the body isn't dynamic.
  ///
  /// @param massData the mass properties.
  void setMassData(MassData massData) {
    // TODO_ERIN adjust linear velocity and torque to account for movement of center.
    assert(!world.isLocked);
    if (_bodyType != BodyType.dynamic) {
      return;
    }

    _inverseMass = 0.0;
    inertia = 0.0;
    inverseInertia = 0.0;

    _mass = massData.mass;
    if (_mass <= 0.0) {
      _mass = 1.0;
    }

    _inverseMass = 1.0 / _mass;

    if (massData.I > 0.0 && (flags & fixedRotationFlag) == 0.0) {
      inertia = massData.I - _mass * massData.center.dot(massData.center);
      assert(inertia > 0.0);
      inverseInertia = 1.0 / inertia;
    }

    // Move center of mass.
    final oldCenter = Vector2.copy(sweep.c);
    sweep.localCenter.setFrom(massData.center);
    sweep.c0.setFrom(Transform.mulVec2(transform, sweep.localCenter));
    sweep.c.setFrom(sweep.c0);

    // Update center of mass velocity.
    final temp = Vector2.copy(sweep.c)..sub(oldCenter);
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
    _inverseMass = 0.0;
    inertia = 0.0;
    inverseInertia = 0.0;
    sweep.localCenter.setZero();

    // Static and kinematic bodies have zero mass.
    if (_bodyType == BodyType.static || _bodyType == BodyType.kinematic) {
      sweep.c0.setFrom(transform.p);
      sweep.c.setFrom(transform.p);
      sweep.a0 = sweep.a;
      return;
    }

    assert(_bodyType == BodyType.dynamic);

    // Accumulate mass over all fixtures.
    final localCenter = Vector2.zero();
    final temp = Vector2.zero();
    final massData = _pmd;
    for (final f in fixtures) {
      if (f.density == 0.0) {
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
      _inverseMass = 1.0 / _mass;
      localCenter.scale(_inverseMass);
    } else {
      // Force all dynamic bodies to have a positive mass.
      _mass = 1.0;
      _inverseMass = 1.0;
    }

    if (inertia > 0.0 && (flags & fixedRotationFlag) == 0.0) {
      // Center the inertia about the center of mass.
      inertia -= _mass * localCenter.dot(localCenter);
      assert(inertia > 0.0);
      inverseInertia = 1.0 / inertia;
    } else {
      inertia = 0.0;
      inverseInertia = 0.0;
    }

    // Move center of mass.
    final oldCenter = Vector2.copy(sweep.c);
    sweep.localCenter.setFrom(localCenter);
    sweep.c0.setFrom(Transform.mulVec2(transform, sweep.localCenter));
    sweep.c.setFrom(sweep.c0);

    // Update center of mass velocity.
    (temp..setFrom(sweep.c)).sub(oldCenter);

    final temp2 = oldCenter;
    temp.scaleOrthogonalInto(_angularVelocity, temp2);
    linearVelocity.add(temp2);
  }

  /// Get the world coordinates of a point given the local coordinates.
  ///
  /// @param localPoint a point on the body measured relative the the body's origin.
  /// @return the same point expressed in world coordinates.
  Vector2 worldPoint(Vector2 localPoint) {
    return Transform.mulVec2(transform, localPoint);
  }

  /// Get the world coordinates of a vector given the local coordinates.
  ///
  /// @param localVector a vector fixed in the body.
  /// @return the same vector expressed in world coordinates.
  Vector2 worldVector(Vector2 localVector) {
    return Rot.mulVec2(transform.q, localVector);
  }

  /// Gets a local point relative to the body's origin given a world point.
  ///
  /// @param a point in world coordinates.
  /// @return the corresponding local point relative to the body's origin.
  Vector2 localPoint(Vector2 worldPoint) {
    return Transform.mulTransVec2(transform, worldPoint);
  }

  /// Gets a local vector given a world vector.
  ///
  /// @param a vector in world coordinates.
  /// @return the corresponding local vector.
  Vector2 localVector(Vector2 worldVector) {
    return Rot.mulTransVec2(transform.q, worldVector);
  }

  /// Get the world linear velocity of a world point attached to this body.
  ///
  /// @param a point in world coordinates.
  /// @return the world velocity of a point.
  Vector2 linearVelocityFromWorldPoint(Vector2 worldPoint) {
    return Vector2(
      -_angularVelocity * (worldPoint.y - sweep.c.y) + linearVelocity.x,
      _angularVelocity * (worldPoint.x - sweep.c.x) + linearVelocity.y,
    );
  }

  /// Get the world velocity of a local point.
  ///
  /// @param a point in local coordinates.
  /// @return the world velocity of a point.
  Vector2 linearVelocityFromLocalPoint(Vector2 localPoint) {
    return linearVelocityFromWorldPoint(worldPoint(localPoint));
  }

  /// Remove all the current forces on the body
  void clearForces() {
    force.setZero();
    _torque = 0;
  }

  /// Set the type of this body. This may alter the mass and velocity.
  ///
  /// @param type
  void setType(BodyType type) {
    assert(!world.isLocked);
    if (_bodyType == type) {
      return;
    }

    _bodyType = type;

    resetMassData();

    if (_bodyType == BodyType.static) {
      linearVelocity.setZero();
      _angularVelocity = 0.0;
      sweep.a0 = sweep.a;
      sweep.c0.setFrom(sweep.c);
      synchronizeFixtures();
    }

    setAwake(true);

    force.setZero();
    _torque = 0.0;

    // Delete the attached contacts.
    while (contacts.isNotEmpty) {
      world.contactManager.destroy(contacts.first);
    }
    contacts.clear();

    // Touch the proxies so that new contacts will be created (when appropriate)
    final broadPhase = world.contactManager.broadPhase;
    for (final f in fixtures) {
      final proxyCount = f.proxyCount;
      for (var i = 0; i < proxyCount; ++i) {
        broadPhase.touchProxy(f.proxies[i].proxyId);
      }
    }
  }

  /// Is this body treated like a bullet for continuous collision detection?
  bool isBullet() {
    return (flags & bulletFlag) == bulletFlag;
  }

  /// Should this body be treated like a bullet for continuous collision detection?
  void setBullet(bool flag) {
    if (flag) {
      flags |= bulletFlag;
    } else {
      flags &= ~bulletFlag;
    }
  }

  /// You can disable sleeping on this body. If you disable sleeping, the body will be woken.
  ///
  /// @param flag
  void setSleepingAllowed(bool flag) {
    if (flag) {
      flags |= autoSleepFlag;
    } else {
      flags &= ~autoSleepFlag;
      setAwake(true);
    }
  }

  /// Is this body allowed to sleep
  ///
  /// @return
  bool isSleepingAllowed() {
    return (flags & autoSleepFlag) == autoSleepFlag;
  }

  /// Set the sleep state of the body. A sleeping body has very low CPU cost.
  ///
  /// @param awaken set to false to put body to sleep, true to wake it.
  void setAwake(bool awaken) {
    if (awaken) {
      if ((flags & awakeFlag) == 0) {
        flags |= awakeFlag;
        sleepTime = 0.0;
      }
    } else {
      flags &= ~awakeFlag;
      sleepTime = 0.0;
      linearVelocity.setZero();
      _angularVelocity = 0.0;
      force.setZero();
      _torque = 0.0;
    }
  }

  /// Get the sleeping state of this body.
  ///
  /// @return true if the body is awake.
  bool get isAwake => (flags & awakeFlag) == awakeFlag;

  /// Set the active state of the body. An inactive body is not simulated and cannot be collided with
  /// or woken up. If you pass a flag of true, all fixtures will be added to the broad-phase. If you
  /// pass a flag of false, all fixtures will be removed from the broad-phase and all contacts will
  /// be destroyed. Fixtures and joints are otherwise unaffected. You may continue to create/destroy
  /// fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive
  /// and will not participate in collisions, ray-casts, or queries. Joints connected to an inactive
  /// body are implicitly inactive. An inactive body is still owned by a World object and remains in
  /// the body list.
  void setActive(bool flag) {
    assert(!world.isLocked);

    if (flag == isActive) {
      return;
    }

    if (flag) {
      flags |= activeFlag;

      // Create all proxies.
      final broadPhase = world.contactManager.broadPhase;
      for (final f in fixtures) {
        f.createProxies(broadPhase, transform);
      }

      // Contacts are created the next time step.
    } else {
      flags &= ~activeFlag;

      // Destroy all proxies.
      final broadPhase = world.contactManager.broadPhase;
      for (final f in fixtures) {
        f.destroyProxies(broadPhase);
      }

      // Destroy the attached contacts.
      while (contacts.isNotEmpty) {
        world.contactManager.destroy(contacts.first);
      }
      contacts.clear();
    }
  }

  /// Get the active state of the body.
  ///
  /// @return
  bool get isActive => (flags & activeFlag) == activeFlag;

  /// Set this body to have fixed rotation. This causes the mass to be reset.
  ///
  /// @param flag
  void setFixedRotation(bool flag) {
    if (flag) {
      flags |= fixedRotationFlag;
    } else {
      flags &= ~fixedRotationFlag;
    }

    resetMassData();
  }

  /// Does this body have fixed rotation?
  ///
  /// @return
  bool isFixedRotation() {
    return (flags & fixedRotationFlag) == fixedRotationFlag;
  }

  // djm pooling
  final Transform _pxf = Transform.zero();

  void synchronizeFixtures() {
    final xf1 = _pxf;
    xf1.q.s = sin(sweep.a0);
    xf1.q.c = cos(sweep.a0);
    xf1.p.x = sweep.c0.x -
        xf1.q.c * sweep.localCenter.x +
        xf1.q.s * sweep.localCenter.y;
    xf1.p.y = sweep.c0.y -
        xf1.q.s * sweep.localCenter.x -
        xf1.q.c * sweep.localCenter.y;

    for (final f in fixtures) {
      f.synchronize(world.contactManager.broadPhase, xf1, transform);
    }
  }

  void synchronizeTransform() {
    transform.q.s = sin(sweep.a);
    transform.q.c = cos(sweep.a);
    final q = transform.q;
    final v = sweep.localCenter;
    transform.p.x = sweep.c.x - q.c * v.x + q.s * v.y;
    transform.p.y = sweep.c.y - q.s * v.x - q.c * v.y;
  }

  /// This is used to prevent connected bodies from colliding. It may lie, depending on the
  /// collideConnected flag.
  ///
  /// @param other
  /// @return
  bool shouldCollide(Body other) {
    // At least one body should be dynamic.
    if (_bodyType != BodyType.dynamic && other._bodyType != BodyType.dynamic) {
      return false;
    }

    // Does a joint prevent collision?
    for (final joint in joints) {
      if (joint.containsBody(other) && !joint.collideConnected) {
        return false;
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
    transform.p.setFrom(Rot.mulVec2(transform.q, sweep.localCenter));
    (transform.p..scale(-1.0)).add(sweep.c);
  }

  @override
  String toString() {
    return 'Body[pos: $position linVel: $linearVelocity angVel: $angularVelocity]';
  }
}
