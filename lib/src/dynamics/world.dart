part of forge2d;

/// The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
/// world also contains efficient memory management facilities.
class World {
  static const int WORLD_POOL_SIZE = 100;
  static const int WORLD_POOL_CONTAINER_SIZE = 10;

  static const int NEW_FIXTURE = 0x0001;
  static const int LOCKED = 0x0002;
  static const int CLEAR_FORCES = 0x0004;

  // TODO.spydon: Don't have these fields as static
  static final Distance distance = Distance();
  static final Collision collision = Collision();
  static final TimeOfImpact toi = TimeOfImpact();

  int _flags = 0;

  ContactManager _contactManager;
  final List<Body> bodies = <Body>[];
  final List<Joint> joints = <Joint>[];

  final Vector2 _gravity;
  bool _allowSleep = false;

  DestructionListener _destructionListener;
  ParticleDestructionListener _particleDestructionListener;
  DebugDraw debugDraw;

  /// This is used to compute the time step ratio to support a variable time step.
  double _invDt0 = 0.0;

  // these are for debugging the solver
  bool _warmStarting = false;
  bool _continuousPhysics = false;
  bool _subStepping = false;

  bool _stepComplete = false;

  Profile _profile;

  ParticleSystem _particleSystem;

  /// Construct a world object.
  ///
  /// @param gravity the world gravity vector.
  /// @param broadPhase what type of broad phase strategy that should be used.
  World([Vector2 gravity, BroadPhase broadPhase])
      : _gravity = Vector2.copy(gravity ?? Vector2.zero()) {
    broadPhase ??= DefaultBroadPhaseBuffer(DynamicTree());

    _warmStarting = true;
    _continuousPhysics = true;
    _subStepping = false;
    _stepComplete = true;

    _allowSleep = true;

    _flags = CLEAR_FORCES;

    _invDt0 = 0.0;

    _contactManager = ContactManager(broadPhase);
    _profile = Profile();

    _particleSystem = ParticleSystem(this);
  }

  void setAllowSleep(bool flag) {
    if (flag == _allowSleep) {
      return;
    }

    _allowSleep = flag;
    if (_allowSleep == false) {
      for (Body b in bodies) {
        b.setAwake(true);
      }
    }
  }

  void setSubStepping(bool subStepping) {
    _subStepping = subStepping;
  }

  bool isSubStepping() {
    return _subStepping;
  }

  bool isAllowSleep() {
    return _allowSleep;
  }

  DestructionListener getDestructionListener() {
    return _destructionListener;
  }

  ParticleDestructionListener getParticleDestructionListener() {
    return _particleDestructionListener;
  }

  void setParticleDestructionListener(ParticleDestructionListener listener) {
    _particleDestructionListener = listener;
  }

  /// Register a destruction listener. The listener is owned by you and must remain in scope.
  ///
  /// @param listener
  void setDestructionListener(DestructionListener listener) {
    _destructionListener = listener;
  }

  /// Register a contact filter to provide specific control over collision. Otherwise the default
  /// filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
  ///
  /// @param filter
  void setContactFilter(ContactFilter filter) {
    _contactManager.contactFilter = filter;
  }

  /// Register a contact event listener. The listener is owned by you and must remain in scope.
  ///
  /// @param listener
  void setContactListener(ContactListener listener) {
    _contactManager.contactListener = listener;
  }

  /// Register a routine for debug drawing. The debug draw functions are called inside with
  /// World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
  ///
  /// @param debugDraw
  void setDebugDraw(DebugDraw debugDraw) {
    debugDraw = debugDraw;
  }

  /// create a rigid body given a definition. No reference to the definition is retained.
  ///
  /// @warning This function is locked during callbacks.
  /// @param def
  /// @return
  Body createBody(BodyDef def) {
    assert(isLocked() == false);
    final Body body = Body(def, this);
    bodies.add(body);
    return body;
  }

  /// Destroys a rigid body given a definition. No reference to the definition is retained. This
  /// function is locked during callbacks.
  ///
  /// @warning This automatically deletes all associated shapes and joints.
  /// @warning This function is locked during callbacks.
  void destroyBody(Body body) {
    assert(bodies.isNotEmpty);
    assert(isLocked() == false);

    // Delete the attached joints.
    JointEdge je = body._jointList;
    while (je != null) {
      final JointEdge je0 = je;
      je = je.next;
      if (_destructionListener != null) {
        _destructionListener.sayGoodbyeJoint(je0.joint);
      }

      destroyJoint(je0.joint);
      body._jointList = je;
    }
    body._jointList = null;

    // Delete the attached contacts.
    ContactEdge ce = body._contactList;
    while (ce != null) {
      final ContactEdge ce0 = ce;
      ce = ce.next;
      _contactManager.destroy(ce0.contact);
    }
    body._contactList = null;

    Fixture f = body._fixtureList;
    while (f != null) {
      final Fixture f0 = f;
      f = f._next;

      if (_destructionListener != null) {
        _destructionListener.sayGoodbyeFixture(f0);
      }

      f0.destroyProxies(_contactManager.broadPhase);
      f0.destroy();
      body._fixtureList = f;
      body._fixtureCount -= 1;
    }
    body._fixtureList = null;
    body._fixtureCount = 0;

    bodies.remove(body);
  }

  /// Create a joint to constrain bodies together. No reference to the definition is retained.
  /// This may cause the connected bodies to cease colliding.
  ///
  /// @warning This function is locked during callbacks.
  Joint createJoint(JointDef def) {
    assert(isLocked() == false);

    final Joint joint = Joint.create(this, def);
    joints.add(joint);

    // Connect to the bodies' doubly linked lists.
    joint._edgeA.joint = joint;
    joint._edgeA.other = joint.getBodyB();
    joint._edgeA.prev = null;
    joint._edgeA.next = joint.getBodyA()._jointList;
    if (joint.getBodyA()._jointList != null) {
      joint.getBodyA()._jointList.prev = joint._edgeA;
    }
    joint.getBodyA()._jointList = joint._edgeA;

    joint._edgeB.joint = joint;
    joint._edgeB.other = joint.getBodyA();
    joint._edgeB.prev = null;
    joint._edgeB.next = joint.getBodyB()._jointList;
    if (joint.getBodyB()._jointList != null) {
      joint.getBodyB()._jointList.prev = joint._edgeB;
    }
    joint.getBodyB()._jointList = joint._edgeB;

    final Body bodyA = def.bodyA;
    final Body bodyB = def.bodyB;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (def.collideConnected == false) {
      ContactEdge edge = bodyB.getContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          edge.contact.flagForFiltering();
        }

        edge = edge.next;
      }
    }

    // Note: creating a joint doesn't wake the bodies.

    return joint;
  }

  /// destroy a joint. This may cause the connected bodies to begin colliding.
  ///
  /// @warning This function is locked during callbacks.
  /// @param joint
  void destroyJoint(Joint j) {
    assert(isLocked() == false);

    final bool collideConnected = j.getCollideConnected();
    joints.remove(j);

    // Disconnect from island graph.
    final Body bodyA = j.getBodyA();
    final Body bodyB = j.getBodyB();

    // Wake up connected bodies.
    bodyA.setAwake(true);
    bodyB.setAwake(true);

    // Remove from body 1.
    if (j._edgeA.prev != null) {
      j._edgeA.prev.next = j._edgeA.next;
    }

    if (j._edgeA.next != null) {
      j._edgeA.next.prev = j._edgeA.prev;
    }

    if (j._edgeA == bodyA._jointList) {
      bodyA._jointList = j._edgeA.next;
    }

    j._edgeA.prev = null;
    j._edgeA.next = null;

    // Remove from body 2
    if (j._edgeB.prev != null) {
      j._edgeB.prev.next = j._edgeB.next;
    }

    if (j._edgeB.next != null) {
      j._edgeB.next.prev = j._edgeB.prev;
    }

    if (j._edgeB == bodyB._jointList) {
      bodyB._jointList = j._edgeB.next;
    }

    j._edgeB.prev = null;
    j._edgeB.next = null;

    Joint.destroy(j);

    assert(joints.isNotEmpty);

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (collideConnected == false) {
      ContactEdge edge = bodyB.getContactList();
      while (edge != null) {
        if (edge.other == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          edge.contact.flagForFiltering();
        }

        edge = edge.next;
      }
    }
  }

  // djm pooling
  // TODO(srdjan): Make fields private.
  final TimeStep step = TimeStep();
  final Timer stepTimer = Timer();
  final Timer tempTimer = Timer();

  /// Take a time step. This performs collision detection, integration, and constraint solution.
  ///
  /// @param timeStep the amount of time to simulate, this should not vary.
  /// @param velocityIterations for the velocity constraint solver.
  /// @param positionIterations for the position constraint solver.
  void stepDt(double dt, int velocityIterations, int positionIterations) {
    stepTimer.reset();
    tempTimer.reset();
    // If new fixtures were added, we need to find the new contacts.
    if ((_flags & NEW_FIXTURE) == NEW_FIXTURE) {
      _contactManager.findNewContacts();
      _flags &= ~NEW_FIXTURE;
    }

    _flags |= LOCKED;

    step.dt = dt;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    if (dt > 0.0) {
      step.invDt = 1.0 / dt;
    } else {
      step.invDt = 0.0;
    }

    step.dtRatio = _invDt0 * dt;

    step.warmStarting = _warmStarting;
    _profile.stepInit.record(tempTimer.getMilliseconds());

    // Update contacts. This is where some contacts are destroyed.
    tempTimer.reset();
    _contactManager.collide();
    _profile.collide.record(tempTimer.getMilliseconds());

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (_stepComplete && step.dt > 0.0) {
      tempTimer.reset();
      _particleSystem.solve(step); // Particle Simulation
      _profile.solveParticleSystem.record(tempTimer.getMilliseconds());
      tempTimer.reset();
      solve(step);
      _profile.solve.record(tempTimer.getMilliseconds());
    }

    // Handle TOI events.
    if (_continuousPhysics && step.dt > 0.0) {
      tempTimer.reset();
      solveTOI(step);
      _profile.solveTOI.record(tempTimer.getMilliseconds());
    }

    if (step.dt > 0.0) {
      _invDt0 = step.invDt;
    }

    if ((_flags & CLEAR_FORCES) == CLEAR_FORCES) {
      clearForces();
    }

    _flags &= ~LOCKED;

    _profile.step.record(stepTimer.getMilliseconds());
  }

  /// Call this after you are done with time steps to clear the forces. You normally call this after
  /// each call to Step, unless you are performing sub-steps. By default, forces will be
  /// automatically cleared, so you don't need to call this function.
  ///
  /// @see setAutoClearForces
  void clearForces() {
    for (Body body in bodies) {
      body._force.setZero();
      body._torque = 0.0;
    }
  }

  final Color3i color = Color3i.zero();
  final Transform xf = Transform.zero();
  final Vector2 cA = Vector2.zero();
  final Vector2 cB = Vector2.zero();

  /// Call this to draw shapes and other debug draw data.
  void drawDebugData() {
    if (debugDraw == null) {
      return;
    }

    final int flags = debugDraw.drawFlags;
    final bool wireframe = (flags & DebugDraw.WIREFRAME_DRAWING_BIT) != 0;

    if ((flags & DebugDraw.SHAPE_BIT) != 0) {
      for (Body b in bodies) {
        xf.set(b._transform);
        for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
          if (b.isActive() == false) {
            color.setFromRGBd(0.5, 0.5, 0.3);
            drawShape(f, xf, color, wireframe);
          } else if (b.getType() == BodyType.STATIC) {
            color.setFromRGBd(0.5, 0.9, 0.3);
            drawShape(f, xf, color, wireframe);
          } else if (b.getType() == BodyType.KINEMATIC) {
            color.setFromRGBd(0.5, 0.5, 0.9);
            drawShape(f, xf, color, wireframe);
          } else if (b.isAwake() == false) {
            color.setFromRGBd(0.5, 0.5, 0.5);
            drawShape(f, xf, color, wireframe);
          } else {
            color.setFromRGBd(0.9, 0.7, 0.7);
            drawShape(f, xf, color, wireframe);
          }
        }
      }
      drawParticleSystem(_particleSystem);
    }

    if ((flags & DebugDraw.JOINT_BIT) != 0) {
      joints.forEach(drawJoint);
    }

    if ((flags & DebugDraw.PAIR_BIT) != 0) {
      color.setFromRGBd(0.3, 0.9, 0.9);
      for (Contact c = _contactManager.contactList;
          c != null;
          c = c.getNext()) {
        final Fixture fixtureA = c.fixtureA;
        final Fixture fixtureB = c.fixtureB;
        cA.setFrom(fixtureA.getAABB(c.getChildIndexA()).getCenter());
        cB.setFrom(fixtureB.getAABB(c.getChildIndexB()).getCenter());
        debugDraw.drawSegment(cA, cB, color);
      }
    }

    if ((flags & DebugDraw.AABB_BIT) != 0) {
      color.setFromRGBd(0.9, 0.3, 0.9);

      for (Body b in bodies) {
        if (b.isActive() == false) {
          continue;
        }

        for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
          for (int i = 0; i < f._proxyCount; ++i) {
            final FixtureProxy proxy = f._proxies[i];
            final AABB aabb =
                _contactManager.broadPhase.getFatAABB(proxy.proxyId);
            if (aabb != null) {
              final List<Vector2> vs = List<Vector2>(4);
              vs[0].setValues(aabb.lowerBound.x, aabb.lowerBound.y);
              vs[1].setValues(aabb.upperBound.x, aabb.lowerBound.y);
              vs[2].setValues(aabb.upperBound.x, aabb.upperBound.y);
              vs[3].setValues(aabb.lowerBound.x, aabb.upperBound.y);
              debugDraw.drawPolygon(vs, 4, color);
            }
          }
        }
      }
    }

    if ((flags & DebugDraw.CENTER_OF_MASS_BIT) != 0) {
      final Color3i xfColor = Color3i(255, 0, 0);
      for (Body b in bodies) {
        xf.set(b._transform);
        xf.p.setFrom(b.worldCenter);
        debugDraw.drawTransform(xf, xfColor);
      }
    }

    if ((flags & DebugDraw.DYNAMIC_TREE_BIT) != 0) {
      _contactManager.broadPhase.drawTree(debugDraw);
    }

    debugDraw.flush();
  }

  final WorldQueryWrapper wqwrapper = WorldQueryWrapper();

  /// Query the world for all fixtures that potentially overlap the provided AABB.
  ///
  /// @param callback a user implemented callback class.
  /// @param aabb the query box.
  void queryAABB(QueryCallback callback, AABB aabb) {
    wqwrapper.broadPhase = _contactManager.broadPhase;
    wqwrapper.callback = callback;
    _contactManager.broadPhase.query(wqwrapper, aabb);
  }

  /// Query the world for all fixtures and particles that potentially overlap the provided AABB.
  ///
  /// @param callback a user implemented callback class.
  /// @param particleCallback callback for particles.
  /// @param aabb the query box.
  void queryAABBTwoCallbacks(QueryCallback callback,
      ParticleQueryCallback particleCallback, AABB aabb) {
    wqwrapper.broadPhase = _contactManager.broadPhase;
    wqwrapper.callback = callback;
    _contactManager.broadPhase.query(wqwrapper, aabb);
    _particleSystem.queryAABB(particleCallback, aabb);
  }

  /// Query the world for all particles that potentially overlap the provided AABB.
  ///
  /// @param particleCallback callback for particles.
  /// @param aabb the query box.
  void queryAABBParticle(ParticleQueryCallback particleCallback, AABB aabb) {
    _particleSystem.queryAABB(particleCallback, aabb);
  }

  final WorldRayCastWrapper wrcwrapper = WorldRayCastWrapper();
  final RayCastInput input = RayCastInput();

  /// Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
  /// get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
  /// starting point.
  ///
  /// @param callback a user implemented callback class.
  /// @param point1 the ray starting point
  /// @param point2 the ray ending point
  void raycast(RayCastCallback callback, Vector2 point1, Vector2 point2) {
    wrcwrapper.broadPhase = _contactManager.broadPhase;
    wrcwrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setFrom(point1);
    input.p2.setFrom(point2);
    _contactManager.broadPhase.raycast(wrcwrapper, input);
  }

  /// Ray-cast the world for all fixtures and particles in the path of the ray. Your callback
  /// controls whether you get the closest point, any point, or n-points. The ray-cast ignores shapes
  /// that contain the starting point.
  ///
  /// @param callback a user implemented callback class.
  /// @param particleCallback the particle callback class.
  /// @param point1 the ray starting point
  /// @param point2 the ray ending point
  void raycastTwoCallBacks(
      RayCastCallback callback,
      ParticleRaycastCallback particleCallback,
      Vector2 point1,
      Vector2 point2) {
    wrcwrapper.broadPhase = _contactManager.broadPhase;
    wrcwrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setFrom(point1);
    input.p2.setFrom(point2);
    _contactManager.broadPhase.raycast(wrcwrapper, input);
    _particleSystem.raycast(particleCallback, point1, point2);
  }

  /// Ray-cast the world for all particles in the path of the ray. Your callback controls whether you
  /// get the closest point, any point, or n-points.
  ///
  /// @param particleCallback the particle callback class.
  /// @param point1 the ray starting point
  /// @param point2 the ray ending point
  void raycastParticle(ParticleRaycastCallback particleCallback, Vector2 point1,
      Vector2 point2) {
    _particleSystem.raycast(particleCallback, point1, point2);
  }

  /// Get the world contact list. With the returned contact, use Contact.getNext to get the next
  /// contact in the world list. A null contact indicates the end of the list.
  ///
  /// @return the head of the world contact list.
  /// @warning contacts are created and destroyed in the middle of a time step. Use ContactListener
  ///          to avoid missing contacts.
  Contact getContactList() {
    return _contactManager.contactList;
  }

  /// Get the number of broad-phase proxies.
  int getProxyCount() {
    return _contactManager.broadPhase.getProxyCount();
  }

  /// Get the number of contacts (each may have 0 or more contact points).
  int getContactCount() {
    return _contactManager.contactCount;
  }

  /// Gets the height of the dynamic tree
  int getTreeHeight() {
    return _contactManager.broadPhase.getTreeHeight();
  }

  /// Gets the balance of the dynamic tree
  int getTreeBalance() {
    return _contactManager.broadPhase.getTreeBalance();
  }

  /// Gets the quality of the dynamic tree
  double getTreeQuality() {
    return _contactManager.broadPhase.getTreeQuality();
  }

  /// Change the global gravity vector.
  void setGravity(Vector2 gravity) {
    _gravity.setFrom(gravity);
  }

  /// Get the global gravity vector.
  Vector2 getGravity() {
    return _gravity;
  }

  /// Is the world locked (in the middle of a time step).
  bool isLocked() {
    return (_flags & LOCKED) == LOCKED;
  }

  /// Set flag to control automatic clearing of forces after each time step.
  void setAutoClearForces(bool flag) {
    if (flag) {
      _flags |= CLEAR_FORCES;
    } else {
      _flags &= ~CLEAR_FORCES;
    }
  }

  /// Get the flag that controls automatic clearing of forces after each time step.
  bool getAutoClearForces() {
    return (_flags & CLEAR_FORCES) == CLEAR_FORCES;
  }

  final Island island = Island();
  // TODO djm find a good initial stack number;
  List<Body> stack = List<Body>(10);
  final Timer broadphaseTimer = Timer();

  void solve(TimeStep step) {
    _profile.solveInit.startAccum();
    _profile.solveVelocity.startAccum();
    _profile.solvePosition.startAccum();

    // update previous transforms
    for (Body b in bodies) {
      b._xf0.set(b._transform);
    }

    // Size the island for the worst case.
    island.init(_contactManager.contactListener);

    // Clear all the island flags.
    for (Body b in bodies) {
      b._flags &= ~Body.ISLAND_FLAG;
    }
    for (Contact c = _contactManager.contactList; c != null; c = c._next) {
      c.flags &= ~Contact.ISLAND_FLAG;
    }
    for (Joint j in joints) {
      j._islandFlag = false;
    }

    // Build and simulate all awake islands.
    final int stackSize = bodies.length;
    if (stack.length < stackSize) {
      stack = List<Body>(stackSize);
    }
    for (Body seed in bodies) {
      if ((seed._flags & Body.ISLAND_FLAG) == Body.ISLAND_FLAG) {
        continue;
      }

      if (seed.isAwake() == false || seed.isActive() == false) {
        continue;
      }

      // The seed can be dynamic or kinematic.
      if (seed.getType() == BodyType.STATIC) {
        continue;
      }

      // Reset island and stack.
      island.clear();
      int stackCount = 0;
      stack[stackCount++] = seed;
      seed._flags |= Body.ISLAND_FLAG;

      // Perform a depth first search (DFS) on the constraint graph.
      while (stackCount > 0) {
        // Grab the next body off the stack and add it to the island.
        final Body b = stack[--stackCount];
        assert(b.isActive() == true);
        island.addBody(b);

        // Make sure the body is awake.
        b.setAwake(true);

        // To keep islands as small as possible, we don't
        // propagate islands across static bodies.
        if (b.getType() == BodyType.STATIC) {
          continue;
        }

        // Search all contacts connected to this body.
        for (ContactEdge ce = b._contactList; ce != null; ce = ce.next) {
          final Contact contact = ce.contact;

          // Has this contact already been added to an island?
          if ((contact.flags & Contact.ISLAND_FLAG) == Contact.ISLAND_FLAG) {
            continue;
          }

          // Is this contact solid and touching?
          if (contact.isEnabled() == false || contact.isTouching() == false) {
            continue;
          }

          // Skip sensors.
          final bool sensorA = contact._fixtureA._isSensor;
          final bool sensorB = contact._fixtureB._isSensor;
          if (sensorA || sensorB) {
            continue;
          }

          island.addContact(contact);
          contact.flags |= Contact.ISLAND_FLAG;

          final Body other = ce.other;

          // Was the other body already added to this island?
          if ((other._flags & Body.ISLAND_FLAG) == Body.ISLAND_FLAG) {
            continue;
          }

          assert(stackCount < stackSize);
          stack[stackCount++] = other;
          other._flags |= Body.ISLAND_FLAG;
        }

        // Search all joints connect to this body.
        for (JointEdge je = b._jointList; je != null; je = je.next) {
          if (je.joint._islandFlag == true) {
            continue;
          }

          final Body other = je.other;

          // Don't simulate joints connected to inactive bodies.
          if (other.isActive() == false) {
            continue;
          }

          island.addJoint(je.joint);
          je.joint._islandFlag = true;

          if ((other._flags & Body.ISLAND_FLAG) == Body.ISLAND_FLAG) {
            continue;
          }

          assert(stackCount < stackSize);
          stack[stackCount++] = other;
          other._flags |= Body.ISLAND_FLAG;
        }
      }
      island.solve(_profile, step, _gravity, _allowSleep);

      // Post solve cleanup.
      for (BodyMeta bodyMeta in island._bodies) {
        // Allow static bodies to participate in other islands.
        final Body b = bodyMeta.body;
        if (b.getType() == BodyType.STATIC) {
          b._flags &= ~Body.ISLAND_FLAG;
        }
      }
    }
    _profile.solveInit.endAccum();
    _profile.solveVelocity.endAccum();
    _profile.solvePosition.endAccum();

    broadphaseTimer.reset();
    // Synchronize fixtures, check for out of range bodies.
    for (Body b in bodies) {
      // If a body was not in an island then it did not move.
      if ((b._flags & Body.ISLAND_FLAG) == 0) {
        continue;
      }

      if (b.getType() == BodyType.STATIC) {
        continue;
      }

      // Update fixtures (for broad-phase).
      b.synchronizeFixtures();
    }

    // Look for new contacts.
    _contactManager.findNewContacts();
    _profile.broadphase.record(broadphaseTimer.getMilliseconds());
  }

  final Island toiIsland = Island();
  final TOIInput toiInput = TOIInput();
  final TOIOutput toiOutput = TOIOutput();
  final TimeStep subStep = TimeStep();
  final List<Body> tempBodies = List<Body>(2);
  final Sweep backup1 = Sweep();
  final Sweep backup2 = Sweep();

  void solveTOI(final TimeStep step) {
    final Island island = toiIsland..init(_contactManager.contactListener);
    if (_stepComplete) {
      for (Body b in bodies) {
        b._flags &= ~Body.ISLAND_FLAG;
        b._sweep.alpha0 = 0.0;
      }

      for (Contact c = _contactManager.contactList; c != null; c = c._next) {
        // Invalidate TOI
        c.flags &= ~(Contact.TOI_FLAG | Contact.ISLAND_FLAG);
        c.toiCount = 0;
        c.toi = 1.0;
      }
    }

    // Find TOI events and solve them.
    while (true) {
      // Find the first TOI.
      Contact minContact;
      double minAlpha = 1.0;

      for (Contact c = _contactManager.contactList; c != null; c = c._next) {
        // Is this contact disabled?
        if (c.isEnabled() == false) {
          continue;
        }

        // Prevent excessive sub-stepping.
        if (c.toiCount > settings.maxSubSteps) {
          continue;
        }

        double alpha = 1.0;
        if ((c.flags & Contact.TOI_FLAG) != 0) {
          // This contact has a valid cached TOI.
          alpha = c.toi;
        } else {
          final Fixture fA = c.fixtureA;
          final Fixture fB = c.fixtureB;

          // Is there a sensor?
          if (fA.isSensor() || fB.isSensor()) {
            continue;
          }

          final Body bA = fA.getBody();
          final Body bB = fB.getBody();

          final BodyType typeA = bA._bodyType;
          final BodyType typeB = bB._bodyType;
          assert(typeA == BodyType.DYNAMIC || typeB == BodyType.DYNAMIC);

          final bool activeA = bA.isAwake() && typeA != BodyType.STATIC;
          final bool activeB = bB.isAwake() && typeB != BodyType.STATIC;

          // Is at least one body active (awake and dynamic or kinematic)?
          if (activeA == false && activeB == false) {
            continue;
          }

          final bool collideA = bA.isBullet() || typeA != BodyType.DYNAMIC;
          final bool collideB = bB.isBullet() || typeB != BodyType.DYNAMIC;

          // Are these two non-bullet dynamic bodies?
          if (collideA == false && collideB == false) {
            continue;
          }

          // Compute the TOI for this contact.
          // Put the sweeps onto the same time interval.
          double alpha0 = bA._sweep.alpha0;

          if (bA._sweep.alpha0 < bB._sweep.alpha0) {
            alpha0 = bB._sweep.alpha0;
            bA._sweep.advance(alpha0);
          } else if (bB._sweep.alpha0 < bA._sweep.alpha0) {
            alpha0 = bA._sweep.alpha0;
            bB._sweep.advance(alpha0);
          }

          assert(alpha0 < 1.0);

          final int indexA = c.getChildIndexA();
          final int indexB = c.getChildIndexB();

          // Compute the time of impact in interval [0, minTOI]
          final TOIInput input = toiInput;
          input.proxyA.set(fA.getShape(), indexA);
          input.proxyB.set(fB.getShape(), indexB);
          input.sweepA.set(bA._sweep);
          input.sweepB.set(bB._sweep);
          input.tMax = 1.0;

          toi.timeOfImpact(toiOutput, input);

          // Beta is the fraction of the remaining portion of the .
          final double beta = toiOutput.t;
          if (toiOutput.state == TOIOutputState.TOUCHING) {
            alpha = math.min(alpha0 + (1.0 - alpha0) * beta, 1.0);
          } else {
            alpha = 1.0;
          }

          c.toi = alpha;
          c.flags |= Contact.TOI_FLAG;
        }

        if (alpha < minAlpha) {
          // This is the minimum TOI found so far.
          minContact = c;
          minAlpha = alpha;
        }
      }

      if (minContact == null || 1.0 - 10.0 * settings.EPSILON < minAlpha) {
        // No more TOI events. Done!
        _stepComplete = true;
        break;
      }

      // Advance the bodies to the TOI.
      final Fixture fA = minContact.fixtureA;
      final Fixture fB = minContact.fixtureB;
      final Body bA = fA.getBody();
      final Body bB = fB.getBody();

      backup1.set(bA._sweep);
      backup2.set(bB._sweep);

      bA.advance(minAlpha);
      bB.advance(minAlpha);

      // The TOI contact likely has some new contact points.
      minContact.update(_contactManager.contactListener);
      minContact.flags &= ~Contact.TOI_FLAG;
      ++minContact.toiCount;

      // Is the contact solid?
      if (minContact.isEnabled() == false || minContact.isTouching() == false) {
        // Restore the sweeps.
        minContact.setEnabled(false);
        bA._sweep.set(backup1);
        bB._sweep.set(backup2);
        bA.synchronizeTransform();
        bB.synchronizeTransform();
        continue;
      }

      bA.setAwake(true);
      bB.setAwake(true);

      // Build the island
      island.clear();
      island.addBody(bA);
      island.addBody(bB);
      island.addContact(minContact);

      bA._flags |= Body.ISLAND_FLAG;
      bB._flags |= Body.ISLAND_FLAG;
      minContact.flags |= Contact.ISLAND_FLAG;

      // Get contacts on bodyA and bodyB.
      tempBodies[0] = bA;
      tempBodies[1] = bB;
      for (int i = 0; i < 2; ++i) {
        final Body body = tempBodies[i];
        if (body._bodyType == BodyType.DYNAMIC) {
          for (ContactEdge ce = body._contactList; ce != null; ce = ce.next) {
            final Contact contact = ce.contact;

            // Has this contact already been added to the island?
            if ((contact.flags & Contact.ISLAND_FLAG) != 0) {
              continue;
            }

            // Only add static, kinematic, or bullet bodies.
            final Body other = ce.other;
            if (other._bodyType == BodyType.DYNAMIC &&
                body.isBullet() == false &&
                other.isBullet() == false) {
              continue;
            }

            // Skip sensors.
            final bool sensorA = contact._fixtureA._isSensor;
            final bool sensorB = contact._fixtureB._isSensor;
            if (sensorA || sensorB) {
              continue;
            }

            // Tentatively advance the body to the TOI.
            backup1.set(other._sweep);
            if ((other._flags & Body.ISLAND_FLAG) == 0) {
              other.advance(minAlpha);
            }

            // Update the contact points
            contact.update(_contactManager.contactListener);

            // Was the contact disabled by the user?
            if (contact.isEnabled() == false) {
              other._sweep.set(backup1);
              other.synchronizeTransform();
              continue;
            }

            // Are there contact points?
            if (contact.isTouching() == false) {
              other._sweep.set(backup1);
              other.synchronizeTransform();
              continue;
            }

            // Add the contact to the island
            contact.flags |= Contact.ISLAND_FLAG;
            island.addContact(contact);

            // Has the other body already been added to the island?
            if ((other._flags & Body.ISLAND_FLAG) != 0) {
              continue;
            }

            // Add the other body to the island.
            other._flags |= Body.ISLAND_FLAG;

            if (other._bodyType != BodyType.STATIC) {
              other.setAwake(true);
            }

            island.addBody(other);
          }
        }
      }

      subStep.dt = (1.0 - minAlpha) * step.dt;
      subStep.invDt = 1.0 / subStep.dt;
      subStep.dtRatio = 1.0;
      subStep.positionIterations = 20;
      subStep.velocityIterations = step.velocityIterations;
      subStep.warmStarting = false;
      island.solveTOI(subStep, bA.islandIndex, bB.islandIndex);

      // Reset island flags and synchronize broad-phase proxies.
      for (BodyMeta bodyMeta in island._bodies) {
        final Body body = bodyMeta.body;
        body._flags &= ~Body.ISLAND_FLAG;

        if (body._bodyType != BodyType.DYNAMIC) {
          continue;
        }

        body.synchronizeFixtures();

        // Invalidate all contact TOIs on this displaced body.
        for (ContactEdge ce = body._contactList; ce != null; ce = ce.next) {
          ce.contact.flags &= ~(Contact.TOI_FLAG | Contact.ISLAND_FLAG);
        }
      }

      // Commit fixture proxy movements to the broad-phase so that new contacts are created.
      // Also, some contacts can be destroyed.
      _contactManager.findNewContacts();

      if (_subStepping) {
        _stepComplete = false;
        break;
      }
    }
  }

  void drawJoint(Joint joint) {
    final Body bodyA = joint.getBodyA();
    final Body bodyB = joint.getBodyB();
    final Transform xf1 = bodyA._transform;
    final Transform xf2 = bodyB._transform;
    final Vector2 x1 = xf1.p;
    final Vector2 x2 = xf2.p;
    final Vector2 p1 = Vector2.copy(joint.getAnchorA());
    final Vector2 p2 = Vector2.copy(joint.getAnchorB());

    color.setFromRGBd(0.5, 0.8, 0.8);

    switch (joint.getType()) {
      // TODO djm write after writing joints
      case JointType.DISTANCE:
        debugDraw.drawSegment(p1, p2, color);
        break;

      case JointType.PULLEY:
        {
          final pulley = joint as PulleyJoint;
          final Vector2 s1 = pulley.getGroundAnchorA();
          final Vector2 s2 = pulley.getGroundAnchorB();
          debugDraw.drawSegment(s1, p1, color);
          debugDraw.drawSegment(s2, p2, color);
          debugDraw.drawSegment(s1, s2, color);
        }
        break;

      case JointType.FRICTION:
        debugDraw.drawSegment(x1, x2, color);
        break;

      case JointType.CONSTANT_VOLUME:
      case JointType.MOUSE:
        // don't draw this
        break;
      default:
        debugDraw.drawSegment(x1, p1, color);
        debugDraw.drawSegment(p1, p2, color);
        debugDraw.drawSegment(x2, p2, color);
    }
  }

  // NOTE this corresponds to the liquid test, so the debugdraw can draw
  // the liquid particles correctly. They should be the same.
  static const int LIQUID_INT = 1234598372;
  double liquidLength = .12;
  double averageLinearVel = -1.0;
  final Vector2 liquidOffset = Vector2.zero();
  final Vector2 circCenterMoved = Vector2.zero();
  final Color3i liquidColor = Color3i.fromRGBd(.4, .4, 1.0);

  final Vector2 center = Vector2.zero();
  final Vector2 axis = Vector2.zero();
  final Vector2 v1 = Vector2.zero();
  final Vector2 v2 = Vector2.zero();

  void drawShape(Fixture fixture, Transform xf, Color3i color, bool wireframe) {
    switch (fixture.getType()) {
      case ShapeType.CIRCLE:
        {
          final circle = fixture.getShape() as CircleShape;

          center.setFrom(Transform.mulVec2(xf, circle.position));
          final double radius = circle.radius;
          xf.q.getXAxis(axis);

          if (fixture.userData != null && fixture.userData == LIQUID_INT) {
            final Body b = fixture.getBody();
            liquidOffset.setFrom(b._linearVelocity);
            final double linVelLength = b._linearVelocity.length;
            if (averageLinearVel == -1) {
              averageLinearVel = linVelLength;
            } else {
              averageLinearVel = .98 * averageLinearVel + .02 * linVelLength;
            }
            liquidOffset.scale(liquidLength / averageLinearVel / 2);
            circCenterMoved
              ..setFrom(center)
              ..add(liquidOffset);
            center.sub(liquidOffset);
            debugDraw.drawSegment(center, circCenterMoved, liquidColor);
            return;
          }
          if (wireframe) {
            debugDraw.drawCircleAxis(center, radius, axis, color);
          } else {
            debugDraw.drawSolidCircle(center, radius, axis, color);
          }
        }
        break;
      case ShapeType.POLYGON:
        {
          final poly = fixture.getShape() as PolygonShape;
          final int vertexCount = poly.count;
          assert(vertexCount <= settings.maxPolygonVertices);
          final List<Vector2> vertices =
              List<Vector2>(settings.maxPolygonVertices);

          for (int i = 0; i < vertexCount; ++i) {
            vertices[i] = Transform.mulVec2(xf, poly.vertices[i]);
          }
          if (wireframe) {
            debugDraw.drawPolygon(vertices, vertexCount, color);
          } else {
            debugDraw.drawSolidPolygon(vertices, vertexCount, color);
          }
        }
        break;
      case ShapeType.EDGE:
        {
          final edge = fixture.getShape() as EdgeShape;
          v1.setFrom(Transform.mulVec2(xf, edge.vertex1));
          v2.setFrom(Transform.mulVec2(xf, edge.vertex2));
          debugDraw.drawSegment(v1, v2, color);
        }
        break;
      case ShapeType.CHAIN:
        {
          final chain = fixture.getShape() as ChainShape;
          final int count = chain.vertexCount;
          final List<Vector2> vertices = chain._vertices;

          v1.setFrom(Transform.mulVec2(xf, vertices[0]));
          for (int i = 1; i < count; ++i) {
            v2.setFrom(Transform.mulVec2(xf, vertices[i]));
            debugDraw.drawSegment(v1, v2, color);
            debugDraw.drawCircle(v1, 0.05, color);
            v1.setFrom(v2);
          }
        }
        break;
      default:
        break;
    }
  }

  void drawParticleSystem(ParticleSystem system) {
    final bool wireframe =
        (debugDraw.drawFlags & DebugDraw.WIREFRAME_DRAWING_BIT) != 0;
    final int particleCount = system.getParticleCount();
    if (particleCount != 0) {
      final double particleRadius = system.getParticleRadius();
      final List<Vector2> positionBuffer = system.getParticlePositionBuffer();
      List<ParticleColor> colorBuffer;
      if (system.colorBuffer.data != null) {
        colorBuffer = system.getParticleColorBuffer();
      }
      if (wireframe) {
        debugDraw.drawParticlesWireframe(
            positionBuffer, particleRadius, colorBuffer, particleCount);
      } else {
        debugDraw.drawParticles(
            positionBuffer, particleRadius, colorBuffer, particleCount);
      }
    }
  }

  /// Create a particle whose properties have been defined. No reference to the definition is
  /// retained. A simulation step must occur before it's possible to interact with a newly created
  /// particle. For example, DestroyParticleInShape() will not destroy a particle until Step() has
  /// been called.
  ///
  /// @warning This function is locked during callbacks.
  /// @return the index of the particle.
  int createParticle(ParticleDef def) {
    assert(isLocked() == false);
    return _particleSystem.createParticle(def);
  }

  /// Destroy a particle. The particle is removed after the next step.
  ///
  /// @param index
  void destroyParticle(int index) {
    destroyParticleFlag(index, false);
  }

  /// Destroy a particle. The particle is removed after the next step.
  ///
  /// @param Index of the particle to destroy.
  /// @param Whether to call the destruction listener just before the particle is destroyed.
  void destroyParticleFlag(int index, bool callDestructionListener) {
    _particleSystem.destroyParticle(index, callDestructionListener);
  }

  /// Destroy particles inside a shape without enabling the destruction callback for destroyed
  /// particles. This function is locked during callbacks. For more information see
  /// DestroyParticleInShape(Shape&, Transform&,bool).
  ///
  /// @param Shape which encloses particles that should be destroyed.
  /// @param Transform applied to the shape.
  /// @warning This function is locked during callbacks.
  /// @return Number of particles destroyed.
  int destroyParticlesInShape(Shape shape, Transform xf) {
    return destroyParticlesInShapeFlag(shape, xf, false);
  }

  /// Destroy particles inside a shape. This function is locked during callbacks. In addition, this
  /// function immediately destroys particles in the shape in contrast to DestroyParticle() which
  /// defers the destruction until the next simulation step.
  ///
  /// @param Shape which encloses particles that should be destroyed.
  /// @param Transform applied to the shape.
  /// @param Whether to call the world b2DestructionListener for each particle destroyed.
  /// @warning This function is locked during callbacks.
  /// @return Number of particles destroyed.
  int destroyParticlesInShapeFlag(
      Shape shape, Transform xf, bool callDestructionListener) {
    assert(isLocked() == false);
    return _particleSystem.destroyParticlesInShape(
        shape, xf, callDestructionListener);
  }

  /// Create a particle group whose properties have been defined. No reference to the definition is
  /// retained.
  ///
  /// @warning This function is locked during callbacks.
  ParticleGroup createParticleGroup(ParticleGroupDef def) {
    assert(isLocked() == false);
    return _particleSystem.createParticleGroup(def);
  }

  /// Join two particle groups.
  ///
  /// @param the first group. Expands to encompass the second group.
  /// @param the second group. It is destroyed.
  /// @warning This function is locked during callbacks.
  void joinParticleGroups(ParticleGroup groupA, ParticleGroup groupB) {
    assert(isLocked() == false);
    _particleSystem.joinParticleGroups(groupA, groupB);
  }

  /// Destroy particles in a group. This function is locked during callbacks.
  ///
  /// @param The particle group to destroy.
  /// @param Whether to call the world b2DestructionListener for each particle is destroyed.
  /// @warning This function is locked during callbacks.
  void destroyParticlesInGroupFlag(
      ParticleGroup group, bool callDestructionListener) {
    assert(isLocked() == false);
    _particleSystem.destroyParticlesInGroup(group, callDestructionListener);
  }

  /// Destroy particles in a group without enabling the destruction callback for destroyed particles.
  /// This function is locked during callbacks.
  ///
  /// @param The particle group to destroy.
  /// @warning This function is locked during callbacks.
  void destroyParticlesInGroup(ParticleGroup group) {
    destroyParticlesInGroupFlag(group, false);
  }

  /// Get the world particle group list. With the returned group, use ParticleGroup::GetNext to get
  /// the next group in the world list. A NULL group indicates the end of the list.
  ///
  /// @return the head of the world particle group list.
  List<ParticleGroup> getParticleGroupList() {
    return _particleSystem.getParticleGroupList();
  }

  /// Get the number of particle groups.
  ///
  /// @return
  int getParticleGroupCount() {
    return _particleSystem.getParticleGroupCount();
  }

  /// Get the number of particles.
  ///
  /// @return
  int getParticleCount() {
    return _particleSystem.getParticleCount();
  }

  /// Get the maximum number of particles.
  ///
  /// @return
  int getParticleMaxCount() {
    return _particleSystem.getParticleMaxCount();
  }

  /// Set the maximum number of particles.
  ///
  /// @param count
  void setParticleMaxCount(int count) {
    _particleSystem.setParticleMaxCount(count);
  }

  /// Change the particle density.
  ///
  /// @param density
  void setParticleDensity(double density) {
    _particleSystem.setParticleDensity(density);
  }

  /// Get the particle density.
  ///
  /// @return
  double getParticleDensity() {
    return _particleSystem.getParticleDensity();
  }

  /// Change the particle gravity scale. Adjusts the effect of the global gravity vector on
  /// particles. Default value is 1.0.
  ///
  /// @param gravityScale
  void setParticleGravityScale(double gravityScale) {
    _particleSystem.setParticleGravityScale(gravityScale);
  }

  /// Get the particle gravity scale.
  ///
  /// @return
  double getParticleGravityScale() {
    return _particleSystem.getParticleGravityScale();
  }

  /// Damping is used to reduce the velocity of particles. The damping parameter can be larger than
  /// 1.0 but the damping effect becomes sensitive to the time step when the damping parameter is
  /// large.
  ///
  /// @param damping
  void setParticleDamping(double damping) {
    _particleSystem.setParticleDamping(damping);
  }

  /// Get damping for particles
  ///
  /// @return
  double getParticleDamping() {
    return _particleSystem.getParticleDamping();
  }

  /// Change the particle radius. You should set this only once, on world start. If you change the
  /// radius during execution, existing particles may explode, shrink, or behave unexpectedly.
  ///
  /// @param radius
  void setParticleRadius(double radius) {
    _particleSystem.setParticleRadius(radius);
  }

  /// Get the particle radius.
  ///
  /// @return
  double getParticleRadius() {
    return _particleSystem.getParticleRadius();
  }

  /// Get the particle data. @return the pointer to the head of the particle data.
  ///
  /// @return
  List<int> getParticleFlagsBuffer() {
    return _particleSystem.getParticleFlagsBuffer();
  }

  List<Vector2> getParticlePositionBuffer() {
    return _particleSystem.getParticlePositionBuffer();
  }

  List<Vector2> getParticleVelocityBuffer() {
    return _particleSystem.getParticleVelocityBuffer();
  }

  List<ParticleColor> getParticleColorBuffer() {
    return _particleSystem.getParticleColorBuffer();
  }

  List<ParticleGroup> getParticleGroupBuffer() {
    return _particleSystem.getParticleGroupBuffer();
  }

  List<Object> getParticleUserDataBuffer() {
    return _particleSystem.getParticleUserDataBuffer();
  }

  /// Set a buffer for particle data.
  ///
  /// @param buffer is a pointer to a block of memory.
  /// @param size is the number of values in the block.
  void setParticleFlagsBuffer(List<int> buffer, int capacity) {
    _particleSystem.setParticleFlagsBuffer(buffer, capacity);
  }

  void setParticlePositionBuffer(List<Vector2> buffer, int capacity) {
    _particleSystem.setParticlePositionBuffer(buffer, capacity);
  }

  void setParticleVelocityBuffer(List<Vector2> buffer, int capacity) {
    _particleSystem.setParticleVelocityBuffer(buffer, capacity);
  }

  void setParticleColorBuffer(List<ParticleColor> buffer, int capacity) {
    _particleSystem.setParticleColorBuffer(buffer, capacity);
  }

  void setParticleUserDataBuffer(List<Object> buffer, int capacity) {
    _particleSystem.setParticleUserDataBuffer(buffer, capacity);
  }

  /// Get contacts between particles
  ///
  /// @return
  List<ParticleContact> getParticleContacts() {
    return _particleSystem.contactBuffer;
  }

  int getParticleContactCount() {
    return _particleSystem.contactCount;
  }

  /// Get contacts between particles and bodies
  ///
  /// @return
  List<ParticleBodyContact> getParticleBodyContacts() {
    return _particleSystem.bodyContactBuffer;
  }

  int getParticleBodyContactCount() {
    return _particleSystem.bodyContactCount;
  }

  /// Compute the kinetic energy that can be lost by damping force
  ///
  /// @return
  double computeParticleCollisionEnergy() {
    return _particleSystem.computeParticleCollisionEnergy();
  }
}

class WorldQueryWrapper implements TreeCallback {
  @override
  bool treeCallback(int nodeId) {
    final proxy = broadPhase.getUserData(nodeId) as FixtureProxy;
    return callback.reportFixture(proxy.fixture);
  }

  BroadPhase broadPhase;
  QueryCallback callback;
}

class WorldRayCastWrapper implements TreeRayCastCallback {
  // djm pooling
  final RayCastOutput _output = RayCastOutput();
  final Vector2 _temp = Vector2.zero();
  final Vector2 _point = Vector2.zero();

  @override
  double raycastCallback(RayCastInput input, int nodeId) {
    final userData = broadPhase.getUserData(nodeId) as FixtureProxy;
    final FixtureProxy proxy = userData;
    final Fixture fixture = proxy.fixture;
    final int index = proxy.childIndex;
    final bool hit = fixture.raycast(_output, input, index);

    if (hit) {
      final double fraction = _output.fraction;
      // Vec2 point = (1.0 - fraction) * input.p1 + fraction * input.p2;
      _temp
        ..setFrom(input.p2)
        ..scale(fraction);
      _point
        ..setFrom(input.p1)
        ..scale(1 - fraction)
        ..add(_temp);
      return callback.reportFixture(fixture, _point, _output.normal, fraction);
    }

    return input.maxFraction;
  }

  BroadPhase broadPhase;
  RayCastCallback callback;
}
