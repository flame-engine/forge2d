import 'dart:math';

import '../../forge2d.dart';
import '../settings.dart' as settings;

/// The world class manages all physics entities, dynamic simulation, and
/// asynchronous queries. The world also contains efficient memory management
/// facilities.
class World {
  static const int newFixture = 0x0001;
  static const int locked = 0x0002;
  static const int clearForcesBit = 0x0004;

  // TODO(spydon): Don't have these fields as static
  static final Distance distance = Distance();
  static final Collision collision = Collision();
  static final TimeOfImpact toi = TimeOfImpact();

  int flags = 0;

  late ContactManager contactManager;
  final List<Body> bodies = <Body>[];
  final List<Joint> joints = <Joint>[];

  final Vector2 _gravity;
  Vector2 get gravity => _gravity;
  bool _allowSleep = false;

  DestroyListener? destroyListener;
  ParticleDestroyListener? particleDestroyListener;
  DebugDraw? debugDraw;

  /// This is used to compute the time step ratio to support a variable time step.
  double _invDt0 = 0.0;

  // these are for debugging the solver
  bool _warmStarting = false;
  bool _continuousPhysics = false;
  bool _subStepping = false;

  bool _stepComplete = false;

  late Profile _profile;

  late ParticleSystem particleSystem;

  /// Construct a world object.
  ///
  /// @param gravity the world gravity vector.
  /// @param broadPhase what type of broad phase strategy that should be used.
  World([Vector2? gravity, BroadPhase? broadPhase])
      : _gravity = Vector2.copy(gravity ?? Vector2.zero()) {
    broadPhase ??= DefaultBroadPhaseBuffer(DynamicTree());

    _warmStarting = true;
    _continuousPhysics = true;
    _subStepping = false;
    _stepComplete = true;

    _allowSleep = true;

    flags = clearForcesBit;

    _invDt0 = 0.0;

    contactManager = ContactManager(broadPhase);
    _profile = Profile();

    particleSystem = ParticleSystem(this);
  }

  void setAllowSleep(bool flag) {
    if (flag == _allowSleep) {
      return;
    }

    _allowSleep = flag;
    if (!_allowSleep) {
      for (final b in bodies) {
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

  /// Register a contact filter to provide specific control over collision. Otherwise the default
  /// filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
  ///
  /// @param filter
  void setContactFilter(ContactFilter filter) {
    contactManager.contactFilter = filter;
  }

  /// Register a contact event listener. The listener is owned by you and must remain in scope.
  ///
  /// @param listener
  void setContactListener(ContactListener listener) {
    contactManager.contactListener = listener;
  }

  /// create a rigid body given a definition. No reference to the definition is retained.
  ///
  /// @warning This function is locked during callbacks.
  /// @param def
  /// @return
  Body createBody(BodyDef def) {
    assert(!isLocked);
    final body = Body(def, this);
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
    assert(!isLocked);

    // Delete the attached joints.
    while (body.joints.isNotEmpty) {
      final joint = body.joints.first;
      destroyListener?.onDestroyJoint(joint);
      destroyJoint(joint);
    }

    // Delete the attached contacts.
    while (body.contacts.isNotEmpty) {
      contactManager.destroy(body.contacts.first);
    }
    body.contacts.clear();

    for (final f in body.fixtures) {
      destroyListener?.onDestroyFixture(f);
      f.destroyProxies(contactManager.broadPhase);
    }
    bodies.remove(body);
  }

  /// Adds a joint to constrain bodies together.
  ///
  /// This may cause the connected bodies to cease colliding.
  ///
  /// Adding a joint doesn't wake up the bodies.
  ///
  /// Warning: This function is locked during callbacks.
  void createJoint(Joint joint) {
    assert(!isLocked);
    joints.add(joint);

    final bodyA = joint.bodyA;
    final bodyB = joint.bodyB;
    bodyA.joints.add(joint);
    bodyB.joints.add(joint);

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (!joint.collideConnected) {
      for (final contact in bodyB.contacts) {
        if (contact.getOtherBody(bodyB) == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          contact.flagForFiltering();
        }
      }
    }
  }

  /// destroy a joint. This may cause the connected bodies to begin colliding.
  ///
  /// @warning This function is locked during callbacks.
  /// @param joint
  void destroyJoint(Joint joint) {
    assert(!isLocked);

    final collideConnected = joint.collideConnected;
    joints.remove(joint);

    // Disconnect from island graph.
    final bodyA = joint.bodyA;
    final bodyB = joint.bodyB;

    // Wake up connected bodies.
    bodyA.setAwake(true);
    bodyB.setAwake(true);

    bodyA.joints.remove(joint);
    bodyB.joints.remove(joint);

    Joint.destroy(joint);

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (!collideConnected) {
      for (final contact in bodyB.contacts) {
        if (contact.getOtherBody(bodyB) == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          contact.flagForFiltering();
        }
      }
    }
  }

  final TimeStep _step = TimeStep();
  final Timer _stepTimer = Timer();
  final Timer _tempTimer = Timer();

  /// Take a time step. This performs collision detection, integration, and constraint solution.
  ///
  /// @param dt the amount of time that has passed since the last step
  void stepDt(double dt) {
    _stepTimer.reset();
    _tempTimer.reset();
    // If new fixtures were added, we need to find the new contacts.
    if ((flags & newFixture) == newFixture) {
      contactManager.findNewContacts();
      flags &= ~newFixture;
    }

    flags |= locked;

    _step.dt = dt;
    _step.velocityIterations = settings.velocityIterations;
    _step.positionIterations = settings.positionIterations;
    if (dt > 0.0) {
      _step.invDt = 1.0 / dt;
    } else {
      _step.invDt = 0.0;
    }

    _step.dtRatio = _invDt0 * dt;

    _step.warmStarting = _warmStarting;
    _profile.stepInit.record(_tempTimer.getMilliseconds());

    // Update contacts. This is where some contacts are destroyed.
    _tempTimer.reset();
    contactManager.collide();
    _profile.collide.record(_tempTimer.getMilliseconds());

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (_stepComplete && _step.dt > 0.0) {
      _tempTimer.reset();
      particleSystem.solve(_step); // Particle Simulation
      _profile.solveParticleSystem.record(_tempTimer.getMilliseconds());
      _tempTimer.reset();
      solve(_step);
      _profile.solve.record(_tempTimer.getMilliseconds());
    }

    // Handle TOI events.
    if (_continuousPhysics && _step.dt > 0.0) {
      _tempTimer.reset();
      solveTOI(_step);
      _profile.solveTOI.record(_tempTimer.getMilliseconds());
    }

    if (_step.dt > 0.0) {
      _invDt0 = _step.invDt;
    }

    if ((flags & clearForcesBit) == clearForcesBit) {
      clearForces();
    }

    flags &= ~locked;

    _profile.step.record(_stepTimer.getMilliseconds());
  }

  /// Call this after you are done with time steps to clear the forces. You normally call this after
  /// each call to Step, unless you are performing sub-steps. By default, forces will be
  /// automatically cleared, so you don't need to call this function.
  ///
  /// @see setAutoClearForces
  void clearForces() {
    bodies.forEach((b) => b.clearForces());
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

    final flags = debugDraw!.drawFlags;
    final wireframe = (flags & DebugDraw.wireFrameDrawingBit) != 0;

    if ((flags & DebugDraw.shapeBit) != 0) {
      for (final body in bodies) {
        xf.set(body.transform);
        for (final fixture in body.fixtures) {
          if (!body.isActive) {
            color.setFromRGBd(0.5, 0.5, 0.3);
            fixture.render(debugDraw!, xf, color, wireframe);
          } else if (body.bodyType == BodyType.static) {
            color.setFromRGBd(0.5, 0.9, 0.3);
            fixture.render(debugDraw!, xf, color, wireframe);
          } else if (body.bodyType == BodyType.kinematic) {
            color.setFromRGBd(0.5, 0.5, 0.9);
            fixture.render(debugDraw!, xf, color, wireframe);
          } else if (!body.isAwake) {
            color.setFromRGBd(0.5, 0.5, 0.5);
            fixture.render(debugDraw!, xf, color, wireframe);
          } else {
            color.setFromRGBd(0.9, 0.7, 0.7);
            fixture.render(debugDraw!, xf, color, wireframe);
          }
        }
      }
      particleSystem.render(debugDraw!);
    }

    if ((flags & DebugDraw.jointBit) != 0) {
      joints.forEach((j) => j.render(debugDraw!));
    }

    if ((flags & DebugDraw.pairBit) != 0) {
      color.setFromRGBd(0.3, 0.9, 0.9);
      for (final c in contactManager.contacts) {
        final fixtureA = c.fixtureA;
        final fixtureB = c.fixtureB;
        cA.setFrom(fixtureA.getAABB(c.indexA).center);
        cB.setFrom(fixtureB.getAABB(c.indexB).center);
        debugDraw!.drawSegment(cA, cB, color);
      }
    }

    if ((flags & DebugDraw.aabbBit) != 0) {
      color.setFromRGBd(0.9, 0.3, 0.9);

      for (final b in bodies) {
        if (b.isActive == false) {
          continue;
        }

        for (final f in b.fixtures) {
          for (var i = 0; i < f.proxyCount; ++i) {
            final proxy = f.proxies[i];
            final aabb = contactManager.broadPhase.fatAABB(proxy.proxyId);
            final vs = <Vector2>[
              Vector2(aabb.lowerBound.x, aabb.lowerBound.y),
              Vector2(aabb.upperBound.x, aabb.lowerBound.y),
              Vector2(aabb.upperBound.x, aabb.upperBound.y),
              Vector2(aabb.lowerBound.x, aabb.upperBound.y),
            ];
            debugDraw!.drawPolygon(vs, color);
          }
        }
      }
    }

    if ((flags & DebugDraw.centerOfMassBit) != 0) {
      final xfColor = Color3i(255, 0, 0);
      for (final b in bodies) {
        xf.set(b.transform);
        xf.p.setFrom(b.worldCenter);
        debugDraw!.drawTransform(xf, xfColor);
      }
    }

    if ((flags & DebugDraw.dynamicTreeBit) != 0) {
      contactManager.broadPhase.drawTree(debugDraw!);
    }

    debugDraw!.flush();
  }

  final WorldQueryWrapper _worldQueryWrapper = WorldQueryWrapper();

  /// Query the world for all fixtures that potentially overlap the provided AABB.
  ///
  /// @param callback a user implemented callback class.
  /// @param aabb the query box.
  void queryAABB(QueryCallback callback, AABB aabb) {
    _worldQueryWrapper.broadPhase = contactManager.broadPhase;
    _worldQueryWrapper.callback = callback;
    contactManager.broadPhase.query(_worldQueryWrapper, aabb);
  }

  /// Query the world for all fixtures and particles that potentially overlap the provided AABB.
  ///
  /// @param callback a user implemented callback class.
  /// @param particleCallback callback for particles.
  /// @param aabb the query box.
  void queryAABBTwoCallbacks(
    QueryCallback callback,
    ParticleQueryCallback particleCallback,
    AABB aabb,
  ) {
    _worldQueryWrapper.broadPhase = contactManager.broadPhase;
    _worldQueryWrapper.callback = callback;
    contactManager.broadPhase.query(_worldQueryWrapper, aabb);
    particleSystem.queryAABB(particleCallback, aabb);
  }

  /// Query the world for all particles that potentially overlap the provided AABB.
  ///
  /// @param particleCallback callback for particles.
  /// @param aabb the query box.
  void queryAABBParticle(ParticleQueryCallback particleCallback, AABB aabb) {
    particleSystem.queryAABB(particleCallback, aabb);
  }

  final WorldRayCastWrapper _raycastWrapper = WorldRayCastWrapper();
  final RayCastInput _input = RayCastInput();

  /// Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
  /// get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
  /// starting point.
  ///
  /// @param callback a user implemented callback class.
  /// @param point1 the ray starting point
  /// @param point2 the ray ending point
  void raycast(RayCastCallback callback, Vector2 point1, Vector2 point2) {
    _raycastWrapper.broadPhase = contactManager.broadPhase;
    _raycastWrapper.callback = callback;
    _input.maxFraction = 1.0;
    _input.p1.setFrom(point1);
    _input.p2.setFrom(point2);
    contactManager.broadPhase.raycast(_raycastWrapper, _input);
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
    Vector2 point2,
  ) {
    _raycastWrapper.broadPhase = contactManager.broadPhase;
    _raycastWrapper.callback = callback;
    _input.maxFraction = 1.0;
    _input.p1.setFrom(point1);
    _input.p2.setFrom(point2);
    contactManager.broadPhase.raycast(_raycastWrapper, _input);
    particleSystem.raycast(particleCallback, point1, point2);
  }

  /// Ray-cast the world for all particles in the path of the ray. Your callback controls whether you
  /// get the closest point, any point, or n-points.
  ///
  /// @param particleCallback the particle callback class.
  /// @param point1 the ray starting point
  /// @param point2 the ray ending point
  void raycastParticle(
    ParticleRaycastCallback particleCallback,
    Vector2 point1,
    Vector2 point2,
  ) {
    particleSystem.raycast(particleCallback, point1, point2);
  }

  /// Get the number of broad-phase proxies.
  int get proxyCount => contactManager.broadPhase.proxyCount;

  /// Gets the height of the dynamic tree
  int getTreeHeight() => contactManager.broadPhase.getTreeHeight();

  /// Gets the balance of the dynamic tree
  int getTreeBalance() => contactManager.broadPhase.getTreeBalance();

  /// Gets the quality of the dynamic tree
  double getTreeQuality() => contactManager.broadPhase.getTreeQuality();

  /// Change the global gravity vector.
  void setGravity(Vector2 gravity) {
    _gravity.setFrom(gravity);
  }

  /// Is the world locked (in the middle of a time step).
  bool get isLocked => (flags & locked) == locked;

  /// Set flag to control automatic clearing of forces after each time step.
  void setAutoClearForces(bool shouldAutoClear) {
    if (shouldAutoClear) {
      flags |= clearForcesBit;
    } else {
      flags &= ~clearForcesBit;
    }
  }

  /// Get the flag that controls automatic clearing of forces after each time step.
  bool get autoClearForces => (flags & clearForcesBit) == clearForcesBit;

  final Island island = Island();
  final List<Body> stack = [];
  final Timer broadphaseTimer = Timer();

  void solve(TimeStep step) {
    _profile.solveInit.startAccum();
    _profile.solveVelocity.startAccum();
    _profile.solvePosition.startAccum();

    // update previous transforms
    for (final b in bodies) {
      b.previousTransform.set(b.transform);
    }

    // Size the island for the worst case.
    island.init(contactManager.contactListener);

    // Clear all the island flags.
    for (final b in bodies) {
      b.flags &= ~Body.islandFlag;
    }
    for (final c in contactManager.contacts) {
      c.flags &= ~Contact.islandFlag;
    }
    for (final j in joints) {
      j.islandFlag = false;
    }

    for (final seed in bodies) {
      if ((seed.flags & Body.islandFlag) == Body.islandFlag) {
        continue;
      }

      if (seed.isAwake == false || seed.isActive == false) {
        continue;
      }

      // The seed can be dynamic or kinematic.
      if (seed.bodyType == BodyType.static) {
        continue;
      }

      // Reset island and stack.
      island.clear();
      stack.clear();
      stack.add(seed);
      seed.flags |= Body.islandFlag;

      // Perform a depth first search (DFS) on the constraint graph.
      while (stack.isNotEmpty) {
        // Grab the next body off the stack and add it to the island.
        final body = stack.removeLast();
        assert(body.isActive);
        island.addBody(body);

        // Make sure the body is awake.
        body.setAwake(true);

        // To keep islands as small as possible, we don't
        // propagate islands across static bodies.
        if (body.bodyType == BodyType.static) {
          continue;
        }

        // Search all contacts connected to this body.
        for (final contact in body.contacts) {
          // Has this contact already been added to an island?
          if ((contact.flags & Contact.islandFlag) == Contact.islandFlag) {
            continue;
          }

          // Is this contact solid and touching?
          if (contact.isEnabled() == false || contact.isTouching() == false) {
            continue;
          }

          // Skip sensors.
          final sensorA = contact.fixtureA.isSensor;
          final sensorB = contact.fixtureB.isSensor;
          if (sensorA || sensorB) {
            continue;
          }

          island.addContact(contact);
          contact.flags |= Contact.islandFlag;

          final other = contact.getOtherBody(body);

          // Was the other body already added to this island?
          if ((other.flags & Body.islandFlag) == Body.islandFlag) {
            continue;
          }

          stack.add(other);
          other.flags |= Body.islandFlag;
        }

        // Search all joints connect to this body.
        for (final joint in body.joints) {
          if (joint.islandFlag) {
            continue;
          }

          final other = joint.otherBody(body);

          // Don't simulate joints connected to inactive bodies.
          if (!other.isActive) {
            continue;
          }

          island.addJoint(joint);
          joint.islandFlag = true;

          if ((other.flags & Body.islandFlag) == Body.islandFlag) {
            continue;
          }

          stack.add(other);
          other.flags |= Body.islandFlag;
        }
      }
      island.solve(_profile, step, _gravity, _allowSleep);

      // Post solve cleanup.
      for (final bodyMeta in island.bodies) {
        // Allow static bodies to participate in other islands.
        final b = bodyMeta.body;
        if (b.bodyType == BodyType.static) {
          b.flags &= ~Body.islandFlag;
        }
      }
    }
    _profile.solveInit.endAccum();
    _profile.solveVelocity.endAccum();
    _profile.solvePosition.endAccum();

    broadphaseTimer.reset();
    // Synchronize fixtures, check for out of range bodies.
    for (final b in bodies) {
      // If a body was not in an island then it did not move.
      if ((b.flags & Body.islandFlag) == 0) {
        continue;
      }

      if (b.bodyType == BodyType.static) {
        continue;
      }

      // Update fixtures (for broad-phase).
      b.synchronizeFixtures();
    }

    // Look for new contacts.
    contactManager.findNewContacts();
    _profile.broadphase.record(broadphaseTimer.getMilliseconds());
  }

  final Island _toiIsland = Island();
  final TOIInput _toiInput = TOIInput();
  final TOIOutput _toiOutput = TOIOutput();
  final TimeStep _subStep = TimeStep();
  final Sweep _backup1 = Sweep();
  final Sweep _backup2 = Sweep();

  void solveTOI(final TimeStep step) {
    final island = _toiIsland..init(contactManager.contactListener);
    if (_stepComplete) {
      for (final b in bodies) {
        b.flags &= ~Body.islandFlag;
        b.sweep.alpha0 = 0.0;
      }

      for (final c in contactManager.contacts) {
        // Invalidate TOI
        c.flags &= ~(Contact.toiFlag | Contact.islandFlag);
        c.toiCount = 0;
        c.toi = 1.0;
      }
    }

    // Find TOI events and solve them.
    for (;;) {
      // Find the first TOI.
      Contact? minContact;
      var minAlpha = 1.0;

      for (final contact in contactManager.contacts) {
        // Is this contact disabled?
        if (contact.isEnabled() == false) {
          continue;
        }

        // Prevent excessive sub-stepping.
        if (contact.toiCount > settings.maxSubSteps) {
          continue;
        }

        var alpha = 1.0;
        if ((contact.flags & Contact.toiFlag) != 0) {
          // This contact has a valid cached TOI.
          alpha = contact.toi;
        } else {
          final fixtureA = contact.fixtureA;
          final fixtureB = contact.fixtureB;

          // Is there a sensor?
          if (fixtureA.isSensor || fixtureB.isSensor) {
            continue;
          }

          final bodyA = fixtureA.body;
          final bodyB = fixtureB.body;

          final typeA = bodyA.bodyType;
          final typeB = bodyB.bodyType;
          assert(typeA == BodyType.dynamic || typeB == BodyType.dynamic);

          final activeA = bodyA.isAwake && typeA != BodyType.static;
          final activeB = bodyB.isAwake && typeB != BodyType.static;

          // Is at least one body active (awake and dynamic or kinematic)?
          if (activeA == false && activeB == false) {
            continue;
          }

          final collideA = bodyA.isBullet() || typeA != BodyType.dynamic;
          final collideB = bodyB.isBullet() || typeB != BodyType.dynamic;

          // Are these two non-bullet dynamic bodies?
          if (collideA == false && collideB == false) {
            continue;
          }

          // Compute the TOI for this contact.
          // Put the sweeps onto the same time interval.
          var alpha0 = bodyA.sweep.alpha0;

          if (bodyA.sweep.alpha0 < bodyB.sweep.alpha0) {
            alpha0 = bodyB.sweep.alpha0;
            bodyA.sweep.advance(alpha0);
            // NOTE: The following line is ignored due to a false positive
            // analyzer warning.
            // https://github.com/dart-lang/linter/issues/811
            // ignore: invariant_booleans
          } else if (bodyB.sweep.alpha0 < bodyA.sweep.alpha0) {
            alpha0 = bodyA.sweep.alpha0;
            bodyB.sweep.advance(alpha0);
          }

          assert(alpha0 < 1.0);

          final indexA = contact.indexA;
          final indexB = contact.indexB;

          // Compute the time of impact in interval [0, minTOI]
          final input = _toiInput;
          input.proxyA.set(fixtureA.shape, indexA);
          input.proxyB.set(fixtureB.shape, indexB);
          input.sweepA.set(bodyA.sweep);
          input.sweepB.set(bodyB.sweep);
          input.tMax = 1.0;

          toi.timeOfImpact(_toiOutput, input);

          // Beta is the fraction of the remaining portion of the .
          final beta = _toiOutput.t;
          if (_toiOutput.state == TOIOutputState.touching) {
            alpha = min(alpha0 + (1.0 - alpha0) * beta, 1.0);
          } else {
            alpha = 1.0;
          }

          contact.toi = alpha;
          contact.flags |= Contact.toiFlag;
        }

        if (alpha < minAlpha) {
          // This is the minimum TOI found so far.
          minContact = contact;
          minAlpha = alpha;
        }
      }

      if (minContact == null || 1.0 - 10.0 * settings.epsilon < minAlpha) {
        // No more TOI events. Done!
        _stepComplete = true;
        break;
      }

      final bodyA = minContact.fixtureA.body;
      final bodyB = minContact.fixtureB.body;

      _backup1.set(bodyA.sweep);
      _backup2.set(bodyB.sweep);

      // Advance the bodies to the TOI.
      bodyA.advance(minAlpha);
      bodyB.advance(minAlpha);

      // The TOI contact likely has some new contact points.
      minContact.update(contactManager.contactListener);
      minContact.flags &= ~Contact.toiFlag;
      ++minContact.toiCount;

      // Is the contact solid?
      if (minContact.isEnabled() == false || minContact.isTouching() == false) {
        // Restore the sweeps.
        minContact.setEnabled(false);
        bodyA.sweep.set(_backup1);
        bodyB.sweep.set(_backup2);
        bodyA.synchronizeTransform();
        bodyB.synchronizeTransform();
        continue;
      }

      bodyA.setAwake(true);
      bodyB.setAwake(true);

      // Build the island
      island.clear();
      island.addBody(bodyA);
      island.addBody(bodyB);
      island.addContact(minContact);

      bodyA.flags |= Body.islandFlag;
      bodyB.flags |= Body.islandFlag;
      minContact.flags |= Contact.islandFlag;

      // Get contacts on bodyA and bodyB.
      for (final body in [bodyA, bodyB]) {
        if (body.bodyType == BodyType.dynamic) {
          for (final contact in body.contacts) {
            // Has this contact already been added to the island?
            if ((contact.flags & Contact.islandFlag) != 0) {
              continue;
            }

            // Only add static, kinematic, or bullet bodies.
            final other = contact.getOtherBody(body);
            if (other.bodyType == BodyType.dynamic &&
                body.isBullet() == false &&
                other.isBullet() == false) {
              continue;
            }

            // Skip sensors.
            final sensorA = contact.fixtureA.isSensor;
            final sensorB = contact.fixtureB.isSensor;
            if (sensorA || sensorB) {
              continue;
            }

            // Tentatively advance the body to the TOI.
            _backup1.set(other.sweep);
            if ((other.flags & Body.islandFlag) == 0) {
              other.advance(minAlpha);
            }

            // Update the contact points
            contact.update(contactManager.contactListener);

            // Was the contact disabled by the user?
            if (contact.isEnabled() == false) {
              other.sweep.set(_backup1);
              other.synchronizeTransform();
              continue;
            }

            // Are there contact points?
            if (contact.isTouching() == false) {
              other.sweep.set(_backup1);
              other.synchronizeTransform();
              continue;
            }

            // Add the contact to the island
            contact.flags |= Contact.islandFlag;
            island.addContact(contact);

            // Has the other body already been added to the island?
            if ((other.flags & Body.islandFlag) != 0) {
              continue;
            }

            // Add the other body to the island.
            other.flags |= Body.islandFlag;

            if (other.bodyType != BodyType.static) {
              other.setAwake(true);
            }

            island.addBody(other);
          }
        }
      }

      _subStep.dt = (1.0 - minAlpha) * step.dt;
      _subStep.invDt = 1.0 / _subStep.dt;
      _subStep.dtRatio = 1.0;
      _subStep.positionIterations = 20;
      _subStep.velocityIterations = step.velocityIterations;
      _subStep.warmStarting = false;
      island.solveTOI(_subStep, bodyA.islandIndex, bodyB.islandIndex);

      // Reset island flags and synchronize broad-phase proxies.
      for (final bodyMeta in island.bodies) {
        final body = bodyMeta.body;
        body.flags &= ~Body.islandFlag;

        if (body.bodyType != BodyType.dynamic) {
          continue;
        }

        body.synchronizeFixtures();

        // Invalidate all contact TOIs on this displaced body.
        for (final contact in body.contacts) {
          contact.flags &= ~(Contact.toiFlag | Contact.islandFlag);
        }
      }

      // Commit fixture proxy movements to the broad-phase so that new contacts are created.
      // Also, some contacts can be destroyed.
      contactManager.findNewContacts();

      if (_subStepping) {
        _stepComplete = false;
        break;
      }
    }
  }
}

class WorldQueryWrapper implements TreeCallback {
  @override
  bool treeCallback(int nodeId) {
    final userData = broadPhase.getUserData(nodeId);
    if (userData == null) {
      return false;
    }
    final proxy = userData as FixtureProxy;
    return callback?.reportFixture(proxy.fixture) ?? false;
  }

  late BroadPhase broadPhase;
  QueryCallback? callback;
}

class WorldRayCastWrapper implements TreeRayCastCallback {
  // djm pooling
  final RayCastOutput _output = RayCastOutput();
  final Vector2 _temp = Vector2.zero();
  final Vector2 _point = Vector2.zero();

  @override
  double raycastCallback(RayCastInput input, int nodeId) {
    final userData = broadPhase.getUserData(nodeId);
    if (userData == null) {
      return 0;
    }
    final proxy = userData as FixtureProxy;
    final fixture = proxy.fixture;
    final index = proxy.childIndex;
    final hit = fixture.raycast(_output, input, index);

    if (hit) {
      final fraction = _output.fraction;
      // Vec2 point = (1.0 - fraction) * input.p1 + fraction * input.p2;
      _temp
        ..setFrom(input.p2)
        ..scale(fraction);
      _point
        ..setFrom(input.p1)
        ..scale(1 - fraction)
        ..add(_temp);
      return callback?.reportFixture(
            fixture,
            _point,
            _output.normal,
            fraction,
          ) ??
          0;
    }

    return input.maxFraction;
  }

  late BroadPhase broadPhase;
  RayCastCallback? callback;
}
