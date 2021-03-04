import 'dart:math';

import '../../forge2d.dart';
import '../callbacks/contact_filter.dart';
import '../callbacks/contact_listener.dart';
import '../callbacks/debug_draw.dart';
import '../callbacks/destruction_listener.dart';
import '../callbacks/particle_destruction_listener.dart';
import '../callbacks/particle_query_callback.dart';
import '../callbacks/particle_raycast_callback.dart';
import '../callbacks/query_callback.dart';
import '../callbacks/raycast_callback.dart';
import '../callbacks/tree_callback.dart';
import '../callbacks/tree_raycast_callback.dart';
import '../settings.dart' as settings;

/// The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
/// world also contains efficient memory management facilities.
class World {
  static const int newFixture = 0x0001;
  static const int locked = 0x0002;
  static const int clearForcesBit = 0x0004;

  // TODO.spydon: Don't have these fields as static
  static final Distance distance = Distance();
  static final Collision collision = Collision();
  static final TimeOfImpact toi = TimeOfImpact();

  int flags = 0;

  ContactManager contactManager;
  final List<Body> bodies = <Body>[];
  final List<Joint> joints = <Joint>[];

  final Vector2 _gravity;
  bool _allowSleep = false;

  DestructionListener destructionListener;
  ParticleDestructionListener particleDestructionListener;
  DebugDraw debugDraw;

  /// This is used to compute the time step ratio to support a variable time step.
  double _invDt0 = 0.0;

  // these are for debugging the solver
  bool _warmStarting = false;
  bool _continuousPhysics = false;
  bool _subStepping = false;

  bool _stepComplete = false;

  Profile _profile;

  ParticleSystem particleSystem;

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
    if (_allowSleep == false) {
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
    assert(isLocked() == false);
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
    assert(isLocked() == false);

    // Delete the attached joints.
    for (final joint in body.joints) {
      destructionListener?.sayGoodbyeJoint(joint);
      destroyJoint(joint);
    }

    // Delete the attached contacts.
    while (body.contacts.isNotEmpty) {
      contactManager.destroy(body.contacts.first);
    }
    body.contacts.clear();

    for (final f in body.fixtures) {
      destructionListener?.sayGoodbyeFixture(f);
      f.destroyProxies(contactManager.broadPhase);
    }
    bodies.remove(body);
  }

  /// Create a joint to constrain bodies together. No reference to the definition is retained.
  /// This may cause the connected bodies to cease colliding.
  ///
  /// @warning This function is locked during callbacks.
  Joint createJoint(JointDef def) {
    assert(isLocked() == false);

    final joint = Joint.create(this, def);
    joints.add(joint);

    joint.bodyA.joints.add(joint);
    joint.bodyB.joints.add(joint);

    final bodyA = def.bodyA;
    final bodyB = def.bodyB;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (def.collideConnected == false) {
      for (final contact in bodyB.contacts) {
        if (contact.getOtherBody(bodyB) == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          contact.flagForFiltering();
        }
      }
    }

    // Note: creating a joint doesn't wake the bodies.

    return joint;
  }

  /// destroy a joint. This may cause the connected bodies to begin colliding.
  ///
  /// @warning This function is locked during callbacks.
  /// @param joint
  void destroyJoint(Joint joint) {
    assert(isLocked() == false);

    final collideConnected = joint.getCollideConnected();
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
    if (collideConnected == false) {
      for (final contact in bodyB.contacts) {
        if (contact.getOtherBody(bodyB) == bodyA) {
          // Flag the contact for filtering at the next time step (where either
          // body is awake).
          contact.flagForFiltering();
        }
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
    if ((flags & newFixture) == newFixture) {
      contactManager.findNewContacts();
      flags &= ~newFixture;
    }

    flags |= locked;

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
    contactManager.collide();
    _profile.collide.record(tempTimer.getMilliseconds());

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (_stepComplete && step.dt > 0.0) {
      tempTimer.reset();
      particleSystem.solve(step); // Particle Simulation
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

    if ((flags & clearForcesBit) == clearForcesBit) {
      clearForces();
    }

    flags &= ~locked;

    _profile.step.record(stepTimer.getMilliseconds());
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

    final flags = debugDraw.drawFlags;
    final wireframe = (flags & DebugDraw.wireFrameDrawingBit) != 0;

    if ((flags & DebugDraw.shapeBit) != 0) {
      for (final b in bodies) {
        xf.set(b.transform);
        for (final f in b.fixtures) {
          if (b.isActive() == false) {
            color.setFromRGBd(0.5, 0.5, 0.3);
            drawShape(f, xf, color, wireframe);
          } else if (b.bodyType == BodyType.static) {
            color.setFromRGBd(0.5, 0.9, 0.3);
            drawShape(f, xf, color, wireframe);
          } else if (b.bodyType == BodyType.kinematic) {
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
      drawParticleSystem(particleSystem);
    }

    if ((flags & DebugDraw.jointBit) != 0) {
      joints.forEach(drawJoint);
    }

    if ((flags & DebugDraw.pairBit) != 0) {
      color.setFromRGBd(0.3, 0.9, 0.9);
      for (final c in contactManager.contacts) {
        final fixtureA = c.fixtureA;
        final fixtureB = c.fixtureB;
        cA.setFrom(fixtureA.getAABB(c.indexA).getCenter());
        cB.setFrom(fixtureB.getAABB(c.indexB).getCenter());
        debugDraw.drawSegment(cA, cB, color);
      }
    }

    if ((flags & DebugDraw.aabbBit) != 0) {
      color.setFromRGBd(0.9, 0.3, 0.9);

      for (final b in bodies) {
        if (b.isActive() == false) {
          continue;
        }

        for (final f in b.fixtures) {
          for (var i = 0; i < f.proxyCount; ++i) {
            final proxy = f.proxies[i];
            final aabb = contactManager.broadPhase.getFatAABB(proxy.proxyId);
            if (aabb != null) {
              final vs = <Vector2>[
                Vector2(aabb.lowerBound.x, aabb.lowerBound.y),
                Vector2(aabb.upperBound.x, aabb.lowerBound.y),
                Vector2(aabb.upperBound.x, aabb.upperBound.y),
                Vector2(aabb.lowerBound.x, aabb.upperBound.y),
              ];
              debugDraw.drawPolygon(vs, color);
            }
          }
        }
      }
    }

    if ((flags & DebugDraw.centerOfMassBit) != 0) {
      final xfColor = Color3i(255, 0, 0);
      for (final b in bodies) {
        xf.set(b.transform);
        xf.p.setFrom(b.worldCenter);
        debugDraw.drawTransform(xf, xfColor);
      }
    }

    if ((flags & DebugDraw.dynamicTreeBit) != 0) {
      contactManager.broadPhase.drawTree(debugDraw);
    }

    debugDraw.flush();
  }

  final WorldQueryWrapper wqwrapper = WorldQueryWrapper();

  /// Query the world for all fixtures that potentially overlap the provided AABB.
  ///
  /// @param callback a user implemented callback class.
  /// @param aabb the query box.
  void queryAABB(QueryCallback callback, AABB aabb) {
    wqwrapper.broadPhase = contactManager.broadPhase;
    wqwrapper.callback = callback;
    contactManager.broadPhase.query(wqwrapper, aabb);
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
    wqwrapper.broadPhase = contactManager.broadPhase;
    wqwrapper.callback = callback;
    contactManager.broadPhase.query(wqwrapper, aabb);
    particleSystem.queryAABB(particleCallback, aabb);
  }

  /// Query the world for all particles that potentially overlap the provided AABB.
  ///
  /// @param particleCallback callback for particles.
  /// @param aabb the query box.
  void queryAABBParticle(ParticleQueryCallback particleCallback, AABB aabb) {
    particleSystem.queryAABB(particleCallback, aabb);
  }

  final WorldRayCastWrapper raycastWrapper = WorldRayCastWrapper();
  final RayCastInput input = RayCastInput();

  /// Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
  /// get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
  /// starting point.
  ///
  /// @param callback a user implemented callback class.
  /// @param point1 the ray starting point
  /// @param point2 the ray ending point
  void raycast(RayCastCallback callback, Vector2 point1, Vector2 point2) {
    raycastWrapper.broadPhase = contactManager.broadPhase;
    raycastWrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setFrom(point1);
    input.p2.setFrom(point2);
    contactManager.broadPhase.raycast(raycastWrapper, input);
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
    raycastWrapper.broadPhase = contactManager.broadPhase;
    raycastWrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setFrom(point1);
    input.p2.setFrom(point2);
    contactManager.broadPhase.raycast(raycastWrapper, input);
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
  int getProxyCount() {
    return contactManager.broadPhase.getProxyCount();
  }

  /// Gets the height of the dynamic tree
  int getTreeHeight() {
    return contactManager.broadPhase.getTreeHeight();
  }

  /// Gets the balance of the dynamic tree
  int getTreeBalance() {
    return contactManager.broadPhase.getTreeBalance();
  }

  /// Gets the quality of the dynamic tree
  double getTreeQuality() {
    return contactManager.broadPhase.getTreeQuality();
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
    return (flags & locked) == locked;
  }

  /// Set flag to control automatic clearing of forces after each time step.
  void setAutoClearForces(bool shouldAutoClear) {
    if (shouldAutoClear) {
      flags |= clearForcesBit;
    } else {
      flags &= ~clearForcesBit;
    }
  }

  /// Get the flag that controls automatic clearing of forces after each time step.
  bool getAutoClearForces() {
    return (flags & clearForcesBit) == clearForcesBit;
  }

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

      if (seed.isAwake() == false || seed.isActive() == false) {
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
        assert(body.isActive() == true);
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
          if (joint.islandFlag == true) {
            continue;
          }

          final other = joint.getOtherBody(body);

          // Don't simulate joints connected to inactive bodies.
          if (other.isActive() == false) {
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

  final Island toiIsland = Island();
  final TOIInput toiInput = TOIInput();
  final TOIOutput toiOutput = TOIOutput();
  final TimeStep subStep = TimeStep();
  final Sweep backup1 = Sweep();
  final Sweep backup2 = Sweep();

  void solveTOI(final TimeStep step) {
    final island = toiIsland..init(contactManager.contactListener);
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
      Contact minContact;
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

          final activeA = bodyA.isAwake() && typeA != BodyType.static;
          final activeB = bodyB.isAwake() && typeB != BodyType.static;

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
          } else if (bodyB.sweep.alpha0 < bodyA.sweep.alpha0) {
            alpha0 = bodyA.sweep.alpha0;
            bodyB.sweep.advance(alpha0);
          }

          assert(alpha0 < 1.0);

          final indexA = contact.indexA;
          final indexB = contact.indexB;

          // Compute the time of impact in interval [0, minTOI]
          final input = toiInput;
          input.proxyA.set(fixtureA.shape, indexA);
          input.proxyB.set(fixtureB.shape, indexB);
          input.sweepA.set(bodyA.sweep);
          input.sweepB.set(bodyB.sweep);
          input.tMax = 1.0;

          toi.timeOfImpact(toiOutput, input);

          // Beta is the fraction of the remaining portion of the .
          final beta = toiOutput.t;
          if (toiOutput.state == TOIOutputState.touching) {
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

      backup1.set(bodyA.sweep);
      backup2.set(bodyB.sweep);

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
        bodyA.sweep.set(backup1);
        bodyB.sweep.set(backup2);
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
            backup1.set(other.sweep);
            if ((other.flags & Body.islandFlag) == 0) {
              other.advance(minAlpha);
            }

            // Update the contact points
            contact.update(contactManager.contactListener);

            // Was the contact disabled by the user?
            if (contact.isEnabled() == false) {
              other.sweep.set(backup1);
              other.synchronizeTransform();
              continue;
            }

            // Are there contact points?
            if (contact.isTouching() == false) {
              other.sweep.set(backup1);
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

      subStep.dt = (1.0 - minAlpha) * step.dt;
      subStep.invDt = 1.0 / subStep.dt;
      subStep.dtRatio = 1.0;
      subStep.positionIterations = 20;
      subStep.velocityIterations = step.velocityIterations;
      subStep.warmStarting = false;
      island.solveTOI(subStep, bodyA.islandIndex, bodyB.islandIndex);

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

  void drawJoint(Joint joint) {
    final bodyA = joint.bodyA;
    final bodyB = joint.bodyB;
    final xf1 = bodyA.transform;
    final xf2 = bodyB.transform;
    final x1 = xf1.p;
    final x2 = xf2.p;
    final p1 = Vector2.copy(joint.getAnchorA());
    final p2 = Vector2.copy(joint.getAnchorB());

    color.setFromRGBd(0.5, 0.8, 0.8);

    switch (joint.getType()) {
      // TODO djm write after writing joints
      case JointType.distance:
        debugDraw.drawSegment(p1, p2, color);
        break;

      case JointType.pulley:
        {
          final pulley = joint as PulleyJoint;
          final s1 = pulley.getGroundAnchorA();
          final s2 = pulley.getGroundAnchorB();
          debugDraw.drawSegment(s1, p1, color);
          debugDraw.drawSegment(s2, p2, color);
          debugDraw.drawSegment(s1, s2, color);
        }
        break;

      case JointType.friction:
        debugDraw.drawSegment(x1, x2, color);
        break;

      case JointType.constantVolume:
      case JointType.mouse:
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
  static const int liquidFlag = 1234598372;
  double liquidLength = .12;
  double averageLinearVel = -1.0;
  final Vector2 liquidOffset = Vector2.zero();
  final Vector2 circleCenterMoved = Vector2.zero();
  final Color3i liquidColor = Color3i.fromRGBd(0.4, .4, 1.0);

  final Vector2 center = Vector2.zero();
  final Vector2 axis = Vector2.zero();
  final Vector2 v1 = Vector2.zero();
  final Vector2 v2 = Vector2.zero();

  void drawShape(Fixture fixture, Transform xf, Color3i color, bool wireframe) {
    switch (fixture.getType()) {
      case ShapeType.circle:
        {
          final circle = fixture.shape as CircleShape;

          center.setFrom(Transform.mulVec2(xf, circle.position));
          final radius = circle.radius;
          xf.q.getXAxis(axis);

          if (fixture.userData != null && fixture.userData == liquidFlag) {
            final b = fixture.body;
            liquidOffset.setFrom(b.linearVelocity);
            final linVelLength = b.linearVelocity.length;
            if (averageLinearVel == -1) {
              averageLinearVel = linVelLength;
            } else {
              averageLinearVel = .98 * averageLinearVel + .02 * linVelLength;
            }
            liquidOffset.scale(liquidLength / averageLinearVel / 2);
            circleCenterMoved
              ..setFrom(center)
              ..add(liquidOffset);
            center.sub(liquidOffset);
            debugDraw.drawSegment(center, circleCenterMoved, liquidColor);
            return;
          }
          if (wireframe) {
            debugDraw.drawCircleAxis(center, radius, axis, color);
          } else {
            debugDraw.drawSolidCircle(center, radius, color);
          }
        }
        break;
      case ShapeType.polygon:
        {
          final poly = fixture.shape as PolygonShape;
          assert(poly.vertices.length <= settings.maxPolygonVertices);
          final vertices = poly.vertices
              .map(
                (vertex) => Transform.mulVec2(xf, vertex),
              )
              .toList();

          if (wireframe) {
            debugDraw.drawPolygon(vertices, color);
          } else {
            debugDraw.drawSolidPolygon(vertices, color);
          }
        }
        break;
      case ShapeType.edge:
        {
          final edge = fixture.shape as EdgeShape;
          v1.setFrom(Transform.mulVec2(xf, edge.vertex1));
          v2.setFrom(Transform.mulVec2(xf, edge.vertex2));
          debugDraw.drawSegment(v1, v2, color);
        }
        break;
      case ShapeType.chain:
        {
          final chain = fixture.shape as ChainShape;
          final count = chain.vertexCount;
          final vertices = chain.vertices;

          v1.setFrom(Transform.mulVec2(xf, vertices[0]));
          for (var i = 1; i < count; ++i) {
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
    final wireframe =
        (debugDraw.drawFlags & DebugDraw.wireFrameDrawingBit) != 0;
    if (system.particles.isNotEmpty) {
      final particleRadius = system.particleRadius;
      if (wireframe) {
        debugDraw.drawParticlesWireframe(system.particles, particleRadius);
      } else {
        debugDraw.drawParticles(system.particles, particleRadius);
      }
    }
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
    final proxy = userData;
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
      return callback.reportFixture(fixture, _point, _output.normal, fraction);
    }

    return input.maxFraction;
  }

  BroadPhase broadPhase;
  RayCastCallback callback;
}
