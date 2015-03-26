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
 * The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
 * world also contains efficient memory management facilities.
 */
class World {
  static const int WORLD_POOL_SIZE = 100;
  static const int WORLD_POOL_CONTAINER_SIZE = 10;

  static const int NEW_FIXTURE = 0x0001;
  static const int LOCKED = 0x0002;
  static const int CLEAR_FORCES = 0x0004;

  // statistics gathering
  int activeContacts = 0;
  int contactPoolCount = 0;

  int m_flags = 0;

  ContactManager m_contactManager;
  // TODO(srdjan): Make fields private.
  Body m_bodyList;
  Joint m_jointList;

  int m_bodyCount = 0;
  int m_jointCount = 0;

  final Vector2 m_gravity;
  bool m_allowSleep = false;

  // Body m_groundBody;

  DestructionListener m_destructionListener;
  ParticleDestructionListener m_particleDestructionListener;
  DebugDraw m_debugDraw;

  final IWorldPool pool;

  /**
   * This is used to compute the time step ratio to support a variable time step.
   */
  double m_inv_dt0 = 0.0;

  // these are for debugging the solver
  bool m_warmStarting = false;
  bool m_continuousPhysics = false;
  bool m_subStepping = false;

  bool m_stepComplete = false;

  Profile m_profile;

  ParticleSystem m_particleSystem;

  static List<List<ContactRegister>> _create2D(int a, int b) {
    var res = new List<List<ContactRegister>>(a);
    for (int i = 0; i < a; i++) {
      res[i] = new List<ContactRegister>(b);
    }
    return res;
  }

  List<List<ContactRegister>> contactStacks =
      _create2D(ShapeType.values.length, ShapeType.values.length);

  /**
   * Construct a world object.
   *
   * @param gravity the world gravity vector.
   */
  factory World.withGravity(Vector2 gravity) {
    var w = new World.withPool(gravity,
        new DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE));
    return w;
  }

  /**
   * Construct a world object.
   *
   * @param gravity the world gravity vector.
   */
  factory World.withPool(Vector2 gravity, IWorldPool pool) {
    var w = new World.withPoolAndStrategy(gravity, pool, new DynamicTree());
    return w;
  }

  factory World.withPoolAndStrategy(
      Vector2 gravity, IWorldPool pool, BroadPhaseStrategy strategy) {
    var w = new World(gravity, pool, new DefaultBroadPhaseBuffer(strategy));
    return w;
  }

  World(Vector2 gravity, this.pool, BroadPhase broadPhase)
      : m_gravity = new Vector2.copy(gravity) {
    m_destructionListener = null;
    m_debugDraw = null;

    m_bodyList = null;
    m_jointList = null;

    m_bodyCount = 0;
    m_jointCount = 0;

    m_warmStarting = true;
    m_continuousPhysics = true;
    m_subStepping = false;
    m_stepComplete = true;

    m_allowSleep = true;

    m_flags = CLEAR_FORCES;

    m_inv_dt0 = 0.0;

    m_contactManager = new ContactManager(this, broadPhase);
    m_profile = new Profile();

    m_particleSystem = new ParticleSystem(this);

    _initializeRegisters();
  }

  void setAllowSleep(bool flag) {
    if (flag == m_allowSleep) {
      return;
    }

    m_allowSleep = flag;
    if (m_allowSleep == false) {
      for (Body b = m_bodyList; b != null; b = b.m_next) {
        b.setAwake(true);
      }
    }
  }

  void setSubStepping(bool subStepping) {
    this.m_subStepping = subStepping;
  }

  bool isSubStepping() {
    return m_subStepping;
  }

  bool isAllowSleep() {
    return m_allowSleep;
  }

  void _addType(
      IDynamicStack<Contact> creator, ShapeType type1, ShapeType type2) {
    ContactRegister register = new ContactRegister();
    register.creator = creator;
    register.primary = true;
    contactStacks[type1.index][type2.index] = register;

    if (type1 != type2) {
      ContactRegister register2 = new ContactRegister();
      register2.creator = creator;
      register2.primary = false;
      contactStacks[type2.index][type1.index] = register2;
    }
  }

  void _initializeRegisters() {
    _addType(pool.getCircleContactStack(), ShapeType.CIRCLE, ShapeType.CIRCLE);
    _addType(
        pool.getPolyCircleContactStack(), ShapeType.POLYGON, ShapeType.CIRCLE);
    _addType(pool.getPolyContactStack(), ShapeType.POLYGON, ShapeType.POLYGON);
    _addType(
        pool.getEdgeCircleContactStack(), ShapeType.EDGE, ShapeType.CIRCLE);
    _addType(pool.getEdgePolyContactStack(), ShapeType.EDGE, ShapeType.POLYGON);
    _addType(
        pool.getChainCircleContactStack(), ShapeType.CHAIN, ShapeType.CIRCLE);
    _addType(
        pool.getChainPolyContactStack(), ShapeType.CHAIN, ShapeType.POLYGON);
  }

  DestructionListener getDestructionListener() {
    return m_destructionListener;
  }

  ParticleDestructionListener getParticleDestructionListener() {
    return m_particleDestructionListener;
  }

  void setParticleDestructionListener(ParticleDestructionListener listener) {
    m_particleDestructionListener = listener;
  }

  Contact popContact(
      Fixture fixtureA, int indexA, Fixture fixtureB, int indexB) {
    final ShapeType type1 = fixtureA.getType();
    final ShapeType type2 = fixtureB.getType();

    final ContactRegister reg = contactStacks[type1.index][type2.index];
    if (reg != null) {
      if (reg.primary) {
        Contact c = reg.creator.pop();
        c.init(fixtureA, indexA, fixtureB, indexB);
        return c;
      } else {
        Contact c = reg.creator.pop();
        c.init(fixtureB, indexB, fixtureA, indexA);
        return c;
      }
    } else {
      return null;
    }
  }

  void pushContact(Contact contact) {
    Fixture fixtureA = contact.fixtureA;
    Fixture fixtureB = contact.fixtureB;

    if (contact.m_manifold.pointCount > 0 &&
        !fixtureA.isSensor() &&
        !fixtureB.isSensor()) {
      fixtureA.getBody().setAwake(true);
      fixtureB.getBody().setAwake(true);
    }

    ShapeType type1 = fixtureA.getType();
    ShapeType type2 = fixtureB.getType();

    IDynamicStack<Contact> creator =
        contactStacks[type1.index][type2.index].creator;
    creator.push(contact);
  }

  IWorldPool getPool() {
    return pool;
  }

  /**
   * Register a destruction listener. The listener is owned by you and must remain in scope.
   *
   * @param listener
   */
  void setDestructionListener(DestructionListener listener) {
    m_destructionListener = listener;
  }

  /**
   * Register a contact filter to provide specific control over collision. Otherwise the default
   * filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
   *
   * @param filter
   */
  void setContactFilter(ContactFilter filter) {
    m_contactManager.m_contactFilter = filter;
  }

  /**
   * Register a contact event listener. The listener is owned by you and must remain in scope.
   *
   * @param listener
   */
  void setContactListener(ContactListener listener) {
    m_contactManager.m_contactListener = listener;
  }

  /**
   * Register a routine for debug drawing. The debug draw functions are called inside with
   * World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
   *
   * @param debugDraw
   */
  void setDebugDraw(DebugDraw debugDraw) {
    m_debugDraw = debugDraw;
  }

  /**
   * create a rigid body given a definition. No reference to the definition is retained.
   *
   * @warning This function is locked during callbacks.
   * @param def
   * @return
   */
  Body createBody(BodyDef def) {
    assert(isLocked() == false);
    if (isLocked()) {
      return null;
    }
    // TODO djm pooling
    Body b = new Body(def, this);

    // add to world doubly linked list
    b.m_prev = null;
    b.m_next = m_bodyList;
    if (m_bodyList != null) {
      m_bodyList.m_prev = b;
    }
    m_bodyList = b;
    ++m_bodyCount;

    return b;
  }

  /**
   * destroy a rigid body given a definition. No reference to the definition is retained. This
   * function is locked during callbacks.
   *
   * @warning This automatically deletes all associated shapes and joints.
   * @warning This function is locked during callbacks.
   * @param body
   */
  void destroyBody(Body body) {
    assert(m_bodyCount > 0);
    assert(isLocked() == false);
    if (isLocked()) {
      return;
    }

    // Delete the attached joints.
    JointEdge je = body.m_jointList;
    while (je != null) {
      JointEdge je0 = je;
      je = je.next;
      if (m_destructionListener != null) {
        m_destructionListener.sayGoodbyeJoint(je0.joint);
      }

      destroyJoint(je0.joint);

      body.m_jointList = je;
    }
    body.m_jointList = null;

    // Delete the attached contacts.
    ContactEdge ce = body.m_contactList;
    while (ce != null) {
      ContactEdge ce0 = ce;
      ce = ce.next;
      m_contactManager.destroy(ce0.contact);
    }
    body.m_contactList = null;

    Fixture f = body.m_fixtureList;
    while (f != null) {
      Fixture f0 = f;
      f = f.m_next;

      if (m_destructionListener != null) {
        m_destructionListener.sayGoodbyeFixture(f0);
      }

      f0.destroyProxies(m_contactManager.m_broadPhase);
      f0.destroy();
      // TODO djm recycle fixtures (here or in that destroy method)
      body.m_fixtureList = f;
      body.m_fixtureCount -= 1;
    }
    body.m_fixtureList = null;
    body.m_fixtureCount = 0;

    // Remove world body list.
    if (body.m_prev != null) {
      body.m_prev.m_next = body.m_next;
    }

    if (body.m_next != null) {
      body.m_next.m_prev = body.m_prev;
    }

    if (body == m_bodyList) {
      m_bodyList = body.m_next;
    }

    --m_bodyCount;
    // TODO djm recycle body
  }

  /**
   * create a joint to constrain bodies together. No reference to the definition is retained. This
   * may cause the connected bodies to cease colliding.
   *
   * @warning This function is locked during callbacks.
   * @param def
   * @return
   */
  Joint createJoint(JointDef def) {
    assert(isLocked() == false);
    if (isLocked()) {
      return null;
    }

    Joint j = Joint.create(this, def);

    // Connect to the world list.
    j.m_prev = null;
    j.m_next = m_jointList;
    if (m_jointList != null) {
      m_jointList.m_prev = j;
    }
    m_jointList = j;
    ++m_jointCount;

    // Connect to the bodies' doubly linked lists.
    j.m_edgeA.joint = j;
    j.m_edgeA.other = j.getBodyB();
    j.m_edgeA.prev = null;
    j.m_edgeA.next = j.getBodyA().m_jointList;
    if (j.getBodyA().m_jointList != null) {
      j.getBodyA().m_jointList.prev = j.m_edgeA;
    }
    j.getBodyA().m_jointList = j.m_edgeA;

    j.m_edgeB.joint = j;
    j.m_edgeB.other = j.getBodyA();
    j.m_edgeB.prev = null;
    j.m_edgeB.next = j.getBodyB().m_jointList;
    if (j.getBodyB().m_jointList != null) {
      j.getBodyB().m_jointList.prev = j.m_edgeB;
    }
    j.getBodyB().m_jointList = j.m_edgeB;

    Body bodyA = def.bodyA;
    Body bodyB = def.bodyB;

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

    return j;
  }

  /**
   * destroy a joint. This may cause the connected bodies to begin colliding.
   *
   * @warning This function is locked during callbacks.
   * @param joint
   */
  void destroyJoint(Joint j) {
    assert(isLocked() == false);
    if (isLocked()) {
      return;
    }

    bool collideConnected = j.getCollideConnected();

    // Remove from the doubly linked list.
    if (j.m_prev != null) {
      j.m_prev.m_next = j.m_next;
    }

    if (j.m_next != null) {
      j.m_next.m_prev = j.m_prev;
    }

    if (j == m_jointList) {
      m_jointList = j.m_next;
    }

    // Disconnect from island graph.
    Body bodyA = j.getBodyA();
    Body bodyB = j.getBodyB();

    // Wake up connected bodies.
    bodyA.setAwake(true);
    bodyB.setAwake(true);

    // Remove from body 1.
    if (j.m_edgeA.prev != null) {
      j.m_edgeA.prev.next = j.m_edgeA.next;
    }

    if (j.m_edgeA.next != null) {
      j.m_edgeA.next.prev = j.m_edgeA.prev;
    }

    if (j.m_edgeA == bodyA.m_jointList) {
      bodyA.m_jointList = j.m_edgeA.next;
    }

    j.m_edgeA.prev = null;
    j.m_edgeA.next = null;

    // Remove from body 2
    if (j.m_edgeB.prev != null) {
      j.m_edgeB.prev.next = j.m_edgeB.next;
    }

    if (j.m_edgeB.next != null) {
      j.m_edgeB.next.prev = j.m_edgeB.prev;
    }

    if (j.m_edgeB == bodyB.m_jointList) {
      bodyB.m_jointList = j.m_edgeB.next;
    }

    j.m_edgeB.prev = null;
    j.m_edgeB.next = null;

    Joint.destroy(j);

    assert(m_jointCount > 0);
    --m_jointCount;

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
  final TimeStep step = new TimeStep();
  final Timer stepTimer = new Timer();
  final Timer tempTimer = new Timer();

  /**
   * Take a time step. This performs collision detection, integration, and constraint solution.
   *
   * @param timeStep the amount of time to simulate, this should not vary.
   * @param velocityIterations for the velocity constraint solver.
   * @param positionIterations for the position constraint solver.
   */
  void stepDt(double dt, int velocityIterations, int positionIterations) {
    stepTimer.reset();
    tempTimer.reset();
    // log.debug("Starting step");
    // If new fixtures were added, we need to find the new contacts.
    if ((m_flags & NEW_FIXTURE) == NEW_FIXTURE) {
      // log.debug("There's a new fixture, lets look for new contacts");
      m_contactManager.findNewContacts();
      m_flags &= ~NEW_FIXTURE;
    }

    m_flags |= LOCKED;

    step.dt = dt;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    if (dt > 0.0) {
      step.inv_dt = 1.0 / dt;
    } else {
      step.inv_dt = 0.0;
    }

    step.dtRatio = m_inv_dt0 * dt;

    step.warmStarting = m_warmStarting;
    m_profile.stepInit.record(tempTimer.getMilliseconds());

    // Update contacts. This is where some contacts are destroyed.
    tempTimer.reset();
    m_contactManager.collide();
    m_profile.collide.record(tempTimer.getMilliseconds());

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (m_stepComplete && step.dt > 0.0) {
      tempTimer.reset();
      m_particleSystem.solve(step); // Particle Simulation
      m_profile.solveParticleSystem.record(tempTimer.getMilliseconds());
      tempTimer.reset();
      solve(step);
      m_profile.solve.record(tempTimer.getMilliseconds());
    }

    // Handle TOI events.
    if (m_continuousPhysics && step.dt > 0.0) {
      tempTimer.reset();
      solveTOI(step);
      m_profile.solveTOI.record(tempTimer.getMilliseconds());
    }

    if (step.dt > 0.0) {
      m_inv_dt0 = step.inv_dt;
    }

    if ((m_flags & CLEAR_FORCES) == CLEAR_FORCES) {
      clearForces();
    }

    m_flags &= ~LOCKED;
    // log.debug("ending step");

    m_profile.step.record(stepTimer.getMilliseconds());
  }

  /**
   * Call this after you are done with time steps to clear the forces. You normally call this after
   * each call to Step, unless you are performing sub-steps. By default, forces will be
   * automatically cleared, so you don't need to call this function.
   *
   * @see setAutoClearForces
   */
  void clearForces() {
    for (Body body = m_bodyList; body != null; body = body.getNext()) {
      body.m_force.setZero();
      body.m_torque = 0.0;
    }
  }

  final Color3i color = new Color3i.zero();
  final Transform xf = new Transform.zero();
  final Vector2 cA = new Vector2.zero();
  final Vector2 cB = new Vector2.zero();
  final Vec2Array avs = new Vec2Array();

  /**
   * Call this to draw shapes and other debug draw data.
   */
  void drawDebugData() {
    if (m_debugDraw == null) {
      return;
    }

    int flags = m_debugDraw.getFlags();
    bool wireframe = (flags & DebugDraw.e_wireframeDrawingBit) != 0;

    if ((flags & DebugDraw.e_shapeBit) != 0) {
      for (Body b = m_bodyList; b != null; b = b.getNext()) {
        xf.set(b.getTransform());
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
      drawParticleSystem(m_particleSystem);
    }

    if ((flags & DebugDraw.e_jointBit) != 0) {
      for (Joint j = m_jointList; j != null; j = j.getNext()) {
        drawJoint(j);
      }
    }

    if ((flags & DebugDraw.e_pairBit) != 0) {
      color.setFromRGBd(0.3, 0.9, 0.9);
      for (Contact c = m_contactManager.m_contactList;
          c != null;
          c = c.getNext()) {
        Fixture fixtureA = c.fixtureA;
        Fixture fixtureB = c.fixtureB;
        fixtureA.getAABB(c.getChildIndexA()).getCenterToOut(cA);
        fixtureB.getAABB(c.getChildIndexB()).getCenterToOut(cB);
        m_debugDraw.drawSegment(cA, cB, color);
      }
    }

    if ((flags & DebugDraw.e_aabbBit) != 0) {
      color.setFromRGBd(0.9, 0.3, 0.9);

      for (Body b = m_bodyList; b != null; b = b.getNext()) {
        if (b.isActive() == false) {
          continue;
        }

        for (Fixture f = b.getFixtureList(); f != null; f = f.getNext()) {
          for (int i = 0; i < f.m_proxyCount; ++i) {
            FixtureProxy proxy = f.m_proxies[i];
            AABB aabb = m_contactManager.m_broadPhase.getFatAABB(proxy.proxyId);
            if (aabb != null) {
              List<Vector2> vs = avs.get(4);
              vs[0].setValues(aabb.lowerBound.x, aabb.lowerBound.y);
              vs[1].setValues(aabb.upperBound.x, aabb.lowerBound.y);
              vs[2].setValues(aabb.upperBound.x, aabb.upperBound.y);
              vs[3].setValues(aabb.lowerBound.x, aabb.upperBound.y);
              m_debugDraw.drawPolygon(vs, 4, color);
            }
          }
        }
      }
    }

    if ((flags & DebugDraw.e_centerOfMassBit) != 0) {
      final Color3i xfColor = new Color3i(255, 0, 0);
      for (Body b = m_bodyList; b != null; b = b.getNext()) {
        xf.set(b.getTransform());
        xf.p.setFrom(b.worldCenter);
        m_debugDraw.drawTransform(xf, xfColor);
      }
    }

    if ((flags & DebugDraw.e_dynamicTreeBit) != 0) {
      m_contactManager.m_broadPhase.drawTree(m_debugDraw);
    }

    m_debugDraw.flush();
  }

  final WorldQueryWrapper wqwrapper = new WorldQueryWrapper();

  /**
   * Query the world for all fixtures that potentially overlap the provided AABB.
   *
   * @param callback a user implemented callback class.
   * @param aabb the query box.
   */
  void queryAABB(QueryCallback callback, AABB aabb) {
    wqwrapper.broadPhase = m_contactManager.m_broadPhase;
    wqwrapper.callback = callback;
    m_contactManager.m_broadPhase.query(wqwrapper, aabb);
  }

  /**
   * Query the world for all fixtures and particles that potentially overlap the provided AABB.
   *
   * @param callback a user implemented callback class.
   * @param particleCallback callback for particles.
   * @param aabb the query box.
   */
  void queryAABBTwoCallbacks(QueryCallback callback,
      ParticleQueryCallback particleCallback, AABB aabb) {
    wqwrapper.broadPhase = m_contactManager.m_broadPhase;
    wqwrapper.callback = callback;
    m_contactManager.m_broadPhase.query(wqwrapper, aabb);
    m_particleSystem.queryAABB(particleCallback, aabb);
  }

  /**
   * Query the world for all particles that potentially overlap the provided AABB.
   *
   * @param particleCallback callback for particles.
   * @param aabb the query box.
   */
  void queryAABBParticle(ParticleQueryCallback particleCallback, AABB aabb) {
    m_particleSystem.queryAABB(particleCallback, aabb);
  }

  final WorldRayCastWrapper wrcwrapper = new WorldRayCastWrapper();
  final RayCastInput input = new RayCastInput();

  /**
   * Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
   * get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
   * starting point.
   *
   * @param callback a user implemented callback class.
   * @param point1 the ray starting point
   * @param point2 the ray ending point
   */
  void raycast(RayCastCallback callback, Vector2 point1, Vector2 point2) {
    wrcwrapper.broadPhase = m_contactManager.m_broadPhase;
    wrcwrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setFrom(point1);
    input.p2.setFrom(point2);
    m_contactManager.m_broadPhase.raycast(wrcwrapper, input);
  }

  /**
   * Ray-cast the world for all fixtures and particles in the path of the ray. Your callback
   * controls whether you get the closest point, any point, or n-points. The ray-cast ignores shapes
   * that contain the starting point.
   *
   * @param callback a user implemented callback class.
   * @param particleCallback the particle callback class.
   * @param point1 the ray starting point
   * @param point2 the ray ending point
   */
  void raycastTwoCallBacks(RayCastCallback callback,
      ParticleRaycastCallback particleCallback, Vector2 point1,
      Vector2 point2) {
    wrcwrapper.broadPhase = m_contactManager.m_broadPhase;
    wrcwrapper.callback = callback;
    input.maxFraction = 1.0;
    input.p1.setFrom(point1);
    input.p2.setFrom(point2);
    m_contactManager.m_broadPhase.raycast(wrcwrapper, input);
    m_particleSystem.raycast(particleCallback, point1, point2);
  }

  /**
   * Ray-cast the world for all particles in the path of the ray. Your callback controls whether you
   * get the closest point, any point, or n-points.
   *
   * @param particleCallback the particle callback class.
   * @param point1 the ray starting point
   * @param point2 the ray ending point
   */
  void raycastParticle(ParticleRaycastCallback particleCallback, Vector2 point1,
      Vector2 point2) {
    m_particleSystem.raycast(particleCallback, point1, point2);
  }

  /**
   * Get the world body list. With the returned body, use Body.getNext to get the next body in the
   * world list. A null body indicates the end of the list.
   *
   * @return the head of the world body list.
   */
  Body getBodyList() {
    return m_bodyList;
  }

  /**
   * Get the world joint list. With the returned joint, use Joint.getNext to get the next joint in
   * the world list. A null joint indicates the end of the list.
   *
   * @return the head of the world joint list.
   */
  Joint getJointList() {
    return m_jointList;
  }

  /**
   * Get the world contact list. With the returned contact, use Contact.getNext to get the next
   * contact in the world list. A null contact indicates the end of the list.
   *
   * @return the head of the world contact list.
   * @warning contacts are created and destroyed in the middle of a time step. Use ContactListener
   *          to avoid missing contacts.
   */
  Contact getContactList() {
    return m_contactManager.m_contactList;
  }

  bool isSleepingAllowed() {
    return m_allowSleep;
  }

  void setSleepingAllowed(bool sleepingAllowed) {
    m_allowSleep = sleepingAllowed;
  }

  /**
   * Enable/disable warm starting. For testing.
   *
   * @param flag
   */
  void setWarmStarting(bool flag) {
    m_warmStarting = flag;
  }

  bool isWarmStarting() {
    return m_warmStarting;
  }

  /**
   * Enable/disable continuous physics. For testing.
   *
   * @param flag
   */
  void setContinuousPhysics(bool flag) {
    m_continuousPhysics = flag;
  }

  bool isContinuousPhysics() {
    return m_continuousPhysics;
  }

  /**
   * Get the number of broad-phase proxies.
   *
   * @return
   */
  int getProxyCount() {
    return m_contactManager.m_broadPhase.getProxyCount();
  }

  /**
   * Get the number of bodies.
   *
   * @return
   */
  int getBodyCount() {
    return m_bodyCount;
  }

  /**
   * Get the number of joints.
   *
   * @return
   */
  int getJointCount() {
    return m_jointCount;
  }

  /**
   * Get the number of contacts (each may have 0 or more contact points).
   *
   * @return
   */
  int getContactCount() {
    return m_contactManager.m_contactCount;
  }

  /**
   * Gets the height of the dynamic tree
   *
   * @return
   */
  int getTreeHeight() {
    return m_contactManager.m_broadPhase.getTreeHeight();
  }

  /**
   * Gets the balance of the dynamic tree
   *
   * @return
   */
  int getTreeBalance() {
    return m_contactManager.m_broadPhase.getTreeBalance();
  }

  /**
   * Gets the quality of the dynamic tree
   *
   * @return
   */
  double getTreeQuality() {
    return m_contactManager.m_broadPhase.getTreeQuality();
  }

  /**
   * Change the global gravity vector.
   *
   * @param gravity
   */
  void setGravity(Vector2 gravity) {
    m_gravity.setFrom(gravity);
  }

  /**
   * Get the global gravity vector.
   *
   * @return
   */
  Vector2 getGravity() {
    return m_gravity;
  }

  /**
   * Is the world locked (in the middle of a time step).
   *
   * @return
   */
  bool isLocked() {
    return (m_flags & LOCKED) == LOCKED;
  }

  /**
   * Set flag to control automatic clearing of forces after each time step.
   *
   * @param flag
   */
  void setAutoClearForces(bool flag) {
    if (flag) {
      m_flags |= CLEAR_FORCES;
    } else {
      m_flags &= ~CLEAR_FORCES;
    }
  }

  /**
   * Get the flag that controls automatic clearing of forces after each time step.
   *
   * @return
   */
  bool getAutoClearForces() {
    return (m_flags & CLEAR_FORCES) == CLEAR_FORCES;
  }

  /**
   * Get the contact manager for testing purposes
   *
   * @return
   */
  ContactManager getContactManager() {
    return m_contactManager;
  }

  Profile getProfile() {
    return m_profile;
  }

  final Island island = new Island();
  List<Body> stack =
      new List<Body>(10); // TODO djm find a good initial stack number;
  final Timer broadphaseTimer = new Timer();

  void solve(TimeStep step) {
    m_profile.solveInit.startAccum();
    m_profile.solveVelocity.startAccum();
    m_profile.solvePosition.startAccum();

    // update previous transforms
    for (Body b = m_bodyList; b != null; b = b.m_next) {
      b.m_xf0.set(b.m_xf);
    }

    // Size the island for the worst case.
    island.init(m_bodyCount, m_contactManager.m_contactCount, m_jointCount,
        m_contactManager.m_contactListener);

    // Clear all the island flags.
    for (Body b = m_bodyList; b != null; b = b.m_next) {
      b.m_flags &= ~Body.e_islandFlag;
    }
    for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next) {
      c.m_flags &= ~Contact.ISLAND_FLAG;
    }
    for (Joint j = m_jointList; j != null; j = j.m_next) {
      j.m_islandFlag = false;
    }

    // Build and simulate all awake islands.
    int stackSize = m_bodyCount;
    if (stack.length < stackSize) {
      stack = new List<Body>(stackSize);
    }
    for (Body seed = m_bodyList; seed != null; seed = seed.m_next) {
      if ((seed.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
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
      seed.m_flags |= Body.e_islandFlag;

      // Perform a depth first search (DFS) on the constraint graph.
      while (stackCount > 0) {
        // Grab the next body off the stack and add it to the island.
        Body b = stack[--stackCount];
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
        for (ContactEdge ce = b.m_contactList; ce != null; ce = ce.next) {
          Contact contact = ce.contact;

          // Has this contact already been added to an island?
          if ((contact.m_flags & Contact.ISLAND_FLAG) == Contact.ISLAND_FLAG) {
            continue;
          }

          // Is this contact solid and touching?
          if (contact.isEnabled() == false || contact.isTouching() == false) {
            continue;
          }

          // Skip sensors.
          bool sensorA = contact._fixtureA.m_isSensor;
          bool sensorB = contact._fixtureB.m_isSensor;
          if (sensorA || sensorB) {
            continue;
          }

          island.addContact(contact);
          contact.m_flags |= Contact.ISLAND_FLAG;

          Body other = ce.other;

          // Was the other body already added to this island?
          if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
            continue;
          }

          assert(stackCount < stackSize);
          stack[stackCount++] = other;
          other.m_flags |= Body.e_islandFlag;
        }

        // Search all joints connect to this body.
        for (JointEdge je = b.m_jointList; je != null; je = je.next) {
          if (je.joint.m_islandFlag == true) {
            continue;
          }

          Body other = je.other;

          // Don't simulate joints connected to inactive bodies.
          if (other.isActive() == false) {
            continue;
          }

          island.addJoint(je.joint);
          je.joint.m_islandFlag = true;

          if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag) {
            continue;
          }

          assert(stackCount < stackSize);
          stack[stackCount++] = other;
          other.m_flags |= Body.e_islandFlag;
        }
      }
      island.solve(m_profile, step, m_gravity, m_allowSleep);

      // Post solve cleanup.
      for (int i = 0; i < island.m_bodyCount; ++i) {
        // Allow static bodies to participate in other islands.
        Body b = island.m_bodies[i];
        if (b.getType() == BodyType.STATIC) {
          b.m_flags &= ~Body.e_islandFlag;
        }
      }
    }
    m_profile.solveInit.endAccum();
    m_profile.solveVelocity.endAccum();
    m_profile.solvePosition.endAccum();

    broadphaseTimer.reset();
    // Synchronize fixtures, check for out of range bodies.
    for (Body b = m_bodyList; b != null; b = b.getNext()) {
      // If a body was not in an island then it did not move.
      if ((b.m_flags & Body.e_islandFlag) == 0) {
        continue;
      }

      if (b.getType() == BodyType.STATIC) {
        continue;
      }

      // Update fixtures (for broad-phase).
      b.synchronizeFixtures();
    }

    // Look for new contacts.
    m_contactManager.findNewContacts();
    m_profile.broadphase.record(broadphaseTimer.getMilliseconds());
  }

  final Island toiIsland = new Island();
  final TOIInput toiInput = new TOIInput();
  final TOIOutput toiOutput = new TOIOutput();
  final TimeStep subStep = new TimeStep();
  final List<Body> tempBodies = new List<Body>(2);
  final Sweep backup1 = new Sweep();
  final Sweep backup2 = new Sweep();

  void solveTOI(final TimeStep step) {
    final Island island = toiIsland;
    island.init(2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0,
        m_contactManager.m_contactListener);
    if (m_stepComplete) {
      for (Body b = m_bodyList; b != null; b = b.m_next) {
        b.m_flags &= ~Body.e_islandFlag;
        b.m_sweep.alpha0 = 0.0;
      }

      for (Contact c = m_contactManager.m_contactList;
          c != null;
          c = c.m_next) {
        // Invalidate TOI
        c.m_flags &= ~(Contact.TOI_FLAG | Contact.ISLAND_FLAG);
        c.m_toiCount = 0;
        c.m_toi = 1.0;
      }
    }

    // Find TOI events and solve them.
    for (;;) {
      // Find the first TOI.
      Contact minContact = null;
      double minAlpha = 1.0;

      for (Contact c = m_contactManager.m_contactList;
          c != null;
          c = c.m_next) {
        // Is this contact disabled?
        if (c.isEnabled() == false) {
          continue;
        }

        // Prevent excessive sub-stepping.
        if (c.m_toiCount > Settings.maxSubSteps) {
          continue;
        }

        double alpha = 1.0;
        if ((c.m_flags & Contact.TOI_FLAG) != 0) {
          // This contact has a valid cached TOI.
          alpha = c.m_toi;
        } else {
          Fixture fA = c.fixtureA;
          Fixture fB = c.fixtureB;

          // Is there a sensor?
          if (fA.isSensor() || fB.isSensor()) {
            continue;
          }

          Body bA = fA.getBody();
          Body bB = fB.getBody();

          BodyType typeA = bA.m_type;
          BodyType typeB = bB.m_type;
          assert(typeA == BodyType.DYNAMIC || typeB == BodyType.DYNAMIC);

          bool activeA = bA.isAwake() && typeA != BodyType.STATIC;
          bool activeB = bB.isAwake() && typeB != BodyType.STATIC;

          // Is at least one body active (awake and dynamic or kinematic)?
          if (activeA == false && activeB == false) {
            continue;
          }

          bool collideA = bA.isBullet() || typeA != BodyType.DYNAMIC;
          bool collideB = bB.isBullet() || typeB != BodyType.DYNAMIC;

          // Are these two non-bullet dynamic bodies?
          if (collideA == false && collideB == false) {
            continue;
          }

          // Compute the TOI for this contact.
          // Put the sweeps onto the same time interval.
          double alpha0 = bA.m_sweep.alpha0;

          if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0) {
            alpha0 = bB.m_sweep.alpha0;
            bA.m_sweep.advance(alpha0);
          } else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0) {
            alpha0 = bA.m_sweep.alpha0;
            bB.m_sweep.advance(alpha0);
          }

          assert(alpha0 < 1.0);

          int indexA = c.getChildIndexA();
          int indexB = c.getChildIndexB();

          // Compute the time of impact in interval [0, minTOI]
          final TOIInput input = toiInput;
          input.proxyA.set(fA.getShape(), indexA);
          input.proxyB.set(fB.getShape(), indexB);
          input.sweepA.set(bA.m_sweep);
          input.sweepB.set(bB.m_sweep);
          input.tMax = 1.0;

          pool.getTimeOfImpact().timeOfImpact(toiOutput, input);

          // Beta is the fraction of the remaining portion of the .
          double beta = toiOutput.t;
          if (toiOutput.state == TOIOutputState.TOUCHING) {
            alpha = Math.min(alpha0 + (1.0 - alpha0) * beta, 1.0);
          } else {
            alpha = 1.0;
          }

          c.m_toi = alpha;
          c.m_flags |= Contact.TOI_FLAG;
        }

        if (alpha < minAlpha) {
          // This is the minimum TOI found so far.
          minContact = c;
          minAlpha = alpha;
        }
      }

      if (minContact == null || 1.0 - 10.0 * Settings.EPSILON < minAlpha) {
        // No more TOI events. Done!
        m_stepComplete = true;
        break;
      }

      // Advance the bodies to the TOI.
      Fixture fA = minContact.fixtureA;
      Fixture fB = minContact.fixtureB;
      Body bA = fA.getBody();
      Body bB = fB.getBody();

      backup1.set(bA.m_sweep);
      backup2.set(bB.m_sweep);

      bA.advance(minAlpha);
      bB.advance(minAlpha);

      // The TOI contact likely has some new contact points.
      minContact.update(m_contactManager.m_contactListener);
      minContact.m_flags &= ~Contact.TOI_FLAG;
      ++minContact.m_toiCount;

      // Is the contact solid?
      if (minContact.isEnabled() == false || minContact.isTouching() == false) {
        // Restore the sweeps.
        minContact.setEnabled(false);
        bA.m_sweep.set(backup1);
        bB.m_sweep.set(backup2);
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

      bA.m_flags |= Body.e_islandFlag;
      bB.m_flags |= Body.e_islandFlag;
      minContact.m_flags |= Contact.ISLAND_FLAG;

      // Get contacts on bodyA and bodyB.
      tempBodies[0] = bA;
      tempBodies[1] = bB;
      for (int i = 0; i < 2; ++i) {
        Body body = tempBodies[i];
        if (body.m_type == BodyType.DYNAMIC) {
          for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next) {
            if (island.m_bodyCount == island.m_bodyCapacity) {
              break;
            }

            if (island.m_contactCount == island.m_contactCapacity) {
              break;
            }

            Contact contact = ce.contact;

            // Has this contact already been added to the island?
            if ((contact.m_flags & Contact.ISLAND_FLAG) != 0) {
              continue;
            }

            // Only add static, kinematic, or bullet bodies.
            Body other = ce.other;
            if (other.m_type == BodyType.DYNAMIC &&
                body.isBullet() == false &&
                other.isBullet() == false) {
              continue;
            }

            // Skip sensors.
            bool sensorA = contact._fixtureA.m_isSensor;
            bool sensorB = contact._fixtureB.m_isSensor;
            if (sensorA || sensorB) {
              continue;
            }

            // Tentatively advance the body to the TOI.
            backup1.set(other.m_sweep);
            if ((other.m_flags & Body.e_islandFlag) == 0) {
              other.advance(minAlpha);
            }

            // Update the contact points
            contact.update(m_contactManager.m_contactListener);

            // Was the contact disabled by the user?
            if (contact.isEnabled() == false) {
              other.m_sweep.set(backup1);
              other.synchronizeTransform();
              continue;
            }

            // Are there contact points?
            if (contact.isTouching() == false) {
              other.m_sweep.set(backup1);
              other.synchronizeTransform();
              continue;
            }

            // Add the contact to the island
            contact.m_flags |= Contact.ISLAND_FLAG;
            island.addContact(contact);

            // Has the other body already been added to the island?
            if ((other.m_flags & Body.e_islandFlag) != 0) {
              continue;
            }

            // Add the other body to the island.
            other.m_flags |= Body.e_islandFlag;

            if (other.m_type != BodyType.STATIC) {
              other.setAwake(true);
            }

            island.addBody(other);
          }
        }
      }

      subStep.dt = (1.0 - minAlpha) * step.dt;
      subStep.inv_dt = 1.0 / subStep.dt;
      subStep.dtRatio = 1.0;
      subStep.positionIterations = 20;
      subStep.velocityIterations = step.velocityIterations;
      subStep.warmStarting = false;
      island.solveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex);

      // Reset island flags and synchronize broad-phase proxies.
      for (int i = 0; i < island.m_bodyCount; ++i) {
        Body body = island.m_bodies[i];
        body.m_flags &= ~Body.e_islandFlag;

        if (body.m_type != BodyType.DYNAMIC) {
          continue;
        }

        body.synchronizeFixtures();

        // Invalidate all contact TOIs on this displaced body.
        for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next) {
          ce.contact.m_flags &= ~(Contact.TOI_FLAG | Contact.ISLAND_FLAG);
        }
      }

      // Commit fixture proxy movements to the broad-phase so that new contacts are created.
      // Also, some contacts can be destroyed.
      m_contactManager.findNewContacts();

      if (m_subStepping) {
        m_stepComplete = false;
        break;
      }
    }
  }

  void drawJoint(Joint joint) {
    Body bodyA = joint.getBodyA();
    Body bodyB = joint.getBodyB();
    Transform xf1 = bodyA.getTransform();
    Transform xf2 = bodyB.getTransform();
    Vector2 x1 = xf1.p;
    Vector2 x2 = xf2.p;
    Vector2 p1 = pool.popVec2();
    Vector2 p2 = pool.popVec2();
    joint.getAnchorA(p1);
    joint.getAnchorB(p2);

    color.setFromRGBd(0.5, 0.8, 0.8);

    switch (joint.getType()) {
      // TODO djm write after writing joints
      case JointType.DISTANCE:
        m_debugDraw.drawSegment(p1, p2, color);
        break;

      case JointType.PULLEY:
        {
          PulleyJoint pulley = joint;
          Vector2 s1 = pulley.getGroundAnchorA();
          Vector2 s2 = pulley.getGroundAnchorB();
          m_debugDraw.drawSegment(s1, p1, color);
          m_debugDraw.drawSegment(s2, p2, color);
          m_debugDraw.drawSegment(s1, s2, color);
        }
        break;
      case JointType.CONSTANT_VOLUME:
      case JointType.MOUSE:
        // don't draw this
        break;
      default:
        m_debugDraw.drawSegment(x1, p1, color);
        m_debugDraw.drawSegment(p1, p2, color);
        m_debugDraw.drawSegment(x2, p2, color);
    }
    pool.pushVec2(2);
  }

  // NOTE this corresponds to the liquid test, so the debugdraw can draw
  // the liquid particles correctly. They should be the same.
  static int LIQUID_INT = 1234598372;
  double liquidLength = .12;
  double averageLinearVel = -1.0;
  final Vector2 liquidOffset = new Vector2.zero();
  final Vector2 circCenterMoved = new Vector2.zero();
  final Color3i liquidColor = new Color3i.fromRGBd(.4, .4, 1.0);

  final Vector2 center = new Vector2.zero();
  final Vector2 axis = new Vector2.zero();
  final Vector2 v1 = new Vector2.zero();
  final Vector2 v2 = new Vector2.zero();
  final Vec2Array tlvertices = new Vec2Array();

  void drawShape(Fixture fixture, Transform xf, Color3i color, bool wireframe) {
    switch (fixture.getType()) {
      case ShapeType.CIRCLE:
        {
          CircleShape circle = fixture.getShape();

          // Vec2 center = Mul(xf, circle.m_p);
          Transform.mulToOutUnsafeVec2(xf, circle.m_p, center);
          double radius = circle.radius;
          xf.q.getXAxis(axis);

          if (fixture.userData != null && fixture.userData == LIQUID_INT) {
            Body b = fixture.getBody();
            liquidOffset.setFrom(b._linearVelocity);
            double linVelLength = b._linearVelocity.length;
            if (averageLinearVel == -1) {
              averageLinearVel = linVelLength;
            } else {
              averageLinearVel = .98 * averageLinearVel + .02 * linVelLength;
            }
            liquidOffset.scale(liquidLength / averageLinearVel / 2);
            circCenterMoved.setFrom(center).add(liquidOffset);
            center.sub(liquidOffset);
            m_debugDraw.drawSegment(center, circCenterMoved, liquidColor);
            return;
          }
          if (wireframe) {
            m_debugDraw.drawCircleAxis(center, radius, axis, color);
          } else {
            m_debugDraw.drawSolidCircle(center, radius, axis, color);
          }
        }
        break;

      case ShapeType.POLYGON:
        {
          PolygonShape poly = fixture.getShape();
          int vertexCount = poly.m_count;
          assert(vertexCount <= Settings.maxPolygonVertices);
          List<Vector2> vertices = tlvertices.get(Settings.maxPolygonVertices);

          for (int i = 0; i < vertexCount; ++i) {
            // vertices[i] = Mul(xf, poly.m_vertices[i]);
            Transform.mulToOutUnsafeVec2(xf, poly.m_vertices[i], vertices[i]);
          }
          if (wireframe) {
            m_debugDraw.drawPolygon(vertices, vertexCount, color);
          } else {
            m_debugDraw.drawSolidPolygon(vertices, vertexCount, color);
          }
        }
        break;
      case ShapeType.EDGE:
        {
          EdgeShape edge = fixture.getShape();
          Transform.mulToOutUnsafeVec2(xf, edge.m_vertex1, v1);
          Transform.mulToOutUnsafeVec2(xf, edge.m_vertex2, v2);
          m_debugDraw.drawSegment(v1, v2, color);
        }
        break;
      case ShapeType.CHAIN:
        {
          ChainShape chain = fixture.getShape();
          int count = chain.m_count;
          List<Vector2> vertices = chain.m_vertices;

          Transform.mulToOutUnsafeVec2(xf, vertices[0], v1);
          for (int i = 1; i < count; ++i) {
            Transform.mulToOutUnsafeVec2(xf, vertices[i], v2);
            m_debugDraw.drawSegment(v1, v2, color);
            m_debugDraw.drawCircle(v1, 0.05, color);
            v1.setFrom(v2);
          }
        }
        break;
      default:
        break;
    }
  }

  void drawParticleSystem(ParticleSystem system) {
    bool wireframe =
        (m_debugDraw.getFlags() & DebugDraw.e_wireframeDrawingBit) != 0;
    int particleCount = system.getParticleCount();
    if (particleCount != 0) {
      double particleRadius = system.getParticleRadius();
      List<Vector2> positionBuffer = system.getParticlePositionBuffer();
      List<ParticleColor> colorBuffer = null;
      if (system.m_colorBuffer.data != null) {
        colorBuffer = system.getParticleColorBuffer();
      }
      if (wireframe) {
        m_debugDraw.drawParticlesWireframe(
            positionBuffer, particleRadius, colorBuffer, particleCount);
      } else {
        m_debugDraw.drawParticles(
            positionBuffer, particleRadius, colorBuffer, particleCount);
      }
    }
  }

  /**
   * Create a particle whose properties have been defined. No reference to the definition is
   * retained. A simulation step must occur before it's possible to interact with a newly created
   * particle. For example, DestroyParticleInShape() will not destroy a particle until Step() has
   * been called.
   *
   * @warning This function is locked during callbacks.
   * @return the index of the particle.
   */
  int createParticle(ParticleDef def) {
    assert(isLocked() == false);
    if (isLocked()) {
      return 0;
    }
    int p = m_particleSystem.createParticle(def);
    return p;
  }

  /**
   * Destroy a particle. The particle is removed after the next step.
   *
   * @param index
   */
  void destroyParticle(int index) {
    destroyParticleFlag(index, false);
  }

  /**
   * Destroy a particle. The particle is removed after the next step.
   *
   * @param Index of the particle to destroy.
   * @param Whether to call the destruction listener just before the particle is destroyed.
   */
  void destroyParticleFlag(int index, bool callDestructionListener) {
    m_particleSystem.destroyParticle(index, callDestructionListener);
  }

  /**
   * Destroy particles inside a shape without enabling the destruction callback for destroyed
   * particles. This function is locked during callbacks. For more information see
   * DestroyParticleInShape(Shape&, Transform&,bool).
   *
   * @param Shape which encloses particles that should be destroyed.
   * @param Transform applied to the shape.
   * @warning This function is locked during callbacks.
   * @return Number of particles destroyed.
   */
  int destroyParticlesInShape(Shape shape, Transform xf) {
    return destroyParticlesInShapeFlag(shape, xf, false);
  }

  /**
   * Destroy particles inside a shape. This function is locked during callbacks. In addition, this
   * function immediately destroys particles in the shape in contrast to DestroyParticle() which
   * defers the destruction until the next simulation step.
   *
   * @param Shape which encloses particles that should be destroyed.
   * @param Transform applied to the shape.
   * @param Whether to call the world b2DestructionListener for each particle destroyed.
   * @warning This function is locked during callbacks.
   * @return Number of particles destroyed.
   */
  int destroyParticlesInShapeFlag(
      Shape shape, Transform xf, bool callDestructionListener) {
    assert(isLocked() == false);
    if (isLocked()) {
      return 0;
    }
    return m_particleSystem.destroyParticlesInShape(
        shape, xf, callDestructionListener);
  }

  /**
   * Create a particle group whose properties have been defined. No reference to the definition is
   * retained.
   *
   * @warning This function is locked during callbacks.
   */
  ParticleGroup createParticleGroup(ParticleGroupDef def) {
    assert(isLocked() == false);
    if (isLocked()) {
      return null;
    }
    ParticleGroup g = m_particleSystem.createParticleGroup(def);
    return g;
  }

  /**
   * Join two particle groups.
   *
   * @param the first group. Expands to encompass the second group.
   * @param the second group. It is destroyed.
   * @warning This function is locked during callbacks.
   */
  void joinParticleGroups(ParticleGroup groupA, ParticleGroup groupB) {
    assert(isLocked() == false);
    if (isLocked()) {
      return;
    }
    m_particleSystem.joinParticleGroups(groupA, groupB);
  }

  /**
   * Destroy particles in a group. This function is locked during callbacks.
   *
   * @param The particle group to destroy.
   * @param Whether to call the world b2DestructionListener for each particle is destroyed.
   * @warning This function is locked during callbacks.
   */
  void destroyParticlesInGroupFlag(
      ParticleGroup group, bool callDestructionListener) {
    assert(isLocked() == false);
    if (isLocked()) {
      return;
    }
    m_particleSystem.destroyParticlesInGroup(group, callDestructionListener);
  }

  /**
   * Destroy particles in a group without enabling the destruction callback for destroyed particles.
   * This function is locked during callbacks.
   *
   * @param The particle group to destroy.
   * @warning This function is locked during callbacks.
   */
  void destroyParticlesInGroup(ParticleGroup group) {
    destroyParticlesInGroupFlag(group, false);
  }

  /**
   * Get the world particle group list. With the returned group, use ParticleGroup::GetNext to get
   * the next group in the world list. A NULL group indicates the end of the list.
   *
   * @return the head of the world particle group list.
   */
  List<ParticleGroup> getParticleGroupList() {
    return m_particleSystem.getParticleGroupList();
  }

  /**
   * Get the number of particle groups.
   *
   * @return
   */
  int getParticleGroupCount() {
    return m_particleSystem.getParticleGroupCount();
  }

  /**
   * Get the number of particles.
   *
   * @return
   */
  int getParticleCount() {
    return m_particleSystem.getParticleCount();
  }

  /**
   * Get the maximum number of particles.
   *
   * @return
   */
  int getParticleMaxCount() {
    return m_particleSystem.getParticleMaxCount();
  }

  /**
   * Set the maximum number of particles.
   *
   * @param count
   */
  void setParticleMaxCount(int count) {
    m_particleSystem.setParticleMaxCount(count);
  }

  /**
   * Change the particle density.
   *
   * @param density
   */
  void setParticleDensity(double density) {
    m_particleSystem.setParticleDensity(density);
  }

  /**
   * Get the particle density.
   *
   * @return
   */
  double getParticleDensity() {
    return m_particleSystem.getParticleDensity();
  }

  /**
   * Change the particle gravity scale. Adjusts the effect of the global gravity vector on
   * particles. Default value is 1.0.
   *
   * @param gravityScale
   */
  void setParticleGravityScale(double gravityScale) {
    m_particleSystem.setParticleGravityScale(gravityScale);
  }

  /**
   * Get the particle gravity scale.
   *
   * @return
   */
  double getParticleGravityScale() {
    return m_particleSystem.getParticleGravityScale();
  }

  /**
   * Damping is used to reduce the velocity of particles. The damping parameter can be larger than
   * 1.0 but the damping effect becomes sensitive to the time step when the damping parameter is
   * large.
   *
   * @param damping
   */
  void setParticleDamping(double damping) {
    m_particleSystem.setParticleDamping(damping);
  }

  /**
   * Get damping for particles
   *
   * @return
   */
  double getParticleDamping() {
    return m_particleSystem.getParticleDamping();
  }

  /**
   * Change the particle radius. You should set this only once, on world start. If you change the
   * radius during execution, existing particles may explode, shrink, or behave unexpectedly.
   *
   * @param radius
   */
  void setParticleRadius(double radius) {
    m_particleSystem.setParticleRadius(radius);
  }

  /**
   * Get the particle radius.
   *
   * @return
   */
  double getParticleRadius() {
    return m_particleSystem.getParticleRadius();
  }

  /**
   * Get the particle data. @return the pointer to the head of the particle data.
   *
   * @return
   */
  List<int> getParticleFlagsBuffer() {
    return m_particleSystem.getParticleFlagsBuffer();
  }

  List<Vector2> getParticlePositionBuffer() {
    return m_particleSystem.getParticlePositionBuffer();
  }

  List<Vector2> getParticleVelocityBuffer() {
    return m_particleSystem.getParticleVelocityBuffer();
  }

  List<ParticleColor> getParticleColorBuffer() {
    return m_particleSystem.getParticleColorBuffer();
  }

  List<ParticleGroup> getParticleGroupBuffer() {
    return m_particleSystem.getParticleGroupBuffer();
  }

  List<Object> getParticleUserDataBuffer() {
    return m_particleSystem.getParticleUserDataBuffer();
  }

  /**
   * Set a buffer for particle data.
   *
   * @param buffer is a pointer to a block of memory.
   * @param size is the number of values in the block.
   */
  void setParticleFlagsBuffer(List<int> buffer, int capacity) {
    m_particleSystem.setParticleFlagsBuffer(buffer, capacity);
  }

  void setParticlePositionBuffer(List<Vector2> buffer, int capacity) {
    m_particleSystem.setParticlePositionBuffer(buffer, capacity);
  }

  void setParticleVelocityBuffer(List<Vector2> buffer, int capacity) {
    m_particleSystem.setParticleVelocityBuffer(buffer, capacity);
  }

  void setParticleColorBuffer(List<ParticleColor> buffer, int capacity) {
    m_particleSystem.setParticleColorBuffer(buffer, capacity);
  }

  void setParticleUserDataBuffer(List<Object> buffer, int capacity) {
    m_particleSystem.setParticleUserDataBuffer(buffer, capacity);
  }

  /**
   * Get contacts between particles
   *
   * @return
   */
  List<ParticleContact> getParticleContacts() {
    return m_particleSystem.m_contactBuffer;
  }

  int getParticleContactCount() {
    return m_particleSystem.m_contactCount;
  }

  /**
   * Get contacts between particles and bodies
   *
   * @return
   */
  List<ParticleBodyContact> getParticleBodyContacts() {
    return m_particleSystem.m_bodyContactBuffer;
  }

  int getParticleBodyContactCount() {
    return m_particleSystem.m_bodyContactCount;
  }

  /**
   * Compute the kinetic energy that can be lost by damping force
   *
   * @return
   */
  double computeParticleCollisionEnergy() {
    return m_particleSystem.computeParticleCollisionEnergy();
  }

  // For debugging purposes.
  void forEachBody(void action(Body body)) {
    for (Body body = m_bodyList; body != null; body = body.getNext()) {
      action(body);
    }
  }
} // class World.

class WorldQueryWrapper implements TreeCallback {
  bool treeCallback(int nodeId) {
    FixtureProxy proxy = broadPhase.getUserData(nodeId);
    return callback.reportFixture(proxy.fixture);
  }

  BroadPhase broadPhase;
  QueryCallback callback;
}

class WorldRayCastWrapper implements TreeRayCastCallback {

  // djm pooling
  final RayCastOutput _output = new RayCastOutput();
  final Vector2 _temp = new Vector2.zero();
  final Vector2 _point = new Vector2.zero();

  double raycastCallback(RayCastInput input, int nodeId) {
    Object userData = broadPhase.getUserData(nodeId);
    FixtureProxy proxy = userData;
    Fixture fixture = proxy.fixture;
    int index = proxy.childIndex;
    bool hit = fixture.raycast(_output, input, index);

    if (hit) {
      double fraction = _output.fraction;
      // Vec2 point = (1.0 - fraction) * input.p1 + fraction * input.p2;
      _temp.setFrom(input.p2).scale(fraction);
      _point.setFrom(input.p1).scale(1 - fraction).add(_temp);
      return callback.reportFixture(fixture, _point, _output.normal, fraction);
    }

    return input.maxFraction;
  }

  BroadPhase broadPhase;
  RayCastCallback callback;
}
