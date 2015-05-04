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

/*
 Position Correction Notes
 =========================
 I tried the several algorithms for position correction of the 2D revolute joint.
 I looked at these systems:
 - simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
 - suspension bridge with 30 1m long planks of length 1m.
 - multi-link chain with 30 1m long links.

 Here are the algorithms:

 Baumgarte - A fraction of the position error is added to the velocity error. There is no
 separate position solver.

 Pseudo Velocities - After the velocity solver and position integration,
 the position error, Jacobian, and effective mass are recomputed. Then
 the velocity constraints are solved with pseudo velocities and a fraction
 of the position error is added to the pseudo velocity error. The pseudo
 velocities are initialized to zero and there is no warm-starting. After
 the position solver, the pseudo velocities are added to the positions.
 This is also called the First Order World method or the Position LCP method.

 Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
 position error is re-computed for each raint and the positions are updated
 after the raint is solved. The radius vectors (aka Jacobians) are
 re-computed too (otherwise the algorithm has horrible instability). The pseudo
 velocity states are not needed because they are effectively zero at the beginning
 of each iteration. Since we have the current position error, we allow the
 iterations to terminate early if the error becomes smaller than Settings.linearSlop.

 Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
 each time a raint is solved.

 Here are the results:
 Baumgarte - this is the cheapest algorithm but it has some stability problems,
 especially with the bridge. The chain links separate easily close to the root
 and they jitter as they struggle to pull together. This is one of the most common
 methods in the field. The big drawback is that the position correction artificially
 affects the momentum, thus leading to instabilities and false bounce. I used a
 bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
 factor makes joints and contacts more spongy.

 Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
 stable. However, joints still separate with large angular velocities. Drag the
 simple pendulum in a circle quickly and the joint will separate. The chain separates
 easily and does not recover. I used a bias factor of 0.2. A larger value lead to
 the bridge collapsing when a heavy cube drops on it.

 Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
 Velocities, but in other ways it is worse. The bridge and chain are much more
 stable, but the simple pendulum goes unstable at high angular velocities.

 Full NGS - stable in all tests. The joints display good stiffness. The bridge
 still sags, but this is better than infinite forces.

 Recommendations
 Pseudo Velocities are not really worthwhile because the bridge and chain cannot
 recover from joint separation. In other cases the benefit over Baumgarte is small.

 Modified NGS is not a robust method for the revolute joint due to the violent
 instability seen in the simple pendulum. Perhaps it is viable with other raint
 types, especially scalar constraints where the effective mass is a scalar.

 This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
 and is very fast. I don't think we can escape Baumgarte, especially in highly
 demanding cases where high raint fidelity is not needed.

 Full NGS is robust and easy on the eyes. I recommend this as an option for
 higher fidelity simulation and certainly for suspension bridges and long chains.
 Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
 joint separation can be problematic. The number of NGS iterations can be reduced
 for better performance without harming robustness much.

 Each joint in a can be handled differently in the position solver. So I recommend
 a system where the user can select the algorithm on a per joint basis. I would
 probably default to the slower Full NGS and let the user select the faster
 Baumgarte method in performance critical scenarios.
 */

/*
 Cache Performance

 The Box2D solvers are dominated by cache misses. Data structures are designed
 to increase the number of cache hits. Much of misses are due to random access
 to body data. The raint structures are iterated over linearly, which leads
 to few cache misses.

 The bodies are not accessed during iteration. Instead read only data, such as
 the mass values are stored with the constraints. The mutable data are the raint
 impulses and the bodies velocities/positions. The impulses are held inside the
 raint structures. The body velocities/positions are held in compact, temporary
 arrays to increase the number of cache hits. Linear and angular velocity are
 stored in a single array since multiple arrays lead to multiple misses.
 */

/*
 2D Rotation

 R = [cos(theta) -sin(theta)]
 [sin(theta) cos(theta) ]

 thetaDot = omega

 Let q1 = cos(theta), q2 = sin(theta).
 R = [q1 -q2]
 [q2  q1]

 q1Dot = -thetaDot * q2
 q2Dot = thetaDot * q1

 q1_new = q1_old - dt * w * q2
 q2_new = q2_old + dt * w * q1
 then normalize.

 This might be faster than computing sin+cos.
 However, we can compute sin+cos of the same angle fast.
 */

/**
 * This is an internal class.
 */
class Island {
  ContactListener m_listener;

  List<Body> m_bodies;
  List<Contact> m_contacts;
  List<Joint> m_joints;

  List<Position> m_positions;
  List<Velocity> m_velocities;

  int m_bodyCount = 0;
  int m_jointCount = 0;
  int m_contactCount = 0;

  int m_bodyCapacity = 0;
  int m_contactCapacity = 0;
  int m_jointCapacity = 0;

  void init(int bodyCapacity, int contactCapacity, int jointCapacity,
      ContactListener listener) {
    // System.out.println("Initializing Island");
    m_bodyCapacity = bodyCapacity;
    m_contactCapacity = contactCapacity;
    m_jointCapacity = jointCapacity;
    m_bodyCount = 0;
    m_contactCount = 0;
    m_jointCount = 0;

    m_listener = listener;

    if (m_bodies == null || m_bodyCapacity > m_bodies.length) {
      m_bodies = new List<Body>(m_bodyCapacity);
    }
    if (m_joints == null || m_jointCapacity > m_joints.length) {
      m_joints = new List<Joint>(m_jointCapacity);
    }
    if (m_contacts == null || m_contactCapacity > m_contacts.length) {
      m_contacts = new List<Contact>(m_contactCapacity);
    }

    // dynamic array
    if (m_velocities == null || m_bodyCapacity > m_velocities.length) {
      final List<Velocity> old =
          m_velocities == null ? new List<Velocity>(0) : m_velocities;
      m_velocities = new List<Velocity>(m_bodyCapacity);
      BufferUtils.arraycopy(old, 0, m_velocities, 0, old.length);
      for (int i = old.length; i < m_velocities.length; i++) {
        m_velocities[i] = new Velocity();
      }
    }

    // dynamic array
    if (m_positions == null || m_bodyCapacity > m_positions.length) {
      final List<Position> old =
          m_positions == null ? new List<Position>(0) : m_positions;
      m_positions = new List<Position>(m_bodyCapacity);
      BufferUtils.arraycopy(old, 0, m_positions, 0, old.length);
      for (int i = old.length; i < m_positions.length; i++) {
        m_positions[i] = new Position();
      }
    }
  }

  void clear() {
    m_bodyCount = 0;
    m_contactCount = 0;
    m_jointCount = 0;
  }

  final ContactSolver _contactSolver = new ContactSolver();
  final Timer _timer = new Timer();
  final SolverData _solverData = new SolverData();
  final ContactSolverDef _solverDef = new ContactSolverDef();

  void solve(Profile profile, TimeStep step, Vector2 gravity, bool allowSleep) {

    // System.out.println("Solving Island");
    double h = step.dt;

    // Integrate velocities and apply damping. Initialize the body state.
    for (int i = 0; i < m_bodyCount; ++i) {
      final Body b = m_bodies[i];
      final Sweep bm_sweep = b.sweep;
      final Vector2 c = bm_sweep.c;
      double a = bm_sweep.a;
      final Vector2 v = b._linearVelocity;
      double w = b._angularVelocity;

      // Store positions for continuous collision.
      bm_sweep.c0.setFrom(bm_sweep.c);
      bm_sweep.a0 = bm_sweep.a;

      if (b._bodyType == BodyType.DYNAMIC) {
        // Integrate velocities.
        // v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
        v.x += h * (b.gravityScale * gravity.x + b.invMass * b.force.x);
        v.y += h * (b.gravityScale * gravity.y + b.invMass * b.force.y);
        w += h * b.invI * b.torque;

        // Apply damping.
        // ODE: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v *
        // exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        v.x *= 1.0 / (1.0 + h * b.linearDamping);
        v.y *= 1.0 / (1.0 + h * b.linearDamping);
        w *= 1.0 / (1.0 + h * b.angularDamping);
      }

      m_positions[i].c.x = c.x;
      m_positions[i].c.y = c.y;
      m_positions[i].a = a;
      m_velocities[i].v.x = v.x;
      m_velocities[i].v.y = v.y;
      m_velocities[i].w = w;
    }

    _timer.reset();

    // Solver data
    _solverData.step = step;
    _solverData.positions = m_positions;
    _solverData.velocities = m_velocities;

    // Initialize velocity constraints.
    _solverDef.step = step;
    _solverDef.contacts = m_contacts;
    _solverDef.count = m_contactCount;
    _solverDef.positions = m_positions;
    _solverDef.velocities = m_velocities;

    _contactSolver.init(_solverDef);
    // System.out.println("island init vel");
    _contactSolver.initializeVelocityConstraints();

    if (step.warmStarting) {
      // System.out.println("island warm start");
      _contactSolver.warmStart();
    }

    for (int i = 0; i < m_jointCount; ++i) {
      m_joints[i].initVelocityConstraints(_solverData);
    }

    profile.solveInit.accum(_timer.getMilliseconds());

    // Solve velocity constraints
    _timer.reset();
    // System.out.println("island solving velocities");
    for (int i = 0; i < step.velocityIterations; ++i) {
      for (int j = 0; j < m_jointCount; ++j) {
        m_joints[j].solveVelocityConstraints(_solverData);
      }

      _contactSolver.solveVelocityConstraints();
    }

    // Store impulses for warm starting
    _contactSolver.storeImpulses();
    profile.solveVelocity.accum(_timer.getMilliseconds());

    // Integrate positions
    for (int i = 0; i < m_bodyCount; ++i) {
      final Vector2 c = m_positions[i].c;
      double a = m_positions[i].a;
      final Vector2 v = m_velocities[i].v;
      double w = m_velocities[i].w;

      // Check for large velocities
      double translationx = v.x * h;
      double translationy = v.y * h;

      if (translationx * translationx + translationy * translationy >
          Settings.maxTranslationSquared) {
        double ratio = Settings.maxTranslation /
            Math.sqrt(
                translationx * translationx + translationy * translationy);
        v.x *= ratio;
        v.y *= ratio;
      }

      double rotation = h * w;
      if (rotation * rotation > Settings.maxRotationSquared) {
        double ratio = Settings.maxRotation / rotation.abs();
        w *= ratio;
      }

      // Integrate
      c.x += h * v.x;
      c.y += h * v.y;
      a += h * w;

      m_positions[i].a = a;
      m_velocities[i].w = w;
    }

    // Solve position constraints
    _timer.reset();
    bool positionSolved = false;
    for (int i = 0; i < step.positionIterations; ++i) {
      bool contactsOkay = _contactSolver.solvePositionConstraints();

      bool jointsOkay = true;
      for (int j = 0; j < m_jointCount; ++j) {
        bool jointOkay = m_joints[j].solvePositionConstraints(_solverData);
        jointsOkay = jointsOkay && jointOkay;
      }

      if (contactsOkay && jointsOkay) {
        // Exit early if the position errors are small.
        positionSolved = true;
        break;
      }
    }

    // Copy state buffers back to the bodies
    for (int i = 0; i < m_bodyCount; ++i) {
      Body body = m_bodies[i];
      body.sweep.c.x = m_positions[i].c.x;
      body.sweep.c.y = m_positions[i].c.y;
      body.sweep.a = m_positions[i].a;
      body._linearVelocity.x = m_velocities[i].v.x;
      body._linearVelocity.y = m_velocities[i].v.y;
      body._angularVelocity = m_velocities[i].w;
      body.synchronizeTransform();
    }

    profile.solvePosition.accum(_timer.getMilliseconds());

    report(_contactSolver.m_velocityConstraints);

    if (allowSleep) {
      double minSleepTime = double.MAX_FINITE;

      final double linTolSqr =
          Settings.linearSleepTolerance * Settings.linearSleepTolerance;
      final double angTolSqr =
          Settings.angularSleepTolerance * Settings.angularSleepTolerance;

      for (int i = 0; i < m_bodyCount; ++i) {
        Body b = m_bodies[i];
        if (b.getType() == BodyType.STATIC) {
          continue;
        }

        if ((b.flags & Body.AUTO_SLEEP_FLAG) == 0 ||
            b._angularVelocity * b._angularVelocity > angTolSqr ||
            b._linearVelocity.dot(b._linearVelocity) > linTolSqr) {
          b.sleepTime = 0.0;
          minSleepTime = 0.0;
        } else {
          b.sleepTime += h;
          minSleepTime = Math.min(minSleepTime, b.sleepTime);
        }
      }

      if (minSleepTime >= Settings.timeToSleep && positionSolved) {
        for (int i = 0; i < m_bodyCount; ++i) {
          Body b = m_bodies[i];
          b.setAwake(false);
        }
      }
    }
  }

  final ContactSolver _toiContactSolver = new ContactSolver();
  final ContactSolverDef _toiSolverDef = new ContactSolverDef();

  void solveTOI(TimeStep subStep, int toiIndexA, int toiIndexB) {
    assert(toiIndexA < m_bodyCount);
    assert(toiIndexB < m_bodyCount);

    // Initialize the body state.
    for (int i = 0; i < m_bodyCount; ++i) {
      m_positions[i].c.x = m_bodies[i].sweep.c.x;
      m_positions[i].c.y = m_bodies[i].sweep.c.y;
      m_positions[i].a = m_bodies[i].sweep.a;
      m_velocities[i].v.x = m_bodies[i]._linearVelocity.x;
      m_velocities[i].v.y = m_bodies[i]._linearVelocity.y;
      m_velocities[i].w = m_bodies[i]._angularVelocity;
    }

    _toiSolverDef.contacts = m_contacts;
    _toiSolverDef.count = m_contactCount;
    _toiSolverDef.step = subStep;
    _toiSolverDef.positions = m_positions;
    _toiSolverDef.velocities = m_velocities;
    _toiContactSolver.init(_toiSolverDef);

    // Solve position constraints.
    for (int i = 0; i < subStep.positionIterations; ++i) {
      bool contactsOkay =
          _toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
      if (contactsOkay) {
        break;
      }
    }
    // #if 0
    // // Is the new position really safe?
    // for (int i = 0; i < m_contactCount; ++i)
    // {
    // Contact* c = m_contacts[i];
    // Fixture* fA = c.fixtureA;
    // Fixture* fB = c.fixtureB;
    //
    // Body bA = fA.GetBody();
    // Body bB = fB.GetBody();
    //
    // int indexA = c.GetChildIndexA();
    // int indexB = c.GetChildIndexB();
    //
    // DistanceInput input;
    // input.proxyA.Set(fA.GetShape(), indexA);
    // input.proxyB.Set(fB.GetShape(), indexB);
    // input.transformA = bA.GetTransform();
    // input.transformB = bB.GetTransform();
    // input.useRadii = false;
    //
    // DistanceOutput output;
    // SimplexCache cache;
    // cache.count = 0;
    // Distance(&output, &cache, &input);
    //
    // if (output.distance == 0 || cache.count == 3)
    // {
    // cache.count += 0;
    // }
    // }
    // #endif

    // Leap of faith to new safe state.
    m_bodies[toiIndexA].sweep.c0.x = m_positions[toiIndexA].c.x;
    m_bodies[toiIndexA].sweep.c0.y = m_positions[toiIndexA].c.y;
    m_bodies[toiIndexA].sweep.a0 = m_positions[toiIndexA].a;
    m_bodies[toiIndexB].sweep.c0.setFrom(m_positions[toiIndexB].c);
    m_bodies[toiIndexB].sweep.a0 = m_positions[toiIndexB].a;

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.
    _toiContactSolver.initializeVelocityConstraints();

    // Solve velocity constraints.
    for (int i = 0; i < subStep.velocityIterations; ++i) {
      _toiContactSolver.solveVelocityConstraints();
    }

    // Don't store the TOI contact forces for warm starting
    // because they can be quite large.

    double h = subStep.dt;

    // Integrate positions
    for (int i = 0; i < m_bodyCount; ++i) {
      Vector2 c = m_positions[i].c;
      double a = m_positions[i].a;
      Vector2 v = m_velocities[i].v;
      double w = m_velocities[i].w;

      // Check for large velocities
      double translationx = v.x * h;
      double translationy = v.y * h;
      if (translationx * translationx + translationy * translationy >
          Settings.maxTranslationSquared) {
        double ratio = Settings.maxTranslation /
            Math.sqrt(
                translationx * translationx + translationy * translationy);
        v.scale(ratio);
      }

      double rotation = h * w;
      if (rotation * rotation > Settings.maxRotationSquared) {
        double ratio = Settings.maxRotation / rotation.abs();
        w *= ratio;
      }

      // Integrate
      c.x += v.x * h;
      c.y += v.y * h;
      a += h * w;

      m_positions[i].c.x = c.x;
      m_positions[i].c.y = c.y;
      m_positions[i].a = a;
      m_velocities[i].v.x = v.x;
      m_velocities[i].v.y = v.y;
      m_velocities[i].w = w;

      // Sync bodies
      Body body = m_bodies[i];
      body.sweep.c.x = c.x;
      body.sweep.c.y = c.y;
      body.sweep.a = a;
      body._linearVelocity.x = v.x;
      body._linearVelocity.y = v.y;
      body._angularVelocity = w;
      body.synchronizeTransform();
    }

    report(_toiContactSolver.m_velocityConstraints);
  }

  void addBody(Body body) {
    assert(m_bodyCount < m_bodyCapacity);
    body.islandIndex = m_bodyCount;
    m_bodies[m_bodyCount] = body;
    ++m_bodyCount;
  }

  void addContact(Contact contact) {
    assert(m_contactCount < m_contactCapacity);
    m_contacts[m_contactCount++] = contact;
  }

  void addJoint(Joint joint) {
    assert(m_jointCount < m_jointCapacity);
    m_joints[m_jointCount++] = joint;
  }

  final ContactImpulse _impulse = new ContactImpulse();

  void report(List<ContactVelocityConstraint> constraints) {
    if (m_listener == null) {
      return;
    }

    for (int i = 0; i < m_contactCount; ++i) {
      Contact c = m_contacts[i];

      ContactVelocityConstraint vc = constraints[i];
      _impulse.count = vc.pointCount;
      for (int j = 0; j < vc.pointCount; ++j) {
        _impulse.normalImpulses[j] = vc.points[j].normalImpulse;
        _impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
      }

      m_listener.postSolve(c, _impulse);
    }
  }
}
