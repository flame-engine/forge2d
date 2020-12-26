part of forge2d;

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

// TODO: Refactor this later
class BodyMeta {
  final Body body;
  final Velocity velocity = Velocity();
  final Position position = Position();

  BodyMeta(this.body);
}

/// This is an internal class.
class Island {
  ContactListener _listener;

  List<BodyMeta> _bodies;
  List<Contact> _contacts;
  List<Joint> _joints;

  // TODO: Make this as efficient as it used to be
  List<Position> get _positions {
    return _bodies?.map((BodyMeta bodyMeta) => bodyMeta.position)?.toList() ??
        <Position>[];
  }

  List<Velocity> get _velocities {
    return _bodies?.map((BodyMeta bodyMeta) => bodyMeta.velocity)?.toList() ??
        <Velocity>[];
  }

  Island() {
    _bodies = <BodyMeta>[];
    _contacts = <Contact>[];
    _joints = <Joint>[];
  }

  void init(ContactListener listener) {
    _listener = listener;
  }

  void clear() {
    _bodies.clear();
    _contacts.clear();
    _joints.clear();
  }

  final ContactSolver _contactSolver = ContactSolver();
  final SolverData _solverData = SolverData();
  final ContactSolverDef _solverDef = ContactSolverDef();

  void solve(Profile profile, TimeStep step, Vector2 gravity, bool allowSleep) {
    final double dt = step.dt;

    // Integrate velocities and apply damping. Initialize the body state.
    for (BodyMeta bodyMeta in _bodies) {
      final Body b = bodyMeta.body;
      final Sweep bmSweep = b._sweep;
      final Vector2 c = bmSweep.c;
      final double a = bmSweep.a;
      final Vector2 v = b.linearVelocity;
      double w = b._angularVelocity;

      // Store positions for continuous collision.
      bmSweep.c0.setFrom(bmSweep.c);
      bmSweep.a0 = bmSweep.a;

      if (b._bodyType == BodyType.DYNAMIC) {
        // Integrate velocities.
        v.x += dt * (b._gravityScale * gravity.x + b._invMass * b._force.x);
        v.y += dt * (b._gravityScale * gravity.y + b._invMass * b._force.y);
        w += dt * b.inverseInertia * b._torque;

        // Apply damping.
        // ODE: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v *
        // exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        v.x *= 1.0 / (1.0 + dt * b.linearDamping);
        v.y *= 1.0 / (1.0 + dt * b.linearDamping);
        w *= 1.0 / (1.0 + dt * b.angularDamping);
      }

      bodyMeta.position.c.x = c.x;
      bodyMeta.position.c.y = c.y;
      bodyMeta.position.a = a;
      bodyMeta.velocity.v.x = v.x;
      bodyMeta.velocity.v.y = v.y;
      bodyMeta.velocity.w = w;
    }

    // Solver data
    _solverData.step = step;
    _solverData.positions = _positions;
    _solverData.velocities = _velocities;

    // Initialize velocity constraints.
    _solverDef.step = step;
    _solverDef.contacts = _contacts;
    _solverDef.positions = _positions;
    _solverDef.velocities = _velocities;

    _contactSolver.init(_solverDef);
    _contactSolver.initializeVelocityConstraints();

    if (step.warmStarting) {
      _contactSolver.warmStart();
    }

    for (Joint joint in _joints) {
      joint.initVelocityConstraints(_solverData);
    }

    for (int i = 0; i < step.velocityIterations; ++i) {
      for (Joint joint in _joints) {
        joint.solveVelocityConstraints(_solverData);
      }

      _contactSolver.solveVelocityConstraints();
    }

    // Store impulses for warm starting
    _contactSolver.storeImpulses();

    // Integrate positions
    for (BodyMeta bodyMeta in _bodies) {
      final Vector2 c = bodyMeta.position.c;
      double a = bodyMeta.position.a;
      final Vector2 v = bodyMeta.velocity.v;
      double w = bodyMeta.velocity.w;

      // Check for large velocities
      final double translationX = v.x * dt;
      final double translationY = v.y * dt;

      if (translationX * translationX + translationY * translationY >
          settings.maxTranslationSquared) {
        final double ratio = settings.maxTranslation /
            math.sqrt(
                translationX * translationX + translationY * translationY);
        v.x *= ratio;
        v.y *= ratio;
      }

      final double rotation = dt * w;
      if (rotation * rotation > settings.maxRotationSquared) {
        final double ratio = settings.maxRotation / rotation.abs();
        w *= ratio;
      }

      // Integrate
      c.x += dt * v.x;
      c.y += dt * v.y;
      a += dt * w;

      bodyMeta.position.a = a;
      bodyMeta.velocity.w = w;
    }

    // Solve position constraints
    bool positionSolved = false;
    for (int i = 0; i < step.positionIterations; ++i) {
      final bool contactsOkay = _contactSolver.solvePositionConstraints();

      bool jointsOkay = true;
      for (Joint joint in _joints) {
        final bool jointOkay = joint.solvePositionConstraints(_solverData);
        jointsOkay = jointsOkay && jointOkay;
      }

      if (contactsOkay && jointsOkay) {
        // Exit early if the position errors are small.
        positionSolved = true;
        break;
      }
    }

    // Copy state buffers back to the bodies
    for (BodyMeta bodyMeta in _bodies) {
      final Body body = bodyMeta.body;
      body._sweep.c.x = bodyMeta.position.c.x;
      body._sweep.c.y = bodyMeta.position.c.y;
      body._sweep.a = bodyMeta.position.a;
      body.linearVelocity.x = bodyMeta.velocity.v.x;
      body.linearVelocity.y = bodyMeta.velocity.v.y;
      body._angularVelocity = bodyMeta.velocity.w;
      body.synchronizeTransform();
    }

    reportVelocityConstraints();

    if (allowSleep) {
      double minSleepTime = double.maxFinite;

      const double linTolSqr =
          settings.linearSleepTolerance * settings.linearSleepTolerance;
      const double angTolSqr =
          settings.angularSleepTolerance * settings.angularSleepTolerance;

      for (BodyMeta bodyMeta in _bodies) {
        final Body b = bodyMeta.body;
        if (b.getType() == BodyType.STATIC) {
          continue;
        }

        if ((b._flags & Body.AUTO_SLEEP_FLAG) == 0 ||
            b._angularVelocity * b._angularVelocity > angTolSqr ||
            b.linearVelocity.dot(b.linearVelocity) > linTolSqr) {
          b._sleepTime = 0.0;
          minSleepTime = 0.0;
        } else {
          b._sleepTime += dt;
          minSleepTime = math.min(minSleepTime, b._sleepTime);
        }
      }

      if (minSleepTime >= settings.timeToSleep && positionSolved) {
        _bodies.forEach((b) => b.body.setAwake(false));
      }
    }
  }

  final ContactSolver _toiContactSolver = ContactSolver();
  final ContactSolverDef _toiSolverDef = ContactSolverDef();

  void solveTOI(TimeStep subStep, int toiIndexA, int toiIndexB) {
    assert(toiIndexA < _bodies.length);
    assert(toiIndexB < _bodies.length);

    // Initialize the body state.
    for (BodyMeta bodyMeta in _bodies) {
      final Body body = bodyMeta.body;
      bodyMeta.position.c.x = body._sweep.c.x;
      bodyMeta.position.c.y = body._sweep.c.y;
      bodyMeta.position.a = body._sweep.a;
      bodyMeta.velocity.v.x = body.linearVelocity.x;
      bodyMeta.velocity.v.y = body.linearVelocity.y;
      bodyMeta.velocity.w = body._angularVelocity;
    }

    // TODO: Is this correct, since it is no longer a fixed list?
    _toiSolverDef.contacts = _contacts;
    _toiSolverDef.step = subStep;
    _toiSolverDef.positions = _positions;
    _toiSolverDef.velocities = _velocities;
    _toiContactSolver.init(_toiSolverDef);

    // Solve position constraints.
    for (int i = 0; i < subStep.positionIterations; ++i) {
      final bool contactsOkay =
          _toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
      if (contactsOkay) {
        break;
      }
    }

    // Leap of faith to new safe state.
    _bodies[toiIndexA].body._sweep.c0.x = _positions[toiIndexA].c.x;
    _bodies[toiIndexA].body._sweep.c0.y = _positions[toiIndexA].c.y;
    _bodies[toiIndexA].body._sweep.a0 = _positions[toiIndexA].a;
    _bodies[toiIndexB].body._sweep.c0.setFrom(_positions[toiIndexB].c);
    _bodies[toiIndexB].body._sweep.a0 = _positions[toiIndexB].a;

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.
    _toiContactSolver.initializeVelocityConstraints();

    // Solve velocity constraints.
    for (int i = 0; i < subStep.velocityIterations; ++i) {
      _toiContactSolver.solveVelocityConstraints();
    }

    // Don't store the TOI contact forces for warm starting
    // because they can be quite large.

    final double dt = subStep.dt;

    // Integrate positions
    for (BodyMeta bodyMeta in _bodies) {
      final Position position = bodyMeta.position;
      final Velocity velocity = bodyMeta.velocity;
      final Vector2 c = position.c;
      double a = position.a;
      final Vector2 v = velocity.v;
      double w = velocity.w;

      // Check for large velocities
      final double translationX = v.x * dt;
      final double translationY = v.y * dt;
      if (translationX * translationX + translationY * translationY >
          settings.maxTranslationSquared) {
        final double ratio = settings.maxTranslation /
            math.sqrt(
                translationX * translationX + translationY * translationY);
        v.scale(ratio);
      }

      final double rotation = dt * w;
      if (rotation * rotation > settings.maxRotationSquared) {
        final double ratio = settings.maxRotation / rotation.abs();
        w *= ratio;
      }

      // Integrate
      c.x += v.x * dt;
      c.y += v.y * dt;
      a += dt * w;

      position.c.x = c.x;
      position.c.y = c.y;
      position.a = a;
      velocity.v.x = v.x;
      velocity.v.y = v.y;
      velocity.w = w;

      // Sync bodies
      final Body body = bodyMeta.body;
      body._sweep.c.x = c.x;
      body._sweep.c.y = c.y;
      body._sweep.a = a;
      body.linearVelocity.x = v.x;
      body.linearVelocity.y = v.y;
      body._angularVelocity = w;
      body.synchronizeTransform();
    }

    reportVelocityConstraints();
  }

  void addBody(Body body) {
    body.islandIndex = _bodies.length;
    _bodies.add(BodyMeta(body));
  }

  void addContact(Contact contact) {
    _contacts.add(contact);
  }

  void addJoint(Joint joint) {
    _joints.add(joint);
  }

  final ContactImpulse _impulse = ContactImpulse();

  void reportVelocityConstraints() {
    if (_listener == null) {
      return;
    }

    for (Contact contact in _contacts) {
      final ContactVelocityConstraint vc = contact.velocityConstraint;
      _impulse.count = vc.pointCount;
      for (int j = 0; j < vc.pointCount; ++j) {
        _impulse.normalImpulses[j] = vc.points[j].normalImpulse;
        _impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
      }

      _listener.postSolve(contact, _impulse);
    }
  }
}
