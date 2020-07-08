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

/// This is an internal class.
class Island {
  ContactListener _listener;

  List<Body> _bodies;
  List<Contact> _contacts;
  List<Joint> _joints;

  List<Position> _positions;
  List<Velocity> _velocities;

  int _bodyCount = 0;
  int _jointCount = 0;
  int _contactCount = 0;

  int _bodyCapacity = 0;
  int _contactCapacity = 0;
  int _jointCapacity = 0;

  void init(int bodyCapacity, int contactCapacity, int jointCapacity,
      ContactListener listener) {
    _bodyCapacity = bodyCapacity;
    _contactCapacity = contactCapacity;
    _jointCapacity = jointCapacity;
    _bodyCount = 0;
    _contactCount = 0;
    _jointCount = 0;

    _listener = listener;

    _bodies ??= List<Body>(_bodyCapacity);
    _joints ??= List<Joint>(_jointCapacity);
    _contacts ??= List<Contact>(_contactCapacity);
    print(_contacts.length);
    _velocities ??= List<Velocity>(0);
    _positions ??= List<Position>(0);

    // dynamic array
    if (_bodyCapacity > _velocities.length) {
      List<Velocity> updatedVelocities =
          List.filled(_bodyCapacity, Velocity(), growable: false);
      for (int i = 0; i < _velocities.length; i++) {
        updatedVelocities[i] = _velocities[i];
      }
      _velocities = updatedVelocities;
    }

    // dynamic array
    if (_bodyCapacity > _positions.length) {
      List<Position> updatedPositions =
          List.filled(_bodyCapacity, Position(), growable: false);
      for (int i = 0; i < _positions.length; i++) {
        updatedPositions[i] = _positions[i];
      }
      _positions = updatedPositions;
    }
  }

  void clear() {
    _bodyCount = 0;
    _contactCount = 0;
    _jointCount = 0;
  }

  final ContactSolver _contactSolver = ContactSolver();
  final SolverData _solverData = SolverData();
  final ContactSolverDef _solverDef = ContactSolverDef();

  void solve(Profile profile, TimeStep step, Vector2 gravity, bool allowSleep) {
    double h = step.dt;

    // Integrate velocities and apply damping. Initialize the body state.
    for (int i = 0; i < _bodyCount; ++i) {
      final Body b = _bodies[i];
      final Sweep bm_sweep = b._sweep;
      final Vector2 c = bm_sweep.c;
      double a = bm_sweep.a;
      final Vector2 v = b._linearVelocity;
      double w = b._angularVelocity;

      // Store positions for continuous collision.
      bm_sweep.c0.setFrom(bm_sweep.c);
      bm_sweep.a0 = bm_sweep.a;

      if (b._bodyType == BodyType.DYNAMIC) {
        // Integrate velocities.
        v.x += h * (b._gravityScale * gravity.x + b._invMass * b._force.x);
        v.y += h * (b._gravityScale * gravity.y + b._invMass * b._force.y);
        w += h * b._invI * b._torque;

        // Apply damping.
        // ODE: dv/dt + c * v = 0
        // Solution: v(t) = v0 * exp(-c * t)
        // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v *
        // exp(-c * dt)
        // v2 = exp(-c * dt) * v1
        // Pade approximation:
        // v2 = v1 * 1 / (1 + c * dt)
        v.x *= 1.0 / (1.0 + h * b._linearDamping);
        v.y *= 1.0 / (1.0 + h * b._linearDamping);
        w *= 1.0 / (1.0 + h * b.angularDamping);
      }

      _positions[i].c.x = c.x;
      _positions[i].c.y = c.y;
      _positions[i].a = a;
      _velocities[i].v.x = v.x;
      _velocities[i].v.y = v.y;
      _velocities[i].w = w;
    }

    // Solver data
    _solverData.step = step;
    _solverData.positions = _positions;
    _solverData.velocities = _velocities;

    // Initialize velocity constraints.
    _solverDef.step = step;
    _solverDef.contacts = _contacts;
    _solverDef.count = _contactCount;
    _solverDef.positions = _positions;
    _solverDef.velocities = _velocities;

    _contactSolver.init(_solverDef);
    _contactSolver.initializeVelocityConstraints();

    if (step.warmStarting) {
      _contactSolver.warmStart();
    }

    for (int i = 0; i < _jointCount; ++i) {
      _joints[i].initVelocityConstraints(_solverData);
    }

    for (int i = 0; i < step.velocityIterations; ++i) {
      for (int j = 0; j < _jointCount; ++j) {
        _joints[j].solveVelocityConstraints(_solverData);
      }

      _contactSolver.solveVelocityConstraints();
    }

    // Store impulses for warm starting
    _contactSolver.storeImpulses();

    // Integrate positions
    for (int i = 0; i < _bodyCount; ++i) {
      final Vector2 c = _positions[i].c;
      double a = _positions[i].a;
      final Vector2 v = _velocities[i].v;
      double w = _velocities[i].w;

      // Check for large velocities
      double translationX = v.x * h;
      double translationY = v.y * h;

      if (translationX * translationX + translationY * translationY >
          Settings.maxTranslationSquared) {
        double ratio = Settings.maxTranslation /
            Math.sqrt(
                translationX * translationX + translationY * translationY);
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

      _positions[i].a = a;
      _velocities[i].w = w;
    }

    // Solve position constraints
    bool positionSolved = false;
    for (int i = 0; i < step.positionIterations; ++i) {
      bool contactsOkay = _contactSolver.solvePositionConstraints();

      bool jointsOkay = true;
      for (int j = 0; j < _jointCount; ++j) {
        bool jointOkay = _joints[j].solvePositionConstraints(_solverData);
        jointsOkay = jointsOkay && jointOkay;
      }

      if (contactsOkay && jointsOkay) {
        // Exit early if the position errors are small.
        positionSolved = true;
        break;
      }
    }

    // Copy state buffers back to the bodies
    for (int i = 0; i < _bodyCount; ++i) {
      Body body = _bodies[i];
      body._sweep.c.x = _positions[i].c.x;
      body._sweep.c.y = _positions[i].c.y;
      body._sweep.a = _positions[i].a;
      body._linearVelocity.x = _velocities[i].v.x;
      body._linearVelocity.y = _velocities[i].v.y;
      body._angularVelocity = _velocities[i].w;
      body.synchronizeTransform();
    }

    report(_contactSolver._velocityConstraints);

    if (allowSleep) {
      double minSleepTime = double.maxFinite;

      final double linTolSqr =
          Settings.linearSleepTolerance * Settings.linearSleepTolerance;
      final double angTolSqr =
          Settings.angularSleepTolerance * Settings.angularSleepTolerance;

      for (int i = 0; i < _bodyCount; ++i) {
        Body b = _bodies[i];
        if (b.getType() == BodyType.STATIC) {
          continue;
        }

        if ((b._flags & Body.AUTO_SLEEP_FLAG) == 0 ||
            b._angularVelocity * b._angularVelocity > angTolSqr ||
            b._linearVelocity.dot(b._linearVelocity) > linTolSqr) {
          b._sleepTime = 0.0;
          minSleepTime = 0.0;
        } else {
          b._sleepTime += h;
          minSleepTime = Math.min(minSleepTime, b._sleepTime);
        }
      }

      if (minSleepTime >= Settings.timeToSleep && positionSolved) {
        _bodies.forEach((b) => b.setAwake(false));
      }
    }
  }

  final ContactSolver _toiContactSolver = ContactSolver();
  final ContactSolverDef _toiSolverDef = ContactSolverDef();

  void solveTOI(TimeStep subStep, int toiIndexA, int toiIndexB) {
    assert(toiIndexA < _bodyCount);
    assert(toiIndexB < _bodyCount);

    // Initialize the body state.
    for (int i = 0; i < _bodyCount; ++i) {
      _positions[i].c.x = _bodies[i]._sweep.c.x;
      _positions[i].c.y = _bodies[i]._sweep.c.y;
      _positions[i].a = _bodies[i]._sweep.a;
      _velocities[i].v.x = _bodies[i]._linearVelocity.x;
      _velocities[i].v.y = _bodies[i]._linearVelocity.y;
      _velocities[i].w = _bodies[i]._angularVelocity;
    }

    _toiSolverDef.contacts = _contacts;
    _toiSolverDef.count = _contactCount;
    _toiSolverDef.step = subStep;
    _toiSolverDef.positions = _positions;
    _toiSolverDef.velocities = _velocities;
    _toiContactSolver.init(_toiSolverDef);

    // Solve position constraints.
    for (int i = 0; i < subStep.positionIterations; ++i) {
      bool contactsOkay =
          _toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
      if (contactsOkay) {
        break;
      }
    }

    // Leap of faith to new safe state.
    _bodies[toiIndexA]._sweep.c0.x = _positions[toiIndexA].c.x;
    _bodies[toiIndexA]._sweep.c0.y = _positions[toiIndexA].c.y;
    _bodies[toiIndexA]._sweep.a0 = _positions[toiIndexA].a;
    _bodies[toiIndexB]._sweep.c0.setFrom(_positions[toiIndexB].c);
    _bodies[toiIndexB]._sweep.a0 = _positions[toiIndexB].a;

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
    for (int i = 0; i < _bodyCount; ++i) {
      Vector2 c = _positions[i].c;
      double a = _positions[i].a;
      Vector2 v = _velocities[i].v;
      double w = _velocities[i].w;

      // Check for large velocities
      double translationX = v.x * h;
      double translationY = v.y * h;
      if (translationX * translationX + translationY * translationY >
          Settings.maxTranslationSquared) {
        double ratio = Settings.maxTranslation /
            Math.sqrt(
                translationX * translationX + translationY * translationY);
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

      _positions[i].c.x = c.x;
      _positions[i].c.y = c.y;
      _positions[i].a = a;
      _velocities[i].v.x = v.x;
      _velocities[i].v.y = v.y;
      _velocities[i].w = w;

      // Sync bodies
      Body body = _bodies[i];
      body._sweep.c.x = c.x;
      body._sweep.c.y = c.y;
      body._sweep.a = a;
      body._linearVelocity.x = v.x;
      body._linearVelocity.y = v.y;
      body._angularVelocity = w;
      body.synchronizeTransform();
    }

    report(_toiContactSolver._velocityConstraints);
  }

  void addBody(Body body) {
    assert(_bodyCount < _bodyCapacity);
    body._islandIndex = _bodyCount;
    _bodies[_bodyCount] = body;
    ++_bodyCount;
  }

  void addContact(Contact contact) {
    assert(_contactCount < _contactCapacity);
    _contacts[_contactCount++] = contact;
  }

  void addJoint(Joint joint) {
    assert(_jointCount < _jointCapacity);
    _joints[_jointCount++] = joint;
  }

  final ContactImpulse _impulse = ContactImpulse();

  void report(List<ContactVelocityConstraint> constraints) {
    if (_listener == null) {
      return;
    }

    for (int i = 0; i < _contactCount; ++i) {
      Contact c = _contacts[i];

      ContactVelocityConstraint vc = constraints[i];
      _impulse.count = vc.pointCount;
      for (int j = 0; j < vc.pointCount; ++j) {
        _impulse.normalImpulses[j] = vc.points[j].normalImpulse;
        _impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
      }

      _listener.postSolve(c, _impulse);
    }
  }
}
