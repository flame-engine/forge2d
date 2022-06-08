import 'dart:math';

import '../../forge2d.dart';
import '../settings.dart' as settings;

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

// TODO(spydon): Refactor this later
class BodyMeta {
  final Body body;
  final Velocity velocity = Velocity();
  final Position position = Position();

  BodyMeta(this.body);
}

/// This is an internal class.
class Island {
  ContactListener? _listener;

  final List<BodyMeta> bodies = <BodyMeta>[];
  late List<Contact> _contacts;
  late List<Joint> _joints;

  // TODO(spydon): Make this as efficient as it used to be
  List<Position> get _positions {
    return bodies.map((BodyMeta bodyMeta) => bodyMeta.position).toList();
  }

  List<Velocity> get _velocities {
    return bodies.map((BodyMeta bodyMeta) => bodyMeta.velocity).toList();
  }

  Island() {
    _contacts = <Contact>[];
    _joints = <Joint>[];
  }

  void init(ContactListener? listener) {
    _listener = listener;
  }

  void clear() {
    bodies.clear();
    _contacts.clear();
    _joints.clear();
  }

  final ContactSolver _contactSolver = ContactSolver();
  final SolverData _solverData = SolverData();
  final ContactSolverDef _solverDef = ContactSolverDef();

  void solve(Profile profile, TimeStep step, Vector2 gravity, bool allowSleep) {
    final dt = step.dt;

    // Integrate velocities and apply damping. Initialize the body state.
    for (final bodyMeta in bodies) {
      final b = bodyMeta.body;
      final bmSweep = b.sweep;
      final c = bmSweep.c;
      final a = bmSweep.a;
      final v = b.linearVelocity;
      var w = b.angularVelocity;

      // Store positions for continuous collision.
      bmSweep.c0.setFrom(bmSweep.c);
      bmSweep.a0 = bmSweep.a;

      if (b.bodyType == BodyType.dynamic) {
        // Integrate velocities.
        final bodyGravity = b.gravityOverride ?? gravity;
        v.x += dt *
            ((b.gravityScale?.x ?? 1) * bodyGravity.x +
                b.inverseMass * b.force.x);
        v.y += dt *
            ((b.gravityScale?.y ?? 1) * bodyGravity.y +
                b.inverseMass * b.force.y);
        w += dt * b.inverseInertia * b.torque;

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

    for (final joint in _joints) {
      joint.initVelocityConstraints(_solverData);
    }

    for (var i = 0; i < step.velocityIterations; ++i) {
      for (final joint in _joints) {
        joint.solveVelocityConstraints(_solverData);
      }

      _contactSolver.solveVelocityConstraints();
    }

    // Store impulses for warm starting
    _contactSolver.storeImpulses();

    // Integrate positions
    for (final bodyMeta in bodies) {
      final c = bodyMeta.position.c;
      var a = bodyMeta.position.a;
      final v = bodyMeta.velocity.v;
      var w = bodyMeta.velocity.w;

      // Check for large velocities
      final translationX = v.x * dt;
      final translationY = v.y * dt;

      if (translationX * translationX + translationY * translationY >
          settings.maxTranslationSquared) {
        final ratio = settings.maxTranslation /
            sqrt(translationX * translationX + translationY * translationY);
        v.x *= ratio;
        v.y *= ratio;
      }

      final rotation = dt * w;
      if (rotation * rotation > settings.maxRotationSquared) {
        final ratio = settings.maxRotation / rotation.abs();
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
    var positionSolved = false;
    for (var i = 0; i < step.positionIterations; ++i) {
      final contactsOkay = _contactSolver.solvePositionConstraints();

      var jointsOkay = true;
      for (final joint in _joints) {
        final jointOkay = joint.solvePositionConstraints(_solverData);
        jointsOkay = jointsOkay && jointOkay;
      }

      if (contactsOkay && jointsOkay) {
        // Exit early if the position errors are small.
        positionSolved = true;
        break;
      }
    }

    // Copy state buffers back to the bodies
    for (final bodyMeta in bodies) {
      final body = bodyMeta.body;
      body.sweep.c.x = bodyMeta.position.c.x;
      body.sweep.c.y = bodyMeta.position.c.y;
      body.sweep.a = bodyMeta.position.a;
      body.linearVelocity.x = bodyMeta.velocity.v.x;
      body.linearVelocity.y = bodyMeta.velocity.v.y;
      body.angularVelocity = bodyMeta.velocity.w;
      body.synchronizeTransform();
    }

    reportVelocityConstraints();

    if (allowSleep) {
      var minSleepTime = double.maxFinite;

      final linTolSqr =
          settings.linearSleepTolerance * settings.linearSleepTolerance;
      final angTolSqr =
          settings.angularSleepTolerance * settings.angularSleepTolerance;

      for (final bodyMeta in bodies) {
        final b = bodyMeta.body;
        if (b.bodyType == BodyType.static) {
          continue;
        }

        if ((b.flags & Body.autoSleepFlag) == 0 ||
            b.angularVelocity * b.angularVelocity > angTolSqr ||
            b.linearVelocity.dot(b.linearVelocity) > linTolSqr) {
          b.sleepTime = 0.0;
          minSleepTime = 0.0;
        } else {
          b.sleepTime += dt;
          minSleepTime = min(minSleepTime, b.sleepTime);
        }
      }

      if (minSleepTime >= settings.timeToSleep && positionSolved) {
        bodies.forEach((b) => b.body.setAwake(false));
      }
    }
  }

  final ContactSolver _toiContactSolver = ContactSolver();
  final ContactSolverDef _toiSolverDef = ContactSolverDef();

  void solveTOI(TimeStep subStep, int toiIndexA, int toiIndexB) {
    assert(toiIndexA < bodies.length);
    assert(toiIndexB < bodies.length);

    // Initialize the body state.
    for (final bodyMeta in bodies) {
      final body = bodyMeta.body;
      bodyMeta.position.c.x = body.sweep.c.x;
      bodyMeta.position.c.y = body.sweep.c.y;
      bodyMeta.position.a = body.sweep.a;
      bodyMeta.velocity.v.x = body.linearVelocity.x;
      bodyMeta.velocity.v.y = body.linearVelocity.y;
      bodyMeta.velocity.w = body.angularVelocity;
    }

    // TODO(spydon): Is this correct, since it is no longer a fixed list?
    _toiSolverDef.contacts = _contacts;
    _toiSolverDef.step = subStep;
    _toiSolverDef.positions = _positions;
    _toiSolverDef.velocities = _velocities;
    _toiContactSolver.init(_toiSolverDef);

    // Solve position constraints.
    for (var i = 0; i < subStep.positionIterations; ++i) {
      final contactsOkay =
          _toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
      if (contactsOkay) {
        break;
      }
    }

    // Leap of faith to new safe state.
    bodies[toiIndexA].body.sweep.c0.x = _positions[toiIndexA].c.x;
    bodies[toiIndexA].body.sweep.c0.y = _positions[toiIndexA].c.y;
    bodies[toiIndexA].body.sweep.a0 = _positions[toiIndexA].a;
    bodies[toiIndexB].body.sweep.c0.setFrom(_positions[toiIndexB].c);
    bodies[toiIndexB].body.sweep.a0 = _positions[toiIndexB].a;

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.
    _toiContactSolver.initializeVelocityConstraints();

    // Solve velocity constraints.
    for (var i = 0; i < subStep.velocityIterations; ++i) {
      _toiContactSolver.solveVelocityConstraints();
    }

    // Don't store the TOI contact forces for warm starting
    // because they can be quite large.

    final dt = subStep.dt;

    // Integrate positions
    for (final bodyMeta in bodies) {
      final position = bodyMeta.position;
      final velocity = bodyMeta.velocity;
      final c = position.c;
      var a = position.a;
      final v = velocity.v;
      var w = velocity.w;

      // Check for large velocities
      final translationX = v.x * dt;
      final translationY = v.y * dt;
      if (translationX * translationX + translationY * translationY >
          settings.maxTranslationSquared) {
        final ratio = settings.maxTranslation /
            sqrt(translationX * translationX + translationY * translationY);
        v.scale(ratio);
      }

      final rotation = dt * w;
      if (rotation * rotation > settings.maxRotationSquared) {
        final ratio = settings.maxRotation / rotation.abs();
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
      final body = bodyMeta.body;
      body.sweep.c.x = c.x;
      body.sweep.c.y = c.y;
      body.sweep.a = a;
      body.linearVelocity.x = v.x;
      body.linearVelocity.y = v.y;
      body.angularVelocity = w;
      body.synchronizeTransform();
    }

    reportVelocityConstraints();
  }

  void addBody(Body body) {
    body.islandIndex = bodies.length;
    bodies.add(BodyMeta(body));
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

    for (final contact in _contacts) {
      final vc = contact.velocityConstraint;
      _impulse.count = vc.pointCount;
      for (var j = 0; j < vc.pointCount; ++j) {
        _impulse.normalImpulses[j] = vc.points[j].normalImpulse;
        _impulse.tangentImpulses[j] = vc.points[j].tangentImpulse;
      }

      _listener!.postSolve(contact, _impulse);
    }
  }
}
