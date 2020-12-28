part of forge2d;

class ConstantVolumeJoint extends Joint {
  final List<Body> _bodies;
  Float64List _targetLengths;
  double _targetVolume = 0.0;

  List<Vector2> _normals;
  double _impulse = 0.0;

  World _world;

  final List<DistanceJoint> _distanceJoints = [];

  List<Body> getBodies() {
    return _bodies;
  }

  List<DistanceJoint> getJoints() {
    return _distanceJoints;
  }

  void inflate(double factor) {
    _targetVolume *= factor;
  }

  ConstantVolumeJoint(World argWorld, ConstantVolumeJointDef def)
      : _bodies = def.bodies.toList(growable: false),
        super(def) {
    _world = argWorld;
    if (def.bodies.length <= 2) {
      throw "You cannot create a constant volume joint with less than three _bodies.";
    }

    _targetLengths = Float64List(_bodies.length);
    for (int i = 0; i < _targetLengths.length; ++i) {
      final int next = (i == _targetLengths.length - 1) ? 0 : i + 1;
      final double dist =
          (_bodies[i].worldCenter - _bodies[next].worldCenter).length;
      _targetLengths[i] = dist;
    }
    _targetVolume = getBodyArea();

    if (def.joints.isNotEmpty && def.joints.length != def.bodies.length) {
      print(def.joints.length);
      print(def.bodies.length);
      throw "Incorrect joint definition. Joints have to correspond to the _bodies";
    }
    if (def.joints.isEmpty) {
      final DistanceJointDef distanceJointDef = DistanceJointDef();
      _distanceJoints.addAll(List<DistanceJoint>.generate(
        _bodies.length,
        (i) {
          final int next = (i == _bodies.length - 1) ? 0 : i + 1;
          distanceJointDef.frequencyHz = def.frequencyHz; // 20.0;
          distanceJointDef.dampingRatio = def.dampingRatio; // 50.0;
          distanceJointDef.collideConnected = def.collideConnected;
          distanceJointDef.initialize(
            _bodies[i],
            _bodies[next],
            _bodies[i].worldCenter,
            _bodies[next].worldCenter,
          );
          return _world.createJoint(distanceJointDef) as DistanceJoint;
        },
      ));
    } else {
      _distanceJoints.clear();
      _distanceJoints.addAll(def.joints);
    }

    _normals = List<Vector2>.generate(_bodies.length, (_) => Vector2.zero());
  }

  @override
  void destructor() {
    for (int i = 0; i < _distanceJoints.length; ++i) {
      _world.destroyJoint(_distanceJoints[i]);
    }
  }

  double getBodyArea() {
    double area = 0.0;
    for (int i = 0; i < _bodies.length; ++i) {
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      area += _bodies[i].worldCenter.x * _bodies[next].worldCenter.y -
          _bodies[next].worldCenter.x * _bodies[i].worldCenter.y;
    }
    area *= .5;
    return area;
  }

  double getSolverArea(List<Position> positions) {
    double area = 0.0;
    for (int i = 0; i < _bodies.length; ++i) {
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      area += positions[_bodies[i].islandIndex].c.x *
              positions[_bodies[next].islandIndex].c.y -
          positions[_bodies[next].islandIndex].c.x *
              positions[_bodies[i].islandIndex].c.y;
    }
    area *= .5;
    return area;
  }

  bool _constrainEdges(List<Position> positions) {
    double perimeter = 0.0;
    for (int i = 0; i < _bodies.length; ++i) {
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      final double dx = positions[_bodies[next].islandIndex].c.x -
          positions[_bodies[i].islandIndex].c.x;
      final double dy = positions[_bodies[next].islandIndex].c.y -
          positions[_bodies[i].islandIndex].c.y;
      double dist = math.sqrt(dx * dx + dy * dy);
      if (dist < settings.EPSILON) {
        dist = 1.0;
      }
      _normals[i].x = dy / dist;
      _normals[i].y = -dx / dist;
      perimeter += dist;
    }

    final Vector2 delta = Vector2.zero();

    final double deltaArea = _targetVolume - getSolverArea(positions);
    final double toExtrude = 0.5 * deltaArea / perimeter; // *relaxationFactor
    // double sumdeltax = 0.0f;
    bool done = true;
    for (int i = 0; i < _bodies.length; ++i) {
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      delta.setValues(toExtrude * (_normals[i].x + _normals[next].x),
          toExtrude * (_normals[i].y + _normals[next].y));
      final double normSqrd = delta.length2;
      if (normSqrd >
          settings.maxLinearCorrection * settings.maxLinearCorrection) {
        delta.scale(settings.maxLinearCorrection / math.sqrt(normSqrd));
      }
      if (normSqrd > settings.linearSlop * settings.linearSlop) {
        done = false;
      }
      positions[_bodies[next].islandIndex].c.x += delta.x;
      positions[_bodies[next].islandIndex].c.y += delta.y;
    }

    return done;
  }

  @override
  void initVelocityConstraints(final SolverData step) {
    final List<Velocity> velocities = step.velocities;
    final List<Position> positions = step.positions;
    final List<Vector2> d = List<Vector2>.generate(
      _bodies.length,
      (i) {
        final int prev = (i == 0) ? _bodies.length - 1 : i - 1;
        final int next = (i == _bodies.length - 1) ? 0 : i + 1;
        return positions[_bodies[next].islandIndex].c -
            positions[_bodies[prev].islandIndex].c;
      },
    );

    if (step.step.warmStarting) {
      _impulse *= step.step.dtRatio;
      for (int i = 0; i < _bodies.length; ++i) {
        velocities[_bodies[i].islandIndex].v.x +=
            _bodies[i]._invMass * d[i].y * .5 * _impulse;
        velocities[_bodies[i].islandIndex].v.y +=
            _bodies[i]._invMass * -d[i].x * .5 * _impulse;
      }
    } else {
      _impulse = 0.0;
    }
  }

  @override
  bool solvePositionConstraints(SolverData step) {
    return _constrainEdges(step.positions);
  }

  @override
  void solveVelocityConstraints(final SolverData step) {
    double crossMassSum = 0.0;
    double dotMassSum = 0.0;

    final List<Velocity> velocities = step.velocities;
    final List<Position> positions = step.positions;
    final List<Vector2> d = List<Vector2>.generate(
      _bodies.length,
      (i) {
        final int prev = (i == 0) ? _bodies.length - 1 : i - 1;
        final int next = (i == _bodies.length - 1) ? 0 : i + 1;
        final v = positions[_bodies[next].islandIndex].c -
            positions[_bodies[prev].islandIndex].c;
        dotMassSum += (v.length2) / _bodies[i].mass;
        crossMassSum += velocities[_bodies[i].islandIndex].v.cross(v);
        return v;
      },
    );
    final double lambda = -2.0 * crossMassSum / dotMassSum;
    _impulse += lambda;
    for (int i = 0; i < _bodies.length; ++i) {
      velocities[_bodies[i].islandIndex].v.x +=
          _bodies[i]._invMass * d[i].y * .5 * lambda;
      velocities[_bodies[i].islandIndex].v.y +=
          _bodies[i]._invMass * -d[i].x * .5 * lambda;
    }
  }

  /// No-op
  @override
  Vector2 getAnchorA() => Vector2.zero();

  /// No-op
  @override
  Vector2 getAnchorB() => Vector2.zero();

  /// No-op
  @override
  Vector2 getReactionForce(double invDt) => Vector2.zero();

  /// No-op
  @override
  double getReactionTorque(double invDt) => 0.0;
}
