part of box2d;

class ConstantVolumeJoint extends Joint {
  final List<Body> _bodies;
  Float64List _targetLengths;
  double _targetVolume = 0.0;

  List<Vector2> _normals;
  double _impulse = 0.0;

  World _world;

  List<DistanceJoint> _distanceJoints;

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
      double dist = (_bodies[i].worldCenter - _bodies[next].worldCenter).length;
      _targetLengths[i] = dist;
    }
    _targetVolume = getBodyArea();

    if (def.joints != null && def.joints.length != def.bodies.length) {
      throw "Incorrect joint definition.  Joints have to correspond to the _bodies";
    }
    if (def.joints == null) {
      final DistanceJointDef djd = DistanceJointDef();
      _distanceJoints = List<DistanceJoint>(_bodies.length);
      for (int i = 0; i < _targetLengths.length; ++i) {
        final int next = (i == _targetLengths.length - 1) ? 0 : i + 1;
        djd.frequencyHz = def.frequencyHz; // 20.0;
        djd.dampingRatio = def.dampingRatio; // 50.0;
        djd.collideConnected = def.collideConnected;
        djd.initialize(_bodies[i], _bodies[next], _bodies[i].worldCenter,
            _bodies[next].worldCenter);
        _distanceJoints[i] = _world.createJoint(djd) as DistanceJoint;
      }
    } else {
      _distanceJoints = def.joints.toList();
    }

    _normals = List<Vector2>(_bodies.length);
    for (int i = 0; i < _normals.length; ++i) {
      _normals[i] = Vector2.zero();
    }
  }

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
      area += positions[_bodies[i]._islandIndex].c.x *
              positions[_bodies[next]._islandIndex].c.y -
          positions[_bodies[next]._islandIndex].c.x *
              positions[_bodies[i]._islandIndex].c.y;
    }
    area *= .5;
    return area;
  }

  bool _constrainEdges(List<Position> positions) {
    double perimeter = 0.0;
    for (int i = 0; i < _bodies.length; ++i) {
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      double dx = positions[_bodies[next]._islandIndex].c.x -
          positions[_bodies[i]._islandIndex].c.x;
      double dy = positions[_bodies[next]._islandIndex].c.y -
          positions[_bodies[i]._islandIndex].c.y;
      double dist = Math.sqrt(dx * dx + dy * dy);
      if (dist < Settings.EPSILON) {
        dist = 1.0;
      }
      _normals[i].x = dy / dist;
      _normals[i].y = -dx / dist;
      perimeter += dist;
    }

    final Vector2 delta = Vector2.zero();

    double deltaArea = _targetVolume - getSolverArea(positions);
    double toExtrude = 0.5 * deltaArea / perimeter; // *relaxationFactor
    // double sumdeltax = 0.0f;
    bool done = true;
    for (int i = 0; i < _bodies.length; ++i) {
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      delta.setValues(toExtrude * (_normals[i].x + _normals[next].x),
          toExtrude * (_normals[i].y + _normals[next].y));
      double normSqrd = delta.length2;
      if (normSqrd >
          Settings.maxLinearCorrection * Settings.maxLinearCorrection) {
        delta.scale(Settings.maxLinearCorrection / Math.sqrt(normSqrd));
      }
      if (normSqrd > Settings.linearSlop * Settings.linearSlop) {
        done = false;
      }
      positions[_bodies[next]._islandIndex].c.x += delta.x;
      positions[_bodies[next]._islandIndex].c.y += delta.y;
    }

    return done;
  }

  void initVelocityConstraints(final SolverData step) {
    List<Velocity> velocities = step.velocities;
    List<Position> positions = step.positions;
    final List<Vector2> d = List<Vector2>(_bodies.length);

    for (int i = 0; i < _bodies.length; ++i) {
      final int prev = (i == 0) ? _bodies.length - 1 : i - 1;
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      d[i] = Vector2.copy(positions[_bodies[next]._islandIndex].c);
      d[i].sub(positions[_bodies[prev]._islandIndex].c);
    }

    if (step.step.warmStarting) {
      _impulse *= step.step.dtRatio;
      for (int i = 0; i < _bodies.length; ++i) {
        velocities[_bodies[i]._islandIndex].v.x +=
            _bodies[i]._invMass * d[i].y * .5 * _impulse;
        velocities[_bodies[i]._islandIndex].v.y +=
            _bodies[i]._invMass * -d[i].x * .5 * _impulse;
      }
    } else {
      _impulse = 0.0;
    }
  }

  bool solvePositionConstraints(SolverData step) {
    return _constrainEdges(step.positions);
  }

  void solveVelocityConstraints(final SolverData step) {
    double crossMassSum = 0.0;
    double dotMassSum = 0.0;

    List<Velocity> velocities = step.velocities;
    List<Position> positions = step.positions;
    final List<Vector2> d = List<Vector2>(_bodies.length);

    for (int i = 0; i < _bodies.length; ++i) {
      final int prev = (i == 0) ? _bodies.length - 1 : i - 1;
      final int next = (i == _bodies.length - 1) ? 0 : i + 1;
      d[i] = Vector2.copy(positions[_bodies[next]._islandIndex].c);
      d[i].sub(positions[_bodies[prev]._islandIndex].c);
      dotMassSum += (d[i].length2) / _bodies[i].mass;
      crossMassSum += velocities[_bodies[i]._islandIndex].v.cross(d[i]);
    }
    double lambda = -2.0 * crossMassSum / dotMassSum;
    _impulse += lambda;
    for (int i = 0; i < _bodies.length; ++i) {
      velocities[_bodies[i]._islandIndex].v.x +=
          _bodies[i]._invMass * d[i].y * .5 * lambda;
      velocities[_bodies[i]._islandIndex].v.y +=
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
  Vector2 getReactionForce(double inv_dt) => Vector2.zero();

  /// No-op
  @override
  double getReactionTorque(double inv_dt) => 0.0;
}
