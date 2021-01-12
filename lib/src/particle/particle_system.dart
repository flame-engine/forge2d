part of forge2d;

/// Connection between two particles
class PsPair {
  final Particle particleA;
  final Particle particleB;
  int flags = 0;
  double strength = 0.0;
  double distance = 0.0;

  PsPair(this.particleA, this.particleB);
}

/// Connection between three particles
class PsTriad {
  final Particle particleA;
  final Particle particleB;
  final Particle particleC;
  int flags = 0;
  double strength = 0.0;
  // TODO.spydon: Give these better names
  final Vector2 pa = Vector2.zero(), pb = Vector2.zero(), pc = Vector2.zero();
  double ka = 0.0, kb = 0.0, kc = 0.0, s = 0.0;

  PsTriad(this.particleA, this.particleB, this.particleC);
}

/// Used for detecting particle contacts
class PsProxy implements Comparable<PsProxy> {
  final Particle particle;
  // TODO.spydon what is a tag?
  int tag = 0;

  PsProxy(this.particle);

  @override
  int compareTo(PsProxy o) => (tag - o.tag) < 0 ? -1 : (o.tag == tag ? 0 : 1);
  bool equals(PsProxy obj) => tag == obj.tag;
}

class NewIndices {
  int start = 0, mid = 0, end = 0;

  int getIndex(final int i) {
    if (i < start) {
      return i;
    } else if (i < mid) {
      return i + end - mid;
    } else if (i < end) {
      return i + start - mid;
    } else {
      return i;
    }
  }
}

class ParticleSystem {
  /// All particle types that require creating pairs
  static const int k_pairFlags = ParticleType.springParticle;

  /// All particle types that require creating triads
  static const int k_triadFlags = ParticleType.elasticParticle;

  /// All particle types that require computing depth
  static const int k_noPressureFlags = ParticleType.powderParticle;

  static const int xTruncBits = 12;
  static const int yTruncBits = 12;
  static const int tagBits = 8 * 4 - 1 /* sizeof(int) */;
  static const int yOffset = 1 << (yTruncBits - 1);
  static const int yShift = tagBits - yTruncBits;
  static const int xShift = tagBits - yTruncBits - xTruncBits;
  static const int xScale = 1 << xShift;
  static const int xOffset = xScale * (1 << (xTruncBits - 1));
  static const int xMask = (1 << xTruncBits) - 1;
  static const int yMask = (1 << yTruncBits) - 1;

  static int computeTag(double x, double y) {
    return ((y + yOffset).toInt() << yShift) + ((xScale * x).toInt() + xOffset);
  }

  static int computeRelativeTag(int tag, int x, int y) {
    return tag + (y << yShift) + (x << xShift);
  }

  int timestamp = 0;
  int allParticleFlags = 0;
  int allGroupFlags = 0;
  double density = 1.0;
  double inverseDensity = 1.0;
  double gravityScale = 1.0;
  double particleDiameter = 1.0;
  double inverseDiameter = 1.0;
  double squaredDiameter = 1.0;

  int get particleCount => particles.length;
  int get particleGroupCount => groupBuffer.length;

  final List<Particle> _particles = [];
  UnmodifiableListView<Particle> get particles {
    return UnmodifiableListView<Particle>(_particles);
  }

  final Set<ParticleGroup> groupBuffer = {};
  final List<PsProxy> proxyBuffer = [];
  final List<ParticleContact> contactBuffer = [];
  final List<ParticleBodyContact> bodyContactBuffer = [];
  final List<PsPair> pairBuffer = [];
  final List<PsTriad> triadBuffer = [];

  double pressureStrength;
  double dampingStrength;
  double elasticStrength;
  double springStrength;
  double viscousStrength;
  double surfaceTensionStrengthA;
  double surfaceTensionStrengthB;
  double powderStrength;
  double ejectionStrength;
  double colorMixingStrength;

  final World world;

  ParticleSystem(this.world) {
    pressureStrength = 0.05;
    dampingStrength = 1.0;
    elasticStrength = 0.25;
    springStrength = 0.25;
    viscousStrength = 0.25;
    surfaceTensionStrengthA = 0.1;
    surfaceTensionStrengthB = 0.2;
    powderStrength = 0.5;
    ejectionStrength = 0.5;
    colorMixingStrength = 0.5;
  }

  void createParticle(Particle particle) {
    particle.group.add(particle);
    groupBuffer.add(particle.group);
    proxyBuffer.add(PsProxy(particle));
    _particles.add(particle);
  }

  void destroyParticle(Particle particle, bool callDestructionListener) {
    int flags = ParticleType.zombieParticle;
    if (callDestructionListener) {
      flags |= ParticleType.destructionListener;
    }
    particle.flags |= flags;
  }

  final AABB _temp = AABB();

  /// Destroy particles inside a shape. In addition, this function immediately
  /// destroys particles in the shape in contrast to DestroyParticle() which
  /// defers the destruction until the next simulation step.
  int destroyParticlesInShape(
    Shape shape,
    Transform xf, {
    bool callDestructionListener = false,
  }) {
    final callback = DestroyParticlesInShapeCallback(
      this,
      shape,
      xf,
      callDestructionListener: callDestructionListener,
    );
    shape.computeAABB(_temp, xf, 0);
    world.queryAABBParticle(callback, _temp);
    return callback.destroyed;
  }

  void destroyParticlesInGroup(
    ParticleGroup group, {
    bool callDestructionListener = false,
  }) {
    group.particles.forEach((p) => destroyParticle(p, callDestructionListener));
  }

  final AABB _temp2 = AABB();
  final Vector2 _tempVec = Vector2.zero();
  final Transform _tempTransform = Transform.zero();
  final Transform _tempTransform2 = Transform.zero();
  final CreateParticleGroupCallback _createParticleGroupCallback =
      CreateParticleGroupCallback();

  ParticleGroup createParticleGroup(ParticleGroupDef groupDef) {
    final double stride = getParticleStride();
    final Transform identity = _tempTransform..setIdentity();
    final Transform transform = _tempTransform2..setIdentity();

    final ParticleGroup group = ParticleGroup(this)
      ..groupFlags = groupDef.groupFlags
      ..strength = groupDef.strength
      ..userData = groupDef.userData
      ..transform.set(transform)
      ..destroyAutomatically = groupDef.destroyAutomatically;

    if (groupDef.shape != null) {
      final Particle seedParticle = Particle(this, group: group)
        ..flags = groupDef.flags
        ..color = groupDef.color
        ..userData = groupDef.userData;
      final Shape shape = groupDef.shape;
      transform.setVec2Angle(groupDef.position, groupDef.angle);
      final AABB aabb = _temp;
      final int childCount = shape.getChildCount();
      for (int childIndex = 0; childIndex < childCount; childIndex++) {
        if (childIndex == 0) {
          shape.computeAABB(aabb, identity, childIndex);
        } else {
          final AABB childAABB = _temp2;
          shape.computeAABB(childAABB, identity, childIndex);
          aabb.combine(childAABB);
        }
      }
      final double upperBoundY = aabb.upperBound.y;
      final double upperBoundX = aabb.upperBound.x;
      for (double y = (aabb.lowerBound.y / stride).floor() * stride;
          y < upperBoundY;
          y += stride) {
        for (double x = (aabb.lowerBound.x / stride).floor() * stride;
            x < upperBoundX;
            x += stride) {
          final Vector2 p = _tempVec..setValues(x, y);
          if (shape.testPoint(identity, p)) {
            p.setFrom(Transform.mulVec2(transform, p));
            final particle = seedParticle.clone();
            p.sub(groupDef.position);
            particle.position.setFrom(p);
            p.scaleOrthogonalInto(
              groupDef.angularVelocity,
              particle.velocity,
            );
            particle.velocity.add(groupDef.linearVelocity);
            createParticle(particle);
          }
        }
      }
      groupBuffer.add(group);
    }

    updateContacts(true);
    if ((groupDef.flags & k_pairFlags) != 0) {
      for (ParticleContact contact in contactBuffer) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        if (group.particles.contains(particleA) &&
            group.particles.contains(particleB)) {
          final PsPair pair = PsPair(particleA, particleB)
            ..flags = contact.flags
            ..strength = groupDef.strength
            ..distance = particleA.position.distanceTo(particleB.position);
          pairBuffer.add(pair);
        }
      }
    }
    if ((groupDef.flags & k_triadFlags) != 0) {
      final VoronoiDiagram diagram = VoronoiDiagram();
      print("group: ${group.particles.length}");
      for (Particle particle in group.particles) {
        diagram.addGenerator(particle.position, particle);
      }
      diagram.generate(stride / 2);
      _createParticleGroupCallback.system = this;
      _createParticleGroupCallback.def = groupDef;
      diagram.getNodes(_createParticleGroupCallback);
    }
    if ((groupDef.groupFlags & ParticleGroupType.solidParticleGroup) != 0) {
      computeDepthForGroup(group);
    }

    return group;
  }

  void joinParticleGroups(ParticleGroup groupA, ParticleGroup groupB) {
    int particleFlags = 0;
    final joinedParticles = groupA.particles + groupB.particles;
    for (Particle particle in joinedParticles) {
      particleFlags |= particle.flags;
    }

    updateContacts(true);
    if ((particleFlags & k_pairFlags) != 0) {
      for (ParticleContact contact in contactBuffer) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        if (groupA.particles.contains(particleA) &&
            groupB.particles.contains(particleB)) {
          final PsPair pair = PsPair(particleA, particleB)
            ..flags = contact.flags
            ..strength = math.min(groupA.strength, groupB.strength)
            ..distance = particleA.position.distanceTo(particleB.position);
          pairBuffer.add(pair);
        }
      }
    }
    if ((particleFlags & k_triadFlags) != 0) {
      final VoronoiDiagram diagram = VoronoiDiagram();
      for (Particle particle in joinedParticles) {
        if ((particle.flags & ParticleType.zombieParticle) == 0) {
          diagram.addGenerator(particle.position, particle);
        }
      }
      diagram.generate(getParticleStride() / 2);
      final JoinParticleGroupsCallback callback = JoinParticleGroupsCallback();
      callback.system = this;
      callback.groupA = groupA;
      callback.groupB = groupB;
      diagram.getNodes(callback);
    }

    for (Particle particle in groupB.particles) {
      groupA.add(particle);
      particle.group = groupA;
    }
    final int groupFlags = groupA.groupFlags | groupB.groupFlags;
    groupA.groupFlags = groupFlags;
    // Remove group b, since all its particles are in group a now
    world.particleDestructionListener?.sayGoodbyeParticleGroup(groupB);
    groupBuffer.remove(groupB);

    if ((groupFlags & ParticleGroupType.solidParticleGroup) != 0) {
      computeDepthForGroup(groupA);
    }
  }

  void computeDepthForGroup(ParticleGroup group) {
    for (Particle particle in group.particles) {
      particle.accumulation = 0.0;
    }
    for (ParticleContact contact in contactBuffer) {
      final Particle particleA = contact.particleA;
      final Particle particleB = contact.particleB;
      if (group.contains(particleA) && group.contains(particleB)) {
        final double w = contact.weight;
        particleA.accumulation += w;
        particleB.accumulation += w;
      }
    }
    for (Particle particle in group.particles) {
      particle.depth = particle.accumulation < 0.8 ? 0.0 : double.maxFinite;
    }
    for (int t = 0; t < group.particles.length; t++) {
      bool updated = false;
      for (ParticleContact contact in contactBuffer) {
        final Particle particleA = contact.particleA;
        final Particle particleB = contact.particleB;
        if (group.contains(particleA) && group.contains(particleB)) {
          final double r = 1 - contact.weight;
          if (particleA.depth > particleB.depth + r) {
            particleA.depth = particleB.depth + r;
            updated = true;
          }
          if (particleB.depth > particleA.depth + r) {
            particleB.depth = particleA.depth + r;
            updated = true;
          }
        }
      }
      if (!updated) {
        break;
      }
    }
    for (Particle particle in group.particles) {
      if (particle.depth < double.maxFinite) {
        // TODO.spydon: it will always go into this case?
        particle.depth *= particleDiameter;
      } else {
        particle.depth = 0.0;
      }
    }
  }

  void addContact(Particle particleA, Particle particleB) {
    assert(
      particleA != particleB,
      "The particles in contact can't be the same",
    );
    final Vector2 pa = particleA.position;
    final Vector2 pb = particleB.position;
    final double dx = pb.x - pa.x;
    final double dy = pb.y - pa.y;
    final double d2 = dx * dx + dy * dy;
    if (d2 < squaredDiameter) {
      final double invD = d2 != 0 ? math.sqrt(1 / d2) : double.maxFinite;
      final ParticleContact contact = ParticleContact(particleA, particleB)
        ..flags = particleA.flags | particleB.flags
        ..weight = 1 - d2 * invD * inverseDiameter
        ..normal.x = invD * dx
        ..normal.y = invD * dy;
      contactBuffer.add(contact);
    }
  }

  void updateContacts(bool exceptZombie) {
    for (PsProxy proxy in proxyBuffer) {
      final Vector2 pos = proxy.particle.position;
      proxy.tag = computeTag(inverseDiameter * pos.x, inverseDiameter * pos.y);
    }
    proxyBuffer.sort();
    contactBuffer.clear();
    int cIndex = 0;
    for (int i = 0; i < proxyBuffer.length; i++) {
      final PsProxy proxyA = proxyBuffer[i];
      final int rightTag = computeRelativeTag(proxyA.tag, 1, 0);
      for (int j = i + 1; j < proxyBuffer.length; j++) {
        final PsProxy proxyB = proxyBuffer[j];
        if (rightTag < proxyB.tag) {
          break;
        }
        addContact(proxyA.particle, proxyB.particle);
      }
      final int bottomLeftTag = computeRelativeTag(proxyA.tag, -1, 1);
      for (; cIndex < proxyBuffer.length; cIndex++) {
        final PsProxy c = proxyBuffer[cIndex];
        if (bottomLeftTag <= c.tag) {
          break;
        }
      }
      final int bottomRightTag = computeRelativeTag(proxyA.tag, 1, 1);

      for (int bIndex = cIndex; bIndex < proxyBuffer.length; bIndex++) {
        final PsProxy proxyB = proxyBuffer[bIndex];
        if (bottomRightTag < proxyB.tag) {
          break;
        }
        addContact(proxyA.particle, proxyB.particle);
      }
    }
    if (exceptZombie) {
      int j = contactBuffer.length;
      for (int i = 0; i < j; i++) {
        if ((contactBuffer[i].flags & ParticleType.zombieParticle) != 0) {
          --j;
          final ParticleContact temp = contactBuffer[j];
          contactBuffer[j] = contactBuffer[i];
          contactBuffer[i] = temp;
          --i;
        }
      }
    }
  }

  final UpdateBodyContactsCallback _ubccallback = UpdateBodyContactsCallback();

  void updateBodyContacts() {
    final AABB aabb = _temp;
    aabb.lowerBound.x = double.maxFinite;
    aabb.lowerBound.y = double.maxFinite;
    aabb.upperBound.x = -double.maxFinite;
    aabb.upperBound.y = -double.maxFinite;
    for (Particle particle in _particles) {
      final position = particle.position;
      Vector2.min(aabb.lowerBound, position, aabb.lowerBound);
      Vector2.max(aabb.upperBound, position, aabb.upperBound);
    }
    aabb.lowerBound.x -= particleDiameter;
    aabb.lowerBound.y -= particleDiameter;
    aabb.upperBound.x += particleDiameter;
    aabb.upperBound.y += particleDiameter;

    _ubccallback.system = this;
    world.queryAABB(_ubccallback, aabb);
  }

  final SolveCollisionCallback _solveCollisionCallback =
      SolveCollisionCallback();

  void solveCollision(TimeStep step) {
    final AABB aabb = _temp;
    final Vector2 lowerBound = aabb.lowerBound;
    final Vector2 upperBound = aabb.upperBound;
    lowerBound.x = double.maxFinite;
    lowerBound.y = double.maxFinite;
    upperBound.x = -double.maxFinite;
    upperBound.y = -double.maxFinite;
    for (Particle particle in _particles) {
      final Vector2 v = particle.velocity;
      final Vector2 p1 = particle.position;
      final double p2x = p1.x + step.dt * v.x;
      final double p2y = p1.y + step.dt * v.y;
      final double bx = p1.x < p2x ? p1.x : p2x;
      final double by = p1.y < p2y ? p1.y : p2y;
      lowerBound.x = lowerBound.x < bx ? lowerBound.x : bx;
      lowerBound.y = lowerBound.y < by ? lowerBound.y : by;
      final double b1x = p1.x > p2x ? p1.x : p2x;
      final double b1y = p1.y > p2y ? p1.y : p2y;
      upperBound.x = upperBound.x > b1x ? upperBound.x : b1x;
      upperBound.y = upperBound.y > b1y ? upperBound.y : b1y;
    }
    _solveCollisionCallback.step = step;
    _solveCollisionCallback.system = this;
    world.queryAABB(_solveCollisionCallback, aabb);
  }

  void solve(TimeStep step) {
    ++timestamp;
    if (_particles.isEmpty) {
      return;
    }
    allParticleFlags = 0;
    for (Particle particle in _particles) {
      allParticleFlags |= particle.flags;
    }
    if ((allParticleFlags & ParticleType.zombieParticle) != 0) {
      solveZombie();
    }
    if (_particles.isEmpty) {
      return;
    }
    allGroupFlags = 0;
    for (ParticleGroup group in groupBuffer) {
      allGroupFlags |= group.groupFlags;
    }
    final double gravityx = step.dt * gravityScale * world.getGravity().x;
    final double gravityy = step.dt * gravityScale * world.getGravity().y;
    final double criticalVelocitySquared = getCriticalVelocitySquared(step);
    for (Particle particle in _particles) {
      final Vector2 v = particle.velocity;
      v.x += gravityx;
      v.y += gravityy;
      final double v2 = v.x * v.x + v.y * v.y;
      if (v2 > criticalVelocitySquared) {
        final double a = v2 == 0
            ? double.maxFinite
            : math.sqrt(criticalVelocitySquared / v2);
        v.x *= a;
        v.y *= a;
      }
    }
    solveCollision(step);
    if ((allGroupFlags & ParticleGroupType.rigidParticleGroup) != 0) {
      solveRigid(step);
    }
    if ((allParticleFlags & ParticleType.wallParticle) != 0) {
      solveWall(step);
    }
    for (Particle particle in _particles) {
      particle.position.setFrom(
        particle.position + (particle.velocity * step.dt),
      );
    }
    updateBodyContacts();
    updateContacts(false);
    if ((allParticleFlags & ParticleType.viscousParticle) != 0) {
      solveViscous(step);
    }
    if ((allParticleFlags & ParticleType.powderParticle) != 0) {
      solvePowder(step);
    }
    if ((allParticleFlags & ParticleType.tensileParticle) != 0) {
      solveTensile(step);
    }
    if ((allParticleFlags & ParticleType.elasticParticle) != 0) {
      solveElastic(step);
    }
    if ((allParticleFlags & ParticleType.springParticle) != 0) {
      solveSpring(step);
    }
    if ((allGroupFlags & ParticleGroupType.solidParticleGroup) != 0) {
      solveSolid(step);
    }
    if ((allParticleFlags & ParticleType.colorMixingParticle) != 0) {
      solveColorMixing(step);
    }
    solvePressure(step);
    solveDamping(step);
  }

  void solvePressure(TimeStep step) {
    // calculates the sum of contact-weights for each particle
    // that means dimensionless density
    for (Particle particle in _particles) {
      particle.accumulation = 0.0;
    }
    for (ParticleBodyContact contact in bodyContactBuffer) {
      contact.particle.accumulation += contact.weight;
    }
    for (ParticleContact contact in contactBuffer) {
      contact.particleA.accumulation += contact.weight;
      contact.particleB.accumulation += contact.weight;
    }
    // ignores powder particles
    if ((allParticleFlags & k_noPressureFlags) != 0) {
      for (Particle particle in _particles) {
        if ((particle.flags & k_noPressureFlags) != 0) {
          particle.accumulation = 0.0;
        }
      }
    }
    // calculates pressure as a linear function of density
    final double pressurePerWeight =
        pressureStrength * getCriticalPressure(step);
    for (Particle particle in _particles) {
      final double w = particle.accumulation;
      final double h = pressurePerWeight *
          math.max(
            0.0,
            math.min(w, settings.maxParticleWeight) -
                settings.minParticleWeight,
          );
      particle.accumulation = h;
    }
    // applies pressure between each particles in contact
    final double velocityPerPressure = step.dt / (density * particleDiameter);
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final Particle particle = contact.particle;
      final Body b = contact.body;
      final double w = contact.weight;
      final double m = contact.mass;
      final Vector2 n = contact.normal;
      final Vector2 p = particle.position;
      final double h = particle.accumulation + pressurePerWeight * w;
      final Vector2 f = _tempVec;
      final double coef = velocityPerPressure * w * m * h;
      f.x = coef * n.x;
      f.y = coef * n.y;
      final Vector2 velData = particle.velocity;
      final double particleInvMass = getParticleInvMass();
      velData.x -= particleInvMass * f.x;
      velData.y -= particleInvMass * f.y;
      b.applyLinearImpulse(f, point: p);
    }
    for (ParticleContact contact in contactBuffer) {
      final Particle particleA = contact.particleA;
      final Particle particleB = contact.particleB;
      final double w = contact.weight;
      final Vector2 n = contact.normal;
      final double h = particleA.accumulation + particleB.accumulation;
      final double fx = velocityPerPressure * w * h * n.x;
      final double fy = velocityPerPressure * w * h * n.y;
      final Vector2 velDataA = particleA.velocity;
      final Vector2 velDataB = particleB.velocity;
      velDataA.x -= fx;
      velDataA.y -= fy;
      velDataB.x += fx;
      velDataB.y += fy;
    }
  }

  void solveDamping(TimeStep step) {
    // reduces normal velocity of each contact
    final double damping = dampingStrength;
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final Particle particle = contact.particle;
      final Body b = contact.body;
      final double w = contact.weight;
      final double m = contact.mass;
      final Vector2 n = contact.normal;
      final Vector2 p = particle.position;
      final double tempX = p.x - b._sweep.c.x;
      final double tempY = p.y - b._sweep.c.y;
      final Vector2 velA = particle.velocity;
      final double vx =
          -b._angularVelocity * tempY + b.linearVelocity.x - velA.x;
      final double vy =
          b._angularVelocity * tempX + b.linearVelocity.y - velA.y;
      final double vn = vx * n.x + vy * n.y;
      if (vn < 0) {
        final Vector2 f = _tempVec;
        f.x = damping * w * m * vn * n.x;
        f.y = damping * w * m * vn * n.y;
        final double invMass = getParticleInvMass();
        velA.x += invMass * f.x;
        velA.y += invMass * f.y;
        f.x = -f.x;
        f.y = -f.y;
        b.applyLinearImpulse(f, point: p);
      }
    }
    for (ParticleContact contact in contactBuffer) {
      final Particle particleA = contact.particleA;
      final Particle particleB = contact.particleB;
      final double w = contact.weight;
      final Vector2 n = contact.normal;
      final Vector2 velA = particleA.velocity;
      final Vector2 velB = particleB.velocity;
      final double vx = velB.x - velA.x;
      final double vy = velB.y - velA.y;
      final double vn = vx * n.x + vy * n.y;
      if (vn < 0) {
        final double fx = damping * w * vn * n.x;
        final double fy = damping * w * vn * n.y;
        velA.x += fx;
        velA.y += fy;
        velB.x -= fx;
        velB.y -= fy;
      }
    }
  }

  void solveWall(TimeStep step) {
    for (Particle particle in _particles) {
      if ((particle.flags & ParticleType.wallParticle) != 0) {
        particle.velocity.setZero();
      }
    }
  }

  final Rot _tempRot = Rot();
  final Transform _tempXf = Transform.zero();
  final Transform _tempXf2 = Transform.zero();

  void solveRigid(final TimeStep step) {
    for (ParticleGroup group in groupBuffer) {
      if ((group.groupFlags & ParticleGroupType.rigidParticleGroup) != 0) {
        group.updateStatistics();
        final Vector2 temp = _tempVec;
        final Rot rotation = _tempRot;
        rotation.setAngle(step.dt * group._angularVelocity);
        final Vector2 cross = Rot.mulVec2(rotation, group._center);
        temp
          ..setFrom(group._linearVelocity)
          ..scale(step.dt)
          ..add(group._center)
          ..sub(cross);
        _tempXf.p.setFrom(temp);
        _tempXf.q.setFrom(rotation);
        group.transform.set(Transform.mul(_tempXf, group.transform));
        final Transform velocityTransform = _tempXf2
          ..p.x = step.invDt * _tempXf.p.x
          ..p.y = step.invDt * _tempXf.p.y
          ..q.s = step.invDt * _tempXf.q.s
          ..q.c = step.invDt * (_tempXf.q.c - 1);
        for (Particle particle in group.particles) {
          particle.velocity.setFrom(
            Transform.mulVec2(velocityTransform, particle.position),
          );
        }
      }
    }
  }

  void solveElastic(final TimeStep step) {
    final double elasticStrength = step.invDt * this.elasticStrength;
    for (PsTriad triad in triadBuffer) {
      if ((triad.flags & ParticleType.elasticParticle) != 0) {
        final Particle particleA = triad.particleA;
        final Particle particleB = triad.particleB;
        final Particle particleC = triad.particleC;
        final Vector2 oa = triad.pa;
        final Vector2 ob = triad.pb;
        final Vector2 oc = triad.pc;
        final Vector2 pa = particleA.position;
        final Vector2 pb = particleB.position;
        final Vector2 pc = particleC.position;
        final double px = 1.0 / 3 * (pa.x + pb.x + pc.x);
        final double py = 1.0 / 3 * (pa.y + pb.y + pc.y);
        double rs = oa.cross(pa) + ob.cross(pb) + oc.cross(pc);
        double rc = oa.dot(pa) + ob.dot(pb) + oc.dot(pc);
        final double r2 = rs * rs + rc * rc;
        final double invR = r2 == 0 ? double.maxFinite : math.sqrt(1.0 / r2);
        rs *= invR;
        rc *= invR;
        final double strength = elasticStrength * triad.strength;
        final double roax = rc * oa.x - rs * oa.y;
        final double roay = rs * oa.x + rc * oa.y;
        final double robx = rc * ob.x - rs * ob.y;
        final double roby = rs * ob.x + rc * ob.y;
        final double rocx = rc * oc.x - rs * oc.y;
        final double rocy = rs * oc.x + rc * oc.y;
        final Vector2 va = particleA.velocity;
        final Vector2 vb = particleB.velocity;
        final Vector2 vc = particleC.velocity;
        va.x += strength * (roax - (pa.x - px));
        va.y += strength * (roay - (pa.y - py));
        vb.x += strength * (robx - (pb.x - px));
        vb.y += strength * (roby - (pb.y - py));
        vc.x += strength * (rocx - (pc.x - px));
        vc.y += strength * (rocy - (pc.y - py));
      }
    }
  }

  void solveSpring(final TimeStep step) {
    final double springStrength = step.invDt * this.springStrength;
    for (PsPair pair in pairBuffer) {
      if ((pair.flags & ParticleType.springParticle) != 0) {
        final Particle particleA = pair.particleA;
        final Particle particleB = pair.particleB;
        final Vector2 pa = particleA.position;
        final Vector2 pb = particleB.position;
        final double dx = pb.x - pa.x;
        final double dy = pb.y - pa.y;
        final double r0 = pair.distance;
        double r1 = math.sqrt(dx * dx + dy * dy);
        r1 = r1 == 0 ? double.maxFinite : r1;
        final double strength = springStrength * pair.strength;
        final double fx = strength * (r0 - r1) / r1 * dx;
        final double fy = strength * (r0 - r1) / r1 * dy;
        final Vector2 va = particleA.velocity;
        final Vector2 vb = particleB.velocity;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveTensile(final TimeStep step) {
    for (Particle particle in _particles) {
      particle.accumulation = 0.0;
      particle.accumulationVector.setZero();
    }
    for (ParticleContact contact in contactBuffer) {
      if ((contact.flags & ParticleType.tensileParticle) != 0) {
        final Particle particleA = contact.particleA;
        final Particle particleB = contact.particleB;
        final double w = contact.weight;
        final Vector2 n = contact.normal;
        particleA.accumulation += w;
        particleB.accumulation += w;
        final Vector2 a2A = particleA.accumulationVector;
        final Vector2 a2B = particleB.accumulationVector;
        final double inter = (1 - w) * w;
        a2A.x -= inter * n.x;
        a2A.y -= inter * n.y;
        a2B.x += inter * n.x;
        a2B.y += inter * n.y;
      }
    }
    final double strengthA =
        surfaceTensionStrengthA * getCriticalVelocity(step);
    final double strengthB =
        surfaceTensionStrengthB * getCriticalVelocity(step);
    for (ParticleContact contact in contactBuffer) {
      if ((contact.flags & ParticleType.tensileParticle) != 0) {
        final Particle particleA = contact.particleA;
        final Particle particleB = contact.particleB;
        final double w = contact.weight;
        final Vector2 n = contact.normal;
        final Vector2 a2A = particleA.accumulationVector;
        final Vector2 a2B = particleB.accumulationVector;
        final double h = particleA.accumulation + particleB.accumulation;
        final double sx = a2B.x - a2A.x;
        final double sy = a2B.y - a2A.y;
        final double fn =
            (strengthA * (h - 2) + strengthB * (sx * n.x + sy * n.y)) * w;
        final double fx = fn * n.x;
        final double fy = fn * n.y;
        final Vector2 va = particleA.velocity;
        final Vector2 vb = particleB.velocity;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveViscous(final TimeStep step) {
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final Particle particle = contact.particle;
      if ((particle.flags & ParticleType.viscousParticle) != 0) {
        final Body b = contact.body;
        final double w = contact.weight;
        final double m = contact.mass;
        final Vector2 p = particle.position;
        final Vector2 va = particle.velocity;
        final double tempX = p.x - b._sweep.c.x;
        final double tempY = p.y - b._sweep.c.y;
        final double vx =
            -b._angularVelocity * tempY + b.linearVelocity.x - va.x;
        final double vy =
            b._angularVelocity * tempX + b.linearVelocity.y - va.y;
        final Vector2 f = _tempVec;
        final double pInvMass = getParticleInvMass();
        f.x = viscousStrength * m * w * vx;
        f.y = viscousStrength * m * w * vy;
        va.x += pInvMass * f.x;
        va.y += pInvMass * f.y;
        f.x = -f.x;
        f.y = -f.y;
        b.applyLinearImpulse(f, point: p);
      }
    }
    for (ParticleContact contact in contactBuffer) {
      if ((contact.flags & ParticleType.viscousParticle) != 0) {
        final Particle particleA = contact.particleA;
        final Particle particleB = contact.particleB;
        final double w = contact.weight;
        final Vector2 va = particleA.velocity;
        final Vector2 vb = particleB.velocity;
        final double vx = vb.x - va.x;
        final double vy = vb.y - va.y;
        final double fx = viscousStrength * w * vx;
        final double fy = viscousStrength * w * vy;
        va.x += fx;
        va.y += fy;
        vb.x -= fx;
        vb.y -= fy;
      }
    }
  }

  void solvePowder(final TimeStep step) {
    final double powderStrength =
        this.powderStrength * getCriticalVelocity(step);
    final double minWeight = 1.0 - settings.particleStride;
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final Particle particle = contact.particle;
      if ((particle.flags & ParticleType.powderParticle) != 0) {
        final double w = contact.weight;
        if (w > minWeight) {
          final Body b = contact.body;
          final double m = contact.mass;
          final Vector2 p = particle.position;
          final Vector2 n = contact.normal;
          final Vector2 f = _tempVec;
          final Vector2 va = particle.velocity;
          final double inter = powderStrength * m * (w - minWeight);
          final double pInvMass = getParticleInvMass();
          f.x = inter * n.x;
          f.y = inter * n.y;
          va.x -= pInvMass * f.x;
          va.y -= pInvMass * f.y;
          b.applyLinearImpulse(f, point: p);
        }
      }
    }
    for (ParticleContact contact in contactBuffer) {
      if ((contact.flags & ParticleType.powderParticle) != 0) {
        final double w = contact.weight;
        if (w > minWeight) {
          final Particle particleA = contact.particleA;
          final Particle particleB = contact.particleB;
          final Vector2 n = contact.normal;
          final Vector2 va = particleA.velocity;
          final Vector2 vb = particleB.velocity;
          final double inter = powderStrength * (w - minWeight);
          final double fx = inter * n.x;
          final double fy = inter * n.y;
          va.x -= fx;
          va.y -= fy;
          vb.x += fx;
          vb.y += fy;
        }
      }
    }
  }

  void solveSolid(final TimeStep step) {
    // applies extra repulsive force from solid particle groups
    // TODO.spydon: Why was this separate depth buffer used?
    //final depthBuffer = Float64List(_particleCount);
    final double ejectionStrength = step.invDt * this.ejectionStrength;
    for (ParticleContact contact in contactBuffer) {
      final Particle particleA = contact.particleA;
      final Particle particleB = contact.particleB;
      if (particleA.group != particleB.group) {
        final double w = contact.weight;
        final Vector2 n = contact.normal;
        final double h = particleA.depth + particleB.depth;
        final Vector2 va = particleA.velocity;
        final Vector2 vb = particleA.velocity;
        final double inter = ejectionStrength * h * w;
        final double fx = inter * n.x;
        final double fy = inter * n.y;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveColorMixing(final TimeStep step) {
    // mixes color between contacting particles
    final int colorMixing256 = (256 * colorMixingStrength).toInt();
    for (ParticleContact contact in contactBuffer) {
      final Particle particleA = contact.particleA;
      final Particle particleB = contact.particleA;
      if ((particleA.flags &
              particleB.flags &
              ParticleType.colorMixingParticle) !=
          0) {
        final Color3i colorA = particleA.color;
        final Color3i colorB = particleB.color;
        final int dr = (colorMixing256 * (colorB.r - colorA.r)).toInt() >> 8;
        final int dg = (colorMixing256 * (colorB.g - colorA.g)).toInt() >> 8;
        final int db = (colorMixing256 * (colorB.b - colorA.b)).toInt() >> 8;
        final int da = (colorMixing256 * (colorB.a - colorA.a)).toInt() >> 8;
        colorA.r += dr;
        colorA.g += dg;
        colorA.b += db;
        colorA.a += da;
        colorB.r -= dr;
        colorB.g -= dg;
        colorB.b -= db;
        colorB.a -= da;
      }
    }
  }

  void solveZombie() {
    // removes particles with zombie flag
    bool isZombie(Particle particle) {
      return (particle.flags & ParticleType.zombieParticle) != 0;
    }

    _particles.removeWhere((p) {
      if (isZombie(p)) {
        if ((p.flags & ParticleType.destructionListener) != 0) {
          world.particleDestructionListener?.sayGoodbyeParticle(p);
        }
        return true;
      }
      return false;
    });

    proxyBuffer.removeWhere((proxy) => isZombie(proxy.particle));
    contactBuffer.removeWhere((c) => [c.particleA, c.particleB].any(isZombie));
    bodyContactBuffer.removeWhere((c) => isZombie(c.particle));
    pairBuffer.removeWhere((p) => [p.particleA, p.particleB].any(isZombie));
    triadBuffer.removeWhere(
      (t) => [t.particleA, t.particleB, t.particleC].any(isZombie),
    );

    groupBuffer.removeWhere((g) {
      g.particles.removeWhere(isZombie);
      final toBeRemoved = g.destroyAutomatically && g.particles.isEmpty;
      if (toBeRemoved) {
        world.particleDestructionListener?.sayGoodbyeParticleGroup(g);
      }
      return toBeRemoved;
    });

    // TODO: split the groups sometimes if they are rigid?
  }

  void setParticleRadius(double radius) {
    particleDiameter = 2 * radius;
    squaredDiameter = particleDiameter * particleDiameter;
    inverseDiameter = 1 / particleDiameter;
  }

  void setParticleDensity(double density) {
    density = density;
    inverseDensity = 1 / density;
  }

  double getParticleDensity() {
    return density;
  }

  void setParticleGravityScale(double gravityScale) {
    gravityScale = gravityScale;
  }

  double getParticleGravityScale() {
    return gravityScale;
  }

  void setParticleDamping(double damping) {
    dampingStrength = damping;
  }

  double getParticleDamping() {
    return dampingStrength;
  }

  double getParticleRadius() {
    return particleDiameter / 2;
  }

  double getCriticalVelocity(final TimeStep step) {
    return particleDiameter * step.invDt;
  }

  double getCriticalVelocitySquared(final TimeStep step) {
    final double velocity = getCriticalVelocity(step);
    return velocity * velocity;
  }

  double getCriticalPressure(final TimeStep step) {
    return density * getCriticalVelocitySquared(step);
  }

  double getParticleStride() {
    return settings.particleStride * particleDiameter;
  }

  double getParticleMass() {
    final double stride = getParticleStride();
    return density * stride * stride;
  }

  double getParticleInvMass() {
    return 1.777777 * inverseDensity * inverseDiameter * inverseDiameter;
  }

  static int _lowerBound(Iterable<PsProxy> ray, int tag) {
    return _bound(ray, tag, (int a, int b) => a < b);
  }

  static int _upperBound(Iterable<PsProxy> ray, int tag) {
    return _bound(ray, tag, (int a, int b) => a >= b);
  }

  static int _bound(
    Iterable<PsProxy> ray,
    int tag,
    bool compare(int a, int b),
  ) {
    int left = 0;
    int step, current;
    int length = ray.length;
    final rayList = ray.toList(growable: false);
    while (length > 0) {
      step = length ~/ 2;
      current = left + step;
      if (compare(rayList[current].tag, tag)) {
        left = current + 1;
        length -= step + 1;
      } else {
        length = step;
      }
    }
    return left;
  }

  void queryAABB(ParticleQueryCallback callback, final AABB aabb) {
    if (proxyBuffer.isEmpty) {
      return;
    }

    final double lowerBoundX = aabb.lowerBound.x;
    final double lowerBoundY = aabb.lowerBound.y;
    final double upperBoundX = aabb.upperBound.x;
    final double upperBoundY = aabb.upperBound.y;
    final int firstProxy = _lowerBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * lowerBoundX,
        inverseDiameter * lowerBoundY,
      ),
    );
    final int lastProxy = _upperBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * upperBoundX,
        inverseDiameter * upperBoundY,
      ),
    );
    for (int i = firstProxy; i < lastProxy; ++i) {
      // TODO: Does this still work now when we don't rotate the buffers?
      final Particle particle = proxyBuffer[i].particle;
      final Vector2 p = particle.position;
      if (lowerBoundX < p.x &&
          p.x < upperBoundX &&
          lowerBoundY < p.y &&
          p.y < upperBoundY) {
        if (!callback.reportParticle(particle)) {
          break;
        }
      }
    }
  }

  void raycast(
    ParticleRaycastCallback callback,
    final Vector2 point1,
    final Vector2 point2,
  ) {
    if (proxyBuffer.isEmpty) {
      return;
    }
    final int firstProxy = _lowerBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * math.min(point1.x, point2.x) - 1,
        inverseDiameter * math.min(point1.y, point2.y) - 1,
      ),
    );
    final int lastProxy = _upperBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * math.max(point1.x, point2.x) + 1,
        inverseDiameter * math.max(point1.y, point2.y) + 1,
      ),
    );
    double fraction = 1.0;
    // solving the following equation:
    // ((1-t)*point1+t*point2-position)^2=diameter^2
    // where t is a potential fraction
    final double vx = point2.x - point1.x;
    final double vy = point2.y - point1.y;
    double v2 = vx * vx + vy * vy;
    v2 = v2 == 0 ? double.maxFinite : v2;
    for (int i = firstProxy; i < lastProxy; ++i) {
      // TODO: Is this correct now when we are not rotating the buffers?
      final positionI = proxyBuffer[i].particle.position;
      final double px = point1.x - positionI.x;
      final double py = point1.y - positionI.y;
      final double pv = px * vx + py * vy;
      final double p2 = px * px + py * py;
      final double determinant = pv * pv - v2 * (p2 - squaredDiameter);
      if (determinant >= 0) {
        final double sqrtDeterminant = math.sqrt(determinant);
        // find a solution between 0 and fraction
        double t = (-pv - sqrtDeterminant) / v2;
        if (t > fraction) {
          continue;
        }
        if (t < 0) {
          t = (-pv + sqrtDeterminant) / v2;
          if (t < 0 || t > fraction) {
            continue;
          }
        }
        final Vector2 n = _tempVec;
        _tempVec.x = px + t * vx;
        _tempVec.y = py + t * vy;
        n.normalize();
        final Vector2 point = Vector2(point1.x + t * vx, point1.y + t * vy);
        final double f = callback.reportParticle(i, point, n, t);
        fraction = math.min(fraction, f);
        if (fraction <= 0) {
          break;
        }
      }
    }
  }

  double computeParticleCollisionEnergy() {
    double sumV2 = 0.0;
    for (ParticleContact contact in contactBuffer) {
      final Vector2 collisionVelocity =
          contact.particleA.velocity - contact.particleB.velocity;
      final double vn = collisionVelocity.dot(contact.normal);
      if (vn < 0) {
        sumV2 += vn * vn;
      }
    }
    return 0.5 * getParticleMass() * sumV2;
  }
}
