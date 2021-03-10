import 'dart:collection';
import 'dart:math';

import '../../forge2d.dart';
import '../callbacks/particle_query_callback.dart';
import '../callbacks/particle_raycast_callback.dart';
import '../settings.dart' as settings;

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
  static const int pairFlags = ParticleType.springParticle;

  /// All particle types that require creating triads
  static const int triadFlags = ParticleType.elasticParticle;

  /// All particle types that require computing depth
  static const int noPressureFlags = ParticleType.powderParticle;

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
  double _particleDensity = 1.0;
  double _inverseDensity = 1.0;
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

  late double pressureStrength;
  late double dampingStrength;
  late double elasticStrength;
  late double springStrength;
  late double viscousStrength;
  late double surfaceTensionStrengthA;
  late double surfaceTensionStrengthB;
  late double powderStrength;
  late double ejectionStrength;
  late double colorMixingStrength;

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
    var flags = ParticleType.zombieParticle;
    if (callDestructionListener) {
      flags |= ParticleType.destroyListener;
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
    final stride = particleStride;
    final identity = _tempTransform..setIdentity();
    final transform = _tempTransform2..setIdentity();

    final group = ParticleGroup(this)
      ..groupFlags = groupDef.groupFlags
      ..strength = groupDef.strength
      ..userData = groupDef.userData
      ..transform.set(transform)
      ..destroyAutomatically = groupDef.destroyAutomatically;

    if (groupDef.shape != null) {
      final seedParticle = Particle(this, group: group)
        ..flags = groupDef.flags
        ..color = groupDef.color
        ..userData = groupDef.userData;
      final shape = groupDef.shape;
      transform.setVec2Angle(groupDef.position, groupDef.angle);
      final aabb = _temp;
      final childCount = shape!.getChildCount();
      for (var childIndex = 0; childIndex < childCount; childIndex++) {
        if (childIndex == 0) {
          shape.computeAABB(aabb, identity, childIndex);
        } else {
          final childAABB = _temp2;
          shape.computeAABB(childAABB, identity, childIndex);
          aabb.combine(childAABB);
        }
      }
      final upperBoundY = aabb.upperBound.y;
      final upperBoundX = aabb.upperBound.x;
      for (var y = (aabb.lowerBound.y / stride).floor() * stride;
          y < upperBoundY;
          y += stride) {
        for (var x = (aabb.lowerBound.x / stride).floor() * stride;
            x < upperBoundX;
            x += stride) {
          final p = _tempVec..setValues(x, y);
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
    if ((groupDef.flags & pairFlags) != 0) {
      for (final contact in contactBuffer) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        if (group.particles.contains(particleA) &&
            group.particles.contains(particleB)) {
          final pair = PsPair(particleA, particleB)
            ..flags = contact.flags
            ..strength = groupDef.strength
            ..distance = particleA.position.distanceTo(particleB.position);
          pairBuffer.add(pair);
        }
      }
    }
    if ((groupDef.flags & triadFlags) != 0) {
      final diagram = VoronoiDiagram();
      print('group: ${group.particles.length}');
      for (final particle in group.particles) {
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
    var particleFlags = 0;
    final joinedParticles = groupA.particles + groupB.particles;
    for (final particle in joinedParticles) {
      particleFlags |= particle.flags;
    }

    updateContacts(true);
    if ((particleFlags & pairFlags) != 0) {
      for (final contact in contactBuffer) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        if (groupA.particles.contains(particleA) &&
            groupB.particles.contains(particleB)) {
          final pair = PsPair(particleA, particleB)
            ..flags = contact.flags
            ..strength = min(groupA.strength, groupB.strength)
            ..distance = particleA.position.distanceTo(particleB.position);
          pairBuffer.add(pair);
        }
      }
    }
    if ((particleFlags & triadFlags) != 0) {
      final diagram = VoronoiDiagram();
      for (final particle in joinedParticles) {
        if ((particle.flags & ParticleType.zombieParticle) == 0) {
          diagram.addGenerator(particle.position, particle);
        }
      }
      diagram.generate(particleStride / 2);
      final callback = JoinParticleGroupsCallback();
      callback.system = this;
      callback.groupA = groupA;
      callback.groupB = groupB;
      diagram.getNodes(callback);
    }

    for (final particle in groupB.particles) {
      groupA.add(particle);
      particle.group = groupA;
    }
    final groupFlags = groupA.groupFlags | groupB.groupFlags;
    groupA.groupFlags = groupFlags;
    // Remove group b, since all its particles are in group a now
    world.particleDestroyListener?.onDestroyParticleGroup(groupB);
    groupBuffer.remove(groupB);

    if ((groupFlags & ParticleGroupType.solidParticleGroup) != 0) {
      computeDepthForGroup(groupA);
    }
  }

  void computeDepthForGroup(ParticleGroup group) {
    for (final particle in group.particles) {
      particle.accumulation = 0.0;
    }
    for (final contact in contactBuffer) {
      final particleA = contact.particleA;
      final particleB = contact.particleB;
      if (group.contains(particleA) && group.contains(particleB)) {
        final w = contact.weight;
        particleA.accumulation += w;
        particleB.accumulation += w;
      }
    }
    for (final particle in group.particles) {
      particle.depth = particle.accumulation < 0.8 ? 0.0 : double.maxFinite;
    }
    for (var t = 0; t < group.particles.length; t++) {
      var updated = false;
      for (final contact in contactBuffer) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        if (group.contains(particleA) && group.contains(particleB)) {
          final r = 1 - contact.weight;
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
    for (final particle in group.particles) {
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
    final pa = particleA.position;
    final pb = particleB.position;
    final dx = pb.x - pa.x;
    final dy = pb.y - pa.y;
    final d2 = dx * dx + dy * dy;
    if (d2 < squaredDiameter) {
      final invD = d2 != 0 ? sqrt(1 / d2) : double.maxFinite;
      final contact = ParticleContact(particleA, particleB)
        ..flags = particleA.flags | particleB.flags
        ..weight = 1 - d2 * invD * inverseDiameter
        ..normal.x = invD * dx
        ..normal.y = invD * dy;
      contactBuffer.add(contact);
    }
  }

  void updateContacts(bool exceptZombie) {
    for (final proxy in proxyBuffer) {
      final pos = proxy.particle.position;
      proxy.tag = computeTag(inverseDiameter * pos.x, inverseDiameter * pos.y);
    }
    proxyBuffer.sort();
    contactBuffer.clear();
    var cIndex = 0;
    for (var i = 0; i < proxyBuffer.length; i++) {
      final proxyA = proxyBuffer[i];
      final rightTag = computeRelativeTag(proxyA.tag, 1, 0);
      for (var j = i + 1; j < proxyBuffer.length; j++) {
        final proxyB = proxyBuffer[j];
        if (rightTag < proxyB.tag) {
          break;
        }
        addContact(proxyA.particle, proxyB.particle);
      }
      final bottomLeftTag = computeRelativeTag(proxyA.tag, -1, 1);
      for (; cIndex < proxyBuffer.length; cIndex++) {
        final c = proxyBuffer[cIndex];
        if (bottomLeftTag <= c.tag) {
          break;
        }
      }
      final bottomRightTag = computeRelativeTag(proxyA.tag, 1, 1);

      for (var bIndex = cIndex; bIndex < proxyBuffer.length; bIndex++) {
        final proxyB = proxyBuffer[bIndex];
        if (bottomRightTag < proxyB.tag) {
          break;
        }
        addContact(proxyA.particle, proxyB.particle);
      }
    }
    if (exceptZombie) {
      var j = contactBuffer.length;
      for (var i = 0; i < j; i++) {
        if ((contactBuffer[i].flags & ParticleType.zombieParticle) != 0) {
          --j;
          final temp = contactBuffer[j];
          contactBuffer[j] = contactBuffer[i];
          contactBuffer[i] = temp;
          --i;
        }
      }
    }
  }

  final UpdateBodyContactsCallback _ubccallback = UpdateBodyContactsCallback();

  void updateBodyContacts() {
    final aabb = _temp;
    aabb.lowerBound.x = double.maxFinite;
    aabb.lowerBound.y = double.maxFinite;
    aabb.upperBound.x = -double.maxFinite;
    aabb.upperBound.y = -double.maxFinite;
    for (final particle in _particles) {
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
    final aabb = _temp;
    final lowerBound = aabb.lowerBound;
    final upperBound = aabb.upperBound;
    lowerBound.x = double.maxFinite;
    lowerBound.y = double.maxFinite;
    upperBound.x = -double.maxFinite;
    upperBound.y = -double.maxFinite;
    for (final particle in _particles) {
      final v = particle.velocity;
      final p1 = particle.position;
      final p2x = p1.x + step.dt * v.x;
      final p2y = p1.y + step.dt * v.y;
      final bx = p1.x < p2x ? p1.x : p2x;
      final by = p1.y < p2y ? p1.y : p2y;
      lowerBound.x = lowerBound.x < bx ? lowerBound.x : bx;
      lowerBound.y = lowerBound.y < by ? lowerBound.y : by;
      final b1x = p1.x > p2x ? p1.x : p2x;
      final b1y = p1.y > p2y ? p1.y : p2y;
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
    for (final particle in _particles) {
      allParticleFlags |= particle.flags;
    }
    if ((allParticleFlags & ParticleType.zombieParticle) != 0) {
      solveZombie();
    }
    if (_particles.isEmpty) {
      return;
    }
    allGroupFlags = 0;
    for (final group in groupBuffer) {
      allGroupFlags |= group.groupFlags;
    }
    final gravityx = step.dt * gravityScale * world.getGravity().x;
    final gravityy = step.dt * gravityScale * world.getGravity().y;
    final criticalVelocitySquared = getCriticalVelocitySquared(step);
    for (final particle in _particles) {
      final v = particle.velocity;
      v.x += gravityx;
      v.y += gravityy;
      final v2 = v.x * v.x + v.y * v.y;
      if (v2 > criticalVelocitySquared) {
        final a =
            v2 == 0 ? double.maxFinite : sqrt(criticalVelocitySquared / v2);
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
    for (final particle in _particles) {
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
    for (final particle in _particles) {
      particle.accumulation = 0.0;
    }
    for (final contact in bodyContactBuffer) {
      contact.particle.accumulation += contact.weight;
    }
    for (final contact in contactBuffer) {
      contact.particleA.accumulation += contact.weight;
      contact.particleB.accumulation += contact.weight;
    }
    // ignores powder particles
    if ((allParticleFlags & noPressureFlags) != 0) {
      for (final particle in _particles) {
        if ((particle.flags & noPressureFlags) != 0) {
          particle.accumulation = 0.0;
        }
      }
    }
    // calculates pressure as a linear function of density
    final pressurePerWeight = pressureStrength * getCriticalPressure(step);
    for (final particle in _particles) {
      final w = particle.accumulation;
      final h = pressurePerWeight *
          max(
            0.0,
            min(w, settings.maxParticleWeight) - settings.minParticleWeight,
          );
      particle.accumulation = h;
    }
    // applies pressure between each particles in contact
    final velocityPerPressure = step.dt / (_particleDensity * particleDiameter);
    for (final contact in bodyContactBuffer) {
      final particle = contact.particle;
      final b = contact.body;
      final w = contact.weight;
      final m = contact.mass;
      final n = contact.normal;
      final p = particle.position;
      final h = particle.accumulation + pressurePerWeight * w;
      final f = _tempVec;
      final coef = velocityPerPressure * w * m * h;
      f.x = coef * n.x;
      f.y = coef * n.y;
      final velData = particle.velocity;
      velData.x -= particleInverseMass * f.x;
      velData.y -= particleInverseMass * f.y;
      b.applyLinearImpulse(f, point: p);
    }
    for (final contact in contactBuffer) {
      final particleA = contact.particleA;
      final particleB = contact.particleB;
      final w = contact.weight;
      final n = contact.normal;
      final h = particleA.accumulation + particleB.accumulation;
      final fx = velocityPerPressure * w * h * n.x;
      final fy = velocityPerPressure * w * h * n.y;
      final velDataA = particleA.velocity;
      final velDataB = particleB.velocity;
      velDataA.x -= fx;
      velDataA.y -= fy;
      velDataB.x += fx;
      velDataB.y += fy;
    }
  }

  void solveDamping(TimeStep step) {
    // reduces normal velocity of each contact
    final damping = dampingStrength;
    for (final contact in bodyContactBuffer) {
      final particle = contact.particle;
      final b = contact.body;
      final w = contact.weight;
      final m = contact.mass;
      final n = contact.normal;
      final p = particle.position;
      final tempX = p.x - b.sweep.c.x;
      final tempY = p.y - b.sweep.c.y;
      final velA = particle.velocity;
      final vx = -b.angularVelocity * tempY + b.linearVelocity.x - velA.x;
      final vy = b.angularVelocity * tempX + b.linearVelocity.y - velA.y;
      final vn = vx * n.x + vy * n.y;
      if (vn < 0) {
        final f = _tempVec;
        f.x = damping * w * m * vn * n.x;
        f.y = damping * w * m * vn * n.y;
        velA.x += particleInverseMass * f.x;
        velA.y += particleInverseMass * f.y;
        f.x = -f.x;
        f.y = -f.y;
        b.applyLinearImpulse(f, point: p);
      }
    }
    for (final contact in contactBuffer) {
      final particleA = contact.particleA;
      final particleB = contact.particleB;
      final w = contact.weight;
      final n = contact.normal;
      final velA = particleA.velocity;
      final velB = particleB.velocity;
      final vx = velB.x - velA.x;
      final vy = velB.y - velA.y;
      final vn = vx * n.x + vy * n.y;
      if (vn < 0) {
        final fx = damping * w * vn * n.x;
        final fy = damping * w * vn * n.y;
        velA.x += fx;
        velA.y += fy;
        velB.x -= fx;
        velB.y -= fy;
      }
    }
  }

  void solveWall(TimeStep step) {
    for (final particle in _particles) {
      if ((particle.flags & ParticleType.wallParticle) != 0) {
        particle.velocity.setZero();
      }
    }
  }

  final Rot _tempRot = Rot();
  final Transform _tempXf = Transform.zero();
  final Transform _tempXf2 = Transform.zero();

  void solveRigid(final TimeStep step) {
    for (final group in groupBuffer) {
      if ((group.groupFlags & ParticleGroupType.rigidParticleGroup) != 0) {
        group.updateStatistics();
        final temp = _tempVec;
        final rotation = _tempRot;
        rotation.setAngle(step.dt * group.angularVelocity);
        final cross = Rot.mulVec2(rotation, group.center);
        temp
          ..setFrom(group.linearVelocity)
          ..scale(step.dt)
          ..add(group.center)
          ..sub(cross);
        _tempXf.p.setFrom(temp);
        _tempXf.q.setFrom(rotation);
        group.transform.set(Transform.mul(_tempXf, group.transform));
        final velocityTransform = _tempXf2
          ..p.x = step.invDt * _tempXf.p.x
          ..p.y = step.invDt * _tempXf.p.y
          ..q.s = step.invDt * _tempXf.q.s
          ..q.c = step.invDt * (_tempXf.q.c - 1);
        for (final particle in group.particles) {
          particle.velocity.setFrom(
            Transform.mulVec2(velocityTransform, particle.position),
          );
        }
      }
    }
  }

  void solveElastic(final TimeStep step) {
    final elasticStrength = step.invDt * this.elasticStrength;
    for (final triad in triadBuffer) {
      if ((triad.flags & ParticleType.elasticParticle) != 0) {
        final particleA = triad.particleA;
        final particleB = triad.particleB;
        final particleC = triad.particleC;
        final oa = triad.pa;
        final ob = triad.pb;
        final oc = triad.pc;
        final pa = particleA.position;
        final pb = particleB.position;
        final pc = particleC.position;
        final px = 1.0 / 3 * (pa.x + pb.x + pc.x);
        final py = 1.0 / 3 * (pa.y + pb.y + pc.y);
        var rs = oa.cross(pa) + ob.cross(pb) + oc.cross(pc);
        var rc = oa.dot(pa) + ob.dot(pb) + oc.dot(pc);
        final r2 = rs * rs + rc * rc;
        final invR = r2 == 0 ? double.maxFinite : sqrt(1.0 / r2);
        rs *= invR;
        rc *= invR;
        final strength = elasticStrength * triad.strength;
        final roax = rc * oa.x - rs * oa.y;
        final roay = rs * oa.x + rc * oa.y;
        final robx = rc * ob.x - rs * ob.y;
        final roby = rs * ob.x + rc * ob.y;
        final rocx = rc * oc.x - rs * oc.y;
        final rocy = rs * oc.x + rc * oc.y;
        final va = particleA.velocity;
        final vb = particleB.velocity;
        final vc = particleC.velocity;
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
    final springStrength = step.invDt * this.springStrength;
    for (final pair in pairBuffer) {
      if ((pair.flags & ParticleType.springParticle) != 0) {
        final particleA = pair.particleA;
        final particleB = pair.particleB;
        final pa = particleA.position;
        final pb = particleB.position;
        final dx = pb.x - pa.x;
        final dy = pb.y - pa.y;
        final r0 = pair.distance;
        var r1 = sqrt(dx * dx + dy * dy);
        r1 = r1 == 0 ? double.maxFinite : r1;
        final strength = springStrength * pair.strength;
        final fx = strength * (r0 - r1) / r1 * dx;
        final fy = strength * (r0 - r1) / r1 * dy;
        final va = particleA.velocity;
        final vb = particleB.velocity;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveTensile(final TimeStep step) {
    for (final particle in _particles) {
      particle.accumulation = 0.0;
      particle.accumulationVector.setZero();
    }
    for (final contact in contactBuffer) {
      if ((contact.flags & ParticleType.tensileParticle) != 0) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        final w = contact.weight;
        final n = contact.normal;
        particleA.accumulation += w;
        particleB.accumulation += w;
        final a2A = particleA.accumulationVector;
        final a2B = particleB.accumulationVector;
        final inter = (1 - w) * w;
        a2A.x -= inter * n.x;
        a2A.y -= inter * n.y;
        a2B.x += inter * n.x;
        a2B.y += inter * n.y;
      }
    }
    final strengthA = surfaceTensionStrengthA * getCriticalVelocity(step);
    final strengthB = surfaceTensionStrengthB * getCriticalVelocity(step);
    for (final contact in contactBuffer) {
      if ((contact.flags & ParticleType.tensileParticle) != 0) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        final w = contact.weight;
        final n = contact.normal;
        final a2A = particleA.accumulationVector;
        final a2B = particleB.accumulationVector;
        final h = particleA.accumulation + particleB.accumulation;
        final sx = a2B.x - a2A.x;
        final sy = a2B.y - a2A.y;
        final fn =
            (strengthA * (h - 2) + strengthB * (sx * n.x + sy * n.y)) * w;
        final fx = fn * n.x;
        final fy = fn * n.y;
        final va = particleA.velocity;
        final vb = particleB.velocity;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveViscous(final TimeStep step) {
    for (final contact in bodyContactBuffer) {
      final particle = contact.particle;
      if ((particle.flags & ParticleType.viscousParticle) != 0) {
        final b = contact.body;
        final w = contact.weight;
        final m = contact.mass;
        final p = particle.position;
        final va = particle.velocity;
        final tempX = p.x - b.sweep.c.x;
        final tempY = p.y - b.sweep.c.y;
        final vx = -b.angularVelocity * tempY + b.linearVelocity.x - va.x;
        final vy = b.angularVelocity * tempX + b.linearVelocity.y - va.y;
        final f = _tempVec;
        final pInvMass = particleInverseMass;
        f.x = viscousStrength * m * w * vx;
        f.y = viscousStrength * m * w * vy;
        va.x += pInvMass * f.x;
        va.y += pInvMass * f.y;
        f.x = -f.x;
        f.y = -f.y;
        b.applyLinearImpulse(f, point: p);
      }
    }
    for (final contact in contactBuffer) {
      if ((contact.flags & ParticleType.viscousParticle) != 0) {
        final particleA = contact.particleA;
        final particleB = contact.particleB;
        final w = contact.weight;
        final va = particleA.velocity;
        final vb = particleB.velocity;
        final vx = vb.x - va.x;
        final vy = vb.y - va.y;
        final fx = viscousStrength * w * vx;
        final fy = viscousStrength * w * vy;
        va.x += fx;
        va.y += fy;
        vb.x -= fx;
        vb.y -= fy;
      }
    }
  }

  void solvePowder(final TimeStep step) {
    final powderStrength = this.powderStrength * getCriticalVelocity(step);
    final minWeight = 1.0 - settings.particleStride;
    for (final contact in bodyContactBuffer) {
      final particle = contact.particle;
      if ((particle.flags & ParticleType.powderParticle) != 0) {
        final w = contact.weight;
        if (w > minWeight) {
          final b = contact.body;
          final m = contact.mass;
          final p = particle.position;
          final n = contact.normal;
          final f = _tempVec;
          final va = particle.velocity;
          final inter = powderStrength * m * (w - minWeight);
          final pInvMass = particleInverseMass;
          f.x = inter * n.x;
          f.y = inter * n.y;
          va.x -= pInvMass * f.x;
          va.y -= pInvMass * f.y;
          b.applyLinearImpulse(f, point: p);
        }
      }
    }
    for (final contact in contactBuffer) {
      if ((contact.flags & ParticleType.powderParticle) != 0) {
        final w = contact.weight;
        if (w > minWeight) {
          final particleA = contact.particleA;
          final particleB = contact.particleB;
          final n = contact.normal;
          final va = particleA.velocity;
          final vb = particleB.velocity;
          final inter = powderStrength * (w - minWeight);
          final fx = inter * n.x;
          final fy = inter * n.y;
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
    final ejectionStrength = step.invDt * this.ejectionStrength;
    for (final contact in contactBuffer) {
      final particleA = contact.particleA;
      final particleB = contact.particleB;
      if (particleA.group != particleB.group) {
        final w = contact.weight;
        final n = contact.normal;
        final h = particleA.depth + particleB.depth;
        final va = particleA.velocity;
        final vb = particleA.velocity;
        final inter = ejectionStrength * h * w;
        final fx = inter * n.x;
        final fy = inter * n.y;
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveColorMixing(final TimeStep step) {
    // mixes color between contacting particles
    final colorMixing256 = (256 * colorMixingStrength).toInt();
    for (final contact in contactBuffer) {
      final particleA = contact.particleA;
      final particleB = contact.particleA;
      if ((particleA.flags &
              particleB.flags &
              ParticleType.colorMixingParticle) !=
          0) {
        final colorA = particleA.color;
        final colorB = particleB.color;
        final dr = (colorMixing256 * (colorB.r - colorA.r)).toInt() >> 8;
        final dg = (colorMixing256 * (colorB.g - colorA.g)).toInt() >> 8;
        final db = (colorMixing256 * (colorB.b - colorA.b)).toInt() >> 8;
        final da = (colorMixing256 * (colorB.a - colorA.a)).toInt() >> 8;
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
        if ((p.flags & ParticleType.destroyListener) != 0) {
          world.particleDestroyListener?.onDestroyParticle(p);
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
        world.particleDestroyListener?.onDestroyParticleGroup(g);
      }
      return toBeRemoved;
    });

    // TODO: split the groups sometimes if they are rigid?
  }

  double get particleRadius => particleDiameter / 2;

  set particleRadius(double radius) {
    particleDiameter = 2 * radius;
    squaredDiameter = particleDiameter * particleDiameter;
    inverseDiameter = 1 / particleDiameter;
  }

  double get inverseDensity => _inverseDensity;

  set particleDensity(double density) {
    _particleDensity = density;
    _inverseDensity = 1 / density;
  }

  double get particleDensity => _particleDensity;

  double getCriticalVelocity(final TimeStep step) {
    return particleDiameter * step.invDt;
  }

  double getCriticalVelocitySquared(final TimeStep step) {
    final velocity = getCriticalVelocity(step);
    return velocity * velocity;
  }

  double getCriticalPressure(final TimeStep step) {
    return particleDensity * getCriticalVelocitySquared(step);
  }

  double get particleStride => settings.particleStride * particleDiameter;

  double get particleMass => particleDensity * particleStride * particleStride;

  double get particleInverseMass {
    return 1.777777 * inverseDensity * inverseDiameter * inverseDiameter;
  }

  static int lowerBound(Iterable<PsProxy> ray, int tag) {
    return _bound(ray, tag, (int a, int b) => a < b);
  }

  static int upperBound(Iterable<PsProxy> ray, int tag) {
    return _bound(ray, tag, (int a, int b) => a >= b);
  }

  static int _bound(
    Iterable<PsProxy> ray,
    int tag,
    bool Function(int a, int b) compare,
  ) {
    var left = 0;
    int step, current;
    var length = ray.length;
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

    final lowerBoundX = aabb.lowerBound.x;
    final lowerBoundY = aabb.lowerBound.y;
    final upperBoundX = aabb.upperBound.x;
    final upperBoundY = aabb.upperBound.y;
    final firstProxy = lowerBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * lowerBoundX,
        inverseDiameter * lowerBoundY,
      ),
    );
    final lastProxy = upperBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * upperBoundX,
        inverseDiameter * upperBoundY,
      ),
    );
    for (var i = firstProxy; i < lastProxy; ++i) {
      // TODO: Does this still work now when we don't rotate the buffers?
      final particle = proxyBuffer[i].particle;
      final p = particle.position;
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
    final firstProxy = lowerBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * min(point1.x, point2.x) - 1,
        inverseDiameter * min(point1.y, point2.y) - 1,
      ),
    );
    final lastProxy = upperBound(
      proxyBuffer,
      computeTag(
        inverseDiameter * max(point1.x, point2.x) + 1,
        inverseDiameter * max(point1.y, point2.y) + 1,
      ),
    );
    var fraction = 1.0;
    // solving the following equation:
    // ((1-t)*point1+t*point2-position)^2=diameter^2
    // where t is a potential fraction
    final vx = point2.x - point1.x;
    final vy = point2.y - point1.y;
    var v2 = vx * vx + vy * vy;
    v2 = v2 == 0 ? double.maxFinite : v2;
    for (var i = firstProxy; i < lastProxy; ++i) {
      // TODO: Is this correct now when we are not rotating the buffers?
      final positionI = proxyBuffer[i].particle.position;
      final px = point1.x - positionI.x;
      final py = point1.y - positionI.y;
      final pv = px * vx + py * vy;
      final p2 = px * px + py * py;
      final determinant = pv * pv - v2 * (p2 - squaredDiameter);
      if (determinant >= 0) {
        final sqrtDeterminant = sqrt(determinant);
        // find a solution between 0 and fraction
        var t = (-pv - sqrtDeterminant) / v2;
        if (t > fraction) {
          continue;
        }
        if (t < 0) {
          t = (-pv + sqrtDeterminant) / v2;
          if (t < 0 || t > fraction) {
            continue;
          }
        }
        final n = _tempVec;
        _tempVec.x = px + t * vx;
        _tempVec.y = py + t * vy;
        n.normalize();
        final point = Vector2(point1.x + t * vx, point1.y + t * vy);
        final f = callback.reportParticle(i, point, n, t);
        fraction = min(fraction, f);
        if (fraction <= 0) {
          break;
        }
      }
    }
  }

  double computeParticleCollisionEnergy() {
    var sumV2 = 0.0;
    for (final contact in contactBuffer) {
      final collisionVelocity =
          contact.particleA.velocity - contact.particleB.velocity;
      final vn = collisionVelocity.dot(contact.normal);
      if (vn < 0) {
        sumV2 += vn * vn;
      }
    }
    return 0.5 * particleMass * sumV2;
  }

  void render(DebugDraw debugDraw) {
    final wireframe =
        (debugDraw.drawFlags & DebugDraw.wireFrameDrawingBit) != 0;
    if (particles.isNotEmpty) {
      if (wireframe) {
        debugDraw.drawParticlesWireframe(particles, particleRadius);
      } else {
        debugDraw.drawParticles(particles, particleRadius);
      }
    }
  }
}
