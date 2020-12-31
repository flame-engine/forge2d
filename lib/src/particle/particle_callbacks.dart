part of forge2d;

class DestroyParticlesInShapeCallback implements ParticleQueryCallback {
  ParticleSystem system;
  Shape shape;
  Transform xf;
  bool callDestructionListener = false;
  int destroyed = 0;

  DestroyParticlesInShapeCallback();

  void init(ParticleSystem system, Shape shape, Transform xf,
      bool callDestructionListener) {
    this.system = system;
    this.shape = shape;
    this.xf = xf;
    this.callDestructionListener = callDestructionListener;
    destroyed = 0;
  }

  @override
  bool reportParticle(Particle particle) {
    if (shape.testPoint(xf, particle.position)) {
      system.destroyParticle(particle, callDestructionListener);
      destroyed++;
    }
    return true;
  }
}

class UpdateBodyContactsCallback implements QueryCallback {
  ParticleSystem system;

  final Vector2 _tempVec = Vector2.zero();

  @override
  bool reportFixture(Fixture fixture) {
    if (fixture.isSensor()) {
      return true;
    }
    final Shape shape = fixture.shape;
    final Body b = fixture.body;
    final Vector2 bp = b.worldCenter;
    final double bm = b.mass;
    final double bI = b.getInertia() - bm * b.getLocalCenter().length2;
    final double invBm = bm > 0 ? 1 / bm : 0.0;
    final double invBI = bI > 0 ? 1 / bI : 0.0;
    final int childCount = shape.getChildCount();
    for (int childIndex = 0; childIndex < childCount; childIndex++) {
      final AABB aabb = fixture.getAABB(childIndex);
      final double aabbLowerBoundX =
          aabb.lowerBound.x - system.particleDiameter;
      final double aabbLowerBoundY =
          aabb.lowerBound.y - system.particleDiameter;
      final double aabbUpperBoundX =
          aabb.upperBound.x + system.particleDiameter;
      final double aabbUpperBoundY =
          aabb.upperBound.y + system.particleDiameter;
      final int firstProxy = ParticleSystem._lowerBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbLowerBoundX,
          system.inverseDiameter * aabbLowerBoundY,
        ),
      );
      final int lastProxy = ParticleSystem._upperBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbUpperBoundX,
          system.inverseDiameter * aabbUpperBoundY,
        ),
      );

      for (int i = firstProxy; i != lastProxy; ++i) {
        final Particle particle = system.proxyBuffer[i].particle;
        final Vector2 ap = particle.position;
        if (aabbLowerBoundX <= ap.x &&
            ap.x <= aabbUpperBoundX &&
            aabbLowerBoundY <= ap.y &&
            ap.y <= aabbUpperBoundY) {
          double d;
          final Vector2 n = _tempVec;
          d = fixture.computeDistance(ap, childIndex, n);
          if (d < system.particleDiameter) {
            final double invAm =
                (particle.flags & ParticleType.wallParticle) != 0
                    ? 0.0
                    : system.getParticleInvMass();
            final double rpx = ap.x - bp.x;
            final double rpy = ap.y - bp.y;
            final double rpn = rpx * n.y - rpy * n.x;
            final ParticleBodyContact contact = ParticleBodyContact(particle)
              ..body = b
              ..weight = 1 - d * system.inverseDiameter
              ..normal.x = -n.x
              ..normal.y = -n.y
              ..mass = 1 / (invAm + invBm + invBI * rpn * rpn);
            system.bodyContactBuffer.add(contact);
          }
        }
      }
    }
    return true;
  }
}

// Callback used with VoronoiDiagram.
class CreateParticleGroupCallback implements VoronoiDiagramCallback {
  ParticleSystem system;
  ParticleGroupDef def;

  @override
  void call(Particle particleA, Particle particleB, Particle particleC) {
    final Vector2 pa = particleA.position;
    final Vector2 pb = particleB.position;
    final Vector2 pc = particleC.position;
    final double dabx = pa.x - pb.x;
    final double daby = pa.y - pb.y;
    final double dbcx = pb.x - pc.x;
    final double dbcy = pb.y - pc.y;
    final double dcax = pc.x - pa.x;
    final double dcay = pc.y - pa.y;
    final double maxDistanceSquared =
        settings.maxTriadDistanceSquared * system.squaredDiameter;
    if (dabx * dabx + daby * daby < maxDistanceSquared &&
        dbcx * dbcx + dbcy * dbcy < maxDistanceSquared &&
        dcax * dcax + dcay * dcay < maxDistanceSquared) {
      final double midPointX = 1.0 / 3.0 * (pa.x + pb.x + pc.x);
      final double midPointY = 1.0 / 3.0 * (pa.y + pb.y + pc.y);
      final PsTriad triad = PsTriad(particleA, particleB, particleC)
        ..flags = particleA.flags | particleB.flags | particleC.flags
        ..strength = def.strength
        ..pa.x = pa.x - midPointX
        ..pa.y = pa.y - midPointY
        ..pb.x = pb.x - midPointX
        ..pb.y = pb.y - midPointY
        ..pc.x = pc.x - midPointX
        ..pc.y = pc.y - midPointY
        ..ka = -(dcax * dabx + dcay * daby)
        ..kb = -(dabx * dbcx + daby * dbcy)
        ..kc = -(dbcx * dcax + dbcy * dcay)
        ..s = pa.cross(pb) + pb.cross(pc) + pc.cross(pa);
      system.triadBuffer.add(triad);
    }
  }
}

// Callback used with VoronoiDiagram.
class JoinParticleGroupsCallback implements VoronoiDiagramCallback {
  ParticleSystem system;
  ParticleGroup groupA;
  ParticleGroup groupB;

  @override
  void call(Particle particleA, Particle particleB, Particle particleC) {
    final callParticles = [particleA, particleB, particleC];
    bool hasCallParticle(ParticleGroup group) {
      return group.particles.any((p) => callParticles.contains(p));
    }

    // Create a triad if it will contain particles from both groups.
    if (hasCallParticle(groupA) && hasCallParticle(groupB)) {
      final int af = particleA.flags;
      final int bf = particleB.flags;
      final int cf = particleC.flags;
      if ((af & bf & cf & ParticleSystem.k_triadFlags) != 0) {
        final Vector2 pa = particleA.position;
        final Vector2 pb = particleB.position;
        final Vector2 pc = particleC.position;
        final double dabx = pa.x - pb.x;
        final double daby = pa.y - pb.y;
        final double dbcx = pb.x - pc.x;
        final double dbcy = pb.y - pc.y;
        final double dcax = pc.x - pa.x;
        final double dcay = pc.y - pa.y;
        final double maxDistanceSquared =
            settings.maxTriadDistanceSquared * system.squaredDiameter;
        if (dabx * dabx + daby * daby < maxDistanceSquared &&
            dbcx * dbcx + dbcy * dbcy < maxDistanceSquared &&
            dcax * dcax + dcay * dcay < maxDistanceSquared) {
          final double midPointX = 1.0 / 3.0 * (pa.x + pb.x + pc.x);
          final double midPointY = 1.0 / 3.0 * (pa.y + pb.y + pc.y);
          final PsTriad triad = PsTriad(particleA, particleB, particleC)
            ..flags = af | bf | cf
            ..strength = math.min(groupA._strength, groupB._strength)
            ..pa.x = pa.x - midPointX
            ..pa.y = pa.y - midPointY
            ..pb.x = pb.x - midPointX
            ..pb.y = pb.y - midPointY
            ..pc.x = pc.x - midPointX
            ..pc.y = pc.y - midPointY
            ..ka = -(dcax * dabx + dcay * daby)
            ..kb = -(dabx * dbcx + daby * dbcy)
            ..kc = -(dbcx * dcax + dbcy * dcay)
            ..s = pa.cross(pb) + pb.cross(pc) + pc.cross(pa);
          system.triadBuffer.add(triad);
        }
      }
    }
  }
}

class SolveCollisionCallback implements QueryCallback {
  ParticleSystem system;
  TimeStep step;

  final RayCastInput input = RayCastInput();
  final RayCastOutput output = RayCastOutput();

  @override
  bool reportFixture(Fixture fixture) {
    if (fixture.isSensor()) {
      return true;
    }
    final Shape shape = fixture.shape;
    final Body body = fixture.body;
    final int childCount = shape.getChildCount();
    for (int childIndex = 0; childIndex < childCount; childIndex++) {
      final AABB aabb = fixture.getAABB(childIndex);
      final particleDiameter = system.particleDiameter;
      final double aabbLowerBoundx = aabb.lowerBound.x - particleDiameter;
      final double aabbLowerBoundy = aabb.lowerBound.y - particleDiameter;
      final double aabbUpperBoundx = aabb.upperBound.x + particleDiameter;
      final double aabbUpperBoundy = aabb.upperBound.y + particleDiameter;
      final int firstProxy = ParticleSystem._lowerBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbLowerBoundx,
          system.inverseDiameter * aabbLowerBoundy,
        ),
      );
      final int lastProxy = ParticleSystem._upperBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbUpperBoundx,
          system.inverseDiameter * aabbUpperBoundy,
        ),
      );

      for (int i = firstProxy; i != lastProxy; ++i) {
        final particle = system.proxyBuffer[i].particle;
        final Vector2 ap = particle.position;
        if (aabbLowerBoundx <= ap.x &&
            ap.x <= aabbUpperBoundx &&
            aabbLowerBoundy <= ap.y &&
            ap.y <= aabbUpperBoundy) {
          final Vector2 av = particle.velocity;
          final Vector2 temp = Transform.mulTransVec2(body._xf0, ap);
          input.p1.setFrom(Transform.mulVec2(body._transform, temp));
          input.p2.x = ap.x + step.dt * av.x;
          input.p2.y = ap.y + step.dt * av.y;
          input.maxFraction = 1.0;
          if (fixture.raycast(output, input, childIndex)) {
            final Vector2 p = Vector2(
              (1 - output.fraction) * input.p1.x +
                  output.fraction * input.p2.x +
                  settings.linearSlop * output.normal.x,
              (1 - output.fraction) * input.p1.y +
                  output.fraction * input.p2.y +
                  settings.linearSlop * output.normal.y,
            );

            final double vx = step.invDt * (p.x - ap.x);
            final double vy = step.invDt * (p.y - ap.y);
            av.x = vx;
            av.y = vy;
            final double particleMass = system.getParticleMass();
            final double ax = particleMass * (av.x - vx);
            final double ay = particleMass * (av.y - vy);
            final Vector2 b = output.normal;
            final double fdn = ax * b.x + ay * b.y;
            final Vector2 f = Vector2(fdn * b.x, fdn * b.y);
            body.applyLinearImpulse(f, point: p);
          }
        }
      }
    }
    return true;
  }
}
