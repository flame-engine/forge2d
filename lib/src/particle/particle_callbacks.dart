import 'dart:math';

import '../../forge2d.dart';
import '../settings.dart' as settings;

class DestroyParticlesInShapeCallback implements ParticleQueryCallback {
  final ParticleSystem system;
  final Shape shape;
  final Transform xf;
  bool callDestructionListener;
  int destroyed = 0;

  DestroyParticlesInShapeCallback(
    this.system,
    this.shape,
    this.xf, {
    this.callDestructionListener = false,
  });

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
  late ParticleSystem system;

  final Vector2 _tempVec = Vector2.zero();

  @override
  bool reportFixture(Fixture fixture) {
    if (fixture.isSensor) {
      return true;
    }
    final shape = fixture.shape;
    final b = fixture.body;
    final bp = b.worldCenter;
    final bm = b.mass;
    final bI = b.getInertia() - bm * b.getLocalCenter().length2;
    final invBm = bm > 0 ? 1 / bm : 0.0;
    final invBI = bI > 0 ? 1 / bI : 0.0;
    final childCount = shape.getChildCount();
    for (var childIndex = 0; childIndex < childCount; childIndex++) {
      final aabb = fixture.getAABB(childIndex);
      final aabbLowerBoundX = aabb.lowerBound.x - system.particleDiameter;
      final aabbLowerBoundY = aabb.lowerBound.y - system.particleDiameter;
      final aabbUpperBoundX = aabb.upperBound.x + system.particleDiameter;
      final aabbUpperBoundY = aabb.upperBound.y + system.particleDiameter;
      final firstProxy = ParticleSystem.lowerBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbLowerBoundX,
          system.inverseDiameter * aabbLowerBoundY,
        ),
      );
      final lastProxy = ParticleSystem.upperBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbUpperBoundX,
          system.inverseDiameter * aabbUpperBoundY,
        ),
      );

      //print("first: $firstProxy");
      //print("last: $lastProxy");
      //print("size of proxies: ${system.proxyBuffer.length}");
      //print("size of particles: ${system.particles.length}");
      for (var i = firstProxy; i < lastProxy; ++i) {
        final particle = system.proxyBuffer[i].particle;
        final ap = particle.position;
        if (aabbLowerBoundX <= ap.x &&
            ap.x <= aabbUpperBoundX &&
            aabbLowerBoundY <= ap.y &&
            ap.y <= aabbUpperBoundY) {
          double d;
          final n = _tempVec;
          d = fixture.computeDistance(ap, childIndex, n);
          if (d < system.particleDiameter) {
            final invAm = (particle.flags & ParticleType.wallParticle) != 0
                ? 0.0
                : system.particleInverseMass;
            final rpx = ap.x - bp.x;
            final rpy = ap.y - bp.y;
            final rpn = rpx * n.y - rpy * n.x;
            final contact = ParticleBodyContact(particle)
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
  late ParticleSystem system;
  late ParticleGroupDef def;

  @override
  void call(Particle particleA, Particle particleB, Particle particleC) {
    final pa = particleA.position;
    final pb = particleB.position;
    final pc = particleC.position;
    final dabx = pa.x - pb.x;
    final daby = pa.y - pb.y;
    final dbcx = pb.x - pc.x;
    final dbcy = pb.y - pc.y;
    final dcax = pc.x - pa.x;
    final dcay = pc.y - pa.y;
    final maxDistanceSquared =
        settings.maxTriadDistanceSquared * system.squaredDiameter;
    if (dabx * dabx + daby * daby < maxDistanceSquared &&
        dbcx * dbcx + dbcy * dbcy < maxDistanceSquared &&
        dcax * dcax + dcay * dcay < maxDistanceSquared) {
      final midPointX = 1.0 / 3.0 * (pa.x + pb.x + pc.x);
      final midPointY = 1.0 / 3.0 * (pa.y + pb.y + pc.y);
      final triad = PsTriad(particleA, particleB, particleC)
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
  late ParticleSystem system;
  late ParticleGroup groupA;
  late ParticleGroup groupB;

  @override
  void call(Particle particleA, Particle particleB, Particle particleC) {
    final callParticles = [particleA, particleB, particleC];
    bool hasCallParticle(ParticleGroup group) {
      return group.particles.any(callParticles.contains);
    }

    // Create a triad if it will contain particles from both groups.
    if (hasCallParticle(groupA) && hasCallParticle(groupB)) {
      final af = particleA.flags;
      final bf = particleB.flags;
      final cf = particleC.flags;
      if ((af & bf & cf & ParticleSystem.triadFlags) != 0) {
        final pa = particleA.position;
        final pb = particleB.position;
        final pc = particleC.position;
        final dabx = pa.x - pb.x;
        final daby = pa.y - pb.y;
        final dbcx = pb.x - pc.x;
        final dbcy = pb.y - pc.y;
        final dcax = pc.x - pa.x;
        final dcay = pc.y - pa.y;
        final maxDistanceSquared =
            settings.maxTriadDistanceSquared * system.squaredDiameter;
        if (dabx * dabx + daby * daby < maxDistanceSquared &&
            dbcx * dbcx + dbcy * dbcy < maxDistanceSquared &&
            dcax * dcax + dcay * dcay < maxDistanceSquared) {
          final midPointX = 1.0 / 3.0 * (pa.x + pb.x + pc.x);
          final midPointY = 1.0 / 3.0 * (pa.y + pb.y + pc.y);
          final triad = PsTriad(particleA, particleB, particleC)
            ..flags = af | bf | cf
            ..strength = min(groupA.strength, groupB.strength)
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
  late ParticleSystem system;
  late TimeStep step;

  final RayCastInput input = RayCastInput();
  final RayCastOutput output = RayCastOutput();

  @override
  bool reportFixture(Fixture fixture) {
    if (fixture.isSensor) {
      return true;
    }
    final shape = fixture.shape;
    final body = fixture.body;
    final childCount = shape.getChildCount();
    for (var childIndex = 0; childIndex < childCount; childIndex++) {
      final aabb = fixture.getAABB(childIndex);
      final particleDiameter = system.particleDiameter;
      final aabbLowerBoundx = aabb.lowerBound.x - particleDiameter;
      final aabbLowerBoundy = aabb.lowerBound.y - particleDiameter;
      final aabbUpperBoundx = aabb.upperBound.x + particleDiameter;
      final aabbUpperBoundy = aabb.upperBound.y + particleDiameter;
      final firstProxy = ParticleSystem.lowerBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbLowerBoundx,
          system.inverseDiameter * aabbLowerBoundy,
        ),
      );
      final lastProxy = ParticleSystem.upperBound(
        system.proxyBuffer,
        ParticleSystem.computeTag(
          system.inverseDiameter * aabbUpperBoundx,
          system.inverseDiameter * aabbUpperBoundy,
        ),
      );

      for (var i = firstProxy; i < lastProxy; ++i) {
        final particle = system.proxyBuffer[i].particle;
        final ap = particle.position;
        if (aabbLowerBoundx <= ap.x &&
            ap.x <= aabbUpperBoundx &&
            aabbLowerBoundy <= ap.y &&
            ap.y <= aabbUpperBoundy) {
          final av = particle.velocity;
          final temp = Transform.mulTransVec2(
            body.previousTransform,
            ap,
          );
          input.p1.setFrom(Transform.mulVec2(body.transform, temp));
          input.p2.x = ap.x + step.dt * av.x;
          input.p2.y = ap.y + step.dt * av.y;
          input.maxFraction = 1.0;
          if (fixture.raycast(output, input, childIndex)) {
            final p = Vector2(
              (1 - output.fraction) * input.p1.x +
                  output.fraction * input.p2.x +
                  settings.linearSlop * output.normal.x,
              (1 - output.fraction) * input.p1.y +
                  output.fraction * input.p2.y +
                  settings.linearSlop * output.normal.y,
            );

            final vx = step.invDt * (p.x - ap.x);
            final vy = step.invDt * (p.y - ap.y);
            av.x = vx;
            av.y = vy;
            final particleMass = system.particleMass;
            final ax = particleMass * (av.x - vx);
            final ay = particleMass * (av.y - vy);
            final b = output.normal;
            final fdn = ax * b.x + ay * b.y;
            final f = Vector2(fdn * b.x, fdn * b.y);
            body.applyLinearImpulse(f, point: p);
          }
        }
      }
    }
    return true;
  }
}
