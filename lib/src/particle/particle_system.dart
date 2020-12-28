part of forge2d;

class ParticleBuffer<T> {
  final List<T> data = [];
  final AllocClosure<T> allocClosure;
  int userSuppliedCapacity = 0;
  ParticleBuffer(this.allocClosure);
}

typedef T AllocClosure<T>();

class ParticleBufferInt {
  final List<int> data = [];
  int userSuppliedCapacity = 0;
}

/// Connection between two particles
class PsPair {
  int indexA = 0;
  int indexB = 0;
  int flags = 0;
  double strength = 0.0;
  double distance = 0.0;
}

/// Connection between three particles
class PsTriad {
  int indexA = 0, indexB = 0, indexC = 0;
  int flags = 0;
  double strength = 0.0;
  final Vector2 pa = Vector2.zero(), pb = Vector2.zero(), pc = Vector2.zero();
  double ka = 0.0, kb = 0.0, kc = 0.0, s = 0.0;
}

/// Used for detecting particle contacts
class PsProxy implements Comparable<PsProxy> {
  int index = 0;
  int tag = 0;

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
  bool reportParticle(int index) {
    assert(index >= 0 && index < system.count);
    if (shape.testPoint(xf, system.positionBuffer.data[index])) {
      system.destroyParticle(index, callDestructionListener);
      destroyed++;
    }
    return true;
  }
}

class UpdateBodyContactsCallback implements QueryCallback {
  ParticleSystem system;

  final Vector2 _tempVec = Vector2.zero();

  static ParticleBodyContact allocParticleBodyContact() =>
      ParticleBodyContact();

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

      for (int proxy = firstProxy; proxy != lastProxy; ++proxy) {
        final int a = system.proxyBuffer[proxy].index;
        final Vector2 ap = system.positionBuffer.data[a];
        if (aabbLowerBoundX <= ap.x &&
            ap.x <= aabbUpperBoundX &&
            aabbLowerBoundY <= ap.y &&
            ap.y <= aabbUpperBoundY) {
          double d;
          final Vector2 n = _tempVec;
          d = fixture.computeDistance(ap, childIndex, n);
          if (d < system.particleDiameter) {
            final double invAm =
                (system.flagsBuffer.data[a] & ParticleType.wallParticle) != 0
                    ? 0.0
                    : system.getParticleInvMass();
            final double rpx = ap.x - bp.x;
            final double rpy = ap.y - bp.y;
            final double rpn = rpx * n.y - rpy * n.x;
            final ParticleBodyContact contact = ParticleBodyContact();
            contact.index = a;
            contact.body = b;
            contact.weight = 1 - d * system.inverseDiameter;
            contact.normal.x = -n.x;
            contact.normal.y = -n.y;
            contact.mass = 1 / (invAm + invBm + invBI * rpn * rpn);
            system.bodyContactBuffer.add(contact);
          }
        }
      }
    }
    return true;
  }
}

PsTriad allocPsTriad() => PsTriad();

// Callback used with VoronoiDiagram.
class CreateParticleGroupCallback implements VoronoiDiagramCallback {
  @override
  void call(int a, int b, int c) {
    final Vector2 pa = system.positionBuffer.data[a];
    final Vector2 pb = system.positionBuffer.data[b];
    final Vector2 pc = system.positionBuffer.data[c];
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
      final PsTriad triad = PsTriad()
        ..indexA = a
        ..indexB = b
        ..indexC = c
        ..flags = system.flagsBuffer.data[a] |
            system.flagsBuffer.data[b] |
            system.flagsBuffer.data[c]
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

  ParticleSystem system;
  ParticleGroupDef def;
  int firstIndex;
}

// Callback used with VoronoiDiagram.
class JoinParticleGroupsCallback implements VoronoiDiagramCallback {
  @override
  void call(int a, int b, int c) {
    // Create a triad if it will contain particles from both groups.
    final int countA = ((a < groupB._firstIndex) ? 1 : 0) +
        ((b < groupB._firstIndex) ? 1 : 0) +
        ((c < groupB._firstIndex) ? 1 : 0);
    if (countA > 0 && countA < 3) {
      final int af = system.flagsBuffer.data[a];
      final int bf = system.flagsBuffer.data[b];
      final int cf = system.flagsBuffer.data[c];
      if ((af & bf & cf & ParticleSystem.k_triadFlags) != 0) {
        final Vector2 pa = system.positionBuffer.data[a];
        final Vector2 pb = system.positionBuffer.data[b];
        final Vector2 pc = system.positionBuffer.data[c];
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
          final PsTriad triad = PsTriad()
            ..indexA = a
            ..indexB = b
            ..indexC = c
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

  ParticleSystem system;
  ParticleGroup groupA;
  ParticleGroup groupB;
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
      final double aabblowerBoundx =
          aabb.lowerBound.x - system.particleDiameter;
      final double aabblowerBoundy =
          aabb.lowerBound.y - system.particleDiameter;
      final double aabbupperBoundx =
          aabb.upperBound.x + system.particleDiameter;
      final double aabbupperBoundy =
          aabb.upperBound.y + system.particleDiameter;
      final int firstProxy = ParticleSystem._lowerBound(
          system.proxyBuffer,
          ParticleSystem.computeTag(
            system.inverseDiameter * aabblowerBoundx,
              system.inverseDiameter * aabblowerBoundy,
          ),
      );
      final int lastProxy = ParticleSystem._upperBound(
          system.proxyBuffer,
          ParticleSystem.computeTag(
              system.inverseDiameter * aabbupperBoundx,
              system.inverseDiameter * aabbupperBoundy,
          ),
      );

      for (int proxy = firstProxy; proxy != lastProxy; ++proxy) {
        final int a = system.proxyBuffer[proxy].index;
        final Vector2 ap = system.positionBuffer.data[a];
        if (aabblowerBoundx <= ap.x &&
            ap.x <= aabbupperBoundx &&
            aabblowerBoundy <= ap.y &&
            ap.y <= aabbupperBoundy) {
          final Vector2 av = system.velocityBuffer.data[a];
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

class ParticleSystemTest {
  static bool isProxyInvalid(final PsProxy proxy) {
    return proxy.index < 0;
  }

  static bool isContactInvalid(final ParticleContact contact) {
    return contact.indexA < 0 || contact.indexB < 0;
  }

  static bool isBodyContactInvalid(final ParticleBodyContact contact) {
    return contact.index < 0;
  }

  static bool isPairInvalid(final PsPair pair) {
    return pair.indexA < 0.0 || pair.indexB < 0.0;
  }

  static bool isTriadInvalid(final PsTriad triad) {
    return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0;
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

  static int limitCapacity(int capacity, int maxCount) {
    return maxCount != 0 && capacity > maxCount ? maxCount : capacity;
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

  int count = 0;
  int internalAllocatedCapacity = 0;
  int maxCount = 0;
  ParticleBufferInt flagsBuffer;
  ParticleBuffer<Vector2> positionBuffer;
  ParticleBuffer<Vector2> velocityBuffer;

  Float64List accumulationBuffer; // temporary values
  List<Vector2> accumulation2Buffer; // temporary vector values
  Float64List depthBuffer; // distance from the surface

  ParticleBuffer<ParticleColor> colorBuffer;
  List<ParticleGroup> groupBuffer;
  ParticleBuffer<Object> userDataBuffer;

  List<PsProxy> proxyBuffer = [];
  List<ParticleContact> contactBuffer = [];
  List<ParticleBodyContact> bodyContactBuffer = [];
  List<PsPair> pairBuffer = [];
  List<PsTriad> triadBuffer = [];

  int groupCount = 0;
  ParticleGroup groupList;

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

  static Vector2 allocVec2() => Vector2.zero();
  static Object allocObject() => Object();
  static ParticleColor allocParticleColor() => ParticleColor();
  static ParticleGroup allocParticleGroup() => ParticleGroup();
  static PsProxy allocPsProxy() => PsProxy();

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

    flagsBuffer = ParticleBufferInt();
    positionBuffer = ParticleBuffer<Vector2>(allocVec2);
    velocityBuffer = ParticleBuffer<Vector2>(allocVec2);
    colorBuffer = ParticleBuffer<ParticleColor>(allocParticleColor);
    userDataBuffer = ParticleBuffer<Object>(allocObject);
  }

  int createParticle(ParticleDef def) {
    if (count >= internalAllocatedCapacity) {
      return settings.invalidParticleIndex;
    }
    final int index = count++;
    flagsBuffer.data.add(def.flags);
    positionBuffer.data.add(def.position.clone());
    velocityBuffer.data.add(def.velocity.clone());
    groupBuffer.add(ParticleGroup());
    depthBuffer?.add(0.0);
    if (def.color != null) {
      colorBuffer.data.add(ParticleColor()..setParticleColor(def.color));
    }
    if (def.userData != null) {
      userDataBuffer.data.add(def.userData);
    }
    proxyBuffer.add(PsProxy()..index = index);
    return index;
  }

  // reallocate a buffer
  static List<T> reallocateBuffer<T>(
      ParticleBuffer<T> buffer, int oldCapacity, int newCapacity) {
    assert(newCapacity > oldCapacity);
    return buffer_utils.reallocateBufferWithAlloc<T>(
      buffer.data,
      newCapacity,
      buffer.allocClosure,
    );
  }

  List<T> requestParticleBuffer<T>(T allocClosure()) {
    return List<T>.generate(
      internalAllocatedCapacity,
      (_) => allocClosure(),
    );
  }

  Float64List requestParticleBufferFloat64(Float64List buffer) {
    buffer ??= Float64List(internalAllocatedCapacity);
    return buffer;
  }

  void destroyParticle(int index, bool callDestructionListener) {
    int flags = ParticleType.zombieParticle;
    if (callDestructionListener) {
      flags |= ParticleType.destructionListener;
    }
    flagsBuffer.data[index] |= flags;
  }

  final AABB _temp = AABB();
  final DestroyParticlesInShapeCallback _dpcallback =
      DestroyParticlesInShapeCallback();

  int destroyParticlesInShape(
      Shape shape, Transform xf, bool callDestructionListener) {
    _dpcallback.init(this, shape, xf, callDestructionListener);
    shape.computeAABB(_temp, xf, 0);
    world.queryAABBParticle(_dpcallback, _temp);
    return _dpcallback.destroyed;
  }

  void destroyParticlesInGroup(
      ParticleGroup group, bool callDestructionListener) {
    for (int i = group._firstIndex; i < group._lastIndex; i++) {
      destroyParticle(i, callDestructionListener);
    }
  }

  final AABB _temp2 = AABB();
  final Vector2 _tempVec = Vector2.zero();
  final Transform _tempTransform = Transform.zero();
  final Transform _tempTransform2 = Transform.zero();
  final CreateParticleGroupCallback _createParticleGroupCallback =
      CreateParticleGroupCallback();
  final ParticleDef _tempParticleDef = ParticleDef();

  ParticleGroup createParticleGroup(ParticleGroupDef groupDef) {
    final double stride = getParticleStride();
    final Transform identity = _tempTransform;
    identity.setIdentity();
    final Transform transform = _tempTransform2;
    transform.setIdentity();
    final int firstIndex = count;
    if (groupDef.shape != null) {
      final ParticleDef particleDef = _tempParticleDef;
      particleDef.flags = groupDef.flags;
      particleDef.color = groupDef.color;
      particleDef.userData = groupDef.userData;
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
          final Vector2 p = _tempVec;
          p.x = x;
          p.y = y;
          if (shape.testPoint(identity, p)) {
            p.setFrom(Transform.mulVec2(transform, p));
            particleDef.position.x = p.x;
            particleDef.position.y = p.y;
            p.sub(groupDef.position);
            p.scaleOrthogonalInto(
                groupDef.angularVelocity, particleDef.velocity);
            particleDef.velocity.add(groupDef.linearVelocity);
            createParticle(particleDef);
          }
        }
      }
    }
    final int lastIndex = count;

    final ParticleGroup group = ParticleGroup();
    group._system = this;
    group._firstIndex = firstIndex;
    group._lastIndex = lastIndex;
    group._groupFlags = groupDef.groupFlags;
    group._strength = groupDef.strength;
    group._userData = groupDef.userData;
    group._transform.set(transform);
    group._destroyAutomatically = groupDef.destroyAutomatically;
    group._prev = null;
    group._next = groupList;
    if (groupList != null) {
      groupList._prev = group;
    }
    groupList = group;
    ++groupCount;
    for (int i = firstIndex; i < lastIndex; i++) {
      groupBuffer[i] = group;
    }

    updateContacts(true);
    if ((groupDef.flags & k_pairFlags) != 0) {
      for (ParticleContact contact in contactBuffer) {
        int a = contact.indexA;
        int b = contact.indexB;
        if (a > b) {
          final int temp = a;
          a = b;
          b = temp;
        }
        if (firstIndex <= a && b < lastIndex) {
          final PsPair pair = PsPair()
          ..indexA = a
          ..indexB = b
          ..flags = contact.flags
          ..strength = groupDef.strength
          ..distance =
              positionBuffer.data[a].distanceTo(positionBuffer.data[b]);
          pairBuffer.add(pair);
        }
      }
    }
    if ((groupDef.flags & k_triadFlags) != 0) {
      final VoronoiDiagram diagram = VoronoiDiagram();
      for (int i = firstIndex; i < lastIndex; i++) {
        diagram.addGenerator(positionBuffer.data[i], i);
      }
      diagram.generate(stride / 2);
      _createParticleGroupCallback.system = this;
      _createParticleGroupCallback.def = groupDef;
      _createParticleGroupCallback.firstIndex = firstIndex;
      diagram.getNodes(_createParticleGroupCallback);
    }
    if ((groupDef.groupFlags & ParticleGroupType.solidParticleGroup) != 0) {
      computeDepthForGroup(group);
    }

    return group;
  }

  static PsPair allocPsPair() => PsPair();

  void joinParticleGroups(ParticleGroup groupA, ParticleGroup groupB) {
    assert(groupA != groupB);
    rotateBuffer(groupB._firstIndex, groupB._lastIndex, count);
    assert(groupB._lastIndex == count);
    rotateBuffer(groupA._firstIndex, groupA._lastIndex, groupB._firstIndex);
    assert(groupA._lastIndex == groupB._firstIndex);

    int particleFlags = 0;
    for (int i = groupA._firstIndex; i < groupB._lastIndex; i++) {
      particleFlags |= flagsBuffer.data[i];
    }

    updateContacts(true);
    if ((particleFlags & k_pairFlags) != 0) {
      for (ParticleContact contact in contactBuffer) {
        int a = contact.indexA;
        int b = contact.indexB;
        if (a > b) {
          final int temp = a;
          a = b;
          b = temp;
        }
        if (groupA._firstIndex <= a &&
            a < groupA._lastIndex &&
            groupB._firstIndex <= b &&
            b < groupB._lastIndex) {
          final PsPair pair = PsPair()
            ..indexA = a
            ..indexB = b
            ..flags = contact.flags
            ..strength = math.min(groupA._strength, groupB._strength)
            ..distance =
            positionBuffer.data[a].distanceTo(positionBuffer.data[b]);
          pairBuffer.add(pair);
        }
      }
    }
    if ((particleFlags & k_triadFlags) != 0) {
      final VoronoiDiagram diagram = VoronoiDiagram();
      for (int i = groupA._firstIndex; i < groupB._lastIndex; i++) {
        if ((flagsBuffer.data[i] & ParticleType.zombieParticle) == 0) {
          diagram.addGenerator(positionBuffer.data[i], i);
        }
      }
      diagram.generate(getParticleStride() / 2);
      final JoinParticleGroupsCallback callback = JoinParticleGroupsCallback();
      callback.system = this;
      callback.groupA = groupA;
      callback.groupB = groupB;
      diagram.getNodes(callback);
    }

    for (int i = groupB._firstIndex; i < groupB._lastIndex; i++) {
      groupBuffer[i] = groupA;
    }
    final int groupFlags = groupA._groupFlags | groupB._groupFlags;
    groupA._groupFlags = groupFlags;
    groupA._lastIndex = groupB._lastIndex;
    groupB._firstIndex = groupB._lastIndex;
    destroyParticleGroup(groupB);

    if ((groupFlags & ParticleGroupType.solidParticleGroup) != 0) {
      computeDepthForGroup(groupA);
    }
  }

  // Only called from solveZombie() or joinParticleGroups().
  void destroyParticleGroup(ParticleGroup group) {
    assert(groupCount > 0);
    assert(group != null);

    if (world.getParticleDestructionListener() != null) {
      world.getParticleDestructionListener().sayGoodbyeParticleGroup(group);
    }

    for (int i = group._firstIndex; i < group._lastIndex; i++) {
      groupBuffer[i] = null;
    }

    if (group._prev != null) {
      group._prev._next = group._next;
    }
    if (group._next != null) {
      group._next._prev = group._prev;
    }
    if (group == groupList) {
      groupList = group._next;
    }

    --groupCount;
  }

  void computeDepthForGroup(ParticleGroup group) {
    for (int i = group._firstIndex; i < group._lastIndex; i++) {
      accumulationBuffer[i] = 0.0;
    }
    for (ParticleContact contact in contactBuffer) {
      final int a = contact.indexA;
      final int b = contact.indexB;
      if (a >= group._firstIndex &&
          a < group._lastIndex &&
          b >= group._firstIndex &&
          b < group._lastIndex) {
        final double w = contact.weight;
        accumulationBuffer[a] += w;
        accumulationBuffer[b] += w;
      }
    }
    depthBuffer = requestParticleBufferFloat64(depthBuffer);
    for (int i = group._firstIndex; i < group._lastIndex; i++) {
      final double w = accumulationBuffer[i];
      depthBuffer[i] = w < 0.8 ? 0.0 : double.maxFinite;
    }
    final int interationCount = group.getParticleCount();
    for (int t = 0; t < interationCount; t++) {
      bool updated = false;
      for (ParticleContact contact in contactBuffer) {
        final int a = contact.indexA;
        final int b = contact.indexB;
        if (a >= group._firstIndex &&
            a < group._lastIndex &&
            b >= group._firstIndex &&
            b < group._lastIndex) {
          final double r = 1 - contact.weight;
          final double ap0 = depthBuffer[a];
          final double bp0 = depthBuffer[b];
          final double ap1 = bp0 + r;
          final double bp1 = ap0 + r;
          if (ap0 > ap1) {
            depthBuffer[a] = ap1;
            updated = true;
          }
          if (bp0 > bp1) {
            depthBuffer[b] = bp1;
            updated = true;
          }
        }
      }
      if (!updated) {
        break;
      }
    }
    for (int i = group._firstIndex; i < group._lastIndex; i++) {
      if (depthBuffer[i] < double.maxFinite) {
        depthBuffer[i] *= particleDiameter;
      } else {
        depthBuffer[i] = 0.0;
      }
    }
  }

  static ParticleContact allocParticleContact() => ParticleContact();

  void addContact(int a, int b) {
    assert(a != b);
    final Vector2 pa = positionBuffer.data[a];
    final Vector2 pb = positionBuffer.data[b];
    final double dx = pb.x - pa.x;
    final double dy = pb.y - pa.y;
    final double d2 = dx * dx + dy * dy;
    if (d2 < squaredDiameter) {
      final double invD = d2 != 0 ? math.sqrt(1 / d2) : double.maxFinite;
      final ParticleContact contact = ParticleContact()
        ..indexA = a
        ..indexB = b
        ..flags = flagsBuffer.data[a] | flagsBuffer.data[b]
        ..weight = 1 - d2 * invD * inverseDiameter
        ..normal.x = invD * dx
        ..normal.y = invD * dy;
      contactBuffer.add(contact);
    }
  }

  void updateContacts(bool exceptZombie) {
    for (PsProxy proxy in proxyBuffer) {
      final int i = proxy.index;
      final Vector2 pos = positionBuffer.data[i];
      proxy.tag = computeTag(inverseDiameter * pos.x, inverseDiameter * pos.y);
    }
    proxyBuffer.sort();
    contactBuffer.clear();
    int cIndex = 0;
    for (int i = 0; i < proxyBuffer.length; i++) {
      final PsProxy a = proxyBuffer[i];
      final int rightTag = computeRelativeTag(a.tag, 1, 0);
      for (int j = i + 1; j < proxyBuffer.length; j++) {
        final PsProxy b = proxyBuffer[j];
        if (rightTag < b.tag) {
          break;
        }
        addContact(a.index, b.index);
      }
      final int bottomLeftTag = computeRelativeTag(a.tag, -1, 1);
      for (; cIndex < proxyBuffer.length; cIndex++) {
        final PsProxy c = proxyBuffer[cIndex];
        if (bottomLeftTag <= c.tag) {
          break;
        }
      }
      final int bottomRightTag = computeRelativeTag(a.tag, 1, 1);

      for (int bIndex = cIndex; bIndex < proxyBuffer.length; bIndex++) {
        final PsProxy b = proxyBuffer[bIndex];
        if (bottomRightTag < b.tag) {
          break;
        }
        addContact(a.index, b.index);
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
    for (Vector2 position in positionBuffer.data) {
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
    for (int i = 0; i < count; i++) {
      final Vector2 v = velocityBuffer.data[i];
      final Vector2 p1 = positionBuffer.data[i];
      final double p1x = p1.x;
      final double p1y = p1.y;
      final double p2x = p1x + step.dt * v.x;
      final double p2y = p1y + step.dt * v.y;
      final double bx = p1x < p2x ? p1x : p2x;
      final double by = p1y < p2y ? p1y : p2y;
      lowerBound.x = lowerBound.x < bx ? lowerBound.x : bx;
      lowerBound.y = lowerBound.y < by ? lowerBound.y : by;
      final double b1x = p1x > p2x ? p1x : p2x;
      final double b1y = p1y > p2y ? p1y : p2y;
      upperBound.x = upperBound.x > b1x ? upperBound.x : b1x;
      upperBound.y = upperBound.y > b1y ? upperBound.y : b1y;
    }
    _solveCollisionCallback.step = step;
    _solveCollisionCallback.system = this;
    world.queryAABB(_solveCollisionCallback, aabb);
  }

  void solve(TimeStep step) {
    ++timestamp;
    if (count == 0) {
      return;
    }
    allParticleFlags = 0;
    for (int i = 0; i < count; i++) {
      allParticleFlags |= flagsBuffer.data[i];
    }
    if ((allParticleFlags & ParticleType.zombieParticle) != 0) {
      solveZombie();
    }
    if (count == 0) {
      return;
    }
    allGroupFlags = 0;
    for (ParticleGroup group = groupList;
        group != null;
        group = group.getNext()) {
      allGroupFlags |= group._groupFlags;
    }
    final double gravityx = step.dt * gravityScale * world.getGravity().x;
    final double gravityy = step.dt * gravityScale * world.getGravity().y;
    final double criticalVelocitySquared = getCriticalVelocitySquared(step);
    for (int i = 0; i < count; i++) {
      final Vector2 v = velocityBuffer.data[i];
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
    for (int i = 0; i < count; i++) {
      final Vector2 pos = positionBuffer.data[i];
      final Vector2 vel = velocityBuffer.data[i];
      pos.x += step.dt * vel.x;
      pos.y += step.dt * vel.y;
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
    for (int i = 0; i < count; i++) {
      accumulationBuffer[i] = 0.0;
    }
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final int a = contact.index;
      final double w = contact.weight;
      accumulationBuffer[a] += w;
    }
    for (ParticleContact contact in contactBuffer) {
      final int a = contact.indexA;
      final int b = contact.indexB;
      final double w = contact.weight;
      accumulationBuffer[a] += w;
      accumulationBuffer[b] += w;
    }
    // ignores powder particles
    if ((allParticleFlags & k_noPressureFlags) != 0) {
      for (int i = 0; i < count; i++) {
        if ((flagsBuffer.data[i] & k_noPressureFlags) != 0) {
          accumulationBuffer[i] = 0.0;
        }
      }
    }
    // calculates pressure as a linear function of density
    final double pressurePerWeight =
        pressureStrength * getCriticalPressure(step);
    for (int i = 0; i < count; i++) {
      final double w = accumulationBuffer[i];
      final double h = pressurePerWeight *
          math.max(
              0.0,
              math.min(w, settings.maxParticleWeight) -
                  settings.minParticleWeight);
      accumulationBuffer[i] = h;
    }
    // applies pressure between each particles in contact
    final double velocityPerPressure = step.dt / (density * particleDiameter);
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final int a = contact.index;
      final Body b = contact.body;
      final double w = contact.weight;
      final double m = contact.mass;
      final Vector2 n = contact.normal;
      final Vector2 p = positionBuffer.data[a];
      final double h = accumulationBuffer[a] + pressurePerWeight * w;
      final Vector2 f = _tempVec;
      final double coef = velocityPerPressure * w * m * h;
      f.x = coef * n.x;
      f.y = coef * n.y;
      final Vector2 velData = velocityBuffer.data[a];
      final double particleInvMass = getParticleInvMass();
      velData.x -= particleInvMass * f.x;
      velData.y -= particleInvMass * f.y;
      b.applyLinearImpulse(f, point: p);
    }
    for (ParticleContact contact in contactBuffer) {
      final int a = contact.indexA;
      final int b = contact.indexB;
      final double w = contact.weight;
      final Vector2 n = contact.normal;
      final double h = accumulationBuffer[a] + accumulationBuffer[b];
      final double fx = velocityPerPressure * w * h * n.x;
      final double fy = velocityPerPressure * w * h * n.y;
      final Vector2 velDataA = velocityBuffer.data[a];
      final Vector2 velDataB = velocityBuffer.data[b];
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
      final int a = contact.index;
      final Body b = contact.body;
      final double w = contact.weight;
      final double m = contact.mass;
      final Vector2 n = contact.normal;
      final Vector2 p = positionBuffer.data[a];
      final double tempX = p.x - b._sweep.c.x;
      final double tempY = p.y - b._sweep.c.y;
      final Vector2 velA = velocityBuffer.data[a];
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
      final int a = contact.indexA;
      final int b = contact.indexB;
      final double w = contact.weight;
      final Vector2 n = contact.normal;
      final Vector2 velA = velocityBuffer.data[a];
      final Vector2 velB = velocityBuffer.data[b];
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
    for (int i = 0; i < count; i++) {
      if ((flagsBuffer.data[i] & ParticleType.wallParticle) != 0) {
        velocityBuffer.data[i].setFrom(Vector2.zero());
      }
    }
  }

  final Rot _tempRot = Rot();
  final Transform _tempXf = Transform.zero();
  final Transform _tempXf2 = Transform.zero();

  void solveRigid(final TimeStep step) {
    for (ParticleGroup group = groupList;
        group != null;
        group = group.getNext()) {
      if ((group._groupFlags & ParticleGroupType.rigidParticleGroup) != 0) {
        group.updateStatistics();
        final Vector2 temp = _tempVec;
        final Rot rotation = _tempRot;
        rotation.setAngle(step.dt * group._angularVelocity);
        final Vector2 cross = Rot.mulVec2(rotation, group._center);
        temp
          ..setFrom(group.linearVelocity)
          ..scale(step.dt)
          ..add(group._center)
          ..sub(cross);
        _tempXf.p.setFrom(temp);
        _tempXf.q.setFrom(rotation);
        group._transform.set(Transform.mul(_tempXf, group._transform));
        final Transform velocityTransform = _tempXf2;
        velocityTransform.p.x = step.invDt * _tempXf.p.x;
        velocityTransform.p.y = step.invDt * _tempXf.p.y;
        velocityTransform.q.s = step.invDt * _tempXf.q.s;
        velocityTransform.q.c = step.invDt * (_tempXf.q.c - 1);
        for (int i = group._firstIndex; i < group._lastIndex; i++) {
          velocityBuffer.data[i].setFrom(
            Transform.mulVec2(velocityTransform, positionBuffer.data[i]),
          );
        }
      }
    }
  }

  void solveElastic(final TimeStep step) {
    final double elasticStrength = step.invDt * this.elasticStrength;
    for (PsTriad triad in triadBuffer) {
      if ((triad.flags & ParticleType.elasticParticle) != 0) {
        final int a = triad.indexA;
        final int b = triad.indexB;
        final int c = triad.indexC;
        final Vector2 oa = triad.pa;
        final Vector2 ob = triad.pb;
        final Vector2 oc = triad.pc;
        final Vector2 pa = positionBuffer.data[a];
        final Vector2 pb = positionBuffer.data[b];
        final Vector2 pc = positionBuffer.data[c];
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
        final Vector2 va = velocityBuffer.data[a];
        final Vector2 vb = velocityBuffer.data[b];
        final Vector2 vc = velocityBuffer.data[c];
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
        final int a = pair.indexA;
        final int b = pair.indexB;
        final Vector2 pa = positionBuffer.data[a];
        final Vector2 pb = positionBuffer.data[b];
        final double dx = pb.x - pa.x;
        final double dy = pb.y - pa.y;
        final double r0 = pair.distance;
        double r1 = math.sqrt(dx * dx + dy * dy);
        r1 = r1 == 0 ? double.maxFinite : r1;
        final double strength = springStrength * pair.strength;
        final double fx = strength * (r0 - r1) / r1 * dx;
        final double fy = strength * (r0 - r1) / r1 * dy;
        final Vector2 va = velocityBuffer.data[a];
        final Vector2 vb = velocityBuffer.data[b];
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveTensile(final TimeStep step) {
    accumulation2Buffer ??= requestParticleBuffer(allocVec2);
    for (int i = 0; i < count; i++) {
      accumulationBuffer[i] = 0.0;
      accumulation2Buffer[i].setZero();
    }
    for (ParticleContact contact in contactBuffer) {
      if ((contact.flags & ParticleType.tensileParticle) != 0) {
        final int a = contact.indexA;
        final int b = contact.indexB;
        final double w = contact.weight;
        final Vector2 n = contact.normal;
        accumulationBuffer[a] += w;
        accumulationBuffer[b] += w;
        final Vector2 a2A = accumulation2Buffer[a];
        final Vector2 a2B = accumulation2Buffer[b];
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
        final int a = contact.indexA;
        final int b = contact.indexB;
        final double w = contact.weight;
        final Vector2 n = contact.normal;
        final Vector2 a2A = accumulation2Buffer[a];
        final Vector2 a2B = accumulation2Buffer[b];
        final double h = accumulationBuffer[a] + accumulationBuffer[b];
        final double sx = a2B.x - a2A.x;
        final double sy = a2B.y - a2A.y;
        final double fn =
            (strengthA * (h - 2) + strengthB * (sx * n.x + sy * n.y)) * w;
        final double fx = fn * n.x;
        final double fy = fn * n.y;
        final Vector2 va = velocityBuffer.data[a];
        final Vector2 vb = velocityBuffer.data[b];
        va.x -= fx;
        va.y -= fy;
        vb.x += fx;
        vb.y += fy;
      }
    }
  }

  void solveViscous(final TimeStep step) {
    for (ParticleBodyContact contact in bodyContactBuffer) {
      final int a = contact.index;
      if ((flagsBuffer.data[a] & ParticleType.viscousParticle) != 0) {
        final Body b = contact.body;
        final double w = contact.weight;
        final double m = contact.mass;
        final Vector2 p = positionBuffer.data[a];
        final Vector2 va = velocityBuffer.data[a];
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
        final int a = contact.indexA;
        final int b = contact.indexB;
        final double w = contact.weight;
        final Vector2 va = velocityBuffer.data[a];
        final Vector2 vb = velocityBuffer.data[b];
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
      final int a = contact.index;
      if ((flagsBuffer.data[a] & ParticleType.powderParticle) != 0) {
        final double w = contact.weight;
        if (w > minWeight) {
          final Body b = contact.body;
          final double m = contact.mass;
          final Vector2 p = positionBuffer.data[a];
          final Vector2 n = contact.normal;
          final Vector2 f = _tempVec;
          final Vector2 va = velocityBuffer.data[a];
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
          final int a = contact.indexA;
          final int b = contact.indexB;
          final Vector2 n = contact.normal;
          final Vector2 va = velocityBuffer.data[a];
          final Vector2 vb = velocityBuffer.data[b];
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
    depthBuffer = requestParticleBufferFloat64(depthBuffer);
    final double ejectionStrength = step.invDt * this.ejectionStrength;
    for (ParticleContact contact in contactBuffer) {
      final int a = contact.indexA;
      final int b = contact.indexB;
      if (groupBuffer[a] != groupBuffer[b]) {
        final double w = contact.weight;
        final Vector2 n = contact.normal;
        final double h = depthBuffer[a] + depthBuffer[b];
        final Vector2 va = velocityBuffer.data[a];
        final Vector2 vb = velocityBuffer.data[b];
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
      final int a = contact.indexA;
      final int b = contact.indexB;
      if ((flagsBuffer.data[a] &
              flagsBuffer.data[b] &
              ParticleType.colorMixingParticle) !=
          0) {
        final ParticleColor colorA = colorBuffer.data[a];
        final ParticleColor colorB = colorBuffer.data[b];
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
    int newCount = 0;
    final List<int> newIndices = List<int>.filled(count, 0);
    for (int i = 0; i < count; i++) {
      final int flags = flagsBuffer.data[i];
      if ((flags & ParticleType.zombieParticle) != 0) {
        final ParticleDestructionListener destructionListener =
            world.getParticleDestructionListener();
        if ((flags & ParticleType.destructionListener) != 0 &&
            destructionListener != null) {
          destructionListener.sayGoodbyeIndex(i);
        }
        newIndices[i] = settings.invalidParticleIndex;
      } else {
        newIndices[i] = newCount;
        if (i != newCount) {
          flagsBuffer.data[newCount] = flagsBuffer.data[i];
          positionBuffer.data[newCount].setFrom(positionBuffer.data[i]);
          velocityBuffer.data[newCount].setFrom(velocityBuffer.data[i]);
          groupBuffer[newCount] = groupBuffer[i];
          if (depthBuffer != null) {
            depthBuffer[newCount] = depthBuffer[i];
          }
          if (colorBuffer.data != null) {
            colorBuffer.data[newCount].setParticleColor(colorBuffer.data[i]);
          }
          if (userDataBuffer.data != null) {
            userDataBuffer.data[newCount] = userDataBuffer.data[i];
          }
        }
        newCount++;
      }
    }

    // update proxies
    proxyBuffer.forEach((proxy) => proxy.index = newIndices[proxy.index]);

    int j = proxyBuffer.length;
    for (int i = 0; i < j; i++) {
      if (ParticleSystemTest.isProxyInvalid(proxyBuffer[i])) {
        --j;
        final PsProxy temp = proxyBuffer[j];
        proxyBuffer[j] = proxyBuffer[i];
        proxyBuffer[i] = temp;
        --i;
      }
    }

    // update contacts
    for (ParticleContact contact in contactBuffer) {
      contact.indexA = newIndices[contact.indexA];
      contact.indexB = newIndices[contact.indexB];
    }

    j = contactBuffer.length;
    for (int i = 0; i < j; i++) {
      if (ParticleSystemTest.isContactInvalid(contactBuffer[i])) {
        --j;
        final ParticleContact temp = contactBuffer[j];
        contactBuffer[j] = contactBuffer[i];
        contactBuffer[i] = temp;
        --i;
      }
    }

    // update particle-body contacts
    bodyContactBuffer.forEach((c) => c.index = newIndices[c.index]);

    j = bodyContactBuffer.length;
    for (int i = 0; i < j; i++) {
      if (ParticleSystemTest.isBodyContactInvalid(bodyContactBuffer[i])) {
        --j;
        final ParticleBodyContact temp = bodyContactBuffer[j];
        bodyContactBuffer[j] = bodyContactBuffer[i];
        bodyContactBuffer[i] = temp;
        --i;
      }
    }

    // update pairs
    for (PsPair pair in pairBuffer) {
      pair.indexA = newIndices[pair.indexA];
      pair.indexB = newIndices[pair.indexB];
    }

    j = pairBuffer.length;
    for (int i = 0; i < j; i++) {
      if (ParticleSystemTest.isPairInvalid(pairBuffer[i])) {
        --j;
        final PsPair temp = pairBuffer[j];
        pairBuffer[j] = pairBuffer[i];
        pairBuffer[i] = temp;
        --i;
      }
    }

    // update triads
    for (PsTriad triad in triadBuffer) {
      triad.indexA = newIndices[triad.indexA];
      triad.indexB = newIndices[triad.indexB];
      triad.indexC = newIndices[triad.indexC];
    }

    j = triadBuffer.length;
    for (int i = 0; i < j; i++) {
      if (ParticleSystemTest.isTriadInvalid(triadBuffer[i])) {
        --j;
        final PsTriad temp = triadBuffer[j];
        triadBuffer[j] = triadBuffer[i];
        triadBuffer[i] = temp;
        --i;
      }
    }

    // update groups
    for (ParticleGroup group = groupList;
        group != null;
        group = group.getNext()) {
      int firstIndex = newCount;
      int lastIndex = 0;
      bool modified = false;
      for (int i = group._firstIndex; i < group._lastIndex; i++) {
        j = newIndices[i];
        if (j >= 0) {
          firstIndex = math.min(firstIndex, j);
          lastIndex = math.max(lastIndex, j + 1);
        } else {
          modified = true;
        }
      }
      if (firstIndex < lastIndex) {
        group._firstIndex = firstIndex;
        group._lastIndex = lastIndex;
        if (modified) {
          if ((group._groupFlags & ParticleGroupType.rigidParticleGroup) != 0) {
            group._toBeSplit = true;
          }
        }
      } else {
        group._firstIndex = 0;
        group._lastIndex = 0;
        if (group._destroyAutomatically) {
          group._toBeDestroyed = true;
        }
      }
    }

    // update particle count
    count = newCount;
    // _world._stackAllocator.Free(newIndices);

    // destroy bodies with no particles
    for (ParticleGroup group = groupList; group != null;) {
      final ParticleGroup next = group.getNext();
      if (group._toBeDestroyed) {
        destroyParticleGroup(group);
      } else if (group._toBeSplit) {
        // TODO: split the group
      }
      group = next;
    }
  }

  final NewIndices _newIndices = NewIndices();

  void rotateBuffer(int start, int mid, int end) {
    // move the particles assigned to the given group toward the end of array
    if (start == mid || mid == end) {
      return;
    }
    _newIndices.start = start;
    _newIndices.mid = mid;
    _newIndices.end = end;

    buffer_utils.rotate(flagsBuffer.data, start, mid, end);
    buffer_utils.rotate(positionBuffer.data, start, mid, end);
    buffer_utils.rotate(velocityBuffer.data, start, mid, end);
    buffer_utils.rotate(groupBuffer, start, mid, end);
    if (depthBuffer != null) {
      buffer_utils.rotate(depthBuffer, start, mid, end);
    }
    if (colorBuffer.data != null) {
      buffer_utils.rotate(colorBuffer.data, start, mid, end);
    }
    if (userDataBuffer.data != null) {
      buffer_utils.rotate(userDataBuffer.data, start, mid, end);
    }

    // update proxies
    for (PsProxy proxy in proxyBuffer) {
      proxy.index = _newIndices.getIndex(proxy.index);
    }

    // update contacts
    for (ParticleContact contact in contactBuffer) {
      contact.indexA = _newIndices.getIndex(contact.indexA);
      contact.indexB = _newIndices.getIndex(contact.indexB);
    }

    // update particle-body contacts
    for (ParticleBodyContact contact in bodyContactBuffer) {
      contact.index = _newIndices.getIndex(contact.index);
    }

    // update pairs
    for (PsPair pair in pairBuffer) {
      pair.indexA = _newIndices.getIndex(pair.indexA);
      pair.indexB = _newIndices.getIndex(pair.indexB);
    }

    // update triads
    for (PsTriad triad in triadBuffer) {
      triad.indexA = _newIndices.getIndex(triad.indexA);
      triad.indexB = _newIndices.getIndex(triad.indexB);
      triad.indexC = _newIndices.getIndex(triad.indexC);
    }

    // update groups
    for (ParticleGroup group = groupList;
        group != null;
        group = group.getNext()) {
      group._firstIndex = _newIndices.getIndex(group._firstIndex);
      group._lastIndex = _newIndices.getIndex(group._lastIndex - 1) + 1;
    }
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

  List<int> getParticleFlagsBuffer() {
    return flagsBuffer.data;
  }

  List<Vector2> getParticlePositionBuffer() {
    return positionBuffer.data;
  }

  List<Vector2> getParticleVelocityBuffer() {
    return velocityBuffer.data;
  }

  List<ParticleColor> getParticleColorBuffer() {
    return colorBuffer.data;
  }

  List<Object> getParticleUserDataBuffer() {
    return userDataBuffer.data;
  }

  List<ParticleGroup> getParticleGroupBuffer() {
    return groupBuffer;
  }

  int getParticleGroupCount() {
    return groupCount;
  }

  List<ParticleGroup> getParticleGroupList() {
    return groupBuffer;
  }

  int getParticleCount() {
    return count;
  }

  static int _lowerBound(Iterable<PsProxy> ray, int tag) {
    return _bound(ray, tag, (int a, int b) => a < b);
  }

  static int _upperBound(Iterable<PsProxy> ray, int tag) {
    return _bound(ray, tag, (int a, int b) => a >= b);
  }

  static int _bound(Iterable<PsProxy> ray, int tag, bool compare(int a, int b)) {
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
    for (int proxy = firstProxy; proxy < lastProxy; ++proxy) {
      final int i = proxyBuffer[proxy].index;
      final Vector2 p = positionBuffer.data[i];
      if (lowerBoundX < p.x &&
          p.x < upperBoundX &&
          lowerBoundY < p.y &&
          p.y < upperBoundY) {
        if (!callback.reportParticle(i)) {
          break;
        }
      }
    }
  }

  /// @param callback
  /// @param point1
  /// @param point2
  void raycast(ParticleRaycastCallback callback, final Vector2 point1,
      final Vector2 point2) {
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
    for (int proxy = firstProxy; proxy < lastProxy; ++proxy) {
      final int i = proxyBuffer[proxy].index;
      final Vector2 posI = positionBuffer.data[i];
      final double px = point1.x - posI.x;
      final double py = point1.y - posI.y;
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
      final int a = contact.indexA;
      final int b = contact.indexB;
      final Vector2 n = contact.normal;
      final Vector2 va = velocityBuffer.data[a];
      final Vector2 vb = velocityBuffer.data[b];
      final double vx = vb.x - va.x;
      final double vy = vb.y - va.y;
      final double vn = vx * n.x + vy * n.y;
      if (vn < 0) {
        sumV2 += vn * vn;
      }
    }
    return 0.5 * getParticleMass() * sumV2;
  }
}
