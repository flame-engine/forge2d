import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:forge2d/src/backend/raw_box2d.dart';
import 'package:forge2d/src/ffi/box2d.g.dart' as b2;

/// Prepares the FFI backend. Nothing to do: the native library is bundled by
/// the build hook and loaded lazily.
Future<void> initializeBackend() async {}

/// Creates the FFI backend.
RawBox2D createRawBox2D() => RawBox2DFfi();

/// The dart:ffi implementation of [RawBox2D], calling the generated bindings
/// directly.
final class RawBox2DFfi implements RawBox2D {
  b2.b2WorldId _world(int id) => Struct.create<b2.b2WorldId>()
    ..index1 = id & 0xFFFF
    ..generation = (id >> 16) & 0xFFFF;

  int _packWorld(b2.b2WorldId id) => id.index1 | (id.generation << 16);

  b2.b2BodyId _body(int index1, int wg) => Struct.create<b2.b2BodyId>()
    ..index1 = index1
    ..world0 = wg & 0xFFFF
    ..generation = (wg >> 16) & 0xFFFF;

  (int, int) _packBody(b2.b2BodyId id) => (
    id.index1,
    id.world0 | (id.generation << 16),
  );

  b2.b2ShapeId _shape(int index1, int wg) => Struct.create<b2.b2ShapeId>()
    ..index1 = index1
    ..world0 = wg & 0xFFFF
    ..generation = (wg >> 16) & 0xFFFF;

  (int, int) _packShape(b2.b2ShapeId id) => (
    id.index1,
    id.world0 | (id.generation << 16),
  );

  b2.b2ChainId _chain(int index1, int wg) => Struct.create<b2.b2ChainId>()
    ..index1 = index1
    ..world0 = wg & 0xFFFF
    ..generation = (wg >> 16) & 0xFFFF;

  (int, int) _packChain(b2.b2ChainId id) => (
    id.index1,
    id.world0 | (id.generation << 16),
  );

  b2.b2Vec2 _vec2(double x, double y) => Struct.create<b2.b2Vec2>()
    ..x = x
    ..y = y;

  b2.b2Rot _rot(double cos, double sin) => Struct.create<b2.b2Rot>()
    ..c = cos
    ..s = sin;

  // World.

  @override
  int createWorld({
    required double gravityX,
    required double gravityY,
    required double restitutionThreshold,
    required double hitEventThreshold,
    required double contactHertz,
    required double contactDampingRatio,
    required double maxContactPushSpeed,
    required double maximumLinearSpeed,
    required bool enableSleep,
    required bool enableContinuous,
  }) {
    return using((arena) {
      final def = arena<b2.b2WorldDef>()..ref = b2.b2DefaultWorldDef();
      def.ref
        ..gravity.x = gravityX
        ..gravity.y = gravityY
        ..restitutionThreshold = restitutionThreshold
        ..hitEventThreshold = hitEventThreshold
        ..contactHertz = contactHertz
        ..contactDampingRatio = contactDampingRatio
        ..maxContactPushSpeed = maxContactPushSpeed
        ..maximumLinearSpeed = maximumLinearSpeed
        ..enableSleep = enableSleep
        ..enableContinuous = enableContinuous;
      return _packWorld(b2.b2CreateWorld(def));
    });
  }

  @override
  void destroyWorld(int worldId) => b2.b2DestroyWorld(_world(worldId));

  @override
  bool worldIsValid(int worldId) => b2.b2World_IsValid(_world(worldId));

  @override
  void worldStep(int worldId, double timeStep, int subStepCount) =>
      b2.b2World_Step(_world(worldId), timeStep, subStepCount);

  @override
  void worldSetGravity(int worldId, double x, double y) =>
      b2.b2World_SetGravity(_world(worldId), _vec2(x, y));

  @override
  (double, double) worldGetGravity(int worldId) {
    final gravity = b2.b2World_GetGravity(_world(worldId));
    return (gravity.x, gravity.y);
  }

  @override
  void worldEnableSleeping(int worldId, {required bool enabled}) =>
      b2.b2World_EnableSleeping(_world(worldId), enabled);

  @override
  bool worldIsSleepingEnabled(int worldId) =>
      b2.b2World_IsSleepingEnabled(_world(worldId));

  @override
  void worldEnableContinuous(int worldId, {required bool enabled}) =>
      b2.b2World_EnableContinuous(_world(worldId), enabled);

  @override
  bool worldIsContinuousEnabled(int worldId) =>
      b2.b2World_IsContinuousEnabled(_world(worldId));

  // Bodies.

  @override
  (int, int) createBody(
    int worldId, {
    required int type,
    required double positionX,
    required double positionY,
    required double rotationCos,
    required double rotationSin,
    required double linearVelocityX,
    required double linearVelocityY,
    required double angularVelocity,
    required double linearDamping,
    required double angularDamping,
    required double gravityScale,
    required double sleepThreshold,
    required String? name,
    required bool enableSleep,
    required bool isAwake,
    required bool fixedRotation,
    required bool isBullet,
    required bool isEnabled,
    required bool allowFastRotation,
  }) {
    return using((arena) {
      final def = arena<b2.b2BodyDef>()..ref = b2.b2DefaultBodyDef();
      def.ref
        ..typeAsInt = type
        ..position.x = positionX
        ..position.y = positionY
        ..rotation.c = rotationCos
        ..rotation.s = rotationSin
        ..linearVelocity.x = linearVelocityX
        ..linearVelocity.y = linearVelocityY
        ..angularVelocity = angularVelocity
        ..linearDamping = linearDamping
        ..angularDamping = angularDamping
        ..gravityScale = gravityScale
        ..sleepThreshold = sleepThreshold
        ..enableSleep = enableSleep
        ..isAwake = isAwake
        ..fixedRotation = fixedRotation
        ..isBullet = isBullet
        ..isEnabled = isEnabled
        ..allowFastRotation = allowFastRotation;
      if (name != null) {
        def.ref.name = name.toNativeUtf8(allocator: arena).cast();
      }
      return _packBody(b2.b2CreateBody(_world(worldId), def));
    });
  }

  @override
  void destroyBody(int index1, int wg) => b2.b2DestroyBody(_body(index1, wg));

  @override
  bool bodyIsValid(int index1, int wg) => b2.b2Body_IsValid(_body(index1, wg));

  @override
  (double, double) bodyGetPosition(int index1, int wg) {
    final position = b2.b2Body_GetPosition(_body(index1, wg));
    return (position.x, position.y);
  }

  @override
  (double, double) bodyGetRotation(int index1, int wg) {
    final rotation = b2.b2Body_GetRotation(_body(index1, wg));
    return (rotation.c, rotation.s);
  }

  @override
  void bodySetTransform(
    int index1,
    int wg,
    double positionX,
    double positionY,
    double rotationCos,
    double rotationSin,
  ) => b2.b2Body_SetTransform(
    _body(index1, wg),
    _vec2(positionX, positionY),
    _rot(rotationCos, rotationSin),
  );

  @override
  (double, double) bodyGetLinearVelocity(int index1, int wg) {
    final velocity = b2.b2Body_GetLinearVelocity(_body(index1, wg));
    return (velocity.x, velocity.y);
  }

  @override
  void bodySetLinearVelocity(int index1, int wg, double x, double y) =>
      b2.b2Body_SetLinearVelocity(_body(index1, wg), _vec2(x, y));

  @override
  double bodyGetAngularVelocity(int index1, int wg) =>
      b2.b2Body_GetAngularVelocity(_body(index1, wg));

  @override
  void bodySetAngularVelocity(int index1, int wg, double value) =>
      b2.b2Body_SetAngularVelocity(_body(index1, wg), value);

  @override
  void bodyApplyForce(
    int index1,
    int wg,
    double forceX,
    double forceY,
    double pointX,
    double pointY, {
    required bool wake,
  }) => b2.b2Body_ApplyForce(
    _body(index1, wg),
    _vec2(forceX, forceY),
    _vec2(pointX, pointY),
    wake,
  );

  @override
  void bodyApplyForceToCenter(
    int index1,
    int wg,
    double forceX,
    double forceY, {
    required bool wake,
  }) => b2.b2Body_ApplyForceToCenter(
    _body(index1, wg),
    _vec2(forceX, forceY),
    wake,
  );

  @override
  void bodyApplyTorque(
    int index1,
    int wg,
    double torque, {
    required bool wake,
  }) => b2.b2Body_ApplyTorque(_body(index1, wg), torque, wake);

  @override
  void bodyApplyLinearImpulse(
    int index1,
    int wg,
    double impulseX,
    double impulseY,
    double pointX,
    double pointY, {
    required bool wake,
  }) => b2.b2Body_ApplyLinearImpulse(
    _body(index1, wg),
    _vec2(impulseX, impulseY),
    _vec2(pointX, pointY),
    wake,
  );

  @override
  void bodyApplyLinearImpulseToCenter(
    int index1,
    int wg,
    double impulseX,
    double impulseY, {
    required bool wake,
  }) => b2.b2Body_ApplyLinearImpulseToCenter(
    _body(index1, wg),
    _vec2(impulseX, impulseY),
    wake,
  );

  @override
  void bodyApplyAngularImpulse(
    int index1,
    int wg,
    double impulse, {
    required bool wake,
  }) => b2.b2Body_ApplyAngularImpulse(_body(index1, wg), impulse, wake);

  @override
  double bodyGetMass(int index1, int wg) =>
      b2.b2Body_GetMass(_body(index1, wg));

  @override
  double bodyGetRotationalInertia(int index1, int wg) =>
      b2.b2Body_GetRotationalInertia(_body(index1, wg));

  @override
  (double, double) bodyGetLocalCenterOfMass(int index1, int wg) {
    final center = b2.b2Body_GetLocalCenterOfMass(_body(index1, wg));
    return (center.x, center.y);
  }

  @override
  (double, double) bodyGetWorldCenterOfMass(int index1, int wg) {
    final center = b2.b2Body_GetWorldCenterOfMass(_body(index1, wg));
    return (center.x, center.y);
  }

  @override
  void bodySetMassData(
    int index1,
    int wg,
    double mass,
    double rotationalInertia,
    double centerX,
    double centerY,
  ) {
    final massData = Struct.create<b2.b2MassData>()
      ..mass = mass
      ..rotationalInertia = rotationalInertia
      ..center.x = centerX
      ..center.y = centerY;
    b2.b2Body_SetMassData(_body(index1, wg), massData);
  }

  @override
  void bodyApplyMassFromShapes(int index1, int wg) =>
      b2.b2Body_ApplyMassFromShapes(_body(index1, wg));

  @override
  int bodyGetType(int index1, int wg) =>
      b2.b2Body_GetType(_body(index1, wg)).value;

  @override
  void bodySetType(int index1, int wg, int type) => b2.b2Body_SetType(
    _body(index1, wg),
    b2.b2BodyType.fromValue(type),
  );

  @override
  String bodyGetName(int index1, int wg) {
    final name = b2.b2Body_GetName(_body(index1, wg));
    return name == nullptr ? '' : name.cast<Utf8>().toDartString();
  }

  @override
  void bodySetName(int index1, int wg, String? name) {
    using((arena) {
      b2.b2Body_SetName(
        _body(index1, wg),
        name == null ? nullptr : name.toNativeUtf8(allocator: arena).cast(),
      );
    });
  }

  @override
  bool bodyIsAwake(int index1, int wg) => b2.b2Body_IsAwake(_body(index1, wg));

  @override
  void bodySetAwake(int index1, int wg, {required bool awake}) =>
      b2.b2Body_SetAwake(_body(index1, wg), awake);

  @override
  bool bodyIsSleepEnabled(int index1, int wg) =>
      b2.b2Body_IsSleepEnabled(_body(index1, wg));

  @override
  void bodyEnableSleep(int index1, int wg, {required bool enabled}) =>
      b2.b2Body_EnableSleep(_body(index1, wg), enabled);

  @override
  double bodyGetSleepThreshold(int index1, int wg) =>
      b2.b2Body_GetSleepThreshold(_body(index1, wg));

  @override
  void bodySetSleepThreshold(int index1, int wg, double value) =>
      b2.b2Body_SetSleepThreshold(_body(index1, wg), value);

  @override
  bool bodyIsEnabled(int index1, int wg) =>
      b2.b2Body_IsEnabled(_body(index1, wg));

  @override
  void bodyDisable(int index1, int wg) => b2.b2Body_Disable(_body(index1, wg));

  @override
  void bodyEnable(int index1, int wg) => b2.b2Body_Enable(_body(index1, wg));

  @override
  bool bodyIsFixedRotation(int index1, int wg) =>
      b2.b2Body_IsFixedRotation(_body(index1, wg));

  @override
  void bodySetFixedRotation(int index1, int wg, {required bool flag}) =>
      b2.b2Body_SetFixedRotation(_body(index1, wg), flag);

  @override
  bool bodyIsBullet(int index1, int wg) =>
      b2.b2Body_IsBullet(_body(index1, wg));

  @override
  void bodySetBullet(int index1, int wg, {required bool flag}) =>
      b2.b2Body_SetBullet(_body(index1, wg), flag);

  @override
  double bodyGetGravityScale(int index1, int wg) =>
      b2.b2Body_GetGravityScale(_body(index1, wg));

  @override
  void bodySetGravityScale(int index1, int wg, double scale) =>
      b2.b2Body_SetGravityScale(_body(index1, wg), scale);

  @override
  double bodyGetLinearDamping(int index1, int wg) =>
      b2.b2Body_GetLinearDamping(_body(index1, wg));

  @override
  void bodySetLinearDamping(int index1, int wg, double damping) =>
      b2.b2Body_SetLinearDamping(_body(index1, wg), damping);

  @override
  double bodyGetAngularDamping(int index1, int wg) =>
      b2.b2Body_GetAngularDamping(_body(index1, wg));

  @override
  void bodySetAngularDamping(int index1, int wg, double damping) =>
      b2.b2Body_SetAngularDamping(_body(index1, wg), damping);

  @override
  (double, double) bodyGetWorldPoint(int index1, int wg, double x, double y) {
    final point = b2.b2Body_GetWorldPoint(_body(index1, wg), _vec2(x, y));
    return (point.x, point.y);
  }

  @override
  (double, double) bodyGetLocalPoint(int index1, int wg, double x, double y) {
    final point = b2.b2Body_GetLocalPoint(_body(index1, wg), _vec2(x, y));
    return (point.x, point.y);
  }

  @override
  List<int> bodyGetShapes(int index1, int wg) {
    return using((arena) {
      final body = _body(index1, wg);
      final count = b2.b2Body_GetShapeCount(body);
      final shapes = arena<b2.b2ShapeId>(count);
      final written = b2.b2Body_GetShapes(body, shapes, count);
      return [
        for (var i = 0; i < written; i++) ...[
          shapes[i].index1,
          shapes[i].world0 | (shapes[i].generation << 16),
        ],
      ];
    });
  }

  // Shapes.

  Pointer<b2.b2ShapeDef> _shapeDef(Arena arena, RawShapeDef def) {
    final nativeDef = arena<b2.b2ShapeDef>()..ref = b2.b2DefaultShapeDef();
    nativeDef.ref
      ..material.friction = def.friction
      ..material.restitution = def.restitution
      ..material.rollingResistance = def.rollingResistance
      ..material.tangentSpeed = def.tangentSpeed
      ..material.userMaterialId = def.userMaterialId
      ..material.customColor = def.customColor
      ..density = def.density
      ..filter.categoryBits = def.categoryBits
      ..filter.maskBits = def.maskBits
      ..filter.groupIndex = def.groupIndex
      ..isSensor = def.isSensor
      ..enableSensorEvents = def.enableSensorEvents
      ..enableContactEvents = def.enableContactEvents
      ..enableHitEvents = def.enableHitEvents
      ..enablePreSolveEvents = def.enablePreSolveEvents
      ..invokeContactCreation = def.invokeContactCreation
      ..updateBodyMass = def.updateBodyMass;
    return nativeDef;
  }

  @override
  (int, int) createCircleShape(
    int bodyIndex1,
    int bodyWg, {
    required double centerX,
    required double centerY,
    required double radius,
    required RawShapeDef def,
  }) {
    return using((arena) {
      final circle = arena<b2.b2Circle>();
      circle.ref
        ..center.x = centerX
        ..center.y = centerY
        ..radius = radius;
      return _packShape(
        b2.b2CreateCircleShape(
          _body(bodyIndex1, bodyWg),
          _shapeDef(arena, def),
          circle,
        ),
      );
    });
  }

  @override
  (int, int) createCapsuleShape(
    int bodyIndex1,
    int bodyWg, {
    required double center1X,
    required double center1Y,
    required double center2X,
    required double center2Y,
    required double radius,
    required RawShapeDef def,
  }) {
    return using((arena) {
      final capsule = arena<b2.b2Capsule>();
      capsule.ref
        ..center1.x = center1X
        ..center1.y = center1Y
        ..center2.x = center2X
        ..center2.y = center2Y
        ..radius = radius;
      return _packShape(
        b2.b2CreateCapsuleShape(
          _body(bodyIndex1, bodyWg),
          _shapeDef(arena, def),
          capsule,
        ),
      );
    });
  }

  @override
  (int, int) createSegmentShape(
    int bodyIndex1,
    int bodyWg, {
    required double point1X,
    required double point1Y,
    required double point2X,
    required double point2Y,
    required RawShapeDef def,
  }) {
    return using((arena) {
      final segment = arena<b2.b2Segment>();
      segment.ref
        ..point1.x = point1X
        ..point1.y = point1Y
        ..point2.x = point2X
        ..point2.y = point2Y;
      return _packShape(
        b2.b2CreateSegmentShape(
          _body(bodyIndex1, bodyWg),
          _shapeDef(arena, def),
          segment,
        ),
      );
    });
  }

  @override
  (int, int) createBoxShape(
    int bodyIndex1,
    int bodyWg, {
    required double halfWidth,
    required double halfHeight,
    required double centerX,
    required double centerY,
    required double rotationCos,
    required double rotationSin,
    required double radius,
    required RawShapeDef def,
  }) {
    return using((arena) {
      final polygon = arena<b2.b2Polygon>()
        ..ref = b2.b2MakeOffsetRoundedBox(
          halfWidth,
          halfHeight,
          _vec2(centerX, centerY),
          _rot(rotationCos, rotationSin),
          radius,
        );
      return _packShape(
        b2.b2CreatePolygonShape(
          _body(bodyIndex1, bodyWg),
          _shapeDef(arena, def),
          polygon,
        ),
      );
    });
  }

  @override
  (int, int) createPolygonShape(
    int bodyIndex1,
    int bodyWg, {
    required List<double> points,
    required double radius,
    required RawShapeDef def,
  }) {
    return using((arena) {
      final count = points.length ~/ 2;
      final vertices = arena<b2.b2Vec2>(count);
      for (var i = 0; i < count; i++) {
        vertices[i]
          ..x = points[2 * i]
          ..y = points[2 * i + 1];
      }
      final hull = arena<b2.b2Hull>()..ref = b2.b2ComputeHull(vertices, count);
      if (hull.ref.count == 0) {
        throw ArgumentError(
          'The points do not form a valid convex hull: at least three '
          'distinct, non-collinear points are required',
        );
      }
      final polygon = arena<b2.b2Polygon>()
        ..ref = b2.b2MakePolygon(hull, radius);
      return _packShape(
        b2.b2CreatePolygonShape(
          _body(bodyIndex1, bodyWg),
          _shapeDef(arena, def),
          polygon,
        ),
      );
    });
  }

  @override
  void destroyShape(int index1, int wg, {required bool updateBodyMass}) =>
      b2.b2DestroyShape(_shape(index1, wg), updateBodyMass);

  @override
  bool shapeIsValid(int index1, int wg) =>
      b2.b2Shape_IsValid(_shape(index1, wg));

  @override
  int shapeGetType(int index1, int wg) =>
      b2.b2Shape_GetType(_shape(index1, wg)).value;

  @override
  (int, int) shapeGetBody(int index1, int wg) =>
      _packBody(b2.b2Shape_GetBody(_shape(index1, wg)));

  @override
  bool shapeIsSensor(int index1, int wg) =>
      b2.b2Shape_IsSensor(_shape(index1, wg));

  @override
  double shapeGetDensity(int index1, int wg) =>
      b2.b2Shape_GetDensity(_shape(index1, wg));

  @override
  void shapeSetDensity(
    int index1,
    int wg,
    double density, {
    required bool updateBodyMass,
  }) => b2.b2Shape_SetDensity(_shape(index1, wg), density, updateBodyMass);

  @override
  double shapeGetFriction(int index1, int wg) =>
      b2.b2Shape_GetFriction(_shape(index1, wg));

  @override
  void shapeSetFriction(int index1, int wg, double friction) =>
      b2.b2Shape_SetFriction(_shape(index1, wg), friction);

  @override
  double shapeGetRestitution(int index1, int wg) =>
      b2.b2Shape_GetRestitution(_shape(index1, wg));

  @override
  void shapeSetRestitution(int index1, int wg, double restitution) =>
      b2.b2Shape_SetRestitution(_shape(index1, wg), restitution);

  @override
  (int, int, int) shapeGetFilter(int index1, int wg) {
    final filter = b2.b2Shape_GetFilter(_shape(index1, wg));
    return (filter.categoryBits, filter.maskBits, filter.groupIndex);
  }

  @override
  void shapeSetFilter(
    int index1,
    int wg,
    int categoryBits,
    int maskBits,
    int groupIndex,
  ) {
    final filter = Struct.create<b2.b2Filter>()
      ..categoryBits = categoryBits
      ..maskBits = maskBits
      ..groupIndex = groupIndex;
    b2.b2Shape_SetFilter(_shape(index1, wg), filter);
  }

  @override
  bool shapeAreSensorEventsEnabled(int index1, int wg) =>
      b2.b2Shape_AreSensorEventsEnabled(_shape(index1, wg));

  @override
  void shapeEnableSensorEvents(int index1, int wg, {required bool enabled}) =>
      b2.b2Shape_EnableSensorEvents(_shape(index1, wg), enabled);

  @override
  bool shapeAreContactEventsEnabled(int index1, int wg) =>
      b2.b2Shape_AreContactEventsEnabled(_shape(index1, wg));

  @override
  void shapeEnableContactEvents(int index1, int wg, {required bool enabled}) =>
      b2.b2Shape_EnableContactEvents(_shape(index1, wg), enabled);

  @override
  bool shapeAreHitEventsEnabled(int index1, int wg) =>
      b2.b2Shape_AreHitEventsEnabled(_shape(index1, wg));

  @override
  void shapeEnableHitEvents(int index1, int wg, {required bool enabled}) =>
      b2.b2Shape_EnableHitEvents(_shape(index1, wg), enabled);

  @override
  bool shapeArePreSolveEventsEnabled(int index1, int wg) =>
      b2.b2Shape_ArePreSolveEventsEnabled(_shape(index1, wg));

  @override
  void shapeEnablePreSolveEvents(
    int index1,
    int wg, {
    required bool enabled,
  }) => b2.b2Shape_EnablePreSolveEvents(_shape(index1, wg), enabled);

  @override
  bool shapeTestPoint(int index1, int wg, double x, double y) =>
      b2.b2Shape_TestPoint(_shape(index1, wg), _vec2(x, y));

  @override
  (double, double, double, double) shapeGetAabb(int index1, int wg) {
    final aabb = b2.b2Shape_GetAABB(_shape(index1, wg));
    return (
      aabb.lowerBound.x,
      aabb.lowerBound.y,
      aabb.upperBound.x,
      aabb.upperBound.y,
    );
  }

  // Chains.

  @override
  (int, int) createChain(
    int bodyIndex1,
    int bodyWg, {
    required List<double> points,
    required List<double> materials,
    required int categoryBits,
    required int maskBits,
    required int groupIndex,
    required bool isLoop,
    required bool enableSensorEvents,
  }) {
    return using((arena) {
      final pointCount = points.length ~/ 2;
      final vertices = arena<b2.b2Vec2>(pointCount);
      for (var i = 0; i < pointCount; i++) {
        vertices[i]
          ..x = points[2 * i]
          ..y = points[2 * i + 1];
      }

      const materialStride = 6;
      final materialCount = materials.length ~/ materialStride;
      final nativeMaterials = arena<b2.b2SurfaceMaterial>(materialCount);
      for (var i = 0; i < materialCount; i++) {
        final offset = i * materialStride;
        nativeMaterials[i]
          ..friction = materials[offset]
          ..restitution = materials[offset + 1]
          ..rollingResistance = materials[offset + 2]
          ..tangentSpeed = materials[offset + 3]
          ..userMaterialId = materials[offset + 4].toInt()
          ..customColor = materials[offset + 5].toInt();
      }

      final def = arena<b2.b2ChainDef>()..ref = b2.b2DefaultChainDef();
      def.ref
        ..points = vertices
        ..count = pointCount
        ..materials = nativeMaterials
        ..materialCount = materialCount
        ..filter.categoryBits = categoryBits
        ..filter.maskBits = maskBits
        ..filter.groupIndex = groupIndex
        ..isLoop = isLoop
        ..enableSensorEvents = enableSensorEvents;
      return _packChain(b2.b2CreateChain(_body(bodyIndex1, bodyWg), def));
    });
  }

  @override
  void destroyChain(int index1, int wg) =>
      b2.b2DestroyChain(_chain(index1, wg));

  @override
  bool chainIsValid(int index1, int wg) =>
      b2.b2Chain_IsValid(_chain(index1, wg));

  @override
  void chainSetFriction(int index1, int wg, double friction) =>
      b2.b2Chain_SetFriction(_chain(index1, wg), friction);

  @override
  double chainGetFriction(int index1, int wg) =>
      b2.b2Chain_GetFriction(_chain(index1, wg));

  @override
  void chainSetRestitution(int index1, int wg, double restitution) =>
      b2.b2Chain_SetRestitution(_chain(index1, wg), restitution);

  @override
  double chainGetRestitution(int index1, int wg) =>
      b2.b2Chain_GetRestitution(_chain(index1, wg));

  @override
  List<int> chainGetSegments(int index1, int wg) {
    return using((arena) {
      final chain = _chain(index1, wg);
      final count = b2.b2Chain_GetSegmentCount(chain);
      final segments = arena<b2.b2ShapeId>(count);
      final written = b2.b2Chain_GetSegments(chain, segments, count);
      return [
        for (var i = 0; i < written; i++) ...[
          segments[i].index1,
          segments[i].world0 | (segments[i].generation << 16),
        ],
      ];
    });
  }
}
