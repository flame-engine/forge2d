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

  @override
  List<int> bodyGetJoints(int index1, int wg) {
    return using((arena) {
      final body = _body(index1, wg);
      final count = b2.b2Body_GetJointCount(body);
      final joints = arena<b2.b2JointId>(count);
      final written = b2.b2Body_GetJoints(body, joints, count);
      return [
        for (var i = 0; i < written; i++) ...[
          joints[i].index1,
          joints[i].world0 | (joints[i].generation << 16),
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

  // Joints.

  b2.b2JointId _joint(int index1, int wg) => Struct.create<b2.b2JointId>()
    ..index1 = index1
    ..world0 = wg & 0xFFFF
    ..generation = (wg >> 16) & 0xFFFF;

  (int, int) _packJoint(b2.b2JointId id) => (
    id.index1,
    id.world0 | (id.generation << 16),
  );

  @override
  (int, int) createDistanceJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required double length,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double minLength,
    required double maxLength,
    required bool enableMotor,
    required double maxMotorForce,
    required double motorSpeed,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2DistanceJointDef>()
        ..ref = b2.b2DefaultDistanceJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..localAnchorA.x = localAnchorA.$1
        ..localAnchorA.y = localAnchorA.$2
        ..localAnchorB.x = localAnchorB.$1
        ..localAnchorB.y = localAnchorB.$2
        ..length = length
        ..enableSpring = enableSpring
        ..hertz = hertz
        ..dampingRatio = dampingRatio
        ..enableLimit = enableLimit
        ..minLength = minLength
        ..maxLength = maxLength
        ..enableMotor = enableMotor
        ..maxMotorForce = maxMotorForce
        ..motorSpeed = motorSpeed
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateDistanceJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createFilterJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
  }) {
    return using((arena) {
      final def = arena<b2.b2FilterJointDef>()
        ..ref = b2.b2DefaultFilterJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2);
      return _packJoint(b2.b2CreateFilterJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createMotorJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) linearOffset,
    required double angularOffset,
    required double maxForce,
    required double maxTorque,
    required double correctionFactor,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2MotorJointDef>()
        ..ref = b2.b2DefaultMotorJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..linearOffset.x = linearOffset.$1
        ..linearOffset.y = linearOffset.$2
        ..angularOffset = angularOffset
        ..maxForce = maxForce
        ..maxTorque = maxTorque
        ..correctionFactor = correctionFactor
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateMotorJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createMouseJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) target,
    required double hertz,
    required double dampingRatio,
    required double maxForce,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2MouseJointDef>()
        ..ref = b2.b2DefaultMouseJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..target.x = target.$1
        ..target.y = target.$2
        ..hertz = hertz
        ..dampingRatio = dampingRatio
        ..maxForce = maxForce
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateMouseJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createPrismaticJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required (double, double) localAxisA,
    required double referenceAngle,
    required double targetTranslation,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double lowerTranslation,
    required double upperTranslation,
    required bool enableMotor,
    required double maxMotorForce,
    required double motorSpeed,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2PrismaticJointDef>()
        ..ref = b2.b2DefaultPrismaticJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..localAnchorA.x = localAnchorA.$1
        ..localAnchorA.y = localAnchorA.$2
        ..localAnchorB.x = localAnchorB.$1
        ..localAnchorB.y = localAnchorB.$2
        ..localAxisA.x = localAxisA.$1
        ..localAxisA.y = localAxisA.$2
        ..referenceAngle = referenceAngle
        ..targetTranslation = targetTranslation
        ..enableSpring = enableSpring
        ..hertz = hertz
        ..dampingRatio = dampingRatio
        ..enableLimit = enableLimit
        ..lowerTranslation = lowerTranslation
        ..upperTranslation = upperTranslation
        ..enableMotor = enableMotor
        ..maxMotorForce = maxMotorForce
        ..motorSpeed = motorSpeed
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreatePrismaticJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createRevoluteJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required double referenceAngle,
    required double targetAngle,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double lowerAngle,
    required double upperAngle,
    required bool enableMotor,
    required double maxMotorTorque,
    required double motorSpeed,
    required double drawSize,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2RevoluteJointDef>()
        ..ref = b2.b2DefaultRevoluteJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..localAnchorA.x = localAnchorA.$1
        ..localAnchorA.y = localAnchorA.$2
        ..localAnchorB.x = localAnchorB.$1
        ..localAnchorB.y = localAnchorB.$2
        ..referenceAngle = referenceAngle
        ..targetAngle = targetAngle
        ..enableSpring = enableSpring
        ..hertz = hertz
        ..dampingRatio = dampingRatio
        ..enableLimit = enableLimit
        ..lowerAngle = lowerAngle
        ..upperAngle = upperAngle
        ..enableMotor = enableMotor
        ..maxMotorTorque = maxMotorTorque
        ..motorSpeed = motorSpeed
        ..drawSize = drawSize
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateRevoluteJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createWeldJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required double referenceAngle,
    required double linearHertz,
    required double angularHertz,
    required double linearDampingRatio,
    required double angularDampingRatio,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2WeldJointDef>()..ref = b2.b2DefaultWeldJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..localAnchorA.x = localAnchorA.$1
        ..localAnchorA.y = localAnchorA.$2
        ..localAnchorB.x = localAnchorB.$1
        ..localAnchorB.y = localAnchorB.$2
        ..referenceAngle = referenceAngle
        ..linearHertz = linearHertz
        ..angularHertz = angularHertz
        ..linearDampingRatio = linearDampingRatio
        ..angularDampingRatio = angularDampingRatio
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateWeldJoint(_world(worldId), def));
    });
  }

  @override
  (int, int) createWheelJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required (double, double) localAxisA,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double lowerTranslation,
    required double upperTranslation,
    required bool enableMotor,
    required double maxMotorTorque,
    required double motorSpeed,
    required bool collideConnected,
  }) {
    return using((arena) {
      final def = arena<b2.b2WheelJointDef>()
        ..ref = b2.b2DefaultWheelJointDef();
      def.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..localAnchorA.x = localAnchorA.$1
        ..localAnchorA.y = localAnchorA.$2
        ..localAnchorB.x = localAnchorB.$1
        ..localAnchorB.y = localAnchorB.$2
        ..localAxisA.x = localAxisA.$1
        ..localAxisA.y = localAxisA.$2
        ..enableSpring = enableSpring
        ..hertz = hertz
        ..dampingRatio = dampingRatio
        ..enableLimit = enableLimit
        ..lowerTranslation = lowerTranslation
        ..upperTranslation = upperTranslation
        ..enableMotor = enableMotor
        ..maxMotorTorque = maxMotorTorque
        ..motorSpeed = motorSpeed
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateWheelJoint(_world(worldId), def));
    });
  }

  @override
  void destroyJoint(int index1, int wg) =>
      b2.b2DestroyJoint(_joint(index1, wg));

  @override
  bool jointIsValid(int index1, int wg) =>
      b2.b2Joint_IsValid(_joint(index1, wg));

  @override
  int jointGetType(int index1, int wg) =>
      b2.b2Joint_GetType(_joint(index1, wg)).value;

  @override
  (int, int) jointGetBodyA(int index1, int wg) =>
      _packBody(b2.b2Joint_GetBodyA(_joint(index1, wg)));

  @override
  (int, int) jointGetBodyB(int index1, int wg) =>
      _packBody(b2.b2Joint_GetBodyB(_joint(index1, wg)));

  @override
  (double, double) jointGetLocalAnchorA(int index1, int wg) {
    final anchor = b2.b2Joint_GetLocalAnchorA(_joint(index1, wg));
    return (anchor.x, anchor.y);
  }

  @override
  (double, double) jointGetLocalAnchorB(int index1, int wg) {
    final anchor = b2.b2Joint_GetLocalAnchorB(_joint(index1, wg));
    return (anchor.x, anchor.y);
  }

  @override
  bool jointGetCollideConnected(int index1, int wg) =>
      b2.b2Joint_GetCollideConnected(_joint(index1, wg));

  @override
  void jointSetCollideConnected(int index1, int wg, {required bool value}) =>
      b2.b2Joint_SetCollideConnected(_joint(index1, wg), value);

  @override
  void jointWakeBodies(int index1, int wg) =>
      b2.b2Joint_WakeBodies(_joint(index1, wg));

  @override
  (double, double) jointGetConstraintForce(int index1, int wg) {
    final force = b2.b2Joint_GetConstraintForce(_joint(index1, wg));
    return (force.x, force.y);
  }

  @override
  double jointGetConstraintTorque(int index1, int wg) =>
      b2.b2Joint_GetConstraintTorque(_joint(index1, wg));

  // Distance joint.

  @override
  double distanceJointGetLength(int index1, int wg) =>
      b2.b2DistanceJoint_GetLength(_joint(index1, wg));

  @override
  void distanceJointSetLength(int index1, int wg, double length) =>
      b2.b2DistanceJoint_SetLength(_joint(index1, wg), length);

  @override
  double distanceJointGetCurrentLength(int index1, int wg) =>
      b2.b2DistanceJoint_GetCurrentLength(_joint(index1, wg));

  @override
  bool distanceJointIsSpringEnabled(int index1, int wg) =>
      b2.b2DistanceJoint_IsSpringEnabled(_joint(index1, wg));

  @override
  void distanceJointEnableSpring(int index1, int wg, {required bool enabled}) =>
      b2.b2DistanceJoint_EnableSpring(_joint(index1, wg), enabled);

  @override
  double distanceJointGetSpringHertz(int index1, int wg) =>
      b2.b2DistanceJoint_GetSpringHertz(_joint(index1, wg));

  @override
  void distanceJointSetSpringHertz(int index1, int wg, double hertz) =>
      b2.b2DistanceJoint_SetSpringHertz(_joint(index1, wg), hertz);

  @override
  double distanceJointGetSpringDampingRatio(int index1, int wg) =>
      b2.b2DistanceJoint_GetSpringDampingRatio(_joint(index1, wg));

  @override
  void distanceJointSetSpringDampingRatio(int index1, int wg, double ratio) =>
      b2.b2DistanceJoint_SetSpringDampingRatio(_joint(index1, wg), ratio);

  @override
  bool distanceJointIsLimitEnabled(int index1, int wg) =>
      b2.b2DistanceJoint_IsLimitEnabled(_joint(index1, wg));

  @override
  void distanceJointEnableLimit(int index1, int wg, {required bool enabled}) =>
      b2.b2DistanceJoint_EnableLimit(_joint(index1, wg), enabled);

  @override
  double distanceJointGetMinLength(int index1, int wg) =>
      b2.b2DistanceJoint_GetMinLength(_joint(index1, wg));

  @override
  double distanceJointGetMaxLength(int index1, int wg) =>
      b2.b2DistanceJoint_GetMaxLength(_joint(index1, wg));

  @override
  void distanceJointSetLengthRange(
    int index1,
    int wg,
    double minLength,
    double maxLength,
  ) => b2.b2DistanceJoint_SetLengthRange(
    _joint(index1, wg),
    minLength,
    maxLength,
  );

  @override
  bool distanceJointIsMotorEnabled(int index1, int wg) =>
      b2.b2DistanceJoint_IsMotorEnabled(_joint(index1, wg));

  @override
  void distanceJointEnableMotor(int index1, int wg, {required bool enabled}) =>
      b2.b2DistanceJoint_EnableMotor(_joint(index1, wg), enabled);

  @override
  double distanceJointGetMotorSpeed(int index1, int wg) =>
      b2.b2DistanceJoint_GetMotorSpeed(_joint(index1, wg));

  @override
  void distanceJointSetMotorSpeed(int index1, int wg, double speed) =>
      b2.b2DistanceJoint_SetMotorSpeed(_joint(index1, wg), speed);

  @override
  double distanceJointGetMaxMotorForce(int index1, int wg) =>
      b2.b2DistanceJoint_GetMaxMotorForce(_joint(index1, wg));

  @override
  void distanceJointSetMaxMotorForce(int index1, int wg, double force) =>
      b2.b2DistanceJoint_SetMaxMotorForce(_joint(index1, wg), force);

  @override
  double distanceJointGetMotorForce(int index1, int wg) =>
      b2.b2DistanceJoint_GetMotorForce(_joint(index1, wg));

  // Motor joint.

  @override
  (double, double) motorJointGetLinearOffset(int index1, int wg) {
    final offset = b2.b2MotorJoint_GetLinearOffset(_joint(index1, wg));
    return (offset.x, offset.y);
  }

  @override
  void motorJointSetLinearOffset(int index1, int wg, double x, double y) =>
      b2.b2MotorJoint_SetLinearOffset(_joint(index1, wg), _vec2(x, y));

  @override
  double motorJointGetAngularOffset(int index1, int wg) =>
      b2.b2MotorJoint_GetAngularOffset(_joint(index1, wg));

  @override
  void motorJointSetAngularOffset(int index1, int wg, double offset) =>
      b2.b2MotorJoint_SetAngularOffset(_joint(index1, wg), offset);

  @override
  double motorJointGetMaxForce(int index1, int wg) =>
      b2.b2MotorJoint_GetMaxForce(_joint(index1, wg));

  @override
  void motorJointSetMaxForce(int index1, int wg, double force) =>
      b2.b2MotorJoint_SetMaxForce(_joint(index1, wg), force);

  @override
  double motorJointGetMaxTorque(int index1, int wg) =>
      b2.b2MotorJoint_GetMaxTorque(_joint(index1, wg));

  @override
  void motorJointSetMaxTorque(int index1, int wg, double torque) =>
      b2.b2MotorJoint_SetMaxTorque(_joint(index1, wg), torque);

  @override
  double motorJointGetCorrectionFactor(int index1, int wg) =>
      b2.b2MotorJoint_GetCorrectionFactor(_joint(index1, wg));

  @override
  void motorJointSetCorrectionFactor(int index1, int wg, double factor) =>
      b2.b2MotorJoint_SetCorrectionFactor(_joint(index1, wg), factor);

  // Mouse joint.

  @override
  (double, double) mouseJointGetTarget(int index1, int wg) {
    final target = b2.b2MouseJoint_GetTarget(_joint(index1, wg));
    return (target.x, target.y);
  }

  @override
  void mouseJointSetTarget(int index1, int wg, double x, double y) =>
      b2.b2MouseJoint_SetTarget(_joint(index1, wg), _vec2(x, y));

  @override
  double mouseJointGetSpringHertz(int index1, int wg) =>
      b2.b2MouseJoint_GetSpringHertz(_joint(index1, wg));

  @override
  void mouseJointSetSpringHertz(int index1, int wg, double hertz) =>
      b2.b2MouseJoint_SetSpringHertz(_joint(index1, wg), hertz);

  @override
  double mouseJointGetSpringDampingRatio(int index1, int wg) =>
      b2.b2MouseJoint_GetSpringDampingRatio(_joint(index1, wg));

  @override
  void mouseJointSetSpringDampingRatio(int index1, int wg, double ratio) =>
      b2.b2MouseJoint_SetSpringDampingRatio(_joint(index1, wg), ratio);

  @override
  double mouseJointGetMaxForce(int index1, int wg) =>
      b2.b2MouseJoint_GetMaxForce(_joint(index1, wg));

  @override
  void mouseJointSetMaxForce(int index1, int wg, double force) =>
      b2.b2MouseJoint_SetMaxForce(_joint(index1, wg), force);

  // Prismatic joint.

  @override
  bool prismaticJointIsSpringEnabled(int index1, int wg) =>
      b2.b2PrismaticJoint_IsSpringEnabled(_joint(index1, wg));

  @override
  void prismaticJointEnableSpring(
    int index1,
    int wg, {
    required bool enabled,
  }) => b2.b2PrismaticJoint_EnableSpring(_joint(index1, wg), enabled);

  @override
  double prismaticJointGetSpringHertz(int index1, int wg) =>
      b2.b2PrismaticJoint_GetSpringHertz(_joint(index1, wg));

  @override
  void prismaticJointSetSpringHertz(int index1, int wg, double hertz) =>
      b2.b2PrismaticJoint_SetSpringHertz(_joint(index1, wg), hertz);

  @override
  double prismaticJointGetSpringDampingRatio(int index1, int wg) =>
      b2.b2PrismaticJoint_GetSpringDampingRatio(_joint(index1, wg));

  @override
  void prismaticJointSetSpringDampingRatio(int index1, int wg, double ratio) =>
      b2.b2PrismaticJoint_SetSpringDampingRatio(_joint(index1, wg), ratio);

  @override
  double prismaticJointGetTargetTranslation(int index1, int wg) =>
      b2.b2PrismaticJoint_GetTargetTranslation(_joint(index1, wg));

  @override
  void prismaticJointSetTargetTranslation(int index1, int wg, double value) =>
      b2.b2PrismaticJoint_SetTargetTranslation(_joint(index1, wg), value);

  @override
  bool prismaticJointIsLimitEnabled(int index1, int wg) =>
      b2.b2PrismaticJoint_IsLimitEnabled(_joint(index1, wg));

  @override
  void prismaticJointEnableLimit(int index1, int wg, {required bool enabled}) =>
      b2.b2PrismaticJoint_EnableLimit(_joint(index1, wg), enabled);

  @override
  double prismaticJointGetLowerLimit(int index1, int wg) =>
      b2.b2PrismaticJoint_GetLowerLimit(_joint(index1, wg));

  @override
  double prismaticJointGetUpperLimit(int index1, int wg) =>
      b2.b2PrismaticJoint_GetUpperLimit(_joint(index1, wg));

  @override
  void prismaticJointSetLimits(
    int index1,
    int wg,
    double lower,
    double upper,
  ) => b2.b2PrismaticJoint_SetLimits(_joint(index1, wg), lower, upper);

  @override
  bool prismaticJointIsMotorEnabled(int index1, int wg) =>
      b2.b2PrismaticJoint_IsMotorEnabled(_joint(index1, wg));

  @override
  void prismaticJointEnableMotor(int index1, int wg, {required bool enabled}) =>
      b2.b2PrismaticJoint_EnableMotor(_joint(index1, wg), enabled);

  @override
  double prismaticJointGetMotorSpeed(int index1, int wg) =>
      b2.b2PrismaticJoint_GetMotorSpeed(_joint(index1, wg));

  @override
  void prismaticJointSetMotorSpeed(int index1, int wg, double speed) =>
      b2.b2PrismaticJoint_SetMotorSpeed(_joint(index1, wg), speed);

  @override
  double prismaticJointGetMaxMotorForce(int index1, int wg) =>
      b2.b2PrismaticJoint_GetMaxMotorForce(_joint(index1, wg));

  @override
  void prismaticJointSetMaxMotorForce(int index1, int wg, double force) =>
      b2.b2PrismaticJoint_SetMaxMotorForce(_joint(index1, wg), force);

  @override
  double prismaticJointGetMotorForce(int index1, int wg) =>
      b2.b2PrismaticJoint_GetMotorForce(_joint(index1, wg));

  @override
  double prismaticJointGetTranslation(int index1, int wg) =>
      b2.b2PrismaticJoint_GetTranslation(_joint(index1, wg));

  @override
  double prismaticJointGetSpeed(int index1, int wg) =>
      b2.b2PrismaticJoint_GetSpeed(_joint(index1, wg));

  // Revolute joint.

  @override
  bool revoluteJointIsSpringEnabled(int index1, int wg) =>
      b2.b2RevoluteJoint_IsSpringEnabled(_joint(index1, wg));

  @override
  void revoluteJointEnableSpring(int index1, int wg, {required bool enabled}) =>
      b2.b2RevoluteJoint_EnableSpring(_joint(index1, wg), enabled);

  @override
  double revoluteJointGetSpringHertz(int index1, int wg) =>
      b2.b2RevoluteJoint_GetSpringHertz(_joint(index1, wg));

  @override
  void revoluteJointSetSpringHertz(int index1, int wg, double hertz) =>
      b2.b2RevoluteJoint_SetSpringHertz(_joint(index1, wg), hertz);

  @override
  double revoluteJointGetSpringDampingRatio(int index1, int wg) =>
      b2.b2RevoluteJoint_GetSpringDampingRatio(_joint(index1, wg));

  @override
  void revoluteJointSetSpringDampingRatio(int index1, int wg, double ratio) =>
      b2.b2RevoluteJoint_SetSpringDampingRatio(_joint(index1, wg), ratio);

  @override
  double revoluteJointGetTargetAngle(int index1, int wg) =>
      b2.b2RevoluteJoint_GetTargetAngle(_joint(index1, wg));

  @override
  void revoluteJointSetTargetAngle(int index1, int wg, double angle) =>
      b2.b2RevoluteJoint_SetTargetAngle(_joint(index1, wg), angle);

  @override
  double revoluteJointGetAngle(int index1, int wg) =>
      b2.b2RevoluteJoint_GetAngle(_joint(index1, wg));

  @override
  bool revoluteJointIsLimitEnabled(int index1, int wg) =>
      b2.b2RevoluteJoint_IsLimitEnabled(_joint(index1, wg));

  @override
  void revoluteJointEnableLimit(int index1, int wg, {required bool enabled}) =>
      b2.b2RevoluteJoint_EnableLimit(_joint(index1, wg), enabled);

  @override
  double revoluteJointGetLowerLimit(int index1, int wg) =>
      b2.b2RevoluteJoint_GetLowerLimit(_joint(index1, wg));

  @override
  double revoluteJointGetUpperLimit(int index1, int wg) =>
      b2.b2RevoluteJoint_GetUpperLimit(_joint(index1, wg));

  @override
  void revoluteJointSetLimits(int index1, int wg, double lower, double upper) =>
      b2.b2RevoluteJoint_SetLimits(_joint(index1, wg), lower, upper);

  @override
  bool revoluteJointIsMotorEnabled(int index1, int wg) =>
      b2.b2RevoluteJoint_IsMotorEnabled(_joint(index1, wg));

  @override
  void revoluteJointEnableMotor(int index1, int wg, {required bool enabled}) =>
      b2.b2RevoluteJoint_EnableMotor(_joint(index1, wg), enabled);

  @override
  double revoluteJointGetMotorSpeed(int index1, int wg) =>
      b2.b2RevoluteJoint_GetMotorSpeed(_joint(index1, wg));

  @override
  void revoluteJointSetMotorSpeed(int index1, int wg, double speed) =>
      b2.b2RevoluteJoint_SetMotorSpeed(_joint(index1, wg), speed);

  @override
  double revoluteJointGetMaxMotorTorque(int index1, int wg) =>
      b2.b2RevoluteJoint_GetMaxMotorTorque(_joint(index1, wg));

  @override
  void revoluteJointSetMaxMotorTorque(int index1, int wg, double torque) =>
      b2.b2RevoluteJoint_SetMaxMotorTorque(_joint(index1, wg), torque);

  @override
  double revoluteJointGetMotorTorque(int index1, int wg) =>
      b2.b2RevoluteJoint_GetMotorTorque(_joint(index1, wg));

  // Weld joint.

  @override
  double weldJointGetLinearHertz(int index1, int wg) =>
      b2.b2WeldJoint_GetLinearHertz(_joint(index1, wg));

  @override
  void weldJointSetLinearHertz(int index1, int wg, double hertz) =>
      b2.b2WeldJoint_SetLinearHertz(_joint(index1, wg), hertz);

  @override
  double weldJointGetAngularHertz(int index1, int wg) =>
      b2.b2WeldJoint_GetAngularHertz(_joint(index1, wg));

  @override
  void weldJointSetAngularHertz(int index1, int wg, double hertz) =>
      b2.b2WeldJoint_SetAngularHertz(_joint(index1, wg), hertz);

  @override
  double weldJointGetLinearDampingRatio(int index1, int wg) =>
      b2.b2WeldJoint_GetLinearDampingRatio(_joint(index1, wg));

  @override
  void weldJointSetLinearDampingRatio(int index1, int wg, double ratio) =>
      b2.b2WeldJoint_SetLinearDampingRatio(_joint(index1, wg), ratio);

  @override
  double weldJointGetAngularDampingRatio(int index1, int wg) =>
      b2.b2WeldJoint_GetAngularDampingRatio(_joint(index1, wg));

  @override
  void weldJointSetAngularDampingRatio(int index1, int wg, double ratio) =>
      b2.b2WeldJoint_SetAngularDampingRatio(_joint(index1, wg), ratio);

  // Wheel joint.

  @override
  bool wheelJointIsSpringEnabled(int index1, int wg) =>
      b2.b2WheelJoint_IsSpringEnabled(_joint(index1, wg));

  @override
  void wheelJointEnableSpring(int index1, int wg, {required bool enabled}) =>
      b2.b2WheelJoint_EnableSpring(_joint(index1, wg), enabled);

  @override
  double wheelJointGetSpringHertz(int index1, int wg) =>
      b2.b2WheelJoint_GetSpringHertz(_joint(index1, wg));

  @override
  void wheelJointSetSpringHertz(int index1, int wg, double hertz) =>
      b2.b2WheelJoint_SetSpringHertz(_joint(index1, wg), hertz);

  @override
  double wheelJointGetSpringDampingRatio(int index1, int wg) =>
      b2.b2WheelJoint_GetSpringDampingRatio(_joint(index1, wg));

  @override
  void wheelJointSetSpringDampingRatio(int index1, int wg, double ratio) =>
      b2.b2WheelJoint_SetSpringDampingRatio(_joint(index1, wg), ratio);

  @override
  bool wheelJointIsLimitEnabled(int index1, int wg) =>
      b2.b2WheelJoint_IsLimitEnabled(_joint(index1, wg));

  @override
  void wheelJointEnableLimit(int index1, int wg, {required bool enabled}) =>
      b2.b2WheelJoint_EnableLimit(_joint(index1, wg), enabled);

  @override
  double wheelJointGetLowerLimit(int index1, int wg) =>
      b2.b2WheelJoint_GetLowerLimit(_joint(index1, wg));

  @override
  double wheelJointGetUpperLimit(int index1, int wg) =>
      b2.b2WheelJoint_GetUpperLimit(_joint(index1, wg));

  @override
  void wheelJointSetLimits(int index1, int wg, double lower, double upper) =>
      b2.b2WheelJoint_SetLimits(_joint(index1, wg), lower, upper);

  @override
  bool wheelJointIsMotorEnabled(int index1, int wg) =>
      b2.b2WheelJoint_IsMotorEnabled(_joint(index1, wg));

  @override
  void wheelJointEnableMotor(int index1, int wg, {required bool enabled}) =>
      b2.b2WheelJoint_EnableMotor(_joint(index1, wg), enabled);

  @override
  double wheelJointGetMotorSpeed(int index1, int wg) =>
      b2.b2WheelJoint_GetMotorSpeed(_joint(index1, wg));

  @override
  void wheelJointSetMotorSpeed(int index1, int wg, double speed) =>
      b2.b2WheelJoint_SetMotorSpeed(_joint(index1, wg), speed);

  @override
  double wheelJointGetMaxMotorTorque(int index1, int wg) =>
      b2.b2WheelJoint_GetMaxMotorTorque(_joint(index1, wg));

  @override
  void wheelJointSetMaxMotorTorque(int index1, int wg, double torque) =>
      b2.b2WheelJoint_SetMaxMotorTorque(_joint(index1, wg), torque);

  @override
  double wheelJointGetMotorTorque(int index1, int wg) =>
      b2.b2WheelJoint_GetMotorTorque(_joint(index1, wg));
}
