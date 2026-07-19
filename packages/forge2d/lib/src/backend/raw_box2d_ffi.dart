import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:forge2d/src/backend/raw_box2d.dart';
import 'package:forge2d/src/ffi/box2d.g.dart' as b2;

/// Prepares the FFI backend. Nothing to do: the native library is bundled by
/// the build hook and loaded lazily. [wasmUri] only applies on the web.
Future<void> initializeBackend({Uri? wasmUri}) async {}

/// Creates the FFI backend.
RawBox2D createRawBox2D() => RawBox2DFfi();

/// The dart:ffi implementation of [RawBox2D], calling the generated bindings
/// directly.
final class RawBox2DFfi implements RawBox2D {
  b2.b2WorldId _world(int id) => Struct.create<b2.b2WorldId>()
    ..index1 = id & 0xFFFF
    ..generation = (id >> 16) & 0xFFFF;

  int _packWorld(b2.b2WorldId id) => id.index1 | (id.generation << 16);

  b2.b2BodyId _body(int index1, int worldAndGeneration) =>
      Struct.create<b2.b2BodyId>()
        ..index1 = index1
        ..world0 = worldAndGeneration & 0xFFFF
        ..generation = (worldAndGeneration >> 16) & 0xFFFF;

  (int, int) _packBody(b2.b2BodyId id) => (
    id.index1,
    id.world0 | (id.generation << 16),
  );

  b2.b2ShapeId _shape(int index1, int worldAndGeneration) =>
      Struct.create<b2.b2ShapeId>()
        ..index1 = index1
        ..world0 = worldAndGeneration & 0xFFFF
        ..generation = (worldAndGeneration >> 16) & 0xFFFF;

  (int, int) _packShape(b2.b2ShapeId id) => (
    id.index1,
    id.world0 | (id.generation << 16),
  );

  b2.b2ChainId _chain(int index1, int worldAndGeneration) =>
      Struct.create<b2.b2ChainId>()
        ..index1 = index1
        ..world0 = worldAndGeneration & 0xFFFF
        ..generation = (worldAndGeneration >> 16) & 0xFFFF;

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
      final definition = arena<b2.b2WorldDef>()..ref = b2.b2DefaultWorldDef();
      definition.ref
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
      return _packWorld(b2.b2CreateWorld(definition));
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
      final definition = arena<b2.b2BodyDef>()..ref = b2.b2DefaultBodyDef();
      definition.ref
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
        definition.ref.name = name.toNativeUtf8(allocator: arena).cast();
      }
      return _packBody(b2.b2CreateBody(_world(worldId), definition));
    });
  }

  @override
  void destroyBody(int index1, int worldAndGeneration) =>
      b2.b2DestroyBody(_body(index1, worldAndGeneration));

  @override
  bool bodyIsValid(int index1, int worldAndGeneration) =>
      b2.b2Body_IsValid(_body(index1, worldAndGeneration));

  @override
  (double, double) bodyGetPosition(int index1, int worldAndGeneration) {
    final position = b2.b2Body_GetPosition(_body(index1, worldAndGeneration));
    return (position.x, position.y);
  }

  @override
  (double, double) bodyGetRotation(int index1, int worldAndGeneration) {
    final rotation = b2.b2Body_GetRotation(_body(index1, worldAndGeneration));
    return (rotation.c, rotation.s);
  }

  @override
  void bodySetTransform(
    int index1,
    int worldAndGeneration,
    double positionX,
    double positionY,
    double rotationCos,
    double rotationSin,
  ) => b2.b2Body_SetTransform(
    _body(index1, worldAndGeneration),
    _vec2(positionX, positionY),
    _rot(rotationCos, rotationSin),
  );

  @override
  (double, double) bodyGetLinearVelocity(int index1, int worldAndGeneration) {
    final velocity = b2.b2Body_GetLinearVelocity(
      _body(index1, worldAndGeneration),
    );
    return (velocity.x, velocity.y);
  }

  @override
  void bodySetLinearVelocity(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) => b2.b2Body_SetLinearVelocity(
    _body(index1, worldAndGeneration),
    _vec2(x, y),
  );

  @override
  double bodyGetAngularVelocity(int index1, int worldAndGeneration) =>
      b2.b2Body_GetAngularVelocity(_body(index1, worldAndGeneration));

  @override
  void bodySetAngularVelocity(
    int index1,
    int worldAndGeneration,
    double value,
  ) => b2.b2Body_SetAngularVelocity(_body(index1, worldAndGeneration), value);

  @override
  void bodyApplyForce(
    int index1,
    int worldAndGeneration,
    double forceX,
    double forceY,
    double pointX,
    double pointY, {
    required bool wake,
  }) => b2.b2Body_ApplyForce(
    _body(index1, worldAndGeneration),
    _vec2(forceX, forceY),
    _vec2(pointX, pointY),
    wake,
  );

  @override
  void bodyApplyForceToCenter(
    int index1,
    int worldAndGeneration,
    double forceX,
    double forceY, {
    required bool wake,
  }) => b2.b2Body_ApplyForceToCenter(
    _body(index1, worldAndGeneration),
    _vec2(forceX, forceY),
    wake,
  );

  @override
  void bodyApplyTorque(
    int index1,
    int worldAndGeneration,
    double torque, {
    required bool wake,
  }) => b2.b2Body_ApplyTorque(_body(index1, worldAndGeneration), torque, wake);

  @override
  void bodyApplyLinearImpulse(
    int index1,
    int worldAndGeneration,
    double impulseX,
    double impulseY,
    double pointX,
    double pointY, {
    required bool wake,
  }) => b2.b2Body_ApplyLinearImpulse(
    _body(index1, worldAndGeneration),
    _vec2(impulseX, impulseY),
    _vec2(pointX, pointY),
    wake,
  );

  @override
  void bodyApplyLinearImpulseToCenter(
    int index1,
    int worldAndGeneration,
    double impulseX,
    double impulseY, {
    required bool wake,
  }) => b2.b2Body_ApplyLinearImpulseToCenter(
    _body(index1, worldAndGeneration),
    _vec2(impulseX, impulseY),
    wake,
  );

  @override
  void bodyApplyAngularImpulse(
    int index1,
    int worldAndGeneration,
    double impulse, {
    required bool wake,
  }) => b2.b2Body_ApplyAngularImpulse(
    _body(index1, worldAndGeneration),
    impulse,
    wake,
  );

  @override
  double bodyGetMass(int index1, int worldAndGeneration) =>
      b2.b2Body_GetMass(_body(index1, worldAndGeneration));

  @override
  double bodyGetRotationalInertia(int index1, int worldAndGeneration) =>
      b2.b2Body_GetRotationalInertia(_body(index1, worldAndGeneration));

  @override
  (double, double) bodyGetLocalCenterOfMass(
    int index1,
    int worldAndGeneration,
  ) {
    final center = b2.b2Body_GetLocalCenterOfMass(
      _body(index1, worldAndGeneration),
    );
    return (center.x, center.y);
  }

  @override
  (double, double) bodyGetWorldCenterOfMass(
    int index1,
    int worldAndGeneration,
  ) {
    final center = b2.b2Body_GetWorldCenterOfMass(
      _body(index1, worldAndGeneration),
    );
    return (center.x, center.y);
  }

  @override
  void bodySetMassData(
    int index1,
    int worldAndGeneration,
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
    b2.b2Body_SetMassData(_body(index1, worldAndGeneration), massData);
  }

  @override
  void bodyApplyMassFromShapes(int index1, int worldAndGeneration) =>
      b2.b2Body_ApplyMassFromShapes(_body(index1, worldAndGeneration));

  @override
  int bodyGetType(int index1, int worldAndGeneration) =>
      b2.b2Body_GetType(_body(index1, worldAndGeneration)).value;

  @override
  void bodySetType(int index1, int worldAndGeneration, int type) =>
      b2.b2Body_SetType(
        _body(index1, worldAndGeneration),
        b2.b2BodyType.fromValue(type),
      );

  @override
  String bodyGetName(int index1, int worldAndGeneration) {
    final name = b2.b2Body_GetName(_body(index1, worldAndGeneration));
    return name == nullptr ? '' : name.cast<Utf8>().toDartString();
  }

  @override
  void bodySetName(int index1, int worldAndGeneration, String? name) {
    using((arena) {
      b2.b2Body_SetName(
        _body(index1, worldAndGeneration),
        name == null ? nullptr : name.toNativeUtf8(allocator: arena).cast(),
      );
    });
  }

  @override
  bool bodyIsAwake(int index1, int worldAndGeneration) =>
      b2.b2Body_IsAwake(_body(index1, worldAndGeneration));

  @override
  void bodySetAwake(
    int index1,
    int worldAndGeneration, {
    required bool awake,
  }) => b2.b2Body_SetAwake(_body(index1, worldAndGeneration), awake);

  @override
  bool bodyIsSleepEnabled(int index1, int worldAndGeneration) =>
      b2.b2Body_IsSleepEnabled(_body(index1, worldAndGeneration));

  @override
  void bodyEnableSleep(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2Body_EnableSleep(_body(index1, worldAndGeneration), enabled);

  @override
  double bodyGetSleepThreshold(int index1, int worldAndGeneration) =>
      b2.b2Body_GetSleepThreshold(_body(index1, worldAndGeneration));

  @override
  void bodySetSleepThreshold(
    int index1,
    int worldAndGeneration,
    double value,
  ) => b2.b2Body_SetSleepThreshold(_body(index1, worldAndGeneration), value);

  @override
  bool bodyIsEnabled(int index1, int worldAndGeneration) =>
      b2.b2Body_IsEnabled(_body(index1, worldAndGeneration));

  @override
  void bodyDisable(int index1, int worldAndGeneration) =>
      b2.b2Body_Disable(_body(index1, worldAndGeneration));

  @override
  void bodyEnable(int index1, int worldAndGeneration) =>
      b2.b2Body_Enable(_body(index1, worldAndGeneration));

  @override
  bool bodyIsFixedRotation(int index1, int worldAndGeneration) =>
      b2.b2Body_IsFixedRotation(_body(index1, worldAndGeneration));

  @override
  void bodySetFixedRotation(
    int index1,
    int worldAndGeneration, {
    required bool flag,
  }) => b2.b2Body_SetFixedRotation(_body(index1, worldAndGeneration), flag);

  @override
  bool bodyIsBullet(int index1, int worldAndGeneration) =>
      b2.b2Body_IsBullet(_body(index1, worldAndGeneration));

  @override
  void bodySetBullet(
    int index1,
    int worldAndGeneration, {
    required bool flag,
  }) => b2.b2Body_SetBullet(_body(index1, worldAndGeneration), flag);

  @override
  double bodyGetGravityScale(int index1, int worldAndGeneration) =>
      b2.b2Body_GetGravityScale(_body(index1, worldAndGeneration));

  @override
  void bodySetGravityScale(int index1, int worldAndGeneration, double scale) =>
      b2.b2Body_SetGravityScale(_body(index1, worldAndGeneration), scale);

  @override
  double bodyGetLinearDamping(int index1, int worldAndGeneration) =>
      b2.b2Body_GetLinearDamping(_body(index1, worldAndGeneration));

  @override
  void bodySetLinearDamping(
    int index1,
    int worldAndGeneration,
    double damping,
  ) => b2.b2Body_SetLinearDamping(_body(index1, worldAndGeneration), damping);

  @override
  double bodyGetAngularDamping(int index1, int worldAndGeneration) =>
      b2.b2Body_GetAngularDamping(_body(index1, worldAndGeneration));

  @override
  void bodySetAngularDamping(
    int index1,
    int worldAndGeneration,
    double damping,
  ) => b2.b2Body_SetAngularDamping(_body(index1, worldAndGeneration), damping);

  @override
  (double, double) bodyGetWorldPoint(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) {
    final point = b2.b2Body_GetWorldPoint(
      _body(index1, worldAndGeneration),
      _vec2(x, y),
    );
    return (point.x, point.y);
  }

  @override
  (double, double) bodyGetLocalPoint(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) {
    final point = b2.b2Body_GetLocalPoint(
      _body(index1, worldAndGeneration),
      _vec2(x, y),
    );
    return (point.x, point.y);
  }

  @override
  List<int> bodyGetShapes(int index1, int worldAndGeneration) {
    return using((arena) {
      final body = _body(index1, worldAndGeneration);
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
  List<int> bodyGetJoints(int index1, int worldAndGeneration) {
    return using((arena) {
      final body = _body(index1, worldAndGeneration);
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

  Pointer<b2.b2ShapeDef> _shapeDef(Arena arena, RawShapeDef definition) {
    final nativeDef = arena<b2.b2ShapeDef>()..ref = b2.b2DefaultShapeDef();
    nativeDef.ref
      ..material.friction = definition.friction
      ..material.restitution = definition.restitution
      ..material.rollingResistance = definition.rollingResistance
      ..material.tangentSpeed = definition.tangentSpeed
      ..material.userMaterialId = definition.userMaterialId
      ..material.customColor = definition.customColor
      ..density = definition.density
      ..filter.categoryBits = definition.categoryBits
      ..filter.maskBits = definition.maskBits
      ..filter.groupIndex = definition.groupIndex
      ..isSensor = definition.isSensor
      ..enableSensorEvents = definition.enableSensorEvents
      ..enableContactEvents = definition.enableContactEvents
      ..enableHitEvents = definition.enableHitEvents
      ..enablePreSolveEvents = definition.enablePreSolveEvents
      ..invokeContactCreation = definition.invokeContactCreation
      ..updateBodyMass = definition.updateBodyMass;
    return nativeDef;
  }

  @override
  (int, int) createCircleShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double centerX,
    required double centerY,
    required double radius,
    required RawShapeDef definition,
  }) {
    return using((arena) {
      final circle = arena<b2.b2Circle>();
      circle.ref
        ..center.x = centerX
        ..center.y = centerY
        ..radius = radius;
      return _packShape(
        b2.b2CreateCircleShape(
          _body(bodyIndex1, bodyWorldAndGeneration),
          _shapeDef(arena, definition),
          circle,
        ),
      );
    });
  }

  @override
  (int, int) createCapsuleShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double center1X,
    required double center1Y,
    required double center2X,
    required double center2Y,
    required double radius,
    required RawShapeDef definition,
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
          _body(bodyIndex1, bodyWorldAndGeneration),
          _shapeDef(arena, definition),
          capsule,
        ),
      );
    });
  }

  @override
  (int, int) createSegmentShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double point1X,
    required double point1Y,
    required double point2X,
    required double point2Y,
    required RawShapeDef definition,
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
          _body(bodyIndex1, bodyWorldAndGeneration),
          _shapeDef(arena, definition),
          segment,
        ),
      );
    });
  }

  @override
  (int, int) createBoxShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double halfWidth,
    required double halfHeight,
    required double centerX,
    required double centerY,
    required double rotationCos,
    required double rotationSin,
    required double radius,
    required RawShapeDef definition,
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
          _body(bodyIndex1, bodyWorldAndGeneration),
          _shapeDef(arena, definition),
          polygon,
        ),
      );
    });
  }

  @override
  (int, int) createPolygonShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required List<double> points,
    required double radius,
    required RawShapeDef definition,
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
          _body(bodyIndex1, bodyWorldAndGeneration),
          _shapeDef(arena, definition),
          polygon,
        ),
      );
    });
  }

  @override
  void destroyShape(
    int index1,
    int worldAndGeneration, {
    required bool updateBodyMass,
  }) => b2.b2DestroyShape(_shape(index1, worldAndGeneration), updateBodyMass);

  @override
  bool shapeIsValid(int index1, int worldAndGeneration) =>
      b2.b2Shape_IsValid(_shape(index1, worldAndGeneration));

  @override
  int shapeGetType(int index1, int worldAndGeneration) =>
      b2.b2Shape_GetType(_shape(index1, worldAndGeneration)).value;

  @override
  (int, int) shapeGetBody(int index1, int worldAndGeneration) =>
      _packBody(b2.b2Shape_GetBody(_shape(index1, worldAndGeneration)));

  @override
  bool shapeIsSensor(int index1, int worldAndGeneration) =>
      b2.b2Shape_IsSensor(_shape(index1, worldAndGeneration));

  @override
  double shapeGetDensity(int index1, int worldAndGeneration) =>
      b2.b2Shape_GetDensity(_shape(index1, worldAndGeneration));

  @override
  void shapeSetDensity(
    int index1,
    int worldAndGeneration,
    double density, {
    required bool updateBodyMass,
  }) => b2.b2Shape_SetDensity(
    _shape(index1, worldAndGeneration),
    density,
    updateBodyMass,
  );

  @override
  double shapeGetFriction(int index1, int worldAndGeneration) =>
      b2.b2Shape_GetFriction(_shape(index1, worldAndGeneration));

  @override
  void shapeSetFriction(int index1, int worldAndGeneration, double friction) =>
      b2.b2Shape_SetFriction(_shape(index1, worldAndGeneration), friction);

  @override
  double shapeGetRestitution(int index1, int worldAndGeneration) =>
      b2.b2Shape_GetRestitution(_shape(index1, worldAndGeneration));

  @override
  void shapeSetRestitution(
    int index1,
    int worldAndGeneration,
    double restitution,
  ) => b2.b2Shape_SetRestitution(
    _shape(index1, worldAndGeneration),
    restitution,
  );

  @override
  (int, int, int) shapeGetFilter(int index1, int worldAndGeneration) {
    final filter = b2.b2Shape_GetFilter(_shape(index1, worldAndGeneration));
    return (filter.categoryBits, filter.maskBits, filter.groupIndex);
  }

  @override
  void shapeSetFilter(
    int index1,
    int worldAndGeneration,
    int categoryBits,
    int maskBits,
    int groupIndex,
  ) {
    final filter = Struct.create<b2.b2Filter>()
      ..categoryBits = categoryBits
      ..maskBits = maskBits
      ..groupIndex = groupIndex;
    b2.b2Shape_SetFilter(_shape(index1, worldAndGeneration), filter);
  }

  @override
  bool shapeAreSensorEventsEnabled(int index1, int worldAndGeneration) =>
      b2.b2Shape_AreSensorEventsEnabled(_shape(index1, worldAndGeneration));

  @override
  void shapeEnableSensorEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2Shape_EnableSensorEvents(
    _shape(index1, worldAndGeneration),
    enabled,
  );

  @override
  bool shapeAreContactEventsEnabled(int index1, int worldAndGeneration) =>
      b2.b2Shape_AreContactEventsEnabled(_shape(index1, worldAndGeneration));

  @override
  void shapeEnableContactEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2Shape_EnableContactEvents(
    _shape(index1, worldAndGeneration),
    enabled,
  );

  @override
  bool shapeAreHitEventsEnabled(int index1, int worldAndGeneration) =>
      b2.b2Shape_AreHitEventsEnabled(_shape(index1, worldAndGeneration));

  @override
  void shapeEnableHitEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2Shape_EnableHitEvents(_shape(index1, worldAndGeneration), enabled);

  @override
  bool shapeArePreSolveEventsEnabled(int index1, int worldAndGeneration) =>
      b2.b2Shape_ArePreSolveEventsEnabled(_shape(index1, worldAndGeneration));

  @override
  void shapeEnablePreSolveEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2Shape_EnablePreSolveEvents(
    _shape(index1, worldAndGeneration),
    enabled,
  );

  @override
  bool shapeTestPoint(int index1, int worldAndGeneration, double x, double y) =>
      b2.b2Shape_TestPoint(_shape(index1, worldAndGeneration), _vec2(x, y));

  @override
  (double, double, double, double) shapeGetAabb(
    int index1,
    int worldAndGeneration,
  ) {
    final aabb = b2.b2Shape_GetAABB(_shape(index1, worldAndGeneration));
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
    int bodyWorldAndGeneration, {
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

      final definition = arena<b2.b2ChainDef>()..ref = b2.b2DefaultChainDef();
      definition.ref
        ..points = vertices
        ..count = pointCount
        ..materials = nativeMaterials
        ..materialCount = materialCount
        ..filter.categoryBits = categoryBits
        ..filter.maskBits = maskBits
        ..filter.groupIndex = groupIndex
        ..isLoop = isLoop
        ..enableSensorEvents = enableSensorEvents;
      return _packChain(
        b2.b2CreateChain(_body(bodyIndex1, bodyWorldAndGeneration), definition),
      );
    });
  }

  @override
  void destroyChain(int index1, int worldAndGeneration) =>
      b2.b2DestroyChain(_chain(index1, worldAndGeneration));

  @override
  bool chainIsValid(int index1, int worldAndGeneration) =>
      b2.b2Chain_IsValid(_chain(index1, worldAndGeneration));

  @override
  void chainSetFriction(int index1, int worldAndGeneration, double friction) =>
      b2.b2Chain_SetFriction(_chain(index1, worldAndGeneration), friction);

  @override
  double chainGetFriction(int index1, int worldAndGeneration) =>
      b2.b2Chain_GetFriction(_chain(index1, worldAndGeneration));

  @override
  void chainSetRestitution(
    int index1,
    int worldAndGeneration,
    double restitution,
  ) => b2.b2Chain_SetRestitution(
    _chain(index1, worldAndGeneration),
    restitution,
  );

  @override
  double chainGetRestitution(int index1, int worldAndGeneration) =>
      b2.b2Chain_GetRestitution(_chain(index1, worldAndGeneration));

  @override
  List<int> chainGetSegments(int index1, int worldAndGeneration) {
    return using((arena) {
      final chain = _chain(index1, worldAndGeneration);
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

  b2.b2JointId _joint(int index1, int worldAndGeneration) =>
      Struct.create<b2.b2JointId>()
        ..index1 = index1
        ..world0 = worldAndGeneration & 0xFFFF
        ..generation = (worldAndGeneration >> 16) & 0xFFFF;

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
      final definition = arena<b2.b2DistanceJointDef>()
        ..ref = b2.b2DefaultDistanceJointDef();
      definition.ref
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
      return _packJoint(b2.b2CreateDistanceJoint(_world(worldId), definition));
    });
  }

  @override
  (int, int) createFilterJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
  }) {
    return using((arena) {
      final definition = arena<b2.b2FilterJointDef>()
        ..ref = b2.b2DefaultFilterJointDef();
      definition.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2);
      return _packJoint(b2.b2CreateFilterJoint(_world(worldId), definition));
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
      final definition = arena<b2.b2MotorJointDef>()
        ..ref = b2.b2DefaultMotorJointDef();
      definition.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..linearOffset.x = linearOffset.$1
        ..linearOffset.y = linearOffset.$2
        ..angularOffset = angularOffset
        ..maxForce = maxForce
        ..maxTorque = maxTorque
        ..correctionFactor = correctionFactor
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateMotorJoint(_world(worldId), definition));
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
      final definition = arena<b2.b2MouseJointDef>()
        ..ref = b2.b2DefaultMouseJointDef();
      definition.ref
        ..bodyIdA = _body(bodyA.$1, bodyA.$2)
        ..bodyIdB = _body(bodyB.$1, bodyB.$2)
        ..target.x = target.$1
        ..target.y = target.$2
        ..hertz = hertz
        ..dampingRatio = dampingRatio
        ..maxForce = maxForce
        ..collideConnected = collideConnected;
      return _packJoint(b2.b2CreateMouseJoint(_world(worldId), definition));
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
      final definition = arena<b2.b2PrismaticJointDef>()
        ..ref = b2.b2DefaultPrismaticJointDef();
      definition.ref
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
      return _packJoint(b2.b2CreatePrismaticJoint(_world(worldId), definition));
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
      final definition = arena<b2.b2RevoluteJointDef>()
        ..ref = b2.b2DefaultRevoluteJointDef();
      definition.ref
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
      return _packJoint(b2.b2CreateRevoluteJoint(_world(worldId), definition));
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
      final definition = arena<b2.b2WeldJointDef>()
        ..ref = b2.b2DefaultWeldJointDef();
      definition.ref
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
      return _packJoint(b2.b2CreateWeldJoint(_world(worldId), definition));
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
      final definition = arena<b2.b2WheelJointDef>()
        ..ref = b2.b2DefaultWheelJointDef();
      definition.ref
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
      return _packJoint(b2.b2CreateWheelJoint(_world(worldId), definition));
    });
  }

  @override
  void destroyJoint(int index1, int worldAndGeneration) =>
      b2.b2DestroyJoint(_joint(index1, worldAndGeneration));

  @override
  bool jointIsValid(int index1, int worldAndGeneration) =>
      b2.b2Joint_IsValid(_joint(index1, worldAndGeneration));

  @override
  int jointGetType(int index1, int worldAndGeneration) =>
      b2.b2Joint_GetType(_joint(index1, worldAndGeneration)).value;

  @override
  (int, int) jointGetBodyA(int index1, int worldAndGeneration) =>
      _packBody(b2.b2Joint_GetBodyA(_joint(index1, worldAndGeneration)));

  @override
  (int, int) jointGetBodyB(int index1, int worldAndGeneration) =>
      _packBody(b2.b2Joint_GetBodyB(_joint(index1, worldAndGeneration)));

  @override
  (double, double) jointGetLocalAnchorA(int index1, int worldAndGeneration) {
    final anchor = b2.b2Joint_GetLocalAnchorA(
      _joint(index1, worldAndGeneration),
    );
    return (anchor.x, anchor.y);
  }

  @override
  (double, double) jointGetLocalAnchorB(int index1, int worldAndGeneration) {
    final anchor = b2.b2Joint_GetLocalAnchorB(
      _joint(index1, worldAndGeneration),
    );
    return (anchor.x, anchor.y);
  }

  @override
  bool jointGetCollideConnected(int index1, int worldAndGeneration) =>
      b2.b2Joint_GetCollideConnected(_joint(index1, worldAndGeneration));

  @override
  void jointSetCollideConnected(
    int index1,
    int worldAndGeneration, {
    required bool value,
  }) =>
      b2.b2Joint_SetCollideConnected(_joint(index1, worldAndGeneration), value);

  @override
  void jointWakeBodies(int index1, int worldAndGeneration) =>
      b2.b2Joint_WakeBodies(_joint(index1, worldAndGeneration));

  @override
  (double, double) jointGetConstraintForce(int index1, int worldAndGeneration) {
    final force = b2.b2Joint_GetConstraintForce(
      _joint(index1, worldAndGeneration),
    );
    return (force.x, force.y);
  }

  @override
  double jointGetConstraintTorque(int index1, int worldAndGeneration) =>
      b2.b2Joint_GetConstraintTorque(_joint(index1, worldAndGeneration));

  // Distance joint.

  @override
  double distanceJointGetLength(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetLength(_joint(index1, worldAndGeneration));

  @override
  void distanceJointSetLength(
    int index1,
    int worldAndGeneration,
    double length,
  ) => b2.b2DistanceJoint_SetLength(_joint(index1, worldAndGeneration), length);

  @override
  double distanceJointGetCurrentLength(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetCurrentLength(_joint(index1, worldAndGeneration));

  @override
  bool distanceJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_IsSpringEnabled(_joint(index1, worldAndGeneration));

  @override
  void distanceJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2DistanceJoint_EnableSpring(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double distanceJointGetSpringHertz(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetSpringHertz(_joint(index1, worldAndGeneration));

  @override
  void distanceJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => b2.b2DistanceJoint_SetSpringHertz(
    _joint(index1, worldAndGeneration),
    hertz,
  );

  @override
  double distanceJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  ) => b2.b2DistanceJoint_GetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
  );

  @override
  void distanceJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2DistanceJoint_SetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  @override
  bool distanceJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_IsLimitEnabled(_joint(index1, worldAndGeneration));

  @override
  void distanceJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2DistanceJoint_EnableLimit(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double distanceJointGetMinLength(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetMinLength(_joint(index1, worldAndGeneration));

  @override
  double distanceJointGetMaxLength(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetMaxLength(_joint(index1, worldAndGeneration));

  @override
  void distanceJointSetLengthRange(
    int index1,
    int worldAndGeneration,
    double minLength,
    double maxLength,
  ) => b2.b2DistanceJoint_SetLengthRange(
    _joint(index1, worldAndGeneration),
    minLength,
    maxLength,
  );

  @override
  bool distanceJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_IsMotorEnabled(_joint(index1, worldAndGeneration));

  @override
  void distanceJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2DistanceJoint_EnableMotor(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double distanceJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetMotorSpeed(_joint(index1, worldAndGeneration));

  @override
  void distanceJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => b2.b2DistanceJoint_SetMotorSpeed(
    _joint(index1, worldAndGeneration),
    speed,
  );

  @override
  double distanceJointGetMaxMotorForce(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetMaxMotorForce(_joint(index1, worldAndGeneration));

  @override
  void distanceJointSetMaxMotorForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => b2.b2DistanceJoint_SetMaxMotorForce(
    _joint(index1, worldAndGeneration),
    force,
  );

  @override
  double distanceJointGetMotorForce(int index1, int worldAndGeneration) =>
      b2.b2DistanceJoint_GetMotorForce(_joint(index1, worldAndGeneration));

  // Motor joint.

  @override
  (double, double) motorJointGetLinearOffset(
    int index1,
    int worldAndGeneration,
  ) {
    final offset = b2.b2MotorJoint_GetLinearOffset(
      _joint(index1, worldAndGeneration),
    );
    return (offset.x, offset.y);
  }

  @override
  void motorJointSetLinearOffset(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) => b2.b2MotorJoint_SetLinearOffset(
    _joint(index1, worldAndGeneration),
    _vec2(x, y),
  );

  @override
  double motorJointGetAngularOffset(int index1, int worldAndGeneration) =>
      b2.b2MotorJoint_GetAngularOffset(_joint(index1, worldAndGeneration));

  @override
  void motorJointSetAngularOffset(
    int index1,
    int worldAndGeneration,
    double offset,
  ) => b2.b2MotorJoint_SetAngularOffset(
    _joint(index1, worldAndGeneration),
    offset,
  );

  @override
  double motorJointGetMaxForce(int index1, int worldAndGeneration) =>
      b2.b2MotorJoint_GetMaxForce(_joint(index1, worldAndGeneration));

  @override
  void motorJointSetMaxForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => b2.b2MotorJoint_SetMaxForce(_joint(index1, worldAndGeneration), force);

  @override
  double motorJointGetMaxTorque(int index1, int worldAndGeneration) =>
      b2.b2MotorJoint_GetMaxTorque(_joint(index1, worldAndGeneration));

  @override
  void motorJointSetMaxTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  ) => b2.b2MotorJoint_SetMaxTorque(_joint(index1, worldAndGeneration), torque);

  @override
  double motorJointGetCorrectionFactor(int index1, int worldAndGeneration) =>
      b2.b2MotorJoint_GetCorrectionFactor(_joint(index1, worldAndGeneration));

  @override
  void motorJointSetCorrectionFactor(
    int index1,
    int worldAndGeneration,
    double factor,
  ) => b2.b2MotorJoint_SetCorrectionFactor(
    _joint(index1, worldAndGeneration),
    factor,
  );

  // Mouse joint.

  @override
  (double, double) mouseJointGetTarget(int index1, int worldAndGeneration) {
    final target = b2.b2MouseJoint_GetTarget(
      _joint(index1, worldAndGeneration),
    );
    return (target.x, target.y);
  }

  @override
  void mouseJointSetTarget(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) => b2.b2MouseJoint_SetTarget(
    _joint(index1, worldAndGeneration),
    _vec2(x, y),
  );

  @override
  double mouseJointGetSpringHertz(int index1, int worldAndGeneration) =>
      b2.b2MouseJoint_GetSpringHertz(_joint(index1, worldAndGeneration));

  @override
  void mouseJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) =>
      b2.b2MouseJoint_SetSpringHertz(_joint(index1, worldAndGeneration), hertz);

  @override
  double mouseJointGetSpringDampingRatio(int index1, int worldAndGeneration) =>
      b2.b2MouseJoint_GetSpringDampingRatio(_joint(index1, worldAndGeneration));

  @override
  void mouseJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2MouseJoint_SetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  @override
  double mouseJointGetMaxForce(int index1, int worldAndGeneration) =>
      b2.b2MouseJoint_GetMaxForce(_joint(index1, worldAndGeneration));

  @override
  void mouseJointSetMaxForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => b2.b2MouseJoint_SetMaxForce(_joint(index1, worldAndGeneration), force);

  // Prismatic joint.

  @override
  bool prismaticJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_IsSpringEnabled(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2PrismaticJoint_EnableSpring(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double prismaticJointGetSpringHertz(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetSpringHertz(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => b2.b2PrismaticJoint_SetSpringHertz(
    _joint(index1, worldAndGeneration),
    hertz,
  );

  @override
  double prismaticJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  ) => b2.b2PrismaticJoint_GetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
  );

  @override
  void prismaticJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2PrismaticJoint_SetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  @override
  double prismaticJointGetTargetTranslation(
    int index1,
    int worldAndGeneration,
  ) => b2.b2PrismaticJoint_GetTargetTranslation(
    _joint(index1, worldAndGeneration),
  );

  @override
  void prismaticJointSetTargetTranslation(
    int index1,
    int worldAndGeneration,
    double value,
  ) => b2.b2PrismaticJoint_SetTargetTranslation(
    _joint(index1, worldAndGeneration),
    value,
  );

  @override
  bool prismaticJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_IsLimitEnabled(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2PrismaticJoint_EnableLimit(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double prismaticJointGetLowerLimit(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetLowerLimit(_joint(index1, worldAndGeneration));

  @override
  double prismaticJointGetUpperLimit(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetUpperLimit(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  ) => b2.b2PrismaticJoint_SetLimits(
    _joint(index1, worldAndGeneration),
    lower,
    upper,
  );

  @override
  bool prismaticJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_IsMotorEnabled(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2PrismaticJoint_EnableMotor(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double prismaticJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetMotorSpeed(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => b2.b2PrismaticJoint_SetMotorSpeed(
    _joint(index1, worldAndGeneration),
    speed,
  );

  @override
  double prismaticJointGetMaxMotorForce(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetMaxMotorForce(_joint(index1, worldAndGeneration));

  @override
  void prismaticJointSetMaxMotorForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => b2.b2PrismaticJoint_SetMaxMotorForce(
    _joint(index1, worldAndGeneration),
    force,
  );

  @override
  double prismaticJointGetMotorForce(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetMotorForce(_joint(index1, worldAndGeneration));

  @override
  double prismaticJointGetTranslation(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetTranslation(_joint(index1, worldAndGeneration));

  @override
  double prismaticJointGetSpeed(int index1, int worldAndGeneration) =>
      b2.b2PrismaticJoint_GetSpeed(_joint(index1, worldAndGeneration));

  // Revolute joint.

  @override
  bool revoluteJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_IsSpringEnabled(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2RevoluteJoint_EnableSpring(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double revoluteJointGetSpringHertz(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetSpringHertz(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => b2.b2RevoluteJoint_SetSpringHertz(
    _joint(index1, worldAndGeneration),
    hertz,
  );

  @override
  double revoluteJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  ) => b2.b2RevoluteJoint_GetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
  );

  @override
  void revoluteJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2RevoluteJoint_SetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  @override
  double revoluteJointGetTargetAngle(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetTargetAngle(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointSetTargetAngle(
    int index1,
    int worldAndGeneration,
    double angle,
  ) => b2.b2RevoluteJoint_SetTargetAngle(
    _joint(index1, worldAndGeneration),
    angle,
  );

  @override
  double revoluteJointGetAngle(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetAngle(_joint(index1, worldAndGeneration));

  @override
  bool revoluteJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_IsLimitEnabled(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2RevoluteJoint_EnableLimit(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double revoluteJointGetLowerLimit(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetLowerLimit(_joint(index1, worldAndGeneration));

  @override
  double revoluteJointGetUpperLimit(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetUpperLimit(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  ) => b2.b2RevoluteJoint_SetLimits(
    _joint(index1, worldAndGeneration),
    lower,
    upper,
  );

  @override
  bool revoluteJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_IsMotorEnabled(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => b2.b2RevoluteJoint_EnableMotor(
    _joint(index1, worldAndGeneration),
    enabled,
  );

  @override
  double revoluteJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetMotorSpeed(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => b2.b2RevoluteJoint_SetMotorSpeed(
    _joint(index1, worldAndGeneration),
    speed,
  );

  @override
  double revoluteJointGetMaxMotorTorque(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetMaxMotorTorque(_joint(index1, worldAndGeneration));

  @override
  void revoluteJointSetMaxMotorTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  ) => b2.b2RevoluteJoint_SetMaxMotorTorque(
    _joint(index1, worldAndGeneration),
    torque,
  );

  @override
  double revoluteJointGetMotorTorque(int index1, int worldAndGeneration) =>
      b2.b2RevoluteJoint_GetMotorTorque(_joint(index1, worldAndGeneration));

  // Weld joint.

  @override
  double weldJointGetLinearHertz(int index1, int worldAndGeneration) =>
      b2.b2WeldJoint_GetLinearHertz(_joint(index1, worldAndGeneration));

  @override
  void weldJointSetLinearHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => b2.b2WeldJoint_SetLinearHertz(_joint(index1, worldAndGeneration), hertz);

  @override
  double weldJointGetAngularHertz(int index1, int worldAndGeneration) =>
      b2.b2WeldJoint_GetAngularHertz(_joint(index1, worldAndGeneration));

  @override
  void weldJointSetAngularHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) =>
      b2.b2WeldJoint_SetAngularHertz(_joint(index1, worldAndGeneration), hertz);

  @override
  double weldJointGetLinearDampingRatio(int index1, int worldAndGeneration) =>
      b2.b2WeldJoint_GetLinearDampingRatio(_joint(index1, worldAndGeneration));

  @override
  void weldJointSetLinearDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2WeldJoint_SetLinearDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  @override
  double weldJointGetAngularDampingRatio(int index1, int worldAndGeneration) =>
      b2.b2WeldJoint_GetAngularDampingRatio(_joint(index1, worldAndGeneration));

  @override
  void weldJointSetAngularDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2WeldJoint_SetAngularDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  // Wheel joint.

  @override
  bool wheelJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_IsSpringEnabled(_joint(index1, worldAndGeneration));

  @override
  void wheelJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) =>
      b2.b2WheelJoint_EnableSpring(_joint(index1, worldAndGeneration), enabled);

  @override
  double wheelJointGetSpringHertz(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetSpringHertz(_joint(index1, worldAndGeneration));

  @override
  void wheelJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) =>
      b2.b2WheelJoint_SetSpringHertz(_joint(index1, worldAndGeneration), hertz);

  @override
  double wheelJointGetSpringDampingRatio(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetSpringDampingRatio(_joint(index1, worldAndGeneration));

  @override
  void wheelJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => b2.b2WheelJoint_SetSpringDampingRatio(
    _joint(index1, worldAndGeneration),
    ratio,
  );

  @override
  bool wheelJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_IsLimitEnabled(_joint(index1, worldAndGeneration));

  @override
  void wheelJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) =>
      b2.b2WheelJoint_EnableLimit(_joint(index1, worldAndGeneration), enabled);

  @override
  double wheelJointGetLowerLimit(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetLowerLimit(_joint(index1, worldAndGeneration));

  @override
  double wheelJointGetUpperLimit(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetUpperLimit(_joint(index1, worldAndGeneration));

  @override
  void wheelJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  ) => b2.b2WheelJoint_SetLimits(
    _joint(index1, worldAndGeneration),
    lower,
    upper,
  );

  @override
  bool wheelJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_IsMotorEnabled(_joint(index1, worldAndGeneration));

  @override
  void wheelJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) =>
      b2.b2WheelJoint_EnableMotor(_joint(index1, worldAndGeneration), enabled);

  @override
  double wheelJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetMotorSpeed(_joint(index1, worldAndGeneration));

  @override
  void wheelJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => b2.b2WheelJoint_SetMotorSpeed(_joint(index1, worldAndGeneration), speed);

  @override
  double wheelJointGetMaxMotorTorque(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetMaxMotorTorque(_joint(index1, worldAndGeneration));

  @override
  void wheelJointSetMaxMotorTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  ) => b2.b2WheelJoint_SetMaxMotorTorque(
    _joint(index1, worldAndGeneration),
    torque,
  );

  @override
  double wheelJointGetMotorTorque(int index1, int worldAndGeneration) =>
      b2.b2WheelJoint_GetMotorTorque(_joint(index1, worldAndGeneration));

  // Events.

  @override
  RawContactEvents worldGetContactEvents(int worldId) {
    final events = b2.b2World_GetContactEvents(_world(worldId));
    return (
      begin: [
        for (var i = 0; i < events.beginCount; i++)
          _contactBeginEvent(events.beginEvents[i]),
      ],
      end: [
        for (var i = 0; i < events.endCount; i++)
          (
            shapeAIndex1: events.endEvents[i].shapeIdA.index1,
            shapeAWorldAndGeneration: _wgOf(events.endEvents[i].shapeIdA),
            shapeBIndex1: events.endEvents[i].shapeIdB.index1,
            shapeBWorldAndGeneration: _wgOf(events.endEvents[i].shapeIdB),
          ),
      ],
      hit: [
        for (var i = 0; i < events.hitCount; i++)
          (
            shapeAIndex1: events.hitEvents[i].shapeIdA.index1,
            shapeAWorldAndGeneration: _wgOf(events.hitEvents[i].shapeIdA),
            shapeBIndex1: events.hitEvents[i].shapeIdB.index1,
            shapeBWorldAndGeneration: _wgOf(events.hitEvents[i].shapeIdB),
            pointX: events.hitEvents[i].point.x,
            pointY: events.hitEvents[i].point.y,
            normalX: events.hitEvents[i].normal.x,
            normalY: events.hitEvents[i].normal.y,
            approachSpeed: events.hitEvents[i].approachSpeed,
          ),
      ],
    );
  }

  int _wgOf(b2.b2ShapeId id) => id.world0 | (id.generation << 16);

  RawContactBeginEvent _contactBeginEvent(b2.b2ContactBeginTouchEvent event) {
    final manifold = event.manifold;
    return (
      shapeAIndex1: event.shapeIdA.index1,
      shapeAWorldAndGeneration: _wgOf(event.shapeIdA),
      shapeBIndex1: event.shapeIdB.index1,
      shapeBWorldAndGeneration: _wgOf(event.shapeIdB),
      normalX: manifold.normal.x,
      normalY: manifold.normal.y,
      points: [
        for (var i = 0; i < manifold.pointCount; i++)
          (
            x: manifold.points[i].point.x,
            y: manifold.points[i].point.y,
            separation: manifold.points[i].separation,
          ),
      ],
    );
  }

  @override
  RawSensorEvents worldGetSensorEvents(int worldId) {
    final events = b2.b2World_GetSensorEvents(_world(worldId));
    return (
      begin: [
        for (var i = 0; i < events.beginCount; i++)
          (
            sensorIndex1: events.beginEvents[i].sensorShapeId.index1,
            sensorWorldAndGeneration: _wgOf(
              events.beginEvents[i].sensorShapeId,
            ),
            visitorIndex1: events.beginEvents[i].visitorShapeId.index1,
            visitorWorldAndGeneration: _wgOf(
              events.beginEvents[i].visitorShapeId,
            ),
          ),
      ],
      end: [
        for (var i = 0; i < events.endCount; i++)
          (
            sensorIndex1: events.endEvents[i].sensorShapeId.index1,
            sensorWorldAndGeneration: _wgOf(events.endEvents[i].sensorShapeId),
            visitorIndex1: events.endEvents[i].visitorShapeId.index1,
            visitorWorldAndGeneration: _wgOf(
              events.endEvents[i].visitorShapeId,
            ),
          ),
      ],
    );
  }

  @override
  List<RawBodyMoveEvent> worldGetBodyEvents(int worldId) {
    final events = b2.b2World_GetBodyEvents(_world(worldId));
    return [
      for (var i = 0; i < events.moveCount; i++)
        (
          bodyIndex1: events.moveEvents[i].bodyId.index1,
          bodyWorldAndGeneration:
              events.moveEvents[i].bodyId.world0 |
              (events.moveEvents[i].bodyId.generation << 16),
          x: events.moveEvents[i].transform.p.x,
          y: events.moveEvents[i].transform.p.y,
          rotationCos: events.moveEvents[i].transform.q.c,
          rotationSin: events.moveEvents[i].transform.q.s,
          fellAsleep: events.moveEvents[i].fellAsleep,
        ),
    ];
  }

  // Queries.

  b2.b2QueryFilter _queryFilter(int categoryBits, int maskBits) =>
      Struct.create<b2.b2QueryFilter>()
        ..categoryBits = categoryBits
        ..maskBits = maskBits;

  @override
  RawRayHit? worldCastRayClosest(
    int worldId,
    double originX,
    double originY,
    double translationX,
    double translationY,
    int categoryBits,
    int maskBits,
  ) {
    final result = b2.b2World_CastRayClosest(
      _world(worldId),
      _vec2(originX, originY),
      _vec2(translationX, translationY),
      _queryFilter(categoryBits, maskBits),
    );
    if (!result.hit) {
      return null;
    }
    return (
      shapeIndex1: result.shapeId.index1,
      shapeWorldAndGeneration: _wgOf(result.shapeId),
      pointX: result.point.x,
      pointY: result.point.y,
      normalX: result.normal.x,
      normalY: result.normal.y,
      fraction: result.fraction,
    );
  }

  /// Queries run synchronously on the calling thread, so a stack of active
  /// callbacks supports re-entrant queries from within a callback.
  static final List<double Function(RawRayHit)> _castRayCallbacks = [];

  static double _onCastRay(
    b2.b2ShapeId shapeId,
    b2.b2Vec2 point,
    b2.b2Vec2 normal,
    double fraction,
    Pointer<Void> context,
  ) => _castRayCallbacks.last((
    shapeIndex1: shapeId.index1,
    shapeWorldAndGeneration: shapeId.world0 | (shapeId.generation << 16),
    pointX: point.x,
    pointY: point.y,
    normalX: normal.x,
    normalY: normal.y,
    fraction: fraction,
  ));

  static final _castRayNative =
      NativeCallable<
          Float Function(
            b2.b2ShapeId,
            b2.b2Vec2,
            b2.b2Vec2,
            Float,
            Pointer<Void>,
          )
        >.isolateLocal(_onCastRay, exceptionalReturn: 0.0)
        ..keepIsolateAlive = false;

  @override
  void worldCastRay(
    int worldId,
    double originX,
    double originY,
    double translationX,
    double translationY,
    int categoryBits,
    int maskBits,
    double Function(RawRayHit hit) callback,
  ) {
    _castRayCallbacks.add(callback);
    try {
      b2.b2World_CastRay(
        _world(worldId),
        _vec2(originX, originY),
        _vec2(translationX, translationY),
        _queryFilter(categoryBits, maskBits),
        _castRayNative.nativeFunction,
        nullptr,
      );
    } finally {
      _castRayCallbacks.removeLast();
    }
  }

  static final List<List<int>> _overlapResults = [];

  static bool _onOverlap(b2.b2ShapeId shapeId, Pointer<Void> context) {
    _overlapResults.last
      ..add(shapeId.index1)
      ..add(shapeId.world0 | (shapeId.generation << 16));
    return true;
  }

  static final _overlapNative =
      NativeCallable<Bool Function(b2.b2ShapeId, Pointer<Void>)>.isolateLocal(
        _onOverlap,
        exceptionalReturn: false,
      )..keepIsolateAlive = false;

  @override
  List<int> worldOverlapAabb(
    int worldId,
    double lowerX,
    double lowerY,
    double upperX,
    double upperY,
    int categoryBits,
    int maskBits,
  ) {
    final aabb = Struct.create<b2.b2AABB>();
    aabb.lowerBound
      ..x = lowerX
      ..y = lowerY;
    aabb.upperBound
      ..x = upperX
      ..y = upperY;
    final results = <int>[];
    _overlapResults.add(results);
    try {
      b2.b2World_OverlapAABB(
        _world(worldId),
        aabb,
        _queryFilter(categoryBits, maskBits),
        _overlapNative.nativeFunction,
        nullptr,
      );
    } finally {
      _overlapResults.removeLast();
    }
    return results;
  }

  @override
  void worldExplode(
    int worldId, {
    required int maskBits,
    required double positionX,
    required double positionY,
    required double radius,
    required double falloff,
    required double impulsePerLength,
  }) {
    using((arena) {
      final definition = arena<b2.b2ExplosionDef>()
        ..ref = b2.b2DefaultExplosionDef();
      definition.ref
        ..maskBits = maskBits
        ..position.x = positionX
        ..position.y = positionY
        ..radius = radius
        ..falloff = falloff
        ..impulsePerLength = impulsePerLength;
      b2.b2World_Explode(_world(worldId), definition);
    });
  }

  // Simulation callbacks, keyed by world id through the context pointer.

  static final Map<int, bool Function(int, int, int, int)> _filterCallbacks =
      {};

  static bool _onCustomFilter(
    b2.b2ShapeId shapeIdA,
    b2.b2ShapeId shapeIdB,
    Pointer<Void> context,
  ) {
    final callback = _filterCallbacks[context.address];
    if (callback == null) {
      return true;
    }
    return callback(
      shapeIdA.index1,
      shapeIdA.world0 | (shapeIdA.generation << 16),
      shapeIdB.index1,
      shapeIdB.world0 | (shapeIdB.generation << 16),
    );
  }

  static final _customFilterNative =
      NativeCallable<
          Bool Function(b2.b2ShapeId, b2.b2ShapeId, Pointer<Void>)
        >.isolateLocal(_onCustomFilter, exceptionalReturn: true)
        ..keepIsolateAlive = false;

  @override
  void worldSetCustomFilterCallback(
    int worldId,
    bool Function(
      int shapeAIndex1,
      int shapeAWorldAndGeneration,
      int shapeBIndex1,
      int shapeBWorldAndGeneration,
    )?
    callback,
  ) {
    if (callback == null) {
      _filterCallbacks.remove(worldId);
      b2.b2World_SetCustomFilterCallback(_world(worldId), nullptr, nullptr);
    } else {
      _filterCallbacks[worldId] = callback;
      b2.b2World_SetCustomFilterCallback(
        _world(worldId),
        _customFilterNative.nativeFunction,
        Pointer<Void>.fromAddress(worldId),
      );
    }
  }

  static final Map<int, bool Function(int, int, int, int, double, double)>
  _preSolveCallbacks = {};

  static bool _onPreSolve(
    b2.b2ShapeId shapeIdA,
    b2.b2ShapeId shapeIdB,
    Pointer<b2.b2Manifold> manifold,
    Pointer<Void> context,
  ) {
    final callback = _preSolveCallbacks[context.address];
    if (callback == null) {
      return true;
    }
    return callback(
      shapeIdA.index1,
      shapeIdA.world0 | (shapeIdA.generation << 16),
      shapeIdB.index1,
      shapeIdB.world0 | (shapeIdB.generation << 16),
      manifold.ref.normal.x,
      manifold.ref.normal.y,
    );
  }

  static final _preSolveNative =
      NativeCallable<
          Bool Function(
            b2.b2ShapeId,
            b2.b2ShapeId,
            Pointer<b2.b2Manifold>,
            Pointer<Void>,
          )
        >.isolateLocal(_onPreSolve, exceptionalReturn: true)
        ..keepIsolateAlive = false;

  @override
  void worldSetPreSolveCallback(
    int worldId,
    bool Function(
      int shapeAIndex1,
      int shapeAWorldAndGeneration,
      int shapeBIndex1,
      int shapeBWorldAndGeneration,
      double normalX,
      double normalY,
    )?
    callback,
  ) {
    if (callback == null) {
      _preSolveCallbacks.remove(worldId);
      b2.b2World_SetPreSolveCallback(_world(worldId), nullptr, nullptr);
    } else {
      _preSolveCallbacks[worldId] = callback;
      b2.b2World_SetPreSolveCallback(
        _world(worldId),
        _preSolveNative.nativeFunction,
        Pointer<Void>.fromAddress(worldId),
      );
    }
  }

  // Debug drawing. World drawing is synchronous, so a stack of active draw
  // targets supports nested worlds.

  static final List<RawDebugDraw> _debugDraws = [];

  static List<double> _flattenVertices(Pointer<b2.b2Vec2> vertices, int count) {
    return [
      for (var i = 0; i < count; i++) ...[vertices[i].x, vertices[i].y],
    ];
  }

  static final _drawPolygonNative =
      NativeCallable<
          Void Function(
            Pointer<b2.b2Vec2>,
            Int,
            UnsignedInt,
            Pointer<Void>,
          )
        >.isolateLocal((
          Pointer<b2.b2Vec2> vertices,
          int vertexCount,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawPolygon(
            _flattenVertices(vertices, vertexCount),
            color,
          );
        })
        ..keepIsolateAlive = false;

  static final _drawSolidPolygonNative =
      NativeCallable<
          Void Function(
            b2.b2Transform,
            Pointer<b2.b2Vec2>,
            Int,
            Float,
            UnsignedInt,
            Pointer<Void>,
          )
        >.isolateLocal((
          b2.b2Transform transform,
          Pointer<b2.b2Vec2> vertices,
          int vertexCount,
          double radius,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawSolidPolygon(
            transform.p.x,
            transform.p.y,
            transform.q.c,
            transform.q.s,
            _flattenVertices(vertices, vertexCount),
            radius,
            color,
          );
        })
        ..keepIsolateAlive = false;

  static final _drawCircleNative =
      NativeCallable<
          Void Function(b2.b2Vec2, Float, UnsignedInt, Pointer<Void>)
        >.isolateLocal((
          b2.b2Vec2 center,
          double radius,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawCircle(center.x, center.y, radius, color);
        })
        ..keepIsolateAlive = false;

  static final _drawSolidCircleNative =
      NativeCallable<
          Void Function(b2.b2Transform, Float, UnsignedInt, Pointer<Void>)
        >.isolateLocal((
          b2.b2Transform transform,
          double radius,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawSolidCircle(
            transform.p.x,
            transform.p.y,
            transform.q.c,
            transform.q.s,
            radius,
            color,
          );
        })
        ..keepIsolateAlive = false;

  static final _drawSolidCapsuleNative =
      NativeCallable<
          Void Function(
            b2.b2Vec2,
            b2.b2Vec2,
            Float,
            UnsignedInt,
            Pointer<Void>,
          )
        >.isolateLocal((
          b2.b2Vec2 point1,
          b2.b2Vec2 point2,
          double radius,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawSolidCapsule(
            point1.x,
            point1.y,
            point2.x,
            point2.y,
            radius,
            color,
          );
        })
        ..keepIsolateAlive = false;

  static final _drawSegmentNative =
      NativeCallable<
          Void Function(b2.b2Vec2, b2.b2Vec2, UnsignedInt, Pointer<Void>)
        >.isolateLocal((
          b2.b2Vec2 point1,
          b2.b2Vec2 point2,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawSegment(
            point1.x,
            point1.y,
            point2.x,
            point2.y,
            color,
          );
        })
        ..keepIsolateAlive = false;

  static final _drawTransformNative =
      NativeCallable<Void Function(b2.b2Transform, Pointer<Void>)>.isolateLocal(
        (b2.b2Transform transform, Pointer<Void> context) {
          _debugDraws.last.drawTransform(
            transform.p.x,
            transform.p.y,
            transform.q.c,
            transform.q.s,
          );
        },
      )..keepIsolateAlive = false;

  static final _drawPointNative =
      NativeCallable<
          Void Function(b2.b2Vec2, Float, UnsignedInt, Pointer<Void>)
        >.isolateLocal((
          b2.b2Vec2 point,
          double size,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawPoint(point.x, point.y, size, color);
        })
        ..keepIsolateAlive = false;

  static final _drawStringNative =
      NativeCallable<
          Void Function(
            b2.b2Vec2,
            Pointer<Char>,
            UnsignedInt,
            Pointer<Void>,
          )
        >.isolateLocal((
          b2.b2Vec2 point,
          Pointer<Char> text,
          int color,
          Pointer<Void> context,
        ) {
          _debugDraws.last.drawString(
            point.x,
            point.y,
            text == nullptr ? '' : text.cast<Utf8>().toDartString(),
            color,
          );
        })
        ..keepIsolateAlive = false;

  @override
  void worldDraw(int worldId, RawDebugDraw draw) {
    _debugDraws.add(draw);
    try {
      using((arena) {
        final nativeDraw = arena<b2.b2DebugDraw>()
          ..ref = b2.b2DefaultDebugDraw();
        nativeDraw.ref
          ..DrawPolygonFcn = _drawPolygonNative.nativeFunction
          ..DrawSolidPolygonFcn = _drawSolidPolygonNative.nativeFunction
          ..DrawCircleFcn = _drawCircleNative.nativeFunction
          ..DrawSolidCircleFcn = _drawSolidCircleNative.nativeFunction
          ..DrawSolidCapsuleFcn = _drawSolidCapsuleNative.nativeFunction
          ..DrawSegmentFcn = _drawSegmentNative.nativeFunction
          ..DrawTransformFcn = _drawTransformNative.nativeFunction
          ..DrawPointFcn = _drawPointNative.nativeFunction
          ..DrawStringFcn = _drawStringNative.nativeFunction
          ..useDrawingBounds = draw.drawingBounds != null
          ..drawShapes = draw.drawShapes
          ..drawJoints = draw.drawJoints
          ..drawJointExtras = draw.drawJointExtras
          ..drawBounds = draw.drawBounds
          ..drawMass = draw.drawMass
          ..drawBodyNames = draw.drawBodyNames
          ..drawContacts = draw.drawContacts
          ..drawGraphColors = draw.drawGraphColors
          ..drawContactNormals = draw.drawContactNormals
          ..drawContactImpulses = draw.drawContactImpulses
          ..drawContactFeatures = draw.drawContactFeatures
          ..drawFrictionImpulses = draw.drawFrictionImpulses
          ..drawIslands = draw.drawIslands;
        if (draw.drawingBounds case (
          final lowerX,
          final lowerY,
          final upperX,
          final upperY,
        )) {
          nativeDraw.ref.drawingBounds.lowerBound
            ..x = lowerX
            ..y = lowerY;
          nativeDraw.ref.drawingBounds.upperBound
            ..x = upperX
            ..y = upperY;
        }
        b2.b2World_Draw(_world(worldId), nativeDraw);
      });
    } finally {
      _debugDraws.removeLast();
    }
  }
}
