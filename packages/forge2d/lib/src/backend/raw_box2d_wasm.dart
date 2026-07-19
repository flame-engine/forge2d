import 'package:forge2d/src/backend/raw_box2d.dart';
import 'package:forge2d/src/backend/wasm/wasm_runtime.dart';

WasmRuntime? _runtime;

/// Loads and instantiates the Box2D WebAssembly module.
///
/// Called by `initializeForge2D`. [wasmUri] overrides the default lookup
/// locations: the package asset path used by Dart web tooling, then a
/// `box2d.wasm` next to the page for Flutter web apps.
Future<void> initializeBackend({Uri? wasmUri}) async {
  if (_runtime != null) {
    return;
  }
  _runtime = await WasmRuntime.load([
    if (wasmUri != null)
      wasmUri
    else ...[
      // Dart web tooling serves package files directly.
      Uri.parse('packages/forge2d/src/backend/wasm/box2d.wasm'),
      Uri.parse('/packages/forge2d/src/backend/wasm/box2d.wasm'),
      // Flutter web bundles the module as a package asset automatically.
      Uri.parse('assets/packages/forge2d/lib/src/backend/wasm/box2d.wasm'),
      // Last resort: a manually hosted copy next to the page.
      Uri.parse('box2d.wasm'),
    ],
  ]);
}

/// Creates the WebAssembly backend.
RawBox2D createRawBox2D() {
  final runtime = _runtime;
  if (runtime == null) {
    throw StateError(
      'forge2d is not initialized. On the web you must call and await '
      'initializeForge2D() before creating a World.',
    );
  }
  return RawBox2DWasm(runtime);
}

/// The dart:js_interop implementation of [RawBox2D], calling the `f2d_`
/// shim exports of box2d.wasm. See native/wasm/f2d_shim.c for the ABI.
final class RawBox2DWasm implements RawBox2D {
  /// Creates the backend over a loaded [WasmRuntime].
  RawBox2DWasm(this._runtime);

  final WasmRuntime _runtime;

  // Scratch arena layout (see WasmRuntime.scratch, 4096 bytes):
  // - byte 0: small out-parameters (up to 128 bytes)
  // - byte 128: the f2dShapeDef struct (19 * 4 bytes)
  // Strings go through the runtime's growable string buffer instead.
  static const _outOffset = 0;
  static const _shapeDefOffset = 128;

  int get _out => _runtime.scratch + _outOffset;
  int get _shapeDefPointer => _runtime.scratch + _shapeDefOffset;

  num _call(String name, List<num> arguments) => _runtime.call(name, arguments);

  double _callF(String name, List<num> arguments) =>
      _call(name, arguments).toDouble();

  int _callI(String name, List<num> arguments) =>
      _call(name, arguments).toInt();

  bool _callB(String name, List<num> arguments) => _call(name, arguments) != 0;

  static int _b(bool value) => value ? 1 : 0;

  /// Splits a Dart int holding a 64-bit bit field into unsigned 32-bit
  /// halves. -1 is the seam's "all bits set" sentinel; other negative
  /// values are 64-bit patterns with the top bit set, which only exist
  /// where ints are 64-bit (the VM and dart2wasm) and split exactly there.
  static (int, int) _splitBits(int value) {
    if (value == -1) {
      return (0xFFFFFFFF, 0xFFFFFFFF);
    }
    if (value < 0) {
      return (value & 0xFFFFFFFF, (value >> 32) & 0xFFFFFFFF);
    }
    return (value % 0x100000000, value ~/ 0x100000000);
  }

  static int _joinBits(int low, int high) {
    final unsignedLow = low < 0 ? low + 0x100000000 : low;
    final unsignedHigh = high < 0 ? high + 0x100000000 : high;
    if (unsignedLow == 0xFFFFFFFF && unsignedHigh == 0xFFFFFFFF) {
      return -1;
    }
    return unsignedHigh * 0x100000000 + unsignedLow;
  }

  (double, double) _outVec2() =>
      (_runtime.readF32(_out), _runtime.readF32(_out + 4));

  /// Reinterprets a value read through a signed 32-bit view as unsigned,
  /// keeping ids consistent with the unsigned values in event records even
  /// once a slot's generation count sets the top bit.
  static int _unsigned32(int value) => value < 0 ? value + 0x100000000 : value;

  /// Normalizes the worldAndGeneration halves of a flat id pair list.
  static List<int> _idPairs(List<int> raw) => [
    for (var i = 0; i < raw.length; i++) i.isOdd ? _unsigned32(raw[i]) : raw[i],
  ];

  (int, int) _outIdPair() => (
    _runtime.readI32(_out),
    _unsigned32(_runtime.readI32(_out + 4)),
  );

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
  }) => _callI('f2d_create_world', [
    gravityX,
    gravityY,
    restitutionThreshold,
    hitEventThreshold,
    contactHertz,
    contactDampingRatio,
    maxContactPushSpeed,
    maximumLinearSpeed,
    _b(enableSleep),
    _b(enableContinuous),
  ]);

  @override
  void destroyWorld(int worldId) => _call('f2d_destroy_world', [worldId]);

  @override
  bool worldIsValid(int worldId) => _callB('f2d_world_is_valid', [worldId]);

  @override
  void worldStep(int worldId, double timeStep, int subStepCount) =>
      _call('f2d_world_step', [worldId, timeStep, subStepCount]);

  @override
  void worldSetGravity(int worldId, double x, double y) =>
      _call('f2d_world_set_gravity', [worldId, x, y]);

  @override
  (double, double) worldGetGravity(int worldId) {
    _call('f2d_world_get_gravity', [worldId, _out]);
    return _outVec2();
  }

  @override
  void worldEnableSleeping(int worldId, {required bool enabled}) =>
      _call('f2d_world_enable_sleeping', [worldId, _b(enabled)]);

  @override
  bool worldIsSleepingEnabled(int worldId) =>
      _callB('f2d_world_is_sleeping_enabled', [worldId]);

  @override
  void worldEnableContinuous(int worldId, {required bool enabled}) =>
      _call('f2d_world_enable_continuous', [worldId, _b(enabled)]);

  @override
  bool worldIsContinuousEnabled(int worldId) =>
      _callB('f2d_world_is_continuous_enabled', [worldId]);

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
    final namePointer = _runtime.writeCString(name);
    _call('f2d_create_body', [
      worldId,
      type,
      positionX,
      positionY,
      rotationCos,
      rotationSin,
      linearVelocityX,
      linearVelocityY,
      angularVelocity,
      linearDamping,
      angularDamping,
      gravityScale,
      sleepThreshold,
      namePointer,
      _b(enableSleep),
      _b(isAwake),
      _b(fixedRotation),
      _b(isBullet),
      _b(isEnabled),
      _b(allowFastRotation),
      _out,
    ]);
    return _outIdPair();
  }

  @override
  void destroyBody(int index1, int worldAndGeneration) =>
      _call('f2d_destroy_body', [index1, worldAndGeneration]);

  @override
  bool bodyIsValid(int index1, int worldAndGeneration) =>
      _callB('f2d_body_is_valid', [index1, worldAndGeneration]);

  @override
  (double, double) bodyGetPosition(int index1, int worldAndGeneration) {
    _call('f2d_body_get_position', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  (double, double) bodyGetRotation(int index1, int worldAndGeneration) {
    _call('f2d_body_get_rotation', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  void bodySetTransform(
    int index1,
    int worldAndGeneration,
    double positionX,
    double positionY,
    double rotationCos,
    double rotationSin,
  ) => _call('f2d_body_set_transform', [
    index1,
    worldAndGeneration,
    positionX,
    positionY,
    rotationCos,
    rotationSin,
  ]);

  @override
  (double, double) bodyGetLinearVelocity(int index1, int worldAndGeneration) {
    _call('f2d_body_get_linear_velocity', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  void bodySetLinearVelocity(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) =>
      _call('f2d_body_set_linear_velocity', [index1, worldAndGeneration, x, y]);

  @override
  double bodyGetAngularVelocity(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_angular_velocity', [index1, worldAndGeneration]);

  @override
  void bodySetAngularVelocity(
    int index1,
    int worldAndGeneration,
    double value,
  ) => _call('f2d_body_set_angular_velocity', [
    index1,
    worldAndGeneration,
    value,
  ]);

  @override
  void bodyApplyForce(
    int index1,
    int worldAndGeneration,
    double forceX,
    double forceY,
    double pointX,
    double pointY, {
    required bool wake,
  }) => _call('f2d_body_apply_force', [
    index1,
    worldAndGeneration,
    forceX,
    forceY,
    pointX,
    pointY,
    _b(wake),
  ]);

  @override
  void bodyApplyForceToCenter(
    int index1,
    int worldAndGeneration,
    double forceX,
    double forceY, {
    required bool wake,
  }) => _call('f2d_body_apply_force_to_center', [
    index1,
    worldAndGeneration,
    forceX,
    forceY,
    _b(wake),
  ]);

  @override
  void bodyApplyTorque(
    int index1,
    int worldAndGeneration,
    double torque, {
    required bool wake,
  }) => _call('f2d_body_apply_torque', [
    index1,
    worldAndGeneration,
    torque,
    _b(wake),
  ]);

  @override
  void bodyApplyLinearImpulse(
    int index1,
    int worldAndGeneration,
    double impulseX,
    double impulseY,
    double pointX,
    double pointY, {
    required bool wake,
  }) => _call('f2d_body_apply_linear_impulse', [
    index1,
    worldAndGeneration,
    impulseX,
    impulseY,
    pointX,
    pointY,
    _b(wake),
  ]);

  @override
  void bodyApplyLinearImpulseToCenter(
    int index1,
    int worldAndGeneration,
    double impulseX,
    double impulseY, {
    required bool wake,
  }) => _call('f2d_body_apply_linear_impulse_to_center', [
    index1,
    worldAndGeneration,
    impulseX,
    impulseY,
    _b(wake),
  ]);

  @override
  void bodyApplyAngularImpulse(
    int index1,
    int worldAndGeneration,
    double impulse, {
    required bool wake,
  }) => _call('f2d_body_apply_angular_impulse', [
    index1,
    worldAndGeneration,
    impulse,
    _b(wake),
  ]);

  @override
  double bodyGetMass(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_mass', [index1, worldAndGeneration]);

  @override
  double bodyGetRotationalInertia(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_rotational_inertia', [index1, worldAndGeneration]);

  @override
  (double, double) bodyGetLocalCenterOfMass(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_body_get_local_center', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  (double, double) bodyGetWorldCenterOfMass(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_body_get_world_center', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  void bodySetMassData(
    int index1,
    int worldAndGeneration,
    double mass,
    double rotationalInertia,
    double centerX,
    double centerY,
  ) => _call('f2d_body_set_mass_data', [
    index1,
    worldAndGeneration,
    mass,
    rotationalInertia,
    centerX,
    centerY,
  ]);

  @override
  void bodyApplyMassFromShapes(int index1, int worldAndGeneration) =>
      _call('f2d_body_apply_mass_from_shapes', [index1, worldAndGeneration]);

  @override
  int bodyGetType(int index1, int worldAndGeneration) =>
      _callI('f2d_body_get_type', [index1, worldAndGeneration]);

  @override
  void bodySetType(int index1, int worldAndGeneration, int type) =>
      _call('f2d_body_set_type', [index1, worldAndGeneration, type]);

  @override
  String bodyGetName(int index1, int worldAndGeneration) => _runtime
      .readCString(_callI('f2d_body_get_name', [index1, worldAndGeneration]));

  @override
  void bodySetName(int index1, int worldAndGeneration, String? name) => _call(
    'f2d_body_set_name',
    [index1, worldAndGeneration, _runtime.writeCString(name)],
  );

  @override
  bool bodyIsAwake(int index1, int worldAndGeneration) =>
      _callB('f2d_body_is_awake', [index1, worldAndGeneration]);

  @override
  void bodySetAwake(
    int index1,
    int worldAndGeneration, {
    required bool awake,
  }) => _call('f2d_body_set_awake', [index1, worldAndGeneration, _b(awake)]);

  @override
  bool bodyIsSleepEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_body_is_sleep_enabled', [index1, worldAndGeneration]);

  @override
  void bodyEnableSleep(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) =>
      _call('f2d_body_enable_sleep', [index1, worldAndGeneration, _b(enabled)]);

  @override
  double bodyGetSleepThreshold(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_sleep_threshold', [index1, worldAndGeneration]);

  @override
  void bodySetSleepThreshold(
    int index1,
    int worldAndGeneration,
    double value,
  ) => _call('f2d_body_set_sleep_threshold', [
    index1,
    worldAndGeneration,
    value,
  ]);

  @override
  bool bodyIsEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_body_is_enabled', [index1, worldAndGeneration]);

  @override
  void bodyDisable(int index1, int worldAndGeneration) =>
      _call('f2d_body_disable', [index1, worldAndGeneration]);

  @override
  void bodyEnable(int index1, int worldAndGeneration) =>
      _call('f2d_body_enable', [index1, worldAndGeneration]);

  @override
  bool bodyIsFixedRotation(int index1, int worldAndGeneration) =>
      _callB('f2d_body_is_fixed_rotation', [index1, worldAndGeneration]);

  @override
  void bodySetFixedRotation(
    int index1,
    int worldAndGeneration, {
    required bool flag,
  }) => _call('f2d_body_set_fixed_rotation', [
    index1,
    worldAndGeneration,
    _b(flag),
  ]);

  @override
  bool bodyIsBullet(int index1, int worldAndGeneration) =>
      _callB('f2d_body_is_bullet', [index1, worldAndGeneration]);

  @override
  void bodySetBullet(
    int index1,
    int worldAndGeneration, {
    required bool flag,
  }) => _call('f2d_body_set_bullet', [index1, worldAndGeneration, _b(flag)]);

  @override
  double bodyGetGravityScale(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_gravity_scale', [index1, worldAndGeneration]);

  @override
  void bodySetGravityScale(int index1, int worldAndGeneration, double scale) =>
      _call('f2d_body_set_gravity_scale', [index1, worldAndGeneration, scale]);

  @override
  double bodyGetLinearDamping(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_linear_damping', [index1, worldAndGeneration]);

  @override
  void bodySetLinearDamping(
    int index1,
    int worldAndGeneration,
    double damping,
  ) => _call('f2d_body_set_linear_damping', [
    index1,
    worldAndGeneration,
    damping,
  ]);

  @override
  double bodyGetAngularDamping(int index1, int worldAndGeneration) =>
      _callF('f2d_body_get_angular_damping', [index1, worldAndGeneration]);

  @override
  void bodySetAngularDamping(
    int index1,
    int worldAndGeneration,
    double damping,
  ) => _call('f2d_body_set_angular_damping', [
    index1,
    worldAndGeneration,
    damping,
  ]);

  @override
  (double, double) bodyGetWorldPoint(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) {
    _call('f2d_body_get_world_point', [index1, worldAndGeneration, x, y, _out]);
    return _outVec2();
  }

  @override
  (double, double) bodyGetLocalPoint(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) {
    _call('f2d_body_get_local_point', [index1, worldAndGeneration, x, y, _out]);
    return _outVec2();
  }

  @override
  List<int> bodyGetShapes(int index1, int worldAndGeneration) {
    final count = _callI('f2d_body_get_shape_count', [
      index1,
      worldAndGeneration,
    ]);
    if (count == 0) {
      return const [];
    }
    final buffer = _runtime.bulk(count * 8);
    final written = _callI('f2d_body_get_shapes', [
      index1,
      worldAndGeneration,
      buffer,
      count,
    ]);
    return _idPairs(_runtime.readI32List(buffer, written * 2));
  }

  @override
  List<int> bodyGetJoints(int index1, int worldAndGeneration) {
    final count = _callI('f2d_body_get_joint_count', [
      index1,
      worldAndGeneration,
    ]);
    if (count == 0) {
      return const [];
    }
    final buffer = _runtime.bulk(count * 8);
    final written = _callI('f2d_body_get_joints', [
      index1,
      worldAndGeneration,
      buffer,
      count,
    ]);
    return _idPairs(_runtime.readI32List(buffer, written * 2));
  }

  // Shapes.

  void _writeShapeDef(RawShapeDef definition) {
    final pointer = _shapeDefPointer;
    final (categoryLow, categoryHigh) = _splitBits(definition.categoryBits);
    final (maskLow, maskHigh) = _splitBits(definition.maskBits);
    _runtime
      ..writeF32(pointer, definition.friction)
      ..writeF32(pointer + 4, definition.restitution)
      ..writeF32(pointer + 8, definition.rollingResistance)
      ..writeF32(pointer + 12, definition.tangentSpeed)
      ..writeI32(pointer + 16, definition.userMaterialId)
      ..writeI32(pointer + 20, definition.customColor)
      ..writeF32(pointer + 24, definition.density)
      ..writeI32(pointer + 28, categoryLow)
      ..writeI32(pointer + 32, categoryHigh)
      ..writeI32(pointer + 36, maskLow)
      ..writeI32(pointer + 40, maskHigh)
      ..writeI32(pointer + 44, definition.groupIndex)
      ..writeI32(pointer + 48, _b(definition.isSensor))
      ..writeI32(pointer + 52, _b(definition.enableSensorEvents))
      ..writeI32(pointer + 56, _b(definition.enableContactEvents))
      ..writeI32(pointer + 60, _b(definition.enableHitEvents))
      ..writeI32(pointer + 64, _b(definition.enablePreSolveEvents))
      ..writeI32(pointer + 68, _b(definition.invokeContactCreation))
      ..writeI32(pointer + 72, _b(definition.updateBodyMass));
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
    _writeShapeDef(definition);
    _call('f2d_create_circle_shape', [
      bodyIndex1,
      bodyWorldAndGeneration,
      _shapeDefPointer,
      centerX,
      centerY,
      radius,
      _out,
    ]);
    return _outIdPair();
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
    _writeShapeDef(definition);
    _call('f2d_create_capsule_shape', [
      bodyIndex1,
      bodyWorldAndGeneration,
      _shapeDefPointer,
      center1X,
      center1Y,
      center2X,
      center2Y,
      radius,
      _out,
    ]);
    return _outIdPair();
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
    _writeShapeDef(definition);
    _call('f2d_create_segment_shape', [
      bodyIndex1,
      bodyWorldAndGeneration,
      _shapeDefPointer,
      point1X,
      point1Y,
      point2X,
      point2Y,
      _out,
    ]);
    return _outIdPair();
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
    _writeShapeDef(definition);
    _call('f2d_create_box_shape', [
      bodyIndex1,
      bodyWorldAndGeneration,
      _shapeDefPointer,
      halfWidth,
      halfHeight,
      centerX,
      centerY,
      rotationCos,
      rotationSin,
      radius,
      _out,
    ]);
    return _outIdPair();
  }

  @override
  (int, int) createPolygonShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required List<double> points,
    required double radius,
    required RawShapeDef definition,
  }) {
    _writeShapeDef(definition);
    final buffer = _runtime.bulk(points.length * 4);
    _runtime.writeF32List(buffer, points);
    final ok = _callB('f2d_create_polygon_shape', [
      bodyIndex1,
      bodyWorldAndGeneration,
      _shapeDefPointer,
      buffer,
      points.length ~/ 2,
      radius,
      _out,
    ]);
    if (!ok) {
      throw ArgumentError(
        'The points do not form a valid convex hull: at least three '
        'distinct, non-collinear points are required',
      );
    }
    return _outIdPair();
  }

  @override
  void destroyShape(
    int index1,
    int worldAndGeneration, {
    required bool updateBodyMass,
  }) => _call('f2d_destroy_shape', [
    index1,
    worldAndGeneration,
    _b(updateBodyMass),
  ]);

  @override
  bool shapeIsValid(int index1, int worldAndGeneration) =>
      _callB('f2d_shape_is_valid', [index1, worldAndGeneration]);

  @override
  int shapeGetType(int index1, int worldAndGeneration) =>
      _callI('f2d_shape_get_type', [index1, worldAndGeneration]);

  @override
  (int, int) shapeGetBody(int index1, int worldAndGeneration) {
    _call('f2d_shape_get_body', [index1, worldAndGeneration, _out]);
    return _outIdPair();
  }

  @override
  bool shapeIsSensor(int index1, int worldAndGeneration) =>
      _callB('f2d_shape_is_sensor', [index1, worldAndGeneration]);

  @override
  double shapeGetDensity(int index1, int worldAndGeneration) =>
      _callF('f2d_shape_get_density', [index1, worldAndGeneration]);

  @override
  void shapeSetDensity(
    int index1,
    int worldAndGeneration,
    double density, {
    required bool updateBodyMass,
  }) => _call('f2d_shape_set_density', [
    index1,
    worldAndGeneration,
    density,
    _b(updateBodyMass),
  ]);

  @override
  double shapeGetFriction(int index1, int worldAndGeneration) =>
      _callF('f2d_shape_get_friction', [index1, worldAndGeneration]);

  @override
  void shapeSetFriction(int index1, int worldAndGeneration, double friction) =>
      _call('f2d_shape_set_friction', [index1, worldAndGeneration, friction]);

  @override
  double shapeGetRestitution(int index1, int worldAndGeneration) =>
      _callF('f2d_shape_get_restitution', [index1, worldAndGeneration]);

  @override
  void shapeSetRestitution(
    int index1,
    int worldAndGeneration,
    double restitution,
  ) => _call('f2d_shape_set_restitution', [
    index1,
    worldAndGeneration,
    restitution,
  ]);

  @override
  (int, int, int) shapeGetFilter(int index1, int worldAndGeneration) {
    _call('f2d_shape_get_filter', [index1, worldAndGeneration, _out]);
    return (
      _joinBits(_runtime.readI32(_out), _runtime.readI32(_out + 4)),
      _joinBits(_runtime.readI32(_out + 8), _runtime.readI32(_out + 12)),
      _runtime.readI32(_out + 16),
    );
  }

  @override
  void shapeSetFilter(
    int index1,
    int worldAndGeneration,
    int categoryBits,
    int maskBits,
    int groupIndex,
  ) {
    final (categoryLow, categoryHigh) = _splitBits(categoryBits);
    final (maskLow, maskHigh) = _splitBits(maskBits);
    _call('f2d_shape_set_filter', [
      index1,
      worldAndGeneration,
      categoryLow,
      categoryHigh,
      maskLow,
      maskHigh,
      groupIndex,
    ]);
  }

  @override
  bool shapeAreSensorEventsEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_shape_are_sensor_events_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void shapeEnableSensorEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_shape_enable_sensor_events', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  bool shapeAreContactEventsEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_shape_are_contact_events_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void shapeEnableContactEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_shape_enable_contact_events', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  bool shapeAreHitEventsEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_shape_are_hit_events_enabled', [index1, worldAndGeneration]);

  @override
  void shapeEnableHitEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_shape_enable_hit_events', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  bool shapeArePreSolveEventsEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_shape_are_pre_solve_events_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void shapeEnablePreSolveEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_shape_enable_pre_solve_events', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  bool shapeTestPoint(int index1, int worldAndGeneration, double x, double y) =>
      _callB('f2d_shape_test_point', [index1, worldAndGeneration, x, y]);

  @override
  (double, double, double) shapeGetCircle(int index1, int worldAndGeneration) {
    _call('f2d_shape_get_circle', [index1, worldAndGeneration, _out]);
    return (
      _runtime.readF32(_out),
      _runtime.readF32(_out + 4),
      _runtime.readF32(_out + 8),
    );
  }

  @override
  (double, double, double, double, double) shapeGetCapsule(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_shape_get_capsule', [index1, worldAndGeneration, _out]);
    return (
      _runtime.readF32(_out),
      _runtime.readF32(_out + 4),
      _runtime.readF32(_out + 8),
      _runtime.readF32(_out + 12),
      _runtime.readF32(_out + 16),
    );
  }

  @override
  (double, double, double, double) shapeGetSegment(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_shape_get_segment', [index1, worldAndGeneration, _out]);
    return (
      _runtime.readF32(_out),
      _runtime.readF32(_out + 4),
      _runtime.readF32(_out + 8),
      _runtime.readF32(_out + 12),
    );
  }

  @override
  (double, double, double, double) shapeGetChainSegment(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_shape_get_chain_segment', [index1, worldAndGeneration, _out]);
    return (
      _runtime.readF32(_out),
      _runtime.readF32(_out + 4),
      _runtime.readF32(_out + 8),
      _runtime.readF32(_out + 12),
    );
  }

  @override
  ({List<double> points, double radius}) shapeGetPolygon(
    int index1,
    int worldAndGeneration,
  ) {
    final count = _callI('f2d_shape_get_polygon', [
      index1,
      worldAndGeneration,
      _out,
    ]);
    return (
      points: _runtime.readF32List(_out + 4, count * 2),
      radius: _runtime.readF32(_out),
    );
  }

  @override
  (double, double, double, double) shapeGetAabb(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_shape_get_aabb', [index1, worldAndGeneration, _out]);
    return (
      _runtime.readF32(_out),
      _runtime.readF32(_out + 4),
      _runtime.readF32(_out + 8),
      _runtime.readF32(_out + 12),
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
    final pointBytes = points.length * 4;
    final buffer = _runtime.bulk(pointBytes + materials.length * 4);
    _runtime
      ..writeF32List(buffer, points)
      ..writeF32List(buffer + pointBytes, materials);
    final (categoryLow, categoryHigh) = _splitBits(categoryBits);
    final (maskLow, maskHigh) = _splitBits(maskBits);
    _call('f2d_create_chain', [
      bodyIndex1,
      bodyWorldAndGeneration,
      buffer,
      points.length ~/ 2,
      buffer + pointBytes,
      materials.length ~/ 6,
      categoryLow,
      categoryHigh,
      maskLow,
      maskHigh,
      groupIndex,
      _b(isLoop),
      _b(enableSensorEvents),
      _out,
    ]);
    return _outIdPair();
  }

  @override
  void destroyChain(int index1, int worldAndGeneration) =>
      _call('f2d_destroy_chain', [index1, worldAndGeneration]);

  @override
  bool chainIsValid(int index1, int worldAndGeneration) =>
      _callB('f2d_chain_is_valid', [index1, worldAndGeneration]);

  @override
  void chainSetFriction(int index1, int worldAndGeneration, double friction) =>
      _call('f2d_chain_set_friction', [index1, worldAndGeneration, friction]);

  @override
  double chainGetFriction(int index1, int worldAndGeneration) =>
      _callF('f2d_chain_get_friction', [index1, worldAndGeneration]);

  @override
  void chainSetRestitution(
    int index1,
    int worldAndGeneration,
    double restitution,
  ) => _call('f2d_chain_set_restitution', [
    index1,
    worldAndGeneration,
    restitution,
  ]);

  @override
  double chainGetRestitution(int index1, int worldAndGeneration) =>
      _callF('f2d_chain_get_restitution', [index1, worldAndGeneration]);

  @override
  List<int> chainGetSegments(int index1, int worldAndGeneration) {
    final count = _callI('f2d_chain_get_segment_count', [
      index1,
      worldAndGeneration,
    ]);
    if (count == 0) {
      return const [];
    }
    final buffer = _runtime.bulk(count * 8);
    final written = _callI('f2d_chain_get_segments', [
      index1,
      worldAndGeneration,
      buffer,
      count,
    ]);
    return _idPairs(_runtime.readI32List(buffer, written * 2));
  }

  // Joints.

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
    _call('f2d_create_distance_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      localAnchorA.$1,
      localAnchorA.$2,
      localAnchorB.$1,
      localAnchorB.$2,
      length,
      _b(enableSpring),
      hertz,
      dampingRatio,
      _b(enableLimit),
      minLength,
      maxLength,
      _b(enableMotor),
      maxMotorForce,
      motorSpeed,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
  }

  @override
  (int, int) createFilterJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
  }) {
    _call('f2d_create_filter_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      _out,
    ]);
    return _outIdPair();
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
    _call('f2d_create_motor_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      linearOffset.$1,
      linearOffset.$2,
      angularOffset,
      maxForce,
      maxTorque,
      correctionFactor,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
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
    _call('f2d_create_mouse_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      target.$1,
      target.$2,
      hertz,
      dampingRatio,
      maxForce,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
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
    _call('f2d_create_prismatic_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      localAnchorA.$1,
      localAnchorA.$2,
      localAnchorB.$1,
      localAnchorB.$2,
      localAxisA.$1,
      localAxisA.$2,
      referenceAngle,
      targetTranslation,
      _b(enableSpring),
      hertz,
      dampingRatio,
      _b(enableLimit),
      lowerTranslation,
      upperTranslation,
      _b(enableMotor),
      maxMotorForce,
      motorSpeed,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
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
    _call('f2d_create_revolute_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      localAnchorA.$1,
      localAnchorA.$2,
      localAnchorB.$1,
      localAnchorB.$2,
      referenceAngle,
      targetAngle,
      _b(enableSpring),
      hertz,
      dampingRatio,
      _b(enableLimit),
      lowerAngle,
      upperAngle,
      _b(enableMotor),
      maxMotorTorque,
      motorSpeed,
      drawSize,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
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
    _call('f2d_create_weld_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      localAnchorA.$1,
      localAnchorA.$2,
      localAnchorB.$1,
      localAnchorB.$2,
      referenceAngle,
      linearHertz,
      angularHertz,
      linearDampingRatio,
      angularDampingRatio,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
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
    _call('f2d_create_wheel_joint', [
      worldId,
      bodyA.$1,
      bodyA.$2,
      bodyB.$1,
      bodyB.$2,
      localAnchorA.$1,
      localAnchorA.$2,
      localAnchorB.$1,
      localAnchorB.$2,
      localAxisA.$1,
      localAxisA.$2,
      _b(enableSpring),
      hertz,
      dampingRatio,
      _b(enableLimit),
      lowerTranslation,
      upperTranslation,
      _b(enableMotor),
      maxMotorTorque,
      motorSpeed,
      _b(collideConnected),
      _out,
    ]);
    return _outIdPair();
  }

  @override
  void destroyJoint(int index1, int worldAndGeneration) =>
      _call('f2d_destroy_joint', [index1, worldAndGeneration]);

  @override
  bool jointIsValid(int index1, int worldAndGeneration) =>
      _callB('f2d_joint_is_valid', [index1, worldAndGeneration]);

  @override
  int jointGetType(int index1, int worldAndGeneration) =>
      _callI('f2d_joint_get_type', [index1, worldAndGeneration]);

  @override
  (int, int) jointGetBodyA(int index1, int worldAndGeneration) {
    _call('f2d_joint_get_body_a', [index1, worldAndGeneration, _out]);
    return _outIdPair();
  }

  @override
  (int, int) jointGetBodyB(int index1, int worldAndGeneration) {
    _call('f2d_joint_get_body_b', [index1, worldAndGeneration, _out]);
    return _outIdPair();
  }

  @override
  (double, double) jointGetLocalAnchorA(int index1, int worldAndGeneration) {
    _call('f2d_joint_get_local_anchor_a', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  (double, double) jointGetLocalAnchorB(int index1, int worldAndGeneration) {
    _call('f2d_joint_get_local_anchor_b', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  bool jointGetCollideConnected(int index1, int worldAndGeneration) =>
      _callB('f2d_joint_get_collide_connected', [index1, worldAndGeneration]);

  @override
  void jointSetCollideConnected(
    int index1,
    int worldAndGeneration, {
    required bool value,
  }) => _call('f2d_joint_set_collide_connected', [
    index1,
    worldAndGeneration,
    _b(value),
  ]);

  @override
  void jointWakeBodies(int index1, int worldAndGeneration) =>
      _call('f2d_joint_wake_bodies', [index1, worldAndGeneration]);

  @override
  (double, double) jointGetConstraintForce(int index1, int worldAndGeneration) {
    _call('f2d_joint_get_constraint_force', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  double jointGetConstraintTorque(int index1, int worldAndGeneration) =>
      _callF('f2d_joint_get_constraint_torque', [index1, worldAndGeneration]);

  // Distance joint.

  @override
  double distanceJointGetLength(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_length', [index1, worldAndGeneration]);

  @override
  void distanceJointSetLength(
    int index1,
    int worldAndGeneration,
    double length,
  ) => _call('f2d_distance_joint_set_length', [
    index1,
    worldAndGeneration,
    length,
  ]);

  @override
  double distanceJointGetCurrentLength(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_current_length', [
        index1,
        worldAndGeneration,
      ]);

  @override
  bool distanceJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_distance_joint_is_spring_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void distanceJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_distance_joint_enable_spring', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double distanceJointGetSpringHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_spring_hertz', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void distanceJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_distance_joint_set_spring_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double distanceJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  ) => _callF('f2d_distance_joint_get_spring_damping_ratio', [
    index1,
    worldAndGeneration,
  ]);

  @override
  void distanceJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_distance_joint_set_spring_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  @override
  bool distanceJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_distance_joint_is_limit_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void distanceJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_distance_joint_enable_limit', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double distanceJointGetMinLength(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_min_length', [index1, worldAndGeneration]);

  @override
  double distanceJointGetMaxLength(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_max_length', [index1, worldAndGeneration]);

  @override
  void distanceJointSetLengthRange(
    int index1,
    int worldAndGeneration,
    double minLength,
    double maxLength,
  ) => _call('f2d_distance_joint_set_length_range', [
    index1,
    worldAndGeneration,
    minLength,
    maxLength,
  ]);

  @override
  bool distanceJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_distance_joint_is_motor_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void distanceJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_distance_joint_enable_motor', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double distanceJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_motor_speed', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void distanceJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => _call('f2d_distance_joint_set_motor_speed', [
    index1,
    worldAndGeneration,
    speed,
  ]);

  @override
  double distanceJointGetMaxMotorForce(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_max_motor_force', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void distanceJointSetMaxMotorForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => _call('f2d_distance_joint_set_max_motor_force', [
    index1,
    worldAndGeneration,
    force,
  ]);

  @override
  double distanceJointGetMotorForce(int index1, int worldAndGeneration) =>
      _callF('f2d_distance_joint_get_motor_force', [
        index1,
        worldAndGeneration,
      ]);

  // Motor joint.

  @override
  (double, double) motorJointGetLinearOffset(
    int index1,
    int worldAndGeneration,
  ) {
    _call('f2d_motor_joint_get_linear_offset', [
      index1,
      worldAndGeneration,
      _out,
    ]);
    return _outVec2();
  }

  @override
  void motorJointSetLinearOffset(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) => _call('f2d_motor_joint_set_linear_offset', [
    index1,
    worldAndGeneration,
    x,
    y,
  ]);

  @override
  double motorJointGetAngularOffset(int index1, int worldAndGeneration) =>
      _callF('f2d_motor_joint_get_angular_offset', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void motorJointSetAngularOffset(
    int index1,
    int worldAndGeneration,
    double offset,
  ) => _call('f2d_motor_joint_set_angular_offset', [
    index1,
    worldAndGeneration,
    offset,
  ]);

  @override
  double motorJointGetMaxForce(int index1, int worldAndGeneration) =>
      _callF('f2d_motor_joint_get_max_force', [index1, worldAndGeneration]);

  @override
  void motorJointSetMaxForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => _call('f2d_motor_joint_set_max_force', [
    index1,
    worldAndGeneration,
    force,
  ]);

  @override
  double motorJointGetMaxTorque(int index1, int worldAndGeneration) =>
      _callF('f2d_motor_joint_get_max_torque', [index1, worldAndGeneration]);

  @override
  void motorJointSetMaxTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  ) => _call('f2d_motor_joint_set_max_torque', [
    index1,
    worldAndGeneration,
    torque,
  ]);

  @override
  double motorJointGetCorrectionFactor(int index1, int worldAndGeneration) =>
      _callF('f2d_motor_joint_get_correction_factor', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void motorJointSetCorrectionFactor(
    int index1,
    int worldAndGeneration,
    double factor,
  ) => _call('f2d_motor_joint_set_correction_factor', [
    index1,
    worldAndGeneration,
    factor,
  ]);

  // Mouse joint.

  @override
  (double, double) mouseJointGetTarget(int index1, int worldAndGeneration) {
    _call('f2d_mouse_joint_get_target', [index1, worldAndGeneration, _out]);
    return _outVec2();
  }

  @override
  void mouseJointSetTarget(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  ) => _call('f2d_mouse_joint_set_target', [index1, worldAndGeneration, x, y]);

  @override
  double mouseJointGetSpringHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_mouse_joint_get_spring_hertz', [index1, worldAndGeneration]);

  @override
  void mouseJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_mouse_joint_set_spring_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double mouseJointGetSpringDampingRatio(int index1, int worldAndGeneration) =>
      _callF('f2d_mouse_joint_get_spring_damping_ratio', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void mouseJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_mouse_joint_set_spring_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  @override
  double mouseJointGetMaxForce(int index1, int worldAndGeneration) =>
      _callF('f2d_mouse_joint_get_max_force', [index1, worldAndGeneration]);

  @override
  void mouseJointSetMaxForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => _call('f2d_mouse_joint_set_max_force', [
    index1,
    worldAndGeneration,
    force,
  ]);

  // Prismatic joint.

  @override
  bool prismaticJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_prismatic_joint_is_spring_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_prismatic_joint_enable_spring', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double prismaticJointGetSpringHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_spring_hertz', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_prismatic_joint_set_spring_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double prismaticJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  ) => _callF('f2d_prismatic_joint_get_spring_damping_ratio', [
    index1,
    worldAndGeneration,
  ]);

  @override
  void prismaticJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_prismatic_joint_set_spring_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  @override
  double prismaticJointGetTargetTranslation(
    int index1,
    int worldAndGeneration,
  ) => _callF('f2d_prismatic_joint_get_target_translation', [
    index1,
    worldAndGeneration,
  ]);

  @override
  void prismaticJointSetTargetTranslation(
    int index1,
    int worldAndGeneration,
    double value,
  ) => _call('f2d_prismatic_joint_set_target_translation', [
    index1,
    worldAndGeneration,
    value,
  ]);

  @override
  bool prismaticJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_prismatic_joint_is_limit_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_prismatic_joint_enable_limit', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double prismaticJointGetLowerLimit(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_lower_limit', [
        index1,
        worldAndGeneration,
      ]);

  @override
  double prismaticJointGetUpperLimit(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_upper_limit', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  ) => _call('f2d_prismatic_joint_set_limits', [
    index1,
    worldAndGeneration,
    lower,
    upper,
  ]);

  @override
  bool prismaticJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_prismatic_joint_is_motor_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_prismatic_joint_enable_motor', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double prismaticJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_motor_speed', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => _call('f2d_prismatic_joint_set_motor_speed', [
    index1,
    worldAndGeneration,
    speed,
  ]);

  @override
  double prismaticJointGetMaxMotorForce(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_max_motor_force', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void prismaticJointSetMaxMotorForce(
    int index1,
    int worldAndGeneration,
    double force,
  ) => _call('f2d_prismatic_joint_set_max_motor_force', [
    index1,
    worldAndGeneration,
    force,
  ]);

  @override
  double prismaticJointGetMotorForce(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_motor_force', [
        index1,
        worldAndGeneration,
      ]);

  @override
  double prismaticJointGetTranslation(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_translation', [
        index1,
        worldAndGeneration,
      ]);

  @override
  double prismaticJointGetSpeed(int index1, int worldAndGeneration) =>
      _callF('f2d_prismatic_joint_get_speed', [index1, worldAndGeneration]);

  // Revolute joint.

  @override
  bool revoluteJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_revolute_joint_is_spring_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_revolute_joint_enable_spring', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double revoluteJointGetSpringHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_spring_hertz', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_revolute_joint_set_spring_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double revoluteJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  ) => _callF('f2d_revolute_joint_get_spring_damping_ratio', [
    index1,
    worldAndGeneration,
  ]);

  @override
  void revoluteJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_revolute_joint_set_spring_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  @override
  double revoluteJointGetTargetAngle(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_target_angle', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointSetTargetAngle(
    int index1,
    int worldAndGeneration,
    double angle,
  ) => _call('f2d_revolute_joint_set_target_angle', [
    index1,
    worldAndGeneration,
    angle,
  ]);

  @override
  double revoluteJointGetAngle(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_angle', [index1, worldAndGeneration]);

  @override
  bool revoluteJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_revolute_joint_is_limit_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_revolute_joint_enable_limit', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double revoluteJointGetLowerLimit(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_lower_limit', [
        index1,
        worldAndGeneration,
      ]);

  @override
  double revoluteJointGetUpperLimit(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_upper_limit', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  ) => _call('f2d_revolute_joint_set_limits', [
    index1,
    worldAndGeneration,
    lower,
    upper,
  ]);

  @override
  bool revoluteJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_revolute_joint_is_motor_enabled', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_revolute_joint_enable_motor', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double revoluteJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_motor_speed', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => _call('f2d_revolute_joint_set_motor_speed', [
    index1,
    worldAndGeneration,
    speed,
  ]);

  @override
  double revoluteJointGetMaxMotorTorque(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_max_motor_torque', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void revoluteJointSetMaxMotorTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  ) => _call('f2d_revolute_joint_set_max_motor_torque', [
    index1,
    worldAndGeneration,
    torque,
  ]);

  @override
  double revoluteJointGetMotorTorque(int index1, int worldAndGeneration) =>
      _callF('f2d_revolute_joint_get_motor_torque', [
        index1,
        worldAndGeneration,
      ]);

  // Weld joint.

  @override
  double weldJointGetLinearHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_weld_joint_get_linear_hertz', [index1, worldAndGeneration]);

  @override
  void weldJointSetLinearHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_weld_joint_set_linear_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double weldJointGetAngularHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_weld_joint_get_angular_hertz', [index1, worldAndGeneration]);

  @override
  void weldJointSetAngularHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_weld_joint_set_angular_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double weldJointGetLinearDampingRatio(int index1, int worldAndGeneration) =>
      _callF('f2d_weld_joint_get_linear_damping_ratio', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void weldJointSetLinearDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_weld_joint_set_linear_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  @override
  double weldJointGetAngularDampingRatio(int index1, int worldAndGeneration) =>
      _callF('f2d_weld_joint_get_angular_damping_ratio', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void weldJointSetAngularDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_weld_joint_set_angular_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  // Wheel joint.

  @override
  bool wheelJointIsSpringEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_wheel_joint_is_spring_enabled', [index1, worldAndGeneration]);

  @override
  void wheelJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_wheel_joint_enable_spring', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double wheelJointGetSpringHertz(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_spring_hertz', [index1, worldAndGeneration]);

  @override
  void wheelJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  ) => _call('f2d_wheel_joint_set_spring_hertz', [
    index1,
    worldAndGeneration,
    hertz,
  ]);

  @override
  double wheelJointGetSpringDampingRatio(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_spring_damping_ratio', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void wheelJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  ) => _call('f2d_wheel_joint_set_spring_damping_ratio', [
    index1,
    worldAndGeneration,
    ratio,
  ]);

  @override
  bool wheelJointIsLimitEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_wheel_joint_is_limit_enabled', [index1, worldAndGeneration]);

  @override
  void wheelJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_wheel_joint_enable_limit', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double wheelJointGetLowerLimit(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_lower_limit', [index1, worldAndGeneration]);

  @override
  double wheelJointGetUpperLimit(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_upper_limit', [index1, worldAndGeneration]);

  @override
  void wheelJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  ) => _call('f2d_wheel_joint_set_limits', [
    index1,
    worldAndGeneration,
    lower,
    upper,
  ]);

  @override
  bool wheelJointIsMotorEnabled(int index1, int worldAndGeneration) =>
      _callB('f2d_wheel_joint_is_motor_enabled', [index1, worldAndGeneration]);

  @override
  void wheelJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  }) => _call('f2d_wheel_joint_enable_motor', [
    index1,
    worldAndGeneration,
    _b(enabled),
  ]);

  @override
  double wheelJointGetMotorSpeed(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_motor_speed', [index1, worldAndGeneration]);

  @override
  void wheelJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  ) => _call('f2d_wheel_joint_set_motor_speed', [
    index1,
    worldAndGeneration,
    speed,
  ]);

  @override
  double wheelJointGetMaxMotorTorque(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_max_motor_torque', [
        index1,
        worldAndGeneration,
      ]);

  @override
  void wheelJointSetMaxMotorTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  ) => _call('f2d_wheel_joint_set_max_motor_torque', [
    index1,
    worldAndGeneration,
    torque,
  ]);

  @override
  double wheelJointGetMotorTorque(int index1, int worldAndGeneration) =>
      _callF('f2d_wheel_joint_get_motor_torque', [index1, worldAndGeneration]);

  // Events.

  @override
  RawContactEvents worldGetContactEvents(int worldId) {
    _call('f2d_world_get_contact_event_counts', [worldId, _out]);
    final beginCount = _runtime.readI32(_out);
    final endCount = _runtime.readI32(_out + 4);
    final hitCount = _runtime.readI32(_out + 8);

    var begin = <RawContactBeginEvent>[];
    if (beginCount > 0) {
      final buffer = _runtime.bulk(beginCount * 13 * 8);
      final written = _callI('f2d_world_get_contact_begin_events', [
        worldId,
        buffer,
      ]);
      final data = _runtime.readF64List(buffer, written * 13);
      begin = [
        for (var i = 0; i < written; i++)
          (
            shapeAIndex1: data[i * 13].toInt(),
            shapeAWorldAndGeneration: data[i * 13 + 1].toInt(),
            shapeBIndex1: data[i * 13 + 2].toInt(),
            shapeBWorldAndGeneration: data[i * 13 + 3].toInt(),
            normalX: data[i * 13 + 4],
            normalY: data[i * 13 + 5],
            points: [
              for (var p = 0; p < data[i * 13 + 6].toInt(); p++)
                (
                  x: data[i * 13 + 7 + 3 * p],
                  y: data[i * 13 + 8 + 3 * p],
                  separation: data[i * 13 + 9 + 3 * p],
                ),
            ],
          ),
      ];
    }

    var end = <RawContactEndEvent>[];
    if (endCount > 0) {
      final buffer = _runtime.bulk(endCount * 4 * 8);
      final written = _callI('f2d_world_get_contact_end_events', [
        worldId,
        buffer,
      ]);
      final data = _runtime.readF64List(buffer, written * 4);
      end = [
        for (var i = 0; i < written; i++)
          (
            shapeAIndex1: data[i * 4].toInt(),
            shapeAWorldAndGeneration: data[i * 4 + 1].toInt(),
            shapeBIndex1: data[i * 4 + 2].toInt(),
            shapeBWorldAndGeneration: data[i * 4 + 3].toInt(),
          ),
      ];
    }

    var hit = <RawContactHitEvent>[];
    if (hitCount > 0) {
      final buffer = _runtime.bulk(hitCount * 9 * 8);
      final written = _callI('f2d_world_get_contact_hit_events', [
        worldId,
        buffer,
      ]);
      final data = _runtime.readF64List(buffer, written * 9);
      hit = [
        for (var i = 0; i < written; i++)
          (
            shapeAIndex1: data[i * 9].toInt(),
            shapeAWorldAndGeneration: data[i * 9 + 1].toInt(),
            shapeBIndex1: data[i * 9 + 2].toInt(),
            shapeBWorldAndGeneration: data[i * 9 + 3].toInt(),
            pointX: data[i * 9 + 4],
            pointY: data[i * 9 + 5],
            normalX: data[i * 9 + 6],
            normalY: data[i * 9 + 7],
            approachSpeed: data[i * 9 + 8],
          ),
      ];
    }

    return (begin: begin, end: end, hit: hit);
  }

  @override
  RawSensorEvents worldGetSensorEvents(int worldId) {
    _call('f2d_world_get_sensor_event_counts', [worldId, _out]);
    final beginCount = _runtime.readI32(_out);
    final endCount = _runtime.readI32(_out + 4);

    List<RawSensorEvent> read(String function, int count) {
      if (count == 0) {
        return const [];
      }
      final buffer = _runtime.bulk(count * 4 * 8);
      final written = _callI(function, [worldId, buffer]);
      final data = _runtime.readF64List(buffer, written * 4);
      return [
        for (var i = 0; i < written; i++)
          (
            sensorIndex1: data[i * 4].toInt(),
            sensorWorldAndGeneration: data[i * 4 + 1].toInt(),
            visitorIndex1: data[i * 4 + 2].toInt(),
            visitorWorldAndGeneration: data[i * 4 + 3].toInt(),
          ),
      ];
    }

    return (
      begin: read('f2d_world_get_sensor_begin_events', beginCount),
      end: read('f2d_world_get_sensor_end_events', endCount),
    );
  }

  @override
  List<RawBodyMoveEvent> worldGetBodyEvents(int worldId) {
    final count = _callI('f2d_world_get_body_event_count', [worldId]);
    if (count == 0) {
      return const [];
    }
    final buffer = _runtime.bulk(count * 7 * 8);
    final written = _callI('f2d_world_get_body_events', [worldId, buffer]);
    final data = _runtime.readF64List(buffer, written * 7);
    return [
      for (var i = 0; i < written; i++)
        (
          bodyIndex1: data[i * 7].toInt(),
          bodyWorldAndGeneration: data[i * 7 + 1].toInt(),
          x: data[i * 7 + 2],
          y: data[i * 7 + 3],
          rotationCos: data[i * 7 + 4],
          rotationSin: data[i * 7 + 5],
          fellAsleep: data[i * 7 + 6] != 0,
        ),
    ];
  }

  // Queries.

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
    final (categoryLow, categoryHigh) = _splitBits(categoryBits);
    final (maskLow, maskHigh) = _splitBits(maskBits);
    final hitSomething = _callB('f2d_world_cast_ray_closest', [
      worldId,
      originX,
      originY,
      translationX,
      translationY,
      categoryLow,
      categoryHigh,
      maskLow,
      maskHigh,
      _out,
    ]);
    if (!hitSomething) {
      return null;
    }
    final data = _runtime.readF64List(_out, 7);
    return (
      shapeIndex1: data[0].toInt(),
      shapeWorldAndGeneration: data[1].toInt(),
      pointX: data[2],
      pointY: data[3],
      normalX: data[4],
      normalY: data[5],
      fraction: data[6],
    );
  }

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
    final (categoryLow, categoryHigh) = _splitBits(categoryBits);
    final (maskLow, maskHigh) = _splitBits(maskBits);
    final previous = _runtime.callbacks.castRay;
    _runtime.callbacks.castRay =
        (
          index1,
          worldAndGeneration,
          pointX,
          pointY,
          normalX,
          normalY,
          fraction,
        ) => callback((
          shapeIndex1: index1,
          shapeWorldAndGeneration: worldAndGeneration,
          pointX: pointX,
          pointY: pointY,
          normalX: normalX,
          normalY: normalY,
          fraction: fraction,
        ));
    try {
      _call('f2d_world_cast_ray', [
        worldId,
        originX,
        originY,
        translationX,
        translationY,
        categoryLow,
        categoryHigh,
        maskLow,
        maskHigh,
      ]);
    } finally {
      _runtime.callbacks.castRay = previous;
    }
  }

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
    final (categoryLow, categoryHigh) = _splitBits(categoryBits);
    final (maskLow, maskHigh) = _splitBits(maskBits);
    final results = <int>[];
    final previous = _runtime.callbacks.overlap;
    _runtime.callbacks.overlap = (index1, worldAndGeneration) {
      results
        ..add(index1)
        ..add(worldAndGeneration);
      return true;
    };
    try {
      _call('f2d_world_overlap_aabb', [
        worldId,
        lowerX,
        lowerY,
        upperX,
        upperY,
        categoryLow,
        categoryHigh,
        maskLow,
        maskHigh,
      ]);
    } finally {
      _runtime.callbacks.overlap = previous;
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
    final (maskLow, maskHigh) = _splitBits(maskBits);
    _call('f2d_world_explode', [
      worldId,
      maskLow,
      maskHigh,
      positionX,
      positionY,
      radius,
      falloff,
      impulsePerLength,
    ]);
  }

  // Simulation callbacks, dispatched by the world index embedded in the
  // shape ids (the low 16 bits of worldAndGeneration). Note the id
  // conventions: a world id's index1 is 1-based while a shape id's world0
  // is 0-based, hence the -1 when registering.

  final Map<int, bool Function(int, int, int, int)> _filterCallbacks = {};
  final Map<int, bool Function(int, int, int, int, double, double)>
  _preSolveCallbacks = {};

  static int _world0(int worldId) => (worldId & 0xFFFF) - 1;

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
      _filterCallbacks.remove(_world0(worldId));
      _call('f2d_world_set_custom_filter', [worldId, 0]);
    } else {
      _filterCallbacks[_world0(worldId)] = callback;
      _runtime.callbacks.customFilter =
          (a1, aWorldAndGeneration, b1, bWorldAndGeneration) =>
              _filterCallbacks[aWorldAndGeneration & 0xFFFF]?.call(
                a1,
                aWorldAndGeneration,
                b1,
                bWorldAndGeneration,
              ) ??
              true;
      _call('f2d_world_set_custom_filter', [worldId, 1]);
    }
  }

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
      _preSolveCallbacks.remove(_world0(worldId));
      _call('f2d_world_set_pre_solve', [worldId, 0]);
    } else {
      _preSolveCallbacks[_world0(worldId)] = callback;
      _runtime.callbacks.preSolve =
          (
            a1,
            aWorldAndGeneration,
            b1,
            bWorldAndGeneration,
            normalX,
            normalY,
          ) =>
              _preSolveCallbacks[aWorldAndGeneration & 0xFFFF]?.call(
                a1,
                aWorldAndGeneration,
                b1,
                bWorldAndGeneration,
                normalX,
                normalY,
              ) ??
              true;
      _call('f2d_world_set_pre_solve', [worldId, 1]);
    }
  }

  // Debug drawing.

  @override
  void worldDraw(int worldId, RawDebugDraw draw) {
    final previous = _runtime.callbacks.draw;
    _runtime.callbacks.draw = WasmDrawTarget(
      drawPolygon: draw.drawPolygon,
      drawSolidPolygon: draw.drawSolidPolygon,
      drawCircle: draw.drawCircle,
      drawSolidCircle: draw.drawSolidCircle,
      drawSolidCapsule: draw.drawSolidCapsule,
      drawSegment: draw.drawSegment,
      drawTransform: draw.drawTransform,
      drawPoint: draw.drawPoint,
      drawString: draw.drawString,
    );
    try {
      final bounds = draw.drawingBounds;
      _call('f2d_world_draw', [
        worldId,
        _b(bounds != null),
        bounds?.$1 ?? 0,
        bounds?.$2 ?? 0,
        bounds?.$3 ?? 0,
        bounds?.$4 ?? 0,
        _b(draw.drawShapes),
        _b(draw.drawJoints),
        _b(draw.drawJointExtras),
        _b(draw.drawBounds),
        _b(draw.drawMass),
        _b(draw.drawBodyNames),
        _b(draw.drawContacts),
        _b(draw.drawGraphColors),
        _b(draw.drawContactNormals),
        _b(draw.drawContactImpulses),
        _b(draw.drawContactFeatures),
        _b(draw.drawFrictionImpulses),
        _b(draw.drawIslands),
      ]);
    } finally {
      _runtime.callbacks.draw = previous;
    }
  }
}
