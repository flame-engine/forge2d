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
  void worldSetGravity(int worldId, double x, double y) {
    final gravity = Struct.create<b2.b2Vec2>()
      ..x = x
      ..y = y;
    b2.b2World_SetGravity(_world(worldId), gravity);
  }

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
}
