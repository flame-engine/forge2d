@TestOn('vm')
library;

import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:test/test.dart';

/// Exercises the backend seam directly, with parameters taken from the
/// default [WorldDef], until the public World API lands.
void main() {
  late int worldId;

  int createDefaultWorld() {
    final def = WorldDef();
    return rawBox2D.createWorld(
      gravityX: def.gravity.x,
      gravityY: def.gravity.y,
      restitutionThreshold: def.restitutionThreshold,
      hitEventThreshold: def.hitEventThreshold,
      contactHertz: def.contactHertz,
      contactDampingRatio: def.contactDampingRatio,
      maxContactPushSpeed: def.maxContactPushSpeed,
      maximumLinearSpeed: def.maximumLinearSpeed,
      enableSleep: def.enableSleep,
      enableContinuous: def.enableContinuous,
    );
  }

  setUpAll(initializeForge2D);
  setUp(() => worldId = createDefaultWorld());
  tearDown(() {
    if (rawBox2D.worldIsValid(worldId)) {
      rawBox2D.destroyWorld(worldId);
    }
  });

  test('a created world is valid until destroyed', () {
    expect(rawBox2D.worldIsValid(worldId), isTrue);
    rawBox2D.destroyWorld(worldId);
    expect(rawBox2D.worldIsValid(worldId), isFalse);
  });

  test('world ids are not reused across generations', () {
    rawBox2D.destroyWorld(worldId);
    final second = createDefaultWorld();
    expect(second, isNot(worldId));
    expect(rawBox2D.worldIsValid(worldId), isFalse);
    expect(rawBox2D.worldIsValid(second), isTrue);
    rawBox2D.destroyWorld(second);
  });

  test('gravity defaults to (0, -10) and can be changed', () {
    expect(rawBox2D.worldGetGravity(worldId), (0.0, -10.0));
    rawBox2D.worldSetGravity(worldId, 1.5, 3);
    expect(rawBox2D.worldGetGravity(worldId), (1.5, 3.0));
  });

  test('sleeping and continuous toggles round-trip', () {
    expect(rawBox2D.worldIsSleepingEnabled(worldId), isTrue);
    rawBox2D.worldEnableSleeping(worldId, enabled: false);
    expect(rawBox2D.worldIsSleepingEnabled(worldId), isFalse);

    expect(rawBox2D.worldIsContinuousEnabled(worldId), isTrue);
    rawBox2D.worldEnableContinuous(worldId, enabled: false);
    expect(rawBox2D.worldIsContinuousEnabled(worldId), isFalse);
  });

  test('stepping an empty world is safe', () {
    for (var i = 0; i < 10; i++) {
      rawBox2D.worldStep(worldId, 1 / 60, 4);
    }
    expect(rawBox2D.worldIsValid(worldId), isTrue);
  });
}
