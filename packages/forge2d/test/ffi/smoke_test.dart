@TestOn('vm')
library;

import 'dart:ffi';

import 'package:ffi/ffi.dart';
import 'package:forge2d/src/ffi/box2d.g.dart' as b2;
import 'package:test/test.dart';

/// Exercises the native build hook and the raw generated bindings end-to-end:
/// a dynamic box falls onto static ground and comes to rest on top of it.
void main() {
  test('a falling box lands on the ground', () {
    using((arena) {
      final worldDef = arena<b2.b2WorldDef>()..ref = b2.b2DefaultWorldDef();
      final worldId = b2.b2CreateWorld(worldDef);
      expect(b2.b2World_IsValid(worldId), isTrue);

      final groundDef = arena<b2.b2BodyDef>()..ref = b2.b2DefaultBodyDef();
      groundDef.ref.position.y = -1.0;
      final groundId = b2.b2CreateBody(worldId, groundDef);
      final groundBox = arena<b2.b2Polygon>()..ref = b2.b2MakeBox(50.0, 1.0);
      final groundShapeDef = arena<b2.b2ShapeDef>()
        ..ref = b2.b2DefaultShapeDef();
      b2.b2CreatePolygonShape(groundId, groundShapeDef, groundBox);

      final bodyDef = arena<b2.b2BodyDef>()..ref = b2.b2DefaultBodyDef();
      bodyDef.ref.typeAsInt = b2.b2BodyType.b2_dynamicBody.value;
      bodyDef.ref.position.y = 10.0;
      final bodyId = b2.b2CreateBody(worldId, bodyDef);
      final box = arena<b2.b2Polygon>()..ref = b2.b2MakeBox(0.5, 0.5);
      final shapeDef = arena<b2.b2ShapeDef>()..ref = b2.b2DefaultShapeDef();
      b2.b2CreatePolygonShape(bodyId, shapeDef, box);

      // Falling from y = 10 takes ~85 steps; the sleep timer needs another
      // 0.5 s (30 steps) of rest on top of that.
      for (var i = 0; i < 200; i++) {
        b2.b2World_Step(worldId, 1 / 60, 4);
      }

      final position = b2.b2Body_GetPosition(bodyId);
      // The ground's top surface is at y = 0, so a 0.5 half-extent box rests
      // with its center at y = 0.5.
      expect(position.x, closeTo(0.0, 0.01));
      expect(position.y, closeTo(0.5, 0.01));
      expect(b2.b2Body_IsAwake(bodyId), isFalse);

      b2.b2DestroyWorld(worldId);
      expect(b2.b2World_IsValid(worldId), isFalse);
    });
  });
}
