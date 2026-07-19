@TestOn('vm')
library;

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  group('World', () {
    test('starts valid and becomes invalid on destroy', () {
      final world = World();
      expect(world.isValid, isTrue);
      world.destroy();
      expect(world.isValid, isFalse);
    });

    test('has the native default gravity', () {
      final world = World();
      addTearDown(world.destroy);
      expect(world.gravity, Vector2(0, -10));
    });

    test('accepts a gravity override', () {
      final world = World(gravity: Vector2(3, -1.5));
      addTearDown(world.destroy);
      expect(world.gravity, Vector2(3, -1.5));
    });

    test('gravity can be changed', () {
      final world = World();
      addTearDown(world.destroy);
      world.gravity = Vector2(0, -3.5);
      expect(world.gravity, Vector2(0, -3.5));
    });

    test('def toggles are applied', () {
      final world = World(
        def: WorldDef(enableSleep: false, enableContinuous: false),
      );
      addTearDown(world.destroy);
      expect(world.sleepingEnabled, isFalse);
      expect(world.continuousEnabled, isFalse);

      world.sleepingEnabled = true;
      world.continuousEnabled = true;
      expect(world.sleepingEnabled, isTrue);
      expect(world.continuousEnabled, isTrue);
    });

    test('destroying a world destroys its bodies', () {
      final world = World();
      final body = world.createBody(BodyDef(userData: 'payload'));
      expect(body.isValid, isTrue);
      world.destroy();
      expect(body.isValid, isFalse);
    });

    test('a falling box lands on a static ground box', () {
      final world = World();
      addTearDown(world.destroy);

      world
          .createBody(BodyDef(position: Vector2(0, -1)))
          .createShape(Polygon.box(50, 1));
      final box = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(0, 10)),
      )..createShape(Polygon.square(0.5));

      for (var i = 0; i < 200; i++) {
        world.step(1 / 60);
      }

      expect(box.position.x, closeTo(0, 0.01));
      expect(box.position.y, closeTo(0.5, 0.01));
      expect(box.isAwake, isFalse);
    });
  });
}
