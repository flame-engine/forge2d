@TestOn('vm')
library;

import 'dart:isolate';

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  test(
    'isolates can create, step and destroy worlds at the same time',
    () async {
      const isolates = 8;
      const worldsPerIsolate = 500;
      final expected = _simulate();

      final results = await Future.wait([
        for (var i = 0; i < isolates; i++)
          Isolate.run(() async {
            await initializeForge2D();
            return [for (var j = 0; j < worldsPerIsolate; j++) _simulate()];
          }),
      ]);

      expect(results, everyElement(everyElement(expected)));
    },
  );

  test(
    'a world in one isolate freezes the length unit in the others',
    () async {
      final world = World();
      addTearDown(world.destroy);

      final error = await Isolate.run(() async {
        try {
          await initializeForge2D(lengthUnitsPerMeter: 2);
          return null;
        } on StateError catch (error) {
          return error.message;
        }
      });

      expect(error, contains('a World has already been created'));
      expect(Tolerances.lengthUnitsPerMeter, 1);
    },
  );
}

/// Drops a short stack of boxes onto the ground and returns where they end
/// up. Box2D is deterministic, so every run gives exactly the same numbers.
List<double> _simulate() {
  final world = World();
  world
      .createBody(BodyDef(position: Vector2(0, -1)))
      .createShape(Polygon.box(20, 1));
  final boxes = [
    for (var i = 0; i < 3; i++)
      world.createBody(
        BodyDef(
          type: BodyType.dynamic,
          position: Vector2(0.1 * i, 1 + 1.1 * i),
        ),
      )..createShape(Polygon.square(0.5)),
  ];
  for (var i = 0; i < 30; i++) {
    world.step(1 / 60);
  }
  final result = [
    for (final box in boxes) ...[box.position.x, box.position.y, box.angle],
  ];
  world.destroy();
  return result;
}
