@TestOn('vm')
library;

import 'dart:isolate';

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

/// Box2D keeps every world in one process-wide table, and every isolate in
/// the process shares the one copy of the native library. Creating and
/// destroying worlds therefore has to be safe from several isolates at once,
/// as long as each world is only used by the isolate that created it.
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
            for (var j = 0; j < worldsPerIsolate; j++) {
              final result = _simulate();
              if (!_sameResult(result, expected)) {
                return 'world $j gave $result';
              }
            }
            return null;
          }),
      ]);

      expect(results, everyElement(isNull));
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

bool _sameResult(List<double> a, List<double> b) {
  if (a.length != b.length) {
    return false;
  }
  for (var i = 0; i < a.length; i++) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}
