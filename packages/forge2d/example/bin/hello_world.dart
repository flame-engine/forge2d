// A minimal forge2d simulation: a box falling onto the ground, printed as a
// height graph. Run with: dart run bin/hello_world.dart
// ignore_for_file: avoid_print
import 'package:forge2d/forge2d.dart';

Future<void> main() async {
  await initializeForge2D();

  final world = World();

  // The ground: a static body with a wide, flat box, its top surface at y=0.
  world
      .createBody(BodyDef(position: Vector2(0, -1)))
      .createShape(Polygon.box(50, 1));

  // A dynamic 1x1 box dropped from y=10.
  final box = world.createBody(
    BodyDef(type: BodyType.dynamic, position: Vector2(0, 10)),
  )..createShape(Polygon.square(0.5));

  for (var frame = 0; frame < 90; frame++) {
    world.step(1 / 60);
    if (frame % 5 == 0) {
      final height = box.position.y;
      final bar = '#' * (height * 4).round().clamp(0, 60);
      print('${height.toStringAsFixed(2).padLeft(6)} $bar');
    }
  }

  print('The box came to rest at y = ${box.position.y.toStringAsFixed(3)}');
  world.destroy();
}
