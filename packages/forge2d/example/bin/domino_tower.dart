// A port of the domino tower demo: a pyramid of dominoes toppled by a heavy
// ball, with contact events counting the collisions.
// Run with: dart run bin/domino_tower.dart
// ignore_for_file: avoid_print
import 'package:forge2d/forge2d.dart';

Future<void> main() async {
  await initializeForge2D();

  final world = World();

  world
      .createBody(BodyDef(position: Vector2(0, -1)))
      .createShape(Polygon.box(50, 1));

  // A pyramid of thin dominoes.
  const rows = 10;
  const dominoWidth = 0.1;
  const dominoHeight = 0.9;
  var dominoes = 0;
  for (var row = 0; row < rows; row++) {
    final count = rows - row;
    final y = dominoHeight / 2 + row * (dominoHeight + 0.05);
    for (var i = 0; i < count; i++) {
      final x = (i - (count - 1) / 2) * (dominoHeight * 1.2);
      world
          .createBody(BodyDef(type: BodyType.dynamic, position: Vector2(x, y)))
          .createShape(
            Polygon.box(dominoWidth / 2, dominoHeight / 2),
            ShapeDef(enableContactEvents: true),
          );
      dominoes++;
    }
  }

  // A heavy ball fired at the tower.
  world
      .createBody(
        BodyDef(
          type: BodyType.dynamic,
          position: Vector2(-20, 4),
          linearVelocity: Vector2(30, 0),
          isBullet: true,
        ),
      )
      .createShape(
        Circle(radius: 0.8),
        ShapeDef(density: 10, enableContactEvents: true),
      );

  var contacts = 0;
  for (var i = 0; i < 300; i++) {
    world.step(1 / 60);
    contacts += world.contactEvents.begin.length;
  }

  final sleeping = world.bodyMoveEvents;
  print('$dominoes dominoes, $contacts contacts begun in 5 simulated seconds');
  print('Simulation settled: ${sleeping.isEmpty ? 'yes' : 'still moving'}');
  world.destroy();
}
