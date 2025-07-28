import 'package:forge2d/forge2d.dart';

import 'demo.dart';

/// Demonstration of dominoes being knocked over.
class DominoTest extends Demo {
  DominoTest() : super('Domino test');

  @override
  void initialize() {
    {
      // Floor
      final shape = PolygonShape()..setAsBoxXY(50.0, 10.0);
      final fixtureDef = FixtureDef(shape, friction: 0.1);

      final bd = BodyDef(position: Vector2(0.0, -10.0));
      final body = world.createBody(bd);
      body.createFixture(fixtureDef);
      bodies.add(body);
    }

    {
      // Platforms
      for (var i = 0; i < 4; i++) {
        final shape = PolygonShape()..setAsBoxXY(15.0, 0.125);
        final fixtureDef = FixtureDef(shape, friction: 0.1);

        final bodyDef = BodyDef(position: Vector2(0.0, 5.0 + 5 * i));
        final body = world.createBody(bodyDef);
        body.createFixture(fixtureDef);
        bodies.add(body);
      }
    }

    // Dominoes
    {
      final shape = PolygonShape()..setAsBoxXY(0.125, 2.0);
      final fixtureDef = FixtureDef(shape, density: 25.0, friction: 0.5);
      final bodyDef = BodyDef(type: BodyType.dynamic);

      const numPerRow = 25;

      for (var i = 0; i < 4; ++i) {
        for (var j = 0; j < numPerRow; j++) {
          bodyDef.position = Vector2(
            -14.75 + j * (29.5 / (numPerRow - 1)),
            7.3 + 5 * i,
          );
          if (i == 2 && j == 0) {
            bodyDef.angle = -.1;
            bodyDef.position.x += .1;
          } else if (i == 3 && j == numPerRow - 1) {
            bodyDef.angle = .1;
            bodyDef.position.x -= .1;
          } else {
            bodyDef.angle = 0.0;
          }
          final body = world.createBody(bodyDef);
          body.createFixture(fixtureDef);
          bodies.add(body);
        }
      }
    }
  }
}

void main() {
  final domino = DominoTest();
  domino.initialize();
  domino.initializeAnimation();
  domino.runAnimation();
}
