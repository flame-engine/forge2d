import 'dart:math';

import 'package:forge2d/forge2d.dart';

import 'demo.dart';

class DominoTower extends Demo {
  static const double dominoWidth = .2;
  static const double dominoFriction = 0.1;
  static const double dominoHeight = 1.0;
  static const int baseCount = 25;

  /// The density of the dominos under construction. Varies for different parts
  /// of the tower.
  late double dominoDensity;

  /// Construct a DominoTower.
  DominoTower() : super('Domino tower');

  void makeDomino(double x, double y, {required bool horizontal}) {
    final shape = PolygonShape()
      ..setAsBoxXY(.5 * dominoWidth, .5 * dominoHeight);
    final fixtureDef = FixtureDef(
      shape,
      density: dominoDensity,
      friction: dominoFriction,
      restitution: 0.65,
    );
    final bodyDef = BodyDef(
      type: BodyType.dynamic,
      position: Vector2(x, y),
      angle: horizontal ? (pi / 2.0) : 0.0,
    );
    final body = world.createBody(bodyDef);
    body.createFixture(fixtureDef);
    bodies.add(body);
  }

  /// Sets up the dominoes.
  @override
  void initialize() {
    // Create the floor.
    {
      final sd = PolygonShape();
      sd.setAsBoxXY(50.0, 10.0);

      final bd = BodyDef(position: Vector2(0.0, -10.0));
      final body = world.createBody(bd);
      body.createFixtureFromShape(sd);
      bodies.add(body);
    }

    {
      dominoDensity = 10.0;
      // Make bullet
      final shape = PolygonShape()..setAsBoxXY(.7, .7);
      final fixtureDef = FixtureDef(shape, density: 35.0, restitution: 0.85);
      final bodyDef = BodyDef(
        type: BodyType.dynamic,
        bullet: true,
        position: Vector2(30.0, 5.00),
      );
      var body = world.createBody(bodyDef);
      bodies.add(body);
      body.createFixture(fixtureDef);
      body.linearVelocity = Vector2(-25.0, -25.0);
      body.angularVelocity = 6.7;

      fixtureDef.density = 25.0;
      bodyDef.position = Vector2(-30.0, 25.0);
      body = world.createBody(bodyDef);
      bodies.add(body);
      body.createFixture(fixtureDef);
      body.linearVelocity = Vector2(35.0, -10.0);
      body.angularVelocity = -8.3;
    }

    {
      double currX;
      // Make base
      for (var i = 0; i < baseCount; ++i) {
        currX = i * 1.5 * dominoHeight - (1.5 * dominoHeight * baseCount / 2);
        makeDomino(currX, dominoHeight / 2.0, horizontal: false);
        makeDomino(currX, dominoHeight + dominoWidth / 2.0, horizontal: true);
      }
      currX =
          baseCount * 1.5 * dominoHeight - (1.5 * dominoHeight * baseCount / 2);

      // Make 'I's
      for (var j = 1; j < baseCount; ++j) {
        if (j > 3) {
          dominoDensity *= .8;
        }

        // The y at the center of the I structure.
        final currY =
            dominoHeight * .5 + (dominoHeight + 2 * dominoWidth) * .99 * j;

        for (var i = 0; i < baseCount - j; ++i) {
          currX =
              i * 1.5 * dominoHeight -
              (1.5 * dominoHeight * (baseCount - j) / 2);
          dominoDensity *= 2.5;
          if (i == 0) {
            makeDomino(
              currX - (1.25 * dominoHeight) + .5 * dominoWidth,
              currY - dominoWidth,
              horizontal: false,
            );
          }
          if (i == baseCount - j - 1) {
            makeDomino(
              currX + (1.25 * dominoHeight) - .5 * dominoWidth,
              currY - dominoWidth,
              horizontal: false,
            );
          }

          dominoDensity /= 2.5;
          makeDomino(currX, currY, horizontal: false);
          makeDomino(
            currX,
            currY + .5 * (dominoWidth + dominoHeight),
            horizontal: true,
          );
          makeDomino(
            currX,
            currY - .5 * (dominoWidth + dominoHeight),
            horizontal: true,
          );
        }
      }
    }
  }
}

void main() {
  final tower = DominoTower();
  tower.initialize();
  tower.initializeAnimation();
  tower.runAnimation();
}
