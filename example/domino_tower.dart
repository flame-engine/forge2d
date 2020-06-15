library DominoTower;

import 'dart:math' as Math;
import 'package:box2d_flame/box2d.dart';

import 'demo.dart';

class DominoTower extends Demo {
  static const double DOMINO_WIDTH = .2;
  static const double DOMINO_FRICTION = 0.1;
  static const double DOMINO_HEIGHT = 1.0;
  static const int BASE_COUNT = 25;

  /// The density of the dominos under construction. Varies for different parts
  /// of the tower.
  double dominoDensity;

  /// Construct a new DominoTower.
  DominoTower() : super("Domino tower");

  /// Entrypoint.
  static void main() {
    final tower = new DominoTower();
    tower.initialize();
    tower.initializeAnimation();
    tower.runAnimation();
  }

  void makeDomino(double x, double y, bool horizontal) {
    PolygonShape sd = new PolygonShape();
    sd.setAsBoxXY(.5 * DOMINO_WIDTH, .5 * DOMINO_HEIGHT);
    FixtureDef fd = new FixtureDef();
    fd.shape = sd;
    fd.density = dominoDensity;
    BodyDef bd = new BodyDef();
    bd.type = BodyType.DYNAMIC;
    fd.friction = DOMINO_FRICTION;
    fd.restitution = 0.65;
    bd.position = new Vector2(x, y);
    bd.angle = horizontal ? (Math.pi / 2.0) : 0.0;
    Body myBody = world.createBody(bd);
    myBody.createFixtureFromFixtureDef(fd);
    bodies.add(myBody);
  }

  /// Sets up the dominoes.
  void initialize() {
    // Create the floor.
    {
      PolygonShape sd = new PolygonShape();
      sd.setAsBoxXY(50.0, 10.0);

      BodyDef bd = new BodyDef();
      bd.position = new Vector2(0.0, -10.0);
      final body = world.createBody(bd);
      body.createFixtureFromShape(sd);
      bodies.add(body);
    }

    {
      dominoDensity = 10.0;
      // Make bullet
      PolygonShape sd = new PolygonShape();
      sd.setAsBoxXY(.7, .7);
      FixtureDef fd = new FixtureDef();
      fd.density = 35.0;
      BodyDef bd = new BodyDef();
      bd.type = BodyType.DYNAMIC;
      fd.shape = sd;
      fd.friction = 0.0;
      fd.restitution = 0.85;
      bd.bullet = true;
      bd.position = new Vector2(30.0, 5.00);
      Body b = world.createBody(bd);
      bodies.add(b);
      b.createFixtureFromFixtureDef(fd);
      b.linearVelocity = new Vector2(-25.0, -25.0);
      b.angularVelocity = 6.7;

      fd.density = 25.0;
      bd.position = new Vector2(-30.0, 25.0);
      b = world.createBody(bd);
      bodies.add(b);
      b.createFixtureFromFixtureDef(fd);
      b.linearVelocity = new Vector2(35.0, -10.0);
      b.angularVelocity = -8.3;
    }

    {
      double currX;
      // Make base
      for (int i = 0; i < BASE_COUNT; ++i) {
        currX =
            i * 1.5 * DOMINO_HEIGHT - (1.5 * DOMINO_HEIGHT * BASE_COUNT / 2);
        makeDomino(currX, DOMINO_HEIGHT / 2.0, false);
        makeDomino(currX, DOMINO_HEIGHT + DOMINO_WIDTH / 2.0, true);
      }
      currX = BASE_COUNT * 1.5 * DOMINO_HEIGHT -
          (1.5 * DOMINO_HEIGHT * BASE_COUNT / 2);

      // Make 'I's
      for (int j = 1; j < BASE_COUNT; ++j) {
        if (j > 3) dominoDensity *= .8;

        // The y at the center of the I structure.
        double currY =
            DOMINO_HEIGHT * .5 + (DOMINO_HEIGHT + 2 * DOMINO_WIDTH) * .99 * j;

        for (int i = 0; i < BASE_COUNT - j; ++i) {
          currX = i * 1.5 * DOMINO_HEIGHT -
              (1.5 * DOMINO_HEIGHT * (BASE_COUNT - j) / 2);
          dominoDensity *= 2.5;
          if (i == 0) {
            makeDomino(currX - (1.25 * DOMINO_HEIGHT) + .5 * DOMINO_WIDTH,
                currY - DOMINO_WIDTH, false);
          }
          if (i == BASE_COUNT - j - 1) {
            makeDomino(currX + (1.25 * DOMINO_HEIGHT) - .5 * DOMINO_WIDTH,
                currY - DOMINO_WIDTH, false);
          }

          dominoDensity /= 2.5;
          makeDomino(currX, currY, false);
          makeDomino(currX, currY + .5 * (DOMINO_WIDTH + DOMINO_HEIGHT), true);
          makeDomino(currX, currY - .5 * (DOMINO_WIDTH + DOMINO_HEIGHT), true);
        }
      }
    }
  }
}

void main() {
  DominoTower.main();
}
