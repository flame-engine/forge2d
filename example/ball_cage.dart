library BallCage;

import 'package:box2d_flame/box2d.dart';

import 'demo.dart';

class BallCage extends Demo {
  /// Starting position of ball cage in the world.
  static const double START_X = -20.0;
  static const double START_Y = -20.0;

  /// The radius of the balls forming the arena.
  static const double WALL_BALL_RADIUS = 2.0;

  /// Radius of the active ball.
  static const double ACTIVE_BALL_RADIUS = 1.0;

  /// Constructs a new BallCage.
  BallCage() : super("Ball cage");

  /// Entrypoint.
  static void main() {
    final cage = BallCage();
    cage.initialize();
    cage.initializeAnimation();
    cage.runAnimation();
  }

  void initialize() {
    // Define the circle shape.
    final circleShape = CircleShape();
    circleShape.radius = WALL_BALL_RADIUS;

    // Create fixture using the circle shape.
    final circleFixtureDef = FixtureDef();
    circleFixtureDef.shape = circleShape;
    circleFixtureDef.friction = .9;
    circleFixtureDef.restitution = 1.0;

    // Create a body def.
    final circleBodyDef = BodyDef();

    int maxShapeinRow = 10;
    final double borderLimitX =
        START_X + maxShapeinRow * 2 * circleShape.radius;
    final double borderLimitY =
        START_Y + maxShapeinRow * 2 * circleShape.radius;

    for (int i = 0; i < maxShapeinRow; i++) {
      final double shiftX = START_X + circleShape.radius * 2 * i;
      final double shiftY = START_Y + circleShape.radius * 2 * i;

      circleBodyDef.position = Vector2(shiftX, START_Y);
      Body circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixtureFromFixtureDef(circleFixtureDef);

      circleBodyDef.position = Vector2(shiftX, borderLimitY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixtureFromFixtureDef(circleFixtureDef);

      circleBodyDef.position = Vector2(START_X, shiftY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixtureFromFixtureDef(circleFixtureDef);

      circleBodyDef.position = Vector2(borderLimitX, shiftY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixtureFromFixtureDef(circleFixtureDef);
    }

    // Create a bouncing ball.
    final bouncingCircle = CircleShape();
    bouncingCircle.radius = ACTIVE_BALL_RADIUS;

    // Create fixture for that ball shape.
    final activeFixtureDef = FixtureDef();
    activeFixtureDef.restitution = 1.0;
    activeFixtureDef.density = 0.05;
    activeFixtureDef.shape = bouncingCircle;

    // Create the active ball body.
    final activeBodyDef = new BodyDef();
    activeBodyDef.linearVelocity = new Vector2(0.0, -20.0);
    activeBodyDef.position = new Vector2(15.0, 15.0);
    activeBodyDef.type = BodyType.DYNAMIC;
    activeBodyDef.bullet = true;
    final activeBody = world.createBody(activeBodyDef);
    bodies.add(activeBody);
    activeBody.createFixtureFromFixtureDef(activeFixtureDef);
  }
}

void main() {
  BallCage.main();
}
