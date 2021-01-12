library particles;

import 'package:forge2d/forge2d.dart';

import 'demo.dart';

class Particles extends Demo {
  /// Starting position of ball cage in the world.
  static const double START_X = -20.0;
  static const double START_Y = -20.0;

  /// The radius of the balls forming the arena.
  static const double WALL_BALL_RADIUS = 2.0;

  /// Radius of the active ball.
  static const double ACTIVE_BALL_RADIUS = 1.0;

  /// Constructs a new Particles example.
  Particles() : super("Particles");

  static void main() {
    Particles()
      ..initialize()
      ..initializeAnimation()
      ..runAnimation();
  }

  @override
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

    const int maxShapeInRow = 10;
    final double borderLimitX =
        START_X + maxShapeInRow * 2 * circleShape.radius;
    final double borderLimitY =
        START_Y + maxShapeInRow * 2 * circleShape.radius;

    for (int i = 0; i < maxShapeInRow; i++) {
      final double shiftX = START_X + circleShape.radius * 2 * i;
      final double shiftY = START_Y + circleShape.radius * 2 * i;

      circleBodyDef.position = Vector2(shiftX, START_Y);
      Body circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixture(circleFixtureDef);

      circleBodyDef.position = Vector2(shiftX, borderLimitY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixture(circleFixtureDef);

      circleBodyDef.position = Vector2(START_X, shiftY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixture(circleFixtureDef);

      circleBodyDef.position = Vector2(borderLimitX, shiftY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixture(circleFixtureDef);
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
    final activeBodyDef = BodyDef();
    activeBodyDef.linearVelocity = Vector2(0.0, -20.0);
    activeBodyDef.position = Vector2(15.0, 15.0);
    activeBodyDef.type = BodyType.DYNAMIC;
    final activeBody = world.createBody(activeBodyDef);
    bodies.add(activeBody);
    activeBody.createFixture(activeFixtureDef);

    // Create particles
    world.particleSystem.setParticleRadius(0.35);
    world.particleSystem.setParticleDamping(0.2);

    final shape = CircleShape()..radius = 5;
    final particleGroup = ParticleGroupDef()
      ..position.setFrom(world.center)
      ..destroyAutomatically = true
      ..color = Color3i.black
      ..flags = ParticleType.waterParticle
      ..shape = shape;
    world.particleSystem.createParticleGroup(particleGroup);
  }
}

void main() {
  Particles.main();
}
