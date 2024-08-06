import 'package:forge2d/forge2d.dart';

import 'demo.dart';

class Particles extends Demo {
  /// Starting position of ball cage in the world.
  static const double startX = -20.0;
  static const double startY = -20.0;

  /// The radius of the balls forming the arena.
  static const double wallBallRadius = 2.0;

  /// Radius of the active ball.
  static const double activeBallRadius = 1.0;

  /// Constructs a new Particles example.
  Particles() : super('Particles');
  @override
  void initialize() {
    // Define the circle shape.
    final circleShape = CircleShape(radius: wallBallRadius);

    // Create fixture using the circle shape.
    final circleFixtureDef = FixtureDef(
      circleShape,
      friction: 0.9,
      restitution: 1.0,
    );

    // Create a body def.
    final circleBodyDef = BodyDef();

    const maxShapeInRow = 10;
    final borderLimitX = startX + maxShapeInRow * 2 * circleShape.radius;
    final borderLimitY = startY + maxShapeInRow * 2 * circleShape.radius;

    for (var i = 0; i < maxShapeInRow; i++) {
      final shiftX = startX + circleShape.radius * 2 * i;
      final shiftY = startY + circleShape.radius * 2 * i;

      circleBodyDef.position = Vector2(shiftX, startY);
      var circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixture(circleFixtureDef);

      circleBodyDef.position = Vector2(shiftX, borderLimitY);
      circleBody = world.createBody(circleBodyDef);
      bodies.add(circleBody);
      circleBody.createFixture(circleFixtureDef);

      circleBodyDef.position = Vector2(startX, shiftY);
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
    bouncingCircle.radius = activeBallRadius;

    // Create fixture for that ball shape.
    final activeFixtureDef = FixtureDef(
      bouncingCircle,
      restitution: 1.0,
      density: 0.05,
    );

    // Create the active ball body.
    final activeBodyDef = BodyDef();
    activeBodyDef.linearVelocity = Vector2(0.0, -20.0);
    activeBodyDef.position = Vector2(0.0, -15.0);
    activeBodyDef.type = BodyType.dynamic;
    final activeBody = world.createBody(activeBodyDef);
    bodies.add(activeBody);
    final fixture = activeBody.createFixture(activeFixtureDef);

    // Create particles
    world.particleSystem.particleRadius = 0.35;
    world.particleSystem.dampingStrength = 0.2;

    final shape = CircleShape(radius: 5);
    final particleGroup = ParticleGroupDef()
      ..position.setFrom(fixture.renderCenter)
      ..destroyAutomatically = true
      ..color = Color3i.blue()
      ..flags = ParticleType.waterParticle
      ..shape = shape;
    world.particleSystem.createParticleGroup(particleGroup);
  }
}

void main() {
  Particles()
    ..initialize()
    ..initializeAnimation()
    ..runAnimation();
}
