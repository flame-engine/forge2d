import 'dart:math';

import 'package:forge2d/forge2d.dart';

import 'demo.dart';

class CircleStress extends Demo {
  /// The number of columns of balls in the pen.
  static const int columns = 8;

  /// This number of balls will be created on each layer.
  static const int loadSize = 20;

  /// Construct a new Circle Stress Demo.
  CircleStress() : super('Circle stress');

  /// Creates all bodies.
  @override
  void initialize() {
    {
      final bd = BodyDef();
      final ground = world.createBody(bd);
      bodies.add(ground);

      final shape = PolygonShape();
      shape.setAsEdge(Vector2(-40.0, 0.0), Vector2(40.0, 0.0));
      ground.createFixtureFromShape(shape);
    }

    {
      // Ground
      final shape = PolygonShape();
      shape.setAsBoxXY(50.0, 10.0);
      final bodyDef = BodyDef()
        ..type = BodyType.static
        ..position = Vector2(0.0, -10.0);
      final body = world.createBody(bodyDef);
      bodies.add(body);
      final fixtureDef = FixtureDef(shape)..friction = 1.0;
      body.createFixture(fixtureDef);

      // Walls
      shape.setAsBoxXY(3.0, 50.0);
      final wallDef = BodyDef()..position = Vector2(45.0, 25.0);
      final rightWall = world.createBody(wallDef);
      bodies.add(rightWall);
      rightWall.createFixtureFromShape(shape);
      wallDef.position = Vector2(-45.0, 25.0);
      final leftWall = world.createBody(wallDef);
      bodies.add(leftWall);
      leftWall.createFixtureFromShape(shape);

      // Corners
      final cornerDef = BodyDef();
      shape.setAsBoxXY(20.0, 3.0);
      cornerDef.angle = -pi / 4.0;
      cornerDef.position = Vector2(-35.0, 8.0);
      var myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(shape);
      cornerDef.angle = pi / 4.0;
      cornerDef.position = Vector2(35.0, 8.0);
      myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(shape);

      // top
      shape.setAsBoxXY(50.0, 10.0);
      final topDef = BodyDef()
        ..type = BodyType.static
        ..angle = 0.0
        ..position = Vector2(0.0, 75.0);
      final topBody = world.createBody(topDef);
      bodies.add(topBody);
      fixtureDef.shape = shape;
      fixtureDef.friction = 1.0;
      topBody.createFixture(fixtureDef);
    }

    {
      final shape = ChainShape();
      final vertices = List<Vector2>.generate(
        20,
        (i) => Vector2(i.toDouble(), i.toDouble() * i / 20),
      );
      shape.createChain(vertices);

      final fixtureDef = FixtureDef(shape)
        ..restitution = 0.0
        ..friction = 0.1;

      final bodyDef = BodyDef()
        ..position = Vector2.zero()
        ..type = BodyType.static;

      final body = world.createBody(bodyDef)..createFixture(fixtureDef);
      bodies.add(body);
    }

    {
      final bd = BodyDef()
        ..type = BodyType.dynamic
        ..position = Vector2(0.0, 10.0);
      const numPieces = 5;
      const radius = 6.0;
      final body = world.createBody(bd);
      bodies.add(body);

      for (var i = 0; i < numPieces; i++) {
        final xPos = radius * cos(2 * pi * (i / numPieces.toDouble()));
        final yPos = radius * sin(2 * pi * (i / numPieces.toDouble()));

        final shape = CircleShape()
          ..radius = 1.2
          ..position.setValues(xPos, yPos);

        final fixtureDef = FixtureDef(shape)
          ..density = 25.0
          ..friction = .1
          ..restitution = .9;

        body.createFixture(fixtureDef);
      }

      body.setBullet(false);

      // Create an empty ground body.
      final bodyDef = BodyDef();
      final groundBody = world.createBody(bodyDef);

      final rjd = RevoluteJointDef()
        ..initialize(body, groundBody, body.position)
        ..motorSpeed = -pi
        ..maxMotorTorque = 1000000.0
        ..enableMotor = true;

      world.createJoint(rjd);

      for (var j = 0; j < columns; j++) {
        for (var i = 0; i < loadSize; i++) {
          final shape = CircleShape()
            ..radius = 1.0 + (i.isEven ? 1.0 : -1.0) * .5 * .75;
          final fd2 = FixtureDef(shape)
            ..density = shape.radius * 1.5
            ..friction = 0.5
            ..restitution = 0.7;
          final xPos = -39.0 + 2 * i;
          final yPos = 50.0 + j;
          final bodyDef = BodyDef()
            ..type = BodyType.dynamic
            ..position = Vector2(xPos, yPos);
          final body = world.createBody(bodyDef);
          bodies.add(body);
          body.createFixture(fd2);
        }
      }
    }
  }
}

void main() {
  CircleStress()
    ..initialize()
    ..initializeAnimation()
    ..viewport.scale = 4.0
    ..runAnimation();
}
