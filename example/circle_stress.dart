library circle_stress;

import 'dart:math' as math;
import 'demo.dart';

import 'package:forge2d/forge2d.dart';

/// Scale of the viewport for this Demo.
const double _MY_VIEWPORT_SCALE = 4.0;

class CircleStress extends Demo {
  /// The number of columns of balls in the pen.
  static const int COLUMNS = 8;

  /// This number of balls will be created on each layer.
  static const int LOAD_SIZE = 20;

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
      final sd = PolygonShape();
      sd.setAsBoxXY(50.0, 10.0);
      final bd = BodyDef();
      bd.type = BodyType.STATIC;
      bd.position = Vector2(0.0, -10.0);
      final b = world.createBody(bd);
      bodies.add(b);
      final fd = FixtureDef();
      fd.shape = sd;
      fd.friction = 1.0;
      b.createFixture(fd);

      // Walls
      sd.setAsBoxXY(3.0, 50.0);
      final wallDef = BodyDef();
      wallDef.position = Vector2(45.0, 25.0);
      final rightWall = world.createBody(wallDef);
      bodies.add(rightWall);
      rightWall.createFixtureFromShape(sd);
      wallDef.position = Vector2(-45.0, 25.0);
      final leftWall = world.createBody(wallDef);
      bodies.add(leftWall);
      leftWall.createFixtureFromShape(sd);

      // Corners
      final cornerDef = BodyDef();
      sd.setAsBoxXY(20.0, 3.0);
      cornerDef.angle = -math.pi / 4.0;
      cornerDef.position = Vector2(-35.0, 8.0);
      var myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(sd);
      cornerDef.angle = math.pi / 4.0;
      cornerDef.position = Vector2(35.0, 8.0);
      myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(sd);

      // top
      sd.setAsBoxXY(50.0, 10.0);
      final topDef = BodyDef()
        ..type = BodyType.STATIC
        ..angle = 0.0
        ..position = Vector2(0.0, 75.0);
      final topBody = world.createBody(topDef);
      bodies.add(topBody);
      fd.shape = sd;
      fd.friction = 1.0;
      topBody.createFixture(fd);
    }

    {
      final shape = ChainShape();
      final vertices = List<Vector2>.generate(
          20, (i) => Vector2(i.toDouble(), i.toDouble() * i / 20));
      shape.createChain(vertices);

      final fixtureDef = FixtureDef()
        ..shape = shape
        ..restitution = 0.0
        ..friction = 0.1;

      final bodyDef = BodyDef()
        ..position = Vector2.zero()
        ..type = BodyType.STATIC;

      final body = world.createBody(bodyDef)..createFixture(fixtureDef);
      bodies.add(body);
    }

    {
      final bd = BodyDef()
        ..type = BodyType.DYNAMIC
        ..position = Vector2(0.0, 10.0);
      const numPieces = 5;
      const radius = 6.0;
      final body = world.createBody(bd);
      bodies.add(body);

      for (var i = 0; i < numPieces; i++) {
        final xPos =
            radius * math.cos(2 * math.pi * (i / numPieces.toDouble()));
        final yPos =
            radius * math.sin(2 * math.pi * (i / numPieces.toDouble()));

        final cd = CircleShape()
          ..radius = 1.2
          ..position.setValues(xPos, yPos);

        final fd = FixtureDef()
          ..shape = cd
          ..density = 25.0
          ..friction = .1
          ..restitution = .9;

        body.createFixture(fd);
      }

      body.setBullet(false);

      // Create an empty ground body.
      final bodyDef = BodyDef();
      final groundBody = world.createBody(bodyDef);

      final rjd = RevoluteJointDef()
        ..initialize(body, groundBody, body.position)
        ..motorSpeed = -math.pi
        ..maxMotorTorque = 1000000.0
        ..enableMotor = true;

      world.createJoint(rjd);

      for (var j = 0; j < COLUMNS; j++) {
        for (var i = 0; i < LOAD_SIZE; i++) {
          final circleShape = CircleShape()
            ..radius = 1.0 + (i % 2 == 0 ? 1.0 : -1.0) * .5 * .75;
          final fd2 = FixtureDef()
            ..shape = circleShape
            ..density = circleShape.radius * 1.5
            ..friction = 0.5
            ..restitution = 0.7;
          final xPos = -39.0 + 2 * i;
          final yPos = 50.0 + j;
          final bodyDef = BodyDef()
            ..type = BodyType.DYNAMIC
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
    ..viewport.scale = _MY_VIEWPORT_SCALE
    ..runAnimation();
}
