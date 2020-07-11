library CircleStress;

import 'dart:math' as Math;
import 'package:box2d_flame/box2d.dart';
import 'demo.dart';

/// Scale of the viewport for this Demo.
const double _MY_VIEWPORT_SCALE = 4.0;

class CircleStress extends Demo {
  /// The number of columns of balls in the pen.
  static const int COLUMNS = 8;

  /// This number of balls will be created on each layer.
  static const int LOAD_SIZE = 20;

  /// Construct a new Circle Stress Demo.
  CircleStress() : super("Circle stress");

  /// Creates all bodies.
  void initialize() {
    {
      final bd = BodyDef();
      final ground = world.createBody(bd);
      bodies.add(ground);

      PolygonShape shape = PolygonShape();
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
      b.createFixtureFromFixtureDef(fd);

      // Walls
      sd.setAsBoxXY(3.0, 50.0);
      final wallDef = BodyDef();
      wallDef.position = Vector2(45.0, 25.0);
      var rightWall = world.createBody(wallDef);
      bodies.add(rightWall);
      rightWall.createFixtureFromShape(sd);
      wallDef.position = Vector2(-45.0, 25.0);
      var leftWall = world.createBody(wallDef);
      bodies.add(leftWall);
      leftWall.createFixtureFromShape(sd);

      // Corners
      final cornerDef = BodyDef();
      sd.setAsBoxXY(20.0, 3.0);
      cornerDef.angle = (-Math.pi / 4.0);
      cornerDef.position = Vector2(-35.0, 8.0);
      Body myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(sd);
      cornerDef.angle = (Math.pi / 4.0);
      cornerDef.position = Vector2(35.0, 8.0);
      myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(sd);

      // top
      sd.setAsBoxXY(50.0, 10.0);
      var topDef = BodyDef()
        ..type = BodyType.STATIC
        ..angle = 0.0
        ..position = Vector2(0.0, 75.0);
      final topBody = world.createBody(topDef);
      bodies.add(topBody);
      fd.shape = sd;
      fd.friction = 1.0;
      topBody.createFixtureFromFixtureDef(fd);
    }

    {
      var bd = BodyDef()
        ..type = BodyType.DYNAMIC
        ..position = Vector2(0.0, 10.0);
      int numPieces = 5;
      double radius = 6.0;
      var body = world.createBody(bd);
      bodies.add(body);

      for (int i = 0; i < numPieces; i++) {
        double xPos =
            radius * Math.cos(2 * Math.pi * (i / numPieces.toDouble()));
        double yPos =
            radius * Math.sin(2 * Math.pi * (i / numPieces.toDouble()));

        var cd = CircleShape()
          ..radius = 1.2
          ..position.setValues(xPos, yPos);

        final fd = FixtureDef()
          ..shape = cd
          ..density = 25.0
          ..friction = .1
          ..restitution = .9;

        body.createFixtureFromFixtureDef(fd);
      }

      body.setBullet(false);

      // Create an empty ground body.
      var bodyDef = BodyDef();
      var groundBody = world.createBody(bodyDef);

      RevoluteJointDef rjd = RevoluteJointDef()
        ..initialize(body, groundBody, body.position)
        ..motorSpeed = Math.pi
        ..maxMotorTorque = 1000000.0
        ..enableMotor = true;

      world.createJoint(rjd);

      for (int j = 0; j < COLUMNS; j++) {
        for (int i = 0; i < LOAD_SIZE; i++) {
          CircleShape circ = CircleShape()
            ..radius = 1.0 + (i % 2 == 0 ? 1.0 : -1.0) * .5 * .75;
          var fd2 = FixtureDef()
            ..shape = circ
            ..density = circ.radius * 1.5
            ..friction = 0.5
            ..restitution = 0.7;
          double xPos = -39.0 + 2 * i;
          double yPos = 50.0 + j;
          var bod = BodyDef()
            ..type = BodyType.DYNAMIC
            ..position = Vector2(xPos, yPos);
          Body myBody = world.createBody(bod);
          bodies.add(myBody);
          myBody.createFixtureFromFixtureDef(fd2);
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
