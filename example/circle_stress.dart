/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

library CircleStress;

import 'dart:math' as Math;
import 'package:box2d_flame/box2d.dart';
import 'demo.dart';

/** Scale of the viewport for this Demo. */
const double _MY_VIEWPORT_SCALE = 4.0;

class CircleStress extends Demo {
  /** The number of columns of balls in the pen. */
  static const int COLUMNS = 8;

  /** This number of balls will be created on each layer. */
  static const int LOAD_SIZE = 20;

  /** Construct a new Circle Stress Demo. */
  CircleStress() : super("Circle stress");

  /** Creates all bodies. */
  void initialize() {
    {
      final bd = new BodyDef();
      final ground = world.createBody(bd);
      bodies.add(ground);

      PolygonShape shape = new PolygonShape();
      shape.setAsEdge(new Vector2(-40.0, 0.0), new Vector2(40.0, 0.0));
      ground.createFixtureFromShape(shape);
    }

    {
      // Ground
      final sd = new PolygonShape();
      sd.setAsBoxXY(50.0, 10.0);
      final bd = new BodyDef();
      bd.type = BodyType.STATIC;
      bd.position = new Vector2(0.0, -10.0);
      final b = world.createBody(bd);
      bodies.add(b);
      final fd = new FixtureDef();
      fd.shape = sd;
      fd.friction = 1.0;
      b.createFixtureFromFixtureDef(fd);

      // Walls
      sd.setAsBoxXY(3.0, 50.0);
      final wallDef = new BodyDef();
      wallDef.position = new Vector2(45.0, 25.0);
      var rightWall = world.createBody(wallDef);
      bodies.add(rightWall);
      rightWall.createFixtureFromShape(sd);
      wallDef.position = new Vector2(-45.0, 25.0);
      var leftWall = world.createBody(wallDef);
      bodies.add(leftWall);
      leftWall.createFixtureFromShape(sd);

      // Corners
      final cornerDef = new BodyDef();
      sd.setAsBoxXY(20.0, 3.0);
      cornerDef.angle = (-Math.pi / 4.0);
      cornerDef.position = new Vector2(-35.0, 8.0);
      Body myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(sd);
      cornerDef.angle = (Math.pi / 4.0);
      cornerDef.position = new Vector2(35.0, 8.0);
      myBod = world.createBody(cornerDef);
      bodies.add(myBod);
      myBod.createFixtureFromShape(sd);

      // top
      sd.setAsBoxXY(50.0, 10.0);
      var topDef = new BodyDef()
        ..type = BodyType.STATIC
        ..angle = 0.0
        ..position = new Vector2(0.0, 75.0);
      final topBody = world.createBody(topDef);
      bodies.add(topBody);
      fd.shape = sd;
      fd.friction = 1.0;
      topBody.createFixtureFromFixtureDef(fd);
    }

    {
      var bd = new BodyDef()
        ..type = BodyType.DYNAMIC
        ..position = new Vector2(0.0, 10.0);
      int numPieces = 5;
      double radius = 6.0;
      var body = world.createBody(bd);
      bodies.add(body);

      for (int i = 0; i < numPieces; i++) {
        double xPos =
            radius * Math.cos(2 * Math.pi * (i / numPieces.toDouble()));
        double yPos =
            radius * Math.sin(2 * Math.pi * (i / numPieces.toDouble()));

        var cd = new CircleShape()
          ..radius = 1.2
          ..p.setValues(xPos, yPos);

        final fd = new FixtureDef()
          ..shape = cd
          ..density = 25.0
          ..friction = .1
          ..restitution = .9;

        body.createFixtureFromFixtureDef(fd);
      }

      body.setBullet(false);

      // Create an empty ground body.
      var bodyDef = new BodyDef();
      var groundBody = world.createBody(bodyDef);

      RevoluteJointDef rjd = new RevoluteJointDef()
        ..initialize(body, groundBody, body.position)
        ..motorSpeed = Math.pi
        ..maxMotorTorque = 1000000.0
        ..enableMotor = true;

      world.createJoint(rjd);

      for (int j = 0; j < COLUMNS; j++) {
        for (int i = 0; i < LOAD_SIZE; i++) {
          CircleShape circ = new CircleShape()
            ..radius = 1.0 + (i % 2 == 0 ? 1.0 : -1.0) * .5 * .75;
          var fd2 = new FixtureDef()
            ..shape = circ
            ..density = circ.radius * 1.5
            ..friction = 0.5
            ..restitution = 0.7;
          double xPos = -39.0 + 2 * i;
          double yPos = 50.0 + j;
          var bod = new BodyDef()
            ..type = BodyType.DYNAMIC
            ..position = new Vector2(xPos, yPos);
          Body myBody = world.createBody(bod);
          bodies.add(myBody);
          myBody.createFixtureFromFixtureDef(fd2);
        }
      }
    }
  }
}

void main() {
  new CircleStress()
    ..initialize()
    ..initializeAnimation()
    ..viewport.scale = _MY_VIEWPORT_SCALE
    ..runAnimation();
}
