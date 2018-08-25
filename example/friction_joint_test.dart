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

library FrictionJointTest;

import 'dart:math' as Math;
import 'package:box2d_flame/box2d.dart';

import 'demo.dart';

class FrictionJointTest extends Demo {
  FrictionJointTest() : super("FrictionJoint test");

  /** Entrypoint. */
  static void main() {
    final test = new FrictionJointTest();
    test.initialize();
    test.initializeAnimation();
    test.debugDraw.appendFlags(DebugDraw.JOINT_BIT);
    test.runAnimation();
  }

  void initialize() {
    assert(null != world);

    _createGround();
    _createBoxShapeAndFixture();
    _createBox();
    _createFrictionBox();
  }

  void _createGround() {
    // Create shape
    final PolygonShape shape = new PolygonShape();

    // Define body
    final BodyDef bodyDef = new BodyDef();
    bodyDef.position.setValues(0.0, 0.0);

    // Create body
    _ground = world.createBody(bodyDef);

    // Set shape 3 times and create fixture on the body for each
    shape.setAsBoxXY(50.0, 0.4);
    _ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, new Vector2(-20.0, 0.0), 0.0);
    _ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, new Vector2(20.0, 0.0), 0.0);
    _ground.createFixtureFromShape(shape);

    // Add composite body to list
    bodies.add(_ground);
  }

  void _createBoxShapeAndFixture() {
    final PolygonShape boxShape = new PolygonShape();
    boxShape.setAsBox(3.0, 1.5, new Vector2.zero(), Math.pi / 2);

    // Define fixture (links body and shape)
    _boxFixture = new FixtureDef();
    _boxFixture.restitution = 0.5;
    _boxFixture.density = 0.10;
    _boxFixture.shape = boxShape;
  }

  void _createBox() {
    // Define body
    final BodyDef bodyDef = new BodyDef();
    bodyDef.type = BodyType.DYNAMIC;
    bodyDef.position = new Vector2(-10.0, 30.0);

    // Create body and fixture from definitions
    final Body fallingBox = world.createBody(bodyDef);
    fallingBox.createFixtureFromFixtureDef(_boxFixture);

    // Add to list
    bodies.add(fallingBox);
  }

  void _createFrictionBox() {
    // Define body
    final BodyDef bodyDef = new BodyDef();
    bodyDef.type = BodyType.DYNAMIC;
    bodyDef.position = new Vector2(10.0, 30.0);

    // Create body and fixture from definitions
    final Body fallingBox = world.createBody(bodyDef);
    fallingBox.createFixtureFromFixtureDef(_boxFixture);

    final FrictionJointDef frictionJointDef = new FrictionJointDef();
    frictionJointDef.bodyA = fallingBox;
    frictionJointDef.bodyB = _ground;
    frictionJointDef.maxForce = 3.0;
    frictionJointDef.maxTorque = 5.0;
    frictionJointDef.collideConnected = true;

    world.createJoint(frictionJointDef);

    // Add to list
    bodies.add(fallingBox);
  }

  Body _ground;
  FixtureDef _boxFixture;
}

void main() {
  FrictionJointTest.main();
}
