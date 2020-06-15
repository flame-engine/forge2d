library FrictionJointTest;

import 'dart:math' as Math;
import 'package:box2d_flame/box2d.dart';

import 'demo.dart';

class FrictionJointTest extends Demo {
  FrictionJointTest() : super("FrictionJoint test");

  /// Entrypoint.
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
