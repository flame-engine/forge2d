import 'dart:math';

import 'package:forge2d/forge2d_browser.dart';

import 'demo.dart';

class FrictionJointTest extends Demo {
  FrictionJointTest() : super('FrictionJoint test');

  /// Entrypoint.
  void main() {
    final test = FrictionJointTest();
    test.initialize();
    test.initializeAnimation();
    test.debugDraw.appendFlags(DebugDraw.jointBit);
    test.runAnimation();
  }

  late Body _ground;
  late FixtureDef _boxFixture;

  @override
  void initialize() {
    _createGround();
    _createBoxShapeAndFixture();
    _createBox();
    _createFrictionBox();
  }

  void _createGround() {
    // Create shape
    final shape = PolygonShape();

    // Define body
    final bodyDef = BodyDef();
    bodyDef.position.setValues(0.0, 0.0);

    // Create body
    _ground = world.createBody(bodyDef);

    // Set shape 3 times and create fixture on the body for each
    shape.setAsBoxXY(50.0, 0.4);
    _ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, Vector2(-20.0, 0.0), 0.0);
    _ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, Vector2(20.0, 0.0), 0.0);
    _ground.createFixtureFromShape(shape);

    // Add composite body to list
    bodies.add(_ground);
  }

  void _createBoxShapeAndFixture() {
    final boxShape = PolygonShape();
    boxShape.setAsBox(3.0, 1.5, Vector2.zero(), pi / 2);

    // Define fixture (links body and shape)
    _boxFixture = FixtureDef(boxShape, restitution: 0.5, density: 0.10);
  }

  void _createBox() {
    // Define body
    final bodyDef = BodyDef(
      type: BodyType.dynamic,
      position: Vector2(-10.0, 30.0),
    );

    // Create body and fixture from definitions
    final fallingBox = world.createBody(bodyDef);
    fallingBox.createFixture(_boxFixture);

    // Add to list
    bodies.add(fallingBox);
  }

  void _createFrictionBox() {
    // Define body
    final bodyDef = BodyDef(
      type: BodyType.dynamic,
      position: Vector2(10.0, 30.0),
    );

    // Create body and fixture from definitions
    final fallingBox = world.createBody(bodyDef);
    fallingBox.createFixture(_boxFixture);

    final frictionJointDef = FrictionJointDef();
    frictionJointDef.bodyA = fallingBox;
    frictionJointDef.bodyB = _ground;
    frictionJointDef.maxForce = 3.0;
    frictionJointDef.maxTorque = 5.0;
    frictionJointDef.collideConnected = true;

    final frictionJoint = FrictionJoint(frictionJointDef);
    world.createJoint(frictionJoint);

    // Add to list
    bodies.add(fallingBox);
  }
}

void main() {
  FrictionJointTest.main();
}
