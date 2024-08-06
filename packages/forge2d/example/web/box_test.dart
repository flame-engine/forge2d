import 'dart:math';

import 'package:forge2d/forge2d.dart';

import 'demo.dart';

class BoxTest extends Demo {
  /// Constructs a new BoxTest.
  BoxTest() : super('Box test');

  @override
  void initialize() {
    _createGround();
    _createBox();
  }

  void _createGround() {
    // Create shape
    final shape = PolygonShape();

    // Define body
    final bodyDef = BodyDef();
    bodyDef.position.setValues(0.0, 0.0);

    // Create body
    final ground = world.createBody(bodyDef);

    // Set shape 3 times and create fixture on the body for each
    shape.setAsBoxXY(50.0, 0.4);
    ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, Vector2(-10.0, 0.0), 0.0);
    ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, Vector2(10.0, 0.0), 0.0);
    ground.createFixtureFromShape(shape);

    // Add composite body to list
    bodies.add(ground);
  }

  void _createBox() {
    // Create shape
    final shape = PolygonShape();
    shape.setAsBox(3.0, 1.5, Vector2.zero(), pi / 2);

    // Define fixture (links body and shape)
    final activeFixtureDef = FixtureDef(shape, restitution: 0.5, density: 0.05);

    // Define body
    final bodyDef = BodyDef(
      type: BodyType.dynamic,
      position: Vector2(0.0, 30.0),
    );

    // Create body and fixture from definitions
    final fallingBox = world.createBody(bodyDef);
    fallingBox.createFixture(activeFixtureDef);

    // Add to list
    bodies.add(fallingBox);
  }
}

void main() {
  final boxTest = BoxTest();
  boxTest.initialize();
  boxTest.initializeAnimation();
  boxTest.runAnimation();
}
