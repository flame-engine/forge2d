library BoxTest;

import 'dart:math' as Math;
import 'package:box2d_flame/box2d.dart';
import 'demo.dart';

class BoxTest extends Demo {
  /// Constructs a new BoxTest.
  BoxTest() : super("Box test");

  /// Entrypoint.
  static void main() {
    final boxTest = new BoxTest();
    boxTest.initialize();
    boxTest.initializeAnimation();
    boxTest.runAnimation();
  }

  void initialize() {
    assert(null != world);
    _createGround();
    _createBox();
  }

  void _createGround() {
    // Create shape
    final PolygonShape shape = new PolygonShape();

    // Define body
    final BodyDef bodyDef = new BodyDef();
    bodyDef.position.setValues(0.0, 0.0);

    // Create body
    final Body ground = world.createBody(bodyDef);

    // Set shape 3 times and create fixture on the body for each
    shape.setAsBoxXY(50.0, 0.4);
    ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, new Vector2(-10.0, 0.0), 0.0);
    ground.createFixtureFromShape(shape);
    shape.setAsBox(0.4, 50.0, new Vector2(10.0, 0.0), 0.0);
    ground.createFixtureFromShape(shape);

    // Add composite body to list
    bodies.add(ground);
  }

  void _createBox() {
    // Create shape
    final PolygonShape shape = new PolygonShape();
    shape.setAsBox(3.0, 1.5, new Vector2.zero(), Math.pi / 2);

    // Define fixture (links body and shape)
    final FixtureDef activeFixtureDef = new FixtureDef();
    activeFixtureDef.restitution = 0.5;
    activeFixtureDef.density = 0.05;
    activeFixtureDef.shape = shape;

    // Define body
    final BodyDef bodyDef = new BodyDef();
    bodyDef.type = BodyType.DYNAMIC;
    bodyDef.position = new Vector2(0.0, 30.0);

    // Create body and fixture from definitions
    final Body fallingBox = world.createBody(bodyDef);
    fallingBox.createFixtureFromFixtureDef(activeFixtureDef);

    // Add to list
    bodies.add(fallingBox);
  }
}

void main() {
  BoxTest.main();
}
