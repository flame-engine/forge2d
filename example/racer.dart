library racer;

import 'dart:html' hide Body;
import 'dart:math';
import 'package:box2d_flame/box2d.dart';
import 'package:box2d_flame/src/math_utils.dart' as MathUtils;

import 'demo.dart';

part 'racer/car.dart';
part 'racer/control_state.dart';
part 'racer/ground_area.dart';
part 'racer/tire.dart';

class Racer extends Demo implements ContactListener {
  static void main() {
    final racer = Racer();
    racer.initialize();
    racer.initializeAnimation();
    document.body.nodes
        .add(Element.html("<p>Use the arrow keys to drive the car.</p>"));
    racer.runAnimation();
  }

  Racer() : super("Racer", Vector2.zero(), 2.5);

  void initialize() {
    _createGround();
    _createBoundary();

    _car = Car(world);
    _controlState = 0;

    // Bind to keyboard events.
    document.onKeyDown.listen(_handleKeyDown);
    document.onKeyUp.listen(_handleKeyUp);

    // Add ourselves as a collision listener.
    world.setContactListener(this);
  }

  void step(num time) {
    _car.update(time - _lastTime, _controlState);
    _lastTime = time;
    super.step(time);
  }

  // ContactListener overrides.
  void beginContact(Contact contact) {
    _handleContact(contact, true);
  }

  void endContact(Contact contact) {
    _handleContact(contact, false);
  }

  void preSolve(Contact contact, Manifold oldManifold) {}
  void postSolve(Contact contact, ContactImpulse impulse) {}

  double radians(double deg) => deg * (pi / 180.0);

  void _createGround() {
    BodyDef def = BodyDef();
    _groundBody = world.createBody(def);
    _groundBody.userData = "Ground";

    PolygonShape shape = PolygonShape();

    FixtureDef fixtureDef = FixtureDef();
    fixtureDef.shape = shape;
    fixtureDef.isSensor = true;

    fixtureDef.userData = GroundArea(0.001, false);
    shape.setAsBox(27.0, 21.0, Vector2(-30.0, 30.0), radians(20.0));
    _groundBody.createFixtureFromFixtureDef(fixtureDef);

    fixtureDef.userData = GroundArea(0.2, false);
    shape.setAsBox(27.0, 15.0, Vector2(20.0, 40.0), radians(-40.0));
    _groundBody.createFixtureFromFixtureDef(fixtureDef);
  }

  void _createBoundary() {
    BodyDef def = BodyDef();
    Body boundaryBody = world.createBody(def);
    boundaryBody.userData = "Boundary";

    PolygonShape shape = PolygonShape();

    FixtureDef fixtureDef = FixtureDef();
    fixtureDef.shape = shape;

    final double boundaryX = 150.0;
    final double boundaryY = 100.0;

    shape.setAsEdge(
        Vector2(-boundaryX, -boundaryY), Vector2(boundaryX, -boundaryY));

    boundaryBody.createFixtureFromFixtureDef(fixtureDef);

    shape.setAsEdge(
        Vector2(boundaryX, -boundaryY), Vector2(boundaryX, boundaryY));
    boundaryBody.createFixtureFromFixtureDef(fixtureDef);

    shape.setAsEdge(
        Vector2(boundaryX, boundaryY), Vector2(-boundaryX, boundaryY));
    boundaryBody.createFixtureFromFixtureDef(fixtureDef);

    shape.setAsEdge(
        Vector2(-boundaryX, boundaryY), Vector2(-boundaryX, -boundaryY));
    boundaryBody.createFixtureFromFixtureDef(fixtureDef);
  }

  void _handleKeyDown(KeyboardEvent event) {
    switch (event.keyCode) {
      case 37:
        _controlState |= ControlState.LEFT;
        break;
      case 38:
        _controlState |= ControlState.UP;
        break;
      case 39:
        _controlState |= ControlState.RIGHT;
        break;
      case 40:
        _controlState |= ControlState.DOWN;
        break;
    }
  }

  void _handleKeyUp(KeyboardEvent event) {
    switch (event.keyCode) {
      case 37:
        _controlState &= ~ControlState.LEFT;
        break;
      case 38:
        _controlState &= ~ControlState.UP;
        break;
      case 39:
        _controlState &= ~ControlState.RIGHT;
        break;
      case 40:
        _controlState &= ~ControlState.DOWN;
        break;
    }
  }

  // TODO: collision filtering.
  //   Tire with Boundary
  //   Tire with GroundArea
  void _handleContact(Contact contact, bool began) {
    final Object fudA = contact.fixtureA.userData;
    final Object fudB = contact.fixtureB.userData;

    // Check for ground area collision.
    // TODO: named parameters instead of swapping order?
    if (fudA is Tire && fudB is GroundArea) {
      _tireVsGroundArea(fudA, fudB, began);
    } else if (fudA is GroundArea && fudB is Tire) {
      _tireVsGroundArea(fudB, fudA, began);
    }
  }

  void _tireVsGroundArea(Tire tire, GroundArea groundArea, bool began) {
    if (began) {
      tire.addGroundArea(groundArea);
    } else {
      tire.removeGroundArea(groundArea);
    }
  }

  int _controlState;
  Body _groundBody;
  Car _car;
  num _lastTime = 0;
}

void main() {
  Racer.main();
}
