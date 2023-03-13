import 'dart:html' hide Body;
import 'dart:math';

import 'package:forge2d/forge2d_browser.dart';

import 'demo.dart';
import 'racer/car.dart';
import 'racer/control_state.dart';
import 'racer/ground_area.dart';
import 'racer/tire.dart';

class Racer extends Demo implements ContactListener {
  static void main() {
    final racer = Racer();
    racer.initialize();
    racer.initializeAnimation();
    final paragraph = ParagraphElement()
      ..innerText = 'Use the arrow keys to drive the car';
    document.body.nodes.add(paragraph);
    racer.runAnimation();
  }

  Racer() : super('Racer', Vector2.zero(), 2.5);

  @override
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

  @override
  void step(num time) {
    _car.update(time - _lastTime, _controlState);
    _lastTime = time;
    super.step(time);
  }

  // ContactListener overrides.
  @override
  void beginContact(Contact contact) {
    _handleContact(contact, true);
  }

  @override
  void endContact(Contact contact) {
    _handleContact(contact, false);
  }

  @override
  void preSolve(Contact contact, Manifold oldManifold) {}
  @override
  void postSolve(Contact contact, ContactImpulse impulse) {}

  double radians(double deg) => deg * (pi / 180.0);

  void _createGround() {
    final def = BodyDef();
    _groundBody = world.createBody(def);
    _groundBody.userData = 'Ground';

    final shape = PolygonShape()
      ..setAsBox(27.0, 21.0, Vector2(-30.0, 30.0), radians(20.0));

    final fixtureDef = FixtureDef(shape)
      ..isSensor = true
      ..userData = GroundArea(0.001, false);
    _groundBody.createFixture(fixtureDef);

    fixtureDef.userData = GroundArea(0.2, false);
    shape.setAsBox(27.0, 15.0, Vector2(20.0, 40.0), radians(-40.0));
    _groundBody.createFixture(fixtureDef);
  }

  void _createBoundary() {
    final def = BodyDef();
    final boundaryBody = world.createBody(def);
    boundaryBody.userData = 'Boundary';

    final shape = PolygonShape();

    final fixtureDef = FixtureDef(shape);

    const boundaryX = 150.0;
    const boundaryY = 100.0;

    shape.setAsEdge(
      Vector2(-boundaryX, -boundaryY),
      Vector2(boundaryX, -boundaryY),
    );

    boundaryBody.createFixture(fixtureDef);

    shape.setAsEdge(
      Vector2(boundaryX, -boundaryY),
      Vector2(boundaryX, boundaryY),
    );
    boundaryBody.createFixture(fixtureDef);

    shape.setAsEdge(
      Vector2(boundaryX, boundaryY),
      Vector2(-boundaryX, boundaryY),
    );
    boundaryBody.createFixture(fixtureDef);

    shape.setAsEdge(
      Vector2(-boundaryX, boundaryY),
      Vector2(-boundaryX, -boundaryY),
    );
    boundaryBody.createFixture(fixtureDef);
  }

  void _handleKeyDown(KeyboardEvent event) {
    switch (event.keyCode) {
      case 37:
        _controlState |= ControlState.left;
        break;
      case 38:
        _controlState |= ControlState.up;
        break;
      case 39:
        _controlState |= ControlState.right;
        break;
      case 40:
        _controlState |= ControlState.down;
        break;
    }
  }

  void _handleKeyUp(KeyboardEvent event) {
    switch (event.keyCode) {
      case 37:
        _controlState &= ~ControlState.left;
        break;
      case 38:
        _controlState &= ~ControlState.up;
        break;
      case 39:
        _controlState &= ~ControlState.right;
        break;
      case 40:
        _controlState &= ~ControlState.down;
        break;
    }
  }

  // TODO(any): collision filtering.
  //   Tire with Boundary
  //   Tire with GroundArea
  void _handleContact(Contact contact, bool began) {
    final fudA = contact.fixtureA.userData;
    final fudB = contact.fixtureB.userData;

    // Check for ground area collision.
    // TODO(any): named parameters instead of swapping order?
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
