import 'dart:math';

import 'package:forge2d/forge2d.dart';

import 'control_state.dart';
import 'ground_area.dart';

class Tire {
  Tire(
    World world,
    this._maxForwardSpeed,
    this._maxBackwardSpeed,
    this._maxDriveForce,
    this._maxLateralImpulse,
  ) {
    final def = BodyDef();
    def.type = BodyType.dynamic;
    body = world.createBody(def);
    body.userData = 'Tire';

    final polygonShape = PolygonShape();
    polygonShape.setAsBoxXY(0.5, 1.25);
    final fixture = body.createFixtureFromShape(polygonShape, density: 1.0);
    fixture.userData = this;

    _currentTraction = 1.0;
  }

  void addGroundArea(GroundArea ga) {
    final newlyAdded = _groundAreas.add(ga);
    if (newlyAdded) {
      _updateTraction();
    }
  }

  void removeGroundArea(GroundArea ga) {
    if (_groundAreas.remove(ga)) {
      _updateTraction();
    }
  }

  void updateFriction() {
    final impulse = _lateralVelocity..scale(-body.mass);
    if (impulse.length > _maxLateralImpulse) {
      impulse.scale(_maxLateralImpulse / impulse.length);
    }
    body.applyLinearImpulse(impulse..scale(_currentTraction));
    body.applyAngularImpulse(
      0.1 * _currentTraction * body.getInertia() * (-body.angularVelocity),
    );

    final currentForwardNormal = _forwardVelocity;
    final currentForwardSpeed = currentForwardNormal.length;
    currentForwardNormal.normalize();
    final dragForceMagnitude = -2 * currentForwardSpeed;
    body.applyForce(
      currentForwardNormal..scale(_currentTraction * dragForceMagnitude),
    );
  }

  void updateDrive(int controlState) {
    var desiredSpeed = 0.0;
    switch (controlState & (ControlState.up | ControlState.down)) {
      case ControlState.up:
        desiredSpeed = _maxForwardSpeed;
        break;
      case ControlState.down:
        desiredSpeed = _maxBackwardSpeed;
        break;
      default:
        return;
    }

    final currentForwardNormal = body.worldVector(Vector2(0.0, 1.0));
    final currentSpeed = _forwardVelocity.dot(currentForwardNormal);
    var force = 0.0;
    if (desiredSpeed < currentSpeed) {
      force = -_maxDriveForce;
    } else if (desiredSpeed > currentSpeed) {
      force = _maxDriveForce;
    }

    if (force.abs() > 0) {
      body.applyForce(currentForwardNormal..scale(_currentTraction * force));
    }
  }

  void updateTurn(int controlState) {
    var desiredTorque = 0.0;
    switch (controlState & (ControlState.left | ControlState.right)) {
      case ControlState.left:
        desiredTorque = 15.0;
        break;
      case ControlState.right:
        desiredTorque = -15.0;
        break;
    }
    body.applyTorque(desiredTorque);
  }

  void _updateTraction() {
    if (_groundAreas.isEmpty) {
      _currentTraction = 1.0;
    } else {
      _currentTraction = 0.0;
      _groundAreas.forEach((element) {
        _currentTraction = max(_currentTraction, element.frictionModifier);
      });
    }
  }

  Vector2 get _lateralVelocity {
    final currentRightNormal = body.worldVector(_worldLeft);
    return currentRightNormal
      ..scale(currentRightNormal.dot(body.linearVelocity));
  }

  Vector2 get _forwardVelocity {
    final currentForwardNormal = body.worldVector(_worldUp);
    return currentForwardNormal
      ..scale(currentForwardNormal.dot(body.linearVelocity));
  }

  Body body;
  final double _maxForwardSpeed;
  final double _maxBackwardSpeed;
  final double _maxDriveForce;
  final double _maxLateralImpulse;
  double _currentTraction;
  final Set<GroundArea> _groundAreas = <GroundArea>{};

  // Cached Vectors to reduce unnecessary object creation.
  final Vector2 _worldLeft = Vector2(1.0, 0.0);
  final Vector2 _worldUp = Vector2(0.0, 1.0);
}
