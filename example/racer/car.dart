import 'dart:math';

import 'package:forge2d/forge2d.dart';

import 'control_state.dart';
import 'tire.dart';

class Car {
  final double _maxForwardSpeed = 250.0;
  final double _maxBackwardSpeed = -40.0;
  final double _backTireMaxDriveForce = 300.0;
  final double _frontTireMaxDriveForce = 500.0;
  final double _backTireMaxLateralImpulse = 8.5;
  final double _frontTireMaxLateralImpulse = 7.5;

  final double _lockAngle = (pi / 180) * 35;
  final double _turnSpeedPerSec = (pi / 180) * 160;

  Body _body;
  Tire _blTire, _brTire, _flTire, _frTire;
  RevoluteJoint _flJoint, _frJoint;

  Car(World world) {
    final def = BodyDef();
    def.type = BodyType.dynamic;
    _body = world.createBody(def);
    _body.userData = 'Car';
    _body.angularDamping = 3.0;

    final vertices = <Vector2>[
      Vector2(1.5, 0.0),
      Vector2(3.0, 2.5),
      Vector2(2.8, 5.5),
      Vector2(1.0, 10.0),
      Vector2(-1.0, 10.0),
      Vector2(-2.8, 5.5),
      Vector2(-3.0, 2.5),
      Vector2(-1.5, 0.0),
    ];

    final shape = PolygonShape()..set(vertices);

    _body.createFixtureFromShape(shape, 0.1);

    final jointDef = RevoluteJointDef();
    jointDef.bodyA = _body;
    jointDef.enableLimit = true;
    jointDef.lowerAngle = 0.0;
    jointDef.upperAngle = 0.0;
    jointDef.localAnchorB.setZero();

    _blTire = Tire(world, _maxForwardSpeed, _maxBackwardSpeed,
        _backTireMaxDriveForce, _backTireMaxLateralImpulse);
    jointDef.bodyB = _blTire.body;
    jointDef.localAnchorA.setValues(-3.0, 0.75);
    world.createJoint(jointDef);

    _brTire = Tire(world, _maxForwardSpeed, _maxBackwardSpeed,
        _backTireMaxDriveForce, _backTireMaxLateralImpulse);
    jointDef.bodyB = _brTire.body;
    jointDef.localAnchorA.setValues(3.0, 0.75);
    world.createJoint(jointDef);

    _flTire = Tire(world, _maxForwardSpeed, _maxBackwardSpeed,
        _frontTireMaxDriveForce, _frontTireMaxLateralImpulse);
    jointDef.bodyB = _flTire.body;
    jointDef.localAnchorA.setValues(-3.0, 8.5);
    _flJoint = world.createJoint(jointDef) as RevoluteJoint;

    _frTire = Tire(world, _maxForwardSpeed, _maxBackwardSpeed,
        _frontTireMaxDriveForce, _frontTireMaxLateralImpulse);
    jointDef.bodyB = _frTire.body;
    jointDef.localAnchorA.setValues(3.0, 8.5);
    _frJoint = world.createJoint(jointDef) as RevoluteJoint;
  }

  void _updateFriction() {
    _blTire.updateFriction();
    _brTire.updateFriction();
    _flTire.updateFriction();
    _frTire.updateFriction();
  }

  void _updateDrive(int controlState) {
    _blTire.updateDrive(controlState);
    _brTire.updateDrive(controlState);
    _flTire.updateDrive(controlState);
    _frTire.updateDrive(controlState);
  }

  void _updateSteering(num time, int controlState) {
    var desiredAngle = 0.0;
    switch (controlState & (ControlState.left | ControlState.right)) {
      case ControlState.left:
        desiredAngle = _lockAngle;
        break;
      case ControlState.right:
        desiredAngle = -_lockAngle;
        break;
    }
    final turnPerTimeStep = _turnSpeedPerSec * 1000 / time;
    final angleNow = _flJoint.getJointAngle();
    final angleToTurn = (desiredAngle - angleNow)
        .clamp(-turnPerTimeStep, turnPerTimeStep)
        .toDouble();
    final angle = angleNow + angleToTurn;
    _flJoint.setLimits(angle, angle);
    _frJoint.setLimits(angle, angle);
  }

  void update(num time, int controlState) {
    if (_body.isAwake() || controlState != 0) {
      _body.setAwake(true);
      _updateFriction();
      _updateDrive(controlState);
      _updateSteering(time, controlState);
    }
  }
}
