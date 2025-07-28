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

  late final Body _body;
  late final Tire _backLeftTire;
  late final Tire _backRightTire;
  late final Tire _frontLeftTire;
  late final Tire _frontRightTire;
  late final RevoluteJoint _frontLeftJoint;
  late final RevoluteJoint _frontRightJoint;

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

    _body.createFixtureFromShape(shape, density: 0.1);

    final jointDef = RevoluteJointDef();
    jointDef.bodyA = _body;
    jointDef.enableLimit = true;
    jointDef.lowerAngle = 0.0;
    jointDef.upperAngle = 0.0;
    jointDef.localAnchorB.setZero();

    _backLeftTire = Tire(
      world,
      _maxForwardSpeed,
      _maxBackwardSpeed,
      _backTireMaxDriveForce,
      _backTireMaxLateralImpulse,
    );
    jointDef.bodyB = _backLeftTire.body;
    jointDef.localAnchorA.setValues(-3.0, 0.75);

    world.createJoint(RevoluteJoint(jointDef));

    _backRightTire = Tire(
      world,
      _maxForwardSpeed,
      _maxBackwardSpeed,
      _backTireMaxDriveForce,
      _backTireMaxLateralImpulse,
    );
    jointDef.bodyB = _backRightTire.body;
    jointDef.localAnchorA.setValues(3.0, 0.75);

    world.createJoint(RevoluteJoint(jointDef));

    _frontLeftTire = Tire(
      world,
      _maxForwardSpeed,
      _maxBackwardSpeed,
      _frontTireMaxDriveForce,
      _frontTireMaxLateralImpulse,
    );
    jointDef.bodyB = _frontLeftTire.body;
    jointDef.localAnchorA.setValues(-3.0, 8.5);

    _frontLeftJoint = RevoluteJoint(jointDef);
    world.createJoint(_frontLeftJoint);

    _frontRightTire = Tire(
      world,
      _maxForwardSpeed,
      _maxBackwardSpeed,
      _frontTireMaxDriveForce,
      _frontTireMaxLateralImpulse,
    );
    jointDef.bodyB = _frontRightTire.body;
    jointDef.localAnchorA.setValues(3.0, 8.5);
    _frontRightJoint = RevoluteJoint(jointDef);
    world.createJoint(_frontRightJoint);
  }

  void _updateFriction() {
    _backLeftTire.updateFriction();
    _backRightTire.updateFriction();
    _frontLeftTire.updateFriction();
    _frontRightTire.updateFriction();
  }

  void _updateDrive(int controlState) {
    _backLeftTire.updateDrive(controlState);
    _backRightTire.updateDrive(controlState);
    _frontLeftTire.updateDrive(controlState);
    _frontRightTire.updateDrive(controlState);
  }

  void _updateSteering(num time, int controlState) {
    var desiredAngle = 0.0;
    switch (controlState & (ControlState.left | ControlState.right)) {
      case ControlState.left:
        desiredAngle = _lockAngle;
      case ControlState.right:
        desiredAngle = -_lockAngle;
    }
    final turnPerTimeStep = _turnSpeedPerSec * 1000 / time;
    final angleNow = _frontLeftJoint.jointAngle();
    final angleToTurn = (desiredAngle - angleNow).clamp(
      -turnPerTimeStep,
      turnPerTimeStep,
    );
    final angle = angleNow + angleToTurn;
    _frontLeftJoint.setLimits(angle, angle);
    _frontRightJoint.setLimits(angle, angle);
  }

  void update(num time, int controlState) {
    if (_body.isAwake || controlState != 0) {
      _body.setAwake(true);
      _updateFriction();
      _updateDrive(controlState);
      _updateSteering(time, controlState);
    }
  }
}
