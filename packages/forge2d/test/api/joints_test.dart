@TestOn('vm')
library;

import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  late World world;
  late Body ground;

  Body dynamicBox(double x, double y) => world.createBody(
    BodyDef(type: BodyType.dynamic, position: Vector2(x, y)),
  )..createShape(Polygon.square(0.5));

  setUp(() {
    world = World();
    ground = world.createBody(BodyDef(position: Vector2(0, -1)))
      ..createShape(Polygon.box(50, 1));
  });
  tearDown(() => world.destroy());

  group('Joint (common)', () {
    test('exposes bodies, anchors, type, and user data', () {
      final body = dynamicBox(0, 2);
      final joint = world.createRevoluteJoint(
        RevoluteJointDef(
          bodyA: ground,
          bodyB: body,
          localAnchorA: Vector2(0, 3),
          localAnchorB: Vector2(0, 1),
          userData: 'hinge',
        ),
      );

      expect(joint.isValid, isTrue);
      expect(joint.type, JointType.revolute);
      expect(joint.bodyA, ground);
      expect(joint.bodyB, body);
      expect(joint.localAnchorA.y, closeTo(3, 1e-6));
      expect(joint.localAnchorB.y, closeTo(1, 1e-6));
      expect(joint.collideConnected, isFalse);
      expect(joint.userData, 'hinge');

      joint.userData = null;
      expect(world.jointUserData, isEmpty);
    });

    test('destroy removes the joint', () {
      final joint = world.createDistanceJoint(
        DistanceJointDef(
          bodyA: ground,
          bodyB: dynamicBox(0, 5),
          userData: 'x',
        ),
      );
      joint.destroy();
      expect(joint.isValid, isFalse);
      expect(world.jointUserData, isEmpty);
    });

    test('destroying a body destroys its joints', () {
      final body = dynamicBox(0, 5);
      final joint = world.createDistanceJoint(
        DistanceJointDef(bodyA: ground, bodyB: body, userData: 'x'),
      );
      expect(body.joints, hasLength(1));
      body.destroy();
      expect(joint.isValid, isFalse);
      expect(world.jointUserData, isEmpty);
    });

    test('body.joints returns typed joints', () {
      final body = dynamicBox(0, 5);
      world.createWheelJoint(WheelJointDef(bodyA: ground, bodyB: body));
      expect(body.joints.single, isA<WheelJoint>());
    });
  });

  group('DistanceJoint', () {
    test('definition values round-trip', () {
      final joint = world.createDistanceJoint(
        DistanceJointDef(
          bodyA: ground,
          bodyB: dynamicBox(0, 5),
          length: 2.5,
          enableSpring: true,
          hertz: 3,
          dampingRatio: 0.5,
          enableLimit: true,
          minLength: 1,
          maxLength: 4,
          enableMotor: true,
          maxMotorForce: 10,
          motorSpeed: 0.5,
        ),
      );

      expect(joint.length, closeTo(2.5, 1e-6));
      expect(joint.springEnabled, isTrue);
      expect(joint.springHertz, closeTo(3, 1e-6));
      expect(joint.springDampingRatio, closeTo(0.5, 1e-6));
      expect(joint.limitEnabled, isTrue);
      expect(joint.minLength, closeTo(1, 1e-6));
      expect(joint.maxLength, closeTo(4, 1e-6));
      expect(joint.motorEnabled, isTrue);
      expect(joint.maxMotorForce, closeTo(10, 1e-6));
      expect(joint.motorSpeed, closeTo(0.5, 1e-6));
    });

    test('a rigid distance joint holds its length', () {
      final body = dynamicBox(0, 3);
      final joint = world.createDistanceJoint(
        DistanceJointDef(
          bodyA: ground,
          bodyB: body,
          localAnchorA: Vector2(0, 7),
          length: 4,
        ),
      );
      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
      }
      expect(joint.currentLength, closeTo(4, 0.05));
    });
  });

  group('FilterJoint', () {
    test('disables collision between the bodies', () {
      // Two overlapping boxes at the same position: without the filter
      // joint they push each other apart.
      final a = dynamicBox(20, 0.5);
      final b = dynamicBox(20, 0.5);
      world
        ..createFilterJoint(FilterJointDef(bodyA: a, bodyB: b))
        ..step(1 / 60, subStepCount: 8);
      for (var i = 0; i < 60; i++) {
        world.step(1 / 60);
      }
      expect((a.position - b.position).length, lessThan(0.01));
    });
  });

  group('MotorJoint', () {
    test('definition values round-trip and can be updated', () {
      final joint = world.createMotorJoint(
        MotorJointDef(
          bodyA: ground,
          bodyB: dynamicBox(0, 2),
          linearOffset: Vector2(1, 3),
          angularOffset: 0.4,
          maxForce: 100,
          maxTorque: 50,
          correctionFactor: 0.5,
        ),
      );

      expect(joint.linearOffset.x, closeTo(1, 1e-6));
      expect(joint.linearOffset.y, closeTo(3, 1e-6));
      expect(joint.angularOffset, closeTo(0.4, 1e-6));
      expect(joint.maxForce, 100);
      expect(joint.maxTorque, 50);
      expect(joint.correctionFactor, closeTo(0.5, 1e-6));

      joint
        ..linearOffset = Vector2(2, 2)
        ..maxForce = 500;
      expect(joint.linearOffset.x, closeTo(2, 1e-6));
      expect(joint.maxForce, 500);
    });
  });

  group('MouseJoint', () {
    test('drags a body towards the target', () {
      final body = dynamicBox(0, 2);
      final joint = world.createMouseJoint(
        MouseJointDef(
          bodyA: ground,
          bodyB: body,
          target: body.position,
          hertz: 5,
          maxForce: 1000 * body.mass,
        ),
      );
      joint.target = Vector2(5, 6);
      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
      }
      expect(joint.target, Vector2(5, 6));
      expect((body.position - Vector2(5, 6)).length, lessThan(0.5));
    });
  });

  group('PrismaticJoint', () {
    test('restricts motion to the joint axis and respects limits', () {
      final body = dynamicBox(0, 2);
      final joint = world.createPrismaticJoint(
        PrismaticJointDef(
          bodyA: ground,
          bodyB: body,
          localAnchorA: Vector2(0, 3),
          localAxisA: Vector2(0, 1),
          enableLimit: true,
          lowerTranslation: -1,
          upperTranslation: 1,
        ),
      );

      expect(joint.limitEnabled, isTrue);
      expect(joint.lowerLimit, -1);
      expect(joint.upperLimit, 1);

      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
      }
      // Gravity pulls the body down along the axis until the lower limit.
      expect(joint.translation, closeTo(-1, 0.05));
      expect(body.position.x.abs(), lessThan(1e-4));
    });

    test('the motor drives translation', () {
      final body = dynamicBox(0, 2);
      final joint = world.createPrismaticJoint(
        PrismaticJointDef(
          bodyA: ground,
          bodyB: body,
          localAnchorA: Vector2(0, 3),
          localAxisA: Vector2(1, 0),
          enableMotor: true,
          motorSpeed: 2,
          maxMotorForce: 1000,
        ),
      );
      for (var i = 0; i < 60; i++) {
        world.step(1 / 60);
      }
      expect(joint.translation, closeTo(2, 0.1));
      expect(joint.speed, closeTo(2, 0.1));
    });
  });

  group('RevoluteJoint', () {
    test('the limit clamps the angle', () {
      final body = dynamicBox(2, 5);
      final joint = world.createRevoluteJoint(
        RevoluteJointDef(
          bodyA: ground,
          bodyB: body,
          localAnchorA: Vector2(0, 6),
          localAnchorB: Vector2(-2, 0),
          enableLimit: true,
          lowerAngle: -math.pi / 8,
          upperAngle: math.pi / 8,
        ),
      );
      for (var i = 0; i < 240; i++) {
        world.step(1 / 60);
      }
      expect(joint.angle, greaterThanOrEqualTo(-math.pi / 8 - 0.02));
      expect(joint.angle, lessThanOrEqualTo(math.pi / 8 + 0.02));
    });

    test('the motor spins the body', () {
      final body = dynamicBox(0, 5);
      world.createRevoluteJoint(
        RevoluteJointDef(
          bodyA: ground,
          bodyB: body,
          localAnchorA: Vector2(0, 6),
          enableMotor: true,
          motorSpeed: 1,
          maxMotorTorque: 1000,
        ),
      );
      for (var i = 0; i < 60; i++) {
        world.step(1 / 60);
      }
      // At the target speed with the pin through the center of mass, the
      // motor needs no sustained torque, so only the speed is asserted.
      expect(body.angularVelocity, closeTo(1, 0.05));
    });
  });

  group('WeldJoint', () {
    test('rigidly attaches two bodies', () {
      final a = dynamicBox(0, 5);
      final b = dynamicBox(1, 5);
      world.createWeldJoint(
        WeldJointDef(
          bodyA: a,
          bodyB: b,
          localAnchorA: Vector2(0.5, 0),
          localAnchorB: Vector2(-0.5, 0),
        ),
      );
      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
      }
      expect((a.position - b.position).length, closeTo(1, 0.05));
    });

    test('stiffness properties round-trip', () {
      final joint = world.createWeldJoint(
        WeldJointDef(
          bodyA: ground,
          bodyB: dynamicBox(0, 3),
          linearHertz: 2,
          angularHertz: 3,
          linearDampingRatio: 0.4,
          angularDampingRatio: 0.6,
        ),
      );
      expect(joint.linearHertz, closeTo(2, 1e-6));
      expect(joint.angularHertz, closeTo(3, 1e-6));
      expect(joint.linearDampingRatio, closeTo(0.4, 1e-6));
      expect(joint.angularDampingRatio, closeTo(0.6, 1e-6));
    });
  });

  group('WheelJoint', () {
    test('native defaults are applied', () {
      final joint = world.createWheelJoint(
        WheelJointDef(bodyA: ground, bodyB: dynamicBox(0, 2)),
      );
      expect(joint.springEnabled, isTrue);
      expect(joint.springHertz, closeTo(1, 1e-6));
      expect(joint.springDampingRatio, closeTo(0.7, 1e-6));
      expect(joint.motorEnabled, isFalse);
      expect(joint.limitEnabled, isFalse);
    });

    test('the suspension carries a falling wheel', () {
      final wheel = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(0, 3)),
      )..createShape(Circle(radius: 0.5));
      final joint = world.createWheelJoint(
        WheelJointDef(
          bodyA: ground,
          bodyB: wheel,
          localAnchorA: Vector2(0, 4),
          localAxisA: Vector2(0, 1),
          hertz: 2,
        ),
      );
      for (var i = 0; i < 240; i++) {
        world.step(1 / 60);
      }
      // The wheel hangs from the suspension spring near its anchor.
      expect(wheel.position.x.abs(), lessThan(1e-4));
      expect(joint.isValid, isTrue);
    });
  });
}
