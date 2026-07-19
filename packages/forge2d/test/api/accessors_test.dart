import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

/// Exhaustive get/set round-trips for every public accessor, so that a
/// broken seam mapping (wrong native function, wrong argument order) in
/// either backend cannot go unnoticed.
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

  test('Body accessors round-trip', () {
    final body = dynamicBox(0, 5)
      ..angularVelocity = 1.25
      ..linearVelocity = Vector2(2, 3)
      ..gravityScale = 0.5
      ..linearDamping = 0.125
      ..angularDamping = 0.25
      ..sleepThreshold = 0.75
      ..sleepEnabled = false
      ..isBullet = true
      ..name = 'renamed';

    expect(body.angularVelocity, closeTo(1.25, 1e-6));
    expect(body.linearVelocity.x, closeTo(2, 1e-6));
    expect(body.gravityScale, closeTo(0.5, 1e-6));
    expect(body.linearDamping, closeTo(0.125, 1e-6));
    expect(body.angularDamping, closeTo(0.25, 1e-6));
    expect(body.sleepThreshold, closeTo(0.75, 1e-6));
    expect(body.sleepEnabled, isFalse);
    expect(body.isBullet, isTrue);
    // Fixing the rotation zeroes the angular velocity, so it is set last.
    body.fixedRotation = true;
    expect(body.fixedRotation, isTrue);
    expect(body.angularVelocity, 0);
    expect(body.name, 'renamed');
    expect(body.transform.position.y, closeTo(5, 1e-6));
    expect(body.worldCenterOfMass.y, closeTo(5, 1e-6));

    body.applyForce(Vector2(10, 0), point: body.worldPoint(Vector2(0, 0.5)));
    body.applyTorque(3);
    body.applyLinearImpulse(
      Vector2(0, 1),
      point: body.worldPoint(Vector2(0.1, 0)),
    );
    body.applyAngularImpulse(0.5);
    world.step(1 / 60);
    expect(body.isValid, isTrue);
  });

  test('Shape accessors round-trip', () {
    final shape = dynamicBox(0, 5).shapes.single
      ..preSolveEventsEnabled = true
      ..userData = 'tagged';
    expect(shape.preSolveEventsEnabled, isTrue);
    expect(shape.userData, 'tagged');
    shape.userData = null;
    expect(shape.userData, isNull);

    shape.setDensity(4, updateBodyMass: false);
    expect(shape.density, 4);
    shape.body.applyMassFromShapes();
    expect(shape.body.mass, closeTo(4, 1e-5));
  });

  test('Chain accessors round-trip', () {
    final chain = ground.createChain(
      ChainDef(
        points: [Vector2(-6, 4), Vector2(-2, 4), Vector2(2, 4), Vector2(6, 4)],
      ),
    )..userData = 'rail';
    expect(chain.userData, 'rail');
    chain.userData = null;
    expect(chain.userData, isNull);
    expect(chain.segments.first.body, ground);
  });

  test('Joint common accessors round-trip', () {
    final body = dynamicBox(0, 3);
    final joint = world.createDistanceJoint(
      DistanceJointDef(
        bodyA: ground,
        bodyB: body,
        localAnchorA: Vector2(0, 7),
        length: 4,
      ),
    )..collideConnected = true;
    expect(joint.collideConnected, isTrue);
    joint.wakeBodies();
    expect(body.isAwake, isTrue);

    for (var i = 0; i < 30; i++) {
      world.step(1 / 60);
    }
    // The joint carries the hanging body against gravity.
    expect(joint.constraintForce.length, greaterThan(0));
    expect(joint.constraintTorque.isFinite, isTrue);
  });

  test('DistanceJoint accessors round-trip', () {
    final joint =
        world.createDistanceJoint(
            DistanceJointDef(bodyA: ground, bodyB: dynamicBox(0, 4), length: 2),
          )
          ..length = 3
          ..springEnabled = true
          ..springHertz = 2.5
          ..springDampingRatio = 0.3
          ..limitEnabled = true
          ..motorEnabled = true
          ..motorSpeed = 0.5
          ..maxMotorForce = 12
          ..setLengthRange(min: 1, max: 5);
    expect(joint.length, closeTo(3, 1e-6));
    expect(joint.springEnabled, isTrue);
    expect(joint.springHertz, closeTo(2.5, 1e-6));
    expect(joint.springDampingRatio, closeTo(0.3, 1e-6));
    expect(joint.limitEnabled, isTrue);
    expect(joint.minLength, closeTo(1, 1e-6));
    expect(joint.maxLength, closeTo(5, 1e-6));
    expect(joint.motorEnabled, isTrue);
    expect(joint.motorSpeed, closeTo(0.5, 1e-6));
    expect(joint.maxMotorForce, closeTo(12, 1e-6));
    world.step(1 / 60);
    expect(joint.currentLength, greaterThan(0));
    expect(joint.motorForce.isFinite, isTrue);
  });

  test('MotorJoint accessors round-trip', () {
    final joint =
        world.createMotorJoint(
            MotorJointDef(bodyA: ground, bodyB: dynamicBox(0, 3)),
          )
          ..linearOffset = Vector2(1, 2)
          ..angularOffset = 0.5
          ..maxForce = 40
          ..maxTorque = 20
          ..correctionFactor = 0.7;
    expect(joint.linearOffset.x, closeTo(1, 1e-6));
    expect(joint.linearOffset.y, closeTo(2, 1e-6));
    expect(joint.angularOffset, closeTo(0.5, 1e-6));
    expect(joint.maxForce, 40);
    expect(joint.maxTorque, 20);
    expect(joint.correctionFactor, closeTo(0.7, 1e-6));
  });

  test('MouseJoint accessors round-trip', () {
    final joint =
        world.createMouseJoint(
            MouseJointDef(bodyA: ground, bodyB: dynamicBox(0, 3)),
          )
          ..target = Vector2(3, 4)
          ..springHertz = 6
          ..springDampingRatio = 0.9
          ..maxForce = 250;
    expect(joint.target, Vector2(3, 4));
    expect(joint.springHertz, closeTo(6, 1e-6));
    expect(joint.springDampingRatio, closeTo(0.9, 1e-6));
    expect(joint.maxForce, 250);
  });

  test('PrismaticJoint accessors round-trip', () {
    final joint =
        world.createPrismaticJoint(
            PrismaticJointDef(
              bodyA: ground,
              bodyB: dynamicBox(0, 3),
              localAnchorA: Vector2(0, 4),
              localAxisA: Vector2(0, 1),
            ),
          )
          ..springEnabled = true
          ..springHertz = 3
          ..springDampingRatio = 0.4
          ..targetTranslation = 0.5
          ..limitEnabled = true
          ..motorEnabled = true
          ..motorSpeed = 1.5
          ..maxMotorForce = 60
          ..setLimits(lower: -2, upper: 2);
    expect(joint.springEnabled, isTrue);
    expect(joint.springHertz, closeTo(3, 1e-6));
    expect(joint.springDampingRatio, closeTo(0.4, 1e-6));
    expect(joint.targetTranslation, closeTo(0.5, 1e-6));
    expect(joint.limitEnabled, isTrue);
    expect(joint.lowerLimit, closeTo(-2, 1e-6));
    expect(joint.upperLimit, closeTo(2, 1e-6));
    expect(joint.motorEnabled, isTrue);
    expect(joint.motorSpeed, closeTo(1.5, 1e-6));
    expect(joint.maxMotorForce, closeTo(60, 1e-6));
    world.step(1 / 60);
    expect(joint.translation.isFinite, isTrue);
    expect(joint.speed.isFinite, isTrue);
    expect(joint.motorForce.isFinite, isTrue);
  });

  test('RevoluteJoint accessors round-trip', () {
    final joint =
        world.createRevoluteJoint(
            RevoluteJointDef(
              bodyA: ground,
              bodyB: dynamicBox(0, 3),
              localAnchorA: Vector2(0, 4),
            ),
          )
          ..springEnabled = true
          ..springHertz = 2
          ..springDampingRatio = 0.6
          ..targetAngle = 0.25
          ..limitEnabled = true
          ..motorEnabled = true
          ..motorSpeed = 2
          ..maxMotorTorque = 15
          ..setLimits(lower: -math.pi / 4, upper: math.pi / 4);
    expect(joint.springEnabled, isTrue);
    expect(joint.springHertz, closeTo(2, 1e-6));
    expect(joint.springDampingRatio, closeTo(0.6, 1e-6));
    expect(joint.targetAngle, closeTo(0.25, 1e-6));
    expect(joint.limitEnabled, isTrue);
    expect(joint.lowerLimit, closeTo(-math.pi / 4, 1e-6));
    expect(joint.upperLimit, closeTo(math.pi / 4, 1e-6));
    expect(joint.motorEnabled, isTrue);
    expect(joint.motorSpeed, closeTo(2, 1e-6));
    expect(joint.maxMotorTorque, closeTo(15, 1e-6));
    world.step(1 / 60);
    expect(joint.angle.isFinite, isTrue);
    expect(joint.motorTorque.isFinite, isTrue);
  });

  test('WeldJoint accessors round-trip', () {
    final joint =
        world.createWeldJoint(
            WeldJointDef(bodyA: ground, bodyB: dynamicBox(0, 3)),
          )
          ..linearHertz = 1.5
          ..angularHertz = 2.5
          ..linearDampingRatio = 0.2
          ..angularDampingRatio = 0.8;
    expect(joint.linearHertz, closeTo(1.5, 1e-6));
    expect(joint.angularHertz, closeTo(2.5, 1e-6));
    expect(joint.linearDampingRatio, closeTo(0.2, 1e-6));
    expect(joint.angularDampingRatio, closeTo(0.8, 1e-6));
  });

  test('WheelJoint accessors round-trip', () {
    final joint =
        world.createWheelJoint(
            WheelJointDef(bodyA: ground, bodyB: dynamicBox(0, 3)),
          )
          ..springEnabled = false
          ..springHertz = 5
          ..springDampingRatio = 0.1
          ..limitEnabled = true
          ..motorEnabled = true
          ..motorSpeed = 3
          ..maxMotorTorque = 25
          ..setLimits(lower: -1, upper: 1);
    expect(joint.springEnabled, isFalse);
    expect(joint.springHertz, closeTo(5, 1e-6));
    expect(joint.springDampingRatio, closeTo(0.1, 1e-6));
    expect(joint.limitEnabled, isTrue);
    expect(joint.lowerLimit, closeTo(-1, 1e-6));
    expect(joint.upperLimit, closeTo(1, 1e-6));
    expect(joint.motorEnabled, isTrue);
    expect(joint.motorSpeed, closeTo(3, 1e-6));
    expect(joint.maxMotorTorque, closeTo(25, 1e-6));
    world.step(1 / 60);
    expect(joint.motorTorque.isFinite, isTrue);
  });

  test('the DebugDraw defaults are callable no-ops', () {
    dynamicBox(0, 2);
    world.createRevoluteJoint(
      RevoluteJointDef(bodyA: ground, bodyB: dynamicBox(3, 2)),
    );
    // A bare subclass with every toggle on: the default empty draw methods
    // must all be safely callable.
    final draw = _BareDebugDraw()
      ..drawJointExtras = true
      ..drawBounds = true
      ..drawMass = true
      ..drawBodyNames = true
      ..drawContacts = true
      ..drawGraphColors = true
      ..drawContactNormals = true
      ..drawContactImpulses = true
      ..drawContactFeatures = true
      ..drawFrictionImpulses = true
      ..drawIslands = true;
    world
      ..step(1 / 60)
      ..draw(draw);
  });

  test('math conveniences', () {
    expect(const Rot.identity(), Rot.fromAngle(0));
    expect({const Rot.identity(), Rot.fromAngle(0)}, hasLength(1));
    expect(const Rot.identity().toString(), contains('Rot'));
    expect(Transform.identity().toString(), contains('Transform'));
    expect(Aabb(Vector2.zero(), Vector2(1, 1)).toString(), contains('Aabb'));
    final rotated = Rot.fromAngle(math.pi / 2).inverseRotate(Vector2(0, 1));
    expect(rotated.x, closeTo(1, 1e-9));
  });
}

class _BareDebugDraw extends DebugDraw {}
