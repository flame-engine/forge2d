import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  late World world;

  setUp(() => world = World());
  tearDown(() => world.destroy());

  group('Body', () {
    test('reflects the definition it was created from', () {
      final body = world.createBody(
        BodyDef(
          type: BodyType.dynamic,
          position: Vector2(1, 2),
          rotation: Rot.fromAngle(math.pi / 4),
          linearVelocity: Vector2(3, 4),
          angularVelocity: 0.5,
          linearDamping: 0.1,
          angularDamping: 0.2,
          gravityScale: 2,
          sleepThreshold: 0.1,
          name: 'hero',
          userData: 'payload',
          fixedRotation: true,
          isBullet: true,
        ),
      );

      expect(body.isValid, isTrue);
      expect(body.type, BodyType.dynamic);
      expect(body.position.x, closeTo(1, 1e-6));
      expect(body.position.y, closeTo(2, 1e-6));
      expect(body.angle, closeTo(math.pi / 4, 1e-6));
      expect(body.linearVelocity.x, closeTo(3, 1e-6));
      expect(body.linearVelocity.y, closeTo(4, 1e-6));
      expect(body.angularVelocity, closeTo(0.5, 1e-6));
      expect(body.linearDamping, closeTo(0.1, 1e-6));
      expect(body.angularDamping, closeTo(0.2, 1e-6));
      expect(body.gravityScale, 2);
      expect(body.sleepThreshold, closeTo(0.1, 1e-6));
      expect(body.name, 'hero');
      expect(body.userData, 'payload');
      expect(body.fixedRotation, isTrue);
      expect(body.isBullet, isTrue);
    });

    test('defaults match the native body defaults', () {
      final body = world.createBody();
      expect(body.type, BodyType.static);
      expect(body.position, Vector2.zero());
      expect(body.angle, 0);
      expect(body.gravityScale, 1);
      expect(body.sleepThreshold, closeTo(0.05, 1e-6));
      expect(body.name, isEmpty);
      expect(body.userData, isNull);
      // Static bodies are never part of the awake set.
      expect(body.isAwake, isFalse);
      expect(body.isEnabled, isTrue);
      expect(body.sleepEnabled, isTrue);
      expect(body.fixedRotation, isFalse);
      expect(body.isBullet, isFalse);
    });

    test('handles to the same body compare equal', () {
      final body = world.createBody(BodyDef(userData: 'x'));
      final shape = body.createShape(Circle(radius: 1));
      final other = shape.body;
      expect(other, body);
      expect(other.userData, 'x');
    });

    test('destroy removes the body and its user data', () {
      final body = world.createBody(BodyDef(userData: 'gone'));
      final shape = body.createShape(
        Circle(radius: 1),
        ShapeDef(userData: 'shape data'),
      );
      body.destroy();
      expect(body.isValid, isFalse);
      expect(shape.isValid, isFalse);
      expect(world.bodyUserData, isEmpty);
      expect(world.shapeUserData, isEmpty);
    });

    test('a dynamic body with a shape has mass', () {
      final body = world.createBody(BodyDef(type: BodyType.dynamic))
        ..createShape(Polygon.square(0.5), ShapeDef(density: 2));
      // A 1x1 box with density 2 weighs 2 kg.
      expect(body.mass, closeTo(2, 1e-6));
      expect(body.rotationalInertia, greaterThan(0));
    });

    test('mass data can be overridden and recomputed', () {
      final body = world.createBody(BodyDef(type: BodyType.dynamic))
        ..createShape(Polygon.square(0.5));
      final original = body.mass;

      body.massData = MassData(
        mass: 42,
        center: Vector2(0.1, 0),
        rotationalInertia: 3,
      );
      expect(body.mass, closeTo(42, 1e-4));
      expect(body.localCenterOfMass.x, closeTo(0.1, 1e-6));

      body.applyMassFromShapes();
      expect(body.mass, closeTo(original, 1e-6));
    });

    test('impulses change velocity immediately', () {
      final body = world.createBody(BodyDef(type: BodyType.dynamic))
        ..createShape(Polygon.square(0.5));
      body.applyLinearImpulse(Vector2(body.mass, 0));
      expect(body.linearVelocity.x, closeTo(1, 1e-5));

      body.applyAngularImpulse(body.rotationalInertia);
      expect(body.angularVelocity, closeTo(1, 1e-5));
    });

    test('forces accelerate a body over a step', () {
      world.gravity = Vector2.zero();
      final body = world.createBody(BodyDef(type: BodyType.dynamic))
        ..createShape(Polygon.square(0.5));
      body.applyForce(Vector2(body.mass * 60, 0));
      world.step(1 / 60);
      expect(body.linearVelocity.x, closeTo(1, 0.01));
    });

    test('setTransform teleports the body', () {
      final body = world.createBody();
      body.setTransform(Vector2(5, 6), Rot.fromAngle(1));
      expect(body.position.x, closeTo(5, 1e-6));
      expect(body.position.y, closeTo(6, 1e-6));
      expect(body.angle, closeTo(1, 1e-6));
    });

    test('world and local point conversions are inverses', () {
      final body = world.createBody(
        BodyDef(position: Vector2(2, 3), rotation: Rot.fromAngle(0.7)),
      );
      final worldPoint = body.worldPoint(Vector2(1, 1));
      final localPoint = body.localPoint(worldPoint);
      expect(localPoint.x, closeTo(1, 1e-5));
      expect(localPoint.y, closeTo(1, 1e-5));
    });

    test('type changes wake the body and recompute mass', () {
      final body = world.createBody()..createShape(Polygon.square(0.5));
      expect(body.mass, 0);
      body.type = BodyType.dynamic;
      expect(body.type, BodyType.dynamic);
      expect(body.mass, greaterThan(0));
    });

    test('disable and enable round-trip', () {
      final body = world.createBody(BodyDef(type: BodyType.dynamic));
      expect(body.isEnabled, isTrue);
      body.isEnabled = false;
      expect(body.isEnabled, isFalse);
      body.isEnabled = true;
      expect(body.isEnabled, isTrue);
    });

    test('shapes lists all attached shapes', () {
      final body = world.createBody();
      final first = body.createShape(Circle(radius: 1));
      final second = body.createShape(Polygon.square(1));
      expect(body.shapes.toSet(), {first, second});
    });
  });
}
