import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  late World world;

  setUp(() => world = World());
  tearDown(() => world.destroy());

  Body ground() =>
      world.createBody(BodyDef(position: Vector2(0, -1)))
        ..createShape(Polygon.box(50, 1), ShapeDef(enableContactEvents: true));

  group('contact events', () {
    test('begin and end events fire for a bouncing contact', () {
      final groundBody = ground();
      final ball = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)),
      );
      final ballShape = ball.createShape(
        Circle(radius: 0.5),
        ShapeDef(
          enableContactEvents: true,
          material: SurfaceMaterial(restitution: 0.8),
        ),
      );

      final began = <ContactBeginEvent>[];
      final ended = <ContactEndEvent>[];
      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
        final events = world.contactEvents;
        began.addAll(events.begin);
        ended.addAll(events.end);
      }

      expect(began, isNotEmpty);
      final touching = {began.first.shapeA, began.first.shapeB};
      expect(touching, contains(ballShape));
      expect(touching, contains(groundBody.shapes.single));
      expect(began.first.points, isNotEmpty);
      expect(began.first.normal.y.abs(), closeTo(1, 0.01));
      // The bouncy ball leaves the ground again, producing an end event.
      expect(ended, isNotEmpty);
    });

    test('hit events report the approach speed', () {
      ground();
      world
          .createBody(BodyDef(type: BodyType.dynamic, position: Vector2(0, 5)))
          .createShape(
            Circle(radius: 0.5),
            ShapeDef(enableContactEvents: true, enableHitEvents: true),
          );

      final hits = <ContactHitEvent>[];
      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
        hits.addAll(world.contactEvents.hit);
      }

      expect(hits, isNotEmpty);
      // Dropped from 5 meters, the impact speed is above the default 1 m/s
      // threshold.
      expect(hits.first.approachSpeed, greaterThan(1));
    });

    test('no events without enableContactEvents', () {
      world
          .createBody(BodyDef(position: Vector2(0, -1)))
          .createShape(Polygon.box(50, 1));
      world
          .createBody(BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)))
          .createShape(Circle(radius: 0.5));

      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
        expect(world.contactEvents.begin, isEmpty);
      }
    });
  });

  group('sensor events', () {
    test('a body falling through a sensor produces begin and end', () {
      final sensorShape = world
          .createBody(BodyDef(position: Vector2(0, 3)))
          .createShape(
            Polygon.box(2, 0.25),
            // A sensor needs enableSensorEvents on itself as well.
            ShapeDef(isSensor: true, enableSensorEvents: true),
          );
      final visitorShape = world
          .createBody(
            BodyDef(type: BodyType.dynamic, position: Vector2(0, 6)),
          )
          .createShape(
            Circle(radius: 0.3),
            ShapeDef(enableSensorEvents: true),
          );

      final began = <SensorEvent>[];
      final ended = <SensorEvent>[];
      for (var i = 0; i < 180; i++) {
        world.step(1 / 60);
        final events = world.sensorEvents;
        began.addAll(events.begin);
        ended.addAll(events.end);
      }

      expect(began, hasLength(1));
      expect(began.single.sensor, sensorShape);
      expect(began.single.visitor, visitorShape);
      expect(ended, hasLength(1));
    });
  });

  group('body move events', () {
    test('only moving bodies are reported', () {
      ground();
      final faller = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(10, 5)),
      )..createShape(Circle(radius: 0.5));

      world.step(1 / 60);
      final events = world.bodyMoveEvents;
      expect(events.map((event) => event.body), contains(faller));
      final event = events.singleWhere((event) => event.body == faller);
      expect(event.transform.position.y, lessThan(5));
      expect(event.fellAsleep, isFalse);
    });

    test('fellAsleep is reported when a body comes to rest', () {
      ground();
      world
          .createBody(
            BodyDef(type: BodyType.dynamic, position: Vector2(0, 0.6)),
          )
          .createShape(Polygon.square(0.5));

      var sleepReported = false;
      for (var i = 0; i < 300 && !sleepReported; i++) {
        world.step(1 / 60);
        sleepReported = world.bodyMoveEvents.any((event) => event.fellAsleep);
      }
      expect(sleepReported, isTrue);
    });
  });

  group('queries', () {
    test('castRayClosest finds the nearest shape', () {
      ground();
      final near = world.createBody(BodyDef(position: Vector2(2, 0.5)))
        ..createShape(Polygon.square(0.5));
      world
          .createBody(BodyDef(position: Vector2(4, 0.5)))
          .createShape(Polygon.square(0.5));

      final hit = world.castRayClosest(Vector2(0, 0.5), Vector2(10, 0));
      expect(hit, isNotNull);
      expect(hit!.shape.body, near);
      expect(hit.point.x, closeTo(1.5, 0.01));
      expect(hit.normal.x, closeTo(-1, 0.01));
      expect(hit.fraction, closeTo(0.15, 0.01));
    });

    test('castRayClosest returns null on a miss', () {
      expect(world.castRayClosest(Vector2(0, 10), Vector2(1, 0)), isNull);
    });

    test('castRayAll returns hits sorted by fraction', () {
      final first = world.createBody(BodyDef(position: Vector2(2, 0)))
        ..createShape(Circle(radius: 0.5));
      final second = world.createBody(BodyDef(position: Vector2(5, 0)))
        ..createShape(Circle(radius: 0.5));

      final hits = world.castRayAll(Vector2(0, 0), Vector2(10, 0));
      expect(hits, hasLength(2));
      expect(hits.first.shape.body, first);
      expect(hits.last.shape.body, second);
      expect(hits.first.fraction, lessThan(hits.last.fraction));
    });

    test('castRay callback can stop the cast early', () {
      world
          .createBody(BodyDef(position: Vector2(2, 0)))
          .createShape(Circle(radius: 0.5));
      world
          .createBody(BodyDef(position: Vector2(5, 0)))
          .createShape(Circle(radius: 0.5));

      var calls = 0;
      world.castRay(Vector2(0, 0), Vector2(10, 0), (hit) {
        calls++;
        return 0;
      });
      expect(calls, 1);
    });

    test('query filters exclude non-matching categories', () {
      world
          .createBody(BodyDef(position: Vector2(2, 0)))
          .createShape(
            Circle(radius: 0.5),
            ShapeDef(filter: Filter(categoryBits: 2)),
          );

      expect(
        world.castRayClosest(
          Vector2(0, 0),
          Vector2(10, 0),
          filter: QueryFilter(maskBits: 4),
        ),
        isNull,
      );
      expect(
        world.castRayClosest(
          Vector2(0, 0),
          Vector2(10, 0),
          filter: QueryFilter(maskBits: 2),
        ),
        isNotNull,
      );
    });

    test('overlapAabb returns shapes in the box', () {
      final inside = world
          .createBody(BodyDef(position: Vector2(0, 0)))
          .createShape(Circle(radius: 0.5));
      world
          .createBody(BodyDef(position: Vector2(20, 0)))
          .createShape(Circle(radius: 0.5));

      final shapes = world.overlapAabb(
        Aabb(Vector2(-2, -2), Vector2(2, 2)),
      );
      expect(shapes, [inside]);
    });
  });

  group('explosions', () {
    test('an explosion pushes nearby bodies away', () {
      world.gravity = Vector2.zero();
      final body = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(1, 0)),
      )..createShape(Circle(radius: 0.5));

      world.explode(
        ExplosionDef(
          position: Vector2(0, 0),
          radius: 3,
          impulsePerLength: 5,
        ),
      );
      world.step(1 / 60);
      expect(body.linearVelocity.x, greaterThan(0.1));
    });
  });

  group('simulation callbacks', () {
    test('the custom filter can disable a collision', () {
      ground();
      final ball = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(0, 3)),
      )..createShape(Circle(radius: 0.5), ShapeDef(userData: 'ghost'));

      world.customFilterCallback = (shapeA, shapeB) =>
          shapeA.userData != 'ghost' && shapeB.userData != 'ghost';

      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
      }
      // The ball ignored the ground and kept falling.
      expect(ball.position.y, lessThan(-1));

      world.customFilterCallback = null;
    });

    test('the pre-solve callback can disable a contact', () {
      ground();
      final ball =
          world.createBody(
            BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)),
          )..createShape(
            Circle(radius: 0.5),
            ShapeDef(enablePreSolveEvents: true),
          );

      var called = false;
      world.preSolveCallback = (shapeA, shapeB, normal) {
        called = true;
        return false;
      };

      for (var i = 0; i < 120; i++) {
        world.step(1 / 60);
      }
      expect(called, isTrue);
      // With every contact disabled the ball falls through the ground.
      expect(ball.position.y, lessThan(-1));

      world.preSolveCallback = null;
    });
  });
}
