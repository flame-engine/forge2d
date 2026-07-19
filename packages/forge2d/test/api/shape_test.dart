@TestOn('vm')
library;

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  late World world;
  late Body body;

  setUp(() {
    world = World();
    body = world.createBody(BodyDef(type: BodyType.dynamic));
  });
  tearDown(() => world.destroy());

  group('Shape', () {
    test('every geometry kind creates the matching shape type', () {
      expect(body.createShape(Circle(radius: 1)).type, ShapeType.circle);
      expect(
        body
            .createShape(
              Capsule(
                center1: Vector2(0, 0),
                center2: Vector2(0, 1),
                radius: 0.5,
              ),
            )
            .type,
        ShapeType.capsule,
      );
      expect(
        body
            .createShape(Segment(point1: Vector2(0, 0), point2: Vector2(1, 0)))
            .type,
        ShapeType.segment,
      );
      expect(body.createShape(Polygon.box(1, 2)).type, ShapeType.polygon);
      expect(
        body
            .createShape(
              Polygon([Vector2(0, 0), Vector2(1, 0), Vector2(0, 1)]),
            )
            .type,
        ShapeType.polygon,
      );
    });

    test('a degenerate polygon hull throws', () {
      expect(
        () => body.createShape(
          Polygon([Vector2(0, 0), Vector2(1, 0), Vector2(2, 0)]),
        ),
        throwsArgumentError,
      );
    });

    test('reflects the def it was created from', () {
      final shape = body.createShape(
        Circle(radius: 1),
        ShapeDef(
          density: 3,
          material: SurfaceMaterial(friction: 0.9, restitution: 0.4),
          filter: Filter(categoryBits: 2, maskBits: 6, groupIndex: -1),
          userData: 'ball',
          enableContactEvents: true,
        ),
      );

      expect(shape.isValid, isTrue);
      expect(shape.density, 3);
      expect(shape.friction, closeTo(0.9, 1e-6));
      expect(shape.restitution, closeTo(0.4, 1e-6));
      expect(shape.filter.categoryBits, 2);
      expect(shape.filter.maskBits, 6);
      expect(shape.filter.groupIndex, -1);
      expect(shape.userData, 'ball');
      expect(shape.contactEventsEnabled, isTrue);
      expect(shape.isSensor, isFalse);
      expect(shape.body, body);
    });

    test('defaults match the native shape defaults', () {
      final shape = body.createShape(Circle(radius: 1));
      expect(shape.density, 1);
      expect(shape.friction, closeTo(0.6, 1e-6));
      expect(shape.restitution, 0);
      expect(shape.filter.categoryBits, 1);
      expect(shape.filter.groupIndex, 0);
      expect(shape.contactEventsEnabled, isFalse);
      expect(shape.sensorEventsEnabled, isFalse);
      expect(shape.userData, isNull);
    });

    test('properties round-trip', () {
      final shape = body.createShape(Circle(radius: 1))
        ..friction = 0.25
        ..restitution = 0.75
        ..density = 5
        ..filter = Filter(categoryBits: 4, maskBits: 12)
        ..contactEventsEnabled = true
        ..hitEventsEnabled = true
        ..sensorEventsEnabled = true
        ..userData = 'updated';

      expect(shape.friction, closeTo(0.25, 1e-6));
      expect(shape.restitution, closeTo(0.75, 1e-6));
      expect(shape.density, 5);
      expect(shape.filter.categoryBits, 4);
      expect(shape.filter.maskBits, 12);
      expect(shape.contactEventsEnabled, isTrue);
      expect(shape.hitEventsEnabled, isTrue);
      expect(shape.sensorEventsEnabled, isTrue);
      expect(shape.userData, 'updated');
    });

    test('testPoint and aabb agree with the geometry', () {
      final shape = body.createShape(Polygon.square(1));
      expect(shape.testPoint(Vector2(0.5, 0.5)), isTrue);
      expect(shape.testPoint(Vector2(2, 2)), isFalse);

      final aabb = shape.aabb;
      expect(aabb.lowerBound.x, closeTo(-1, 0.1));
      expect(aabb.upperBound.y, closeTo(1, 0.1));
    });

    test('sensors detect but do not collide', () {
      final sensor = body.createShape(
        Circle(radius: 1),
        ShapeDef(isSensor: true),
      );
      expect(sensor.isSensor, isTrue);
    });

    test('destroy removes the shape from the body', () {
      final shape = body.createShape(Circle(radius: 1), ShapeDef(userData: 'x'))
        ..destroy();
      expect(shape.isValid, isFalse);
      expect(body.shapes, isEmpty);
      expect(world.shapeUserData, isEmpty);
    });
  });

  group('Chain', () {
    test('creates one segment shape per chain edge', () {
      final chain = body.createChain(
        ChainDef(
          points: [
            Vector2(-5, 0),
            Vector2(0, 1),
            Vector2(5, 0),
            Vector2(0, 5),
          ],
          isLoop: true,
          userData: 'terrain',
        ),
      );
      expect(chain.isValid, isTrue);
      expect(chain.segments, hasLength(4));
      expect(chain.segments.first.type, ShapeType.chainSegment);
      expect(chain.userData, 'terrain');
    });

    test('friction and restitution apply to all segments', () {
      final chain = body.createChain(
        ChainDef(
          points: [Vector2(-5, 0), Vector2(0, 0), Vector2(5, 0)],
          materials: [SurfaceMaterial(friction: 0.3, restitution: 0.2)],
        ),
      )..friction = 0.8;
      expect(chain.friction, closeTo(0.8, 1e-6));

      chain.restitution = 0.6;
      expect(chain.restitution, closeTo(0.6, 1e-6));
    });

    test('destroy removes the chain and its segments', () {
      final chain = body.createChain(
        ChainDef(points: [Vector2(-5, 0), Vector2(0, 0), Vector2(5, 0)]),
      )..destroy();
      expect(chain.isValid, isFalse);
    });
  });
}
