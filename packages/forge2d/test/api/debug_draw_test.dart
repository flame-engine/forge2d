import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

/// Records every draw call for assertions.
class RecordingDebugDraw extends DebugDraw {
  final solidPolygons = <List<Vector2>>[];
  final solidCircles = <Transform>[];
  final solidCapsules = <(Vector2, Vector2, double)>[];
  final segments = <(Vector2, Vector2)>[];
  final points = <Vector2>[];
  final strings = <String>[];

  @override
  void drawSolidPolygon(
    Transform transform,
    List<Vector2> vertices,
    double radius,
    int color,
  ) => solidPolygons.add(vertices);

  @override
  void drawSolidCircle(Transform transform, double radius, int color) =>
      solidCircles.add(transform);

  @override
  void drawSolidCapsule(
    Vector2 point1,
    Vector2 point2,
    double radius,
    int color,
  ) => solidCapsules.add((point1, point2, radius));

  @override
  void drawSegment(Vector2 point1, Vector2 point2, int color) =>
      segments.add((point1, point2));

  @override
  void drawPoint(Vector2 point, double size, int color) => points.add(point);

  @override
  void drawString(Vector2 point, String text, int color) => strings.add(text);
}

void main() {
  setUpAll(initializeForge2D);

  late World world;

  setUp(() => world = World());
  tearDown(() => world.destroy());

  test('shapes are drawn with the matching primitives', () {
    world.createBody(BodyDef(position: Vector2(1, 2)))
      ..createShape(Polygon.square(0.5))
      ..createShape(Circle(radius: 0.5))
      ..createShape(
        Capsule(center1: Vector2(0, 0), center2: Vector2(0, 1), radius: 0.25),
      );

    final draw = RecordingDebugDraw();
    world.draw(draw);

    expect(draw.solidPolygons, hasLength(1));
    expect(draw.solidPolygons.single, hasLength(4));
    expect(draw.solidCircles, hasLength(1));
    expect(draw.solidCircles.single.position.x, closeTo(1, 1e-5));
    expect(draw.solidCapsules, hasLength(1));
    expect(draw.solidCapsules.single.$3, closeTo(0.25, 1e-6));
  });

  test('joints are drawn when enabled', () {
    final anchor = world.createBody(BodyDef(position: Vector2(0, 5)));
    final swinging = world.createBody(
      BodyDef(type: BodyType.dynamic, position: Vector2(2, 5)),
    )..createShape(Polygon.square(0.25));
    world.createDistanceJoint(
      DistanceJointDef(bodyA: anchor, bodyB: swinging, length: 2),
    );

    final draw = RecordingDebugDraw();
    world.draw(draw);
    expect(draw.segments, isNotEmpty);

    draw.segments.clear();
    draw.drawJoints = false;
    world.draw(draw);
    expect(draw.segments, isEmpty);
  });

  test('body names are drawn when enabled', () {
    world.createBody(BodyDef(name: 'labeled')).createShape(Circle(radius: 1));
    final draw = RecordingDebugDraw()..drawBodyNames = true;
    world.draw(draw);
    expect(draw.strings, contains('labeled'));
  });

  test('drawing bounds cull far away shapes', () {
    world
        .createBody(BodyDef(position: Vector2(100, 100)))
        .createShape(Polygon.square(0.5));
    final draw = RecordingDebugDraw()
      ..drawingBounds = Aabb(Vector2(-10, -10), Vector2(10, 10));
    world.draw(draw);
    expect(draw.solidPolygons, isEmpty);
  });
}
