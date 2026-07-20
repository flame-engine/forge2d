import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  setUpAll(initializeForge2D);

  late World world;

  setUp(() => world = World());
  tearDown(() {
    if (world.isValid) {
      world.destroy();
    }
  });

  Body ground() =>
      world.createBody(BodyDef(position: Vector2(0, -1)))
        ..createShape(Polygon.box(50, 1), ShapeDef(enablePreSolveEvents: true));

  test('destroying a body from a pre-solve callback is deferred', () {
    ground();
    final ball = world.createBody(
      BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)),
    )..createShape(Circle(radius: 0.5), ShapeDef(enablePreSolveEvents: true));

    var sawContact = false;
    world.preSolveCallback = (shapeA, shapeB, normal) {
      sawContact = true;
      // Destroying mid-step must not abort or corrupt; it is deferred.
      ball.destroy();
      expect(ball.isValid, isTrue, reason: 'still valid inside the step');
      return true;
    };

    for (var i = 0; i < 120 && !sawContact; i++) {
      world.step(1 / 60);
    }
    expect(sawContact, isTrue);
    expect(ball.isValid, isFalse, reason: 'destroyed after the step ended');
  });

  test('a deferred destroy runs only once even if requested twice', () {
    ground();
    final ball = world.createBody(
      BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)),
    )..createShape(Circle(radius: 0.5), ShapeDef(enablePreSolveEvents: true));

    var calls = 0;
    world.preSolveCallback = (shapeA, shapeB, normal) {
      calls++;
      ball
        ..destroy()
        ..destroy();
      return true;
    };
    for (var i = 0; i < 120 && calls == 0; i++) {
      world.step(1 / 60);
    }
    expect(ball.isValid, isFalse);
    // A second step must not blow up on the already-destroyed body.
    world.step(1 / 60);
  });

  test('creating a body from a pre-solve callback throws a StateError', () {
    ground();
    world
        .createBody(BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)))
        .createShape(
          Circle(radius: 0.5),
          ShapeDef(enablePreSolveEvents: true),
        );

    Object? caught;
    world.preSolveCallback = (shapeA, shapeB, normal) {
      try {
        world.createBody();
      } on Object catch (error) {
        caught = error;
      }
      return true;
    };
    for (var i = 0; i < 120 && caught == null; i++) {
      world.step(1 / 60);
    }
    expect(caught, isA<StateError>());
  });

  test('destroying a world clears its simulation callbacks', () {
    // A world whose filter callback rejects every collision.
    ground();
    world.customFilterCallback = (shapeA, shapeB) => false;
    world.destroy();

    // A new world likely reuses the same slot; its collisions must not be
    // filtered by the dead world's callback.
    world = World();
    ground();
    final ball = world.createBody(
      BodyDef(type: BodyType.dynamic, position: Vector2(0, 2)),
    )..createShape(Circle(radius: 0.5));
    for (var i = 0; i < 120; i++) {
      world.step(1 / 60);
    }
    expect(
      ball.position.y,
      greaterThan(0),
      reason: "ball must land, not fall through the dead world's filter",
    );
  });

  test('destroying a body releases the user data of its chains', () {
    final carrier = world.createBody();
    carrier.createChain(
      ChainDef(
        points: [Vector2(-6, 0), Vector2(-2, 0), Vector2(2, 0), Vector2(6, 0)],
        userData: 'terrain',
      ),
    );
    expect(world.chainUserData, hasLength(1));
    carrier.destroy();
    expect(world.chainUserData, isEmpty);
    expect(world.chainOwners, isEmpty);
  });

  test('long body names are safe and truncated to 31 bytes', () {
    // The native library caps names at 31 bytes; the point of the long
    // input is that the web backend must not overflow its string buffer.
    final name = 'domino ${'x' * 5000}';
    final body = world.createBody(BodyDef(name: name));
    expect(body.name, name.substring(0, 31));
  });
}
