import 'package:forge2d/forge2d.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

void main() {
  group('Body', () {
    group('gravityOverride', () {
      test("body doesn't move when is zero", () {
        final gravity = Vector2(10, 10);
        final world = World(gravity);

        final body = world.createBody(
          BodyDef()
            ..type = BodyType.dynamic
            ..gravityOverride = Vector2.zero(),
        );

        final bodyInitialPosition = body.position.clone();

        world.stepDt(1);

        expect(bodyInitialPosition.x, equals(body.position.x));
        expect(bodyInitialPosition.y, equals(body.position.y));
      });

      test(
          "body moves with world's gravity "
          'when gravityOverride is not specfied', () {
        final gravity = Vector2(10, 10);
        final world = World(gravity);

        final body = world.createBody(BodyDef()..type = BodyType.dynamic);

        final bodyInitialPosition = body.position.clone();

        world.stepDt(1);

        expect(bodyInitialPosition.x, isNot(equals(body.position.x)));
        expect(bodyInitialPosition.y, isNot(equals(body.position.y)));
      });
    });

    group('gravityScale', () {
      test("body doesn't move when is zero", () {
        final gravity = Vector2(10, 10);
        final world = World(gravity);

        final body = world.createBody(
          BodyDef()
            ..type = BodyType.dynamic
            ..gravityScale = Vector2.zero(),
        );

        final bodyInitialPosition = body.position.clone();

        world.stepDt(1);

        expect(bodyInitialPosition.x, equals(body.position.x));
        expect(bodyInitialPosition.y, equals(body.position.y));
      });

      test(
          "body moves with world's gravity "
          'when gravityScale is not specfied', () {
        final gravity = Vector2(10, 10);
        final world = World(gravity);

        final body = world.createBody(BodyDef()..type = BodyType.dynamic);

        final bodyInitialPosition = body.position.clone();

        world.stepDt(1);

        expect(bodyInitialPosition.x, isNot(equals(body.position.x)));
        expect(bodyInitialPosition.y, isNot(equals(body.position.y)));
      });
    });

    test('constant velocity with BodyType.dynamic', () {
      final gravity = Vector2.zero();
      final world = World(gravity);

      final velocity = Vector2(0, 10);

      final body = world.createBody(
        BodyDef(
          type: BodyType.dynamic,
          linearVelocity: velocity.clone(),
        ),
      );

      expect(
        body.linearVelocity.y,
        equals(velocity.y),
        reason: 'Velocity should be as specified',
      );
      expect(
        body.linearDamping,
        equals(0.0),
        reason: 'No linear damping',
      );

      // Small time steps
      const fps = 30;
      const seconds = 100;
      for (var i = 0; i < fps * seconds; i++) {
        world.stepDt(1 / fps);
      }
      expect(
        body.linearVelocity.y,
        equals(velocity.y),
        reason: 'Velocity should be constant after small steps',
      );

      // Large time step
      world.stepDt(1);
      expect(
        body.linearVelocity.y,
        equals(velocity.y),
        reason: 'Velocity should be constant after large step',
      );
    });
  });
}
