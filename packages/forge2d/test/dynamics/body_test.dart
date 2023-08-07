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
          allowSleep: false,
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

      final timeSteps = [
        for (var i = 0; i < 10; ++i)
          0.2 * i / 10,
        0.2,
        0.201,
      ];
      for (final dt in timeSteps) {
        world.stepDt(dt);
        expect(
          body.linearVelocity.y,
          equals(velocity.y),
          reason: 'Velocity should be constant after a time step of $dt',
        );
      }
    });
  });
}
