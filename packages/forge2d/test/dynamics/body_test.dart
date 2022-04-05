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
  });
}
