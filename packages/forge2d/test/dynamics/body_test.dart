import 'package:forge2d/forge2d.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

void main() {
  group('Body', () {
    group('gravityScale', () {
      // TODO(alestiago): Make this tests pass.
      test('scales appropiately in x', () {
        final gravity = Vector2(10, 0);
        final world = World(gravity);

        final bodyA = world.createBody(BodyDef()..type = BodyType.dynamic);
        final bodyB = world.createBody(
          BodyDef()
            ..type = BodyType.dynamic
            ..gravityScale = Vector2(10, 1)
            ..position = Vector2(0, 10),
        );

        final bodyAPos = bodyA.position.clone();
        final bodyBPos = bodyB.position.clone();

        world.stepDt(1);

        expect(bodyAPos.x, isNot(equals(bodyA.position.x)));
        expect(bodyBPos.x, isNot(equals(bodyB.position.x)));

        expect(bodyB.position.x, greaterThan(bodyA.position.x));
      });

      test('scales appropiately in y', () {
        final gravity = Vector2(0, 10);
        final world = World(gravity);

        final bodyA = world.createBody(BodyDef()..type = BodyType.dynamic);
        final bodyB = world.createBody(
          BodyDef()
            ..type = BodyType.dynamic
            ..gravityScale = Vector2(1, 10)
            ..position = Vector2(10, 0),
        );

        final bodyAPos = bodyA.position.clone();
        final bodyBPos = bodyB.position.clone();

        world.stepDt(1);

        expect(bodyAPos.y, isNot(equals(bodyA.position.y)));
        expect(bodyBPos.y, isNot(equals(bodyB.position.y)));

        expect(bodyB.position.y, greaterThan(bodyA.position.y));
      });
    });
  });
}
