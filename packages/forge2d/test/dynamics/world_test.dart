import 'package:forge2d/forge2d.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

class AnotherRevoluteJoint extends RevoluteJoint {
  AnotherRevoluteJoint(RevoluteJointDef<Body, Body> def) : super(def);
}

void main() {
  group('gravity', () {
    test('can change', () {
      final world = World();

      final newGravity = world.gravity.clone()..x += 1;
      world.gravity = newGravity;

      expect(world.gravity, equals(newGravity));
    });
  });

  group(
    'createJoint',
    () {
      // TODO(alesitiago): Consider improving test and testing for all other
      // Joints.

      test('adds RevoluteJoint to joints', () {
        final world = World();
        expect(world.joints.isEmpty, isTrue);

        final joint = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        world.createJoint(joint);

        expect(world.joints.first, joint);
      });

      test('supports adding RevoluteJoint subclasses', () {
        final world = World();
        final joint = AnotherRevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        expect(() => world.createJoint(joint), returnsNormally);
      });
    },
  );
}
