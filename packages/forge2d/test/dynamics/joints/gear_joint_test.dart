import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('GearJoint', () {
    group('can be instantiated', () {
      test('with two RevoluteJoints', () {
        final world = World();

        final joint1 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final joint2 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final jointDef = GearJointDef()
          ..bodyA = Body(BodyDef(), world)
          ..bodyB = Body(BodyDef(), world)
          ..joint1 = joint1
          ..joint2 = joint2;

        expect(GearJoint(jointDef), isA<GearJoint>());
      });

      test('with two PrismaticJoints', () {
        final world = World();

        final joint1 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final joint2 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final jointDef = GearJointDef()
          ..bodyA = Body(BodyDef(), world)
          ..bodyB = Body(BodyDef(), world)
          ..joint1 = joint1
          ..joint2 = joint2;

        expect(GearJoint(jointDef), isA<GearJoint>());
      });

      test('with one RevoluteJoint and another PrismaticJoint', () {
        final world = World();

        final joint1 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final joint2 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final jointDef = GearJointDef()
          ..bodyA = Body(BodyDef(), world)
          ..bodyB = Body(BodyDef(), world)
          ..joint1 = joint1
          ..joint2 = joint2;

        expect(GearJoint(jointDef), isA<GearJoint>());
      });

      test('with one PrismaticJoint and another RevoluteJoint', () {
        final world = World();

        final joint1 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final joint2 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        final jointDef = GearJointDef()
          ..bodyA = Body(BodyDef(), world)
          ..bodyB = Body(BodyDef(), world)
          ..joint1 = joint1
          ..joint2 = joint2;

        expect(GearJoint(jointDef), isA<GearJoint>());
      });
    });
  });
}
