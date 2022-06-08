import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

import '../../helpers/helpers.dart';

class UnknownJointDef extends JointDef {}

class UnknownJoint extends Joint {
  UnknownJoint(UnknownJointDef super.unknownJointDef);

  @override
  void noSuchMethod(_) {}
}

void main() {
  group('GearJoint', () {
    late World world;

    late GearJointDef revoluteRevoluteGearJointDef;
    late GearJointDef revolutePrismaticGearJointDef;
    late GearJointDef prismaticRevoluteGearJointDef;
    late GearJointDef prismaticPrismaticGearJointDef;

    setUp(() {
      world = World();
      revoluteRevoluteGearJointDef = GearJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world)
        ..joint1 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        )
        ..joint2 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
      revolutePrismaticGearJointDef = GearJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world)
        ..joint1 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        )
        ..joint2 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
      prismaticRevoluteGearJointDef = GearJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world)
        ..joint1 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        )
        ..joint2 = RevoluteJoint(
          RevoluteJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
      prismaticPrismaticGearJointDef = GearJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world)
        ..joint1 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        )
        ..joint2 = PrismaticJoint(
          PrismaticJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
    });

    group('constructor', () {
      group(
        'builds',
        () {
          test('when joint1 and joint2 are both RevoluteJoints', () {
            expect(GearJoint(revoluteRevoluteGearJointDef), isA<GearJoint>());
          });

          test('when joint1 is RevoluteJoint and joint2 is PrismaticJoint', () {
            expect(GearJoint(revolutePrismaticGearJointDef), isA<GearJoint>());
          });

          test('when joint1 is PrismaticJoint and joint2 is RevoluteJoint', () {
            expect(GearJoint(prismaticRevoluteGearJointDef), isA<GearJoint>());
          });

          test('when joint1 and joint2 are both PrismaticJoints', () {
            expect(GearJoint(revoluteRevoluteGearJointDef), isA<GearJoint>());
          });
        },
      );

      test(
        'throws AssertionError '
        'when joints are not RevoluteJoint and/or PrismaticJoint',
        () {
          // TODO(alestiago): Consider removing UnknownJoint and test for all
          // possible Joint subclasses combinations.
          final unknownJoint1 = UnknownJoint(
            UnknownJointDef()
              ..bodyA = Body(BodyDef(), world)
              ..bodyB = Body(BodyDef(), world),
          );
          final unknownJoint2 = UnknownJoint(
            UnknownJointDef()
              ..bodyA = Body(BodyDef(), world)
              ..bodyB = Body(BodyDef(), world),
          );
          final jointDef = GearJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world)
            ..joint1 = unknownJoint1
            ..joint2 = unknownJoint2;

          expect(() => GearJoint(jointDef), throwsA(isA<AssertionError>()));
        },
      );
    });

    group('initVelocityConstraints', () {
      late SolverData data;

      setUp(() {
        data = SolverData()
          ..step = TimeStep()
          ..positions = [Position()]
          ..velocities = [Velocity()];
      });

      group('returns normally', () {
        test('when joint1 and joint2 are both RevoluteJoints', () {
          expect(
            () => GearJoint(revoluteRevoluteGearJointDef)
                .initVelocityConstraints(data),
            returnsNormally,
          );
        });

        test('when joint1 is RevoluteJoint and joint2 is PrismaticJoint', () {
          expect(
            () => GearJoint(revolutePrismaticGearJointDef)
                .initVelocityConstraints(data),
            returnsNormally,
          );
        });

        test('when joint1 is PrismaticJoint and joint2 is RevoluteJoint', () {
          expect(
            () => GearJoint(prismaticRevoluteGearJointDef)
                .initVelocityConstraints(data),
            returnsNormally,
          );
        });

        test('when joint1 and joint2 are both PrismaticJoints', () {
          expect(
            () => GearJoint(prismaticPrismaticGearJointDef)
                .initVelocityConstraints(data),
            returnsNormally,
          );
        });
      });
    });

    group('solvePositionConstraints', () {
      late SolverData data;

      setUp(() {
        data = SolverData()
          ..step = TimeStep()
          ..positions = [Position()]
          ..velocities = [Velocity()];
      });

      group('returns normally', () {
        test('when joint1 and joint2 are both RevoluteJoints', () {
          expect(
            () => GearJoint(revoluteRevoluteGearJointDef)
                .solvePositionConstraints(data),
            returnsNormally,
          );
        });

        test('when joint1 is RevoluteJoint and joint2 is PrismaticJoint', () {
          expect(
            () => GearJoint(revolutePrismaticGearJointDef)
                .solvePositionConstraints(data),
            returnsNormally,
          );
        });

        test('when joint1 is PrismaticJoint and joint2 is RevoluteJoint', () {
          expect(
            () => GearJoint(prismaticRevoluteGearJointDef)
                .solvePositionConstraints(data),
            returnsNormally,
          );
        });

        test('when joint1 and joint2 are both PrismaticJoints', () {
          expect(
            () => GearJoint(prismaticPrismaticGearJointDef)
                .solvePositionConstraints(data),
            returnsNormally,
          );
        });
      });
    });

    group('render', () {
      late DebugDraw debugDraw;

      setUp(() {
        debugDraw = MockDebugDraw();

        registerFallbackValue(Vector2.zero());
        registerFallbackValue(Color3i.black);
      });

      test(
          'draws three segments '
          'when joint1 and joint2 are both RevoluteJoints', () {
        final joint = GearJoint(revoluteRevoluteGearJointDef);
        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });

      test(
          'draws three segments '
          'when joint1 is RevoluteJoint and joint2 is PrismaticJoint', () {
        final joint = GearJoint(revolutePrismaticGearJointDef);
        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });

      test(
          'draws three segments '
          'when joint1 is PrismaticJoint and joint2 is RevoluteJoint', () {
        final joint = GearJoint(prismaticRevoluteGearJointDef);
        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });

      test(
          'draws three segments '
          'when joint1 and joint2 are both PrismaticJoints', () {
        final joint = GearJoint(prismaticPrismaticGearJointDef);
        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });
    });
  });
}
