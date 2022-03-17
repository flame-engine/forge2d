import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

class MockWorld extends Mock implements World {}

class MockJointDef extends Mock implements JointDef {}

class MockJoint extends Mock implements Joint {}

class MockDistanceJoint extends Mock implements DistanceJoint {}

class UnknownJoint extends Joint {
  UnknownJoint(JointDef def) : super(def);

  @override
  void noSuchMethod(_) {}
}

class MockBody extends Mock implements Body {}

void main() {
  group('Joint', () {
    test('destruction of body with joint', () {
      final world = World(Vector2(0.0, -10.0));
      final bodyDef = BodyDef();
      final body1 = world.createBody(bodyDef);
      final body2 = world.createBody(bodyDef..position = Vector2.all(2));
      final shape = CircleShape()
        ..radius = 1.2
        ..position.setValues(10, 10);

      final fixtureDef = FixtureDef(shape)
        ..density = 50.0
        ..friction = .1
        ..restitution = .9;

      body1.createFixture(fixtureDef);
      body2.createFixture(fixtureDef);

      final revoluteJointDef = RevoluteJointDef()
        ..initialize(body1, body2, body1.position);
      world.createJoint(revoluteJointDef);

      expect(body1.joints.length, 1);
      expect(body2.joints.length, 1);
      world.destroyBody(body1);
      expect(body1.joints.length, 0);
      expect(body2.joints.length, 0);
    });

    group('create', () {
      late World world;
      late Body bodyA;
      late Body bodyB;
      late Body bodyC;

      setUp(() {
        world = MockWorld();
        bodyA = MockBody();
        bodyB = MockBody();
        bodyC = MockBody();
      });

      setUpAll(() {
        registerFallbackValue(Vector2.zero());
        registerFallbackValue(MockJointDef());
      });

      test('creates ConstantVolumeJoint', () {
        final jointDef = ConstantVolumeJointDef()
          ..addBody(bodyA)
          ..addBody(bodyB)
          ..addBody(bodyC);

        for (final body in jointDef.bodies) {
          when(() => body.worldCenter).thenReturn(Vector2.zero());
          when(() => body.localPoint(any())).thenReturn(
            Vector2.zero(),
          );
          when(() => world.createJoint(any())).thenReturn(
            MockDistanceJoint(),
          );
        }

        expect(
          Joint.create<ConstantVolumeJoint>(world, jointDef),
          isA<ConstantVolumeJoint>(),
        );
      });

      test('creates DistanceJoint', () {
        final jointDef = DistanceJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<DistanceJoint>(world, jointDef),
          isA<DistanceJoint>(),
        );
      });

      test('creates FrictionJoint', () {
        final jointDef = FrictionJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<FrictionJoint>(world, jointDef),
          isA<FrictionJoint>(),
        );
      });

      test('creates GearJoint', () {
        final jointDef = GearJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<GearJoint>(world, jointDef),
          isA<GearJoint>(),
        );
      });

      test('creates MotorJoint', () {
        final jointDef = MotorJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<MotorJoint>(world, jointDef),
          isA<MotorJoint>(),
        );
      });

      test('creates MouseJoint', () {
        final jointDef = MouseJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<MouseJoint>(world, jointDef),
          isA<MouseJoint>(),
        );
      });

      test('creates PrismaticJoint', () {
        final jointDef = PrismaticJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<PrismaticJoint>(world, jointDef),
          isA<PrismaticJoint>(),
        );
      });

      test('creates PulleyJoint', () {
        final jointDef = PulleyJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<PulleyJoint>(world, jointDef),
          isA<PulleyJoint>(),
        );
      });

      test('creates RevoluteJoint', () {
        final jointDef = RevoluteJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<RevoluteJoint>(world, jointDef),
          isA<RevoluteJoint>(),
        );
      });

      test('creates RopeJoint', () {
        final jointDef = RopeJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<RopeJoint>(world, jointDef),
          isA<RopeJoint>(),
        );
      });

      test('creates WeldJoint', () {
        final jointDef = WeldJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<WeldJoint>(world, jointDef),
          isA<WeldJoint>(),
        );
      });

      test('creates WheelJoint', () {
        final jointDef = WheelJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          Joint.create<WheelJoint>(world, jointDef),
          isA<WheelJoint>(),
        );
      });

      test(
        'throws ArguementError '
        'when Joint type is not defined',
        () {
          expect(
            () => Joint.create<UnknownJoint>(world, MockJointDef()),
            throwsArgumentError,
          );
        },
      );
    });
  });
}
