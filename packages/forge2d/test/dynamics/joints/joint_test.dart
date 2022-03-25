import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

class UnknownJoint extends Joint {
  UnknownJoint(JointDef def) : super(def);

  @override
  void noSuchMethod(_) {}
}

class UnknownJointDef extends JointDef {
  // TODO(alestiago): Remove constructor once JointType is removed.
  UnknownJointDef() : super(JointType.revolute);
}

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
      world.createJoint<RevoluteJoint>(revoluteJointDef);

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
        world = World();
        bodyA = Body(BodyDef(), world);
        bodyB = Body(BodyDef(), world);
        bodyC = Body(BodyDef(), world);
      });

      test('creates ConstantVolumeJoint', () {
        final jointDef = ConstantVolumeJointDef()
          ..addBody(bodyA)
          ..addBody(bodyB)
          ..addBody(bodyC);

        expect(
          Joint.create(world, jointDef),
          isA<ConstantVolumeJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<DistanceJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<FrictionJoint>(),
        );
        expect(
          Joint.create<FrictionJoint>(world, jointDef),
          isA<FrictionJoint>(),
        );
      });

      test('creates GearJoint', () {
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

        expect(
          Joint.create(world, jointDef),
          isA<GearJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<MotorJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<MouseJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<PrismaticJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<PulleyJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<RevoluteJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<RopeJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<WeldJoint>(),
        );
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
          Joint.create(world, jointDef),
          isA<WheelJoint>(),
        );
        expect(
          Joint.create<WheelJoint>(world, jointDef),
          isA<WheelJoint>(),
        );
      });

      test(
        'throws TypeError when Joint type is not valid',
        () {
          expect(
            () => Joint.create<UnknownJoint>(world, UnknownJointDef()),
            throwsA(isA<ArgumentError>()),
          );
        },
      );

      test('throws TypeError when invalid JointDef is given', () {
        final jointDef = WheelJointDef()
          ..bodyA = bodyA
          ..bodyB = bodyB;
        expect(
          () => Joint.create<DistanceJoint>(world, jointDef),
          throwsA(isA<TypeError>()),
        );
      });
    });
  });
}
