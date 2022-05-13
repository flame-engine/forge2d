import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

import '../../helpers/helpers.dart';

void main() {
  group('RevoluteJoint', () {
    late RevoluteJointDef jointDef;

    setUp(() {
      final world = World();
      jointDef = RevoluteJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);
    });

    test('can be instantiated', () {
      expect(RevoluteJoint(jointDef), isA<RevoluteJoint>());
    });

    test('can change motor speed', () {
      final joint = RevoluteJoint(jointDef);

      final oldMotorSpeed = joint.motorSpeed;
      final newMotorSpeed = oldMotorSpeed + 1;
      joint.motorSpeed = newMotorSpeed;

      expect(joint.motorSpeed, equals(newMotorSpeed));
    });

    group('render', () {
      late DebugDraw debugDraw;

      setUp(() {
        debugDraw = MockDebugDraw();

        registerFallbackValue(Vector2.zero());
        registerFallbackValue(Color3i.black);
      });

      test('draws three segments', () {
        final joint = RevoluteJoint(jointDef);
        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });
    });
  });
}
