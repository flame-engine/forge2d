import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

class _MockBody extends Mock implements Body {}

class _MockDebugDraw extends Mock implements DebugDraw {}

void main() {
  group('WheelJoint', () {
    late WheelJointDef jointDef;

    setUp(() {
      jointDef = WheelJointDef()
        ..bodyA = _MockBody()
        ..bodyB = _MockBody();
    });

    test('can be instantiated', () {
      expect(WheelJoint(jointDef), isA<WheelJoint>());
    });

    group('motorSpeed', () {
      test('can change motor speed', () {
        final joint = WheelJoint(jointDef);

        final oldMotorSpeed = joint.motorSpeed;
        final newMotorSpeed = oldMotorSpeed + 1;
        joint.motorSpeed = newMotorSpeed;

        expect(joint.motorSpeed, equals(newMotorSpeed));
      });

      test('wakes up both bodies', () {
        final joint = WheelJoint(jointDef);
        joint.motorSpeed = 1;

        verify(() => joint.bodyA.setAwake(true)).called(1);
        verify(() => joint.bodyB.setAwake(true)).called(1);
      });
    });

    group('render', () {
      late DebugDraw debugDraw;

      setUp(() {
        debugDraw = _MockDebugDraw();

        registerFallbackValue(Vector2.zero());
        registerFallbackValue(Color3i.black);
      });

      test('draws three segments', () {
        final joint = WheelJoint(jointDef);
        when(() => joint.bodyA.transform).thenReturn(Transform.zero());
        when(() => joint.bodyB.transform).thenReturn(Transform.zero());
        when(() => joint.bodyA.worldPoint(any())).thenReturn(Vector2.zero());
        when(() => joint.bodyB.worldPoint(any())).thenReturn(Vector2.zero());

        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });
    });
  });
}
