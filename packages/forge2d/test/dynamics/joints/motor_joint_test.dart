import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('MotorJonit', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = MotorJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(MotorJoint(jointDef), isA<MotorJoint>());
    });
  });
}
