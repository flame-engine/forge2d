import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('WheelJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = WheelJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(WheelJoint(jointDef), isA<WheelJoint>());
    });
  });
}
