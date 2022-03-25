import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('RopeJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = RopeJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(RopeJoint(jointDef), isA<RopeJoint>());
    });
  });
}
