import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('DistanceJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = DistanceJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(DistanceJoint(jointDef), isA<DistanceJoint>());
    });
  });
}
