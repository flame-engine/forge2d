import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('PulleyJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = PulleyJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(PulleyJoint(jointDef), isA<PulleyJoint>());
    });
  });
}
