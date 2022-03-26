import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('RevoluteJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = RevoluteJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(RevoluteJoint(jointDef), isA<RevoluteJoint>());
    });
  });
}
