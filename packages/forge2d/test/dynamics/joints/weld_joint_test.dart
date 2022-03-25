import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('WeldJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = WeldJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(WeldJoint(jointDef), isA<WeldJoint>());
    });
  });
}
