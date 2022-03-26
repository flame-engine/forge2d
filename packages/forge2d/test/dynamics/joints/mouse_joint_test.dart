import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('MouseJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = MouseJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(MouseJoint(jointDef), isA<MouseJoint>());
    });
  });
}
