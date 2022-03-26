import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('FrictionJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = FrictionJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(FrictionJoint(jointDef), isA<FrictionJoint>());
    });
  });
}
