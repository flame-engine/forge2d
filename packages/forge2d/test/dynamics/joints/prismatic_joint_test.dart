import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('PrismaticJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = PrismaticJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(PrismaticJoint(jointDef), isA<PrismaticJoint>());
    });
  });
}
