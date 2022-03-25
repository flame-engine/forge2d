import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('ConstantVolumeJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = ConstantVolumeJointDef()
        ..addBody(Body(BodyDef(), world))
        ..addBody(Body(BodyDef(), world))
        ..addBody(Body(BodyDef(), world));

      expect(
        ConstantVolumeJoint(
          world,
          jointDef,
        ),
        isA<ConstantVolumeJoint>(),
      );
    });
  });
}
