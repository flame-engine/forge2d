import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

import '../../helpers/helpers.dart';

void main() {
  group('DistanceJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = DistanceJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(DistanceJoint(jointDef), isA<DistanceJoint>());
    });

    group('render', () {
      late World world;
      late DebugDraw debugDraw;

      setUp(() {
        world = World();
        debugDraw = MockDebugDraw();

        registerFallbackValue(Vector2.zero());
        registerFallbackValue(Color3i.black());
      });

      test('draws a single segment', () {
        final joint = DistanceJoint(
          DistanceJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        joint.render(debugDraw);

        verify(() => debugDraw.drawSegment(any(), any(), any())).called(1);
      });
    });
  });
}
