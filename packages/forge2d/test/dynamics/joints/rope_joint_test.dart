import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

import '../../helpers/helpers.dart';

void main() {
  group('RopeJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = RopeJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(RopeJoint(jointDef), isA<RopeJoint>());
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

      test('draws three segments', () {
        final joint = RopeJoint(
          RopeJointDef()
            ..bodyA = Body(BodyDef(), world)
            ..bodyB = Body(BodyDef(), world),
        );
        joint.render(debugDraw);
        verify(() => debugDraw.drawSegment(any(), any(), any())).called(3);
      });
    });
  });
}
