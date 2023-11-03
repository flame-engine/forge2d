import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

import '../../helpers/helpers.dart';

void main() {
  group('MouseJoint', () {
    test('can be instantiated', () {
      final world = World();
      final jointDef = MouseJointDef()
        ..bodyA = Body(BodyDef(), world)
        ..bodyB = Body(BodyDef(), world);

      expect(MouseJoint(jointDef), isA<MouseJoint>());
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

      test('does nothing', () {
        final joint = MouseJoint(
          MouseJointDef()
            ..bodyA = (Body(BodyDef(), world))
            ..bodyB = (Body(BodyDef(), world)),
        );
        joint.render(debugDraw);
        verifyNever<void>(() => debugDraw.drawSegment(any(), any(), any()));
      });
    });
  });
}
