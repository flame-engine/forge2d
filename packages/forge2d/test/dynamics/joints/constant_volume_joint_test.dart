import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

import '../../helpers/helpers.dart';

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
        final joint = ConstantVolumeJoint(
          world,
          ConstantVolumeJointDef()
            ..addBody(Body(BodyDef(), world))
            ..addBody(Body(BodyDef(), world))
            ..addBody(Body(BodyDef(), world)),
        );
        joint.render(debugDraw);
        verifyNever<void>(() => debugDraw.drawSegment(any(), any(), any()));
      });
    });
  });
}
