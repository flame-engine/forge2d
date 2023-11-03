import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('RopeJointDef', () {
    test('can be instantiated', () {
      expect(RopeJointDef(), isA<RopeJointDef>());
    });

    group('constructor', () {
      test('sets localAnchorA to zero', () {
        expect(RopeJointDef().localAnchorA, equals(Vector2.zero()));
      });

      test('sets localAnchorB to zero', () {
        expect(RopeJointDef().localAnchorB, equals(Vector2.zero()));
      });
    });
  });
}
