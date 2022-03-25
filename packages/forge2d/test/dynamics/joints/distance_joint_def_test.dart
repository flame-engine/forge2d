import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('DistanceJointDef', () {
    test('can be instantiated', () {
      expect(DistanceJointDef(), isA<DistanceJointDef>());
    });
  });
}
