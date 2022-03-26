import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('ConstantVolumeJointDef', () {
    test('can be instantiated', () {
      expect(ConstantVolumeJointDef(), isA<ConstantVolumeJointDef>());
    });
  });
}
