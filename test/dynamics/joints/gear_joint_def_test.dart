import 'package:forge2d/forge2d.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

void main() {
  group('GearJointDef', () {
    test('can be instantiated', () {
      expect(GearJointDef(), isA<GearJointDef>());
    });
  });
}
