import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('PulleyJointDef', () {
    test('can be instantiated', () {
      expect(PulleyJointDef(), isA<PulleyJointDef>());
    });
  });
}
