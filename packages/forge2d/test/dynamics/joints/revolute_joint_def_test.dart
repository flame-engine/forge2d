import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('RevoluteJointDef', () {
    test('can be instantiated', () {
      expect(RevoluteJointDef(), isA<RevoluteJointDef>());
    });
  });
}
