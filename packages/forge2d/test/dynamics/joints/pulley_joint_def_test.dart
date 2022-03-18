import 'package:forge2d/forge2d.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

void main() {
  group('PulleyJointDef', () {
    test('can be instantiated', () {
      expect(PulleyJointDef(), isA<PulleyJointDef>());
    });
  });
}
