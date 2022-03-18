import 'package:forge2d/forge2d.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

void main() {
  group('MouseJointDef', () {
    test('can be instantiated', () {
      expect(MouseJointDef(), isA<MouseJointDef>());
    });
  });
}
