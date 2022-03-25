import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('FrictionJointDef', () {
    test('can be instantiated', () {
      expect(FrictionJointDef(), isA<FrictionJointDef>());
    });
  });
}
