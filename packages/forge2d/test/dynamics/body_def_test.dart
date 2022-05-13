import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('BodyDef', () {
    test('can be instantiated', () {
      expect(BodyDef(), isA<BodyDef>());
    });
  });
}
