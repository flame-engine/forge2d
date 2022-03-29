import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('FixtureDef', () {
    group('can be instantiated', () {
      test('when Shape is a ChainShape', () {
        expect(FixtureDef(ChainShape()), isA<FixtureDef>());
      });

      test('when Shape is a CircleShape', () {
        expect(FixtureDef(CircleShape()), isA<FixtureDef>());
      });

      test('when Shape is a EdgeShape', () {
        expect(FixtureDef(EdgeShape()), isA<FixtureDef>());
      });

      test('when Shape is a PolygonShape', () {
        expect(FixtureDef(PolygonShape()), isA<FixtureDef>());
      });
    });
  });
}
