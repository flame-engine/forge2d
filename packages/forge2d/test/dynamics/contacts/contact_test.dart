import 'package:forge2d/forge2d.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/test.dart';

class _MockFixture extends Mock implements Fixture {}

class _TestContact extends Contact {
  _TestContact(
    super.fixtureA,
    super.indexA,
    super.fixtureB,
    super.indexB,
  );

  @override
  void evaluate(_, __, ___) => throw UnimplementedError();
}

void main() {
  group('Contact', () {
    late Fixture fixtureA;
    late int indexA;
    late Fixture fixtureB;
    late int indexB;

    setUp(() {
      fixtureA = _MockFixture();
      when(() => fixtureA.friction).thenReturn(0);
      when(() => fixtureA.restitution).thenReturn(0);

      fixtureB = _MockFixture();
      when(() => fixtureB.friction).thenReturn(0);
      when(() => fixtureB.restitution).thenReturn(0);

      indexA = 0;
      indexB = 0;
    });

    test('can be instantiated', () {
      expect(
        _TestContact(
          fixtureA,
          indexA,
          fixtureB,
          indexB,
        ),
        isA<Contact>(),
      );
    });

    group('isEnabled', () {
      test('true by default', () {
        final contact = _TestContact(
          fixtureA,
          indexA,
          fixtureB,
          indexB,
        );

        expect(contact.isEnabled, isTrue);
      });

      test('can change', () {
        final contact = _TestContact(
          fixtureA,
          indexA,
          fixtureB,
          indexB,
        );

        final newIsEnabled = !contact.isEnabled;
        contact.isEnabled = newIsEnabled;

        expect(contact.isEnabled, equals(newIsEnabled));
      });
    });
  });
}
