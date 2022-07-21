import 'package:forge2d/forge2d_browser.dart';
import 'package:mocktail/mocktail.dart';
import 'package:test/expect.dart';
import 'package:test/scaffolding.dart';

class _MockFixture extends Mock implements Fixture {}

class _TestContact extends Contact {
  _TestContact(
    Fixture fixtureA,
    int indexA,
    Fixture fixtureB,
    int indexB,
  ) : super(
          fixtureA,
          indexA,
          fixtureB,
          indexB,
        );

  @override
  void evaluate(_, __, ___) => throw UnimplementedError();
}

void mian() {
  group('Contact', () {
    late Fixture fixtureA;
    late int indexA;
    late Fixture fixtureB;
    late int indexB;

    setUp(() {
      fixtureA = _MockFixture();
      indexA = 0;
      fixtureB = _MockFixture();
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

    group('enabled', () {
      test('enabled by default', () {
        final contact = _TestContact(
          fixtureA,
          indexA,
          fixtureB,
          indexB,
        );

        expect(contact.enabled, isTrue);
      });

      test('can change', () {
        final contact = _TestContact(
          fixtureA,
          indexA,
          fixtureB,
          indexB,
        );

        final newEnabled = !contact.enabled;
        contact.enabled = newEnabled;

        expect(contact.enabled, equals(newEnabled));
      });
    });
  });
}
