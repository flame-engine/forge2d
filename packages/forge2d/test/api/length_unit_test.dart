import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

// The length unit is a global inside Box2D that every suite in the process
// shares, so suites run one at a time; see dart_test.yaml. Within this file
// the tests run in declaration order and build on each other.
void main() {
  setUpAll(initializeForge2D);

  // Puts the length unit back, so that the suites that run after this one see
  // the default whatever order they end up in.
  tearDownAll(() async {
    debugResetLengthUnitLock();
    await initializeForge2D(lengthUnitsPerMeter: 1);
  });

  group('initializeForge2D(lengthUnitsPerMeter:)', () {
    test('rejects values that are not positive and finite', () async {
      for (final value in [0.0, -1.0, double.nan, double.infinity]) {
        await expectLater(
          initializeForge2D(lengthUnitsPerMeter: value),
          throwsArgumentError,
          reason: '$value should be rejected',
        );
      }
      expect(Tolerances.lengthUnitsPerMeter, 1);
    });

    test('scales the tolerances', () async {
      await initializeForge2D(lengthUnitsPerMeter: 100);

      expect(Tolerances.lengthUnitsPerMeter, 100);
      expect(Tolerances.linearSlop, closeTo(0.5, 1e-6));
      expect(Tolerances.speculativeDistance, closeTo(2, 1e-6));
      expect(Tolerances.aabbMargin, closeTo(5, 1e-6));
    });

    test('accepts the value that is already in effect again', () async {
      await initializeForge2D(lengthUnitsPerMeter: 100);
      expect(Tolerances.lengthUnitsPerMeter, 100);
    });

    test('accepts a value that only round trips through float32', () async {
      // 0.04 is not representable in either float32 or float64, so the
      // repeat-request check has to compare the way Box2D stores it.
      debugResetLengthUnitLock();
      await initializeForge2D(lengthUnitsPerMeter: 0.04);
      await expectLater(
        initializeForge2D(lengthUnitsPerMeter: 0.04),
        completes,
      );
      expect(Tolerances.lengthUnitsPerMeter, closeTo(0.04, 1e-9));

      debugResetLengthUnitLock();
      await initializeForge2D(lengthUnitsPerMeter: 100);
    });

    test('throws when a world already exists and the value differs', () async {
      final world = World();
      addTearDown(world.destroy);

      await expectLater(
        initializeForge2D(lengthUnitsPerMeter: 50),
        throwsA(
          isA<StateError>().having(
            (error) => error.message,
            'message',
            contains('a World has already been created'),
          ),
        ),
      );
      expect(Tolerances.lengthUnitsPerMeter, 100);
    });

    test('still accepts the unchanged value once a world exists', () async {
      final world = World();
      addTearDown(world.destroy);

      await expectLater(
        initializeForge2D(lengthUnitsPerMeter: 100),
        completes,
      );
    });
  });
}
