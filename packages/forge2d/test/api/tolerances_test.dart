import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

// The length unit is process-wide, so this file only reads it. The tests that
// change it live in length_unit_test.dart, which puts it back afterwards.
void main() {
  setUpAll(initializeForge2D);

  group('Tolerances', () {
    test('defaults to meters', () {
      expect(Tolerances.lengthUnitsPerMeter, 1);
    });

    test('has the Box2D defaults', () {
      expect(Tolerances.linearSlop, closeTo(0.005, 1e-9));
      expect(Tolerances.speculativeDistance, closeTo(0.02, 1e-9));
      expect(Tolerances.aabbMargin, closeTo(0.05, 1e-9));
    });

    test('reports the speculative distance as four times the slop', () {
      expect(
        Tolerances.speculativeDistance,
        closeTo(4 * Tolerances.linearSlop, 1e-9),
      );
    });
  });
}
