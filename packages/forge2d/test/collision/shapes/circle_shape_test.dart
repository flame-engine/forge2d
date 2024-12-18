import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('CircleShape', () {
    test('Test radius', () {
      const radius = 5.0;
      final circleShape = CircleShape(radius: radius);
      expect(circleShape.radius, equals(radius));
    });

    test('Test position', () {
      final position = Vector2(1.0, 2.0);
      final circleShape = CircleShape(position: position);
      expect(circleShape.position, equals(position));
    });

    test('Test clone', () {
      const radius = 5.0;
      final position = Vector2(1.0, 2.0);
      final circleShape = CircleShape(radius: radius, position: position);
      final clonedCircleShape = circleShape.clone() as CircleShape;
      expect(clonedCircleShape.radius, equals(radius));
      expect(clonedCircleShape.position, equals(position));
    });

    test('Test computeDistanceToOut', () {
      void checkDistance(
        Transform tf,
        Vector2 testPoint,
        double expectedDistance,
        Vector2 expectedNormal,
      ) {
        const radius = 3.0;
        final position = Vector2(1.0, 2.0);
        final circleShape = CircleShape(radius: radius, position: position);
        final normal = Vector2.zero();
        final distance =
            circleShape.computeDistanceToOut(tf, testPoint, 0, normal);
        expect(distance, closeTo(expectedDistance, 0.01));
        expect(normal.x, closeTo(expectedNormal.x, 0.01));
        expect(normal.y, closeTo(expectedNormal.y, 0.01));
      }

      final tfZero = Transform.zero();
      checkDistance(tfZero, Vector2(10, 7), 7.3, Vector2(0.87, 0.49));
      checkDistance(tfZero, Vector2(1, -5), 4.0, Vector2(0.0, -1.0));

      final tf2 = Transform.zero()..setVec2Angle(Vector2(10, 10), 0.2);
      checkDistance(tf2, Vector2(10, 7), 7.59, Vector2(0.89, 0.46));
      checkDistance(tf2, Vector2(1, -5), 4.17, Vector2(0.06, -1.0));
    });
  });
}
