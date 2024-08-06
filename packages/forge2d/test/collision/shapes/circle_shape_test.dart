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
  });
}
