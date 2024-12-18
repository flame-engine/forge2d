import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('PolygonShape', () {
    test('Test setAsBoxXY', () {
      final polygonShape = PolygonShape()..setAsBoxXY(3, 4);
      expect(
        polygonShape.vertices,
        equals([
          Vector2(-3.0, -4.0),
          Vector2(3.0, -4.0),
          Vector2(3.0, 4.0),
          Vector2(-3.0, 4.0),
        ]),
      );
      expect(
        polygonShape.normals,
        equals([
          Vector2(0.0, -1.0),
          Vector2(1.0, 0.0),
          Vector2(0.0, 1.0),
          Vector2(-1.0, 0.0),
        ]),
      );
    });

    test('Test clone', () {
      final polygonShape = PolygonShape()..setAsBoxXY(3, 4);
      final clonedPolygonShape = polygonShape.clone() as PolygonShape;
      expect(clonedPolygonShape.radius, equals(polygonShape.radius));
      expect(clonedPolygonShape.vertices, equals(polygonShape.vertices));
      expect(clonedPolygonShape.normals, equals(polygonShape.normals));
    });

    test('Test computeDistanceToOut', () {
      void checkDistance(
        Transform tf,
        Vector2 testPoint,
        double expectedDistance,
        Vector2 expectedNormal,
      ) {
        final polygonShape = PolygonShape()..setAsBoxXY(3, 4);
        final normal = Vector2.zero();
        final distance =
            polygonShape.computeDistanceToOut(tf, testPoint, 0, normal);
        expect(distance, closeTo(expectedDistance, 0.01));
        expect(normal.x, closeTo(expectedNormal.x, 0.01));
        expect(normal.y, closeTo(expectedNormal.y, 0.01));
      }

      final tfZero = Transform.zero();
      checkDistance(tfZero, Vector2(10, 7), 7.0, Vector2(1.0, 0.0));
      checkDistance(tfZero, Vector2(1, -5), 1.0, Vector2(0.0, -1.0));

      final tf2 = Transform.zero()..setVec2Angle(Vector2(10, 10), 0.2);
      checkDistance(tf2, Vector2(10, 7), 8.2, Vector2(0.98, 0.2));
      checkDistance(tf2, Vector2(1, -5), 1.1, Vector2(0.2, -0.98));
    });
  });
}
