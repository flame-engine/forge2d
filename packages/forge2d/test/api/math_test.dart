import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

void main() {
  group('Rot', () {
    test('identity has an angle of zero', () {
      expect(const Rot.identity().angle, 0);
    });

    test('fromAngle round-trips the angle', () {
      final rot = Rot.fromAngle(math.pi / 3);
      expect(rot.angle, closeTo(math.pi / 3, 1e-9));
    });

    test('rotate and inverseRotate are inverses', () {
      final rot = Rot.fromAngle(1.2);
      final rotated = rot.rotate(Vector2(3, -4));
      final back = rot.inverseRotate(rotated);
      expect(back.x, closeTo(3, 1e-6));
      expect(back.y, closeTo(-4, 1e-6));
    });

    test('composition adds angles', () {
      final composed = Rot.fromAngle(0.5) * Rot.fromAngle(0.25);
      expect(composed.angle, closeTo(0.75, 1e-9));
    });

    test('rotating a unit vector by a quarter turn', () {
      final rotated = Rot.fromAngle(math.pi / 2).rotate(Vector2(1, 0));
      expect(rotated.x, closeTo(0, 1e-9));
      expect(rotated.y, closeTo(1, 1e-9));
    });
  });

  group('Transform', () {
    test('identity maps points onto themselves', () {
      final point = Transform.identity().apply(Vector2(2, 5));
      expect(point, Vector2(2, 5));
    });

    test('apply rotates then translates', () {
      final transform = Transform(Vector2(10, 0), Rot.fromAngle(math.pi / 2));
      final point = transform.apply(Vector2(1, 0));
      expect(point.x, closeTo(10, 1e-6));
      expect(point.y, closeTo(1, 1e-6));
    });

    test('applyInverse inverts apply', () {
      final transform = Transform(Vector2(-3, 7), Rot.fromAngle(2.1));
      final point = Vector2(4, -2);
      final back = transform.applyInverse(transform.apply(point));
      expect(back.x, closeTo(point.x, 1e-6));
      expect(back.y, closeTo(point.y, 1e-6));
    });
  });

  group('Aabb', () {
    test('containingPoints normalizes bounds', () {
      final aabb = Aabb.containingPoints(Vector2(3, -1), Vector2(-2, 4));
      expect(aabb.lowerBound, Vector2(-2, -1));
      expect(aabb.upperBound, Vector2(3, 4));
    });

    test('center and extents', () {
      final aabb = Aabb(Vector2(0, 0), Vector2(4, 2));
      expect(aabb.center, Vector2(2, 1));
      expect(aabb.extents, Vector2(2, 1));
    });

    test('contains and overlaps', () {
      final outer = Aabb(Vector2(0, 0), Vector2(10, 10));
      final inner = Aabb(Vector2(1, 1), Vector2(2, 2));
      final crossing = Aabb(Vector2(9, 9), Vector2(11, 11));
      final outside = Aabb(Vector2(20, 20), Vector2(21, 21));

      expect(outer.contains(inner), isTrue);
      expect(outer.contains(crossing), isFalse);
      expect(outer.overlaps(crossing), isTrue);
      expect(outer.overlaps(outside), isFalse);
    });

    test('union contains both inputs', () {
      final a = Aabb(Vector2(0, 0), Vector2(1, 1));
      final b = Aabb(Vector2(5, -2), Vector2(6, 0));
      final union = a.union(b);
      expect(union.contains(a), isTrue);
      expect(union.contains(b), isTrue);
    });
  });
}
