import 'dart:math';

import '../../forge2d.dart';
import '../settings.dart' as settings;

/// An axis-aligned bounding box.
class AABB {
  /// Bottom left vertex of bounding box.
  final Vector2 lowerBound;

  /// Top right vertex of bounding box.
  final Vector2 upperBound;

  /// Creates the default object, with vertices at 0,0 and 0,0.
  AABB()
      : lowerBound = Vector2.zero(),
        upperBound = Vector2.zero();

  /// Copies from the given object
  /// @param copy the object to copy from
  AABB.copy(final AABB copy)
      : lowerBound = Vector2.copy(copy.lowerBound),
        upperBound = Vector2.copy(copy.upperBound);

  /// Creates an AABB object using the given bounding vertices.
  /// @param lowerVertex the bottom left vertex of the bounding box
  /// @param maxVertex the top right vertex of the bounding box
  AABB.withVec2(final Vector2 lowerVertex, final Vector2 upperVertex)
      : lowerBound = Vector2.copy(lowerVertex),
        upperBound = Vector2.copy(upperVertex);

  /// Sets this object from the given object
  /// @param aabb the object to copy from
  void set(final AABB aabb) {
    final v = aabb.lowerBound;
    lowerBound.x = v.x;
    lowerBound.y = v.y;
    final v1 = aabb.upperBound;
    upperBound.x = v1.x;
    upperBound.y = v1.y;
  }

  /// Verify that the bounds are sorted
  bool isValid() {
    final dx = upperBound.x - lowerBound.x;
    if (dx < 0.0) {
      return false;
    }
    final dy = upperBound.y - lowerBound.y;
    if (dy < 0) {
      return false;
    }
    return !lowerBound.isInfinite &&
        !lowerBound.isNaN &&
        !upperBound.isInfinite &&
        !upperBound.isNaN;
  }

  /// Get the center of the AABB
  Vector2 get center => (lowerBound + upperBound)..scale(0.5);

  /// Get the extents of the AABB (half-widths).
  Vector2 get extents {
    return Vector2.copy(upperBound)
      ..sub(lowerBound)
      ..scale(.5);
  }

  void extentsToOut(final Vector2 out) {
    out.x = (upperBound.x - lowerBound.x) * .5;
    out.y = (upperBound.y - lowerBound.y) * .5; // thanks FDN1
  }

  void getVertices(List<Vector2> argRay) {
    argRay[0].setFrom(lowerBound);
    argRay[1].setFrom(lowerBound);
    argRay[1].x += upperBound.x - lowerBound.x;
    argRay[2].setFrom(upperBound);
    argRay[3].setFrom(upperBound);
    argRay[3].x -= upperBound.x - lowerBound.x;
  }

  /// Combine two AABBs into this one.
  void combine2(final AABB aabb1, final AABB aab) {
    lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x
        ? aabb1.lowerBound.x
        : aab.lowerBound.x;
    lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y
        ? aabb1.lowerBound.y
        : aab.lowerBound.y;
    upperBound.x = aabb1.upperBound.x > aab.upperBound.x
        ? aabb1.upperBound.x
        : aab.upperBound.x;
    upperBound.y = aabb1.upperBound.y > aab.upperBound.y
        ? aabb1.upperBound.y
        : aab.upperBound.y;
  }

  /// Gets the perimeter length
  double get perimeter {
    return 2.0 * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y);
  }

  /// Combines another aabb with this one
  void combine(final AABB aabb) {
    lowerBound.x =
        lowerBound.x < aabb.lowerBound.x ? lowerBound.x : aabb.lowerBound.x;
    lowerBound.y =
        lowerBound.y < aabb.lowerBound.y ? lowerBound.y : aabb.lowerBound.y;
    upperBound.x =
        upperBound.x > aabb.upperBound.x ? upperBound.x : aabb.upperBound.x;
    upperBound.y =
        upperBound.y > aabb.upperBound.y ? upperBound.y : aabb.upperBound.y;
  }

  /// Does this aabb contain the provided AABB.
  bool contains(final AABB aabb) {
    // djm: faster putting all of them together, as if one is false we leave the logic
    // early
    return lowerBound.x <= aabb.lowerBound.x &&
        lowerBound.y <= aabb.lowerBound.y &&
        aabb.upperBound.x <= upperBound.x &&
        aabb.upperBound.y <= upperBound.y;
  }

  /// @deprecated please use {@link #raycast(RayCastOutput, RayCastInput, IWorldPool)} for better performance
  //bool raycast(final RayCastOutput output, final RayCastInput input) {
  //  return raycastWithPool(output, input, DefaultWorldPool(4, 4));
  //}

  /// From Real-time Collision Detection, p179.
  bool raycastWithPool(
    final RayCastOutput output,
    final RayCastInput input,
  ) {
    var tMix = -double.maxFinite;
    var tMax = double.maxFinite;

    final p = Vector2.zero();
    final d = Vector2.zero();
    final absD = Vector2.zero();
    final normal = Vector2.zero();

    p.setFrom(input.p1);
    d
      ..setFrom(input.p2)
      ..sub(input.p1);
    absD
      ..setFrom(d)
      ..absolute();

    // x then y
    if (absD.x < settings.epsilon) {
      // Parallel.
      if (p.x < lowerBound.x || upperBound.x < p.x) {
        return false;
      }
    } else {
      final invD = 1.0 / d.x;
      var t1 = (lowerBound.x - p.x) * invD;
      var t2 = (upperBound.x - p.x) * invD;

      // Sign of the normal vector.
      var s = -1.0;

      if (t1 > t2) {
        final temp = t1;
        t1 = t2;
        t2 = temp;
        s = 1.0;
      }

      // Push the min up
      if (t1 > tMix) {
        normal.setZero();
        normal.x = s;
        tMix = t1;
      }

      // Pull the max down
      tMax = min(tMax, t2);

      if (tMix > tMax) {
        return false;
      }
    }

    if (absD.y < settings.epsilon) {
      // Parallel.
      if (p.y < lowerBound.y || upperBound.y < p.y) {
        return false;
      }
    } else {
      final invD = 1.0 / d.y;
      var t1 = (lowerBound.y - p.y) * invD;
      var t2 = (upperBound.y - p.y) * invD;

      // Sign of the normal vector.
      var s = -1.0;

      if (t1 > t2) {
        final temp = t1;
        t1 = t2;
        t2 = temp;
        s = 1.0;
      }

      // Push the min up
      if (t1 > tMix) {
        normal.setZero();
        normal.y = s;
        tMix = t1;
      }

      // Pull the max down
      tMax = min(tMax, t2);

      if (tMix > tMax) {
        return false;
      }
    }

    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if (tMix < 0.0 || input.maxFraction < tMix) {
      return false;
    }

    // Intersection.
    output.fraction = tMix;
    output.normal.x = normal.x;
    output.normal.y = normal.y;
    return true;
  }

  static bool testOverlap(final AABB a, final AABB b) {
    if (b.lowerBound.x - a.upperBound.x > 0.0 ||
        b.lowerBound.y - a.upperBound.y > 0.0) {
      return false;
    }

    if (a.lowerBound.x - b.upperBound.x > 0.0 ||
        a.lowerBound.y - b.upperBound.y > 0.0) {
      return false;
    }

    return true;
  }

  @override
  String toString() {
    return 'AABB[$lowerBound . $upperBound]';
  }
}
