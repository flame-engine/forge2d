/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

/** An axis-aligned bounding box. */
class AABB {
  /** Bottom left vertex of bounding box. */
  final Vector2 lowerBound;
  /** Top right vertex of bounding box. */
  final Vector2 upperBound;

  /**
   * Creates the default object, with vertices at 0,0 and 0,0.
   */
  AABB()
      : lowerBound = new Vector2.zero(),
        upperBound = new Vector2.zero();

  /**
   * Copies from the given object
   *
   * @param copy the object to copy from
   */
  AABB.copy(final AABB copy)
      : lowerBound = new Vector2.copy(copy.lowerBound),
        upperBound = new Vector2.copy(copy.upperBound);

  /**
   * Creates an AABB object using the given bounding vertices.
   *
   * @param lowerVertex the bottom left vertex of the bounding box
   * @param maxVertex the top right vertex of the bounding box
   */
  AABB.withVec2(final Vector2 lowerVertex, final Vector2 upperVertex)
      : lowerBound = new Vector2.copy(lowerVertex),
        upperBound = new Vector2.copy(upperVertex);

  /**
   * Sets this object from the given object
   *
   * @param aabb the object to copy from
   */
  void set(final AABB aabb) {
    Vector2 v = aabb.lowerBound;
    lowerBound.x = v.x;
    lowerBound.y = v.y;
    Vector2 v1 = aabb.upperBound;
    upperBound.x = v1.x;
    upperBound.y = v1.y;
  }

  /** Verify that the bounds are sorted */
  bool isValid() {
    final double dx = upperBound.x - lowerBound.x;
    if (dx < 0.0) {
      return false;
    }
    final double dy = upperBound.y - lowerBound.y;
    if (dy < 0) {
      return false;
    }
    return MathUtils.vector2IsValid(lowerBound) &&
        MathUtils.vector2IsValid(upperBound);
  }

  /**
   * Get the center of the AABB
   *
   * @return
   */
  Vector2 getCenter() {
    final Vector2 center = new Vector2.copy(lowerBound);
    center.add(upperBound);
    center.scale(.5);
    return center;
  }

  void getCenterToOut(final Vector2 out) {
    out.x = (lowerBound.x + upperBound.x) * .5;
    out.y = (lowerBound.y + upperBound.y) * .5;
  }

  /**
   * Get the extents of the AABB (half-widths).
   *
   * @return
   */
  Vector2 getExtents() {
    final Vector2 center = new Vector2.copy(upperBound);
    center.sub(lowerBound);
    center.scale(.5);
    return center;
  }

  void getExtentsToOut(final Vector2 out) {
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

  /**
   * Combine two AABBs into this one.
   *
   * @param aabb1
   * @param aab
   */
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

  /**
   * Gets the perimeter length
   *
   * @return
   */
  double getPerimeter() {
    return 2.0 * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y);
  }

  /**
   * Combines another aabb with this one
   *
   * @param aabb
   */
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

  /**
   * Does this aabb contain the provided AABB.
   *
   * @return
   */
  bool contains(final AABB aabb) {
    /*
     * boolean result = true; result = result && lowerBound.x <= aabb.lowerBound.x; result = result
     * && lowerBound.y <= aabb.lowerBound.y; result = result && aabb.upperBound.x <= upperBound.x;
     * result = result && aabb.upperBound.y <= upperBound.y; return result;
     */
    // djm: faster putting all of them together, as if one is false we leave the logic
    // early
    return lowerBound.x <= aabb.lowerBound.x &&
        lowerBound.y <= aabb.lowerBound.y &&
        aabb.upperBound.x <= upperBound.x &&
        aabb.upperBound.y <= upperBound.y;
  }

  /**
   * @deprecated please use {@link #raycast(RayCastOutput, RayCastInput, IWorldPool)} for better
   *             performance
   * @param output
   * @param input
   * @return
   */
  bool raycast(final RayCastOutput output, final RayCastInput input) {
    return raycastWithPool(output, input, new DefaultWorldPool(4, 4));
  }

  /**
   * From Real-time Collision Detection, p179.
   *
   * @param output
   * @param input
   */
  bool raycastWithPool(final RayCastOutput output, final RayCastInput input,
      IWorldPool argPool) {
    double tmin = -double.maxFinite;
    double tmax = double.maxFinite;

    final Vector2 p = argPool.popVec2();
    final Vector2 d = argPool.popVec2();
    final Vector2 absD = argPool.popVec2();
    final Vector2 normal = argPool.popVec2();

    p.setFrom(input.p1);
    d
      ..setFrom(input.p2)
      ..sub(input.p1);
    absD
      ..setFrom(d)
      ..absolute();

    // x then y
    if (absD.x < Settings.EPSILON) {
      // Parallel.
      if (p.x < lowerBound.x || upperBound.x < p.x) {
        argPool.pushVec2(4);
        return false;
      }
    } else {
      final double inv_d = 1.0 / d.x;
      double t1 = (lowerBound.x - p.x) * inv_d;
      double t2 = (upperBound.x - p.x) * inv_d;

      // Sign of the normal vector.
      double s = -1.0;

      if (t1 > t2) {
        final double temp = t1;
        t1 = t2;
        t2 = temp;
        s = 1.0;
      }

      // Push the min up
      if (t1 > tmin) {
        normal.setZero();
        normal.x = s;
        tmin = t1;
      }

      // Pull the max down
      tmax = Math.min(tmax, t2);

      if (tmin > tmax) {
        argPool.pushVec2(4);
        return false;
      }
    }

    if (absD.y < Settings.EPSILON) {
      // Parallel.
      if (p.y < lowerBound.y || upperBound.y < p.y) {
        argPool.pushVec2(4);
        return false;
      }
    } else {
      double inv_d = 1.0 / d.y;
      double t1 = (lowerBound.y - p.y) * inv_d;
      double t2 = (upperBound.y - p.y) * inv_d;

      // Sign of the normal vector.
      double s = -1.0;

      if (t1 > t2) {
        final double temp = t1;
        t1 = t2;
        t2 = temp;
        s = 1.0;
      }

      // Push the min up
      if (t1 > tmin) {
        normal.setZero();
        normal.y = s;
        tmin = t1;
      }

      // Pull the max down
      tmax = Math.min(tmax, t2);

      if (tmin > tmax) {
        argPool.pushVec2(4);
        return false;
      }
    }

    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if (tmin < 0.0 || input.maxFraction < tmin) {
      argPool.pushVec2(4);
      return false;
    }

    // Intersection.
    output.fraction = tmin;
    output.normal.x = normal.x;
    output.normal.y = normal.y;
    argPool.pushVec2(4);
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

  String toString() {
    final String s = "AABB[$lowerBound . $upperBound]";
    return s;
  }
}
