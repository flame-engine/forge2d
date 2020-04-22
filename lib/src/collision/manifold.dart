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

/// A manifold for two touching convex shapes. Box2D supports multiple types of contact:
/// <ul>
/// <li>clip point versus plane with radius</li>
/// <li>point versus point with radius (circles)</li>
/// </ul>
/// The local point usage depends on the manifold type:
/// <ul>
/// <li>e_circles: the local center of circleA</li>
/// <li>e_faceA: the center of faceA</li>
/// <li>e_faceB: the center of faceB</li>
/// </ul>
/// Similarly the local normal usage:
/// <ul>
/// <li>e_circles: not used</li>
/// <li>e_faceA: the normal on polygonA</li>
/// <li>e_faceB: the normal on polygonB</li>
/// </ul>
/// We store contacts in this way so that position correction can account for movement, which is
/// critical for continuous physics. All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.

enum ManifoldType { CIRCLES, FACE_A, FACE_B }

class Manifold {
  /// The points of contact.
  final List<ManifoldPoint> points;

  /// not use for Type::e_points
  final Vector2 localNormal;

  /// usage depends on manifold type
  final Vector2 localPoint;

  ManifoldType type = ManifoldType.CIRCLES;

  /// The number of manifold points.
  int pointCount = 0;

  /// creates a manifold with 0 points, with it's points array full of instantiated ManifoldPoints.
  Manifold()
      : points = List<ManifoldPoint>(Settings.maxManifoldPoints),
        localNormal = Vector2.zero(),
        localPoint = Vector2.zero() {
    for (int i = 0; i < Settings.maxManifoldPoints; i++) {
      points[i] = ManifoldPoint();
    }
  }

  /// Creates this manifold as a copy of the other
  Manifold.copy(Manifold other)
      : points = List<ManifoldPoint>(Settings.maxManifoldPoints),
        localNormal = other.localNormal.clone(),
        localPoint = other.localPoint.clone(),
        pointCount = other.pointCount {
    type = other.type;
    // djm: this is correct now
    for (int i = 0; i < Settings.maxManifoldPoints; i++) {
      points[i] = ManifoldPoint.copy(other.points[i]);
    }
  }

  /// copies this manifold from the given one
  ///
  /// @param cp manifold to copy from
  void set(Manifold cp) {
    for (int i = 0; i < cp.pointCount; i++) {
      points[i].set(cp.points[i]);
    }

    type = cp.type;
    localNormal.setFrom(cp.localNormal);
    localPoint.setFrom(cp.localPoint);
    pointCount = cp.pointCount;
  }
}
