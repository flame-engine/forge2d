/// *****************************************************************************
/// Copyright (c) 2015, Daniel Murphy, Google
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
///  * Redistributions of source code must retain the above copyright notice,
///    this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// *****************************************************************************

part of box2d;

/// A shape is used for collision detection. You can create a shape however you like. Shapes used for
/// simulation in World are created automatically when a Fixture is created. Shapes may encapsulate a
/// one or more child shapes.
abstract class Shape {
  final ShapeType shapeType;
  double radius = 0.0;

  Shape(this.shapeType);

  /// Get the number of child primitives
  int getChildCount();

  /// Test a point for containment in this shape. This only works for convex shapes.
  ///
  /// @param xf the shape world transform.
  /// @param p a point in world coordinates.
  bool testPoint(final Transform xf, final Vector2 p);

  /// Cast a ray against a child shape.
  ///
  /// @param argOutput the ray-cast results.
  /// @param argInput the ray-cast input parameters.
  /// @param argTransform the transform to be applied to the shape.
  /// @param argChildIndex the child shape index
  /// @return if hit
  bool raycast(RayCastOutput output, RayCastInput input, Transform transform,
      int childIndex);

  /// Given a transform, compute the associated axis aligned bounding box for a child shape.
  ///
  /// @param argAabb returns the axis aligned box.
  /// @param argXf the world transform of the shape.
  void computeAABB(final AABB aabb, final Transform xf, int childIndex);

  /// Compute the mass properties of this shape using its dimensions and density. The inertia tensor
  /// is computed about the local origin.
  ///
  /// @param massData returns the mass data for this shape.
  /// @param density the density in kilograms per meter squared.
  void computeMass(final MassData massData, final double density);

  /// Compute the distance from the current shape to the specified point. This only works for convex
  /// shapes.
  ///
  /// @param xf the shape world transform.
  /// @param p a point in world coordinates.
  /// @param normalOut returns the direction in which the distance increases.
  /// @return distance returns the distance from the current shape.
  double computeDistanceToOut(
      Transform xf, Vector2 p, int childIndex, Vector2 normalOut);

  Shape clone();
}
