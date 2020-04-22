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

/// Implement this abstract class to allow DBox2d to automatically draw your physics for debugging
/// purposes. Not intended to replace your own custom rendering routines!
abstract class DebugDraw {
  /// Draw shapes
  static const int SHAPE_BIT = 1 << 1;

  /// Draw joint connections
  static const int JOINT_BIT = 1 << 2;

  /// Draw axis aligned bounding boxes
  static const int AABB_BIT = 1 << 3;

  /// Draw pairs of connected objects
  static const int PAIR_BIT = 1 << 4;

  /// Draw center of mass frame
  static const int CENTER_OF_MASS_BIT = 1 << 5;

  /// Draw dynamic tree
  static const int DYNAMIC_TREE_BIT = 1 << 6;

  /// Draw only the wireframe for drawing performance
  static const int WIREFRAME_DRAWING_BIT = 1 << 7;

  int drawFlags = SHAPE_BIT;
  ViewportTransform viewportTransform;

  DebugDraw.zero();
  DebugDraw(this.viewportTransform);

  void setViewportTransform(ViewportTransform viewportTransform) {
    this.viewportTransform = viewportTransform;
  }

  void appendFlags(int flags) {
    drawFlags |= flags;
  }

  void clearFlags(int flags) {
    drawFlags &= ~flags;
  }

  /// Draw a closed polygon provided in CCW order. This implementation uses
  /// {@link #drawSegment(Vec2, Vec2, Color3f)} to draw each side of the polygon.
  void drawPolygon(List<Vector2> vertices, int vertexCount, Color3i color) {
    if (vertexCount == 1) {
      drawSegment(vertices[0], vertices[0], color);
      return;
    }

    for (int i = 0; i < vertexCount - 1; i += 1) {
      drawSegment(vertices[i], vertices[i + 1], color);
    }

    if (vertexCount > 2) {
      drawSegment(vertices[vertexCount - 1], vertices[0], color);
    }
  }

  void drawPoint(Vector2 argPoint, double argRadiusOnScreen, Color3i argColor);

  /// Draw a solid closed polygon provided in CCW order.
  void drawSolidPolygon(List<Vector2> vertices, int vertexCount, Color3i color);

  /// Draw a circle.
  void drawCircle(Vector2 center, double radius, Color3i color);

  /// Draws a circle with an axis
  void drawCircleAxis(
      Vector2 center, double radius, Vector2 axis, Color3i color) {
    drawCircle(center, radius, color);
  }

  /// Draw a solid circle.
  void drawSolidCircle(
      Vector2 center, double radius, Vector2 axis, Color3i color);

  /// Draw a line segment.
  void drawSegment(Vector2 p1, Vector2 p2, Color3i color);

  /// Draw a transform. Choose your own length scale
  void drawTransform(Transform xf, Color3i color);

  /// Draw a string.
  void drawStringXY(double x, double y, String s, Color3i color);

  /// Draw a particle array
  /// @param colors can be null
  void drawParticles(List<Vector2> centers, double radius,
      List<ParticleColor> colors, int count);

  /// Draw a particle array
  /// @param colors can be null
  void drawParticlesWireframe(List<Vector2> centers, double radius,
      List<ParticleColor> colors, int count);

  /// Called at the end of drawing a world
  void flush() {}

  void drawString(Vector2 pos, String s, Color3i color) {
    drawStringXY(pos.x, pos.y, s, color);
  }

  ViewportTransform getViewportTranform() {
    return viewportTransform;
  }

  /// @deprecated use the viewport transform in {@link #getViewportTranform()}
  void setCamera(double x, double y, double scale) {
    viewportTransform.setCamera(x, y, scale);
  }

  /// Takes the world coordinates and returns the corresponding screen
  /// coordinates
  Vector2 getWorldToScreenToOut(Vector2 input) =>
      viewportTransform.getWorldToScreen(input);

  /// Takes the world coordinates and returns the corresponding screen
  /// coordinates
  Vector2 getWorldToScreenToOutXY(double worldX, double worldY) =>
      getWorldToScreenToOut(Vector2(worldX, worldY));

  /// Takes the world coordinate and returns the screen coordinates.
  Vector2 getWorldToScreen(Vector2 argWorld) =>
      viewportTransform.getWorldToScreen(argWorld);

  /// Takes the world coordinates and returns the screen coordinates
  Vector2 getWorldToScreenXY(double worldX, double worldY) =>
      viewportTransform.getWorldToScreen(Vector2(worldX, worldY));

  /// Takes the screen coordinates (argScreen) and returns the world coordinates
  Vector2 getScreenToWorld(Vector2 argScreen) =>
      viewportTransform.getScreenToWorld(argScreen);

  /// Takes the screen coordinates and returns the corresponding world coordinates
  Vector2 getScreenToWorldToOutXY(double screenX, double screenY) =>
      viewportTransform.getScreenToWorld(Vector2(screenX, screenY));
}
