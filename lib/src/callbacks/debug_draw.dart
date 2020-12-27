part of forge2d;

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

  DebugDraw(this.viewportTransform);

  DebugDraw.zero();

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
  void drawPolygon(List<Vector2> vertices, Color3i color) {
    final int vertexCount = vertices.length;
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
  void drawSolidPolygon(List<Vector2> vertices, Color3i color);

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

  /// Takes the world coordinate and returns the screen coordinates.
  Vector2 getWorldToScreen(Vector2 argWorld) =>
      viewportTransform.getWorldToScreen(argWorld);

  /// Takes the world coordinates and returns the screen coordinates
  Vector2 getWorldToScreenXY(double worldX, double worldY) =>
      viewportTransform.getWorldToScreen(Vector2(worldX, worldY));

  /// Takes the screen coordinates (argScreen) and returns the world coordinates
  Vector2 getScreenToWorld(Vector2 argScreen) =>
      viewportTransform.getScreenToWorld(argScreen);
}
