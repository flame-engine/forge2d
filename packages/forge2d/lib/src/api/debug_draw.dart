import 'package:forge2d/src/api/math.dart';
import 'package:vector_math/vector_math.dart';

/// Receives the world's debug geometry when passed to `World.draw`.
///
/// Implement the draw methods for the primitives you care about and flip
/// the toggles for the data you want; everything defaults to drawing only
/// shapes and joints. Colors are 0xRRGGBB ints.
abstract class DebugDraw {
  /// Bounds culling: when set, only objects inside are drawn.
  Aabb? drawingBounds;

  /// Whether shapes are drawn.
  bool drawShapes = true;

  /// Whether joints are drawn.
  bool drawJoints = true;

  /// Whether additional joint detail is drawn.
  bool drawJointExtras = false;

  /// Whether the bounding boxes of shapes are drawn.
  bool drawBounds = false;

  /// Whether the mass and center of mass of dynamic bodies are drawn.
  bool drawMass = false;

  /// Whether body names are drawn.
  bool drawBodyNames = false;

  /// Whether contact points are drawn.
  bool drawContacts = false;

  /// Whether shapes are colored by their constraint graph color.
  bool drawGraphColors = false;

  /// Whether contact normals are drawn.
  bool drawContactNormals = false;

  /// Whether contact normal impulses are drawn.
  bool drawContactImpulses = false;

  /// Whether contact feature ids are drawn.
  bool drawContactFeatures = false;

  /// Whether contact friction impulses are drawn.
  bool drawFrictionImpulses = false;

  /// Whether islands are drawn as bounding boxes.
  bool drawIslands = false;

  /// Draws a closed polygon outline.
  void drawPolygon(List<Vector2> vertices, int color) {}

  /// Draws a filled rounded polygon given in local coordinates with its
  /// world transform.
  void drawSolidPolygon(
    Transform transform,
    List<Vector2> vertices,
    double radius,
    int color,
  ) {}

  /// Draws a circle outline.
  void drawCircle(Vector2 center, double radius, int color) {}

  /// Draws a filled circle at the transform's position; the transform's
  /// rotation gives the reference axis.
  void drawSolidCircle(Transform transform, double radius, int color) {}

  /// Draws a filled capsule between two points.
  void drawSolidCapsule(
    Vector2 point1,
    Vector2 point2,
    double radius,
    int color,
  ) {}

  /// Draws a line segment.
  void drawSegment(Vector2 point1, Vector2 point2, int color) {}

  /// Draws a transform: its position and axes.
  void drawTransform(Transform transform) {}

  /// Draws a point of [size] pixels.
  void drawPoint(Vector2 point, double size, int color) {}

  /// Draws a text string in world coordinates.
  void drawString(Vector2 point, String text, int color) {}
}
