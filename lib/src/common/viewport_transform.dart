import '../../forge2d.dart';

class ViewportTransform {
  /// if we flip the y axis when transforming.
  bool yFlip = false;

  /// This is the half-width and half-height.
  /// This should be the actual half-width and
  /// half-height, not anything transformed or scaled.
  final Vector2 extents;

  /// center of the viewport.
  final Vector2 center;

  /// Returns the scaling factor used in converting from world sizes to rendering
  /// sizes.
  double scale;

  ViewportTransform(this.extents, this.center, this.scale);

  /// Sets the transform's center to the given x and y coordinates,
  /// and using the given scale.
  void setCamera(double x, double y, double s) {
    center.setValues(x, y);
    scale = s;
  }

  /// The current translation is the difference in canvas units between the
  /// actual center of the canvas and the currently specified center. For
  /// example, if the actual canvas center is (5, 5) but the current center is
  /// (6, 6), the translation is (1, 1).
  Vector2 get translation => extents - center;

  set translation(Vector2 translation) {
    center.setFrom(extents);
    center.sub(translation);
  }

  /// Takes the world coordinates and return the corresponding screen coordinates
  Vector2 worldToScreen(Vector2 argWorld) {
    // Correct for canvas considering the upper-left corner, rather than the
    // center, to be the origin.
    final gridCorrected = Vector2(
      (argWorld.x * scale) + extents.x,
      extents.y - (argWorld.y * scale),
    );
    return gridCorrected + (translation..y *= (yFlip ? 1 : -1));
  }

  /// Takes the screen coordinates and return the corresponding world coordinates
  Vector2 screenToWorld(Vector2 argScreen) {
    final translatedCorrected =
        argScreen - (translation..y *= (yFlip ? 1 : -1));
    return ((translatedCorrected - extents)..y *= -1) / scale;
  }
}
