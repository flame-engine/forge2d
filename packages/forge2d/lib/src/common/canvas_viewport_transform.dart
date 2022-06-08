import 'dart:html';

import 'package:forge2d/forge2d.dart';

/// Transform for drawing using a canvas context. Y-flip is permanently set
/// to true.
class CanvasViewportTransform extends ViewportTransform {
  static const double defaultDrawingScale = 20.0;

  /// Constructs a new viewport transform with the default scale.
  CanvasViewportTransform(Vector2 extents, Vector2 center)
      : super(extents, center, defaultDrawingScale) {
    yFlip = true;
  }

  /// Sets the rendering context such that all drawing commands given in terms
  /// of the world coordinate system will display correctly on the canvas
  /// screen.
  void updateTransformation(CanvasRenderingContext2D ctx) {
    // Clear all previous transformation.
    ctx.setTransform(1, 0, 0, 1, 0, 0);

    // Translate to the center of the canvas screen. This will be considered the
    // actual origin.
    ctx.translate(extents.x, extents.y);

    // Translate to account for the currently applied translation.
    ctx.translate(translation.x, translation.y);

    // Scale everything according to the current scale and mirror the y-axis.
    ctx.scale(scale, -scale);
  }
}
